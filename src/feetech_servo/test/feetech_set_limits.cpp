// feetech_set_limits: 設定ファイル(YAML)の角度リミットを全軸へ一括で書き込む。
//
//   ros2 run feetech_servo feetech_set_limits --dry-run    # 書く内容だけ表示（変更しない）
//   ros2 run feetech_servo feetech_set_limits              # 確認のあと一括書き込み
//   ros2 run feetech_servo feetech_set_limits -c ./my.yaml --yes
//
// 書き込むのは EEPROM の addr 9 (MIN_ANGLE_LIMIT) / addr 11 (MAX_ANGLE_LIMIT) だけ。
// 安全のため:
//   - 設定ファイル全体を検証してから接続する（下限>上限などがあれば1軸も書かずに中止）
//   - 現在値と同じ軸は書かない（EEPROM の書き換え回数を消費しない）
//   - トルクONの軸があれば中止する（--torque-off で先に脱力させることもできる）
//   - 新しい範囲の外に現在位置がある軸は警告する（次にトルクを入れたとき範囲端まで動くため）
//   - 書き込みは unlock → 書き → lock → 読み戻し検証 の順
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <scservo/SMS_STS.h>

#include "feetech_servo/feetech_manager.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;

namespace
{

constexpr int kPosMax = 4095;
// 低電圧時は応答が間欠的に欠けるので、ping と読み出しはこの回数まで試す。
constexpr int kReadAttempts = 3;

struct LimitEntry
{
  int id = 0;
  int min = 0;
  int max = 0;
  // 実機から読んだ現状（接続後に埋める）
  bool alive = false;
  bool cur_valid = false;  // 現在のリミットを読めたか（読めない軸には絶対に書かない）
  int cur_min = -1;
  int cur_max = -1;
  int cur_pos = 0;
  bool pos_valid = false;
  bool torque_on = false;
};

struct BusPlan
{
  std::string port;
  std::vector<LimitEntry> entries;
  FeetechBus * bus = nullptr;
};

float deg_of(int steps) { return steps * 360.0f / 4096.0f; }

// 設定ファイルを読む。書式エラーは即 exit(2)（部分的に書き込まないため接続前に落とす）。
std::vector<BusPlan> load_config(const std::string & path, int & baud)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "設定ファイルを読めない: %s\n  %s\n", path.c_str(), e.what());
    std::exit(2);
  }

  if (root["baud"]) {
    baud = root["baud"].as<int>();
  }
  if (!root["buses"] || !root["buses"].IsSequence()) {
    std::fprintf(stderr, "%s: `buses:` が無い\n", path.c_str());
    std::exit(2);
  }

  std::vector<BusPlan> plans;
  int errors = 0;
  for (const auto & b : root["buses"]) {
    BusPlan plan;
    if (!b["port"]) {
      std::fprintf(stderr, "buses の要素に `port:` が無い\n");
      ++errors;
      continue;
    }
    plan.port = b["port"].as<std::string>();
    if (!b["servos"] || !b["servos"].IsMap()) {
      std::fprintf(stderr, "%s: `servos:` が無い\n", plan.port.c_str());
      ++errors;
      continue;
    }
    for (const auto & kv : b["servos"]) {
      LimitEntry e;
      e.id = kv.first.as<int>();
      const YAML::Node & v = kv.second;
      if (v.IsSequence() && v.size() == 2) {          // [下限, 上限]
        e.min = v[0].as<int>();
        e.max = v[1].as<int>();
      } else if (v.IsMap() && v["min"] && v["max"]) {  // {min: , max: }
        e.min = v["min"].as<int>();
        e.max = v["max"].as<int>();
      } else {
        std::fprintf(
          stderr, "%s ID %d: 書式は [下限, 上限] か {min: , max: }\n", plan.port.c_str(), e.id);
        ++errors;
        continue;
      }
      // --- 値の検証 ---
      if (e.id < 1 || e.id > 253) {
        std::fprintf(stderr, "%s: ID %d は範囲外 (1..253)\n", plan.port.c_str(), e.id);
        ++errors;
      }
      if (e.min < 0 || e.min > kPosMax || e.max < 0 || e.max > kPosMax) {
        std::fprintf(
          stderr, "%s ID %d: リミットは 0..%d（指定 %d..%d）\n",
          plan.port.c_str(), e.id, kPosMax, e.min, e.max);
        ++errors;
      }
      if (e.min > e.max) {
        std::fprintf(
          stderr,
          "%s ID %d: 下限 %d > 上限 %d は不正。この設定はサーボを動かなくする\n",
          plan.port.c_str(), e.id, e.min, e.max);
        ++errors;
      }
      if (e.min == e.max && e.min != 0) {
        std::fprintf(
          stderr, "%s ID %d: 下限=上限=%d は可動域ゼロ。制限なしにするなら [0, 0]\n",
          plan.port.c_str(), e.id, e.min);
        ++errors;
      }
      plan.entries.push_back(e);
    }
    std::sort(
      plan.entries.begin(), plan.entries.end(),
      [](const LimitEntry & a, const LimitEntry & b2) {return a.id < b2.id;});
    plans.push_back(std::move(plan));
  }

  if (errors > 0) {
    std::fprintf(stderr, "\n設定に %d 件の問題。1軸も書き込まずに中止する\n", errors);
    std::exit(2);
  }
  return plans;
}

const char * limit_str(int lo, int hi, char * buf, size_t n)
{
  if (lo == 0 && hi == 0) {
    std::snprintf(buf, n, "制限なし");
  } else {
    std::snprintf(buf, n, "%d..%d", lo, hi);
  }
  return buf;
}

void usage(const char * argv0)
{
  std::printf(
    "使い方: %s [-c FILE] [--dry-run] [--yes] [--torque-off] [--baud N]\n"
    "  -c, --config  設定ファイル（既定: share/feetech_servo/config/servo_limits.yaml）\n"
    "  --dry-run     書き込まず、何を変更するかだけ表示する\n"
    "  --yes, -y     確認プロンプトを省略する\n"
    "  --torque-off  トルクONの軸があれば先に脱力させる（機体が落ちるので注意）\n"
    "  --baud        設定ファイルの baud を上書きする\n",
    argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string config_path;
  bool dry_run = false;
  bool assume_yes = false;
  bool torque_off = false;
  int baud_override = 0;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&]() -> const char * {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "%s に値がない\n", a.c_str());
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "-c" || a == "--config") {
      config_path = need();
    } else if (a == "--dry-run" || a == "-n") {
      dry_run = true;
    } else if (a == "--yes" || a == "-y") {
      assume_yes = true;
    } else if (a == "--torque-off") {
      torque_off = true;
    } else if (a == "--baud") {
      baud_override = std::atoi(need());
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "不明な引数: %s\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
  }

  if (config_path.empty()) {
    // 既定はインストール先の config/servo_limits.yaml
    try {
      config_path = ament_index_cpp::get_package_share_directory("feetech_servo") +
        "/config/servo_limits.yaml";
    } catch (const std::exception & e) {
      std::fprintf(stderr, "既定の設定ファイルの場所を解決できない: %s\n", e.what());
      return 2;
    }
  }

  int baud = 1000000;
  std::vector<BusPlan> plans = load_config(config_path, baud);
  if (baud_override > 0) {
    baud = baud_override;
  }
  std::printf("設定: %s (baud=%d)\n", config_path.c_str(), baud);

  // --- 実機の現状を読む（ここではまだ何も書かない）---
  feetech_servo::FeetechManager mgr;
  int missing = 0;
  int unreadable = 0;
  int torque_on_count = 0;
  for (BusPlan & plan : plans) {
    plan.bus = mgr.add_bus(plan.port, baud);
    if (!plan.bus) {
      std::fprintf(stderr, "\n%s を開けない（未接続 / dialout権限なし）。中止\n", plan.port.c_str());
      return 1;
    }
    for (LimitEntry & e : plan.entries) {
      const uint8_t id = static_cast<uint8_t>(e.id);
      // 低電圧などで応答が間欠的に欠けるので数回試す
      for (int a = 0; a < kReadAttempts && !e.alive; ++a) {
        e.alive = plan.bus->ping(id);
      }
      if (!e.alive) {
        ++missing;
        continue;
      }
      for (int a = 0; a < kReadAttempts && !e.cur_valid; ++a) {
        e.cur_min = plan.bus->read_word(id, SMS_STS_MIN_ANGLE_LIMIT_L);
        e.cur_max = plan.bus->read_word(id, SMS_STS_MAX_ANGLE_LIMIT_L);
        e.cur_valid = e.cur_min >= 0 && e.cur_max >= 0;
      }
      if (!e.cur_valid) {
        ++unreadable;
        continue;
      }
      e.torque_on = plan.bus->read_byte(id, SMS_STS_TORQUE_ENABLE) == 1;
      if (e.torque_on) {
        ++torque_on_count;
      }
      const ServoState st = plan.bus->read_state(id);
      e.pos_valid = st.valid;
      e.cur_pos = st.pos;
    }
  }

  // --- 計画の表示 ---
  int to_write = 0;
  int out_of_range = 0;
  for (const BusPlan & plan : plans) {
    std::printf("\n[%s]\n", plan.port.c_str());
    std::printf("  %-4s %-12s %-12s %-16s %s\n", "ID", "現在", "設定", "現在位置", "判定");
    std::printf("  %s\n", std::string(64, '-').c_str());
    for (const LimitEntry & e : plan.entries) {
      char cur[24], want[24], pos[24];
      limit_str(e.min, e.max, want, sizeof(want));
      if (!e.alive) {
        std::printf("  %-4d %-12s %-12s %-16s %s\n", e.id, "-", want, "-", "応答なし → 飛ばす");
        continue;
      }
      if (!e.cur_valid) {
        // 現在値が読めない軸に書くと、何を上書きするのか分からないので触らない
        std::printf(
          "  %-4d %-12s %-12s %-16s %s\n", e.id, "読めず", want, "-",
          "現在値を読めず → 飛ばす");
        continue;
      }
      if (!e.cur_valid) {
        // 現在値が読めない軸に書くと、何を上書きするのか分からないので触らない
        std::printf(
          "  %-4d %-12s %-12s %-16s %s\n", e.id, "読めず", want, "-",
          "現在値を読めず → 飛ばす");
        continue;
      }
      limit_str(e.cur_min, e.cur_max, cur, sizeof(cur));
      if (e.pos_valid) {
        std::snprintf(pos, sizeof(pos), "%d (%.1f deg)", e.cur_pos, deg_of(e.cur_pos));
      } else {
        std::snprintf(pos, sizeof(pos), "読めず");
      }
      const bool same = (e.cur_min == e.min && e.cur_max == e.max);
      std::string verdict;
      if (same) {
        verdict = "同じ → 書かない";
      } else {
        verdict = "変更する";
        ++to_write;
      }
      // 窓の外に今いる軸は、次のトルクON時に窓の端まで動く。
      // 書き換えない軸でも起きるので、変更の有無に関係なく調べる。
      const bool limited = !(e.min == 0 && e.max == 0);
      if (limited && e.pos_valid && (e.cur_pos < e.min || e.cur_pos > e.max)) {
        verdict += "  ※ 現在位置が範囲外";
        ++out_of_range;
      }
      if (e.torque_on) {
        verdict += "  ※ トルクON";
      }
      std::printf("  %-4d %-12s %-12s %-16s %s\n", e.id, cur, want, pos, verdict.c_str());
    }
  }

  std::printf(
    "\n変更する軸: %d / 応答なし: %d / 現在値を読めず: %d / 現在位置が新範囲の外: %d\n",
    to_write, missing, unreadable, out_of_range);
  if (missing > 0 || unreadable > 0) {
    std::printf(
      "  ※ 応答しない軸は書き込み対象から外れる。全軸に入れたいなら電源電圧(~12V)と配線を直してから\n");
  }
  if (out_of_range > 0) {
    std::printf(
      "  ※ 範囲外の軸は、次にトルクを入れた時点で範囲の端まで動く。手で範囲内に戻しておくこと\n");
  }

  if (to_write == 0) {
    std::printf("\n変更なし。何も書き込まずに終了\n");
    return 0;
  }
  if (dry_run) {
    std::printf("\n--dry-run のため書き込まない\n");
    return 0;
  }

  // --- トルクの確認（EEPROM はトルクOFFで書く）---
  if (torque_on_count > 0) {
    if (!torque_off) {
      std::fprintf(
        stderr,
        "\nトルクONの軸が %d 個ある。EEPROM はトルクOFFで書くこと。\n"
        "  そのまま脱力させてよければ --torque-off を付けて再実行（機体が落ちるので支えてから）\n",
        torque_on_count);
      return 1;
    }
    std::printf("\n--torque-off: 対象軸のトルクを切る\n");
    for (BusPlan & plan : plans) {
      for (LimitEntry & e : plan.entries) {
        if (e.alive && e.torque_on) {
          const bool ok = plan.bus->enable_torque(static_cast<uint8_t>(e.id), false);
          std::printf("  %s ID %d … %s\n", plan.port.c_str(), e.id, ok ? "OFF" : "失敗");
          e.torque_on = !ok;
        }
      }
    }
  }

  // --- 確認 ---
  if (!assume_yes) {
    std::printf("\n%d 軸の EEPROM を書き換える。よければ yes と入力: ", to_write);
    std::fflush(stdout);
    std::string ans;
    if (!std::getline(std::cin, ans) || (ans != "yes" && ans != "y")) {
      std::printf("中止した\n");
      return 1;
    }
  }

  // --- 書き込み（unlock → 書き → lock → 読み戻し）---
  int written = 0;
  int failed = 0;
  std::printf("\n書き込み:\n");
  for (BusPlan & plan : plans) {
    for (LimitEntry & e : plan.entries) {
      if (!e.alive || !e.cur_valid || (e.cur_min == e.min && e.cur_max == e.max)) {
        continue;
      }
      const uint8_t id = static_cast<uint8_t>(e.id);
      char want[24];
      limit_str(e.min, e.max, want, sizeof(want));
      if (!plan.bus->unlock_eeprom(id, true)) {
        std::printf("  %s ID %-3d … EEPROMのロック解除に失敗\n", plan.port.c_str(), e.id);
        ++failed;
        continue;
      }
      plan.bus->write_word(id, SMS_STS_MIN_ANGLE_LIMIT_L, static_cast<uint16_t>(e.min));
      plan.bus->write_word(id, SMS_STS_MAX_ANGLE_LIMIT_L, static_cast<uint16_t>(e.max));
      plan.bus->unlock_eeprom(id, false);  // 必ずロックに戻す

      const int rb_min = plan.bus->read_word(id, SMS_STS_MIN_ANGLE_LIMIT_L);
      const int rb_max = plan.bus->read_word(id, SMS_STS_MAX_ANGLE_LIMIT_L);
      if (rb_min == e.min && rb_max == e.max) {
        std::printf("  %s ID %-3d → %s  OK\n", plan.port.c_str(), e.id, want);
        ++written;
      } else {
        std::printf(
          "  %s ID %-3d → %s  失敗（読み戻し %d..%d）\n",
          plan.port.c_str(), e.id, want, rb_min, rb_max);
        ++failed;
      }
    }
  }

  std::printf(
    "\n完了: 書き込み %d 軸, 失敗 %d 軸, 飛ばした軸 %d（応答なし %d / 現在値を読めず %d）\n",
    written, failed, missing + unreadable, missing, unreadable);
  if (failed == 0) {
    std::printf("EEPROM は再ロック済み。電源を入れ直しても設定は残る\n");
  }
  return failed == 0 ? 0 : 1;
}
