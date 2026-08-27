// feetech_shell: サーボを「1軸ずつ手で」操作する対話CLI。
//
// 起動するとプロンプトが出るので、コマンドを打つとその場でシリアルに信号が出る。
// 一括操作ではなく、選択中の1軸（bus + ID）だけに対して読み書きする。
//
//   ros2 run feetech_servo feetech_shell                 # 既定2ポートを開いて対話開始
//   ros2 run feetech_servo feetech_shell --bus 1 --id 5  # 最初から bus1 の ID5 を選択
//   echo -e "id 5\npos\ngo 2047" | ros2 run feetech_servo feetech_shell   # パイプ入力も可
//
// 安全のため:
//   - トルクONは「現在位置を目標に書いてから」入れる（投入時の飛び出し防止）
//   - 現在位置が読めない軸にはトルクを入れない
//   - HLS系は goal_torque が 0 だと駆動しないので、0 のときは警告する
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>  // isatty

#include <scservo/SMS_STS.h>

#include "feetech_servo/feetech_manager.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;

namespace
{

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop = true; }

float deg_of(int steps) { return steps * 360.0f / 4096.0f; }

// 1本のシリアルバス（=コントローラ1台）。開けなかったポートも一覧に残し、後から open できる。
struct BusEntry
{
  std::string port;
  FeetechBus * bus = nullptr;
};

const char * mode_name(int mode)
{
  switch (mode) {
    case SMS_STS_MODE_SERVO: return "位置";
    case SMS_STS_MODE_WHEEL_CLOSED: return "速度(閉)";
    case SMS_STS_MODE_WHEEL_OPEN: return "PWM(開)";
    case SMS_STS_MODE_STEPPER: return "ステッパ";
    default: return "?";
  }
}

void print_help()
{
  std::printf(
    "\n--- コマンド（対象は「選択中の bus + ID」1軸のみ）---\n"
    "  bus [N|PORT]   操作するコントローラを選ぶ／今の選択を表示 (例: bus 1, bus /dev/ttyACM0)\n"
    "  buses          接続中のバス一覧\n"
    "  id [N]         操作するIDを選ぶ（ping で存在確認してから選択）／今の選択を表示\n"
    "  ping [N]       そのIDが応答するか（省略時は選択中のID）\n"
    "  scan [min max] このバスのIDを総当たり ping（既定 1..20）\n"
    "  pos            現在位置を生値で表示\n"
    "  state          位置/速度/負荷/電圧/温度/電流/moving/エラーを表示\n"
    "  info           EEPROM（型番/FW/モード/トルク/角度リミット/目標位置など）を表示\n"
    "  on             トルクON（現在位置を目標に書いてから入れる）\n"
    "  off            トルクOFF（脱力。手で動かせる）\n"
    "  go POS         指定位置へ移動 (0-4095)。例: go 2047\n"
    "  jog D          現在位置から D ステップ相対移動。例: jog -100\n"
    "  watch [秒]     位置を連続表示（既定5秒 / Ctrl-C で中断）\n"
    "  speed [N]      移動速度 step/s を設定／表示（0=最速）\n"
    "  acc [N]        加速度を設定／表示\n"
    "  torque [N]     HLS系の目標トルク 0-1000 を設定／表示（0 だと駆動しない）\n"
    "  getb/getw A    生レジスタを読む (例: getw 9 = 角度リミット下限)\n"
    "  setb/setw A V  生レジスタに書く。EEPROM(addr<40) は自動で unlock/lock\n"
    "  limits [Lo Hi] 角度リミットを表示/設定。全軸 0 0 が既定（0 0 = 制限なし）\n"
    "  stats          このバスの tx / rx_fail\n"
    "  help           このヘルプ\n"
    "  quit           終了（トルクの状態はそのまま）\n"
    "  ※ 行のどこかに @ID を付けると、その行だけ別のIDを対象にできる（例: pos @7）\n\n");
}

void usage(const char * argv0)
{
  std::printf(
    "使い方: %s [--port PORT]... [--baud N] [--bus N] [--id N] [--speed N] [--acc N]\n"
    "         [--torque N] [--family hls|sms] [--timeout MS]\n"
    "  --port     開くシリアルポート（複数指定可。既定: /dev/ttyACM0, /dev/ttyACM1）\n"
    "  --bus      起動時に選択するバス番号（既定 0）\n"
    "  --id       起動時に選択するサーボID\n"
    "  --speed    移動速度 step/s（既定 600、0=最速）\n"
    "  --acc      加速度（既定 20）\n"
    "  --torque   HLS系の目標トルク 0-1000（既定 1000）。0 だと駆動しない\n"
    "  --family   hls | sms（既定 hls。実機は HLS 系 model 4618/5130）\n"
    "  --timeout  1トランザクションの受信タイムアウト ms（既定 20）\n",
    argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> ports;
  int baud = 1000000;
  int timeout_ms = 20;
  int sel_bus = 0;
  int sel_id = -1;
  int speed = 600;
  int acc = 20;
  int goal_torque = 1000;
  auto family = feetech_servo::Family::kHls;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&]() -> const char * {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "%s に値がない\n", a.c_str());
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "--port") {
      ports.push_back(need());
    } else if (a == "--baud") {
      baud = std::atoi(need());
    } else if (a == "--timeout") {
      timeout_ms = std::atoi(need());
    } else if (a == "--bus") {
      sel_bus = std::atoi(need());
    } else if (a == "--id") {
      sel_id = std::atoi(need());
    } else if (a == "--speed") {
      speed = std::atoi(need());
    } else if (a == "--acc") {
      acc = std::atoi(need());
    } else if (a == "--torque") {
      goal_torque = std::atoi(need());
    } else if (a == "--family") {
      const std::string f = need();
      family = (f == "sms" || f == "sts") ? feetech_servo::Family::kSmsSts
        : feetech_servo::Family::kHls;
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "不明な引数: %s\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
  }
  if (ports.empty()) {
    ports = {"/dev/ttyACM0", "/dev/ttyACM1"};
  }

  std::signal(SIGINT, on_sigint);

  // --- バスを開く（開けなくても一覧には残す。あとで bus N で再試行できる）---
  feetech_servo::FeetechManager mgr;
  std::vector<BusEntry> buses;
  std::printf(
    "Feetech 対話シェル (baud=%d, %s系, 目標トルク=%d)\n", baud,
    family == feetech_servo::Family::kHls ? "HLS" : "SMS/STS", goal_torque);
  for (size_t i = 0; i < ports.size(); ++i) {
    BusEntry e;
    e.port = ports[i];
    e.bus = mgr.add_bus(ports[i], baud, /*proto_end=*/0, timeout_ms, family);
    if (e.bus) {
      e.bus->set_goal_torque(static_cast<uint16_t>(goal_torque));
      std::printf("  bus %zu: %s  … 接続\n", i, e.port.c_str());
    } else {
      std::printf("  bus %zu: %s  … 開けない（未接続 / dialout権限なし）\n", i, e.port.c_str());
    }
    buses.push_back(e);
  }
  if (sel_bus < 0 || sel_bus >= static_cast<int>(buses.size())) {
    sel_bus = 0;
  }

  // 選択中のバス。開けていなければ理由を出して nullptr。
  auto cur_bus = [&]() -> FeetechBus * {
      if (sel_bus < 0 || sel_bus >= static_cast<int>(buses.size())) {
        std::printf("バスが選択されていない。`buses` で一覧を見て `bus N`\n");
        return nullptr;
      }
      if (!buses[sel_bus].bus) {
        std::printf(
          "bus %d (%s) は開けていない。ケーブルを挿してから `bus %d` で再試行\n",
          sel_bus, buses[sel_bus].port.c_str(), sel_bus);
        return nullptr;
      }
      return buses[sel_bus].bus;
    };

  // 選択中のバスとID。IDが未選択なら促す。
  auto need_target = [&](FeetechBus *& bus, int & id, int override_id) -> bool {
      bus = cur_bus();
      if (!bus) {
        return false;
      }
      id = override_id >= 0 ? override_id : sel_id;
      if (id < 0) {
        std::printf("IDが未選択。`id N` で選ぶか、行末に @N を付ける\n");
        return false;
      }
      return true;
    };

  // 1軸の現在状態を読む（単軸 SyncRead。読めなければ valid=false）
  auto read_one = [&](FeetechBus * bus, int id) -> ServoState {
      std::vector<ServoState> st;
      bus->sync_read_states({static_cast<uint8_t>(id)}, st);
      return st.empty() ? ServoState{} : st[0];
    };

  // EEPROM(addr<40) を安全に書く: トルクOFF確認 → unlock → 書き → lock → 読み戻し。
  // 書き込み回数に上限があるので、設定修正のときだけ使うこと。
  auto eeprom_write = [&](FeetechBus * bus, int id, int addr, int val, bool is_word) -> bool {
      const uint8_t u = static_cast<uint8_t>(id);
      if (bus->read_byte(u, SMS_STS_TORQUE_ENABLE) == 1) {
        std::printf("  トルクONのまま EEPROM は書かない。先に `off`\n");
        return false;
      }
      if (!bus->unlock_eeprom(u, true)) {
        std::printf("  EEPROMのロック解除に失敗（応答なし）\n");
        return false;
      }
      const bool ok = is_word
        ? bus->write_word(u, static_cast<uint8_t>(addr), static_cast<uint16_t>(val))
        : bus->write_byte(u, static_cast<uint8_t>(addr), static_cast<uint8_t>(val));
      bus->unlock_eeprom(u, false);  // 必ずロックへ戻す
      const int rb = is_word ? bus->read_word(u, static_cast<uint8_t>(addr))
        : bus->read_byte(u, static_cast<uint8_t>(addr));
      std::printf(
        "  addr %d ← %d … %s（読み戻し %d）\n", addr, val, ok ? "OK" : "失敗", rb);
      return ok && rb == val;
    };

  if (sel_id >= 0) {
    std::printf("選択: bus %d (%s), ID %d\n", sel_bus, buses[sel_bus].port.c_str(), sel_id);
  }
  const bool interactive = isatty(STDIN_FILENO) != 0;
  if (interactive) {
    print_help();
  }

  std::string line;
  while (true) {
    if (interactive) {
      std::printf(
        "[bus%d id%s]> ", sel_bus, sel_id < 0 ? "-" : std::to_string(sel_id).c_str());
      std::fflush(stdout);
    }
    if (!std::getline(std::cin, line)) {
      if (g_stop) {  // Ctrl-C で読み込みが中断された場合は継続
        g_stop = false;
        std::cin.clear();
        std::printf("\n");
        continue;
      }
      std::printf("\n");
      break;  // EOF (Ctrl-D / パイプ終端)
    }
    g_stop = false;

    // --- トークン分解。@N は「この行だけ対象にするID」 ---
    std::vector<std::string> tok;
    int override_id = -1;
    {
      std::istringstream iss(line);
      std::string t;
      while (iss >> t) {
        if (t[0] == '@') {
          override_id = std::atoi(t.c_str() + 1);
          continue;
        }
        if (t[0] == '#') {
          break;  // 以降はコメント
        }
        tok.push_back(t);
      }
    }
    if (tok.empty()) {
      continue;
    }
    const std::string cmd = tok[0];
    auto arg_int = [&](size_t i, int def) {
        return i < tok.size() ? std::atoi(tok[i].c_str()) : def;
      };

    FeetechBus * bus = nullptr;
    int id = -1;

    if (cmd == "quit" || cmd == "exit" || cmd == "q") {
      break;

    } else if (cmd == "help" || cmd == "?" || cmd == "h") {
      print_help();

    } else if (cmd == "buses") {
      for (size_t i = 0; i < buses.size(); ++i) {
        std::printf(
          "  %s bus %zu: %-16s %s\n", i == static_cast<size_t>(sel_bus) ? "*" : " ", i,
          buses[i].port.c_str(), buses[i].bus ? "接続" : "未接続");
      }

    } else if (cmd == "bus") {
      if (tok.size() < 2) {
        std::printf(
          "bus %d (%s) %s\n", sel_bus, buses[sel_bus].port.c_str(),
          buses[sel_bus].bus ? "接続" : "未接続");
        continue;
      }
      // 番号でもポート名でも選べる
      int idx = -1;
      if (tok[1].find('/') == std::string::npos) {
        idx = std::atoi(tok[1].c_str());
      } else {
        for (size_t i = 0; i < buses.size(); ++i) {
          if (buses[i].port == tok[1]) {
            idx = static_cast<int>(i);
          }
        }
        if (idx < 0) {  // 一覧に無いポートはその場で追加して開く
          BusEntry e;
          e.port = tok[1];
          e.bus = mgr.add_bus(e.port, baud, 0, timeout_ms, family);
          if (e.bus) {
            e.bus->set_goal_torque(static_cast<uint16_t>(goal_torque));
          }
          buses.push_back(e);
          idx = static_cast<int>(buses.size()) - 1;
        }
      }
      if (idx < 0 || idx >= static_cast<int>(buses.size())) {
        std::printf("そんなバスは無い。`buses` で一覧\n");
        continue;
      }
      sel_bus = idx;
      if (!buses[sel_bus].bus) {  // 未接続なら開き直しを試す
        buses[sel_bus].bus = mgr.add_bus(buses[sel_bus].port, baud, 0, timeout_ms, family);
        if (buses[sel_bus].bus) {
          buses[sel_bus].bus->set_goal_torque(static_cast<uint16_t>(goal_torque));
        }
      }
      std::printf(
        "bus %d (%s) を選択 … %s\n", sel_bus, buses[sel_bus].port.c_str(),
        buses[sel_bus].bus ? "接続" : "開けない（未接続 / dialout権限なし）");

    } else if (cmd == "id") {
      if (tok.size() < 2 && override_id < 0) {
        std::printf("ID %s\n", sel_id < 0 ? "未選択" : std::to_string(sel_id).c_str());
        continue;
      }
      const int want = override_id >= 0 ? override_id : arg_int(1, -1);
      if (want < 0 || want > 253) {
        std::printf("IDは 0..253\n");
        continue;
      }
      bus = cur_bus();
      if (!bus) {
        continue;
      }
      if (bus->ping(static_cast<uint8_t>(want))) {
        sel_id = want;
        const ServoState s = read_one(bus, sel_id);
        std::printf(
          "ID %d を選択（応答あり）", sel_id);
        if (s.valid) {
          std::printf("  pos=%d (%.2f deg) %.1fV %d℃\n", s.pos, s.deg(), s.volt, s.temp);
        } else {
          std::printf("  ※ 状態は読めず\n");
        }
      } else {
        std::printf("ID %d は応答なし（選択は変えない）。電源電圧とIDを確認\n", want);
      }

    } else if (cmd == "ping") {
      const int want = override_id >= 0 ? override_id : arg_int(1, sel_id);
      bus = cur_bus();
      if (!bus) {
        continue;
      }
      if (want < 0) {
        std::printf("IDを指定して（例: ping 5）\n");
        continue;
      }
      std::printf(
        "ID %d: %s\n", want,
        bus->ping(static_cast<uint8_t>(want)) ? "応答あり" : "応答なし");

    } else if (cmd == "scan") {
      bus = cur_bus();
      if (!bus) {
        continue;
      }
      const int lo = arg_int(1, 1);
      const int hi = arg_int(2, tok.size() > 1 ? lo : 20);
      std::printf("bus %d (%s) の ID %d..%d を ping:\n", sel_bus, buses[sel_bus].port.c_str(), lo, hi);
      int n = 0;
      for (int i = lo; i <= hi && i <= 253 && !g_stop; ++i) {
        if (i == SMS_STS_BROADCAST_ID) {
          continue;
        }
        if (bus->ping(static_cast<uint8_t>(i))) {
          const ServoState s = read_one(bus, i);
          std::printf(
            "  ID %-3d 応答あり", i);
          if (s.valid) {
            std::printf("  pos=%-5d (%7.2f deg) %.1fV %d℃%s", s.pos, s.deg(), s.volt, s.temp,
              s.err ? ("  err: " + feetech_servo::err_str(s.err)).c_str() : "");
          }
          std::printf("\n");
          ++n;
        }
      }
      std::printf("  検出 %d 軸\n", n);

    } else if (cmd == "pos") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      const ServoState s = read_one(bus, id);
      if (s.valid) {
        std::printf("ID %d pos = %d  (%.2f deg)\n", id, s.pos, s.deg());
      } else {
        std::printf("ID %d 読めず（電源電圧 / 配線を確認）\n", id);
      }

    } else if (cmd == "state") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      const ServoState s = read_one(bus, id);
      if (!s.valid) {
        std::printf("ID %d 読めず（電源電圧 / 配線を確認）\n", id);
        continue;
      }
      std::printf(
        "ID %d  pos=%d (%.2f deg)  speed=%d step/s  load=%.1f%%  %.1fV  %d℃  %dmA  %s  err=0x%02X %s\n",
        id, s.pos, s.deg(), s.speed, s.load / 10.0f, s.volt, s.temp, s.current,
        s.moving ? "動作中" : "停止", s.err, feetech_servo::err_str(s.err).c_str());

    } else if (cmd == "info") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      const uint8_t u = static_cast<uint8_t>(id);
      const int model = bus->read_word(u, SMS_STS_MODEL_L);
      if (model < 0) {
        std::printf("ID %d は応答しない\n", id);
        continue;
      }
      std::printf("ID %d  型番=%d  FW=%d.%d  モード=%s\n", id, model,
        bus->read_byte(u, SMS_STS_FIRMWARE_VER_L), bus->read_byte(u, SMS_STS_FIRMWARE_VER_H),
        mode_name(bus->read_byte(u, SMS_STS_MODE)));
      std::printf("  トルク      : %s\n",
        bus->read_byte(u, SMS_STS_TORQUE_ENABLE) ? "ON" : "OFF");
      std::printf("  角度リミット: %d .. %d\n",
        bus->read_word(u, SMS_STS_MIN_ANGLE_LIMIT_L), bus->read_word(u, SMS_STS_MAX_ANGLE_LIMIT_L));
      std::printf("  位置オフセット: %d\n", bus->read_word(u, SMS_STS_OFS_L));
      std::printf("  目標位置    : %d\n", bus->read_word(u, SMS_STS_GOAL_POSITION_L));
      std::printf("  目標トルク  : %d  (HLS系。0 だと駆動しない)\n",
        bus->read_word(u, SMS_STS_GOAL_TIME_L));
      std::printf("  トルク上限  : %d\n", bus->read_word(u, SMS_STS_TORQUE_LIMIT_L));
      std::printf("  温度上限    : %d degC\n", bus->read_byte(u, SMS_STS_MAX_TEMPERATURE_LIMIT));

    } else if (cmd == "on") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      // 現在位置を目標に書いてからトルクON（古い目標へ飛び出さないように）
      const ServoState s = read_one(bus, id);
      if (!s.valid) {
        std::printf(
          "ID %d の現在位置が読めない。飛び出す危険があるのでトルクは入れない\n", id);
        continue;
      }
      bus->write_position(
        static_cast<uint8_t>(id), static_cast<int16_t>(s.pos), static_cast<uint16_t>(speed),
        static_cast<uint8_t>(acc));
      if (bus->enable_torque(static_cast<uint8_t>(id), true)) {
        std::printf("ID %d トルクON（現在位置 %d で保持）\n", id, s.pos);
        if (bus->goal_torque() == 0) {
          std::printf("  ※ 目標トルクが0なので位置指令を出しても動かない。`torque 1000`\n");
        }
      } else {
        std::printf("ID %d トルクONに失敗（応答なし）\n", id);
      }

    } else if (cmd == "off") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      std::printf(
        "ID %d トルクOFF%s\n", id,
        bus->enable_torque(static_cast<uint8_t>(id), false) ? "（脱力）" : " … 失敗（応答なし）");

    } else if (cmd == "go" || cmd == "jog") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      if (tok.size() < 2) {
        std::printf("%s に値がない（例: %s %s）\n", cmd.c_str(), cmd.c_str(),
          cmd == "go" ? "2047" : "-100");
        continue;
      }
      const ServoState before = read_one(bus, id);
      int target = std::atoi(tok[1].c_str());
      if (cmd == "jog") {
        if (!before.valid) {
          std::printf("ID %d の現在位置が読めないので相対移動できない\n", id);
          continue;
        }
        target += before.pos;
      }
      if (target < 0 || target > 4095) {
        const int clamped = std::clamp(target, 0, 4095);
        std::printf("  目標 %d は範囲外 → %d に丸めた\n", target, clamped);
        target = clamped;
      }
      if (bus->read_byte(static_cast<uint8_t>(id), SMS_STS_TORQUE_ENABLE) == 0) {
        std::printf("  ※ このIDはトルクOFF。指令は出すが動かない（`on` でトルク投入）\n");
      }
      if (bus->goal_torque() == 0) {
        std::printf("  ※ 目標トルクが0なので動かない（`torque 1000`）\n");
      }
      bus->write_position(
        static_cast<uint8_t>(id), static_cast<int16_t>(target), static_cast<uint16_t>(speed),
        static_cast<uint8_t>(acc));
      std::printf(
        "ID %d → 指令 %d (%.2f deg)  speed=%d acc=%d\n", id, target, deg_of(target), speed, acc);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      const ServoState after = read_one(bus, id);
      if (after.valid) {
        std::printf(
          "  0.3秒後: pos=%d (%.2f deg)  誤差 %+d  load=%.1f%%  %dmA%s\n",
          after.pos, after.deg(), after.pos - target, after.load / 10.0f, after.current,
          after.err ? ("  err: " + feetech_servo::err_str(after.err)).c_str() : "");
      }

    } else if (cmd == "watch") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      const double sec = tok.size() > 1 ? std::atof(tok[1].c_str()) : 5.0;
      const auto t0 = std::chrono::steady_clock::now();
      std::printf("ID %d を %.1f 秒監視（Ctrl-C で中断）\n", id, sec);
      while (!g_stop) {
        const double t = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
        if (t > sec) {
          break;
        }
        const ServoState s = read_one(bus, id);
        if (s.valid) {
          std::printf(
            "\r  %5.1fs  pos=%-5d (%7.2f deg)  spd=%-6d load=%6.1f%%  %4dmA  %s  ",
            t, s.pos, s.deg(), s.speed, s.load / 10.0f, s.current, s.moving ? "動作中" : "停止  ");
        } else {
          std::printf("\r  %5.1fs  読めず                                              ", t);
        }
        std::fflush(stdout);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::printf("\n");
      g_stop = false;

    } else if (cmd == "speed") {
      if (tok.size() > 1) {
        speed = std::max(0, arg_int(1, speed));
      }
      std::printf("speed = %d step/s%s\n", speed, speed == 0 ? "（0=最速）" : "");

    } else if (cmd == "acc") {
      if (tok.size() > 1) {
        acc = std::clamp(arg_int(1, acc), 0, 255);
      }
      std::printf("acc = %d\n", acc);

    } else if (cmd == "torque") {
      if (tok.size() > 1) {
        goal_torque = std::clamp(arg_int(1, goal_torque), 0, 1000);
        for (auto & e : buses) {
          if (e.bus) {
            e.bus->set_goal_torque(static_cast<uint16_t>(goal_torque));
          }
        }
      }
      std::printf(
        "目標トルク = %d%s\n", goal_torque,
        goal_torque == 0 ? "  ※ 0 だと HLS系はまったく駆動しない" : "");

    } else if (cmd == "getb" || cmd == "getw") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      if (tok.size() < 2) {
        std::printf("アドレスを指定して（例: getw 9）\n");
        continue;
      }
      const int addr = arg_int(1, -1);
      if (addr < 0 || addr > 255) {
        std::printf("アドレスは 0..255\n");
        continue;
      }
      const int v = (cmd == "getb") ? bus->read_byte(static_cast<uint8_t>(id),
          static_cast<uint8_t>(addr))
        : bus->read_word(static_cast<uint8_t>(id), static_cast<uint8_t>(addr));
      if (v < 0) {
        std::printf("ID %d addr %d 読めず\n", id, addr);
      } else {
        std::printf("ID %d addr %d = %d\n", id, addr, v);
      }

    } else if (cmd == "setb" || cmd == "setw") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      if (tok.size() < 3) {
        std::printf("書式: %s ADDR VALUE（例: setw 9 0）\n", cmd.c_str());
        continue;
      }
      const int addr = arg_int(1, -1);
      const int val = arg_int(2, 0);
      if (addr < 0 || addr > 255) {
        std::printf("アドレスは 0..255\n");
        continue;
      }
      const bool is_word = (cmd == "setw");
      if (addr < SMS_STS_TORQUE_ENABLE) {  // 40未満 = EEPROM 領域
        std::printf("EEPROM addr %d を書く:\n", addr);
        eeprom_write(bus, id, addr, val, is_word);
      } else {
        const bool ok = is_word
          ? bus->write_word(static_cast<uint8_t>(id), static_cast<uint8_t>(addr),
            static_cast<uint16_t>(val))
          : bus->write_byte(static_cast<uint8_t>(id), static_cast<uint8_t>(addr),
            static_cast<uint8_t>(val));
        std::printf("ID %d addr %d ← %d … %s\n", id, addr, val, ok ? "OK" : "失敗");
      }

    } else if (cmd == "limits") {
      if (!need_target(bus, id, override_id)) {
        continue;
      }
      if (tok.size() >= 3) {
        const int lo = arg_int(1, 0);
        const int hi = arg_int(2, 0);
        if (lo > hi) {
          // MIN>MAX は不正。ファームが目標位置をこの窓に丸めるので事実上どこにも動けなくなる。
          std::printf("下限 %d > 上限 %d は不正な設定。書き込まない\n", lo, hi);
          continue;
        }
        std::printf("ID %d の角度リミットを %d .. %d に設定:\n", id, lo, hi);
        eeprom_write(bus, id, SMS_STS_MIN_ANGLE_LIMIT_L, lo, true);
        eeprom_write(bus, id, SMS_STS_MAX_ANGLE_LIMIT_L, hi, true);
      }
      const int lo = bus->read_word(static_cast<uint8_t>(id), SMS_STS_MIN_ANGLE_LIMIT_L);
      const int hi = bus->read_word(static_cast<uint8_t>(id), SMS_STS_MAX_ANGLE_LIMIT_L);
      std::printf("ID %d 角度リミット: %d .. %d%s\n", id, lo, hi,
        (lo == 0 && hi == 0) ? "（0 0 = 制限なし）"
        : (lo > hi ? "  ※ 下限>上限の不正設定。目標位置がここに丸められて動かない" : ""));

    } else if (cmd == "stats") {
      bus = cur_bus();
      if (!bus) {
        continue;
      }
      std::printf(
        "bus %d (%s): tx=%llu  rx_fail=%llu\n", sel_bus, buses[sel_bus].port.c_str(),
        static_cast<unsigned long long>(bus->tx_count()),
        static_cast<unsigned long long>(bus->rx_fail()));

    } else {
      std::printf("不明なコマンド: %s（`help` で一覧）\n", cmd.c_str());
    }
  }

  std::printf("終了（トルクの状態はそのまま。脱力するなら再度 `off`）\n");
  return 0;
}
