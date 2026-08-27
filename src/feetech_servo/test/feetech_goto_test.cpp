// feetech_goto_test: 全軸を一斉に、約30秒かけて「角度指定」の姿勢へ動かすテスト。
//
// 目標は生カウントではなく**角度**で与える。各軸の原点（機構上の 90deg にあたる生カウント）は
// feetech_calibrate_home が作った config/servo_home.yaml から読み、軸ごとに
//
//     指令カウント = home + (狙い角[deg] - home_deg) * 4096 / 360
//
// で換算する。既定の --angle は home_deg（=90）なので、**引数なしで全軸が校正した姿勢
// （Tポーズ）へ戻る**。原点が無い軸（未校正）は換算できないので動かさない。
//
// 手順（全バス同時。バスごとに1スレッドで、共通の開始時刻に合わせて走らせる）:
//   1) ping で対象軸の生存確認（応答するのに未校正の軸は警告して対象外）
//   2) sync_read_states で「現在位置」を読む（ここが軌道の始点になる）
//   3) 目標位置に現在位置を書いてからトルクON（トルク投入時の飛び出しを防ぐ）
//   4) rate_hz で始点→目標を smoothstep 補間し、SyncWritePosEx で送りながら状態を読む
//   5) 到達誤差（deg）と通信統計を表示
//
//   ros2 run feetech_servo feetech_goto_test                 # 30秒かけて Tポーズ(90deg)へ
//   ros2 run feetech_servo feetech_goto_test --angle 120     # 全軸 120deg へ
//   ros2 run feetech_servo feetech_goto_test --dry-run       # 換算した目標を見るだけ（動かさない）
//   ros2 run feetech_servo feetech_goto_test --duration 10
//   ros2 run feetech_servo feetech_goto_test --motors /dev/feetech_right:1,2,3
//   ros2 run feetech_servo feetech_goto_test --no-torque     # 動作確認用: トルクを入れない
//
// Ctrl-C で中断すると、その時点の指令位置で保持したまま終了する（トルクは切らない）。
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "feetech_servo/feetech_manager.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::Mode;
using feetech_servo::ServoState;
using Clock = std::chrono::steady_clock;

namespace
{

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop = true; }

constexpr int kPosMax = 4095;
constexpr double kStepsPerDeg = 4096.0 / 360.0;

// --- 原点（初期位置）設定 ---------------------------------------------------
// servo_home.yaml の中身。home_deg は「校正した姿勢を何 deg と呼ぶか」。
struct HomeConfig
{
  float home_deg = 90.0f;
  // port → (ID → 原点の生カウント)
  std::map<std::string, std::map<int, int>> home;
  std::vector<std::string> ports;  // ファイルに書かれた順（対象バスの既定順）
};

HomeConfig load_home(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    std::fprintf(
      stderr,
      "原点ファイルを読めない: %s\n  %s\n"
      "  先に `ros2 run feetech_servo feetech_calibrate_home` で原点を取ること\n",
      path.c_str(), e.what());
    std::exit(2);
  }
  HomeConfig cfg;
  if (root["home_deg"]) {
    cfg.home_deg = root["home_deg"].as<float>();
  }
  if (!root["buses"] || !root["buses"].IsSequence()) {
    std::fprintf(stderr, "%s: `buses:` が無い\n", path.c_str());
    std::exit(2);
  }
  for (const auto & b : root["buses"]) {
    if (!b["port"] || !b["servos"] || !b["servos"].IsMap()) {
      continue;
    }
    const std::string port = b["port"].as<std::string>();
    cfg.ports.push_back(port);
    for (const auto & kv : b["servos"]) {
      const int id = kv.first.as<int>();
      int home = -1;
      if (kv.second.IsMap() && kv.second["home"]) {
        home = kv.second["home"].as<int>();
      } else if (kv.second.IsScalar()) {
        home = kv.second.as<int>();
      }
      if (home >= 0 && home <= kPosMax) {
        cfg.home[port][id] = home;
      }
    }
  }
  if (cfg.home.empty()) {
    std::fprintf(stderr, "%s: 原点が1軸も入っていない\n", path.c_str());
    std::exit(2);
  }
  return cfg;
}

// --- 対象モーター（ポート → 軸ID）。--motors で絞り込める。---
struct MotorDef
{
  std::string port;
  std::vector<uint8_t> ids;
};

// 0..1 を滑らかに 0..1 へ（始点・終点で速度0 = 急発進/急停止しない）
double smoothstep(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

// 1バスぶんの実行状態
struct BusRun
{
  std::string port;
  FeetechBus * bus = nullptr;
  std::vector<uint8_t> ids;        // 実際に動かす軸（始点を読めた軸のみ）
  std::vector<int> home;           // 各軸の原点カウント（ids と同順）
  std::vector<int> start_pos;      // 始点（ids と同順）
  std::vector<int> target_pos;     // 角度から換算した目標（ids と同順）
  std::vector<int16_t> cmd;        // 直近の指令位置
  std::vector<ServoState> states;  // 直近の読み取り
  int cycles = 0;
  int read_ok_total = 0;           // 読めた軸数の累計（平均取得率の分子）
  // 直近サイクルで読めた軸数。バススレッドが書き、メインの進捗表示が読むので atomic。
  // （states 自体はバススレッド専有。他スレッドから触るとデータ競合になる）
  std::atomic<int> last_valid{0};

  // 生カウント → その軸の角度[deg]（原点基準）
  double deg_at(size_t i, int pos) const;
};

double g_home_deg = 90.0;  // 表示・換算の基準（HomeConfig から入る）

double BusRun::deg_at(size_t i, int pos) const
{
  return g_home_deg + (pos - home[i]) * 360.0 / 4096.0;
}

// 狙い角[deg] → その軸の生カウント（0..4095 に丸める）
int steps_for(int home, double angle_deg)
{
  const long v = std::lround(home + (angle_deg - g_home_deg) * kStepsPerDeg);
  return static_cast<int>(std::clamp<long>(v, 0, kPosMax));
}

// 対象軸の現在状態を読む。低電圧などで欠ける軸があるので数回リトライする。
int read_start_states(
  BusRun & run, const std::vector<uint8_t> & ids, const std::vector<int> & homes,
  double angle_deg, int attempts = 5)
{
  std::vector<ServoState> got(ids.size());
  std::vector<bool> have(ids.size(), false);
  std::vector<ServoState> tmp;
  for (int a = 0; a < attempts; ++a) {
    run.bus->sync_read_states(ids, tmp);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (!have[i] && tmp[i].valid) {
        got[i] = tmp[i];
        have[i] = true;
      }
    }
    if (std::all_of(have.begin(), have.end(), [](bool b) {return b;})) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  int ok = 0;
  for (size_t i = 0; i < ids.size(); ++i) {
    if (!have[i]) {
      std::printf("    ID %-3d 現在位置を読めず → この軸は動かさない\n", ids[i]);
      continue;
    }
    const int target = steps_for(homes[i], angle_deg);
    run.ids.push_back(ids[i]);
    run.home.push_back(homes[i]);
    run.start_pos.push_back(got[i].pos);
    run.target_pos.push_back(target);
    ++ok;
    const double cur_deg = g_home_deg + (got[i].pos - homes[i]) * 360.0 / 4096.0;
    std::printf(
      "    ID %-3d 原点 %4d | 現在 %5d (%6.1f deg) → 目標 %5d (%.1f deg)  %.1fV %d℃%s\n",
      ids[i], homes[i], got[i].pos, cur_deg, target, angle_deg, got[i].volt, got[i].temp,
      got[i].err ? ("  err: " + feetech_servo::err_str(got[i].err)).c_str() : "");
  }
  return ok;
}

// 共通の開始時刻に合わせて、始点 → 各軸の目標を duration 秒で補間しながら送る。
void run_ramp(
  BusRun & run, double duration_s, double rate_hz, int speed, int acc, Clock::time_point t0)
{
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / rate_hz));
  const size_t n = run.ids.size();
  run.cmd.assign(n, 0);
  std::vector<uint16_t> speeds(n, static_cast<uint16_t>(speed));
  std::vector<uint8_t> accs(n, static_cast<uint8_t>(acc));

  // 全バスがここで同じ t0 を待つので「一斉」に動き出す。
  std::this_thread::sleep_until(t0);

  auto next = t0;
  while (!g_stop) {
    const double t = std::chrono::duration<double>(Clock::now() - t0).count();
    if (t > duration_s) {
      break;
    }
    const double s = smoothstep(t / duration_s);
    for (size_t i = 0; i < n; ++i) {
      run.cmd[i] = static_cast<int16_t>(
        std::lround(run.start_pos[i] + (run.target_pos[i] - run.start_pos[i]) * s));
    }
    run.bus->sync_write_position(run.ids, run.cmd, speeds, accs);
    const int ok = run.bus->sync_read_states(run.ids, run.states);
    run.read_ok_total += ok;
    run.last_valid.store(ok, std::memory_order_relaxed);
    ++run.cycles;

    next += period;
    std::this_thread::sleep_until(next);
  }

  if (!g_stop) {
    // 最終指令は厳密に目標値。整定を待ってから読む。
    for (size_t i = 0; i < n; ++i) {
      run.cmd[i] = static_cast<int16_t>(run.target_pos[i]);
    }
    run.bus->sync_write_position(run.ids, run.cmd, speeds, accs);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    run.last_valid.store(
      run.bus->sync_read_states(run.ids, run.states), std::memory_order_relaxed);
  }
}

std::vector<MotorDef> parse_motors_arg(const std::string & spec)
{
  // "PORT:1,2,3" 形式。呼び出しごとに1バス。
  const size_t colon = spec.rfind(':');
  if (colon == std::string::npos) {
    std::fprintf(stderr, "--motors の書式は PORT:ID,ID,... （例 /dev/feetech_right:1,2,3）\n");
    std::exit(2);
  }
  MotorDef d;
  d.port = spec.substr(0, colon);
  std::string rest = spec.substr(colon + 1);
  size_t p = 0;
  while (p < rest.size()) {
    size_t q = rest.find(',', p);
    if (q == std::string::npos) {
      q = rest.size();
    }
    const int id = std::atoi(rest.substr(p, q - p).c_str());
    if (id > 0 && id < 254) {
      d.ids.push_back(static_cast<uint8_t>(id));
    }
    p = q + 1;
  }
  return {d};
}

void usage(const char * argv0)
{
  std::printf(
    "使い方: %s [--angle DEG] [--home FILE] [--motors PORT:ID,ID,...]...\n"
    "         [--duration S] [--rate HZ] [--speed N] [--acc N] [--no-torque] [--yes]\n"
    "  --angle    全軸の狙い角 deg（既定 = 原点ファイルの home_deg = 校正した姿勢そのもの）\n"
    "  --home     原点ファイル（既定: share/feetech_servo/config/servo_home.yaml）\n"
    "  --motors   対象を絞る（複数指定可）。既定は原点ファイルの全軸\n"
    "  --duration 到達までの秒数（既定 30）\n"
    "  --rate     指令の送信周波数 Hz（既定 50）\n"
    "  --speed    SyncWritePosEx の速度 step/s（既定 600）\n"
    "  --acc      SyncWritePosEx の加速度（既定 20）\n"
    "  --torque   HLS系の目標トルク 0-1000（既定 1000）。0 だと駆動しない\n"
    "  --family   hls | sms（既定 hls。実機は HLS 系 model 4618/5130）\n"
    "  --dry-run, -n 目標の換算結果だけ表示して終了する（指令もトルクも出さない）\n"
    "  --no-torque トルクを入れずに指令だけ流す（トルクが既にONの軸は動くので注意）\n"
    "  --yes      開始前の3秒カウントダウンを省略する\n",
    argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<MotorDef> motors;
  std::string home_path;
  double angle_deg = 0.0;
  bool angle_given = false;
  double duration_s = 30.0;
  double rate_hz = 50.0;
  int speed = 600;
  int acc = 20;
  int baud = 1000000;
  int goal_torque = 1000;
  auto family = feetech_servo::Family::kHls;
  bool skip_countdown = false;
  bool no_torque = false;
  bool dry_run = false;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&]() -> const char * {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "%s に値がない\n", a.c_str());
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "--motors") {
      auto d = parse_motors_arg(need());
      motors.insert(motors.end(), d.begin(), d.end());
    } else if (a == "--angle" || a == "--deg") {
      angle_deg = std::atof(need());
      angle_given = true;
    } else if (a == "--home") {
      home_path = need();
    } else if (a == "--duration") {
      duration_s = std::atof(need());
    } else if (a == "--rate") {
      rate_hz = std::atof(need());
    } else if (a == "--speed") {
      speed = std::atoi(need());
    } else if (a == "--acc") {
      acc = std::atoi(need());
    } else if (a == "--baud") {
      baud = std::atoi(need());
    } else if (a == "--torque") {
      goal_torque = std::atoi(need());
    } else if (a == "--family") {
      const std::string f = need();
      family = (f == "sms" || f == "sts") ? feetech_servo::Family::kSmsSts
        : feetech_servo::Family::kHls;
    } else if (a == "--no-torque") {
      no_torque = true;
    } else if (a == "--dry-run" || a == "-n") {
      dry_run = true;
    } else if (a == "--yes" || a == "-y") {
      skip_countdown = true;
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "不明な引数: %s\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
  }
  if (duration_s <= 0.0 || rate_hz <= 0.0) {
    std::fprintf(stderr, "--duration と --rate は正の値\n");
    return 2;
  }

  // --- 原点ファイル ---
  if (home_path.empty()) {
    try {
      home_path = ament_index_cpp::get_package_share_directory("feetech_servo") +
        "/config/servo_home.yaml";
    } catch (const std::exception & e) {
      std::fprintf(stderr, "既定の原点ファイルの場所を解決できない: %s\n", e.what());
      return 2;
    }
  }
  const HomeConfig home_cfg = load_home(home_path);
  g_home_deg = home_cfg.home_deg;
  if (!angle_given) {
    angle_deg = g_home_deg;  // 既定 = 校正した姿勢そのもの
  }

  // --- 対象軸（既定は原点ファイルの全軸。--motors があればその範囲に絞る）---
  if (motors.empty()) {
    for (const std::string & port : home_cfg.ports) {
      MotorDef d;
      d.port = port;
      for (const auto & kv : home_cfg.home.at(port)) {
        d.ids.push_back(static_cast<uint8_t>(kv.first));
      }
      motors.push_back(std::move(d));
    }
  }

  std::signal(SIGINT, on_sigint);

  std::printf(
    "Feetech 一斉移動テスト（角度指定）: angle=%.1f deg, duration=%.1fs, rate=%.0fHz, "
    "speed=%d, acc=%d, torque=%d (%s系)\n",
    angle_deg, duration_s, rate_hz, speed, acc, goal_torque,
    family == feetech_servo::Family::kHls ? "HLS" : "SMS/STS");
  std::printf(
    "原点: %s（home_deg=%.1f）。目標カウントは軸ごとに原点から換算する\n",
    home_path.c_str(), g_home_deg);
  if (!angle_given) {
    std::printf("  --angle 省略 → 校正した姿勢（Tポーズ）そのものへ戻る\n");
  }

  // --- 1) バスを開いて対象軸の生存確認 ---
  feetech_servo::FeetechManager mgr;
  std::vector<std::unique_ptr<BusRun>> runs;
  int uncalibrated = 0;
  for (const MotorDef & def : motors) {
    std::printf("\n[%s]\n", def.port.c_str());
    const auto it = home_cfg.home.find(def.port);
    if (it == home_cfg.home.end()) {
      std::printf("  原点ファイルにこのポートが無い。スキップ\n");
      continue;
    }
    const std::map<int, int> & port_home = it->second;

    FeetechBus * bus = mgr.add_bus(def.port, baud, /*proto_end=*/0, /*timeout_ms=*/20, family);
    if (!bus) {
      std::printf("  ポートを開けない。スキップ\n");
      continue;
    }
    bus->set_goal_torque(static_cast<uint16_t>(goal_torque));
    auto run = std::make_unique<BusRun>();
    run->port = def.port;
    run->bus = bus;

    // 原点のある軸だけを対象にする（未校正の軸は角度に換算できない）
    std::vector<uint8_t> want;
    std::vector<int> want_home;
    std::vector<uint8_t> no_home;
    for (uint8_t id : def.ids) {
      const auto h = port_home.find(id);
      if (h == port_home.end()) {
        no_home.push_back(id);
      } else {
        want.push_back(id);
        want_home.push_back(h->second);
      }
    }
    // 応答するのに原点が無い軸は、黙って無視せず警告する（bus1 ID7 など）
    for (uint8_t id : bus->scan(no_home)) {
      std::printf(
        "  ID %d は応答するが原点が未校正 → 動かさない"
        "（feetech_calibrate_home --id %d で取れる）\n", id, id);
      ++uncalibrated;
    }

    const std::vector<uint8_t> alive = bus->scan(want);
    if (alive.size() != want.size()) {
      std::printf(
        "  対象 %zu 軸中 %zu 軸のみ応答（欠けた軸は対象外。電源電圧を確認）\n",
        want.size(), alive.size());
    }
    if (alive.empty()) {
      std::printf("  対象軸なし。スキップ\n");
      continue;
    }
    std::vector<int> alive_home;
    alive_home.reserve(alive.size());
    for (uint8_t id : alive) {
      for (size_t i = 0; i < want.size(); ++i) {
        if (want[i] == id) {
          alive_home.push_back(want_home[i]);
          break;
        }
      }
    }

    // --- 2) 現在位置を読む（軌道の始点）---
    std::printf("  現在位置の読み取りと目標の換算:\n");
    if (read_start_states(*run, alive, alive_home, angle_deg) == 0) {
      std::printf("  始点を読めた軸がない。スキップ\n");
      continue;
    }
    runs.push_back(std::move(run));
  }

  if (runs.empty()) {
    std::fprintf(stderr, "\n動かせる軸がない。終了\n");
    return 1;
  }

  size_t total_axes = 0;
  for (const auto & r : runs) {
    total_axes += r->ids.size();
  }
  std::printf("\n対象: %zu バス / 合計 %zu 軸", runs.size(), total_axes);
  if (uncalibrated > 0) {
    std::printf("（原点未校正で除外 %d 軸）", uncalibrated);
  }
  std::printf("\n");

  if (dry_run) {
    std::printf("\n--dry-run のため、指令もトルクも出さずに終了する\n");
    return 0;
  }

  // --- 3) 目標＝現在位置を書いてからトルクON（投入時に飛び出さないように）---
  if (no_torque) {
    std::printf("トルクON: --no-torque 指定のためスキップ（指令は流すが機体は動かない）\n");
  } else {
    std::printf("トルクON:\n");
    for (const auto & rp : runs) {
      BusRun & r = *rp;
      std::vector<int16_t> hold(r.start_pos.begin(), r.start_pos.end());
      r.bus->sync_write_position(r.ids, hold);
      int on = 0;
      for (uint8_t id : r.ids) {
        if (r.bus->enable_torque(id, true)) {
          ++on;
        }
      }
      std::printf("  %s: %d/%zu 軸\n", r.port.c_str(), on, r.ids.size());
    }
  }

  if (!skip_countdown && !g_stop) {
    std::printf(
      "\n%.0f秒かけて全軸を %.1f deg へ動かす。3秒後に開始（Ctrl-Cで中止）",
      duration_s, angle_deg);
    std::fflush(stdout);
    for (int i = 3; i > 0 && !g_stop; --i) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::printf(" %d", i - 1);
      std::fflush(stdout);
    }
    std::printf("\n");
  }
  if (g_stop) {
    std::printf("開始前に中断された\n");
    return 1;
  }

  // --- 4) 全バス同時に補間送信 ---
  const auto t0 = Clock::now() + std::chrono::milliseconds(200);  // 全スレッドが揃うための余裕
  std::vector<std::thread> threads;
  for (const auto & rp : runs) {
    BusRun * r = rp.get();
    threads.emplace_back([r, duration_s, rate_hz, speed, acc, t0]() {
        run_ramp(*r, duration_s, rate_hz, speed, acc, t0);
      });
  }

  // 進捗表示（メインスレッド）
  while (!g_stop) {
    const double t = std::chrono::duration<double>(Clock::now() - t0).count();
    if (t > duration_s) {
      break;
    }
    if (t >= 0.0) {
      std::printf("\r  進捗 %5.1f/%.0f s (%3.0f%%) ", t, duration_s, 100.0 * t / duration_s);
      for (const auto & r : runs) {
        std::printf(
          "| %s: %d/%zu軸 ", r->port.c_str(),
          r->last_valid.load(std::memory_order_relaxed), r->ids.size());
      }
      std::fflush(stdout);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  for (std::thread & th : threads) {
    th.join();
  }
  std::printf("\n");

  if (g_stop) {
    std::printf("\n中断された。各軸は直前の指令位置で保持している（トルクは入れたまま）\n");
  }

  // --- 5) 結果（誤差は deg で見る）---
  std::printf("\n================ 結果 ================\n");
  for (const auto & rp : runs) {
    const BusRun & r = *rp;
    std::printf("\n[%s] サイクル %d, 平均取得率 %.1f%%\n",
      r.port.c_str(), r.cycles,
      r.cycles > 0 ? 100.0 * r.read_ok_total / (r.cycles * static_cast<double>(r.ids.size())) : 0.0);
    std::printf(
      "  %-4s %-6s %-9s %-9s %-9s %-8s %s\n",
      "ID", "原点", "始点deg", "目標deg", "実測deg", "誤差deg", "状態");
    std::printf("  %s\n", std::string(66, '-').c_str());
    for (size_t i = 0; i < r.ids.size(); ++i) {
      const bool ok = i < r.states.size() && r.states[i].valid;
      char meas[16], err[16];
      if (ok) {
        std::snprintf(meas, sizeof(meas), "%.1f", r.deg_at(i, r.states[i].pos));
        std::snprintf(
          err, sizeof(err), "%+.1f", (r.states[i].pos - r.cmd[i]) * 360.0 / 4096.0);
      } else {
        std::snprintf(meas, sizeof(meas), "-");
        std::snprintf(err, sizeof(err), "-");
      }
      char start_s[16], goal_s[16];
      std::snprintf(start_s, sizeof(start_s), "%.1f", r.deg_at(i, r.start_pos[i]));
      std::snprintf(goal_s, sizeof(goal_s), "%.1f", r.deg_at(i, r.target_pos[i]));
      std::printf(
        "  %-4d %-6d %-9s %-9s %-9s %-8s %s\n",
        r.ids[i], r.home[i], start_s, goal_s, meas, err,
        ok ? (r.states[i].err ? feetech_servo::err_str(r.states[i].err).c_str() : "OK") : "読めず");
    }
    std::printf(
      "  通信統計: tx=%llu, rx_fail=%llu\n",
      static_cast<unsigned long long>(r.bus->tx_count()),
      static_cast<unsigned long long>(r.bus->rx_fail()));
  }

  if (no_torque) {
    std::printf("\n完了（--no-torque: トルクを入れていないので機体は動いていない）。\n");
  } else {
    std::printf("\n完了。トルクは入れたまま（保持中）。抜くには電源断か EnableTorque(false)。\n");
  }
  return g_stop ? 1 : 0;
}
