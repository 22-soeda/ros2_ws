// feetech_goto_test: 定義済みの全モーターを一斉に、約30秒かけて位置 2047 へ動かすテスト。
//
// 手順（全バス同時。バスごとに1スレッドで、共通の開始時刻に合わせて走らせる）:
//   1) ping で定義軸の生存確認
//   2) sync_read_states で「現在の角度」を読む（ここが軌道の始点になる）
//   3) 目標位置に現在位置を書いてからトルクON（トルク投入時の飛び出しを防ぐ）
//   4) rate_hz で始点→2047 を smoothstep 補間し、SyncWritePosEx で送りながら状態を読む
//   5) 到達誤差と通信統計を表示
//
//   ros2 run feetech_servo feetech_goto_test                       # 既定の定義で実行
//   ros2 run feetech_servo feetech_goto_test --scan                # 定義ではなく実機の全応答軸
//   ros2 run feetech_servo feetech_goto_test --motors /dev/ttyACM0:1,2,3
//   ros2 run feetech_servo feetech_goto_test --duration 10 --target 2047
//   ros2 run feetech_servo feetech_goto_test --no-torque   # 動作確認用: トルクを入れない
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
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "feetech_servo/feetech_manager.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::Mode;
using feetech_servo::ServoState;
using Clock = std::chrono::steady_clock;

namespace
{

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop = true; }

// --- 定義済みモーター（ポート → 軸ID）。--motors で上書きできる。---
struct MotorDef
{
  std::string port;
  std::vector<uint8_t> ids;
};

// 実機は 1バスあたり ID 1..10 の範囲。応答しない軸は自動で対象外になる。
const std::vector<MotorDef> kDefaultMotors = {
  {"/dev/ttyACM0", {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}},
  {"/dev/ttyACM1", {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}},
};

float deg_of(int steps) { return steps * 360.0f / 4096.0f; }

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
  std::vector<int> start_pos;      // 始点（ids と同順）
  std::vector<int16_t> cmd;        // 直近の指令位置
  std::vector<ServoState> states;  // 直近の読み取り
  int cycles = 0;
  int read_ok_total = 0;           // 読めた軸数の累計（平均取得率の分子）
  // 直近サイクルで読めた軸数。バススレッドが書き、メインの進捗表示が読むので atomic。
  // （states 自体はバススレッド専有。他スレッドから触るとデータ競合になる）
  std::atomic<int> last_valid{0};
};

// 定義軸の現在状態を読む。低電圧などで欠ける軸があるので数回リトライする。
int read_start_states(BusRun & run, const std::vector<uint8_t> & ids, int attempts = 5)
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
    if (have[i]) {
      run.ids.push_back(ids[i]);
      run.start_pos.push_back(got[i].pos);
      ++ok;
      std::printf(
        "    ID %-3d 現在位置 %5d (%6.2f deg)  電圧 %.1fV 温度 %d℃%s\n",
        ids[i], got[i].pos, got[i].deg(), got[i].volt, got[i].temp,
        got[i].err ? ("  err: " + feetech_servo::err_str(got[i].err)).c_str() : "");
    } else {
      std::printf("    ID %-3d 現在位置を読めず → この軸は動かさない\n", ids[i]);
    }
  }
  return ok;
}

// 共通の開始時刻に合わせて、始点 → target を duration 秒で補間しながら送る。
void run_ramp(
  BusRun & run, int target, double duration_s, double rate_hz, int speed, int acc,
  Clock::time_point t0)
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
        std::lround(run.start_pos[i] + (target - run.start_pos[i]) * s));
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
    // 最終指令は厳密に target。整定を待ってから読む。
    std::fill(run.cmd.begin(), run.cmd.end(), static_cast<int16_t>(target));
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
    std::fprintf(stderr, "--motors の書式は PORT:ID,ID,... （例 /dev/ttyACM0:1,2,3）\n");
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
    "使い方: %s [--motors PORT:ID,ID,...]... [--target N] [--duration S]\n"
    "         [--rate HZ] [--speed N] [--acc N] [--scan] [--yes]\n"
    "  --motors   動かすモーターの定義（複数指定可）。既定は kDefaultMotors\n"
    "  --scan     定義を使わず、各ポートで応答した軸すべてを対象にする\n"
    "  --target   目標位置（既定 2047 = 中央）\n"
    "  --duration 到達までの秒数（既定 30）\n"
    "  --rate     指令の送信周波数 Hz（既定 50）\n"
    "  --speed    SyncWritePosEx の速度 step/s（既定 600）\n"
    "  --acc      SyncWritePosEx の加速度（既定 20）\n"
    "  --torque   HLS系の目標トルク 0-1000（既定 1000）。0 だと駆動しない\n"
    "  --family   hls | sms（既定 hls。実機は HLS 系 model 4618/5130）\n"
    "  --no-torque トルクを入れずに指令だけ流す（機体は動かない。手順確認用）\n"
    "  --yes      開始前の3秒カウントダウンを省略する\n",
    argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<MotorDef> motors;
  int target = 2047;
  double duration_s = 30.0;
  double rate_hz = 50.0;
  int speed = 600;
  int acc = 20;
  int baud = 1000000;
  int goal_torque = 1000;
  auto family = feetech_servo::Family::kHls;
  bool use_scan = false;
  bool skip_countdown = false;
  bool no_torque = false;

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
    } else if (a == "--target") {
      target = std::atoi(need());
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
    } else if (a == "--scan") {
      use_scan = true;
    } else if (a == "--no-torque") {
      no_torque = true;
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
  if (motors.empty()) {
    motors = kDefaultMotors;
  }
  if (duration_s <= 0.0 || rate_hz <= 0.0) {
    std::fprintf(stderr, "--duration と --rate は正の値\n");
    return 2;
  }

  std::signal(SIGINT, on_sigint);

  std::printf(
    "Feetech 一斉移動テスト: target=%d (%.2f deg), duration=%.1fs, rate=%.0fHz, "
    "speed=%d, acc=%d, torque=%d (%s系)\n",
    target, deg_of(target), duration_s, rate_hz, speed, acc, goal_torque,
    family == feetech_servo::Family::kHls ? "HLS" : "SMS/STS");

  // --- 1) バスを開いて定義軸の生存確認 ---
  feetech_servo::FeetechManager mgr;
  std::vector<std::unique_ptr<BusRun>> runs;
  for (const MotorDef & def : motors) {
    std::printf("\n[%s]\n", def.port.c_str());
    FeetechBus * bus = mgr.add_bus(def.port, baud, /*proto_end=*/0, /*timeout_ms=*/20, family);
    if (!bus) {
      std::printf("  ポートを開けない。スキップ\n");
      continue;
    }
    bus->set_goal_torque(static_cast<uint16_t>(goal_torque));
    auto run = std::make_unique<BusRun>();
    run->port = def.port;
    run->bus = bus;

    std::vector<uint8_t> want;
    if (use_scan) {
      std::vector<uint8_t> all(253);
      for (int id = 1; id <= 253; ++id) {
        all[id - 1] = static_cast<uint8_t>(id);
      }
      want = bus->scan(all);
      std::printf("  scan: %zu 軸を検出\n", want.size());
    } else {
      want = bus->scan(def.ids);
      if (want.size() != def.ids.size()) {
        std::printf(
          "  定義 %zu 軸中 %zu 軸のみ応答（欠けた軸は対象外。電源電圧を確認）\n",
          def.ids.size(), want.size());
      }
    }
    if (want.empty()) {
      std::printf("  対象軸なし。スキップ\n");
      continue;
    }

    // --- 2) 現在の角度を読む（軌道の始点）---
    std::printf("  現在角度の読み取り:\n");
    if (read_start_states(*run, want) == 0) {
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
  std::printf("\n対象: %zu バス / 合計 %zu 軸\n", runs.size(), total_axes);

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
    std::printf("\n%.0f秒かけて全軸を %d へ動かす。3秒後に開始（Ctrl-Cで中止）", duration_s, target);
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
    threads.emplace_back([r, target, duration_s, rate_hz, speed, acc, t0]() {
        run_ramp(*r, target, duration_s, rate_hz, speed, acc, t0);
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

  // --- 5) 結果 ---
  std::printf("\n================ 結果 ================\n");
  for (const auto & rp : runs) {
    const BusRun & r = *rp;
    std::printf("\n[%s] サイクル %d, 平均取得率 %.1f%%\n",
      r.port.c_str(), r.cycles,
      r.cycles > 0 ? 100.0 * r.read_ok_total / (r.cycles * static_cast<double>(r.ids.size())) : 0.0);
    std::printf("  %-4s %-8s %-8s %-8s %-9s %s\n", "ID", "始点", "指令", "実測", "誤差", "状態");
    std::printf("  %s\n", std::string(60, '-').c_str());
    for (size_t i = 0; i < r.ids.size(); ++i) {
      const bool ok = i < r.states.size() && r.states[i].valid;
      char meas[16], err[16];
      if (ok) {
        std::snprintf(meas, sizeof(meas), "%d", r.states[i].pos);
        std::snprintf(err, sizeof(err), "%+d", r.states[i].pos - r.cmd[i]);
      } else {
        std::snprintf(meas, sizeof(meas), "-");
        std::snprintf(err, sizeof(err), "-");
      }
      std::printf(
        "  %-4d %-8d %-8d %-8s %-9s %s\n",
        r.ids[i], r.start_pos[i], r.cmd[i], meas, err,
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
