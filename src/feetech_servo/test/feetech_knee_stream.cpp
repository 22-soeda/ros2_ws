// feetech_knee_stream: 膝サーボの角度を「読むだけ」で流し続ける。
//
// 膝 4 節リンクの動作確認用。手で膝を曲げたときのモータ回転を読み取って、
// 3D ビジュアライザ (roboone_motion/viz/serve_knee3d.py) に渡す。
//
//   ros2 run feetech_servo feetech_knee_stream                    # 既定: 右バス ID4, 50Hz
//   ros2 run feetech_servo feetech_knee_stream --id 4 --rate 100
//   ros2 run feetech_servo feetech_knee_stream --port /dev/feetech_left --id 4
//
// ===========================================================================
// このツールはサーボに一切書き込まない
// ===========================================================================
// 例外は起動時の **トルク OFF（脱力）** ただ 1 回だけ。手で膝を動かせるように
// するためで、これ以外に書き込む経路をこのファイルは持っていない。
//   * 位置指令を書かない（write_position / sync_write_position を呼ばない）
//   * トルクを入れない（enable_torque(id, true) を呼ばない）
//   * EEPROM を触らない
//   * init_motor() も呼ばない（あれが既定でトルクを入れるため）
// --keep-torque を付けたときだけトルク OFF も省略するが、その場合も読むだけ。
//
// 標準出力は 1 行 1 サンプルの JSON（他のメッセージは標準エラーに出す）。
//
//   {"t":0.020,"ok":true,"raw":2028,"deg":178.24,"speed":0,"load":1.2,
//    "volt":11.8,"temp":38,"current":0,"n":2,"miss":0}
//
// 低電圧だとサーボの応答が間欠的に欠けることがある実機の癖があるので、読めなかった
// サンプルは {"ok":false} として捨てずに流し、欠損数 miss を累積で持たせている。
// 呼び側は ok=false のフレームで前の値を保持すればよい。
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "feetech_servo/feetech_manager.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;

namespace
{

std::atomic<bool> g_stop{false};
void on_sigint(int) {g_stop = true;}

void usage(const char * prog)
{
  std::fprintf(
    stderr,
    "使い方: %s [オプション]\n"
    "  --port PORT    シリアルポート（既定 /dev/feetech_right。udev の固定名を使う）\n"
    "  --id N         膝サーボの ID（既定 4）\n"
    "  --rate HZ      読み取り周期（既定 50、最大 200）\n"
    "  --seconds S    S 秒で終了（既定 0 = Ctrl-C まで）\n"
    "  --keep-torque  起動時のトルク OFF も行わない（読むだけなのは変わらない）\n"
    "  --baud N       ボーレート（既定 1000000）\n"
    "\n"
    "サーボには書き込まない。唯一の例外は起動時のトルク OFF 1 回だけ。\n", prog);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string port = "/dev/feetech_right";
  int id = 4;
  double rate = 50.0;
  double seconds = 0.0;
  int baud = 1000000;
  bool torque_off = true;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char * what) -> const char * {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "%s の値が無い\n", what);
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "--port") {port = next("--port");} else if (a == "--id") {
      id = std::atoi(next("--id"));
    } else if (a == "--rate") {rate = std::atof(next("--rate"));} else if (a == "--seconds") {
      seconds = std::atof(next("--seconds"));
    } else if (a == "--baud") {baud = std::atoi(next("--baud"));} else if (a == "--keep-torque") {
      torque_off = false;
    } else if (a == "-h" || a == "--help") {usage(argv[0]); return 0;} else {
      std::fprintf(stderr, "知らないオプション: %s\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
  }
  if (id < 0 || id > 253) {
    std::fprintf(stderr, "ID が範囲外: %d\n", id);
    return 2;
  }
  rate = rate < 1.0 ? 1.0 : (rate > 200.0 ? 200.0 : rate);

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  FeetechBus bus(port, baud);
  if (!bus.open()) {
    std::fprintf(stderr, "%s が開けない（udev 固定名と電源を確認）\n", port.c_str());
    return 1;
  }
  if (!bus.ping(static_cast<uint8_t>(id))) {
    std::fprintf(
      stderr, "ID %d が応答しない（%s）。バスと ID を確認する。\n"
      "  ros2 run feetech_servo feetech_scan_test --port %s\n",
      id, port.c_str(), port.c_str());
    return 1;
  }

  // ここが唯一の書き込み。手で膝を動かせるように脱力させる。
  if (torque_off) {
    if (bus.enable_torque(static_cast<uint8_t>(id), false)) {
      std::fprintf(stderr, "ID %d トルク OFF（脱力）。手で膝を動かせる。\n", id);
    } else {
      std::fprintf(stderr, "ID %d トルク OFF に失敗（応答なし）。読み取りは続ける。\n", id);
    }
  } else {
    std::fprintf(stderr, "--keep-torque: トルクは触らない。読むだけ。\n");
  }
  std::fprintf(
    stderr, "%s ID %d を %.0f Hz で読む。標準出力に JSON を 1 行ずつ。Ctrl-C で終了。\n",
    port.c_str(), id, rate);

  const auto t0 = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration<double>(1.0 / rate);
  std::uint64_t n = 0, miss = 0;
  std::vector<ServoState> st;

  while (!g_stop) {
    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - t0).count();
    if (seconds > 0.0 && t > seconds) {break;}

    st.clear();
    bus.sync_read_states({static_cast<uint8_t>(id)}, st);
    const ServoState s = st.empty() ? ServoState{} : st[0];
    ++n;
    if (!s.valid) {++miss;}

    if (s.valid) {
      std::printf(
        "{\"t\":%.3f,\"ok\":true,\"raw\":%d,\"deg\":%.3f,\"speed\":%d,\"load\":%.1f,"
        "\"volt\":%.2f,\"temp\":%d,\"current\":%d,\"n\":%llu,\"miss\":%llu}\n",
        t, s.pos, static_cast<double>(s.deg()), s.speed, s.load / 10.0, s.volt, s.temp,
        s.current, static_cast<unsigned long long>(n),
        static_cast<unsigned long long>(miss));
    } else {
      std::printf(
        "{\"t\":%.3f,\"ok\":false,\"n\":%llu,\"miss\":%llu}\n",
        t, static_cast<unsigned long long>(n), static_cast<unsigned long long>(miss));
    }
    std::fflush(stdout);

    std::this_thread::sleep_until(now + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period));
  }

  std::fprintf(
    stderr, "\n終了。%llu サンプル中 %llu 欠損（%.1f%%）。\n",
    static_cast<unsigned long long>(n), static_cast<unsigned long long>(miss),
    n ? 100.0 * static_cast<double>(miss) / static_cast<double>(n) : 0.0);
  return 0;
}
