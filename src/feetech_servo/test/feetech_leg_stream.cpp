// feetech_leg_stream: 両脚 12 軸のサーボ角を「読むだけ」で流し続ける。
//
// 両脚 3D ビジュアライザ (roboone_motion/viz/serve_legs3d.py) に渡す。
// 手で脚を動かしたときのサーボ角を読み取って、関節角と姿勢の表示に使う。
//
//   ros2 run feetech_servo feetech_leg_stream                 # 既定: 両バス, ID 1,2,3,4,6,5, 50Hz
//   ros2 run feetech_servo feetech_leg_stream --rate 100
//   ros2 run feetech_servo feetech_leg_stream --only right
//
// ===========================================================================
// このツールはサーボに一切書き込まない
// ===========================================================================
// 例外は起動時の **トルク OFF（脱力）** ただ 1 回だけ。手で脚を動かせるように
// するためで、これ以外に書き込む経路をこのファイルは持っていない。
//   * 位置指令を書かない（write_position / sync_write_position を呼ばない）
//   * トルクを入れない（enable_torque(id, true) を呼ばない）
//   * EEPROM を触らない（unlock_eeprom / write_byte / write_word を呼ばない）
//   * init_motor() も呼ばない（あれが既定でトルクを入れるため）
// --keep-torque を付けたときはトルク OFF も省略する。どちらでも読むだけ。
//
// ID と関節の対応（docs/servo-registers.md、2026-08-28 実機で確定）
//   1 股ピッチ / 2 股ロール / 3 股ヨー / 4 膝(4節リンク)
//   6 足首パラレル 鎖 0（短ロッド 80mm）/ 5 足首パラレル 鎖 1（長ロッド 115mm）
// ankle_parallel.hpp の鎖の添字に合わせるため、既定の並びは **1,2,3,4,6,5**。
//
// 標準出力は 1 行 1 サンプルの JSON（他のメッセージは標準エラーへ）。
//   {"t":0.020,"n":12,"R":{"ok":true,"raw":[...],"deg":[...],"valid":[...],
//                          "volt":11.8,"temp":38,"miss":0},"L":{...}}
//
// 低電圧だとサーボの応答が間欠的に欠けることがある実機の癖があるので、読めなかった
// 軸は valid=false で流し、欠損数 miss を累積で持たせている。呼び側は前の値を保持する。
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
    "  --right PORT   右脚のポート（既定 /dev/feetech_right）\n"
    "  --left PORT    左脚のポート（既定 /dev/feetech_left）\n"
    "  --only SIDE    right / left / both（既定 both）\n"
    "  --ids LIST     読む ID をカンマ区切りで（既定 1,2,3,4,6,5）\n"
    "  --rate HZ      読み取り周期（既定 50、最大 200）\n"
    "  --seconds S    S 秒で終了（既定 0 = Ctrl-C まで）\n"
    "  --keep-torque  起動時のトルク OFF も行わない（読むだけなのは変わらない）\n"
    "  --baud N       ボーレート（既定 1000000）\n"
    "\n"
    "サーボには書き込まない。唯一の例外は起動時のトルク OFF 1 回だけ。\n", prog);
}

std::vector<uint8_t> parseIds(const std::string & s)
{
  std::vector<uint8_t> out;
  std::size_t p = 0;
  while (p < s.size()) {
    const std::size_t q = s.find(',', p);
    const std::string tok = s.substr(p, q == std::string::npos ? std::string::npos : q - p);
    if (!tok.empty()) {out.push_back(static_cast<uint8_t>(std::atoi(tok.c_str())));}
    if (q == std::string::npos) {break;}
    p = q + 1;
  }
  return out;
}

/// 1 本のバス。開けなかった脚は無い脚として扱い、もう片方だけで動かす。
struct Leg
{
  const char * tag;                  //!< "R" / "L"
  std::string port;
  bool enabled = false;
  std::unique_ptr<FeetechBus> bus;
  std::vector<ServoState> last;      //!< 直近に読めた値（欠損時の保持用ではなく報告用）
  std::uint64_t miss = 0;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string right = "/dev/feetech_right";
  std::string left = "/dev/feetech_left";
  std::string only = "both";
  std::string idstr = "1,2,3,4,6,5";
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
    if (a == "--right") {right = next("--right");} else if (a == "--left") {
      left = next("--left");
    } else if (a == "--only") {only = next("--only");} else if (a == "--ids") {
      idstr = next("--ids");
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

  const std::vector<uint8_t> ids = parseIds(idstr);
  if (ids.empty()) {
    std::fprintf(stderr, "--ids が空\n");
    return 2;
  }
  rate = rate < 1.0 ? 1.0 : (rate > 200.0 ? 200.0 : rate);

  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  Leg legs[2];
  legs[0].tag = "R"; legs[0].port = right;
  legs[1].tag = "L"; legs[1].port = left;
  const bool want[2] = {only != "left", only != "right"};

  int opened = 0;
  for (int k = 0; k < 2; ++k) {
    if (!want[k]) {continue;}
    legs[k].bus = std::make_unique<FeetechBus>(legs[k].port, baud);
    if (!legs[k].bus->open()) {
      std::fprintf(
        stderr, "%s が開けない（udev 固定名と電源を確認）。この脚は飛ばす。\n",
        legs[k].port.c_str());
      legs[k].bus.reset();
      continue;
    }
    legs[k].enabled = true;
    ++opened;

    // ここが唯一の書き込み。手で脚を動かせるように脱力させる。
    if (torque_off) {
      int off = 0;
      for (uint8_t id : ids) {
        if (legs[k].bus->enable_torque(id, false)) {++off;}
      }
      std::fprintf(
        stderr, "%s: %d/%zu 軸をトルク OFF（脱力）。手で動かせる。\n",
        legs[k].tag, off, ids.size());
    } else {
      std::fprintf(stderr, "%s: --keep-torque。トルクは触らない。読むだけ。\n", legs[k].tag);
    }
  }
  if (opened == 0) {
    std::fprintf(stderr, "どちらのバスも開けなかった。\n");
    return 1;
  }

  std::fprintf(stderr, "ID [%s] を %.0f Hz で読む。Ctrl-C で終了。\n", idstr.c_str(), rate);

  const auto t0 = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration<double>(1.0 / rate);
  std::uint64_t n = 0;
  std::vector<ServoState> st;

  while (!g_stop) {
    const auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - t0).count();
    if (seconds > 0.0 && t > seconds) {break;}
    ++n;

    std::printf("{\"t\":%.3f,\"n\":%llu", t, static_cast<unsigned long long>(n));
    for (int k = 0; k < 2; ++k) {
      if (!legs[k].enabled) {continue;}
      st.clear();
      legs[k].bus->sync_read_states(ids, st);

      // sync_read_states は要求順に詰めて返す。欠けた軸は valid=false で来る
      double volt = 0.0;
      int temp = 0, nvalid = 0;
      std::printf(",\"%s\":{\"raw\":[", legs[k].tag);
      for (std::size_t j = 0; j < ids.size(); ++j) {
        const ServoState s = (j < st.size()) ? st[j] : ServoState{};
        std::printf("%s%d", j ? "," : "", s.valid ? s.pos : 0);
        if (s.valid) {volt += s.volt; temp = temp > s.temp ? temp : s.temp; ++nvalid;}
      }
      std::printf("],\"valid\":[");
      for (std::size_t j = 0; j < ids.size(); ++j) {
        const ServoState s = (j < st.size()) ? st[j] : ServoState{};
        std::printf("%s%s", j ? "," : "", s.valid ? "true" : "false");
        if (!s.valid) {++legs[k].miss;}
      }
      std::printf(
        "],\"nvalid\":%d,\"volt\":%.2f,\"temp\":%d,\"miss\":%llu}",
        nvalid, nvalid ? volt / nvalid : 0.0, temp,
        static_cast<unsigned long long>(legs[k].miss));
    }
    std::printf("}\n");
    std::fflush(stdout);

    std::this_thread::sleep_until(now + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(period));
  }

  std::fprintf(stderr, "\n終了。%llu サイクル。\n", static_cast<unsigned long long>(n));
  return 0;
}
