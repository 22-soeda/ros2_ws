// feetech_ankle_goto: 足首の関節角 (θ5, θ6) を指定して、足首パラレルリンクの
// 逆変換 → サーボ角 → 生カウント まで通し、**--move を付けたときだけ**
// ID6 / ID5 を同時に動かす。逆運動学（ankle_parallel.hpp）の実機確認用。
//
//   ros2 run feetech_servo feetech_ankle_goto --th5 10 --th6 -5             # 計算と現在値だけ（動かない）
//   ros2 run feetech_servo feetech_ankle_goto --leg R --pitch -5 --roll 10  # pitch/roll で指定（θ5 = ピッチ, θ6 = ロール）
//   ros2 run feetech_servo feetech_ankle_goto --th5 10 --th6 -5 --move      # ★実機が動く
//   ros2 run feetech_servo feetech_ankle_goto --move --repl                 # ★対話。1 行 "θ5 θ6" で順に動かす
//
// ===========================================================================
// 既定は読むだけ。書くのは --move を付けたときだけ
// ===========================================================================
//   * --move 無し: バスを開いて ID6/ID5 の現在位置を読み、順変換で (θ5,θ6) に戻して
//     印字する。**トルクにも目標位置にも触らない**。バスが開けなくても計算だけは出す。
//   * --move 有り: 「目標＝現在位置を書く → ID6/ID5 だけトルク ON → 始点から目標へ
//     smoothstep で補間しながら sync write」の順（feetech_goto_test と同じ作法）。
//     2 軸を **同じパケット** で送るので、片方ずつ動いて機構に無理がかかることは無い。
//     終了時トルクは入ったまま（--off で切る）。
//
// 何を確かめるツールか
//   指令 (θ5,θ6) → ankleClampJoints → ankleIk → ankleServoFromCrank → count を出し、
//   動かしたあと実測 count → ankleCrankFromServo → ankleFk → (θ5,θ6) に戻して差を出す。
//   差がゼロ近くなら「逆変換と順変換が互いに整合している」ことは言えるが、
//   **足裏が本当にその角度になっているかは目で見る**（ここは自己整合の確認しかできない）。
//
// θ5 / θ6 と pitch / roll の対応（ankle_config.hpp の約束。2026-09-14 に入れ替え済み）
//   θ5 = Ry = ピッチ（上側ピボット・腰に近い方）   θ6 = Rx = ロール（下側ピボット）
//   --pitch は θ5、--roll は θ6 に入る。対応を疑うならまず --th5 だけ振って
//   足裏が前後に傾く（ピッチ）ことを目で見ること。
//
// ID と鎖の対応（leg_live_test と同じ）
//   鎖 0 = ID6（短ロッド 80mm・下側）   鎖 1 = ID5（長ロッド 115mm・上側）
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "feetech_servo/feetech_bus.hpp"
#include "roboone_kinematics/ankle_parallel.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;
namespace rk = roboone_kinematics;
using Clock = std::chrono::steady_clock;

namespace
{
volatile std::sig_atomic_t g_stop = 0;
void on_sigint(int) {g_stop = 1;}

constexpr double kStepsPerDeg = 4096.0 / 360.0;
constexpr double kDeg = 180.0 / M_PI;
constexpr int kPosMax = 4095;

// --- servo_home.yaml（leg_live_test と同じ読み方）------------------------------
struct HomeConfig
{
  std::map<std::string, std::map<int, int>> home;   // port -> (id -> 原点カウント)
};

HomeConfig loadHome(const std::string & path)
{
  HomeConfig cfg;
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "原点ファイルを読めない: %s\n  %s\n", path.c_str(), e.what());
    std::exit(2);
  }
  for (const auto & b : root["buses"]) {
    if (!b["port"] || !b["servos"]) {continue;}
    const std::string port = b["port"].as<std::string>();
    for (const auto & kv : b["servos"]) {
      const int id = kv.first.as<int>();
      if (kv.second.IsMap() && kv.second["home"]) {
        cfg.home[port][id] = kv.second["home"].as<int>();
      }
    }
  }
  if (cfg.home.empty()) {
    std::fprintf(stderr, "%s: 原点が 1 軸も入っていない\n", path.c_str());
    std::exit(2);
  }
  return cfg;
}

/// サーボ角 [rad]（T ポーズ = 0）-> 生カウント。leg_live_test の countToDeg の逆。
int countFromServo(double phi, int home)
{
  const long v = std::lround(home + phi * kDeg * kStepsPerDeg);
  return static_cast<int>(std::clamp<long>(v, 0, kPosMax));
}
double servoFromCount(int pos, int home) {return (pos - home) / kStepsPerDeg / kDeg;}

double smoothstep(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

const char * ikStatusName(rk::AnkleIkStatus s)
{
  switch (s) {
    case rk::AnkleIkStatus::Ok: return "Ok";
    case rk::AnkleIkStatus::Unreachable: return "Unreachable（ロッドが届かない）";
    case rk::AnkleIkStatus::Degenerate: return "Degenerate";
  }
  return "?";
}
const char * fkStatusName(rk::AnkleFkStatus s)
{
  switch (s) {
    case rk::AnkleFkStatus::Ok: return "Ok";
    case rk::AnkleFkStatus::Clamped: return "Clamped（窓の縁）";
    case rk::AnkleFkStatus::NoCurve: return "NoCurve";
    case rk::AnkleFkStatus::NotConverged: return "NotConverged";
  }
  return "?";
}

// --- 指令側の計算結果 ----------------------------------------------------------
struct Target
{
  double th5{0.0}, th6{0.0};          // clamp 後 [rad]
  bool clamped{false};
  rk::AnkleIkResult ik;
  double phi[rk::kAnkleChains]{};     // サーボ角 [rad]
  int count[rk::kAnkleChains]{};      // 生カウント
  bool crankInLimit{true};            // CRANK_LIMIT_DEG の内側か
};

/// (θ5,θ6) [rad] → clamp → IK → サーボ角 → カウント。動かすかどうかに関係なく毎回通す。
Target solve(const rk::AnkleParams & prm, double th5, double th6, const int home[2])
{
  Target t;
  // ★2026-09-15: 丸めをやめた。エンベロープの外かどうかは報告するだけで、
  // 動かすかどうかは逆変換の status とクランクリミットで決める。
  t.th5 = th5;
  t.th6 = th6;
  t.clamped = rk::ankleOutsideEnvelope(th5, th6);
  // clamp=false: 届かないときは最寄りを書き戻さず、status で弾く
  t.ik = rk::ankleIk(prm, t.th5, t.th6, /*clamp=*/false);
  for (int i = 0; i < rk::kAnkleChains; ++i) {
    t.phi[i] = rk::ankleServoFromCrank(prm, i, t.ik.q[i]);
    t.count[i] = countFromServo(t.phi[i], home[i]);
    if (t.ik.q[i] < prm.qMin[i] || t.ik.q[i] > prm.qMax[i]) {t.crankInLimit = false;}
  }
  return t;
}

bool targetOk(const Target & t)
{
  return t.ik.status == rk::AnkleIkStatus::Ok && t.crankInLimit;
}

void printTarget(const Target & t, const int ids[2], double reqTh5, double reqTh6)
{
  std::printf("指令  θ5(ピッチ) %+8.3f deg   θ6(ロール) %+8.3f deg", reqTh5 * kDeg, reqTh6 * kDeg);
  if (t.clamped) {
    std::printf("  → 窓に丸めた: θ5 %+.3f / θ6 %+.3f", t.th5 * kDeg, t.th6 * kDeg);
  }
  std::printf("\n  逆変換 status = %s\n", ikStatusName(t.ik.status));
  for (int i = 0; i < rk::kAnkleChains; ++i) {
    std::printf(
      "  鎖%d ID%-2d  クランク q %+8.3f deg  サーボ φ %+8.3f deg  count %4d   Δ余裕 %6.1f mm²%s\n",
      i, ids[i], t.ik.q[i] * kDeg, t.phi[i] * kDeg, t.count[i], t.ik.delta[i],
      t.ik.delta[i] <= 0.0 ? "  ★届かない" : "");
  }
  if (!t.crankInLimit) {
    std::printf("  ★クランク角が CRANK_LIMIT_DEG（servo_limits の窓）の外。動かさない\n");
  }
}

/// 実測カウント → 順変換 → (θ5,θ6)。指令との差も出す。
void printMeasured(
  const rk::AnkleParams & prm, const int ids[2], const int home[2],
  const std::vector<ServoState> & st, const Target * cmd, double & th6Seed)
{
  double q[rk::kAnkleChains] = {0.0, 0.0};
  bool ok = true;
  std::printf("実測\n");
  for (int i = 0; i < rk::kAnkleChains; ++i) {
    if (static_cast<size_t>(i) >= st.size() || !st[i].valid) {
      std::printf("  鎖%d ID%-2d  応答なし\n", i, ids[i]);
      ok = false;
      continue;
    }
    const double phi = servoFromCount(st[i].pos, home[i]);
    q[i] = rk::ankleCrankFromServo(prm, i, phi);
    std::printf(
      "  鎖%d ID%-2d  count %4d  サーボ φ %+8.3f deg  クランク q %+8.3f deg  %.1fV %d℃%s\n",
      i, ids[i], st[i].pos, phi * kDeg, q[i] * kDeg, st[i].volt, st[i].temp,
      st[i].err ? ("  err: " + feetech_servo::err_str(st[i].err)).c_str() : "");
  }
  if (!ok) {return;}
  const rk::AnkleFkResult fk = rk::ankleFk(prm, q, th6Seed);
  th6Seed = fk.th6;
  std::printf(
    "  順変換 status = %s   θ5(ピッチ) %+8.3f deg   θ6(ロール) %+8.3f deg\n",
    fkStatusName(fk.status), fk.th5 * kDeg, fk.th6 * kDeg);
  if (cmd) {
    std::printf(
      "  指令との差       θ5 %+8.3f deg          θ6 %+8.3f deg\n",
      (fk.th5 - cmd->th5) * kDeg, (fk.th6 - cmd->th6) * kDeg);
  }
}

/// 数回リトライして現在位置を読む（低電圧で応答が欠ける実機の癖）。
bool readNow(FeetechBus & bus, const std::vector<uint8_t> & ids, std::vector<ServoState> & out,
  int attempts = 5)
{
  std::vector<ServoState> got(ids.size());
  std::vector<bool> have(ids.size(), false);
  std::vector<ServoState> tmp;
  for (int a = 0; a < attempts; ++a) {
    bus.sync_read_states(ids, tmp);
    for (size_t i = 0; i < ids.size(); ++i) {
      if (!have[i] && i < tmp.size() && tmp[i].valid) {got[i] = tmp[i]; have[i] = true;}
    }
    if (std::all_of(have.begin(), have.end(), [](bool b) {return b;})) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  out = got;
  return std::all_of(have.begin(), have.end(), [](bool b) {return b;});
}

/// 始点 → 目標を duration 秒で補間して 2 軸同時に送る。★ここが唯一の位置指令。
void ramp(
  FeetechBus & bus, const std::vector<uint8_t> & ids, const int start[2], const int target[2],
  double duration_s, double rate_hz, int speed, int acc)
{
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / rate_hz));
  std::vector<int16_t> cmd(2, 0);
  const std::vector<uint16_t> speeds(2, static_cast<uint16_t>(speed));
  const std::vector<uint8_t> accs(2, static_cast<uint8_t>(acc));
  const auto t0 = Clock::now();
  auto next = t0;
  while (!g_stop) {
    const double t = std::chrono::duration<double>(Clock::now() - t0).count();
    if (t > duration_s) {break;}
    const double s = smoothstep(t / duration_s);
    for (int i = 0; i < 2; ++i) {
      cmd[i] = static_cast<int16_t>(std::lround(start[i] + (target[i] - start[i]) * s));
    }
    bus.sync_write_position(ids, cmd, speeds, accs);
    next += period;
    std::this_thread::sleep_until(next);
  }
  if (!g_stop) {
    for (int i = 0; i < 2; ++i) {cmd[i] = static_cast<int16_t>(target[i]);}
    bus.sync_write_position(ids, cmd, speeds, accs);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));   // 整定待ち
  }
}

void usage()
{
  std::printf(
    "使い方: feetech_ankle_goto [--leg L|R] [--th5 DEG] [--th6 DEG] [--pitch DEG] [--roll DEG]\n"
    "                           [--move] [--repl] [--off] [--yes]\n"
    "                           [--duration S] [--rate HZ] [--speed N] [--acc N] [--torque N]\n"
    "                           [--ids 6,5] [--home FILE]\n"
    "  --leg      どちらの脚か（既定 L）。バスは L=/dev/feetech_left / R=/dev/feetech_right\n"
    "  --th5/--th6  足首の関節角 [deg]。θ5 = ピッチ（上側）/ θ6 = ロール（下側）\n"
    "  --pitch/--roll  同じものを名前で。--pitch → θ5、--roll → θ6 に入る\n"
    "  --move     ★実機を動かす。無ければ計算と現在値の表示だけ（何も書かない）\n"
    "  --repl     標準入力から 1 行 \"θ5 θ6\" [deg] を読んで順に動かす（--move が要る）\n"
    "             空行で現在値を読み直す。q / Ctrl-D で終了\n"
    "  --off      トルクを切る（既定は入れたまま。足が落ちる姿勢なら付けない）。\n"
    "             --move と一緒なら動かしたあとに切り、**単独でも使える**\n"
    "  --yes      開始前の 3 秒カウントダウンを省略\n"
    "  --duration 到達までの秒数（既定 2）    --rate 送信周波数 Hz（既定 50）\n"
    "  --speed    step/s（既定 600。0 は動かない）  --acc 加速度（既定 20）\n"
    "  --torque   HLS 系の目標電流 reg44 0-1000（既定 1000。0 だと駆動しない）\n"
    "  --ids      鎖0(短ロッド),鎖1(長ロッド) の順のサーボ ID（既定 6,5）\n"
    "  --home     原点ファイル（既定 share/feetech_servo/config/servo_home.yaml）\n"
    "\n"
    "★既定は読むだけ。トルク ON と位置指令は --move を付けたときだけ出る。\n");
}
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, on_sigint);

  std::string leg = "L", homePath, idsArg = "6,5";
  bool haveTh5 = false, haveTh6 = false, move = false, repl = false, off = false, yes = false;
  double th5Deg = 0.0, th6Deg = 0.0, duration = 2.0, rate = 50.0;
  int speed = 600, acc = 20, goalTorque = 1000;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&]() -> const char * {
        if (i + 1 >= argc) {usage(); std::exit(2);}
        return argv[++i];
      };
    if (a == "--leg") {leg = need();} else if (
      a == "--th5" || a == "--pitch") {th5Deg = std::atof(need()); haveTh5 = true;} else if (
      a == "--th6" || a == "--roll") {th6Deg = std::atof(need()); haveTh6 = true;} else if (
      a == "--move") {move = true;} else if (a == "--repl") {repl = true;} else if (
      a == "--off") {off = true;} else if (a == "--yes" || a == "-y") {yes = true;} else if (
      a == "--duration") {duration = std::atof(need());} else if (
      a == "--rate") {rate = std::atof(need());} else if (
      a == "--speed") {speed = std::atoi(need());} else if (
      a == "--acc") {acc = std::atoi(need());} else if (
      a == "--torque") {goalTorque = std::atoi(need());} else if (
      a == "--ids") {idsArg = need();} else if (a == "--home") {homePath = need();} else {
      usage();
      return (a == "--help" || a == "-h") ? 0 : 2;
    }
  }
  if (repl && !move) {
    std::fprintf(stderr, "--repl は --move と一緒に使う（動かさない対話には意味が無い）\n");
    return 2;
  }
  if (!haveTh5 && !haveTh6 && !repl && !off) {
    std::fprintf(stderr,
      "何をするか指定が無い。--th5/--th6（または --pitch/--roll）か --repl、"
      "トルクを切るだけなら --off\n");
    usage();
    return 2;
  }
  if (speed == 0) {
    // 実機は速度レジスタ 0 だと「トルクは入るが動かない」（docs/commands.md）
    std::fprintf(stderr, "--speed 0 は実機では動かない。600 程度を指定すること\n");
    return 2;
  }

  if (homePath.empty()) {
    const char * share = std::getenv("FEETECH_SHARE");
    homePath = share ? std::string(share) + "/config/servo_home.yaml"
      : std::string("/home/auto/ros2_ws/install/feetech_servo/share/feetech_servo"
        "/config/servo_home.yaml");
  }
  const HomeConfig homeCfg = loadHome(homePath);

  const bool right = (leg == "R" || leg == "r");
  const std::string port = right ? "/dev/feetech_right" : "/dev/feetech_left";
  if (homeCfg.home.find(port) == homeCfg.home.end()) {
    std::fprintf(stderr, "原点ファイルに %s が無い\n", port.c_str());
    return 2;
  }
  const auto & hmap = homeCfg.home.at(port);

  int ids[2] = {6, 5};
  {
    std::istringstream ss(idsArg);
    std::string tok;
    int n = 0;
    while (std::getline(ss, tok, ',') && n < 2) {ids[n++] = std::atoi(tok.c_str());}
    if (n != 2) {std::fprintf(stderr, "--ids は 2 個（鎖0,鎖1 の順）\n"); return 2;}
  }
  int home[2];
  for (int i = 0; i < 2; ++i) {
    if (hmap.find(ids[i]) == hmap.end()) {
      std::fprintf(stderr, "ID %d の原点が %s に無い\n", ids[i], port.c_str());
      return 2;
    }
    home[i] = hmap.at(ids[i]);
  }
  const std::vector<uint8_t> busIds = {static_cast<uint8_t>(ids[0]), static_cast<uint8_t>(ids[1])};

  const rk::AnkleParams prm = rk::makeAnkleParams(right ? rk::Side::RIGHT : rk::Side::LEFT);

  std::printf("%s脚  %s  鎖0=ID%d(home %d)  鎖1=ID%d(home %d)\n",
    right ? "右" : "左", port.c_str(), ids[0], home[0], ids[1], home[1]);
  std::printf("θ5 = ピッチ（上側ピボット・Ry）/ θ6 = ロール（下側ピボット・Rx）。"
    "--pitch→θ5, --roll→θ6\n\n");

  // --- 1) まず計算だけ（バスが無くてもここまでは出る）---
  Target tgt;
  bool haveTarget = haveTh5 || haveTh6;
  if (haveTarget) {
    tgt = solve(prm, th5Deg / kDeg, th6Deg / kDeg, home);
    printTarget(tgt, ids, th5Deg / kDeg, th6Deg / kDeg);
    std::printf("\n");
  }

  // --- 2) バスを開いて現在値を読む（書かない）---
  FeetechBus bus(port, 1000000, 0, 20, feetech_servo::Family::kHls);
  if (!bus.open()) {
    std::fprintf(stderr, "%s を開けない（motion が掴んでいないか、udev ルールを確認）\n",
      port.c_str());
    return move ? 2 : 0;   // 計算だけが目的なら失敗にしない
  }
  bus.set_goal_torque(static_cast<uint16_t>(goalTorque));
  std::vector<ServoState> st;
  double th6Seed = 0.0;
  if (!readNow(bus, busIds, st)) {
    std::fprintf(stderr, "ID%d/ID%d の現在位置が読めない。電源・電圧を確認\n", ids[0], ids[1]);
    if (move) {return 2;}
  }
  printMeasured(prm, ids, home, st, nullptr, th6Seed);

  if (!move) {
    // --off は単独でも使える。**トルクを切るのは --move が無くても行う**
    // （「切る」は動かす指令ではないので、--move を要求すると使いにくい）。
    if (off) {
      int n = 0;
      for (uint8_t sid : busIds) {n += bus.enable_torque(sid, false) ? 1 : 0;}
      std::printf("\nトルク OFF（%d / 2 軸）\n", n);
      return n == 2 ? 0 : 2;
    }
    std::printf("\n（--move が無いので何も書かずに終了）\n");
    return 0;
  }

  // --- 3) ★ここから書く。目標＝現在位置を書いてからトルク ON（投入時の飛び出し防止）---
  {
    const std::vector<int16_t> hold = {static_cast<int16_t>(st[0].pos), static_cast<int16_t>(st[1].pos)};
    bus.sync_write_position(busIds, hold);
    int on = 0;
    for (uint8_t id : busIds) {on += bus.enable_torque(id, true) ? 1 : 0;}
    std::printf("\nトルク ON: %d / 2 軸（ID%d, ID%d のみ）\n", on, ids[0], ids[1]);
    if (on != 2) {std::fprintf(stderr, "トルクが入らない軸がある。中止\n"); return 2;}
  }

  auto moveTo = [&](const Target & t) -> bool {
      if (!targetOk(t)) {
        std::printf("  → 目標が不正なので動かさない\n");
        return false;
      }
      int start[2] = {st[0].pos, st[1].pos};
      std::printf("  %.1f 秒で count (%d, %d) → (%d, %d)\n", duration, start[0], start[1],
        t.count[0], t.count[1]);
      ramp(bus, busIds, start, t.count, duration, rate, speed, acc);
      if (g_stop) {return false;}
      readNow(bus, busIds, st);
      printMeasured(prm, ids, home, st, &t, th6Seed);
      return true;
    };

  if (haveTarget) {
    if (!yes) {
      std::printf("3 秒後に動かす（Ctrl-C で中止）");
      std::fflush(stdout);
      for (int i = 3; i > 0 && !g_stop; --i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::printf(" %d", i - 1);
        std::fflush(stdout);
      }
      std::printf("\n");
    }
    if (!g_stop) {moveTo(tgt);}
  }

  // --- 4) 対話モード: 1 行 "θ5 θ6" [deg] を読んで順に動かす ---
  if (repl && !g_stop) {
    std::printf("\n--- 対話モード: \"θ5 θ6\" [deg] を 1 行ずつ。空行で読み直し、q で終了 ---\n");
    std::string line;
    while (!g_stop) {
      std::printf("> ");
      std::fflush(stdout);
      if (!std::getline(std::cin, line)) {break;}
      if (line == "q" || line == "quit" || line == "exit") {break;}
      std::istringstream ss(line);
      double a = 0.0, b = 0.0;
      if (!(ss >> a)) {
        readNow(bus, busIds, st);
        printMeasured(prm, ids, home, st, nullptr, th6Seed);
        continue;
      }
      if (!(ss >> b)) {std::printf("  2 つ要る: θ5 θ6\n"); continue;}
      const Target t = solve(prm, a / kDeg, b / kDeg, home);
      printTarget(t, ids, a / kDeg, b / kDeg);
      moveTo(t);
    }
  }

  if (off) {
    int n = 0;
    for (uint8_t id : busIds) {n += bus.enable_torque(id, false) ? 1 : 0;}
    std::printf("\nトルク OFF（%d / 2 軸）\n", n);
  } else {
    std::printf("\n終了。トルクは入ったまま（切るなら --off、または feetech_shell の off）\n");
  }
  return g_stop ? 1 : 0;
}
