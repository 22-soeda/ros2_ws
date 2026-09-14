// feetech_knee_goto: 膝の曲げ量を指定して、4 節リンクの逆変換 → サーボ角 → 生カウント
// まで通し、**--move を付けたときだけ** ID4 を動かす。膝 4 節リンクの実機確認用。
//
//   ros2 run feetech_servo feetech_knee_goto --bend 30                 # 計算と現在値だけ（動かない）
//   ros2 run feetech_servo feetech_knee_goto --leg R --bend 60         # 右脚
//   ros2 run feetech_servo feetech_knee_goto --bend 30 --move          # ★実機が動く
//   ros2 run feetech_servo feetech_knee_goto --move --repl             # ★対話。1 行に曲げ量 [deg]
//
// 足首の feetech_ankle_goto と同じ作法。違うのは 1 軸だけで、入力が「曲げ量」なこと。
//
// ===========================================================================
// 既定は読むだけ。書くのは --move を付けたときだけ
// ===========================================================================
//   * --move 無し: バスを開いて ID4 の現在位置を読み、順変換で曲げ量に戻して印字する。
//     **トルクにも目標位置にも触らない**。バスが開けなくても計算だけは出す。
//   * --move 有り: 「目標＝現在位置を書く → ID4 だけトルク ON → 始点から目標へ
//     smoothstep で補間しながら送る」の順（feetech_ankle_goto と同じ）。
//     終了時トルクは入ったまま（--off で切る）。
//
// ★★ 膝は機体の重さを支える軸。トルクを入れて曲げると**その場で崩れ落ちる**。
//     必ず機体を吊るか横に寝かせてから使うこと。足首より危ない。
//
// 角の 3 段（knee_fourbar.hpp）
//   曲げ量 bend      伸展 0・屈曲 +。leg_config の JOINT_LIMIT[KNEE] = 0..150 deg と同じ量
//   ロッカー角 θ4    bend から θ4 = σ_knee·bend + θ4_zero（θ4_zero = 89.3 deg が T ポーズ）
//   クランク角 θ2    θ4 から 4 節リンクの逆変換 (KN-7)。これがモータの軸
//   サーボ角 φ       φ = σ4·n4·θ2。**T ポーズ（bend = 0）での φ が servo_home.yaml の home**
//
// 何を確かめるツールか
//   指令 bend → kneeIk → θ2 → φ → count を出し、動かしたあと実測 count → φ → θ2 →
//   kneeFk → θ4 → bend に戻して差を出す。差がゼロ近くなら逆変換と順変換が互いに
//   整合しているということで、**実際に何度曲がったかは目で見る**（分度器を当てる）。
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "feetech_servo/feetech_bus.hpp"
#include "roboone_kinematics/knee_fourbar.hpp"
#include "roboone_kinematics/leg_config.hpp"

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

// --- servo_home.yaml（feetech_ankle_goto と同じ読み方）--------------------------
std::map<std::string, std::map<int, int>> loadHome(const std::string & path)
{
  std::map<std::string, std::map<int, int>> home;
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
        home[port][id] = kv.second["home"].as<int>();
      }
    }
  }
  if (home.empty()) {
    std::fprintf(stderr, "%s: 原点が 1 軸も入っていない\n", path.c_str());
    std::exit(2);
  }
  return home;
}

// --- servo_limits.yaml。EEPROM の角度リミット（生カウント）。[0,0] は制限なし -------
struct Window
{
  int lo{0}, hi{0};
  bool has{false};      //!< [0,0]（制限なし）でなく、窓が書いてあるか
};

std::map<std::string, std::map<int, Window>> loadLimits(const std::string & path)
{
  std::map<std::string, std::map<int, Window>> out;
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "リミットファイルを読めない（窓の検査は飛ばす）: %s\n  %s\n",
      path.c_str(), e.what());
    return out;
  }
  for (const auto & b : root["buses"]) {
    if (!b["port"] || !b["servos"]) {continue;}
    const std::string port = b["port"].as<std::string>();
    for (const auto & kv : b["servos"]) {
      const int id = kv.first.as<int>();
      Window w;
      const YAML::Node & v = kv.second;
      if (v.IsSequence() && v.size() == 2) {
        w.lo = v[0].as<int>();
        w.hi = v[1].as<int>();
      } else if (v.IsMap() && v["min"] && v["max"]) {
        w.lo = v["min"].as<int>();
        w.hi = v["max"].as<int>();
      } else {
        continue;
      }
      w.has = !(w.lo == 0 && w.hi == 0);
      out[port][id] = w;
    }
  }
  return out;
}

/// サーボ角 [rad]（T ポーズ = 0 ではなく絶対角）-> 生カウント。
/// T ポーズ（bend = 0）のサーボ角 phiTpose が home に対応する。
int countFromServo(double phi, double phiTpose, int home)
{
  const long v = std::lround(home + (phi - phiTpose) * kDeg * kStepsPerDeg);
  return static_cast<int>(std::clamp<long>(v, 0, kPosMax));
}
double servoFromCount(int pos, double phiTpose, int home)
{
  return phiTpose + (pos - home) / kStepsPerDeg / kDeg;
}

double smoothstep(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

const char * kneeStatusName(rk::KneeStatus s)
{
  switch (s) {
    case rk::KneeStatus::Ok: return "Ok";
    case rk::KneeStatus::Unreachable: return "Unreachable（三角形が閉じない = 可動域の外）";
    case rk::KneeStatus::Degenerate: return "Degenerate（退化姿勢）";
    case rk::KneeStatus::DeadPoint: return "DeadPoint（死点）";
  }
  return "?";
}

// --- 指令側の計算結果 ----------------------------------------------------------
struct Target
{
  double bend{0.0};            //!< 曲げ量 [rad]
  rk::KneePose pose;           //!< θ2 / θ3 / θ4
  double phi{0.0};             //!< サーボ角 [rad]
  int count{0};                //!< 生カウント
  double ratio{0.0};           //!< 伝達比 dθ4/dθ2
  double gamma{0.0};           //!< 伝達角 γ [rad]
  rk::KneeStatus status{rk::KneeStatus::Ok};
  bool inWindow{true};         //!< servo_limits.yaml の窓の内側か
  bool inJointLimit{true};     //!< leg_config の JOINT_LIMIT[KNEE] の内側か
};

/// 曲げ量 [rad] → ロッカー角 → クランク角 → サーボ角 → カウント。
Target solve(
  const rk::KneeParams & prm, double bend, double phiTpose, int home, const Window & win)
{
  Target t;
  t.bend = bend;
  t.status = rk::kneeIk(prm, rk::kneeRockerFromBend(prm, bend), t.pose);
  if (t.status != rk::KneeStatus::Ok) {return t;}
  t.phi = rk::kneeServoFromCrank(prm, t.pose.theta2);
  t.count = countFromServo(t.phi, phiTpose, home);
  rk::kneeRatio(prm, t.pose, t.ratio);
  t.gamma = rk::kneeTransmissionAngle(t.pose);
  if (win.has) {t.inWindow = (t.count >= win.lo && t.count <= win.hi);}
  const double d = bend * kDeg;
  t.inJointLimit = (d >= rk::config::JOINT_LIMIT_LO_DEG[rk::KNEE] - 1e-9 &&
    d <= rk::config::JOINT_LIMIT_HI_DEG[rk::KNEE] + 1e-9);
  return t;
}

bool targetOk(const Target & t)
{
  return t.status == rk::KneeStatus::Ok && t.inWindow;
}

void printTarget(const Target & t, int id, const Window & win)
{
  std::printf("指令  曲げ量 %+8.3f deg\n", t.bend * kDeg);
  std::printf("  逆変換 status = %s\n", kneeStatusName(t.status));
  if (t.status != rk::KneeStatus::Ok) {return;}
  std::printf(
    "  ロッカー θ4 %+8.3f deg   クランク θ2 %+8.3f deg   サーボ φ %+8.3f deg   count %4d\n",
    t.pose.theta4 * kDeg, t.pose.theta2 * kDeg, t.phi * kDeg, t.count);
  std::printf(
    "  伝達比 dθ4/dθ2 %6.3f   伝達角 γ %6.2f deg%s\n",
    t.ratio, t.gamma * kDeg,
    (t.gamma * kDeg < 40.0 || t.gamma * kDeg > 140.0) ? "  ★死点に近い（40-140 の外）" : "");
  if (!t.inJointLimit) {
    std::printf("  ★曲げ量が JOINT_LIMIT[KNEE] (%.0f..%.0f deg) の外\n",
      rk::config::JOINT_LIMIT_LO_DEG[rk::KNEE], rk::config::JOINT_LIMIT_HI_DEG[rk::KNEE]);
  }
  if (!t.inWindow) {
    std::printf(
      "  ★count %d が servo_limits.yaml の ID%d の窓 [%d, %d] の外。動かさない\n",
      t.count, id, win.lo, win.hi);
  }
}

/// 実測カウント → サーボ角 → クランク角 → 順変換 → 曲げ量。
void printMeasured(
  const rk::KneeParams & prm, int id, double phiTpose, int home,
  const ServoState & st, const Target * cmd)
{
  std::printf("実測\n");
  if (!st.valid) {
    std::printf("  ID%-2d  応答なし\n", id);
    return;
  }
  const double phi = servoFromCount(st.pos, phiTpose, home);
  const double th2 = rk::kneeCrankFromServo(prm, phi);
  std::printf(
    "  ID%-2d  count %4d  サーボ φ %+8.3f deg  クランク θ2 %+8.3f deg  %.1fV %d℃%s\n",
    id, st.pos, phi * kDeg, th2 * kDeg, st.volt, st.temp,
    st.err ? ("  err: " + feetech_servo::err_str(st.err)).c_str() : "");
  rk::KneePose pose;
  const rk::KneeStatus fk = rk::kneeFk(prm, th2, pose);
  if (fk != rk::KneeStatus::Ok) {
    std::printf("  順変換 status = %s\n", kneeStatusName(fk));
    return;
  }
  const double bend = rk::kneeBendFromRocker(prm, pose.theta4);
  std::printf("  順変換 status = Ok   ロッカー θ4 %+8.3f deg   曲げ量 %+8.3f deg\n",
    pose.theta4 * kDeg, bend * kDeg);
  if (cmd && cmd->status == rk::KneeStatus::Ok) {
    std::printf("  指令との差           曲げ量 %+8.3f deg\n", (bend - cmd->bend) * kDeg);
  }
}

/// 数回リトライして現在位置を読む（低電圧で応答が欠ける実機の癖）。
bool readNow(FeetechBus & bus, uint8_t id, ServoState & out, int attempts = 5)
{
  const std::vector<uint8_t> ids{id};
  std::vector<ServoState> tmp;
  for (int a = 0; a < attempts; ++a) {
    bus.sync_read_states(ids, tmp);
    if (!tmp.empty() && tmp[0].valid) {out = tmp[0]; return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  out = ServoState{};
  return false;
}

/// 始点 → 目標を duration 秒で補間して送る。★ここが唯一の位置指令。
void ramp(
  FeetechBus & bus, uint8_t id, int start, int target,
  double duration_s, double rate_hz, int speed, int acc)
{
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / rate_hz));
  const std::vector<uint8_t> ids{id};
  std::vector<int16_t> cmd(1, 0);
  const std::vector<uint16_t> speeds(1, static_cast<uint16_t>(speed));
  const std::vector<uint8_t> accs(1, static_cast<uint8_t>(acc));
  const auto t0 = Clock::now();
  auto next = t0;
  while (!g_stop) {
    const double t = std::chrono::duration<double>(Clock::now() - t0).count();
    if (t > duration_s) {break;}
    const double s = smoothstep(t / duration_s);
    cmd[0] = static_cast<int16_t>(std::lround(start + (target - start) * s));
    bus.sync_write_position(ids, cmd, speeds, accs);
    next += period;
    std::this_thread::sleep_until(next);
  }
  if (!g_stop) {
    cmd[0] = static_cast<int16_t>(target);
    bus.sync_write_position(ids, cmd, speeds, accs);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));   // 整定待ち
  }
}

void usage()
{
  std::printf(
    "使い方: feetech_knee_goto [--leg L|R] [--bend DEG] [--rocker DEG] [--crank DEG]\n"
    "                          [--move] [--repl] [--off] [--yes]\n"
    "                          [--duration S] [--rate HZ] [--speed N] [--acc N] [--torque N]\n"
    "                          [--id 4] [--home FILE] [--limits FILE]\n"
    "  --leg      どちらの脚か（既定 L）。バスは L=/dev/feetech_left / R=/dev/feetech_right\n"
    "  --bend     膝の曲げ量 [deg]。伸展 0・屈曲 +（JOINT_LIMIT は 0..150）\n"
    "  --rocker   ロッカー絶対角 θ4 [deg] で指定する（T ポーズが 89.3）\n"
    "  --crank    クランク角 θ2 [deg] で直接指定する（4 節リンクの逆変換を通さない）\n"
    "  --move     ★実機を動かす。無ければ計算と現在値の表示だけ（何も書かない）\n"
    "  --repl     標準入力から 1 行ずつ曲げ量 [deg] を読んで順に動かす（--move が要る）\n"
    "             空行で現在値を読み直す。q / Ctrl-D で終了\n"
    "  --off      トルクを切る（★膝が抜けて機体が落ちる。支えてから）。\n"
    "             --move と一緒なら動かしたあとに切り、**単独でも使える**\n"
    "  --yes      開始前の 3 秒カウントダウンを省略\n"
    "  --duration 到達までの秒数（既定 3）    --rate 送信周波数 Hz（既定 50）\n"
    "  --speed    step/s（既定 300。0 は動かない）  --acc 加速度（既定 20）\n"
    "  --torque   HLS 系の目標電流 reg44 0-1000（既定 1000。0 だと駆動しない）\n"
    "  --id       膝サーボの ID（既定 4）\n"
    "  --home     原点ファイル（既定 share/feetech_servo/config/servo_home.yaml）\n"
    "  --limits   角度リミット（既定 share/feetech_servo/config/servo_limits.yaml）\n"
    "\n"
    "★既定は読むだけ。トルク ON と位置指令は --move を付けたときだけ出る。\n"
    "★★膝は機体を支える軸。トルクを入れて曲げると崩れ落ちる。必ず機体を吊るか寝かせること。\n");
}
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, on_sigint);

  std::string leg = "L", homePath, limitsPath;
  bool haveTarget = false, move = false, repl = false, off = false, yes = false;
  double bendDeg = 0.0, duration = 3.0, rate = 50.0;
  int speed = 300, acc = 20, goalTorque = 1000, id = rk::kconfig::SERVO_ID;
  // 入力の種類。bend / rocker / crank のどれで受け取ったか
  enum class In { Bend, Rocker, Crank } in = In::Bend;
  double inDeg = 0.0;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&]() -> const char * {
        if (i + 1 >= argc) {usage(); std::exit(2);}
        return argv[++i];
      };
    if (a == "--leg") {leg = need();} else if (
      a == "--bend") {inDeg = std::atof(need()); in = In::Bend; haveTarget = true;} else if (
      a == "--rocker") {inDeg = std::atof(need()); in = In::Rocker; haveTarget = true;} else if (
      a == "--crank") {inDeg = std::atof(need()); in = In::Crank; haveTarget = true;} else if (
      a == "--move") {move = true;} else if (a == "--repl") {repl = true;} else if (
      a == "--off") {off = true;} else if (a == "--yes" || a == "-y") {yes = true;} else if (
      a == "--duration") {duration = std::atof(need());} else if (
      a == "--rate") {rate = std::atof(need());} else if (
      a == "--speed") {speed = std::atoi(need());} else if (
      a == "--acc") {acc = std::atoi(need());} else if (
      a == "--torque") {goalTorque = std::atoi(need());} else if (
      a == "--id") {id = std::atoi(need());} else if (
      a == "--home") {homePath = need();} else if (a == "--limits") {limitsPath = need();} else {
      usage();
      return (a == "--help" || a == "-h") ? 0 : 2;
    }
  }
  if (repl && !move) {
    std::fprintf(stderr, "--repl は --move と一緒に使う（動かさない対話には意味が無い）\n");
    return 2;
  }
  if (!haveTarget && !repl && !off) {
    std::fprintf(stderr,
      "何をするか指定が無い。--bend（または --rocker / --crank）か --repl、"
      "トルクを切るだけなら --off\n");
    usage();
    return 2;
  }
  if (speed == 0) {
    // 実機は速度レジスタ 0 だと「トルクは入るが動かない」（docs/commands.md）
    std::fprintf(stderr, "--speed 0 は実機では動かない。300 程度を指定すること\n");
    return 2;
  }

  const char * share = std::getenv("FEETECH_SHARE");
  const std::string shareDir = share ? std::string(share)
    : std::string("/home/auto/ros2_ws/install/feetech_servo/share/feetech_servo");
  if (homePath.empty()) {homePath = shareDir + "/config/servo_home.yaml";}
  if (limitsPath.empty()) {limitsPath = shareDir + "/config/servo_limits.yaml";}

  const auto home = loadHome(homePath);
  const auto limits = loadLimits(limitsPath);

  const bool right = (leg == "R" || leg == "r");
  const std::string port = right ? "/dev/feetech_right" : "/dev/feetech_left";
  if (home.find(port) == home.end() || home.at(port).find(id) == home.at(port).end()) {
    std::fprintf(stderr, "原点ファイルに %s の ID%d が無い\n", port.c_str(), id);
    return 2;
  }
  const int homeCount = home.at(port).at(id);
  Window win;
  if (limits.count(port) && limits.at(port).count(id)) {win = limits.at(port).at(id);}

  const rk::KneeParams prm = rk::makeKneeParams(right ? rk::Side::RIGHT : rk::Side::LEFT);

  // T ポーズ（曲げ量 0 = 脚が伸び切った姿勢）のサーボ角。これが home に対応する。
  double phiTpose = 0.0;
  {
    rk::KneePose ext;
    if (rk::kneeIk(prm, rk::kneeRockerFromBend(prm, 0.0), ext) != rk::KneeStatus::Ok) {
      std::fprintf(stderr, "伸び切り姿勢の逆変換が解けない（knee_config.hpp を確認）\n");
      return 2;
    }
    phiTpose = rk::kneeServoFromCrank(prm, ext.theta2);
  }

  std::printf("%s脚  %s  ID%d(home %d", right ? "右" : "左", port.c_str(), id, homeCount);
  if (win.has) {std::printf(", 窓 [%d, %d]", win.lo, win.hi);} else {std::printf(", 窓なし");}
  std::printf(")\n");
  std::printf("曲げ量 0（T ポーズ・伸び切り）= ロッカー θ4 %.3f deg / サーボ φ %.3f deg = count %d\n\n",
    prm.theta4Zero * kDeg, phiTpose * kDeg, homeCount);

  // 入力を曲げ量に直す
  auto bendFromInput = [&](double deg) -> double {
      switch (in) {
        case In::Rocker: return rk::kneeBendFromRocker(prm, deg / kDeg);
        case In::Crank: {
            rk::KneePose p;
            if (rk::kneeFk(prm, deg / kDeg, p) != rk::KneeStatus::Ok) {
              return std::numeric_limits<double>::quiet_NaN();
            }
            return rk::kneeBendFromRocker(prm, p.theta4);
          }
        default: return deg / kDeg;
      }
    };

  // --- 1) まず計算だけ（バスが無くてもここまでは出る）---
  Target tgt;
  if (haveTarget) {
    const double bend = bendFromInput(inDeg);
    if (std::isnan(bend)) {
      std::fprintf(stderr, "クランク角 %.3f deg では 4 節リンクが組めない\n", inDeg);
      return 2;
    }
    tgt = solve(prm, bend, phiTpose, homeCount, win);
    printTarget(tgt, id, win);
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
  ServoState st;
  if (!readNow(bus, static_cast<uint8_t>(id), st)) {
    std::fprintf(stderr, "ID%d の現在位置が読めない。電源・電圧を確認\n", id);
    if (move) {return 2;}
  }
  printMeasured(prm, id, phiTpose, homeCount, st, nullptr);

  if (!move) {
    // --off は単独でも使える。**トルクを切るのは --move が無くても行う**
    // （「切る」は動かす指令ではないので、--move を要求すると使いにくい）。
    if (off) {
      std::printf("\nトルク OFF: ID%d（★膝が抜けて機体が落ちる）\n", id);
      if (!bus.enable_torque(static_cast<uint8_t>(id), false)) {
        std::fprintf(stderr, "ID%d のトルクを切れなかった（応答なし）\n", id);
        return 2;
      }
      return 0;
    }
    std::printf("\n（--move が無いので何も書かずに終了）\n");
    return 0;
  }

  // --- 3) ★ここから書く。目標＝現在位置を書いてからトルク ON（投入時の飛び出し防止）---
  {
    const std::vector<uint8_t> ids{static_cast<uint8_t>(id)};
    const std::vector<int16_t> hold{static_cast<int16_t>(st.pos)};
    bus.sync_write_position(ids, hold);
    if (!bus.enable_torque(static_cast<uint8_t>(id), true)) {
      std::fprintf(stderr, "ID%d のトルクが入らない。中止\n", id);
      return 2;
    }
    std::printf("\nトルク ON: ID%d のみ（★膝が支えている。機体を吊るか寝かせること）\n", id);
  }

  auto moveTo = [&](const Target & t) -> bool {
      if (!targetOk(t)) {
        std::printf("  → 目標が不正なので動かさない\n");
        return false;
      }
      std::printf("  %.1f 秒で count %d → %d\n", duration, st.pos, t.count);
      ramp(bus, static_cast<uint8_t>(id), st.pos, t.count, duration, rate, speed, acc);
      if (g_stop) {return false;}
      readNow(bus, static_cast<uint8_t>(id), st);
      printMeasured(prm, id, phiTpose, homeCount, st, &t);
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

  // --- 4) 対話モード: 1 行ずつ曲げ量 [deg] を読んで動かす ---
  if (repl && !g_stop) {
    std::printf("\n--- 対話モード: 曲げ量 [deg] を 1 行ずつ。空行で読み直し、q で終了 ---\n");
    std::string line;
    while (!g_stop) {
      std::printf("> ");
      std::fflush(stdout);
      if (!std::getline(std::cin, line)) {break;}
      if (line == "q" || line == "quit" || line == "exit") {break;}
      std::istringstream ss(line);
      double v = 0.0;
      if (!(ss >> v)) {
        readNow(bus, static_cast<uint8_t>(id), st);
        printMeasured(prm, id, phiTpose, homeCount, st, nullptr);
        continue;
      }
      // 対話では常に曲げ量で受ける（--rocker / --crank は起動時の 1 回だけ）
      const Target t = solve(prm, v / kDeg, phiTpose, homeCount, win);
      printTarget(t, id, win);
      moveTo(t);
    }
  }

  if (off) {
    std::printf("\nトルク OFF（★膝が抜ける）\n");
    bus.enable_torque(static_cast<uint8_t>(id), false);
  } else {
    std::printf("\n終了。トルクは入ったまま（切るなら --off、または feetech_shell の off）\n");
  }
  return g_stop ? 1 : 0;
}
