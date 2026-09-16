// motion_leg_goto — 片脚の足裏の目標 (x, y, z, roll, pitch, yaw) を打つと、脚 IK →
// 膝 4 節リンク・足首パラレルリンク → サーボ角 → 生カウント まで通し、
// **--move を付けたときだけ** その脚の 6 軸を同時に動かす。脚 IK 全体の実機確認用。
//
//   ros2 run roboone_motion motion_leg_goto --leg R --p 0 -89.3 -261 --rpy 0 0 0        # 計算と現在値だけ
//   ros2 run roboone_motion motion_leg_goto --leg R --p 0 -89.3 -261 --rpy 0 -20 0 --move   # ★実機が動く
//   ros2 run roboone_motion motion_leg_goto --leg L --move --repl    # ★対話。1 行 "x y z roll pitch yaw"
//   ros2 run roboone_motion motion_leg_goto --leg R --off            # その脚 6 軸のトルクを切る
//
// feetech_ankle_goto / feetech_knee_goto と同じ作法で、こちらは足裏の座標で指定する。
// motion ノードが使っているのと同じ変換（body_pose.hpp の servoFromFootPose /
// footPoseFromServo と ServoMap の生カウント変換）をそのまま通すので、ここで出た
// カウントは motion ノードが出すものと一致する。
//
// ===========================================================================
// 座標
// ===========================================================================
//   p    [mm]  機体座標 Σ_B（x 前 / y 左 / z 上、原点はボディ原点 = 股 3 軸の高さ）。
//              股中心は右脚 (0, -89.3, 0) / 左脚 (0, +89.3, 0)。
//              立位の例: 右脚 (0, -89.3, -261)、左脚 (0, +89.3, -261)
//   rpy  [deg] 足裏の姿勢（roll, pitch, yaw）。水平なら 0 0 0。pitch は - でつま先上げ
//   home_pose.yaml の foot と同じ取り方。--rel を付けると x, y を股中心からの相対で受ける。
//
// ===========================================================================
// 既定は読むだけ。書くのは --move / --off を付けたときだけ
// ===========================================================================
//   * --move 無し: バスを開いて 6 軸の現在位置を読み、順変換で足裏の姿勢に戻して印字する。
//     **トルクにも目標位置にも触らない**。バスが開けなくても計算だけは出す。
//   * --move 有り: 「目標＝現在位置を書く → 6 軸のトルク ON → 始点から目標へ smoothstep
//     で補間しながら 1 パケットで sync write」の順。6 軸が同じ区間で同時に動く。
//     終了時トルクは入ったまま（--off で切る）。
//   * 動かさない条件は 2 つだけ: **IK が解けない**（膝の三角形が閉じない、足首の
//     ロッドが届かない等）/ カウントが servo_limits.yaml の窓の外。理由を印字して止まる。
//     ★2026-09-15: 足首のエンベロープ (TH5_ENVELOPE_DEG)・関節リミット
//     (JOINT_LIMIT)・ReachLevel では止めない（解けるなら動かす）。外に出ていれば
//     表示に ★ が付くだけなので、可動域は印字を見ながら手で詰めること。
//
// ★★ 脚 6 軸をまとめて動かす。トルクを入れた脚が床を蹴って機体が倒れる・机から落ちる。
//     **必ず機体を吊るか、脚が空中にある姿勢で使うこと。**
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "feetech_servo/feetech_bus.hpp"
#include "roboone_motion/body_pose.hpp"
#include "roboone_motion/servo_map.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;
namespace rm = roboone_motion;
namespace rk = roboone_kinematics;
using Clock = std::chrono::steady_clock;

namespace
{
volatile std::sig_atomic_t g_stop = 0;
void on_sigint(int) {g_stop = 1;}

constexpr double kDeg = 180.0 / M_PI;

double smoothstep(double t)
{
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

const char * ikStatusName(rk::IkStatus s)
{
  switch (s) {
    case rk::IkStatus::Ok: return "Ok";
    case rk::IkStatus::AnkleOutOfRange: return "AnkleOutOfRange（足首が膝軸方向に近すぎる）";
    case rk::IkStatus::KneeOutOfRange: return "KneeOutOfRange（脚長に対して遠すぎる/近すぎる）";
    case rk::IkStatus::NoBranch: return "NoBranch（股中心に近すぎる）";
  }
  return "?";
}

const char * servoStatusName(rk::LegServoStatus s)
{
  switch (s) {
    case rk::LegServoStatus::Ok: return "Ok";
    case rk::LegServoStatus::KneeUnreachable: return "KneeUnreachable（膝 4 節が閉じない）";
    case rk::LegServoStatus::KneeDegenerate: return "KneeDegenerate";
    case rk::LegServoStatus::KneeDeadPoint: return "KneeDeadPoint（膝が死点）";
    case rk::LegServoStatus::AnkleUnreachable: return "AnkleUnreachable（足首のロッドが届かない）";
    case rk::LegServoStatus::AnkleDegenerate: return "AnkleDegenerate";
    case rk::LegServoStatus::AnkleNotConverged: return "AnkleNotConverged";
    case rk::LegServoStatus::AnkleSingular: return "AnkleSingular（足首が特異姿勢）";
    case rk::LegServoStatus::AnkleClamped: return "AnkleClamped（足首をエンベロープに丸めた）";
  }
  return "?";
}

// 関節の名前（関節角の列）と、それを動かすサーボの名前（サーボ角・count の列）。
// 股 3 軸は直結なので同じだが、膝は 4 節リンクのクランク、足首は 2 本のクランクを
// 2 つの関節角 (θ5, θ6) から解くので、行ごとに 1 対 1 ではない。
const char * kJointLabel[rk::kNumJoints] = {
  "θ1 股ピッチ", "θ2 股ロール", "θ3 股ヨー", "θ4 膝", "θ5 足首ピッチ", "θ6 足首ロール"};
const char * kServoLabel[rk::kNumJoints] = {
  "股ピッチ", "股ロール", "股ヨー", "膝クランク", "足首鎖0(短)", "足首鎖1(長)"};

// --- 指令側の計算結果 ----------------------------------------------------------
struct Target
{
  rm::FootPose foot;                     //!< 目標（Σ_B, rad）
  rm::LegSolve solve;                    //!< 丸めなしで解いた結果（指令に使う）
  double theta[rk::kNumJoints]{};        //!< 関節角 [rad]
  double servo[rk::kNumJoints]{};        //!< 絶対サーボ角 [rad]
  int count[rk::kNumJoints]{};           //!< 生カウント
  bool clamped[rk::kNumJoints]{};        //!< servo_limits の窓で丸められた
  bool jointOk{true};                    //!< JOINT_LIMIT の内側か（表示だけ）
  bool haveTheta{false};                 //!< 関節角の列が埋まっているか
  bool haveServo{false};                 //!< サーボ角・count の列が埋まっているか
  bool nearest{false};                   //!< 埋まっているのが「最寄り姿勢」（参考値）
  rm::ReachLevel level{rm::ReachLevel::None};
  bool ok{false};                        //!< 動かしてよいか
};

Target solve(const rm::ServoMap & map, int side, const rm::FootPose & foot)
{
  Target t;
  t.foot = foot;
  const rk::LegServoParams & prm = map.leg_params(side);

  // ★丸めない。解ければそのまま指令に使い、解けなければ理由を出して動かさない。
  t.solve = rm::servoFromFootPose(prm, foot, t.servo, t.theta);
  t.haveTheta = (t.solve.ik_status == rk::IkStatus::Ok);
  t.haveServo = t.solve.ok();
  t.level = rm::reachLevel(prm, foot);

  // IK が解けなかったときだけ、**表示のために**最寄り姿勢を出す（指令には使わない）。
  // 「どのくらい外なのか」が見えないと可動域の詰め方が分からないので。
  if (!t.haveTheta && t.solve.ik_status != rk::IkStatus::NoBranch) {
    if (rk::ik(prm.leg, foot.p, rm::matFromRpy(foot.rpy), t.theta, /*clamp=*/true) !=
      rk::IkStatus::NoBranch)
    {
      const rk::AnkleClampResult ac =
        rk::ankleClampJoints(t.theta[rk::ANKLE_PITCH], t.theta[rk::ANKLE_ROLL]);
      t.theta[rk::ANKLE_PITCH] = ac.th5;
      t.theta[rk::ANKLE_ROLL] = ac.th6;
      t.haveTheta = true;
      t.nearest = true;
      t.haveServo =
        (rk::legServoFromJoints(prm, t.theta, t.servo) == rk::LegServoStatus::Ok);
    }
  }

  // 関節リミット（膝は曲げ量で見る）。★2026-09-15: 動かす条件からは外した。
  // JOINT_LIMIT は幾何の限界ではなく方針の箱なので、ここで止めるのはやめて表示だけ。
  if (t.haveTheta) {
    const double bend = rk::kneeBendFromLegAngle(prm.leg, t.theta[rk::KNEE]);
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
      const double v = (j == rk::KNEE ? bend : t.theta[j]) * kDeg;
      if (v < rk::config::JOINT_LIMIT_LO_DEG[j] - 1e-9 ||
        v > rk::config::JOINT_LIMIT_HI_DEG[j] + 1e-9)
      {
        t.jointOk = false;
      }
    }
  }
  bool anyClamp = false;
  if (t.haveServo) {
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
      t.count[j] = map.leg_count_from_servo(side, j, t.servo[j], &t.clamped[j]);
      anyClamp = anyClamp || t.clamped[j];
    }
  }
  // 動かす条件は「解けた」＋「生カウントが servo_limits の窓の中」だけ。
  // エンベロープ・関節リミット・ReachLevel では止めない（解けるなら動かす）。
  t.ok = t.solve.ok() && !anyClamp;
  return t;
}

void printTarget(const rm::ServoMap & map, int side, const Target & t)
{
  const rk::LegServoParams & prm = map.leg_params(side);
  std::printf("指令  p = (%.1f, %.1f, %.1f) mm   rpy = (%.1f, %.1f, %.1f) deg\n",
    t.foot.p.x, t.foot.p.y, t.foot.p.z,
    t.foot.rpy[0] * kDeg, t.foot.rpy[1] * kDeg, t.foot.rpy[2] * kDeg);
  std::printf("  IK %s / 機構 %s / 判定 %s%s\n",
    ikStatusName(t.solve.ik_status), servoStatusName(t.solve.servo_status),
    rm::reachLevelName(t.level),
    t.solve.ankle_outside_envelope ? "  ★足首がエンベロープの外（丸めてはいない）" : "");
  if (!t.haveTheta) {return;}
  if (t.nearest) {
    std::printf("  ※ 以下は参考値（解けないので**最寄り姿勢**を出している。指令には使わない）\n");
  }
  const double bend = rk::kneeBendFromLegAngle(prm.leg, t.theta[rk::KNEE]);
  std::printf("  %-14s %8s   %-6s %-12s %8s %8s %6s\n",
    "関節", "関節角", "サーボ", "", "サーボ角", "Tポーズ", "count");
  for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
    std::printf("  %-14s %+8.2f   ID%-4d %-12s ",
      kJointLabel[j], t.theta[j] * kDeg, rm::kLegServoId[j], kServoLabel[j]);
    if (t.haveServo) {
      std::printf("%+8.2f %+8.2f %6d%s\n",
        t.servo[j] * kDeg, map.leg_tpose_deg_from_servo(side, j, t.servo[j]), t.count[j],
        t.clamped[j] ? "  ★servo_limits の窓の外" : "");
    } else {
      std::printf("%8s %8s %6s\n", "-", "-", "-");
    }
  }
  std::printf("  （足首の 2 つのサーボは θ5・θ6 の両方から決まる。行の対応は並びだけ）\n");
  std::printf("  膝の曲げ量 %.2f deg\n", bend * kDeg);
  if (!t.jointOk) {
    std::printf("  ★関節リミット (leg_config JOINT_LIMIT) の外（止めてはいない）\n");
  }
  if (!t.ok) {std::printf("  → この目標には動かさない\n");}
}

/// 実測カウント → サーボ角 → 関節角 → 順変換 → 足裏の姿勢。
bool printMeasured(
  const rm::ServoMap & map, int side, const std::vector<ServoState> & st,
  const Target * cmd, double & th6Seed, rm::FootPose * outFoot = nullptr)
{
  const rk::LegServoParams & prm = map.leg_params(side);
  std::printf("実測\n");
  double servo[rk::kNumJoints]{};
  bool all = true;
  for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
    if (j >= st.size() || !st[j].valid) {
      std::printf("  ID%-4d 応答なし\n", rm::kLegServoId[j]);
      all = false;
      continue;
    }
    servo[j] = map.leg_servo_from_count(side, j, st[j].pos);
    std::printf("  ID%-4d %-12s count %4d  Tポーズ基準 %+8.2f deg  %.1fV %d℃%s\n",
      rm::kLegServoId[j], kServoLabel[j], st[j].pos,
      map.leg_tpose_deg_from_count(side, j, st[j].pos), st[j].volt, st[j].temp,
      st[j].err ? ("  err: " + feetech_servo::err_str(st[j].err)).c_str() : "");
  }
  if (!all) {return false;}
  rm::FootPose foot;
  double theta[rk::kNumJoints]{};
  const rk::LegServoStatus fst = rm::footPoseFromServo(prm, servo, foot, theta, th6Seed);
  std::printf("  順変換 %s   関節角 [", servoStatusName(fst));
  for (std::size_t j = 0; j < rk::kNumJoints; ++j) {std::printf("%s%+.1f", j ? ", " : "", theta[j] * kDeg);}
  std::printf("] deg\n");
  std::printf("  足裏  p = (%.1f, %.1f, %.1f) mm   rpy = (%.1f, %.1f, %.1f) deg\n",
    foot.p.x, foot.p.y, foot.p.z, foot.rpy[0] * kDeg, foot.rpy[1] * kDeg, foot.rpy[2] * kDeg);
  if (cmd) {
    std::printf("  指令との差  p (%+.1f, %+.1f, %+.1f) mm   rpy (%+.1f, %+.1f, %+.1f) deg\n",
      foot.p.x - cmd->foot.p.x, foot.p.y - cmd->foot.p.y, foot.p.z - cmd->foot.p.z,
      (foot.rpy[0] - cmd->foot.rpy[0]) * kDeg, (foot.rpy[1] - cmd->foot.rpy[1]) * kDeg,
      (foot.rpy[2] - cmd->foot.rpy[2]) * kDeg);
  }
  if (outFoot) {*outFoot = foot;}
  return true;
}

/// 数回リトライして脚 6 軸の現在位置を読む（低電圧で応答が欠ける実機の癖）。
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

/// 始点 → 目標を duration 秒で補間して 6 軸を 1 パケットで送る。★ここが唯一の位置指令。
void ramp(
  FeetechBus & bus, const std::vector<uint8_t> & ids, const std::vector<int> & start,
  const std::vector<int> & target, double duration_s, double rate_hz, int speed, int acc)
{
  const auto period = std::chrono::duration_cast<Clock::duration>(
    std::chrono::duration<double>(1.0 / rate_hz));
  const size_t n = ids.size();
  std::vector<int16_t> cmd(n, 0);
  const std::vector<uint16_t> speeds(n, static_cast<uint16_t>(speed));
  const std::vector<uint8_t> accs(n, static_cast<uint8_t>(acc));
  const auto t0 = Clock::now();
  auto next = t0;
  while (!g_stop) {
    const double t = std::chrono::duration<double>(Clock::now() - t0).count();
    if (t > duration_s) {break;}
    const double s = smoothstep(t / duration_s);
    for (size_t i = 0; i < n; ++i) {
      cmd[i] = static_cast<int16_t>(std::lround(start[i] + (target[i] - start[i]) * s));
    }
    bus.sync_write_position(ids, cmd, speeds, accs);
    next += period;
    std::this_thread::sleep_until(next);
  }
  if (!g_stop) {
    for (size_t i = 0; i < n; ++i) {cmd[i] = static_cast<int16_t>(target[i]);}
    bus.sync_write_position(ids, cmd, speeds, accs);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));   // 整定待ち
  }
}

void usage()
{
  std::printf(
    "使い方: motion_leg_goto --leg L|R [--p X Y Z] [--rpy R P Y] [--rel]\n"
    "                        [--move] [--repl] [--off] [--yes]\n"
    "                        [--duration S] [--rate HZ] [--speed N] [--acc N] [--torque N]\n"
    "                        [--home FILE] [--limits FILE] [--right PORT] [--left PORT]\n"
    "  --leg      どちらの脚か（必須）\n"
    "  --p        足裏中心の目標 [mm]。機体座標 Σ_B（x 前 / y 左 / z 上、原点は股 3 軸の高さ）\n"
    "             股中心は右 (0, -89.3, 0) / 左 (0, +89.3, 0)。立位の例: 右 0 -89.3 -261\n"
    "  --rpy      足裏の姿勢 roll pitch yaw [deg]（既定 0 0 0 = 水平）\n"
    "  --rel      --p の x, y を股中心からの相対で受ける（z はそのまま。0 0 -261 で真下）\n"
    "  --move     ★実機を動かす。無ければ計算と現在値の表示だけ（何も書かない）\n"
    "  --repl     標準入力から 1 行 \"x y z roll pitch yaw\" を読んで順に動かす（--move が要る）\n"
    "             rpy は省略可（0 0 0）。空行で現在値を読み直す。q / Ctrl-D で終了\n"
    "  --off      その脚 6 軸のトルクを切る（★脚が抜ける）。--move と一緒なら動かしたあとに切り、\n"
    "             単独でも使える\n"
    "  --yes      開始前の 3 秒カウントダウンを省略\n"
    "  --duration 到達までの秒数（既定 3）    --rate 送信周波数 Hz（既定 50）\n"
    "  --speed    step/s（既定 300。0 は動かない）  --acc 加速度（既定 20）\n"
    "  --torque   HLS 系の目標電流 reg44 0-1000（既定 1000。0 だと駆動しない）\n"
    "  --home / --limits   servo_home.yaml / servo_limits.yaml（既定 share/feetech_servo/config/）\n"
    "\n"
    "★既定は読むだけ。トルク ON と位置指令は --move を付けたときだけ出る。\n"
    "★★脚 6 軸がまとめて動く。必ず機体を吊るか、脚が空中にある姿勢で使うこと。\n");
}

/// "x y z [roll pitch yaw]" を 1 行読む。
bool parsePose(const std::string & line, rm::FootPose & f)
{
  std::istringstream ss(line);
  double v[6] = {0, 0, 0, 0, 0, 0};
  int n = 0;
  while (n < 6 && (ss >> v[n])) {++n;}
  if (n != 3 && n != 6) {return false;}
  f.p = rk::Vec3{v[0], v[1], v[2]};
  for (int k = 0; k < 3; ++k) {f.rpy[k] = v[3 + k] / kDeg;}
  return true;
}
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, on_sigint);

  std::string leg, homePath, limitsPath;
  std::string portR = "/dev/feetech_right", portL = "/dev/feetech_left";
  bool haveP = false, rel = false, move = false, repl = false, off = false, yes = false;
  rm::FootPose req;
  double duration = 3.0, rate = 50.0;
  int speed = 300, acc = 20, goalTorque = 1000;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&]() -> const char * {
        if (i + 1 >= argc) {usage(); std::exit(2);}
        return argv[++i];
      };
    if (a == "--leg") {leg = need();} else if (a == "--p") {
      req.p.x = std::atof(need()); req.p.y = std::atof(need()); req.p.z = std::atof(need());
      haveP = true;
    } else if (a == "--rpy") {
      for (int k = 0; k < 3; ++k) {req.rpy[k] = std::atof(need()) / kDeg;}
    } else if (a == "--rel") {rel = true;} else if (a == "--move") {move = true;} else if (
      a == "--repl") {repl = true;} else if (a == "--off") {off = true;} else if (
      a == "--yes" || a == "-y") {yes = true;} else if (
      a == "--duration") {duration = std::atof(need());} else if (
      a == "--rate") {rate = std::atof(need());} else if (
      a == "--speed") {speed = std::atoi(need());} else if (
      a == "--acc") {acc = std::atoi(need());} else if (
      a == "--torque") {goalTorque = std::atoi(need());} else if (
      a == "--home") {homePath = need();} else if (a == "--limits") {limitsPath = need();} else if (
      a == "--right") {portR = need();} else if (a == "--left") {portL = need();} else {
      usage();
      return (a == "--help" || a == "-h") ? 0 : 2;
    }
  }
  if (leg != "L" && leg != "l" && leg != "R" && leg != "r") {
    std::fprintf(stderr, "--leg L か --leg R を付ける\n");
    usage();
    return 2;
  }
  if (repl && !move) {
    std::fprintf(stderr, "--repl は --move と一緒に使う（動かさない対話には意味が無い）\n");
    return 2;
  }
  if (!haveP && !repl && !off) {
    std::fprintf(stderr, "何をするか指定が無い。--p X Y Z か --repl、トルクを切るだけなら --off\n");
    usage();
    return 2;
  }
  if (speed == 0) {
    std::fprintf(stderr, "--speed 0 は実機では動かない。300 程度を指定すること\n");
    return 2;
  }

  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory("feetech_servo");
  } catch (const std::exception & e) {
    std::fprintf(stderr, "feetech_servo の share が見つからない: %s\n", e.what());
    return 2;
  }
  if (homePath.empty()) {homePath = share + "/config/servo_home.yaml";}
  if (limitsPath.empty()) {limitsPath = share + "/config/servo_limits.yaml";}

  rm::ServoMap map;
  std::string err;
  if (!map.load(homePath, limitsPath, portR, portL, {"R8", "L9", "R10"}, err)) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 2;
  }
  const bool right = (leg == "R" || leg == "r");
  const int side = right ? rm::kRight : rm::kLeft;
  const std::string port = right ? portR : portL;
  const rk::LegServoParams & prm = map.leg_params(side);
  const rk::Vec3 hip = prm.leg.p0;

  const auto & axes = map.bus(side);
  if (axes.num_leg != static_cast<int>(rk::kNumJoints)) {
    std::fprintf(stderr, "%s に脚 6 軸が揃っていない（servo_home.yaml を確認）\n", port.c_str());
    return 2;
  }
  const std::vector<uint8_t> ids(axes.ids.begin(), axes.ids.begin() + rk::kNumJoints);

  std::printf("%s脚  %s  ID %d,%d,%d,%d,%d,%d   股中心 (%.1f, %.1f, %.1f) mm\n",
    right ? "右" : "左", port.c_str(), ids[0], ids[1], ids[2], ids[3], ids[4], ids[5],
    hip.x, hip.y, hip.z);
  std::printf("座標は Σ_B（x 前 / y 左 / z 上）。rpy は足裏の姿勢 [deg]、pitch - でつま先上げ\n\n");

  auto absolute = [&](rm::FootPose f) {
      if (rel) {f.p.x += hip.x; f.p.y += hip.y;}
      return f;
    };

  // --- 1) まず計算だけ（バスが無くてもここまでは出る）---
  Target tgt;
  if (haveP) {
    tgt = solve(map, side, absolute(req));
    printTarget(map, side, tgt);
    std::printf("\n");
  }

  // --- 2) バスを開いて現在値を読む（書かない）---
  FeetechBus bus(port, 1000000, 0, 20, feetech_servo::Family::kHls);
  if (!bus.open()) {
    std::fprintf(stderr, "%s を開けない（motion が掴んでいないか、udev ルールを確認）\n",
      port.c_str());
    return (move || off) ? 2 : 0;   // 計算だけが目的なら失敗にしない
  }
  bus.set_goal_torque(static_cast<uint16_t>(goalTorque));
  std::vector<ServoState> st;
  double th6Seed = 0.0;
  const bool readOk = readNow(bus, ids, st);
  if (!readOk) {
    std::fprintf(stderr, "脚 6 軸の現在位置が揃わない。電源・電圧を確認\n");
    if (move) {return 2;}
  }
  printMeasured(map, side, st, nullptr, th6Seed);

  if (!move) {
    if (off) {
      int n = 0;
      for (uint8_t id : ids) {n += bus.enable_torque(id, false) ? 1 : 0;}
      std::printf("\nトルク OFF（%d / 6 軸）★脚が抜ける\n", n);
      return n == 6 ? 0 : 2;
    }
    std::printf("\n（--move が無いので何も書かずに終了）\n");
    return 0;
  }

  // --- 3) ★ここから書く。目標＝現在位置を書いてからトルク ON（投入時の飛び出し防止）---
  {
    std::vector<int16_t> hold(ids.size());
    for (size_t i = 0; i < ids.size(); ++i) {hold[i] = static_cast<int16_t>(st[i].pos);}
    bus.sync_write_position(ids, hold);
    int on = 0;
    for (uint8_t id : ids) {on += bus.enable_torque(id, true) ? 1 : 0;}
    std::printf("\nトルク ON: %d / 6 軸（★脚が動く。機体を吊っているか確認）\n", on);
    if (on != 6) {std::fprintf(stderr, "トルクが入らない軸がある。中止\n"); return 2;}
  }

  auto moveTo = [&](const Target & t) -> bool {
      if (!t.ok) {
        std::printf("  → 目標が不正なので動かさない\n");
        return false;
      }
      std::vector<int> start(ids.size()), target(ids.size());
      for (size_t i = 0; i < ids.size(); ++i) {start[i] = st[i].pos; target[i] = t.count[i];}
      std::printf("  %.1f 秒で 6 軸同時に移動\n", duration);
      ramp(bus, ids, start, target, duration, rate, speed, acc);
      if (g_stop) {return false;}
      readNow(bus, ids, st);
      printMeasured(map, side, st, &t, th6Seed);
      return true;
    };

  if (haveP) {
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

  // --- 4) 対話モード: 1 行 "x y z [roll pitch yaw]" を読んで動かす ---
  if (repl && !g_stop) {
    std::printf("\n--- 対話モード: \"x y z [roll pitch yaw]\"（mm, deg）を 1 行ずつ。"
      "空行で読み直し、q で終了 ---\n");
    std::string line;
    while (!g_stop) {
      std::printf("> ");
      std::fflush(stdout);
      if (!std::getline(std::cin, line)) {break;}
      if (line == "q" || line == "quit" || line == "exit") {break;}
      rm::FootPose f;
      if (line.find_first_not_of(" \t") == std::string::npos) {
        readNow(bus, ids, st);
        printMeasured(map, side, st, nullptr, th6Seed);
        continue;
      }
      if (!parsePose(line, f)) {std::printf("  3 つか 6 つ要る: x y z [roll pitch yaw]\n"); continue;}
      const Target t = solve(map, side, absolute(f));
      printTarget(map, side, t);
      moveTo(t);
    }
  }

  if (off) {
    int n = 0;
    for (uint8_t id : ids) {n += bus.enable_torque(id, false) ? 1 : 0;}
    std::printf("\nトルク OFF（%d / 6 軸）★脚が抜ける\n", n);
  } else {
    std::printf("\n終了。トルクは入ったまま（切るなら --off、または feetech_shell の off）\n");
  }
  return g_stop ? 1 : 0;
}
