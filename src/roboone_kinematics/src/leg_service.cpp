// 脚 FK/IK を 1 行 1 リクエストの JSON サービスとして出す。
//   ros2 run roboone_kinematics leg_service
//
// 可視化 (roboone_motion/viz/serve_leg3d.py) から起動され、ブラウザに出る形は
// すべてここを通る。つまり画面に見えているのは leg_kinematics.hpp そのものの
// 出力で、可視化用に書き直した別実装ではない。
//
// 標準入力（1 行 1 コマンド。長さの単位は mm、角度は deg、座標は Σ_B）:
//   params                                 現在の寸法を返す
//   set <key> <value>                      寸法を差し替える（下の kKeys）
//   fk  <L|R> t1 t2 t3 t4 t5 t6            関節角 -> 関節位置
//   ik  <L|R> x y z                        足裏中心の目標 -> 関節角（足姿勢は水平）
//   ikpose <L|R> x y z roll pitch yaw      足姿勢も指定する版（Σ_B の RPY）
//   servo <L|R> d1 d2 d3 d4 d6 d5 [seed]   サーボ実測 -> 関節角 -> 関節位置
//   mech <0|1>                             機構層（膝 4 節・足首パラレル）を出すか
//
// fk / ik / ikpose / servo の応答には既定で "mech" が付く。中身は
//   servo   その姿勢のサーボ指令角 [deg]（Joint 順 J1..J6）
//   dservo  同じものを T ポーズ基準の差分で（ID 順 1,2,3,4,6,5。servo コマンドの入力形）
//   knee    4 節リンクの角と、O4 / O2 / A / B の 4 点（Σ_B）
//   ankle   足首の各鎖のクランク角と、クランク軸 O_i / クランク先端 K_i / ボール B_i
// 点はすべて knee_fourbar.hpp・ankle_parallel.hpp が計算した組み方そのもので、
// 描画用の近似ではない。フレーム数の多い一括計算では mech 0 で切れる。
//
// servo コマンドの入力は **T ポーズ（servo_home.yaml の原点）からの角度差 [deg]**。
// 並びは実機の ID 順 1,2,3,4,6,5（ID6 = 足首の鎖 0・短ロッド、ID5 = 鎖 1・長ロッド）。
// 膝だけは基準が θ2 = 0 ではなく「伸び切りのクランク角 θ2_ext」なので、その分を
// ここで足してから leg_servo.hpp に渡す。足首と股は原点がそのまま 0 に対応する。
// 標準出力は 1 行 1 JSON。エラーも JSON で返し、プロセスは落とさない。
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "roboone_kinematics/leg_kinematics.hpp"
#include "roboone_kinematics/leg_servo.hpp"

using namespace roboone_kinematics;

namespace
{

/// set で差し替えられる寸法。Σ_B の値で、右脚のものとして扱う。
struct Dims
{
  double l3 = config::L3, l4 = config::L4, l5 = config::L5, l6 = config::L6;
  double p3x = config::P3_X, p3y = config::P3_Y, p4y = config::P4_Y;
  double hipx = config::HIP_X, hipy = config::HIP_Y, hipz = config::HIP_Z;
  double p6x = config::P6_X, p6y = config::P6_Y, p6z = config::P6_Z;
  int knee = config::KNEE_FORWARD;
};

Dims g_dims;

/// Σ_B の寸法から左右脚の LegParams（Σ_S 成分）を組み立てる。
/// makeLegParams() と同じ手順を、config 定数のかわりに Dims から行う。
LegParams makeParams(Side side)
{
  const double lat = (side == Side::RIGHT) ? 1.0 : -1.0;
  LegParams prm = makeLegParams(side);       // AXIS_FLIP はここから引き継ぐ
  prm.l3 = g_dims.l3;
  prm.l4 = g_dims.l4;
  prm.l5 = g_dims.l5;
  prm.l6 = g_dims.l6;
  prm.a3 = -g_dims.p3y * lat;
  prm.a4 = -g_dims.p4y * lat;
  prm.b = g_dims.p3x;
  prm.p0 = {-g_dims.hipy * lat, g_dims.hipx, g_dims.hipz};
  prm.p6 = {-g_dims.p6y * lat, g_dims.p6x, g_dims.p6z};
  prm.sigma = -g_dims.knee;
  prm.finalize();
  return prm;
}

/// 脚 + 膝 + 足首の変換層。寸法の上書き（set）は脚の部分にだけ効かせる。
LegServoParams makeServoParams(Side side)
{
  LegServoParams prm = makeLegServoParams(side);
  prm.leg = makeParams(side);
  return prm;
}

/// 伸び切り（曲げ量 0）に対応するクランク角 θ2_ext。servo コマンドの膝の基準。
double kneeCrankAtExtension(const KneeParams & knee, bool & ok)
{
  KneePose pose;
  ok = kneeIk(knee, kneeRockerFromBend(knee, 0.0), pose) == KneeStatus::Ok;
  return pose.theta2;
}

const char * legServoStatusName(LegServoStatus s)
{
  switch (s) {
    case LegServoStatus::Ok: return "ok";
    case LegServoStatus::KneeUnreachable: return "knee_unreachable";
    case LegServoStatus::KneeDegenerate: return "knee_degenerate";
    case LegServoStatus::KneeDeadPoint: return "knee_dead_point";
    case LegServoStatus::AnkleUnreachable: return "ankle_unreachable";
    case LegServoStatus::AnkleDegenerate: return "ankle_degenerate";
    case LegServoStatus::AnkleNotConverged: return "ankle_not_converged";
    default: return "ankle_singular";
  }
}

const char * statusName(IkStatus s)
{
  switch (s) {
    case IkStatus::Ok: return "ok";
    case IkStatus::AnkleOutOfRange: return "ankle_out_of_range";
    case IkStatus::KneeOutOfRange: return "knee_out_of_range";
    default: return "no_branch";
  }
}

void printVec(const Vec3 & v)
{
  std::printf("[%.4f,%.4f,%.4f]", v.x, v.y, v.z);
}

/// 関節位置 5 点と足姿勢を JSON で吐く。
void printPose(const LegParams & prm, const double th[kNumJoints])
{
  Vec3 o[5];
  jointOrigins(prm, th, o);
  Vec3 p;
  Mat3 R;
  fk(prm, th, p, R);

  std::printf("\"theta\":[");
  for (std::size_t k = 0; k < kNumJoints; ++k) {
    std::printf("%s%.6f", k ? "," : "", th[k] * 180.0 / M_PI);
  }
  std::printf("],\"origins\":[");
  for (int k = 0; k < 5; ++k) {
    if (k) {std::printf(",");}
    printVec(o[k]);
  }
  std::printf("],\"foot\":");
  printVec(p);
  std::printf(",\"R\":[");
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {std::printf("%s%.6f", (r || c) ? "," : "", R(r, c));}
  }
  std::printf("]");
}

/// 機構層（膝 4 節・足首パラレル）を出すか。walk の一括計算では切れるようにする。
bool g_mech = true;

const char * kneeStatusName(KneeStatus s)
{
  switch (s) {
    case KneeStatus::Ok: return "ok";
    case KneeStatus::Unreachable: return "unreachable";
    case KneeStatus::Degenerate: return "degenerate";
    default: return "dead_point";
  }
}

const char * ankleIkStatusName(AnkleIkStatus s)
{
  switch (s) {
    case AnkleIkStatus::Ok: return "ok";
    case AnkleIkStatus::Unreachable: return "unreachable";
    default: return "degenerate";
  }
}

Vec3 unitVec(const Vec3 & v)
{
  const double n = v.norm();
  return (n > 1e-9) ? v * (1.0 / n) : v;
}

/// 4 節リンクを矢状面へ置くための基底（原点 = 膝 o4）。
///
/// 膝軸は Σ_B では大腿フレーム Σ_3 の y。大腿は膝軸方向にもオフセット（a3 + a4）を
/// 持つので、**膝軸成分を落としてから**正規化しないと基底が直交せず、埋め込んだ
/// リンクの長さが縮む。規約は serve_knee3d.py の _plane_basis と同じ。
struct SagittalPlane
{
  Vec3 o{};    //!< 膝 o4
  Vec3 e1{};   //!< 膝 -> 股。4 節リンクの +x（地節 O4 -> O2 の向き）
  Vec3 e2{};   //!< 膝軸 x e1
};

SagittalPlane sagittalPlane(const LegParams & leg, const double th[kNumJoints])
{
  Vec3 o[5];
  Mat3 F[4];
  jointOrigins(leg, th, o);
  jointFrames(leg, th, F);
  const Vec3 axis = F[0] * Vec3{0.0, 1.0, 0.0};        // 膝の回転軸（Σ_B）
  const Vec3 v = o[0] - o[1];                          // 膝 -> 股
  const Vec3 e1 = unitVec(v - axis * v.dot(axis));
  return SagittalPlane{o[1], e1, axis.cross(e1)};
}

/// 矢状面で測った下腿（膝 -> 足首）の向き [rad]。
double shankAngle(const LegServoParams & sp, double bend)
{
  double th[kNumJoints]{};
  th[KNEE] = legAngleFromKneeBend(sp.leg, bend);
  const SagittalPlane pl = sagittalPlane(sp.leg, th);
  Vec3 o[5];
  jointOrigins(sp.leg, th, o);
  const Vec3 v = o[2] - o[1];
  return std::atan2(v.dot(pl.e2), v.dot(pl.e1));
}

/// e2 の符号。ロッカーは下腿に固定されているので、両者は必ず同じ向きに回る。
/// 取り付け向きは寸法からは決まらないが**回る向き**は決まるので、ここで揃える。
/// 揃えないと、膝を曲げたときにリンクだけ逆に動いて見える。
double planeSign(const LegServoParams & sp)
{
  const double d30 = 30.0 * M_PI / 180.0;
  const double a0 = shankAngle(sp, 0.0);
  const double a1 = shankAngle(sp, d30);
  const double dsh = std::atan2(std::sin(a1 - a0), std::cos(a1 - a0));
  return (dsh * sp.knee.sigmaJoint * d30 > 0.0) ? 1.0 : -1.0;
}

/// 関節角 -> サーボ指令 + 膝 4 節リンク・足首パラレルの点（Σ_B）を JSON で吐く。
///
/// 通しているのは leg_servo.hpp / knee_fourbar.hpp / ankle_parallel.hpp そのもので、
/// 可視化のために書き直した式は無い。点は「その姿勢のときに機構がどう組まれているか」で、
/// 描画のために近似したものではない。
void printMech(const LegServoParams & sp, const double th[kNumJoints])
{
  const double d2 = 180.0 / M_PI;
  double servo[kNumJoints]{};
  const LegServoStatus lst = legServoFromJoints(sp, th, servo);

  bool kok = true;
  const double t2ext = kneeCrankAtExtension(sp.knee, kok);
  // T ポーズ（servo_home.yaml の原点）からの差分。並びは実機の ID 順 1,2,3,4,6,5 で、
  // leg_service の servo コマンドがそのまま受け取れる形にしてある。
  const double dsv[kNumJoints] = {
    servo[HIP_PITCH], servo[HIP_ROLL], servo[HIP_YAW],
    kok ? servo[KNEE] - kneeServoFromCrank(sp.knee, t2ext) : 0.0,
    servo[ANKLE_PITCH] - sp.ankle.servoHome[0],
    servo[ANKLE_ROLL] - sp.ankle.servoHome[1]};

  std::printf(",\"mech\":{\"status\":\"%s\",\"servo\":[", legServoStatusName(lst));
  for (std::size_t k = 0; k < kNumJoints; ++k) {
    std::printf("%s%.4f", k ? "," : "", servo[k] * d2);
  }
  std::printf("],\"dservo\":[");
  for (std::size_t k = 0; k < kNumJoints; ++k) {
    std::printf("%s%.4f", k ? "," : "", dsv[k] * d2);
  }

  // ---- 膝 4 節リンク ----
  const double bend = kneeBendFromLegAngle(sp.leg, th[KNEE]);
  KneePose kp;
  const KneeStatus kst = kneeIk(sp.knee, kneeRockerFromBend(sp.knee, bend), kp);
  SagittalPlane pl = sagittalPlane(sp.leg, th);
  pl.e2 = pl.e2 * planeSign(sp);
  const Vec2 pts2[4] = {{0.0, 0.0}, {sp.knee.r1, 0.0}, kp.a, kp.b};
  std::printf(
    "],\"knee\":{\"status\":\"%s\",\"bend\":%.4f,\"theta2\":%.4f,\"theta3\":%.4f,"
    "\"theta4\":%.4f,\"gamma\":%.4f,\"r\":[%g,%g,%g,%g],\"pts\":[",
    kneeStatusName(kst), bend * d2, kp.theta2 * d2, kp.theta3 * d2, kp.theta4 * d2,
    kneeTransmissionAngle(kp) * d2, sp.knee.r1, sp.knee.r2, sp.knee.r3, sp.knee.r4);
  for (int k = 0; k < 4; ++k) {
    if (k) {std::printf(",");}
    printVec(pl.o + pl.e1 * pts2[k].x + pl.e2 * pts2[k].y);
  }
  std::printf("]}");

  // ---- 足首パラレルリンク ----
  // 指令側と同じ順（エンベロープで丸めてから逆変換）。丸めた分は clamped で出す。
  const AnkleClampResult env = ankleClampJoints(th[ANKLE_PITCH], th[ANKLE_ROLL]);
  const AnkleIkResult ares = ankleIk(sp.ankle, env.th5, env.th6);
  Vec3 o[5];
  Mat3 F[4];
  jointOrigins(sp.leg, th, o);
  jointFrames(sp.leg, th, F);
  const Mat3 & Rs = F[1];                       // Σ_s = 膝の後の Σ_4 と同じ向き
  std::printf(
    ",\"ankle\":{\"status\":\"%s\",\"clamped\":%d,\"th5\":%.4f,\"th6\":%.4f,\"u\":",
    ankleIkStatusName(ares.status), env.clamped ? 1 : 0, env.th5 * d2, env.th6 * d2);
  printVec(Rs * Vec3{1.0, 0.0, 0.0});           // クランク円の基準方向 û（q = 0）
  std::printf(",\"v\":");
  printVec(Rs * Vec3{0.0, 0.0, 1.0});           // 同 v̂（q = 90°）
  std::printf(",\"chains\":[");
  for (int i = 0; i < kAnkleChains; ++i) {
    std::printf("%s{\"q\":%.4f,\"qlim\":[%.4f,%.4f],\"margin\":%.4f,\"r\":%g,"
      "\"rod\":%g,\"O\":", i ? "," : "", ares.q[i] * d2,
      sp.ankle.qMin[i] * d2, sp.ankle.qMax[i] * d2, ares.margin[i],
      sp.ankle.r[i], sp.ankle.rod[i]);
    printVec(o[2] + Rs * sp.ankle.c[i]);                                  // クランク軸 O_i
    std::printf(",\"K\":");
    printVec(o[2] + Rs * ankleCrank(sp.ankle, i, ares.q[i]));             // クランク先端
    std::printf(",\"B\":");
    printVec(o[2] + Rs * ankleBall(sp.ankle, i, env.th5, env.th6));       // 足側ボール
    std::printf("}");
  }
  std::printf("]}}");
}

bool setKey(const std::string & k, double v)
{
  if (k == "l3") {g_dims.l3 = v;} else if (k == "l4") {g_dims.l4 = v;}
  else if (k == "l5") {g_dims.l5 = v;} else if (k == "l6") {g_dims.l6 = v;}
  else if (k == "p3x") {g_dims.p3x = v;} else if (k == "p3y") {g_dims.p3y = v;}
  else if (k == "p4y") {g_dims.p4y = v;} else if (k == "hipx") {g_dims.hipx = v;}
  else if (k == "hipy") {g_dims.hipy = v;} else if (k == "hipz") {g_dims.hipz = v;}
  else if (k == "p6x") {g_dims.p6x = v;} else if (k == "p6y") {g_dims.p6y = v;}
  else if (k == "p6z") {g_dims.p6z = v;}
  else if (k == "knee") {g_dims.knee = (v < 0.0) ? -1 : +1;}
  else {return false;}
  return true;
}

}  // namespace

int main()
{
  std::ios::sync_with_stdio(false);
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string cmd;
    if (!(is >> cmd)) {continue;}

    if (cmd == "params") {
      const LegParams r = makeParams(Side::RIGHT);
      const LegParams l = makeParams(Side::LEFT);
      std::printf(
        "{\"ok\":1,\"l3\":%g,\"l4\":%g,\"l5\":%g,\"l6\":%g,\"p3x\":%g,\"p3y\":%g,"
        "\"p4y\":%g,\"hipx\":%g,\"hipy\":%g,\"hipz\":%g,\"knee\":%d,\"reach\":%g,"
        "\"frame\":\"x=front,y=left,z=up\",",
        g_dims.l3, g_dims.l4, g_dims.l5, g_dims.l6, g_dims.p3x, g_dims.p3y,
        g_dims.p4y, g_dims.hipx, g_dims.hipy, g_dims.hipz, g_dims.knee,
        r.l3 + r.l4 + r.l5 + r.l6);
      // AXIS_FLIP の実効値（1 = サーボが Σ_B の正方向と逆に回る）
      for (const auto & pr : {std::pair<const char *, const LegParams *>{"flipR", &r},
                              std::pair<const char *, const LegParams *>{"flipL", &l}})
      {
        std::printf("%s\"%s\":[", pr.first[4] == 'L' ? "," : "", pr.first);
        for (std::size_t k = 0; k < kNumJoints; ++k) {
          std::printf("%s%d", k ? "," : "", pr.second->sign[k] < 0 ? 1 : 0);
        }
        std::printf("]");
      }
      // 可動域（画面で「縁に来た」を出すため）。関節は leg_config、クランクと
      // 順変換の窓は ankle_config の値。
      const LegServoParams sp = makeServoParams(Side::RIGHT);
      const double d2 = 180.0 / M_PI;
      std::printf(",\"limit\":[");
      for (std::size_t k = 0; k < kNumJoints; ++k) {
        std::printf("%s[%g,%g]", k ? "," : "",
          config::JOINT_LIMIT_LO_DEG[k], config::JOINT_LIMIT_HI_DEG[k]);
      }
      std::printf("],\"qlim\":[");
      for (int i = 0; i < kAnkleChains; ++i) {
        std::printf("%s[%.3f,%.3f]", i ? "," : "",
          sp.ankle.qMin[i] * d2, sp.ankle.qMax[i] * d2);
      }
      std::printf("],\"th6win\":[%g,%g],\"th5lim\":%g,\"kneeR\":[%g,%g,%g,%g]",
        ankle_config::FK_WINDOW_DEG[0], ankle_config::FK_WINDOW_DEG[1],
        ankle_config::TH5_MECH_LIMIT_DEG,
        sp.knee.r1, sp.knee.r2, sp.knee.r3, sp.knee.r4);
      std::printf("}\n");
      std::fflush(stdout);
      continue;
    }

    if (cmd == "set") {
      std::string key;
      double v = 0.0;
      if (!(is >> key >> v) || !setKey(key, v)) {
        std::printf("{\"ok\":0,\"error\":\"bad set\"}\n");
      } else {
        std::printf("{\"ok\":1}\n");
      }
      std::fflush(stdout);
      continue;
    }

    if (cmd == "mech") {
      int v = 1;
      if (!(is >> v)) {
        std::printf("{\"ok\":0,\"error\":\"mech needs 0 or 1\"}\n");
      } else {
        g_mech = (v != 0);
        std::printf("{\"ok\":1,\"mech\":%d}\n", g_mech ? 1 : 0);
      }
      std::fflush(stdout);
      continue;
    }

    std::string sideStr;
    if (!(is >> sideStr)) {
      std::printf("{\"ok\":0,\"error\":\"no side\"}\n");
      std::fflush(stdout);
      continue;
    }
    const Side side = (sideStr == "L" || sideStr == "l") ? Side::LEFT : Side::RIGHT;
    const LegParams prm = makeParams(side);

    if (cmd == "fk") {
      double th[kNumJoints]{};
      bool ok = true;
      for (std::size_t k = 0; k < kNumJoints; ++k) {
        double d = 0.0;
        ok = ok && static_cast<bool>(is >> d);
        th[k] = d * M_PI / 180.0;
      }
      if (!ok) {
        std::printf("{\"ok\":0,\"error\":\"fk needs 6 angles\"}\n");
      } else {
        std::printf("{\"ok\":1,\"status\":\"ok\",");
        printPose(prm, th);
        if (g_mech) {printMech(makeServoParams(side), th);}
        std::printf("}\n");
      }
      std::fflush(stdout);
      continue;
    }

    if (cmd == "servo") {
      double d[kNumJoints]{};
      bool ok = true;
      for (std::size_t k = 0; k < kNumJoints; ++k) {
        double v = 0.0;
        ok = ok && static_cast<bool>(is >> v);
        d[k] = v * M_PI / 180.0;
      }
      double seed = 0.0;
      is >> seed;                                  // 省略可。前周期の θ6 [deg]
      if (!ok) {
        std::printf("{\"ok\":0,\"error\":\"servo needs 6 deltas\"}\n");
        std::fflush(stdout);
        continue;
      }
      const LegServoParams sp = makeServoParams(side);
      bool kok = true;
      const double t2ext = kneeCrankAtExtension(sp.knee, kok);
      if (!kok) {
        std::printf("{\"ok\":0,\"error\":\"knee extension pose unsolvable\"}\n");
        std::fflush(stdout);
        continue;
      }
      // T ポーズ基準の差分 -> 各変換が期待する絶対サーボ角
      double sv[kNumJoints];
      sv[HIP_PITCH] = d[0];
      sv[HIP_ROLL] = d[1];
      sv[HIP_YAW] = d[2];
      sv[KNEE] = kneeServoFromCrank(sp.knee, t2ext) + d[3];
      sv[ANKLE_PITCH] = sp.ankle.servoHome[0] + d[4];   // ID6 = 鎖 0（短ロッド）
      sv[ANKLE_ROLL] = sp.ankle.servoHome[1] + d[5];    // ID5 = 鎖 1（長ロッド）

      double th[kNumJoints]{};
      const LegServoStatus lst =
        legJointsFromServo(sp, sv, th, seed * M_PI / 180.0);

      // 表示用の中間量（同じ関数を通しているので二重実装にはならない）
      double bend = 0.0;
      kneeBendFromServo(sp.knee, sv[KNEE], bend);
      const double q0 = ankleCrankFromServo(sp.ankle, 0, sv[ANKLE_PITCH]);
      const double q1 = ankleCrankFromServo(sp.ankle, 1, sv[ANKLE_ROLL]);

      std::printf(
        "{\"ok\":%d,\"status\":\"%s\",\"bend\":%.4f,\"theta2\":%.4f,"
        "\"q\":[%.4f,%.4f],",
        lst == LegServoStatus::Ok ? 1 : 0, legServoStatusName(lst),
        bend * 180.0 / M_PI, kneeCrankFromServo(sp.knee, sv[KNEE]) * 180.0 / M_PI,
        q0 * 180.0 / M_PI, q1 * 180.0 / M_PI);
      printPose(sp.leg, th);
      if (g_mech) {printMech(sp, th);}
      std::printf("}\n");
      std::fflush(stdout);
      continue;
    }

    if (cmd == "ik" || cmd == "ikpose") {
      double x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0;
      bool ok = static_cast<bool>(is >> x >> y >> z);
      if (cmd == "ikpose") {ok = ok && static_cast<bool>(is >> roll >> pitch >> yaw);}
      if (!ok) {
        std::printf("{\"ok\":0,\"error\":\"ik needs x y z\"}\n");
        std::fflush(stdout);
        continue;
      }
      const double d2r = M_PI / 180.0;
      // Σ_B の RPY（z-y-x 順）で足姿勢を作る
      const Mat3 R = rotZ(yaw * d2r) * rotY(pitch * d2r) * rotX(roll * d2r);
      double th[kNumJoints]{};
      const IkStatus st = ik(prm, Vec3{x, y, z}, R, th, /*clamp=*/true);
      std::printf("{\"ok\":%d,\"status\":\"%s\",", st == IkStatus::Ok ? 1 : 0,
        statusName(st));
      if (st != IkStatus::NoBranch) {
        printPose(prm, th);
        if (g_mech) {printMech(makeServoParams(side), th);}
      } else {
        std::printf("\"theta\":null,\"origins\":null");
      }
      std::printf("}\n");
      std::fflush(stdout);
      continue;
    }

    std::printf("{\"ok\":0,\"error\":\"unknown command\"}\n");
    std::fflush(stdout);
  }
  return 0;
}
