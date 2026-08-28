// 「機体の 1 姿勢」を表す型と、それとサーボ角の間の変換。
//
// motion ノードとティーチツールが共通で使う。**IK / FK を呼ぶ場所をここ 1 か所に
// 絞る**ためのファイルで、幾何の式は 1 行も持たない（全部 roboone_kinematics）。
//
// ===========================================================================
// 姿勢の表し方
// ===========================================================================
// 足裏中心の位置姿勢 (p, R) を左右ぶんと、運動学を持たない腕の角度。
//
//   p    [mm]  機体座標 Σ_B（x 前・y 左・z 上、原点 = 骨盤）
//   R    RPY [rad]。R = rotZ(yaw) * rotY(pitch) * rotX(roll)
//              （leg_service の ikpose と同じ取り方。config には deg で書く）
//   腕   [deg] T ポーズ（servo_home.yaml の原点）からの差。変換なしで素通し
//
// 姿勢を 3x3 行列ではなく RPY で持つのは、モーション config を人が手で書き換える
// ため。回転行列は捕まえたポーズを貼るぶんには良いが、「つま先をあと 5 度下げる」が
// できない。
//
// ===========================================================================
// 単位が mm なのはなぜか
// ===========================================================================
// roboone_kinematics（leg_config.hpp / leg_service）が mm、roboone_walk_core が
// m で、境界はどこかに要る。**IK に近いほうへ寄せて mm にした**。歩行の出力を
// mm に直すのは motion ノードの 1 か所（walk → foot target）だけで済むが、
// 逆にすると IK・ティーチ・config の全部が m になって CAD 値と突き合わせづらい。
#ifndef ROBOONE_MOTION_NODE__BODY_POSE_HPP_
#define ROBOONE_MOTION_NODE__BODY_POSE_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

#include "roboone_kinematics/leg_servo.hpp"
#include "roboone_motion_node/servo_map.hpp"

namespace roboone_motion_node
{

/// 片足の足裏中心の目標。Σ_B、位置 mm・姿勢 rad。
struct FootPose
{
  rk::Vec3 p{0.0, 0.0, -200.0};
  double rpy[3]{0.0, 0.0, 0.0};      //!< roll, pitch, yaw [rad]
};

/// 機体の 1 姿勢。
struct BodyPose
{
  FootPose foot[kNumSide];           //!< [kRight], [kLeft]
  std::vector<double> arm;           //!< [deg] ServoMap::arms() と同順・同数
};

/// RPY [rad] -> 回転行列（rotZ * rotY * rotX）。
inline rk::Mat3 matFromRpy(const double rpy[3])
{
  return rk::rotZ(rpy[2]) * rk::rotY(rpy[1]) * rk::rotX(rpy[0]);
}

/// 回転行列 -> RPY [rad]。matFromRpy の逆。
///
/// pitch = ±90 deg（ジンバルロック）では roll と yaw が縮退する。足裏がそこまで
/// 傾く姿勢はこの機体では作らないので、縮退時は roll = 0 に寄せるだけにする。
inline void rpyFromMat(const rk::Mat3 & R, double rpy[3])
{
  const double sp = -R(2, 0);
  const double pitch = std::asin(sp > 1.0 ? 1.0 : (sp < -1.0 ? -1.0 : sp));
  if (std::abs(sp) > 0.999999) {
    rpy[0] = 0.0;
    rpy[1] = pitch;
    rpy[2] = std::atan2(-R(0, 1), R(1, 1));
    return;
  }
  rpy[0] = std::atan2(R(2, 1), R(2, 2));
  rpy[1] = pitch;
  rpy[2] = std::atan2(R(1, 0), R(0, 0));
}

/// 片足の変換結果。どこで詰まったかを呼び側が切り分けられるように分けて返す。
struct LegSolve
{
  rk::IkStatus ik_status = rk::IkStatus::Ok;
  rk::LegServoStatus servo_status = rk::LegServoStatus::Ok;
  bool ankle_clamped = false;        //!< 足首の指令が可動域で丸められた
  bool ok() const
  {
    return ik_status == rk::IkStatus::Ok && servo_status == rk::LegServoStatus::Ok;
  }
};

/// 指令側: 足裏の目標 (p, R) -> 絶対サーボ角 6 本 [rad]。
///
/// theta を返すのは /joint_states に出すため（同じ IK を 2 回解かない）。
///
/// **足首は ankleClampJoints() を必ず通す。** θ6 は Δ > 0 のまま特異点に入るので
/// 逆変換だけでは止まらない（ankle_parallel.hpp の注記）。ここを飛ばすと
/// 軌道生成が可動域を超えたときに足首が跳ねる。
inline LegSolve servoFromFootPose(
  const rk::LegServoParams & prm, const FootPose & foot,
  double servo[rk::kNumJoints], double theta[rk::kNumJoints])
{
  LegSolve out;
  out.ik_status = rk::ik(prm.leg, foot.p, matFromRpy(foot.rpy), theta, /*clamp=*/true);
  if (out.ik_status == rk::IkStatus::NoBranch) {return out;}

  const rk::AnkleClampResult ac =
    rk::ankleClampJoints(theta[rk::ANKLE_PITCH], theta[rk::ANKLE_ROLL]);
  theta[rk::ANKLE_PITCH] = ac.th5;
  theta[rk::ANKLE_ROLL] = ac.th6;
  out.ankle_clamped = ac.clamped;

  out.servo_status = rk::legServoFromJoints(prm, theta, servo);
  return out;
}

/// 観測側: 絶対サーボ角 6 本 [rad] -> 関節角と足裏の姿勢。
///
/// th6_seed は足首の順変換（1 変数ニュートン法）の種で、前周期の θ6 を渡す。
/// 収束しなければ粗探し（ankleFkScan）へ 1 回だけ落ちる。起動直後や、脱力中に
/// 手で大きく動かした直後がこれに当たる。
inline rk::LegServoStatus footPoseFromServo(
  const rk::LegServoParams & prm, const double servo[rk::kNumJoints],
  FootPose & foot, double theta[rk::kNumJoints], double & th6_seed)
{
  rk::LegServoStatus st = rk::legJointsFromServo(prm, servo, theta, th6_seed);

  // 足首は脚全体とは別に解き直す（膝が読めなくても足首は出す。leg_live_test と同じ理由）。
  const double q[rk::kAnkleChains] = {
    rk::ankleCrankFromServo(prm.ankle, 0, servo[rk::ANKLE_PITCH]),
    rk::ankleCrankFromServo(prm.ankle, 1, servo[rk::ANKLE_ROLL])};
  rk::AnkleFkResult afk = rk::ankleFk(prm.ankle, q, th6_seed);
  if (afk.status != rk::AnkleFkStatus::Ok) {afk = rk::ankleFkScan(prm.ankle, q);}
  if (afk.status == rk::AnkleFkStatus::Ok) {
    theta[rk::ANKLE_PITCH] = afk.th5;
    theta[rk::ANKLE_ROLL] = afk.th6;
    th6_seed = afk.th6;
  }

  rk::Vec3 p;
  rk::Mat3 R;
  rk::fk(prm.leg, theta, p, R);
  foot.p = p;
  rpyFromMat(R, foot.rpy);
  return st;
}

/// 足裏の目標が「どこまで安心して使えるか」の 3 段階。
///
/// 判定の中身は roboone_walk_core/src/gait_from_kinematics.cpp の probe() と同じ。
/// あちらは歩行パラメータ (gait.yaml の v_max・foot_spacing・z_c) を **決める**ために
/// 到達域を走査し、こちらは走らせている軌道がその範囲に収まっているかを**確かめる**。
/// しきい値はどちらも同じ config ヘッダ (JOINT_LIMIT_* / CRANK_LIMIT_DEG /
/// TH5_LIMIT_DEG) を読むので、値がずれることはない。組み合わせ方だけが 2 か所にある。
enum class ReachLevel
{
  None = 0,     //!< IK が解けない
  IkOnly = 1,   //!< 解けるが関節リミットか機構の外
  Mech = 2,     //!< 機構としては届く (膝 4 節が閉じ、足首クランクが ±60 deg 以内)
  Design = 3,   //!< 足首が設計可動域 (同時 ±15 deg の菱形) の内側。歩行はここで回す
};

inline const char * reachLevelName(ReachLevel l)
{
  switch (l) {
    case ReachLevel::Design: return "design";
    case ReachLevel::Mech: return "mech";
    case ReachLevel::IkOnly: return "ik";
    default: return "none";
  }
}

/// 足裏の目標 (p, R) がどの段階まで実現できるかを見る。
inline ReachLevel reachLevel(const rk::LegServoParams & prm, const FootPose & foot)
{
  const double d2r = M_PI / 180.0;
  double theta[rk::kNumJoints]{};
  // clamp=false。丸めた結果で判定すると、可動域の外を「届いた」と誤って報告する。
  if (rk::ik(prm.leg, foot.p, matFromRpy(foot.rpy), theta, /*clamp=*/false) != rk::IkStatus::Ok) {
    return ReachLevel::None;
  }
  const double bend = rk::kneeBendFromLegAngle(prm.leg, theta[rk::KNEE]);
  for (std::size_t k = 0; k < rk::kNumJoints; ++k) {
    const double v = (k == rk::KNEE) ? bend : theta[k];
    if (v < rk::config::JOINT_LIMIT_LO_DEG[k] * d2r ||
      v > rk::config::JOINT_LIMIT_HI_DEG[k] * d2r)
    {
      return ReachLevel::IkOnly;
    }
  }
  double servo[rk::kNumJoints]{};
  if (rk::legServoFromJoints(prm, theta, servo) != rk::LegServoStatus::Ok) {
    return ReachLevel::IkOnly;
  }
  // 足首クランク ±60 deg。servo = σ·n·q + zero で n = 1・zero = 0 なので |q| そのもの
  for (std::size_t k : {rk::ANKLE_PITCH, rk::ANKLE_ROLL}) {
    if (std::abs(servo[k]) > rk::ankle_config::CRANK_LIMIT_DEG[0][1] * d2r) {
      return ReachLevel::Mech;
    }
  }
  // 設計可動域は同時 ±15 deg の菱形。enum の ANKLE_PITCH は Σ_B ではロール、
  // ANKLE_ROLL はピッチ (leg_kinematics.hpp 冒頭の約束)。
  const double lim = rk::ankle_config::TH5_LIMIT_DEG[1] * d2r;
  const double roll = theta[rk::ANKLE_PITCH], pitch = theta[rk::ANKLE_ROLL];
  if (std::abs(roll) / lim + std::abs(pitch) / lim <= 1.0 + 1e-9) {return ReachLevel::Design;}
  return ReachLevel::Mech;
}

}  // namespace roboone_motion_node

#endif  // ROBOONE_MOTION_NODE__BODY_POSE_HPP_
