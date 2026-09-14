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
#ifndef ROBOONE_MOTION__BODY_POSE_HPP_
#define ROBOONE_MOTION__BODY_POSE_HPP_

#include <cmath>
#include <cstddef>
#include <vector>

#include "roboone_kinematics/leg_servo.hpp"
#include "roboone_motion/servo_map.hpp"

namespace roboone_motion
{

/// 片足の足裏中心の目標。Σ_B、位置 mm・姿勢 rad。
struct FootPose
{
  rk::Vec3 p{0.0, 0.0, -200.0};
  double rpy[3]{0.0, 0.0, 0.0};      //!< roll, pitch, yaw [rad]
};

/// 片脚の「書き方」。左右別・キーフレーム別に混ざってよい。
///
/// Foot は寸法が変わっても IK が吸収するのが取り柄で、届く範囲でしか書けない。
/// Servo はその逆で、IK の到達域の外（可動域の縁、足首の特異点の近く）でも
/// 必ず出せるかわりに、機体を組み替えたらその姿勢は付いてこない。
enum class LegMode
{
  Foot = 0,   //!< 足裏の (p, R) で書く。指令は IK を通る（既定）
  Servo,      //!< サーボ角で書く。指令は IK も FK も通らず、そのままカウントになる
};

/// 機体の 1 姿勢。
///
/// **どちらの書き方で書いた側も、解ける限り両方の表現を埋めておく。** 本体が
/// foot なら leg_servo は IK の影、本体が leg_servo なら foot は FK の影になる。
/// 補間の区間の両端で書き方が食い違ったときに、その場で IK を解かずに同じ土俵へ
/// 乗せられるようにするため（motion_library.hpp「書き方が混ざる区間」）。
/// どちらが本体かは leg_mode だけが決める。**指令 (writeTargets) は本体しか見ない。**
struct BodyPose
{
  FootPose foot[kNumSide];           //!< [kRight], [kLeft]
  std::vector<double> arm;           //!< [deg] ServoMap::arms() と同順・同数
  LegMode leg_mode[kNumSide]{LegMode::Foot, LegMode::Foot};
  //! 絶対サーボ角 [rad]（leg_servo.hpp の [3]）。並びは Joint enum。系は Σ_U
  double leg_servo[kNumSide][rk::kNumJoints]{};
  bool leg_servo_valid[kNumSide]{false, false};   //!< leg_servo が埋まっているか
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

// ===========================================================================
// 胴体の前傾 body_pitch
// ===========================================================================
// walk_core も motions.yaml も home_pose.yaml も、**骨盤が直立している**前提で
// 足裏の (p, R) を書いている。この系を Σ_U と呼ぶ。胴体を ψ だけ前へ倒すと、
// 実際の骨盤系 Σ_B は Σ_U に対して y 軸まわりに +ψ 回る。床に水平な足裏は Σ_B から
// 見ると -ψ 傾いて見え、位置も股ピッチ軸まわりに -ψ 回る:
//
//     p_B = p0 + Ry(-ψ)·(p_U - p0)        R_B = Ry(-ψ)·R_U
//
// これは FK の構造から **股ピッチ θ1 に -ψ を足すのと厳密に等価**である
// （θ1 は最も近位の回転なので、遠位の連鎖がまるごと回る）。したがって
// **膝・足首の関節角は 1 度も変わらない。** 足首 θ6 の余裕を食わずに前傾できる
// のはこのため。
//
// 対して home_pose.yaml の foot.rpy で pitch だけを動かすと、足裏の**位置はそのまま**
// で姿勢だけが回るので、その差を足首が全部吸収する。あちらが -10 deg で
// AnkleUnreachable に当たったのはこれが理由で、前傾そのものが無理だったわけではない。
// 2 つは別の操作なので、両方に値を入れると効果が足し合わさる。
//
// 適用するのは**サーボとの境界だけ**（指令は writeTargets、観測は measuredPose）。
// その上の層（walk_core / モーション再生 / 補間 / ホーム姿勢）は Σ_U のままで動く。
//
// ★★ motion_teach で捕まえた姿勢は Σ_B（実機そのもの）で出る。body_pitch を
//    入れた状態でティーチした行をそのまま motions.yaml へ貼ると、再生時に
//    もう一度 -ψ が掛かって二重に傾く。ティーチ側も同じ ψ を渡すこと。

/// 足裏の目標を股ピッチ軸（Σ_B の y 軸）まわりに a [rad] 回す。
inline void rotateFootAboutHipY(FootPose & f, double a)
{
  // 股ピッチ軸が骨盤原点を通るときだけ「原点まわりの回転」で済む。CAD が変わって
  // 股が前後・上下にずれたら、回転中心を p0 に直すこと（y 成分はこの回転に効かない）。
  static_assert(
    rk::config::HIP_X == 0.0 && rk::config::HIP_Z == 0.0,
    "股ピッチ軸が骨盤原点を通らない。body_pitch の回転中心を p0 へ直すこと");
  const double c = std::cos(a), s = std::sin(a);
  const double x = f.p.x, z = f.p.z;
  f.p.x = c * x + s * z;
  f.p.z = -s * x + c * z;
  const rk::Mat3 R = rk::rotY(a) * matFromRpy(f.rpy);
  rpyFromMat(R, f.rpy);
}

/// Σ_U（直立骨盤系）-> Σ_B（実際の骨盤系）。psi > 0 で胴体が前へ倒れる。
inline void bodyPitchApply(FootPose & f, double psi) {rotateFootAboutHipY(f, -psi);}
/// Σ_B -> Σ_U。実測姿勢を「直立骨盤系で書かれた姿勢」に戻す。
inline void bodyPitchRemove(FootPose & f, double psi) {rotateFootAboutHipY(f, +psi);}

/// サーボ角で書かれた脚に同じ前傾を掛ける（Σ_U -> Σ_B）。
///
/// 上の等価性（前傾 = 股ピッチに -ψ）をそのまま使う。足裏の (p, R) を経由しない
/// ので、**IK で戻せない姿勢でも前傾を掛けられる**。角度書きのキーフレームを
/// 足裏書きと同じ Σ_U に置いておけるのはこれがあるため。
///
/// ★ -ψ を掛けるのは Σ_B の**幾何の**股ピッチ角で、ここで足すのは leg_servo.hpp が
///   やり取りする公開角（AXIS_FLIP 適用後 = サーボの回る向き）。左脚は
///   AXIS_FLIP_LEFT[J1] = 1 で向きが逆なので、sign を掛けないと左だけ後傾する。
///   股 3 軸は「関節角 = サーボ角」で素通しなので、これでサーボ角にもそのまま効く。
inline void bodyPitchApplyServo(
  const rk::LegServoParams & prm, double servo[rk::kNumJoints], double psi)
{
  servo[rk::HIP_PITCH] += -psi * prm.leg.sign[rk::HIP_PITCH];
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
/// **足首は ankleClampJoints() を必ず通す。** ピッチ θ5 は Δ > 0 のまま特異点に
/// 入るので逆変換だけでは止まらない（ankle_parallel.hpp の注記）。ここを飛ばすと
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
/// th6_seed は足首の順変換（1 変数ニュートン法）の種で、前周期の θ6（ロール）を渡す。
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

/// 足裏書きの側に、対になるサーボ角（IK の影）を埋める。
///
/// 埋まらない（IK で解けない）姿勢は leg_servo_valid を false にして返すだけで、
/// foot 側は触らない。**その姿勢は元から実機に出せない**ので、ここで騒ぐより
/// 指令側 (writeTargets) の「解けない目標は送らない」に任せる。
inline bool fillLegServoFromFoot(const rk::LegServoParams & prm, BodyPose & pose, int side)
{
  double theta[rk::kNumJoints];
  const LegSolve r = servoFromFootPose(prm, pose.foot[side], pose.leg_servo[side], theta);
  pose.leg_servo_valid[side] = r.ok();
  return r.ok();
}

/// 角度書きの側に、対になる足裏 (p, R)（FK の影）を埋める。
///
/// **指令には使わない。** ログや /motion/state の表示、歩行へ戻ったときの起点が
/// 「前の技の最後の姿勢」として読める値であるために置く。機構として成り立たない
/// 角度（膝の三角形が閉じない等）だと FK が失敗するので、その場合は foot を
/// 触らずに false を返す（引き継いだ直前の値が残る）。
inline bool fillFootFromLegServo(
  const rk::LegServoParams & prm, BodyPose & pose, int side, double th6_seed = 0.0)
{
  double theta[rk::kNumJoints]{};
  FootPose f;
  const rk::LegServoStatus st =
    footPoseFromServo(prm, pose.leg_servo[side], f, theta, th6_seed);
  if (st != rk::LegServoStatus::Ok && st != rk::LegServoStatus::AnkleClamped) {return false;}
  pose.foot[side] = f;
  return true;
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
  // 足首クランクのリミット (CRANK_LIMIT_DEG。-42..+60 と非対称なので σ を戻して q で見る)
  const std::size_t ankleIdx[2] = {rk::ANKLE_PITCH, rk::ANKLE_ROLL};
  for (int c = 0; c < rk::kAnkleChains; ++c) {
    const double q = rk::ankleCrankFromServo(prm.ankle, c, servo[ankleIdx[c]]);
    if (q < prm.ankle.qMin[c] || q > prm.ankle.qMax[c]) {return ReachLevel::Mech;}
  }
  // 設計可動域は同時 ±15 deg の菱形。ANKLE_PITCH = θ5（上側）、ANKLE_ROLL = θ6（下側）
  const double lim = rk::ankle_config::TH5_LIMIT_DEG[1] * d2r;
  const double pitch = theta[rk::ANKLE_PITCH], roll = theta[rk::ANKLE_ROLL];
  if (std::abs(roll) / lim + std::abs(pitch) / lim <= 1.0 + 1e-9) {return ReachLevel::Design;}
  return ReachLevel::Mech;
}

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__BODY_POSE_HPP_
