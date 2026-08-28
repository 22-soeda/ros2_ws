// 片脚の関節角 <-> サーボ指令。変換層をまとめて 1 か所にする。
//
// leg_kinematics.hpp が出すのは「関節角」であってサーボ指令角ではない（文書 §7）。
// 実機との間には 3 つの変換が挟まる。このヘッダはその 3 つを束ねて、motion ノードが
//
//   指令側: 足先目標 -> ik() -> (θ1..θ6) -> legServoFromJoints() -> サーボ指令
//   観測側: サーボ実測 -> legJointsFromServo() -> (θ1..θ6) -> fk() -> 重心位置
//
// の 2 行で書けるようにする。内訳は
//
//   J1–J3（股 3 軸）  サーボ直結。変換なし
//   J4（膝）          4 節リンク。knee_fourbar.hpp    (KN-5)/(KN-7)
//   J5・J6（足首）    パラレルリンク。ankle_parallel.hpp (AP-8)/(AP-15)
//
// ===========================================================================
// 膝の繋ぎこみ（このヘッダの主眼）
// ===========================================================================
// 脚 IK の公開角 θ4（Σ_B、AXIS_FLIP 適用後）と、4 節リンクのロッカー絶対角 θ4_rocker
// は別物で、2 段の読み替えで結ばれる。
//
//   [1] 公開角 -> 曲げ量 bend（伸展 0・屈曲 +、leg_config の JOINT_LIMIT[KNEE] と同じ量）
//         θ4_S = θ4_B · sign[KNEE] · kAxisSwapSign[KNEE]      (X-swap) と AXIS_FLIP
//         bend = σ_leg · (θ4_S − φ)                            解析解の θ4 = σ·bend + φ
//       既定値（AXIS_FLIP なし・P3_X = 0・KNEE_FORWARD = +1）では bend = θ4_B に落ちる。
//
//   [2] 曲げ量 -> ロッカー角
//         θ4_rocker = σ_knee · bend + θ4_zero                  knee_config.hpp
//
// θ4_zero = 89.3 deg は「脚が伸び切った姿勢（T ポーズ）でのロッカー角」で確定値。
// σ_knee はそこから一意に決まる（+1。knee_config.hpp のコメントを参照）。
//
// ===========================================================================
// 注意
// ===========================================================================
// * J1–J3 は「関節角 = サーボ指令角」として素通しする。サーボごとの原点・ギア比が
//   要るなら、ここではなくサーボドライバ層（feetech_servo）で持つ。
//   TODO(実機): 股 3 軸に原点オフセットが要るか、原点出しのときに確かめる。
// * 足首は ankle_parallel.hpp の約束どおり leg_kinematics.hpp の θ5・θ6 をそのまま
//   渡している。ankle_config.hpp の点の座標系（Σ_s の取り方）は CAD 待ちの TODO が
//   残っているので、足首側は数値が入ってから改めて突き合わせること。膝の経路は
//   knee_selftest / crosscheck_knee.py で検算済み。
// * 足首の順変換だけは閉形式にならず 1 変数ニュートン法なので、前周期の θ6 を
//   種として渡す（文書 §10）。膝と股には反復も種も要らない。
#ifndef ROBOONE_KINEMATICS__LEG_SERVO_HPP_
#define ROBOONE_KINEMATICS__LEG_SERVO_HPP_

#include <cmath>
#include <cstddef>

#include "roboone_kinematics/ankle_parallel.hpp"
#include "roboone_kinematics/knee_fourbar.hpp"
#include "roboone_kinematics/leg_kinematics.hpp"

namespace roboone_kinematics
{

/// legServoFromJoints() / legJointsFromServo() の結果。
/// 歩行中に Ok 以外が出るのは軌道生成が可動域を超えたときなので、握り潰さずに記録する。
enum class LegServoStatus
{
  Ok = 0,
  KneeUnreachable,    //!< 膝の三角形が閉じない (KN-8)
  KneeDegenerate,     //!< 膝が退化姿勢（設計で避ける）
  KneeDeadPoint,      //!< 膝が死点
  AnkleUnreachable,   //!< 足首のロッドが届かない
  AnkleDegenerate,    //!< 足首が退化姿勢
  AnkleNotConverged,  //!< 足首の順変換が収束しなかった
  AnkleSingular,      //!< 足首が特異姿勢
  //! 足首を安全なエンベロープに丸めた。**姿勢としては使える**（暴れない）が、
  //! 軌道生成が可動域を超えた合図なので記録すること
  AnkleClamped,
};

/// 片脚ぶんの変換層のパラメータ。
struct LegServoParams
{
  LegParams leg{};
  KneeParams knee{};
  AnkleParams ankle{};
};

/// 各 config から左右脚のパラメータを組み立てる。
inline LegServoParams makeLegServoParams(Side side)
{
  LegServoParams prm;
  prm.leg = makeLegParams(side);
  prm.knee = makeKneeParams(side);
  prm.ankle = makeAnkleParams(side);
  return prm;
}

// ---------------------------------------------------------------------------
// 膝: 脚 IK の公開角 <-> 曲げ量
// ---------------------------------------------------------------------------
// 掛かるのは ±1 だけなので、どちら向きの変換も同じ式でよい。

/// 脚 IK の公開角 θ4（Σ_B・AXIS_FLIP 適用後）-> 膝の曲げ量（伸展 0・屈曲 +）。
inline double kneeBendFromLegAngle(const LegParams & leg, double theta4B)
{
  const double th4S = theta4B * leg.sign[KNEE] * kAxisSwapSign[KNEE];
  return leg.sigma * (th4S - leg.phi);
}

/// 膝の曲げ量 -> 脚 IK の公開角 θ4。
inline double legAngleFromKneeBend(const LegParams & leg, double bend)
{
  const double th4S = leg.sigma * bend + leg.phi;
  return th4S * leg.sign[KNEE] * kAxisSwapSign[KNEE];
}

// ---------------------------------------------------------------------------
namespace lsdetail
{

inline LegServoStatus fromKnee(KneeStatus st)
{
  switch (st) {
    case KneeStatus::Ok: return LegServoStatus::Ok;
    case KneeStatus::Unreachable: return LegServoStatus::KneeUnreachable;
    case KneeStatus::Degenerate: return LegServoStatus::KneeDegenerate;
    default: return LegServoStatus::KneeDeadPoint;
  }
}

}  // namespace lsdetail

// ---------------------------------------------------------------------------
// 指令側: 関節角 -> サーボ指令
// ---------------------------------------------------------------------------
/// theta  Σ_B の関節角（ik() が返すのと同じ符号）
/// servo  出力。並びは Joint enum と同じ {J1..J6}
///
/// 膝で失敗したら足首は解かずに返す。足首だけが失敗した場合も膝までは書き戻す
/// （呼び側が「どこで可動域を超えたか」を切り分けられるように）。
inline LegServoStatus legServoFromJoints(
  const LegServoParams & prm, const double theta[kNumJoints], double servo[kNumJoints])
{
  // J1–J3 は直結
  servo[HIP_PITCH] = theta[HIP_PITCH];
  servo[HIP_ROLL] = theta[HIP_ROLL];
  servo[HIP_YAW] = theta[HIP_YAW];

  // J4 は 4 節リンク (KN-7)
  const double bend = kneeBendFromLegAngle(prm.leg, theta[KNEE]);
  double kneeServo = 0.0;
  const KneeStatus kst = kneeServoFromBend(prm.knee, bend, kneeServo);
  if (kst != KneeStatus::Ok) {return lsdetail::fromKnee(kst);}
  servo[KNEE] = kneeServo;

  // J5・J6 は足首パラレルリンク (AP-8)
  //
  // ★逆変換の前にエンベロープで丸める。ピッチ θ6 は **Δ が正のまま型 2 特異点に
  //   入れてしまう**ので、Δ を見ている ankleIk() では止められない。ここで止めないと
  //   「指令は通ったのに、その姿勢を順変換で読み戻せない」状態に持っていける。
  const AnkleClampResult env =
    ankleClampJoints(theta[ANKLE_PITCH], theta[ANKLE_ROLL]);
  const AnkleIkResult ares = ankleIk(prm.ankle, env.th5, env.th6);
  servo[ANKLE_PITCH] = ankleServoFromCrank(prm.ankle, 0, ares.q[0]);
  servo[ANKLE_ROLL] = ankleServoFromCrank(prm.ankle, 1, ares.q[1]);
  if (ares.status == AnkleIkStatus::Unreachable) {return LegServoStatus::AnkleUnreachable;}
  if (ares.status == AnkleIkStatus::Degenerate) {return LegServoStatus::AnkleDegenerate;}
  if (env.clamped) {return LegServoStatus::AnkleClamped;}
  return LegServoStatus::Ok;
}

// ---------------------------------------------------------------------------
// 観測側: サーボ実測角 -> 関節角
// ---------------------------------------------------------------------------
/// servo    サーボの実測角。並びは Joint enum と同じ
/// theta    出力。Σ_B の関節角（fk() に渡せる符号）
/// th6Seed  足首の順変換に使う前周期の θ6。**収束を速くするだけで答えは選ばない**
///          ので、起動直後は 0 でよい（ankle_parallel.hpp 冒頭の [1]〜[3]）
///
/// 膝と股には反復も種も要らない。反復するのは足首だけ。
inline LegServoStatus legJointsFromServo(
  const LegServoParams & prm, const double servo[kNumJoints], double theta[kNumJoints],
  double th6Seed = 0.0)
{
  theta[HIP_PITCH] = servo[HIP_PITCH];
  theta[HIP_ROLL] = servo[HIP_ROLL];
  theta[HIP_YAW] = servo[HIP_YAW];

  double bend = 0.0;
  const KneeStatus kst = kneeBendFromServo(prm.knee, servo[KNEE], bend);
  if (kst != KneeStatus::Ok) {return lsdetail::fromKnee(kst);}
  theta[KNEE] = legAngleFromKneeBend(prm.leg, bend);

  const double q[kAnkleChains] = {
    ankleCrankFromServo(prm.ankle, 0, servo[ANKLE_PITCH]),
    ankleCrankFromServo(prm.ankle, 1, servo[ANKLE_ROLL])};
  const AnkleFkResult ares = ankleFk(prm.ankle, q, th6Seed);
  theta[ANKLE_PITCH] = ares.th5;
  theta[ANKLE_ROLL] = ares.th6;
  switch (ares.status) {
    // Clamped も姿勢としては使える（窓の縁で有界・連続）。次の周期の種にしてよい
    case AnkleFkStatus::Ok: return ares.crankClamped ?
        LegServoStatus::AnkleClamped : LegServoStatus::Ok;
    case AnkleFkStatus::Clamped: return LegServoStatus::AnkleClamped;
    case AnkleFkStatus::NoCurve: return LegServoStatus::AnkleUnreachable;
    case AnkleFkStatus::Singular: return LegServoStatus::AnkleSingular;
    default: return LegServoStatus::AnkleNotConverged;
  }
}

// ---------------------------------------------------------------------------
// 膝だけを扱う近道（足首の CAD 値待ちでも使える）
// ---------------------------------------------------------------------------
/// 脚 IK の公開角 θ4 -> 膝サーボ指令。
inline LegServoStatus legKneeServoFromAngle(
  const LegServoParams & prm, double theta4B, double & servo)
{
  return lsdetail::fromKnee(
    kneeServoFromBend(prm.knee, kneeBendFromLegAngle(prm.leg, theta4B), servo));
}

/// 膝サーボ実測角 -> 脚 IK の公開角 θ4。
inline LegServoStatus legKneeAngleFromServo(
  const LegServoParams & prm, double servo, double & theta4B)
{
  double bend = 0.0;
  const KneeStatus st = kneeBendFromServo(prm.knee, servo, bend);
  if (st != KneeStatus::Ok) {return lsdetail::fromKnee(st);}
  theta4B = legAngleFromKneeBend(prm.leg, bend);
  return LegServoStatus::Ok;
}

}  // namespace roboone_kinematics

#endif  // ROBOONE_KINEMATICS__LEG_SERVO_HPP_
