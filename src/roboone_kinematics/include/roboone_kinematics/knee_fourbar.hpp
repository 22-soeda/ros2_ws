// 膝 4 節リンクの順変換 / 逆変換。モータ角 θ2 <-> 膝ロッカー角 θ4。
//
// 土台は docs/膝4節リンク導出.pdf。(KN-n) はその式番号を指す。leg_kinematics.hpp が
// 出す膝関節角と、実機のサーボ角を相互に変換する層で、motion ノードでは
//
//   指令側: 足先目標 -> ik() -> θ4 -> kneeIk() -> θ2 -> サーボ指令
//   観測側: サーボ実測 -> θ2 -> kneeFk() -> θ4 -> fk() -> 重心位置
//
// の順に呼ぶ。leg_kinematics.hpp / ankle_parallel.hpp と同じくヘッダオンリ・
// ROS 非依存・ヒープ無しで、200Hz の制御ループから直接呼べる。
// 寸法は knee_config.hpp に、脚 IK との繋ぎこみは leg_servo.hpp にある。
//
// Python 参照実装 scripts/knee_fourbar.py と同じ式で、両者は
// scripts/crosscheck_knee.py で突き合わせている。
//
// ===========================================================================
// 機構と記号（文書 §1.2・§1.3）
// ===========================================================================
// 大腿に載せたモータがクランクとカプラを介して、下腿に生えたレバーを押し引きする
// 平面 4 節リンク。矢状面に載るので 2 次元で閉じる。
//
//   O4 = (0, 0)                     膝軸。原点
//   O2 = (r1, 0)                    モータ軸
//   A  = O2 + r2·(cos θ2, sin θ2)   クランク先端のピン
//   B  = O4 + r4·(cos θ4, sin θ4)   ロッカー側のピン
//   拘束: |B − A| = r3                                                  (KN-3)
//
// θ3（カプラ角）は A と B が決まれば従属に決まるので独立変数ではない。
//
// ===========================================================================
// 全体の仕掛け
// ===========================================================================
// 出発点は カプラ 1 本ぶんの閉ループ拘束
//
//   Λ(θ2, θ4) := |B(θ4) − A(θ2)|² − r3² = 0                             (KN-3)
//
// で、これが (cos θ2, sin θ2) についても (cos θ4, sin θ4) についても **1 次**
// であることが全ての鍵になる。足首と同じ性質だが、違うのは未知数が 1 つしか
// ないことで、そのため消去が要らない。片方が既知になれば残る 1 組が直接解ける。
//
// 順変換 θ2 -> θ4  A が定点。|B|² = r4² が θ4 に依らないので展開すると
//   Λ = 2(G − E·cos θ4 − F·sin θ4),  E = r4·A_x, F = r4·A_y,
//   G = (r4² + d² − r3²)/2,  d = |A|。Lagrange 恒等式から E² + F² = r4²d² なので
//     θ4 = atan2(A_y, A_x) + β·arccos( (r4² + d² − r3²)/(2·r4·d) )      (KN-5)
//   幾何的には O4 中心・半径 r4 の円と A 中心・半径 r3 の円の交わり。
//
// 逆変換 θ4 -> θ2  B が定点。w := B − O2 と置くと
//   Λ = 2·r2·(S − P'·cos θ2 − Q'·sin θ2),  P' = w_x, Q' = w_y,
//   S = (|w|² + r2² − r3²)/(2·r2)。P'² + Q'² = |w|² なので
//     θ2 = atan2(w_y, w_x) + ε·arccos( (|w|² + r2² − r3²)/(2·r2·|w|) )  (KN-7)
//
// (KN-5) と (KN-7) は同じ形をしている。三角形 O4–A–B を A の側から閉じるか
// B の側から閉じるかの違いしかない。どちらも atan2 1 回と arccos 1 回で閉じ、
// 反復も初期値もいらない。足首で 8 次式が出たのは 2 自由度・2 ループだったからで、
// 膝ではそれが起きない。
//
// 半角正接（Weierstrass）置換で 2 次式に落とす手は使わない。θ = π 近傍で破綻する
// ため、atan2 と arccos の形のまま扱う（文書 §5）。
//
// 枝 β, ε は円と円の 2 交点のどちらに組んであるかで、機体を組んだ時点で決まる定数。
// 到達可能性は arccos の中身の絶対値が 1 以下であることで、これは三角形の成立条件
// そのもの（(KN-8)）。外れたら Unreachable を返し、握り潰さない。
//
// ===========================================================================
// atan2 の分枝
// ===========================================================================
// atan2 の素の戻り値は (−π, π] なので、そのまま使うと θ2 のスイープに対して θ4 が
// 2π 跳ぶ。基準角を [0, 2π) に畳んでから枝を足す（Python 参照実装・既存 MATLAB
// 実装と同じ規約）。ここを揃えないと θ4 の系列が不連続になり、伝達比の符号判定と
// 可動域チェックが壊れる。
#ifndef ROBOONE_KINEMATICS__KNEE_FOURBAR_HPP_
#define ROBOONE_KINEMATICS__KNEE_FOURBAR_HPP_

#include <cmath>

#include "roboone_kinematics/knee_config.hpp"
#include "roboone_kinematics/leg_kinematics.hpp"

namespace roboone_kinematics
{

/// kneeFk() / kneeIk() の結果。
enum class KneeStatus
{
  Ok = 0,
  //! arccos の中身が ±1 を超えた。三角形が閉じない = 可動域の外 (KN-8)
  Unreachable,
  //! d ≈ 0 または |w| ≈ 0。atan2 の向きが決まらない退化姿勢
  Degenerate,
  //! 2 交点が重なる死点。速度の写像（M が特異）が定義できない
  DeadPoint,
};

/// 矢状面の 2 次元ベクトル。この平面の外では使わない。
struct Vec2
{
  double x{0.0}, y{0.0};

  constexpr Vec2 operator+(const Vec2 & o) const {return {x + o.x, y + o.y};}
  constexpr Vec2 operator-(const Vec2 & o) const {return {x - o.x, y - o.y};}
  constexpr double dot(const Vec2 & o) const {return x * o.x + y * o.y;}
  //! 2 次元の外積 u_x·v_y − u_y·v_x
  constexpr double cross(const Vec2 & o) const {return x * o.y - y * o.x;}
  constexpr double normSq() const {return x * x + y * y;}
  double norm() const {return std::sqrt(normSq());}
};

/// 4 節リンクの 1 姿勢。角は rad、長さは mm。
struct KneePose
{
  double theta2{0.0};   //!< クランク角（モータ側）
  double theta3{0.0};   //!< カプラ角（従属）
  double theta4{0.0};   //!< ロッカー角（膝ジョイント側）
  Vec2 a{};             //!< クランク先端のピン A
  Vec2 b{};             //!< ロッカー側のピン B

  //! d = B − A。カプラのベクトル
  constexpr Vec2 coupler() const {return b - a;}
};

// ---------------------------------------------------------------------------
// パラメータ
// ---------------------------------------------------------------------------
struct KneeParams
{
  // ---- 入力（knee_config.hpp の値） ----
  double r1{kconfig::R1};   //!< 地節（大腿）  O4 -> O2
  double r2{kconfig::R2};   //!< クランク（入力）
  double r3{kconfig::R3};   //!< カプラ
  double r4{kconfig::R4};   //!< ロッカー（出力）
  int beta{kconfig::BETA};  //!< 順変換 (KN-5) の枝 ±1
  int eps{kconfig::EPS};    //!< 逆変換 (KN-7) の枝 ±1

  //! θ4_rocker = sigmaJoint·bend + theta4Zero   [rad]
  double theta4Zero{kconfig::THETA4_ZERO_DEG * M_PI / 180.0};
  int sigmaJoint{kconfig::SIGMA_JOINT};

  //! φ4 = servoZero + servoSign·gear·θ2         [rad]
  //! この 3 つだけは左右で違う（膝サーボは左右とも ID 4 で、原点は取り付けごと）。
  //! 既定は右脚。左脚は makeKneeParams(Side::LEFT) で作る。
  double servoZero{kconfig::SERVO_ZERO_DEG[0] * M_PI / 180.0};
  double servoSign{static_cast<double>(kconfig::SERVO_SIGN[0])};
  double gear{kconfig::GEAR_RATIO[0]};

  /// 前提条件を満たしているか。
  bool valid() const
  {
    return r1 >= 0.0 && r2 > 0.0 && r3 > 0.0 && r4 > 0.0 &&
           (beta == 1 || beta == -1) && (eps == 1 || eps == -1) &&
           (sigmaJoint == 1 || sigmaJoint == -1) &&
           gear != 0.0 && std::fabs(std::fabs(servoSign) - 1.0) < 1e-12;
  }
};

/// knee_config.hpp から左右脚のパラメータを組み立てる。
///
/// 矢状面内の寸法・枝・θ4_zero・σ_knee は左右共通（鏡像になるのは膝軸方向の
/// オフセットだけ。文書 §11）。左右で変わるのは**サーボの原点・向き・ギア比**だけで、
/// 膝サーボが左右とも ID 4 なぶん、どちらのバスに繋がっているかがそれを決める。
inline KneeParams makeKneeParams(Side side = Side::RIGHT)
{
  const int k = (side == Side::LEFT) ? 1 : 0;
  KneeParams prm;
  prm.servoZero = kconfig::SERVO_ZERO_DEG[k] * M_PI / 180.0;
  prm.servoSign = static_cast<double>(kconfig::SERVO_SIGN[k]);
  prm.gear = kconfig::GEAR_RATIO[k];
  return prm;
}

// ---------------------------------------------------------------------------
namespace kdetail
{

//! 長さがこれ以下なら atan2 の向きが決まらないものとして退化扱いにする [mm]
inline constexpr double kLenTol = 1e-12;
//! arccos の引数がこの分だけ 1 を超えるのは丸めとみなしてクランプする
inline constexpr double kCosTol = 1e-12;

/// 角を [0, 2π) に畳む。(KN-5)(KN-7) の atan2 の基準角に使う。
inline double wrapTwoPi(double a)
{
  a = std::fmod(a, 2.0 * M_PI);
  return a < 0.0 ? a + 2.0 * M_PI : a;
}

/// 角を (−π, π] に畳む。差を取るときに使う。
inline double wrapPi(double a) {return std::atan2(std::sin(a), std::cos(a));}

}  // namespace kdetail

// ---------------------------------------------------------------------------
// 幾何の小道具
// ---------------------------------------------------------------------------
/// A = O2 + r2·e(θ2)。
inline Vec2 kneeCrankPin(const KneeParams & prm, double theta2)
{
  return {prm.r1 + prm.r2 * std::cos(theta2), prm.r2 * std::sin(theta2)};
}

/// B = O4 + r4·e(θ4)。
inline Vec2 kneeRockerPin(const KneeParams & prm, double theta4)
{
  return {prm.r4 * std::cos(theta4), prm.r4 * std::sin(theta4)};
}

/// 拘束の残差 |B − A|² − r3²（(KN-3) の Λ）。検算用。
inline double kneeConstraint(const KneeParams & prm, double theta2, double theta4)
{
  const Vec2 d = kneeRockerPin(prm, theta4) - kneeCrankPin(prm, theta2);
  return d.normSq() - prm.r3 * prm.r3;
}

/// θ2 で三角形が閉じるか。|d − r4| <= r3 <= d + r4 (KN-8)。
inline bool kneeAssemblableTheta2(const KneeParams & prm, double theta2)
{
  const double d = kneeCrankPin(prm, theta2).norm();
  return std::fabs(d - prm.r4) <= prm.r3 && prm.r3 <= d + prm.r4;
}

/// θ4 で三角形が閉じるか。| |w| − r2 | <= r3 <= |w| + r2。
inline bool kneeAssemblableTheta4(const KneeParams & prm, double theta4)
{
  const Vec2 b = kneeRockerPin(prm, theta4);
  const double w = std::hypot(b.x - prm.r1, b.y);
  return std::fabs(w - prm.r2) <= prm.r3 && prm.r3 <= w + prm.r2;
}

// ---------------------------------------------------------------------------
// 順変換 θ2 -> θ4                                                      (KN-5)
// ---------------------------------------------------------------------------
/// モータ角 -> 膝ロッカー角。反復なしの閉形式（atan2 1 回 + arccos 1 回）。
///
/// 戻り値が Ok 以外のとき pose は触らない。arccos の引数は丸めぶん（1e-12）だけ
/// クランプするので、黙って NaN が返る経路は無い。
inline KneeStatus kneeFk(const KneeParams & prm, double theta2, KneePose & pose)
{
  const Vec2 a = kneeCrankPin(prm, theta2);
  const double d = a.norm();
  if (d < kdetail::kLenTol) {return KneeStatus::Degenerate;}

  // E² + F² = r4²d² なので E, F を経由せずに書ける
  const double c = (prm.r4 * prm.r4 + d * d - prm.r3 * prm.r3) / (2.0 * prm.r4 * d);
  if (!std::isfinite(c) || std::fabs(c) > 1.0 + kdetail::kCosTol) {
    return KneeStatus::Unreachable;
  }

  const double theta4 = kdetail::wrapTwoPi(std::atan2(a.y, a.x)) +
    prm.beta * std::acos(detail::clampUnit(c));
  const Vec2 b = kneeRockerPin(prm, theta4);

  pose.theta2 = theta2;
  pose.theta4 = theta4;
  pose.theta3 = std::atan2(b.y - a.y, b.x - a.x);
  pose.a = a;
  pose.b = b;
  return KneeStatus::Ok;
}

// ---------------------------------------------------------------------------
// 逆変換 θ4 -> θ2                                                      (KN-7)
// ---------------------------------------------------------------------------
/// 膝ロッカー角 -> モータ角。反復なしの閉形式。(KN-5) と同じ形。
///
/// r1 = r4 に乗った簡約（|w| = √(2r1²(1−cos θ4)) や atan2(w) = (θ4+π)/2）は
/// 使っていない。本機はたまたま r1 = r4 = 20 だが、r4 を 26 mm にするだけで
/// 前者は 6.0 mm、後者は 12.0 deg ずれるうえ、エラーも警告も出ない。
inline KneeStatus kneeIk(const KneeParams & prm, double theta4, KneePose & pose)
{
  const Vec2 b = kneeRockerPin(prm, theta4);
  const Vec2 w{b.x - prm.r1, b.y};
  const double wn = w.norm();
  if (wn < kdetail::kLenTol) {return KneeStatus::Degenerate;}

  const double s = (wn * wn + prm.r2 * prm.r2 - prm.r3 * prm.r3) / (2.0 * prm.r2 * wn);
  if (!std::isfinite(s) || std::fabs(s) > 1.0 + kdetail::kCosTol) {
    return KneeStatus::Unreachable;
  }

  const double theta2 = kdetail::wrapTwoPi(std::atan2(w.y, w.x)) +
    prm.eps * std::acos(detail::clampUnit(s));
  const Vec2 a = kneeCrankPin(prm, theta2);

  pose.theta2 = theta2;
  pose.theta4 = theta4;
  pose.theta3 = std::atan2(b.y - a.y, b.x - a.x);
  pose.a = a;
  pose.b = b;
  return KneeStatus::Ok;
}

// ---------------------------------------------------------------------------
// 伝達比と伝達角
// ---------------------------------------------------------------------------
/// 伝達比 dθ4/dθ2 (KN-10)。解析式なので数値微分しない。
///
///   dθ4/dθ2 = ((A − O2) × d) / ((B − O4) × d),   d = B − A
///
/// 分母が消えるのはカプラとロッカーが平行（死点）、分子が消えるのはカプラと
/// クランクが平行になるとき。静力学は τ_θ2 = (dθ4/dθ2)·τ_θ4 で、伝達比が
/// そのままトルク比になる。
inline KneeStatus kneeRatio(const KneeParams & prm, const KneePose & pose, double & out)
{
  const Vec2 d = pose.coupler();
  const Vec2 crank{pose.a.x - prm.r1, pose.a.y};
  const double den = pose.b.cross(d);
  if (std::fabs(den) < kdetail::kLenTol) {return KneeStatus::DeadPoint;}
  out = crank.cross(d) / den;
  return KneeStatus::Ok;
}

/// 伝達角 γ = |θ3 − θ4| [rad] (KN-11)。カプラとロッカーのなす角。
/// 0 や π に近いとカプラの力がロッカーを回す成分をほとんど持たない。
/// 設計の目安は 40 deg <= γ <= 140 deg。
inline double kneeTransmissionAngle(const KneePose & pose)
{
  return std::fabs(kdetail::wrapPi(pose.theta3 - pose.theta4));
}

// ---------------------------------------------------------------------------
// 速度・加速度
// ---------------------------------------------------------------------------
// ループ O2 + r2·e(θ2) + r3·e(θ3) − r4·e(θ4) = 0 を時間で 1 回・2 回微分する。
//
//   M = [ −r3·sin θ3,   r4·sin θ4 ]
//       [  r3·cos θ3,  −r4·cos θ4 ]
//
// **M の行の符号と右辺の符号は必ずセットで揃える。** 両辺を同時に −1 倍しても
// 解は変わらない（掛かって消える）が、片方だけ −1 倍すると速度は正しく出るのに
// 加速度だけが壊れる。knee_selftest がその取り違えを検出する。
namespace kdetail
{

struct Mat2
{
  double m11{0.0}, m12{0.0}, m21{0.0}, m22{0.0}, det{0.0};
};

inline Mat2 loopMatrix(const KneeParams & prm, const KneePose & pose)
{
  const double s3 = std::sin(pose.theta3), c3 = std::cos(pose.theta3);
  const double s4 = std::sin(pose.theta4), c4 = std::cos(pose.theta4);
  Mat2 m;
  m.m11 = -prm.r3 * s3;
  m.m12 = prm.r4 * s4;
  m.m21 = prm.r3 * c3;
  m.m22 = -prm.r4 * c4;
  m.det = m.m11 * m.m22 - m.m12 * m.m21;   // = r3·r4·sin(θ3 − θ4)
  return m;
}

}  // namespace kdetail

/// M·[ω3, ω4]ᵀ = [ r2·ω2·sin θ2, −r2·ω2·cos θ2 ]ᵀ を解く。
inline KneeStatus kneeVelocity(
  const KneeParams & prm, const KneePose & pose, double omega2,
  double & omega3, double & omega4)
{
  const kdetail::Mat2 m = kdetail::loopMatrix(prm, pose);
  if (std::fabs(m.det) < kdetail::kLenTol) {return KneeStatus::DeadPoint;}
  const double b1 = prm.r2 * omega2 * std::sin(pose.theta2);
  const double b2 = -prm.r2 * omega2 * std::cos(pose.theta2);
  omega3 = (b1 * m.m22 - m.m12 * b2) / m.det;
  omega4 = (m.m11 * b2 - b1 * m.m21) / m.det;
  return KneeStatus::Ok;
}

/// 同じループの 2 階微分。右辺は kneeVelocity() と同じ M に対する組。
///
///   M·[α3, α4]ᵀ = [ r2·α2·sinθ2 + r2·ω2²·cosθ2 + r3·ω3²·cosθ3 − r4·ω4²·cosθ4,
///                  −r2·α2·cosθ2 + r2·ω2²·sinθ2 + r3·ω3²·sinθ3 − r4·ω4²·sinθ4 ]ᵀ
inline KneeStatus kneeAcceleration(
  const KneeParams & prm, const KneePose & pose, double omega2, double alpha2,
  double & alpha3, double & alpha4)
{
  double w3 = 0.0, w4 = 0.0;
  const KneeStatus st = kneeVelocity(prm, pose, omega2, w3, w4);
  if (st != KneeStatus::Ok) {return st;}

  const kdetail::Mat2 m = kdetail::loopMatrix(prm, pose);
  const double s2 = std::sin(pose.theta2), c2 = std::cos(pose.theta2);
  const double s3 = std::sin(pose.theta3), c3 = std::cos(pose.theta3);
  const double s4 = std::sin(pose.theta4), c4 = std::cos(pose.theta4);
  const double b1 = prm.r2 * alpha2 * s2 + prm.r2 * omega2 * omega2 * c2 +
    prm.r3 * w3 * w3 * c3 - prm.r4 * w4 * w4 * c4;
  const double b2 = -prm.r2 * alpha2 * c2 + prm.r2 * omega2 * omega2 * s2 +
    prm.r3 * w3 * w3 * s3 - prm.r4 * w4 * w4 * s4;
  alpha3 = (b1 * m.m22 - m.m12 * b2) / m.det;
  alpha4 = (m.m11 * b2 - b1 * m.m21) / m.det;
  return KneeStatus::Ok;
}

// ---------------------------------------------------------------------------
// 膝関節角 <-> ロッカー角 <-> サーボ指令
// ---------------------------------------------------------------------------
// bend は膝の曲げ量（伸展 0・屈曲 +）で、leg_config.hpp の
// JOINT_LIMIT[KNEE] = 0..150 deg と同じ量。脚 IK の公開角との読み替えは
// leg_servo.hpp の kneeBendFromLegAngle() に閉じ込めてある。

/// 曲げ量 -> ロッカー絶対角。
inline double kneeRockerFromBend(const KneeParams & prm, double bend)
{
  return prm.sigmaJoint * bend + prm.theta4Zero;
}

/// ロッカー絶対角 -> 曲げ量。
inline double kneeBendFromRocker(const KneeParams & prm, double rocker)
{
  return prm.sigmaJoint * (rocker - prm.theta4Zero);
}

/// クランク角 -> サーボ指令 φ4 = φ40 + σ4·n4·θ2。
inline double kneeServoFromCrank(const KneeParams & prm, double theta2)
{
  return prm.servoZero + prm.servoSign * prm.gear * theta2;
}

/// サーボ指令 -> クランク角。
inline double kneeCrankFromServo(const KneeParams & prm, double phi)
{
  return (phi - prm.servoZero) / (prm.servoSign * prm.gear);
}

/// 指令側: 膝の曲げ量 -> サーボ指令。
inline KneeStatus kneeServoFromBend(const KneeParams & prm, double bend, double & servo)
{
  KneePose pose;
  const KneeStatus st = kneeIk(prm, kneeRockerFromBend(prm, bend), pose);
  if (st != KneeStatus::Ok) {return st;}
  servo = kneeServoFromCrank(prm, pose.theta2);
  return KneeStatus::Ok;
}

/// 観測側: サーボ実測角 -> 膝の曲げ量。
inline KneeStatus kneeBendFromServo(const KneeParams & prm, double servo, double & bend)
{
  KneePose pose;
  const KneeStatus st = kneeFk(prm, kneeCrankFromServo(prm, servo), pose);
  if (st != KneeStatus::Ok) {return st;}
  bend = kneeBendFromRocker(prm, pose.theta4);
  return KneeStatus::Ok;
}

// ---------------------------------------------------------------------------
// 組み立て時に 1 回だけ使うもの
// ---------------------------------------------------------------------------
/// 実測した 1 姿勢 (θ2, θ4) から枝 (β, ε) を決める。
///
/// 円と円の 2 交点のどちらに組んであるかは機体を組んだ時点で決まる定数なので、
/// 実機で 1 姿勢だけ測ればこれで確定する。結果を knee_config.hpp に書く。
/// 死点（2 交点が重なる姿勢）では決まらないので、そこで測ってはいけない。
inline KneeStatus kneeBranchesFromPose(
  const KneeParams & prm, double theta2, double theta4, int & beta, int & eps)
{
  const Vec2 a = kneeCrankPin(prm, theta2);
  const Vec2 b = kneeRockerPin(prm, theta4);
  if (a.norm() < kdetail::kLenTol || std::hypot(b.x - prm.r1, b.y) < kdetail::kLenTol) {
    return KneeStatus::Degenerate;
  }
  // その姿勢が本当に拘束を満たしているか（測り間違いをここで弾く）
  if (std::fabs((b - a).norm() - prm.r3) > 1e-6) {return KneeStatus::Unreachable;}

  const double dFk = kdetail::wrapPi(theta4 - std::atan2(a.y, a.x));
  const double dIk = kdetail::wrapPi(theta2 - std::atan2(b.y, b.x - prm.r1));
  if (std::fabs(dFk) < 1e-9 || std::fabs(std::fabs(dFk) - M_PI) < 1e-9 ||
    std::fabs(dIk) < 1e-9 || std::fabs(std::fabs(dIk) - M_PI) < 1e-9)
  {
    return KneeStatus::DeadPoint;
  }
  beta = dFk > 0.0 ? +1 : -1;
  eps = dIk > 0.0 ? +1 : -1;
  return KneeStatus::Ok;
}

/// Freudenstein の式 (KN-9) の残差。導出との答え合わせ用。
///
///   K1·cos θ2 − K2·cos θ4 + K3 = cos(θ2 − θ4)
///   K1 = r1/r4,  K2 = r1/r2,  K3 = (r1² + r2² + r4² − r3²)/(2·r2·r4)
///
/// 地節を +x 上に取ってあるので基準角の項が落ちている。
inline double kneeFreudensteinResidual(const KneeParams & prm, const KneePose & pose)
{
  const double k1 = prm.r1 / prm.r4;
  const double k2 = prm.r1 / prm.r2;
  const double k3 = (prm.r1 * prm.r1 + prm.r2 * prm.r2 + prm.r4 * prm.r4 -
    prm.r3 * prm.r3) / (2.0 * prm.r2 * prm.r4);
  return k1 * std::cos(pose.theta2) - k2 * std::cos(pose.theta4) + k3 -
         std::cos(pose.theta2 - pose.theta4);
}

}  // namespace roboone_kinematics

#endif  // ROBOONE_KINEMATICS__KNEE_FOURBAR_HPP_
