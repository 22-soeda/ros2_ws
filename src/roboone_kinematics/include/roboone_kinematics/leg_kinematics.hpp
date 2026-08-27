// 片脚 6 自由度の順運動学 (FK) と逆運動学 (IK)。反復なしの閉形式解。
//
// 土台は docs/脚IK導出.tex。ただしリンク p3, p4 に x 成分を許すよう
// 遠位 3 関節の解き方を組み直してある（下の「x 成分を入れた導出」を参照）。
// (FK-n) / (IK-n) は文書の式番号、(X-n) はこのファイルで導き直した式。
//
// 座標系（文書 §2）
//   x : ロール軸 (J2/J4/J6) の方向。膝が x まわりに曲がるので x は機体の左右。
//   y : ピッチ軸 (J1/J5) の方向。脚の屈曲面が y-z なので y は機体の前後。
//   z : 上向き。ゼロ姿勢で脚は真下、足裏は水平、全 Σ_k は Σ_0 と同じ向き。
//
// 関節
//   θ1 股ピッチ Ry / θ2 股ロール Rx / θ3 股ヨー Rz … 3 軸は股中心 o3 で交わる (A1)
//   θ4 膝ロール Rx  θ5 足首ピッチ Ry  θ6 足首ロール Rx
//   出力は「関節角」でサーボ指令角ではない。4 節リンク・パラレルリンクの
//   変換 f4, f56 は別レイヤ（文書 §7）。
//
// 寸法は leg_config.hpp にまとめてある。CAD 確定後はそちらだけを書き換える。
//
// ===========================================================================
// x 成分を入れた導出
// ===========================================================================
// p3 = (a3, b, -ℓ3), p4 = (a4, 0, -ℓ4), p5 = (0, 0, -ℓ5)。
//
// [1] x は膝軸 (J4) の方向で Rx が動かさないので、o4 を膝軸に沿って δ 滑らせると
//     p3 は +δ、p4 は -δ し、和 a := a3 + a4 だけが不変量として残る。以後 a を使う。
//
// [2] y オフセット b は文書 §6 のとおり ℓ3' = hypot(ℓ3, b)、φ = atan2(b, ℓ3)、
//     θ4' = θ4 - φ に吸収できる。x は Rx で不変なのでこの吸収と干渉しない。
//     a := R4ᵀp3 + p4 = (a, -B, -A),  A = ℓ3'cosθ4' + ℓ4,  B = ℓ3'sinθ4'    (X-1)
//
// [3] w := R5ᵀa + p5 = (a·c5 + A·s5,  -B,  a·s5 - A·c5 - ℓ5)                (X-2)
//     r := R6ᵀw の成分は、V := a·s5 - A·c5 - ℓ5 と置いて
//       r_x = a·c5 + A·s5                                                   (X-3)
//       r_y = -B·c6 + V·s6                                                  (X-4)
//       r_z =  B·s6 + V·c6                                                  (X-5)
//     a = 0 なら V = -(A·c5 + ℓ5) で文書の (IK-2)〜(IK-4) に戻る。
//
// [4] r_y² + r_z² = B² + V²（回転で長さが保たれる）。u² + V² を展開すると
//     交差項 2aA·c5·s5 が消えて
//       |r|² = a² + A² + B² + ℓ5² + 2ℓ5(A·c5 - a·s5)
//     A² + B² = ℓ3'² - ℓ4² + 2ℓ4A（文書 (26) と同じ）を入れて
//       K := |r|² - a² - ℓ3'² + ℓ4² - ℓ5² = 2ℓ4A + 2ℓ5(A·c5 - a·s5)        (X-6)
//
// [5] ここが文書との分かれ目。a ≠ 0 だと (IK-2) の X, Y 分離が崩れ、
//     文書 §6 が言うとおり X についての 2 次方程式にはならない。
//     代わりに (X-6) の両辺に (X-3) を掛け合わせて A を消すと、A の 2 次項が
//     ちょうど打ち消し合って θ5 についての **線形式** が残る:
//       K·s5 + 2(a·ℓ4 - ℓ5·r_x)·c5 = 2(ℓ4·r_x - a·ℓ5)                      (X-7)
//     P·s5 + Q·c5 = C は atan2 ひとつで解ける（下の solveLinearTrig）。
//     掛けた因子は 2(ℓ4 + ℓ5·c5) で、ℓ4 > ℓ5 より 0 にならないので偽根は出ない。
//
// [6] θ5 が出れば A が戻る:
//       A = (K + 2ℓ5·a·s5) / (2(ℓ4 + ℓ5·c5))                               (X-8)
//     以降は文書どおり cosθ4' = (A - ℓ4)/ℓ3'、θ4 = σ·arccos(...) + φ。
//
// [7] (X-4)(X-5) を (c6, s6) について解くと
//       θ6 = atan2(V·r_y + B·r_z,  V·r_z - B·r_y)                           (X-9)
//     a = 0 で V = -(A·c5+ℓ5) を入れると文書 (IK-10) に一致する。
//
// [8] 股 3 軸は文書のまま。M := R·R456ᵀ = R123 から (IK-12)〜(IK-14)。
//
// 三角関数は atan2 が 6 回、asin と arccos が 1 回ずつ、平方根が 2 回。反復なし。
#ifndef ROBOONE_KINEMATICS__LEG_KINEMATICS_HPP_
#define ROBOONE_KINEMATICS__LEG_KINEMATICS_HPP_

#include <cmath>
#include <cstddef>

#include "roboone_kinematics/leg_config.hpp"

namespace roboone_kinematics
{

inline constexpr std::size_t kNumJoints = 6;

enum Joint : std::size_t
{
  HIP_PITCH = 0, HIP_ROLL = 1, HIP_YAW = 2, KNEE = 3, ANKLE_PITCH = 4, ANKLE_ROLL = 5
};

enum class Side { RIGHT, LEFT };

/// ik() の結果。Ok 以外でも clamp=true なら最寄り姿勢が書き戻される
/// （NoBranch を除く）。clamp=false のときは Ok 以外で theta を書かない。
enum class IkStatus
{
  Ok = 0,
  //! (X-7) を満たす θ5 が無い（|C| > |(P,Q)|）。x 方向に遠すぎる
  AnkleOutOfRange,
  //! |cosθ4| > 1。脚長に対して遠すぎる / 近すぎる
  KneeOutOfRange,
  //! A > 0 の枝が無い。足先が股中心に近すぎる。clamp でも救えない
  NoBranch,
};

// ---------------------------------------------------------------------------
// 小さな線形代数（3 次元固定。ヒープも外部ライブラリも使わない）
// ---------------------------------------------------------------------------
struct Vec3
{
  double x{0.0}, y{0.0}, z{0.0};

  constexpr double dot(const Vec3 & o) const { return x * o.x + y * o.y + z * o.z; }
  constexpr double normSq() const { return x * x + y * y + z * z; }
  constexpr Vec3 operator+(const Vec3 & o) const { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vec3 operator-(const Vec3 & o) const { return {x - o.x, y - o.y, z - o.z}; }
};

/// 行優先の 3x3。
struct Mat3
{
  double m[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  constexpr double operator()(int i, int j) const { return m[i][j]; }
  constexpr double & operator()(int i, int j) { return m[i][j]; }

  constexpr Vec3 operator*(const Vec3 & v) const
  {
    return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }

  /// 転置を掛ける（転置行列を作らずに済ませる）。
  constexpr Vec3 mulT(const Vec3 & v) const
  {
    return {m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
            m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
            m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z};
  }

  constexpr Mat3 operator*(const Mat3 & o) const
  {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
      }
    }
    return r;
  }
};

/// 回転行列（文書 式 (1)）。sin/cos を渡す版は使い回しのため。
constexpr Mat3 rotXsc(double c, double s) { return Mat3{{{1, 0, 0}, {0, c, -s}, {0, s, c}}}; }
constexpr Mat3 rotYsc(double c, double s) { return Mat3{{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}}; }
constexpr Mat3 rotZsc(double c, double s) { return Mat3{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}}; }

inline Mat3 rotX(double t) { return rotXsc(std::cos(t), std::sin(t)); }
inline Mat3 rotY(double t) { return rotYsc(std::cos(t), std::sin(t)); }
inline Mat3 rotZ(double t) { return rotZsc(std::cos(t), std::sin(t)); }

// ---------------------------------------------------------------------------
// パラメータ
// ---------------------------------------------------------------------------
/// 片脚の定数。値は leg_config.hpp から来る。make() で派生量まで埋める。
struct LegParams
{
  // ---- 入力（leg_config.hpp の値） ----
  double l3{config::L3}, l4{config::L4}, l5{config::L5}, l6{config::L6};
  double a3{config::A3_X}, a4{config::A4_X};   //!< p3, p4 の x 成分
  double b{config::B_THIGH};                   //!< p3 の y 成分
  Vec3 p0{config::HIP_X, config::HIP_Y, config::HIP_Z};
  Vec3 p6{config::P6_X, config::P6_Y, config::P6_Z};
  int sigma{config::SIGMA};
  double sign[kNumJoints]{1, 1, 1, 1, 1, 1};   //!< AXIS_FLIP を ±1 にしたもの

  // ---- 派生量（make() が埋める） ----
  Vec3 p3{}, p4{}, p5{};
  double a{0.0};      //!< a3 + a4。x はこの和だけが効く（導出 [1]）
  double l3e{0.0};    //!< 有効大腿長 ℓ3' = hypot(ℓ3, b)
  double phi{0.0};    //!< 膝角オフセット φ = atan2(b, ℓ3)

  /// 派生量を計算し直す。入力を直接いじったあとに呼ぶ。
  void finalize()
  {
    p3 = {a3, b, -l3};
    p4 = {a4, 0.0, -l4};
    p5 = {0.0, 0.0, -l5};
    a = a3 + a4;
    l3e = std::hypot(l3, b);
    phi = std::atan2(b, l3);
  }

  /// 前提条件を満たしているか（ℓ4 > ℓ5 は (X-7) の因子が 0 にならない根拠）。
  bool valid() const
  {
    return l3 > 0.0 && l4 > 0.0 && l5 >= 0.0 && l5 < l4 &&
           (sigma == 1 || sigma == -1) && l3e > 0.0;
  }
};

/// leg_config.hpp から左右脚のパラメータを組み立てる。
inline LegParams makeLegParams(Side side)
{
  LegParams prm;
  const double lat = (side == Side::RIGHT) ? 1.0 : -1.0;
  prm.p0 = {lat * config::HIP_X, config::HIP_Y, config::HIP_Z};

  const int * flip = config::AXIS_FLIP;
  if (side == Side::LEFT && config::AXIS_FLIP_LEFT_SEPARATE) {
    flip = config::AXIS_FLIP_LEFT;
  }
  for (std::size_t k = 0; k < kNumJoints; ++k) {
    prm.sign[k] = 1.0 - 2.0 * static_cast<double>(flip[k]);
  }
  prm.finalize();
  return prm;
}

// ---------------------------------------------------------------------------
// 外部の関節角 <-> 文書の符号
// ---------------------------------------------------------------------------
// 符号は ±1 なのでどちら向きの変換も同じ掛け算。幾何は文書の符号で解き、
// 境界でだけ掛け直す。in と out が同じ配列を指してもよい。
inline void applyFlip(const LegParams & prm, const double in[kNumJoints], double out[kNumJoints])
{
  for (std::size_t k = 0; k < kNumJoints; ++k) {out[k] = in[k] * prm.sign[k];}
}

// ---------------------------------------------------------------------------
// 順運動学
// ---------------------------------------------------------------------------
/// 関節角 -> 足先位置 p と足姿勢 R（いずれも Σ_0）。式 (FK-1)。
/// theta は AXIS_FLIP を適用した符号。
inline void fk(const LegParams & prm, const double theta[kNumJoints], Vec3 & p, Mat3 & R)
{
  double t[kNumJoints];
  applyFlip(prm, theta, t);

  const Mat3 R1 = rotY(t[HIP_PITCH]);
  const Mat3 R2 = rotX(t[HIP_ROLL]);
  const Mat3 R3 = rotZ(t[HIP_YAW]);
  const Mat3 R4 = rotX(t[KNEE]);
  const Mat3 R5 = rotY(t[ANKLE_PITCH]);
  const Mat3 R6 = rotX(t[ANKLE_ROLL]);

  const Mat3 R123 = R1 * R2 * R3;               // (FK-3)
  const Mat3 R456 = R4 * R5 * R6;               // (FK-5)

  // 股中心 o3 -> 足先 を Σ_3 で見たベクトル（内側から展開）
  const Vec3 q = prm.p3 + R4 * (prm.p4 + R5 * (prm.p5 + R6 * prm.p6));

  p = prm.p0 + R123 * q;                        // (FK-4)
  R = R123 * R456;
}

/// 各関節の回転中心 [o3, o4, o5, o6, 足先] を Σ_0 で返す。描画・検証用。
inline void jointOrigins(const LegParams & prm, const double theta[kNumJoints], Vec3 out[5])
{
  double t[kNumJoints];
  applyFlip(prm, theta, t);

  const Mat3 R123 = rotY(t[HIP_PITCH]) * rotX(t[HIP_ROLL]) * rotZ(t[HIP_YAW]);
  const Mat3 R1234 = R123 * rotX(t[KNEE]);
  const Mat3 R12345 = R1234 * rotY(t[ANKLE_PITCH]);
  const Mat3 R = R12345 * rotX(t[ANKLE_ROLL]);

  out[0] = prm.p0;
  out[1] = out[0] + R123 * prm.p3;
  out[2] = out[1] + R1234 * prm.p4;
  out[3] = out[2] + R12345 * prm.p5;
  out[4] = out[3] + R * prm.p6;
}

// ---------------------------------------------------------------------------
// 逆運動学
// ---------------------------------------------------------------------------
namespace detail
{

inline double clampUnit(double v) { return v > 1.0 ? 1.0 : (v < -1.0 ? -1.0 : v); }

/// P·sinθ + Q·cosθ = C を解く。根は 2 つ。
/// Rm = hypot(P,Q)、ψ = atan2(Q,P) とすると P·s + Q·c = Rm·sin(θ+ψ) なので
///   θ = asin(C/Rm) - ψ   と   θ = π - asin(C/Rm) - ψ
/// |C| > Rm のときは解が無い（clamp すれば最寄り）。
struct LinearTrigRoots
{
  double t[2]{0.0, 0.0};
  bool exact{false};   //!< |C| <= Rm だったか
  bool ok{false};      //!< Rm > 0 だったか
};

inline LinearTrigRoots solveLinearTrig(double P, double Q, double C)
{
  LinearTrigRoots r;
  const double Rm = std::hypot(P, Q);
  if (Rm < 1e-12) {return r;}          // P = Q = 0。θ に依存しない退化
  r.ok = true;
  const double ratio = C / Rm;
  r.exact = std::fabs(ratio) <= 1.0;
  const double beta = std::asin(clampUnit(ratio));
  const double psi = std::atan2(Q, P);
  r.t[0] = beta - psi;
  r.t[1] = M_PI - beta - psi;
  return r;
}

}  // namespace detail

/// (足先位置 p, 足姿勢 R) -> 関節角 θ1..θ6。反復なしの閉形式解。
///
/// theta   出力。AXIS_FLIP を適用した符号（fk が受け取るのと同じ）。
/// clamp   true なら到達不能を最寄り姿勢に丸めて書き戻す。false なら theta は触らない。
///
/// 戻り値が Ok 以外でも、NoBranch 以外なら clamp=true で theta は埋まる。
inline IkStatus ik(
  const LegParams & prm, const Vec3 & p, const Mat3 & R,
  double theta[kNumJoints], bool clamp = true)
{
  const double l3 = prm.l3e;      // 有効大腿長（y オフセット吸収後）
  const double l4 = prm.l4;
  const double l5 = prm.l5;
  const double a = prm.a;         // x オフセットの和

  // 1. 股中心 -> 足首ロール軸 を Σ_6 で見たベクトル (IK-1)
  const Vec3 r = R.mulT(p - prm.p0) - prm.p6;

  // 2. (X-6)
  const double K = r.normSq() - a * a - l3 * l3 + l4 * l4 - l5 * l5;

  // 3. θ5 についての線形式 (X-7)
  const auto roots = detail::solveLinearTrig(
    K, 2.0 * (a * l4 - l5 * r.x), 2.0 * (l4 * r.x - a * l5));
  if (!roots.ok) {return IkStatus::NoBranch;}
  if (!roots.exact && !clamp) {return IkStatus::AnkleOutOfRange;}

  // 4. 2 つの根から枝を選ぶ。A > 0 を満たすもののうち cosθ5 が大きい方
  //    （足首ピッチが 0 に近い姿勢）を採る。文書 §5.6 の「+ 根 = cosθ5 > 0」の一般化。
  bool found = false;
  bool kneeOver = false;
  double bestT5 = 0.0, bestC5 = -2.0, bestS5 = 0.0, bestA = 0.0, bestC4 = 0.0;
  for (int i = 0; i < 2; ++i) {
    const double t5 = roots.t[i];
    const double c5 = std::cos(t5), s5 = std::sin(t5);

    // (X-8)。分母 2(ℓ4 + ℓ5·c5) は ℓ4 > ℓ5 なので 0 にならない
    const double A = (K + 2.0 * l5 * a * s5) / (2.0 * (l4 + l5 * c5));
    if (A <= 0.0) {continue;}                    // 文書 §5.6「A > 0 の仮定」

    const double c4raw = (A - l4) / l3;
    const bool over = std::fabs(c4raw) > 1.0;

    // 膝が範囲内の枝を優先し、同条件なら cosθ5 が大きい方
    // （足首ピッチが 0 に近い姿勢。文書 §5.6「+ 根 = cosθ5 > 0」の一般化）
    const bool better = !found || (kneeOver && !over) ||
      (over == kneeOver && c5 > bestC5);
    if (better) {
      found = true;
      kneeOver = over;
      bestT5 = t5; bestC5 = c5; bestS5 = s5; bestA = A;
      bestC4 = detail::clampUnit(c4raw);
    }
  }
  // A > 0 の枝が無いのは「近すぎる」で、丸めても意味のある姿勢にならない
  if (!found) {return IkStatus::NoBranch;}
  if (kneeOver && !clamp) {return IkStatus::KneeOutOfRange;}

  // 5. 膝 (IK-9)。arccos の引数は数値ノイズ対策で [-1, 1] に丸め済み
  const double t4e = static_cast<double>(prm.sigma) * std::acos(bestC4);
  const double B = l3 * std::sin(t4e);
  const double t4 = t4e + prm.phi;               // 本来の θ4（文書 §6）

  // 6. 足首ロール (X-9)
  const double V = a * bestS5 - bestA * bestC5 - l5;
  const double t6 = std::atan2(V * r.y + B * r.z, V * r.z - B * r.y);

  // 7. 残った回転から股 3 軸 (IK-11)〜(IK-14)
  const Mat3 M = R * rotXsc(std::cos(t6), -std::sin(t6)) *
    rotYsc(bestC5, -bestS5) * rotXsc(std::cos(t4), -std::sin(t4));

  const double c2 = std::hypot(M(1, 0), M(1, 1));
  const double t2 = std::atan2(-M(1, 2), c2);
  const double t3 = std::atan2(M(1, 0), M(1, 1));
  const double t1 = std::atan2(M(0, 2), M(2, 2));

  const double sol[kNumJoints] = {t1, t2, t3, t4, bestT5, t6};
  applyFlip(prm, sol, theta);                    // 文書の符号 -> 外部の符号

  if (!roots.exact) {return IkStatus::AnkleOutOfRange;}
  return kneeOver ? IkStatus::KneeOutOfRange : IkStatus::Ok;
}

}  // namespace roboone_kinematics

#endif  // ROBOONE_KINEMATICS__LEG_KINEMATICS_HPP_
