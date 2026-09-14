// 片脚 6 自由度の順運動学 (FK) と逆運動学 (IK)。反復なしの閉形式解。
//
// 座標系は **機体座標 Σ_B ひとつだけ**
//   x = 前, y = 左, z = 上   … walk_core / REP-103 と同じ取り方
//   原点はボディ原点。ゼロ姿勢で脚は真下、足裏は水平、全 Σ_k は Σ_B と同じ向き。
//   公開 API も解析解も leg_config.hpp の値も、全部この座標系で書いてある。
//   （以前は docs/脚IK導出.tex の Σ_S = x 右 / y 前 で解いて外側で読み替えていたが、
//     読みづらいので 2026-09-14 に Σ_B へ書き直した。文書の (FK-n)/(IK-n) とは
//     軸の名前が違うので、突き合わせるときは x_S = -y_B、y_S = x_B と読むこと。）
//
// 関節（括弧内は Σ_B での回転軸。実機で確認した並び）
//   θ1 股ピッチ (Ry) / θ2 股ロール (Rx) / θ3 股ヨー (Rz)  … 3 軸は股中心 o3 で交わる
//   θ4 膝 (Ry)
//   θ5 足首ピッチ (Ry)  上側ピボット（腰に近い方）。膝と平行
//   θ6 足首ロール (Rx)  下側ピボット（足に近い方）
//
//   R = Ry(θ1)·Rx(θ2)·Rz(θ3) · Ry(θ4) · Ry(θ5) · Rx(θ6)
//
//   膝は θ4 > 0 で足先が後ろへ振れる（人型の曲げ。KNEE_FORWARD = +1）。
//   出力は「関節角」でサーボ指令角ではない。4 節リンク・パラレルリンクの
//   変換は別レイヤ（leg_servo.hpp）。
//
// 寸法は leg_config.hpp にまとめてある。CAD 確定後はそちらだけを書き換える。
//
// ===========================================================================
// 逆運動学の導出（式番号 (B-n)。leg_selftest の checkClosedForms が成分式を照合する）
// ===========================================================================
// p3 = (b, a3, -ℓ3), p4 = (0, a4, -ℓ4), p5 = (0, 0, -ℓ5)。
//
// [1] y は膝軸 (J4) の方向で Ry が動かさないので、o4 を膝軸に沿って δ 滑らせると
//     p3 は +δ、p4 は -δ し、和 a := a3 + a4 だけが不変量として残る。以後 a を使う。
//
// [2] 前後オフセット b は ℓ3' = hypot(ℓ3, b)、φ = atan2(b, ℓ3)、θ4' = θ4 + φ に
//     吸収できる。y は Ry で不変なのでこの吸収と干渉しない。
//       u := R4ᵀp3 + p4 = (B, a, -A),  A = ℓ3'cosθ4' + ℓ4,  B = ℓ3'sinθ4'    (B-1)
//
// [3] 膝と足首ピッチは同じ軸なので、u を θ5 だけ回して p5 を足す:
//       w := R5ᵀu + p5 = (B·c5 + A·s5,  a,  B·s5 - A·c5 - ℓ5)                (B-2)
//     足首ロールは x を動かさず、y は定数 a のままなので、r := R6ᵀw は
//       r_x = B·c5 + A·s5                                                   (B-3)
//       r_y =  a·c6 + V·s6      V := B·s5 - A·c5 - ℓ5（= w_z）              (B-4)
//       r_z = -a·s6 + V·c6                                                  (B-5)
//     r は「股中心 -> 足首ロール軸 o6」を足板 Σ_6 で見たベクトルで、目標 (p, R) から
//       r = Rᵀ(p - p0) - p6                                                 (B-6)
//     と直接求まる。
//
// [4] (B-4)(B-5) を c6, s6 で組み合わせると V が消えて θ6 だけの式になる:
//       r_y·c6 - r_z·s6 = a                                                 (B-7)
//     ρ := hypot(r_y, r_z)、ψ := atan2(r_z, r_y) と置けば ρ·cos(θ6 + ψ) = a なので
//       θ6 = ±arccos(a/ρ) - ψ                                               (B-8)
//     根は 2 つ。|a| > ρ なら解が無い（足首が膝軸方向のオフセットより股に近い）。
//     **足首ロールが他の関節に先立って閉じる**のがこの導出の要で、a = 0 の実機では
//     θ6 = atan2(-r_y, -r_z)（V < 0 の枝）に落ちる。
//
// [5] θ6 が決まると w が戻る:
//       w_x = r_x,   w_z = V = r_y·s6 + r_z·c6                              (B-9)
//     |u|² = A² + B² + a² = ℓ3'² - ℓ4² + 2ℓ4·A + a² と
//     |u|² = |w - p5|² = w_x² + a² + (w_z + ℓ5)² から
//       A = (w_x² + (w_z + ℓ5)² - ℓ3'² + ℓ4²) / (2ℓ4)                        (B-10)
//     以降 cosθ4' = (A - ℓ4)/ℓ3'、θ4' = σ·arccos(...)、B = ℓ3'sinθ4'、θ4 = θ4' - φ。
//
// [6] 足首ピッチは (B-2) の x-z 成分が (B, -A) を θ5 回したものなので
//       θ5 = atan2(w_z + ℓ5, w_x) - atan2(-A, B)                            (B-11)
//
// [7] 股 3 軸は M := R·R6ᵀ·R5ᵀ·R4ᵀ = Ry(θ1)·Rx(θ2)·Rz(θ3) から取り出す
//     （y-x-z 順のオイラー角。ik() の該当箇所を参照）。
//
// (B-8) の 2 根は V の符号が逆で、a = 0 なら θ6 がちょうど π 違う（足板を下腿に
// 対して裏返した組み方で、股ヨーが 180° 近く回る）。A > 0 の根のうち |θ6| が小さい
// 方を採る。実機は a = 0 なので解は一意で、可動域内で根が入れ替わることはない。
// ★a ≠ 0 だと 2 根の θ6 の差は 2·arccos(a/ρ) < π になり、両方が可動域内に落ちる
//   姿勢が出る（IK の解が一意でない）。そのときも同じ足先姿勢は与える。
//
// 三角関数は atan2 が 5 回、arccos が 2 回、平方根が 2 回。反復なし。
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
  //! (B-7) を満たす θ6 が無い（|a| > ρ）。足首が膝軸方向のオフセット a より股中心に近い
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
  double norm() const { return std::sqrt(normSq()); }
  constexpr Vec3 cross(const Vec3 & o) const
  {
    return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
  }
  constexpr Vec3 operator+(const Vec3 & o) const { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vec3 operator-(const Vec3 & o) const { return {x - o.x, y - o.y, z - o.z}; }
  constexpr Vec3 operator*(double k) const { return {x * k, y * k, z * k}; }
  constexpr Vec3 operator-() const { return {-x, -y, -z}; }
};

constexpr Vec3 operator*(double k, const Vec3 & v) { return v * k; }

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

/// 回転行列。sin/cos を渡す版は使い回しのため。
constexpr Mat3 rotXsc(double c, double s) { return Mat3{{{1, 0, 0}, {0, c, -s}, {0, s, c}}}; }
constexpr Mat3 rotYsc(double c, double s) { return Mat3{{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}}; }
constexpr Mat3 rotZsc(double c, double s) { return Mat3{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}}; }

inline Mat3 rotX(double t) { return rotXsc(std::cos(t), std::sin(t)); }
inline Mat3 rotY(double t) { return rotYsc(std::cos(t), std::sin(t)); }
inline Mat3 rotZ(double t) { return rotZsc(std::cos(t), std::sin(t)); }

// ---------------------------------------------------------------------------
// パラメータ
// ---------------------------------------------------------------------------
/// 片脚の定数。**すべて Σ_B の成分**。leg_config.hpp の値がそのまま入る。
struct LegParams
{
  // ---- 入力 ----
  double l3{config::L3}, l4{config::L4}, l5{config::L5}, l6{config::L6};
  //! p3, p4 の y 成分（= 膝軸方向）。和 a だけが幾何に効く（導出 [1]）
  double a3{config::P3_Y}, a4{config::P4_Y};
  double b{config::P3_X};                      //!< p3 の x 成分（= 前後）
  Vec3 p0{config::HIP_X, config::HIP_Y, config::HIP_Z};
  Vec3 p6{config::P6_X, config::P6_Y, config::P6_Z};
  //! 膝の分岐 σ。+1 で θ4 > 0 の側 = 人型（KNEE_FORWARD）
  int sigma{config::KNEE_FORWARD};
  double sign[kNumJoints]{1, 1, 1, 1, 1, 1};   //!< AXIS_FLIP を ±1 にしたもの

  // ---- 派生量（finalize() が埋める） ----
  Vec3 p3{}, p4{}, p5{};
  double a{0.0};      //!< a3 + a4
  double l3e{0.0};    //!< 有効大腿長 ℓ3' = hypot(ℓ3, b)
  double phi{0.0};    //!< 膝角オフセット φ = atan2(b, ℓ3)。θ4' = θ4 + φ

  /// 派生量を計算し直す。入力を直接いじったあとに呼ぶ。
  void finalize()
  {
    p3 = {b, a3, -l3};
    p4 = {0.0, a4, -l4};
    p5 = {0.0, 0.0, -l5};
    a = a3 + a4;
    l3e = std::hypot(l3, b);
    phi = std::atan2(b, l3);
  }

  /// 前提条件を満たしているか。
  bool valid() const
  {
    return l3 > 0.0 && l4 > 0.0 && l5 >= 0.0 && (sigma == 1 || sigma == -1) && l3e > 0.0;
  }
};

/// leg_config.hpp から左右脚のパラメータを組み立てる。
///
/// leg_config.hpp の値は右脚の Σ_B 成分。左右対称なので左脚は y を反転する。
/// b（前後）と σ は左右で同じ。関節角の定義は左右とも Σ_B 共通なので、鏡像に
/// なるのはサーボの回り方だけで、それは AXIS_FLIP_LEFT が受け持つ。
inline LegParams makeLegParams(Side side)
{
  LegParams prm;
  const double lat = (side == Side::RIGHT) ? 1.0 : -1.0;
  prm.a3 *= lat;
  prm.a4 *= lat;
  prm.p0.y *= lat;
  prm.p6.y *= lat;

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
// 関節角の符号
// ---------------------------------------------------------------------------
// 公開角 = 内部角 × sign（AXIS_FLIP）。掛けるのは ±1 だけなので、どちら向きの
// 変換も同じ式でよい。in と out が同じ配列を指してもよい。
// 内部角は「Σ_B の正軸まわりの右ねじが正」。解析解はこの符号で書いてある。

/// AXIS_FLIP の掛け直し（公開角 <-> 内部角）。
inline void applyFlip(const LegParams & prm, const double in[kNumJoints], double out[kNumJoints])
{
  for (std::size_t k = 0; k < kNumJoints; ++k) {out[k] = in[k] * prm.sign[k];}
}

// ---------------------------------------------------------------------------
// 解析解の層（内部角。AXIS_FLIP はこの外側で掛ける）
// ---------------------------------------------------------------------------
namespace solver
{

/// 関節角（内部角）-> 足先位置 p と足姿勢 R（Σ_B）。
inline void fk(const LegParams & prm, const double t[kNumJoints], Vec3 & p, Mat3 & R)
{
  const Mat3 R1 = rotY(t[HIP_PITCH]);
  const Mat3 R2 = rotX(t[HIP_ROLL]);
  const Mat3 R3 = rotZ(t[HIP_YAW]);
  const Mat3 R4 = rotY(t[KNEE]);
  const Mat3 R5 = rotY(t[ANKLE_PITCH]);
  const Mat3 R6 = rotX(t[ANKLE_ROLL]);

  const Mat3 R123 = R1 * R2 * R3;
  const Mat3 R456 = R4 * R5 * R6;

  // 股中心 o3 -> 足先 を Σ_3 で見たベクトル（内側から展開）
  const Vec3 q = prm.p3 + R4 * (prm.p4 + R5 * (prm.p5 + R6 * prm.p6));

  p = prm.p0 + R123 * q;
  R = R123 * R456;
}

/// 各関節の回転中心 [o3, o4, o5, o6, 足先] を返す。
inline void jointOrigins(const LegParams & prm, const double t[kNumJoints], Vec3 out[5])
{
  const Mat3 R123 = rotY(t[HIP_PITCH]) * rotX(t[HIP_ROLL]) * rotZ(t[HIP_YAW]);
  const Mat3 R1234 = R123 * rotY(t[KNEE]);
  const Mat3 R12345 = R1234 * rotY(t[ANKLE_PITCH]);
  const Mat3 R = R12345 * rotX(t[ANKLE_ROLL]);

  out[0] = prm.p0;
  out[1] = out[0] + R123 * prm.p3;
  out[2] = out[1] + R1234 * prm.p4;
  out[3] = out[2] + R12345 * prm.p5;
  out[4] = out[3] + R * prm.p6;
}

/// 各リンクの姿勢 [Σ_3, Σ_4, Σ_5, Σ_6] を返す。
inline void jointFrames(const LegParams & prm, const double t[kNumJoints], Mat3 out[4])
{
  (void)prm;
  out[0] = rotY(t[HIP_PITCH]) * rotX(t[HIP_ROLL]) * rotZ(t[HIP_YAW]);
  out[1] = out[0] * rotY(t[KNEE]);
  out[2] = out[1] * rotY(t[ANKLE_PITCH]);
  out[3] = out[2] * rotX(t[ANKLE_ROLL]);
}

}  // namespace solver

// ---------------------------------------------------------------------------
// 逆運動学
// ---------------------------------------------------------------------------
namespace detail
{

inline double clampUnit(double v) { return v > 1.0 ? 1.0 : (v < -1.0 ? -1.0 : v); }
inline double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace detail

namespace solver
{

/// (足先位置 p, 足姿勢 R) -> 関節角 θ1..θ6（内部角）。反復なしの閉形式解。
/// 契約は公開版の ik() と同じ。
inline IkStatus ik(
  const LegParams & prm, const Vec3 & p, const Mat3 & R,
  double t[kNumJoints], bool clamp = true)
{
  const double l3 = prm.l3e;      // 有効大腿長（前後オフセット吸収後）
  const double l4 = prm.l4;
  const double l5 = prm.l5;
  const double a = prm.a;         // 膝軸方向のオフセットの和

  // 1. 股中心 -> 足首ロール軸 o6 を足板 Σ_6 で見たベクトル (B-6)
  const Vec3 r = R.mulT(p - prm.p0) - prm.p6;

  // 2. 足首ロール (B-7)(B-8)。ρ cos(θ6 + ψ) = a
  const double rho = std::hypot(r.y, r.z);
  if (rho < 1e-12) {return IkStatus::NoBranch;}     // y-z で股の真横。θ6 が決まらない
  const double ratio = a / rho;
  const bool exact = std::fabs(ratio) <= 1.0;
  if (!exact && !clamp) {return IkStatus::AnkleOutOfRange;}
  const double gamma = std::acos(detail::clampUnit(ratio));
  const double psi = std::atan2(r.z, r.y);

  // 3. 2 つの根から枝を選ぶ。2 根は θ6 がほぼ π 違う（足板を下腿に対して裏返した
  //    組み方）ので、A > 0 のうち **|θ6| が小さい方** を採る。足首ロールが ±90° を
  //    超える姿勢は機体には無い。膝が可動域を超えているかは選んだ根で判定する
  //    （裏返した根が「たまたま膝は届く」からといってそちらへ飛ばない）。
  bool found = false;
  bool kneeOver = false;
  double bestT6 = 0.0, bestT5 = 0.0, bestAbsT6 = 1e9, bestC4 = 0.0;
  for (int k = -1; k <= 1; k += 2) {
    const double t6 = detail::wrapPi(static_cast<double>(k) * gamma - psi);
    const double c6 = std::cos(t6), s6 = std::sin(t6);

    // (B-9)(B-10)
    const double wx = r.x;
    const double wz = r.y * s6 + r.z * c6;
    const double A = (wx * wx + (wz + l5) * (wz + l5) - l3 * l3 + l4 * l4) / (2.0 * l4);
    if (A <= 0.0) {continue;}                    // 膝の三角形が閉じない側
    if (found && std::fabs(t6) >= bestAbsT6) {continue;}

    const double c4raw = (A - l4) / l3;
    const bool over = std::fabs(c4raw) > 1.0 + 1e-12;
    const double c4 = detail::clampUnit(c4raw);
    const double B = l3 * std::sin(static_cast<double>(prm.sigma) * std::acos(c4));

    // (B-11)
    const double t5 = detail::wrapPi(std::atan2(wz + l5, wx) - std::atan2(-A, B));

    found = true;
    kneeOver = over;
    bestT6 = t6; bestT5 = t5; bestAbsT6 = std::fabs(t6); bestC4 = c4;
  }
  // A > 0 の枝が無いのは「近すぎる」で、丸めても意味のある姿勢にならない
  if (!found) {return IkStatus::NoBranch;}
  if (kneeOver && !clamp) {return IkStatus::KneeOutOfRange;}

  // 4. 膝。arccos の引数は数値ノイズ対策で [-1, 1] に丸め済み
  const double t4e = static_cast<double>(prm.sigma) * std::acos(bestC4);
  const double t4 = t4e - prm.phi;               // 本来の θ4（導出 [2]）

  // 5. 残った回転から股 3 軸 (B-7 の後段)
  const Mat3 M = R * rotXsc(std::cos(bestT6), -std::sin(bestT6)) *
    rotYsc(std::cos(bestT5), -std::sin(bestT5)) * rotYsc(std::cos(t4), -std::sin(t4));

  // M = Ry(θ1)·Rx(θ2)·Rz(θ3) からの取り出し（y-x-z 順のオイラー角）。
  //   M(1,·) = (c2s3, c2c3, -s2) / M(0,2) = s1c2 / M(2,2) = c1c2
  const double c2 = std::hypot(M(1, 0), M(1, 1));
  const double t2 = std::atan2(-M(1, 2), c2);
  const double t3 = std::atan2(M(1, 0), M(1, 1));
  const double t1 = std::atan2(M(0, 2), M(2, 2));

  const double sol[kNumJoints] = {t1, t2, t3, t4, bestT5, bestT6};
  for (std::size_t k = 0; k < kNumJoints; ++k) {t[k] = sol[k];}

  if (!exact) {return IkStatus::AnkleOutOfRange;}
  return kneeOver ? IkStatus::KneeOutOfRange : IkStatus::Ok;
}

}  // namespace solver

// ---------------------------------------------------------------------------
// 公開 API（すべて機体座標 Σ_B、AXIS_FLIP 適用後の公開角）
// ---------------------------------------------------------------------------
/// 関節角 -> 足先位置 p と足姿勢 R。
/// theta は Σ_B の関節角で、AXIS_FLIP を適用した符号。
inline void fk(const LegParams & prm, const double theta[kNumJoints], Vec3 & p, Mat3 & R)
{
  double t[kNumJoints];
  applyFlip(prm, theta, t);
  solver::fk(prm, t, p, R);
}

/// 各関節の回転中心 [o3, o4, o5, o6, 足先] を返す。描画・検証用。
inline void jointOrigins(const LegParams & prm, const double theta[kNumJoints], Vec3 out[5])
{
  double t[kNumJoints];
  applyFlip(prm, theta, t);
  solver::jointOrigins(prm, t, out);
}

/// 各リンクの姿勢 [Σ_3, Σ_4, Σ_5, Σ_6] を返す。
///
/// jointOrigins() が「節点がどこか」を返すのに対し、こちらは「そこに載っている
/// リンクがどちらを向いているか」を返す。並びは
///
///   out[0] = Σ_3  股 3 軸の後（大腿に固定）
///   out[1] = Σ_4  膝の後（下腿に固定）。足首パラレルの Σ_s と同じ向き
///   out[2] = Σ_5  足首ピッチ θ5 の後
///   out[3] = Σ_6  足板。fk() が返す R と同じ
///
/// 4 節リンクやパラレルリンクの点（Σ_s / Σ_6 の定数）を Σ_B に置くために要る。
/// 例: 足首のクランク軸 O_i は o5 + out[1] · prm.ankle.c[i]。
inline void jointFrames(const LegParams & prm, const double theta[kNumJoints], Mat3 out[4])
{
  double t[kNumJoints];
  applyFlip(prm, theta, t);
  solver::jointFrames(prm, t, out);
}

/// (足先位置 p, 足姿勢 R) -> 関節角 θ1..θ6。反復なしの閉形式解。
///
/// p, R    Σ_B（x 前・y 左・z 上、原点はボディ原点）。
/// theta   出力。Σ_B の関節角で、AXIS_FLIP を適用した符号（fk と同じ）。
/// clamp   true なら到達不能を最寄り姿勢に丸めて書き戻す。false なら theta は触らない。
///
/// 戻り値が Ok 以外でも、NoBranch 以外なら clamp=true で theta は埋まる。
inline IkStatus ik(
  const LegParams & prm, const Vec3 & p, const Mat3 & R,
  double theta[kNumJoints], bool clamp = true)
{
  double t[kNumJoints];
  const IkStatus st = solver::ik(prm, p, R, t, clamp);
  // solver::ik が t を書かなかったケース（契約は上の doc コメントのとおり）
  if (st == IkStatus::NoBranch || (!clamp && st != IkStatus::Ok)) {return st;}
  applyFlip(prm, t, theta);                      // 内部角 -> 公開角
  return st;
}

}  // namespace roboone_kinematics

#endif  // ROBOONE_KINEMATICS__LEG_KINEMATICS_HPP_
