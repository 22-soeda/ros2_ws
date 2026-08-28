// 足首パラレルリンクの順変換 / 逆変換。
//
// 出典: 「足首パラレルリンク：関節角 ↔ サーボ角 変換の実装仕様」。
// 式番号 §n はその仕様の節番号。寸法は ankle_config.hpp にまとめてある。
//
// leg_kinematics.hpp が出す関節角 θ5・θ6 と、実機の 2 個のサーボのクランク角
// q1・q2 を相互に変換する層。leg_servo.hpp では
//
//   指令側: 足先目標 -> ik() -> (θ5,θ6) -> ankleIk()  -> (q1,q2) -> to servo
//   観測側: サーボ実測 -> (q1,q2) -> ankleFk() -> (θ5,θ6) -> fk() -> 重心位置
//
// の順に呼ぶ。ヘッダオンリ・ROS 非依存・ヒープ確保なしで、200Hz の制御ループから
// 直接呼べる。**最悪実行時間が姿勢によらず一定であること**を優先している。
//
// ---------------------------------------------------------------------------
// 機構（仕様 §2）
// ---------------------------------------------------------------------------
// 下腿の左右に 1 個ずつサーボが付く（取付高さが違う: 下 73mm / 上 108mm）。
// 各サーボはクランク（半径 18mm）を回し、両端ボールジョイントのロッドで足板の
// ボールを押し引きする。足と下腿は中央のユニバーサルジョイント（θ5, θ6）だけで
// 繋がり、ロッドは長さ拘束しか与えない。
//
//   R5 = Rx(θ5)  足首ロール（上側ピボット）
//   R6 = Ry(θ6)  足首ピッチ（下側ピボット）
//   o5 -> o6 = (0, 0, -ℓ5)
//
// ★θ5 がロール、θ6 がピッチ。サーボ名の「足首ピッチ / 足首ロール」とは逆
//   （ankle_config.hpp の冒頭を読むこと）。
//
// ---------------------------------------------------------------------------
// 解き方
// ---------------------------------------------------------------------------
// 逆変換（仕様 §4）は**閉形式・反復なし**。1 鎖あたり平方根 1 回と atan2 1 回。
//   クランク先端は「O_i 中心・半径 r_i1 の円（y=0 平面内）」と
//   「足側ボール中心・半径 r_i2 の球」の交点。球はクランク平面を円で切るので、
//   平面内の 2 円の交わりに落ちる。
//
// 順変換（仕様 §5）は閉形式にならない（消去すると tan(θ6/2) の 8 次式）。
//   鎖 1 本ぶんの拘束を θ5 について解いた曲線 Θ_i(θ6) を 2 本作り、
//   Φ(θ6) = Θ1(θ6) - Θ2(θ6) = 0 を **1 変数**で解く。
//   枝 β_i を固定してあるので Φ の零点に「別の組み方の姿勢」が現れない。
//   2 変数ニュートンや 8 次式は全部の組み方を等しく見るので選別が要る。
//   Φ' が消えるのは det Jθ = 0（型 2 特異点）のときだけで、ソルバが止まる条件と
//   機構が破綻する条件が一致している。これがこの形にする理由。
//
// ---------------------------------------------------------------------------
// 姿勢が暴れないための約束（2026-08-28）
// ---------------------------------------------------------------------------
// 素のニュートンだと、足裏を前後に傾けすぎたとき（θ6 が -65° の型 2 特異点に
// 近づいたとき）Φ' → 0 で発散し、遠くの根へ飛んで姿勢が跳ねていた。
// 特異点そのものは機構の性質なので消せない。消せるのは「跳ねること」のほうで、
// 次の 3 つで押さえてある。
//
//   [1] 解く範囲を窓 FK_WINDOW_DEG = ±55° に閉じる。**この窓の中では Φ が θ6 に
//       ついて狭義単調**であることを、クランク角 ±90°（サーボリミットより広い）
//       の全域で確認してある。根は高々 1 個。
//   [2] 窓の両端の符号で根を挟み、二分法で守ったニュートンで詰める。ニュートンが
//       飛んでもブラケットの外へは出られない ⇒ 出力は必ず窓の中（有界）。
//   [3] 根が窓の外に出たら窓の縁を返す（AnkleFkStatus::Clamped）。出ていく瞬間は
//       Φ(縁) = 0 なので、Ok から Clamped へ連続に移る ⇒ 跳ばない。
//
// おまけに、根が 1 個しかないので**種（前周期の θ6）が答えを選ばなくなった**。
// 起動直後の粗探し（旧 ankleFkScan）も、種の違いで別の姿勢に落ちることも無い。
// 指令側は ankleClampJoints() が同じ窓で θ6 を丸めるので、そもそも特異点へ
// 向かう指令が出ない。
#ifndef ROBOONE_KINEMATICS__ANKLE_PARALLEL_HPP_
#define ROBOONE_KINEMATICS__ANKLE_PARALLEL_HPP_

#include <cmath>
#include <cstddef>

#include "roboone_kinematics/ankle_config.hpp"
#include "roboone_kinematics/leg_kinematics.hpp"

namespace roboone_kinematics
{

inline constexpr int kAnkleChains = 2;

/// ankleIk() の結果。
enum class AnkleIkStatus
{
  Ok = 0,
  //! Δ < 0。ロッドが届かない（可動域の外）。clamp=true なら最寄りが入る
  Unreachable,
  //! ρ ≈ 0。ボールがクランク軸上に来ていて q_i が決まらない（設計で避ける）
  Degenerate,
};

/// ankleFk() の結果。
///
/// **Ok と Clamped はどちらも「使える姿勢」**。Clamped は解が窓の外にあったので
/// 窓の縁を返したという意味で、値は連続かつ有界。暴れない。
/// 残り 3 つは窓の中では起きないはず（寸法を差し替えたときの保険）。
enum class AnkleFkStatus
{
  Ok = 0,
  //! θ6 の解が窓 FK_WINDOW_DEG の外。縁に張り付けて返した（型 2 特異点の手前で止める）
  Clamped,
  //! |W'| > 2√(U'²+V'²)。窓の端まで詰め寄っても曲線 Θ_i が存在しなかった
  NoCurve,
  //! 反復上限に達した。ブラケットは保たれているので値は窓の中にある
  NotConverged,
  //! Φ' ≈ 0。det Jθ = 0 の型 2 特異点（窓の中では起きない）
  Singular,
};

// ---------------------------------------------------------------------------
// パラメータ
// ---------------------------------------------------------------------------
/// 足首パラレルリンクの定数。値は ankle_config.hpp から来る。
/// 添え字 i = 0,1 が仕様の鎖 i = 1,2。
struct AnkleParams
{
  // ---- 入力 ----
  Vec3 a0[kAnkleChains]{};       //!< a_i0  クランク軸 O_i -> o5            (Σ_s)
  Vec3 b[kAnkleChains]{};        //!< b_i   足側ボール中心                  (Σ_6)
  double r[kAnkleChains]{};      //!< r_i1  クランク半径
  double rod[kAnkleChains]{};    //!< r_i2  ロッド長（ボール中心間）
  double l5{config::L5};         //!< ℓ5    o5 -> o6 の高さオフセット

  double servoHome[kAnkleChains]{};  //!< T ポーズでのサーボ値 [rad]（§4.5）
  double servoSign[kAnkleChains]{};  //!< σ_i
  double gear[kAnkleChains]{};       //!< n_i

  int eps[kAnkleChains]{};       //!< ε_i  逆変換の枝（§4.4）
  int del[kAnkleChains]{};       //!< β_i  順変換の枝（§5.1）

  // ---- 派生量（finalize() が埋める） ----
  Vec3 c[kAnkleChains]{};        //!< C_i = -a_i0  クランク円の中心         (Σ_s)
  Vec3 p5{};                     //!< a_i1 = (0, 0, -ℓ5)
  double a0Sq[kAnkleChains]{};   //!< |a_i0|²
  double rSq[kAnkleChains]{};    //!< r_i1²
  double rodSq[kAnkleChains]{};  //!< r_i2²
  //! q_i,neutral。中立姿勢 (θ5,θ6)=(0,0) でのクランク角。T ポーズ原点の基準（§4.5）
  double qNeutral[kAnkleChains]{};
  double qMin[kAnkleChains]{};   //!< クランク角の下限 [rad]（ankle_config::CRANK_LIMIT_DEG）
  double qMax[kAnkleChains]{};   //!< クランク角の上限 [rad]

  void finalize();

  /// 式に効く前提条件を満たしているか。
  bool valid() const
  {
    for (int i = 0; i < kAnkleChains; ++i) {
      if (!(r[i] > 0.0) || !(rod[i] > 0.0)) {return false;}
      if (eps[i] != 1 && eps[i] != -1) {return false;}
      if (del[i] != 1 && del[i] != -1) {return false;}
      if (!(gear[i] != 0.0) || std::fabs(std::fabs(servoSign[i]) - 1.0) > 1e-12) {
        return false;
      }
    }
    return l5 >= 0.0;
  }
};

// ---------------------------------------------------------------------------
// 幾何の素片
// ---------------------------------------------------------------------------
/// Σ_s（原点 o5）で見た足側ボール中心 B_i = R5 (a_i1 + R6 b_i)。
/// R5 が直交行列なので |B_i| は θ5 に依らない。
inline Vec3 ankleBall(
  const AnkleParams & prm, int i, double c5, double s5, double c6, double s6)
{
  const Vec3 & b = prm.b[i];
  // V(θ6) = a_i1 + Ry(θ6) b_i                                        （§5.1）
  const Vec3 t{c6 * b.x + s6 * b.z, b.y, -s6 * b.x + c6 * b.z + prm.p5.z};
  // Rx(θ5) t
  return {t.x, c5 * t.y - s5 * t.z, s5 * t.y + c5 * t.z};
}

inline Vec3 ankleBall(const AnkleParams & prm, int i, double th5, double th6)
{
  return ankleBall(prm, i, std::cos(th5), std::sin(th5), std::cos(th6), std::sin(th6));
}

/// クランク先端 K_i(q_i) = C_i + r_i1 (x̂ cos q_i + ẑ sin q_i)。
/// û = x̂ / v̂ = ẑ はクランク軸が y に平行という前提（§2 の A1）から来る。
inline Vec3 ankleCrank(const AnkleParams & prm, int i, double q)
{
  return {prm.c[i].x + prm.r[i] * std::cos(q), prm.c[i].y,
    prm.c[i].z + prm.r[i] * std::sin(q)};
}

/// 閉ループ拘束の残差。0 なら組めている。単位は mm²。
inline double ankleConstraint(
  const AnkleParams & prm, int i, double th5, double th6, double q)
{
  const Vec3 d = ankleBall(prm, i, th5, th6) - ankleCrank(prm, i, q);
  return d.normSq() - prm.rodSq[i];
}

// ---------------------------------------------------------------------------
// 逆変換 (θ5, θ6) -> (q1, q2)    仕様 §4     閉形式・反復なし
// ---------------------------------------------------------------------------
/// 鎖 1 本ぶんの中間量。w_i はクランク軸の足 O_i から足側ボールへ向かうベクトル。
struct AnkleIkChain
{
  double p{0.0};       //!< w_x（= w·û）
  double q{0.0};       //!< w_z（= w·v̂）
  double h{0.0};       //!< w_y。クランク平面からの外れ。本機は中立で |h| = 1.0mm
  double s{0.0};       //!< (r² + |w|² - L²) / 2                        （§4.2）
  double rho{0.0};     //!< √(w_x² + w_z²)。ボールからクランク軸までの距離
  double delta{0.0};   //!< Δ = r² - s²/ρ²  死点余裕 [mm²]。最大 r²      （§4.3）
  double margin{0.0};  //!< ρ - |s|/ρ [mm]。Δ と符号が一致する長さ次元の指標
};

inline AnkleIkChain ankleIkChain(const AnkleParams & prm, int i, const Vec3 & ball)
{
  const Vec3 w = ball - prm.c[i];
  AnkleIkChain ch;
  ch.p = w.x;
  ch.q = w.z;
  ch.h = w.y;
  ch.s = (prm.rSq[i] + w.normSq() - prm.rodSq[i]) * 0.5;
  const double rho2 = ch.p * ch.p + ch.q * ch.q;
  ch.rho = std::sqrt(rho2);
  if (rho2 > 0.0) {
    ch.delta = prm.rSq[i] - ch.s * ch.s / rho2;
    ch.margin = ch.rho - std::fabs(ch.s) / ch.rho;
  } else {
    ch.delta = -1.0;
    ch.margin = -1.0;
  }
  return ch;
}

struct AnkleIkResult
{
  double q[kAnkleChains]{0.0, 0.0};        //!< クランク角 [rad]
  double delta[kAnkleChains]{0.0, 0.0};    //!< 死点余裕 Δ [mm²]。最大 r² = 324
  double margin[kAnkleChains]{0.0, 0.0};   //!< 同じものを長さ次元で [mm]
  AnkleIkStatus status{AnkleIkStatus::Ok};
};

/// 関節角 -> クランク角。**反復なし。姿勢による分岐は枝の符号だけ**（§7 の約束）。
///
/// clamp=true なら到達不能でも最寄り（Δ=0 とみなした解）を書き戻す。status は
/// Unreachable のまま。delta は必ず埋まるので、呼び側は delta <= 0 を可動域超過
/// として上位に返す（握り潰さない。§4.3）。
inline AnkleIkResult ankleIk(
  const AnkleParams & prm, double th5, double th6, bool clamp = true)
{
  const double c5 = std::cos(th5), s5 = std::sin(th5);
  const double c6 = std::cos(th6), s6 = std::sin(th6);

  AnkleIkResult res;
  for (int i = 0; i < kAnkleChains; ++i) {
    const AnkleIkChain ch = ankleIkChain(prm, i, ankleBall(prm, i, c5, s5, c6, s6));
    res.delta[i] = ch.delta;
    res.margin[i] = ch.margin;

    if (!(ch.rho > 1e-12)) {
      res.status = AnkleIkStatus::Degenerate;
      continue;
    }
    const bool reach = ch.delta >= 0.0;
    if (!reach && res.status == AnkleIkStatus::Ok) {
      res.status = AnkleIkStatus::Unreachable;
    }
    if (!reach && !clamp) {continue;}

    const double root = std::sqrt(reach ? ch.delta : 0.0);
    const double rho2 = ch.rho * ch.rho;
    const double e = static_cast<double>(prm.eps[i]);
    // (§4.2) 複号同順。ε が組み立ての枝
    const double x = ch.s * ch.p / rho2 - e * (ch.q / ch.rho) * root;
    const double z = ch.s * ch.q / rho2 + e * (ch.p / ch.rho) * root;
    res.q[i] = std::atan2(z, x);
  }
  return res;
}

/// 各鎖の死点余裕 Δ だけが欲しいとき（軌道生成のマージン監視用）。
inline void ankleMargin(
  const AnkleParams & prm, double th5, double th6, double delta[kAnkleChains])
{
  const AnkleIkResult r = ankleIk(prm, th5, th6, /*clamp=*/false);
  for (int i = 0; i < kAnkleChains; ++i) {delta[i] = r.delta[i];}
}

// ---------------------------------------------------------------------------
// 順変換 (q1, q2) -> (θ5, θ6)    仕様 §5
// ---------------------------------------------------------------------------
/// 鎖 1 本の拘束を (cos θ5, sin θ5) の 1 次式にまとめた係数。          （§5.1）
struct AnkleFwdCoeff
{
  double u{0.0};      //!< U'
  double v{0.0};      //!< V'
  double w{0.0};      //!< W'
  double amp{0.0};    //!< 2√(U'² + V'²)
};

inline AnkleFwdCoeff ankleFwdCoeff(
  const AnkleParams & prm, int i, double th6, double q)
{
  const Vec3 & P = prm.a0[i];
  const Vec3 & b = prm.b[i];
  const double c6 = std::cos(th6), s6 = std::sin(th6);
  const Vec3 V{c6 * b.x + s6 * b.z, b.y, -s6 * b.x + c6 * b.z + prm.p5.z};
  const Vec3 K = {prm.r[i] * std::cos(q), 0.0, prm.r[i] * std::sin(q)};

  AnkleFwdCoeff co;
  const double pzk = P.z - K.z;
  co.u = P.y * V.y + pzk * V.z;
  co.v = pzk * V.y - P.y * V.z;
  co.w = prm.a0Sq[i] + V.normSq() + prm.rSq[i] - prm.rodSq[i]
    + 2.0 * (P.x - K.x) * V.x - 2.0 * P.dot(K);
  co.amp = 2.0 * std::hypot(co.u, co.v);
  return co;
}

/// 鎖 i が描く曲線 Θ_i(θ6)。ok=false ならその θ6 に解が無い（可動域の縁）。
struct AnkleCurve
{
  double th5{0.0};
  bool ok{false};
};

inline AnkleCurve ankleCurve(const AnkleParams & prm, int i, double th6, double q)
{
  const AnkleFwdCoeff co = ankleFwdCoeff(prm, i, th6, q);
  AnkleCurve cv;
  if (!(co.amp > 1e-12)) {return cv;}
  const double cosArg = -co.w / co.amp;
  if (cosArg < -1.0 || cosArg > 1.0) {return cv;}
  cv.th5 = std::atan2(co.v, co.u) +
    static_cast<double>(prm.del[i]) * std::acos(cosArg);
  cv.ok = true;
  return cv;
}

namespace apdetail
{
inline double wrapPi(double a) {return std::atan2(std::sin(a), std::cos(a));}

inline double clamp(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

/// Φ(θ6) = Θ1 - Θ2。ok=false なら片方の曲線が存在しない。
inline double phi(
  const AnkleParams & prm, const double q[kAnkleChains], double th6, bool & ok)
{
  const AnkleCurve c1 = ankleCurve(prm, 0, th6, q[0]);
  const AnkleCurve c2 = ankleCurve(prm, 1, th6, q[1]);
  ok = c1.ok && c2.ok;
  return ok ? wrapPi(c1.th5 - c2.th5) : 0.0;
}
}  // namespace apdetail

struct AnkleFkResult
{
  double th5{0.0};
  double th6{0.0};
  int iters{0};
  //! 入力のクランク角がリミット外だったのでクランプして解いた
  bool crankClamped{false};
  //! θ6 が窓の縁に張り付いている（この先は型 2 特異点なので解かない）
  bool atWindow{false};
  AnkleFkStatus status{AnkleFkStatus::Ok};
};

namespace apdetail
{
/// 窓の端で曲線が存在しないとき、内側へ詰め寄って評価できる点を探す。
/// 回数は固定なので最悪実行時間は姿勢に依らない。
inline bool phiEdge(
  const AnkleParams & prm, const double q[kAnkleChains], double & x, double toward,
  double & f)
{
  bool ok = false;
  f = phi(prm, q, x, ok);
  if (ok) {return true;}
  double a = x;
  for (int k = 0; k < ankle_config::FK_EDGE_SHRINK; ++k) {
    a = 0.5 * (a + toward);
    f = phi(prm, q, a, ok);
    if (ok) {x = a; return true;}
  }
  return false;
}
}  // namespace apdetail

/// クランク角 -> 関節角。**窓 FK_WINDOW_DEG の中だけを解く**（§5.2 + 二分法で保護）。
///
/// この関数が呼び側に約束すること:
///
///   1. **有界**   返す θ6 は必ず窓の中。θ5 も窓の中の曲線 Θ1 の値なので有界
///   2. **連続**   (q1,q2) を少し動かせば (θ5,θ6) も少ししか動かない。
///                 根が窓から出るときは縁に連続に近づくので、張り付きも跳ばない
///   3. **必ず終わる**  ブラケットを保つので、ニュートンが飛んでも二分法に落ちる
///
/// 窓の中では Φ(θ6) が狭義単調（ankle_config.hpp の FK_WINDOW_DEG 参照）なので、
/// 両端の符号を見るだけで根を挟み込める。根が 1 個しかないため、
/// **種 th6Seed は収束を速くするだけで、答えを選ばない**。前周期の値が無ければ
/// 0 を渡してよく、起動直後に別の姿勢へ飛ぶことはない。
///
/// th6Seed  前周期の θ6。窓の外なら窓に丸めてから使う
inline AnkleFkResult ankleFk(
  const AnkleParams & prm, const double qIn[kAnkleChains], double th6Seed = 0.0)
{
  using namespace ankle_config;
  AnkleFkResult res;

  // --- 1) 入力をクランクリミットの箱に丸める ---------------------------------
  // サーボが物理的に出せない角度が読めたら（通信化け・原点ずれ）ここで切る。
  double q[kAnkleChains];
  for (int i = 0; i < kAnkleChains; ++i) {
    q[i] = apdetail::clamp(qIn[i], prm.qMin[i], prm.qMax[i]);
    if (q[i] != qIn[i]) {res.crankClamped = true;}
  }

  // --- 2) 窓の両端で Φ を評価してブラケットを作る -----------------------------
  double lo = FK_WINDOW_DEG[0] * M_PI / 180.0;
  double hi = FK_WINDOW_DEG[1] * M_PI / 180.0;
  const double mid0 = 0.5 * (lo + hi);
  double flo = 0.0, fhi = 0.0;
  if (!apdetail::phiEdge(prm, q, lo, mid0, flo) ||
    !apdetail::phiEdge(prm, q, hi, mid0, fhi))
  {
    res.th6 = apdetail::clamp(th6Seed, lo, hi);
    res.status = AnkleFkStatus::NoCurve;
    return res;
  }

  auto finish = [&prm, &q](AnkleFkResult & r) {
      const AnkleCurve c1 = ankleCurve(prm, 0, r.th6, q[0]);
      if (c1.ok) {
        r.th5 = apdetail::wrapPi(c1.th5);
      } else {
        r.status = AnkleFkStatus::NoCurve;
      }
    };

  // --- 3) 窓の中に根が無い: 近いほうの縁に張り付ける ---------------------------
  // Φ は窓の中で単調なので、同符号 = 根は窓の外。**縁を返す**。
  // 根が窓から出ていく瞬間は Φ(縁) = 0 なので、Ok から Clamped へは連続に移る。
  if (flo * fhi > 0.0) {
    res.th6 = (std::fabs(flo) <= std::fabs(fhi)) ? lo : hi;
    res.atWindow = true;
    res.status = AnkleFkStatus::Clamped;
    finish(res);
    return res;
  }

  // --- 4) 二分法で守ったニュートン -------------------------------------------
  double x = apdetail::clamp(th6Seed, lo, hi);
  bool ok = false;
  double fx = apdetail::phi(prm, q, x, ok);
  if (!ok) {x = 0.5 * (lo + hi); fx = apdetail::phi(prm, q, x, ok);}

  for (int k = 0; k < FK_MAX_ITER; ++k) {
    res.iters = k + 1;
    if (std::fabs(fx) < FK_TOL_RAD || (hi - lo) < FK_TOL_X_RAD) {
      res.th6 = x;
      finish(res);
      return res;
    }
    // ブラケットを詰める。以降 [lo, hi] は必ず根を挟んでいる
    if (flo * fx <= 0.0) {hi = x; fhi = fx;} else {lo = x; flo = fx;}

    // ニュートンの一歩。数値微分で十分（§5.2）
    bool ok2 = false;
    const double f2 = apdetail::phi(prm, q, x + FK_DERIV_H, ok2);
    double next = 0.5 * (lo + hi);                       // 既定は二分
    if (ok2) {
      const double d = (f2 - fx) / FK_DERIV_H;
      if (std::fabs(d) > 1e-12) {
        const double cand = x - fx / d;
        // ブラケットの外へ出るニュートンは採らない（飛びの元）
        if (cand > lo && cand < hi) {next = cand;}
      }
    }
    x = next;
    fx = apdetail::phi(prm, q, x, ok);
    if (!ok) {                                           // 窓の中で曲線が切れた
      res.th6 = 0.5 * (lo + hi);
      res.status = AnkleFkStatus::NoCurve;
      finish(res);
      return res;
    }
  }

  // 上限まで来ても x はブラケットの中。値は使えるが精度は保証しない
  res.th6 = x;
  res.status = AnkleFkStatus::NotConverged;
  finish(res);
  return res;
}

/// 前周期の θ6 が無いときの入口。
///
/// 窓の中で Φ が単調になったので**粗探しは要らなくなった**（種に依らず同じ解が
/// 出る）。呼び分けを消さずに済むよう名前だけ残してある。
inline AnkleFkResult ankleFkScan(const AnkleParams & prm, const double q[kAnkleChains])
{
  return ankleFk(prm, q, 0.0);
}

// ---------------------------------------------------------------------------
// 指令側のエンベロープ    型 2 特異点へ向かう指令をここで止める
// ---------------------------------------------------------------------------
/// (θ5, θ6) を「順変換が必ず解ける範囲」に丸める。
///
/// θ6 は窓 FK_WINDOW_DEG、θ5 は機構限界 TH5_MECH_LIMIT_DEG の内側に取る。
/// θ5 のほうは Δ < 0 で逆変換が弾くが、θ6 は **Δ が正のまま特異点に入る**ので
/// 逆変換だけでは止まらない。指令を出す前にここを通すこと。
struct AnkleClampResult
{
  double th5{0.0};
  double th6{0.0};
  bool clamped{false};   //!< 丸めた（軌道生成が可動域を超えた。握り潰さず記録する）
};

inline AnkleClampResult ankleClampJoints(double th5, double th6)
{
  using namespace ankle_config;
  const double d = M_PI / 180.0;
  // 機構限界そのものではなく少し内側。Δ = 0 の縁は逆変換の精度が落ちる
  const double t5lim = (TH5_MECH_LIMIT_DEG - 1.0) * d;
  AnkleClampResult r;
  r.th5 = apdetail::clamp(th5, -t5lim, t5lim);
  r.th6 = apdetail::clamp(th6, FK_WINDOW_DEG[0] * d, FK_WINDOW_DEG[1] * d);
  r.clamped = (r.th5 != th5) || (r.th6 != th6);
  return r;
}

// ---------------------------------------------------------------------------
// ヤコビアン    仕様 §6
// ---------------------------------------------------------------------------
/// Jθ (dθ5, dθ6)ᵀ = Jq (dq1, dq2)ᵀ。Jq は対角。
struct AnkleJacobian
{
  double jt[kAnkleChains][2]{};   //!< Jθ  行 i = 鎖 i、列 = (θ5, θ6)
  double jq[kAnkleChains]{};      //!< Jq の対角成分
  double det{0.0};                //!< det Jθ。0 で型 2 特異点
};

inline AnkleJacobian ankleJacobian(
  const AnkleParams & prm, const double q[kAnkleChains], double th5, double th6)
{
  const double c5 = std::cos(th5), s5 = std::sin(th5);
  const double c6 = std::cos(th6), s6 = std::sin(th6);
  // ĵ6 = R5 ŷ、o6 = R5 (0,0,-ℓ5)
  const Vec3 j6{0.0, c5, s5};
  const Vec3 o6{0.0, -s5 * prm.p5.z, c5 * prm.p5.z};
  const Vec3 xh{1.0, 0.0, 0.0};
  const Vec3 eh{ankle_config::CRANK_AXIS[0], ankle_config::CRANK_AXIS[1],
    ankle_config::CRANK_AXIS[2]};

  AnkleJacobian J;
  for (int i = 0; i < kAnkleChains; ++i) {
    const Vec3 B = ankleBall(prm, i, c5, s5, c6, s6);
    const Vec3 K = ankleCrank(prm, i, q[i]);
    const Vec3 d = (B - prm.c[i]) - (K - prm.c[i]);   // ロッドベクトル B - K
    J.jt[i][0] = xh.dot(B.cross(d));
    J.jt[i][1] = j6.dot((B - o6).cross(d));
    J.jq[i] = eh.dot((K - prm.c[i]).cross(d));
  }
  J.det = J.jt[0][0] * J.jt[1][1] - J.jt[0][1] * J.jt[1][0];
  return J;
}

// ---------------------------------------------------------------------------
// サーボ換算    仕様 §4.5
// ---------------------------------------------------------------------------
//   φ_i = φ_i,home + σ_i n_i (q_i - q_i,neutral)
//
// 原点は T ポーズ（脚がまっすぐ真上に伸びた姿勢）で取ってあるので、
// 「サーボ実測 = home」が「(θ5,θ6) = (0,0)」に対応する。q_i,neutral はそのときの
// クランク角で 0 ではない（本機は -0.390° / +1.202°）。finalize() が入れる。
inline double ankleServoFromCrank(const AnkleParams & prm, int i, double q)
{
  return prm.servoHome[i] + prm.servoSign[i] * prm.gear[i] * (q - prm.qNeutral[i]);
}

inline double ankleCrankFromServo(const AnkleParams & prm, int i, double phi)
{
  return prm.qNeutral[i] + (phi - prm.servoHome[i]) / (prm.servoSign[i] * prm.gear[i]);
}

// ---------------------------------------------------------------------------
// 生成
// ---------------------------------------------------------------------------
inline void AnkleParams::finalize()
{
  p5 = {0.0, 0.0, -l5};
  for (int i = 0; i < kAnkleChains; ++i) {
    c[i] = -a0[i];
    a0Sq[i] = a0[i].normSq();
    rSq[i] = r[i] * r[i];
    rodSq[i] = rod[i] * rod[i];
    qNeutral[i] = 0.0;
  }

  // クランク角のリミット（servo_limits.yaml と対。左右で同じ値）。
  for (int i = 0; i < kAnkleChains; ++i) {
    qMin[i] = ankle_config::CRANK_LIMIT_DEG[i][0] * M_PI / 180.0;
    qMax[i] = ankle_config::CRANK_LIMIT_DEG[i][1] * M_PI / 180.0;
  }

  // 中立姿勢のクランク角。T ポーズ原点の基準（§4.5）。
  const AnkleIkResult neutral = ankleIk(*this, 0.0, 0.0, /*clamp=*/true);
  for (int i = 0; i < kAnkleChains; ++i) {qNeutral[i] = neutral.q[i];}

  // 順変換の枝 β_i は「中立で Θ_i(0) = 0 になる側」。自動で選んでおけば、
  // 寸法を差し替えても壊れない（§5.1）。
  if (ankle_config::BRANCH_FROM_NEUTRAL) {
    for (int i = 0; i < kAnkleChains; ++i) {
      double best = 0.0;
      int bestSign = del[i];
      bool first = true;
      for (int sgn = -1; sgn <= 1; sgn += 2) {
        del[i] = sgn;
        const AnkleCurve cv = ankleCurve(*this, i, 0.0, neutral.q[i]);
        if (!cv.ok) {continue;}
        const double err = std::fabs(apdetail::wrapPi(cv.th5));
        if (first || err < best) {best = err; bestSign = sgn; first = false;}
      }
      del[i] = bestSign;
    }
  }
}

/// 左右の脚ぶんのパラメータを作る。**右脚は s = -1 を入れるだけ**（§7 の約束）。
inline AnkleParams makeAnkleParams(Side side)
{
  using namespace ankle_config;
  const double s = (side == Side::LEFT) ? 1.0 : -1.0;

  AnkleParams prm;
  prm.l5 = config::L5;
  // SERVO_SIGN は [脚][鎖]。Side enum の値に依存させず、ここで明示的に選ぶ。
  const int leg = (side == Side::LEFT) ? 0 : 1;
  for (int i = 0; i < kAnkleChains; ++i) {
    prm.a0[i] = {A0[i][0], A0[i][1] * s * A0_Y_SIDE_SIGN, A0[i][2]};
    prm.b[i] = {FOOT_BALL[i][0], FOOT_BALL[i][1] * s * FOOT_BALL_Y_SIDE_SIGN,
      FOOT_BALL[i][2]};
    prm.r[i] = CRANK_RADIUS[i];
    prm.rod[i] = ROD_LENGTH[i];
    prm.eps[i] = BRANCH_EPS[i];
    prm.del[i] = BRANCH_BETA[i];
    prm.servoHome[i] = SERVO_HOME_DEG[i] * M_PI / 180.0;
    prm.servoSign[i] = SERVO_SIGN[leg][i];
    prm.gear[i] = GEAR_RATIO[i];
  }
  prm.finalize();
  return prm;
}

}  // namespace roboone_kinematics

#endif  // ROBOONE_KINEMATICS__ANKLE_PARALLEL_HPP_
