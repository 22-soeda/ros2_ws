// 足首パラレルリンクの順変換 / 逆変換。
//
// 土台は docs/足首パラレルリンク導出.pdf。(AP-n) はその式番号、(n) は同文書の
// 通し番号つき式を指す。leg_kinematics.hpp が出す関節角 θ5・θ6 と、実機の
// 2 個のサーボのクランク角 q1・q2 を相互に変換する層で、motion ノードでは
//
//   指令側: 足先目標 -> ik() -> (θ5,θ6) -> ankleIk()  -> (q1,q2) -> サーボ指令
//   観測側: サーボ実測 (q1,q2) -> ankleFk() -> (θ5,θ6) -> fk() -> 重心位置
//
// の順に呼ぶ。leg_kinematics.hpp と同じくヘッダオンリ・ROS 非依存・ヒープ無しで、
// 200Hz の制御ループから直接呼べる。寸法は ankle_config.hpp にまとめてある。
//
// ===========================================================================
// 機構と記号（文書 §1.2・§1.3）
// ===========================================================================
// 下腿に積んだ 2 個のサーボがクランクとボールリンクで足板を押し引きし、足板と
// 下腿は中央のユニバーサルジョイント（= J5/J6）だけで繋がっている。ロッドの
// 両端はボールジョイントなので、鎖 1 本が与える拘束は「両端の距離が一定」の
// ひとつだけである。
//
//   Σ_s   原点 o5、姿勢は Σ_4（下腿）と同じ。下腿に固定された量はここで定数。
//   C_i   クランク軸へ A_i から下ろした垂線の足        （Σ_s の定数）
//   ê_i   クランク軸の向き / û_i,v̂_i 回転円の面内の正規直交対（Σ_s の定数）
//   r_i   回転円の半径 / q_i クランク角（û_i から測る）
//   A_i   クランク先端のボール中心
//   b_i   ロッドの足側ボール中心                        （Σ_6 の定数）
//   B_i   同じ点を Σ_s で見たもの / L_i ロッド長 |B_i - A_i|
//
// ===========================================================================
// 全体の仕掛け
// ===========================================================================
// 出発点は鎖 1 本につき 1 本の閉ループ拘束
//
//   F_i(q_i, θ5, θ6) := |B_i(θ5,θ6) - A_i(q_i)|² - L_i² = 0                (AP-4)
//
// で、これが (cos q_i, sin q_i)、(c5, s5)、(c6, s6) の **どの 1 組についても
// 1 次** であることが全ての鍵になる。1 組だけが未知なら閉形式で解け、2 組が
// 未知なら消去が要る。
//
// 逆変換 (θ5,θ6) -> (q1,q2)  未知は q_i だけ。鎖 1 と鎖 2 が独立に分離し、
//   1 鎖あたり atan2 1 回と arccos 1 回の閉形式で終わる (AP-8)。反復なし。
//   幾何的には「クランクの円」と「B_i 中心・半径 L_i の球」の交点。
//
// 順変換 (q1,q2) -> (θ5,θ6)  未知が 2 つなので閉形式にはならない。ただし
//   鎖 1 本ぶんの式 (AP-10) は (c5,s5) の 1 次なので、θ6 を与えれば θ5 が
//   閉形式で出る（曲線 ψ_i(θ6)、(AP-13)）。2 本の曲線の交点が答えなので、
//   残差 Δ(θ6) := ψ_1(θ6) - ψ_2(θ6) の零点を 1 変数ニュートン法で解く
//   (AP-15)。導関数はヤコビアン (AP-17) から解析的に出る (AP-18) ので
//   数値微分もいらない。枝を組み立て時に固定してあるぶん、Δ の零点には
//   「別の組み方の姿勢」がそもそも現れない。
//
// 起動直後は前周期の θ6 が無いので、組める姿勢を全部出して選ぶ
//   (ankleFkAllSolutions)。文書 §4.4 は θ5 を消去した G(θ6) = 0 (AP-16) を
//   t = tan(θ6/2) の 8 次式 (AP-20) に直して「組み方は高々 8 通り」を示すが、
//   実装はその道を通らない。理由は ankleFkAllSolutions のコメントに書いた。
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
  //! |S_i| > ρ_i。ロッドが届かない（可動域の外）。clamp=true なら最寄りが入る
  Unreachable,
  //! ρ_i ≈ 0。B_i がクランク軸上に来ていて q_i が決まらない（設計で避ける）
  Degenerate,
};

/// ankleFk() の結果。
enum class AnkleFkStatus
{
  Ok = 0,
  //! |W_i| > 2ρ'_i。その θ6 では鎖 i の曲線 ψ_i が存在しない
  NoCurve,
  //! 反復上限に達した。呼び側は前周期の値を保持して記録する（文書 §10）
  NotConverged,
  //! Δ'(θ6) ≈ 0。det Jθ = 0 の型 2 特異点（文書 §5.2）
  Singular,
};

// ---------------------------------------------------------------------------
// パラメータ
// ---------------------------------------------------------------------------
/// 足首パラレルリンクの定数。値は ankle_config.hpp から来る。
/// 添え字 i = 0,1 が文書の鎖 i = 1,2。
struct AnkleParams
{
  // ---- 入力（ankle_config.hpp の値） ----
  Vec3 c[kAnkleChains]{};        //!< C_i  クランク回転円の中心      (Σ_s)
  Vec3 e[kAnkleChains]{};        //!< ê_i  クランク軸の向き（単位）  (Σ_s)
  Vec3 u[kAnkleChains]{};        //!< û_i  q_i=0 の向き（単位・ê に直交）
  double r[kAnkleChains]{};      //!< r_i  クランク半径
  Vec3 b[kAnkleChains]{};        //!< b_i  足側ボール中心            (Σ_6)
  double rod[kAnkleChains]{};    //!< L_i  ロッド長
  double l5{config::L5};         //!< ℓ5  o5 -> o6 の高さオフセット

  double servoZero[kAnkleChains]{};  //!< φ_i0 [rad]
  double servoSign[kAnkleChains]{};  //!< σ_i
  double gear[kAnkleChains]{};       //!< n_i

  int eps[kAnkleChains]{};       //!< ε_i 逆変換の枝   (AP-8)
  int del[kAnkleChains]{};       //!< δ_i 順変換の枝   (AP-13)

  // ---- 派生量（finalize() が埋める） ----
  Vec3 v[kAnkleChains]{};        //!< v̂_i = ê_i × û_i
  Vec3 p5{};                     //!< (0, 0, -ℓ5)
  double bSq[kAnkleChains]{};    //!< |b_i|²

  void finalize();

  /// 前提条件（文書 §1.2 の P1〜P4 のうち式に効くもの）を満たしているか。
  bool valid() const
  {
    for (int i = 0; i < kAnkleChains; ++i) {
      if (!(r[i] > 0.0) || !(rod[i] > 0.0)) {return false;}
      if (std::fabs(e[i].normSq() - 1.0) > 1e-9) {return false;}
      if (std::fabs(u[i].normSq() - 1.0) > 1e-9) {return false;}
      if (std::fabs(e[i].dot(u[i])) > 1e-9) {return false;}
      if (eps[i] != 1 && eps[i] != -1) {return false;}
      if (del[i] != 1 && del[i] != -1) {return false;}
      if (!(gear[i] != 0.0) || std::fabs(std::fabs(servoSign[i]) - 1.0) > 1e-12) {return false;}
    }
    return l5 >= 0.0;
  }
};

// ---------------------------------------------------------------------------
// 幾何の素片
// ---------------------------------------------------------------------------
/// (AP-1) Σ_s で見た足側ボール中心 B_i = R5 (p5 + R6 b_i)。
/// R5 が直交行列なので |B_i| は θ5 に依らない。これが後で θ5 を消去できる理由。
inline Vec3 ankleBall(
  const AnkleParams & prm, int i, double c5, double s5, double c6, double s6)
{
  const Vec3 & b = prm.b[i];
  const Vec3 t{b.x, prm.p5.y + c6 * b.y - s6 * b.z, prm.p5.z + s6 * b.y + c6 * b.z};
  return {c5 * t.x + s5 * t.z, t.y, -s5 * t.x + c5 * t.z};   // R5 = Ry(θ5)
}

inline Vec3 ankleBall(const AnkleParams & prm, int i, double th5, double th6)
{
  return ankleBall(prm, i, std::cos(th5), std::sin(th5), std::cos(th6), std::sin(th6));
}

/// (AP-3) クランク先端のボール中心 A_i(q_i) = C_i + r_i(û_i cos q_i + v̂_i sin q_i)。
inline Vec3 ankleCrank(const AnkleParams & prm, int i, double q)
{
  return prm.c[i] + (prm.u[i] * std::cos(q) + prm.v[i] * std::sin(q)) * prm.r[i];
}

/// (AP-4) 閉ループ拘束の残差。0 なら組めている。単位は mm²。
inline double ankleConstraint(
  const AnkleParams & prm, int i, double th5, double th6, double q)
{
  const Vec3 d = ankleBall(prm, i, th5, th6) - ankleCrank(prm, i, q);
  return d.normSq() - prm.rod[i] * prm.rod[i];
}

// ---------------------------------------------------------------------------
// 逆変換 (θ5, θ6) -> (q1, q2)   文書 §3
// ---------------------------------------------------------------------------
/// (AP-7) 鎖 1 本ぶんの中間量。w_i = B_i - C_i を û_i, v̂_i, ê_i に分解したもの。
struct AnkleIkChain
{
  double p{0.0};        //!< P_i = w_iᵀû_i
  double q{0.0};        //!< Q_i = w_iᵀv̂_i
  double s{0.0};        //!< S_i = (|w_i|² + r_i² - L_i²) / (2 r_i)
  double rho{0.0};      //!< ρ_i = hypot(P_i, Q_i)。B_i からクランク軸までの距離 (AP-9)
  double h{0.0};        //!< h_i = w_iᵀê_i。軸方向の成分 (AP-9)
  double margin{0.0};   //!< ρ_i - |S_i| [mm]。0 で死点、負で到達不能 (文書 §3.3)
};

inline AnkleIkChain ankleIkChain(const AnkleParams & prm, int i, const Vec3 & ball)
{
  const Vec3 w = ball - prm.c[i];
  AnkleIkChain ch;
  ch.p = w.dot(prm.u[i]);
  ch.q = w.dot(prm.v[i]);
  ch.h = w.dot(prm.e[i]);
  ch.s = (w.normSq() + prm.r[i] * prm.r[i] - prm.rod[i] * prm.rod[i]) / (2.0 * prm.r[i]);
  ch.rho = std::hypot(ch.p, ch.q);
  ch.margin = ch.rho - std::fabs(ch.s);
  return ch;
}

struct AnkleIkResult
{
  double q[kAnkleChains]{0.0, 0.0};        //!< クランク角 [rad]
  double margin[kAnkleChains]{0.0, 0.0};   //!< 死点余裕 ρ_i - |S_i| [mm]
  AnkleIkStatus status{AnkleIkStatus::Ok};
};

/// (AP-8) 関節角 -> クランク角。反復なし。1 鎖あたり atan2 1 回と arccos 1 回。
///
/// clamp=true なら到達不能でも最寄りのクランク角を書き戻す（status は Unreachable
/// のまま）。false なら q は触らない。どちらでも margin は必ず埋まるので、
/// 呼び側は margin <= 0 を可動域超過として上位に返す（文書 §10）。
inline AnkleIkResult ankleIk(
  const AnkleParams & prm, double th5, double th6, bool clamp = true)
{
  const double c5 = std::cos(th5), s5 = std::sin(th5);
  const double c6 = std::cos(th6), s6 = std::sin(th6);

  AnkleIkResult res;
  for (int i = 0; i < kAnkleChains; ++i) {
    const AnkleIkChain ch = ankleIkChain(prm, i, ankleBall(prm, i, c5, s5, c6, s6));
    res.margin[i] = ch.margin;

    if (ch.rho < 1e-12) {                    // B_i がクランク軸上。q が決まらない
      res.status = AnkleIkStatus::Degenerate;
      continue;
    }
    const double ratio = ch.s / ch.rho;
    if (std::fabs(ratio) > 1.0) {
      if (res.status == AnkleIkStatus::Ok) {res.status = AnkleIkStatus::Unreachable;}
      if (!clamp) {continue;}
    }
    res.q[i] = std::atan2(ch.q, ch.p) +
      prm.eps[i] * std::acos(detail::clampUnit(ratio));
  }
  return res;
}

// ---------------------------------------------------------------------------
// 順変換 (q1, q2) -> (θ5, θ6)   文書 §4
// ---------------------------------------------------------------------------
/// (AP-11)・表 2  鎖 1 本ぶんの拘束を (c5, s5) の 1 次にまとめた係数。
///   F_i = W_i - 2 U_i c5 - 2 V_i s5 = 0                                (AP-10)
/// U_i, V_i, W_i は (c6, s6) についても 1 次である。
struct AnkleFwdCoeff
{
  double u{0.0}, v{0.0}, w{0.0};
};

inline AnkleFwdCoeff ankleFwdCoeff(
  const AnkleParams & prm, int i, const Vec3 & a, double c6, double s6)
{
  const Vec3 & b = prm.b[i];
  const double g = c6 * b.y - s6 * b.z;      // (R6 b_i)_y   (3)
  const double k = s6 * b.y + c6 * b.z;      // (R6 b_i)_z   (3)
  const double km = k - prm.l5;

  AnkleFwdCoeff co;
  co.u = a.x * b.x + a.z * km;
  co.v = a.x * km - a.z * b.x;
  co.w = prm.l5 * prm.l5 + prm.bSq[i] + a.normSq() - prm.rod[i] * prm.rod[i] -
    2.0 * prm.l5 * k - 2.0 * a.y * g;
  return co;
}

/// (AP-13) 鎖 1 本が (θ5, θ6) 平面に描く曲線 θ5 = ψ_i(θ6)。
struct AnkleCurve
{
  double th5{0.0};
  double rho{0.0};    //!< ρ'_i = hypot(U_i, V_i)
  AnkleFwdCoeff co{};
  bool ok{false};     //!< |W_i| <= 2ρ'_i。false ならその θ6 に曲線は無い
};

inline AnkleCurve ankleCurve(
  const AnkleParams & prm, int i, const Vec3 & a, double c6, double s6, int branch)
{
  AnkleCurve cv;
  cv.co = ankleFwdCoeff(prm, i, a, c6, s6);
  cv.rho = std::hypot(cv.co.u, cv.co.v);
  if (cv.rho < 1e-12) {return cv;}
  const double ratio = cv.co.w / (2.0 * cv.rho);
  cv.ok = std::fabs(ratio) <= 1.0;
  cv.th5 = std::atan2(cv.co.v, cv.co.u) + branch * std::acos(detail::clampUnit(ratio));
  return cv;
}

/// 組み立てで決まった枝 δ_i を使う版。制御ループはこちらを呼ぶ。
inline AnkleCurve ankleCurve(
  const AnkleParams & prm, int i, const Vec3 & a, double c6, double s6)
{
  return ankleCurve(prm, i, a, c6, s6, prm.del[i]);
}

// ---------------------------------------------------------------------------
// ヤコビアン   文書 §5.1
// ---------------------------------------------------------------------------
/// (AP-17)  Jθ [θ̇5; θ̇6] = Jq [q̇1; q̇2]。Jq が対角なのは鎖同士が直接には
/// 繋がっていないからで、逆変換が鎖ごとに分離したことの微分版。
/// ∂F_i/∂θ5 = 2 (Jθ)_{i1}、∂F_i/∂θ6 = 2 (Jθ)_{i2} でもある。
struct AnkleJacobian
{
  double jt[kAnkleChains][2]{};   //!< Jθ
  double jq[kAnkleChains]{};      //!< Jq の対角成分

  double det() const { return jt[0][0] * jt[1][1] - jt[0][1] * jt[1][0]; }
};

namespace detail
{

/// Jθ の第 i 行と Jq の第 i 対角成分。行ごとに θ5 を変えて呼べるようにしてある
/// （順変換のニュートン法では鎖 i を曲線 ψ_i の上で評価する）。
inline void ankleJacRow(
  const AnkleParams & prm, int i, double c5, double s5, double c6, double s6,
  const Vec3 & a, double row[2], double * jq)
{
  const Vec3 ball = ankleBall(prm, i, c5, s5, c6, s6);
  const Vec3 d = ball - a;
  const Vec3 j5{0.0, 1.0, 0.0};                 // ĵ5 = y 軸
  const Vec3 j6{c5, 0.0, -s5};                  // ĵ6 = R5 x̂
  const Vec3 o6{-s5 * prm.l5, 0.0, -c5 * prm.l5};   // o6 = R5 p5

  row[0] = j5.dot(ball.cross(d));
  row[1] = j6.dot((ball - o6).cross(d));
  if (jq) {*jq = prm.e[i].dot((a - prm.c[i]).cross(d));}
}

inline double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace detail

inline AnkleJacobian ankleJacobian(
  const AnkleParams & prm, double th5, double th6, const double q[kAnkleChains])
{
  const double c5 = std::cos(th5), s5 = std::sin(th5);
  const double c6 = std::cos(th6), s6 = std::sin(th6);
  AnkleJacobian J;
  for (int i = 0; i < kAnkleChains; ++i) {
    detail::ankleJacRow(prm, i, c5, s5, c6, s6, ankleCrank(prm, i, q[i]), J.jt[i], &J.jq[i]);
  }
  return J;
}

// ---------------------------------------------------------------------------
// 順変換：1 変数ニュートン法   文書 §4.3
// ---------------------------------------------------------------------------
struct AnkleFkResult
{
  double th5{0.0}, th6{0.0};
  int iters{0};
  double residual{0.0};   //!< Δ(θ6) = ψ_1 - ψ_2 [rad]
  AnkleFkStatus status{AnkleFkStatus::NotConverged};
};

/// (AP-15) クランク角 -> 関節角。Δ(θ6) := ψ_1(θ6) - ψ_2(θ6) の零点を解く。
///
/// th6Seed には前周期の θ6 を渡す。数回の反復で落ちる。起動直後のように
/// 前周期の値が無いときは ankleFkAllSolutions() で全解を出して選ぶ（文書 §10）。
///
/// 枝 δ_i を組み立て時に固定してあるので、Δ の零点に「別の組み方の姿勢」は
/// 現れない。2 変数ニュートンや 8 次式と違って収束先を選ぶ処理が要らない。
inline AnkleFkResult ankleFk(
  const AnkleParams & prm, const double q[kAnkleChains], double th6Seed = 0.0)
{
  const Vec3 a[kAnkleChains] = {ankleCrank(prm, 0, q[0]), ankleCrank(prm, 1, q[1])};

  AnkleFkResult res;
  double th6 = detail::wrapPi(th6Seed);
  AnkleCurve cv[kAnkleChains];
  auto evalAt = [&](double t, AnkleCurve out[kAnkleChains]) {
      const double c6 = std::cos(t), s6 = std::sin(t);
      for (int i = 0; i < kAnkleChains; ++i) {out[i] = ankleCurve(prm, i, a[i], c6, s6);}
      return out[0].ok && out[1].ok;
    };

  if (!evalAt(th6, cv)) {
    res.status = AnkleFkStatus::NoCurve;
    res.th6 = th6;
    return res;
  }

  for (int it = 1; it <= ankle_config::FK_MAX_ITER; ++it) {
    res.iters = it;
    const double delta = detail::wrapPi(cv[0].th5 - cv[1].th5);
    res.residual = delta;
    if (std::fabs(delta) < ankle_config::FK_TOL_RAD) {
      res.th5 = detail::wrapPi(cv[0].th5 + 0.5 * detail::wrapPi(cv[1].th5 - cv[0].th5));
      res.th6 = th6;
      res.status = AnkleFkStatus::Ok;
      return res;
    }

    // (AP-18) ψ_i'(θ6) = -(Jθ)_{i2} / (Jθ)_{i1}。鎖 i は曲線 ψ_i の上で評価する
    // ので、解に達していなくても各行が正しい接線を与える。
    const double c6 = std::cos(th6), s6 = std::sin(th6);
    double dpsi[kAnkleChains];
    for (int i = 0; i < kAnkleChains; ++i) {
      double row[2];
      detail::ankleJacRow(
        prm, i, std::cos(cv[i].th5), std::sin(cv[i].th5), c6, s6, a[i], row, nullptr);
      if (std::fabs(row[0]) < 1e-12) {
        res.status = AnkleFkStatus::Singular;   // その鎖が死点。∂F_i/∂θ5 = 0
        res.th5 = cv[0].th5;
        res.th6 = th6;
        return res;
      }
      dpsi[i] = -row[1] / row[0];
    }
    const double ddelta = dpsi[0] - dpsi[1];
    if (!std::isfinite(ddelta) || std::fabs(ddelta) < 1e-12) {
      res.status = AnkleFkStatus::Singular;     // det Jθ = 0。2 曲線が接している
      res.th5 = cv[0].th5;
      res.th6 = th6;
      return res;
    }

    // 踏み出した先で曲線が消えていたら（|W_i| > 2ρ'_i）歩幅を半分に戻す。
    double step = -delta / ddelta;
    bool moved = false;
    for (int bt = 0; bt < 12; ++bt) {
      AnkleCurve trial[kAnkleChains];
      const double t = detail::wrapPi(th6 + step);
      if (evalAt(t, trial)) {
        th6 = t;
        cv[0] = trial[0];
        cv[1] = trial[1];
        moved = true;
        break;
      }
      step *= 0.5;
    }
    if (!moved) {
      res.status = AnkleFkStatus::NoCurve;
      res.th5 = cv[0].th5;
      res.th6 = th6;
      return res;
    }
  }

  res.th5 = cv[0].th5;
  res.th6 = th6;
  res.status = AnkleFkStatus::NotConverged;
  return res;
}

// ---------------------------------------------------------------------------
// 順変換：全解   文書 §4.4
// ---------------------------------------------------------------------------
/// (AP-14) 2 本の (AP-10) を (c5, s5) の連立 1 次と見たときの分子・分母。
/// 解くのには使わない（下の ankleFkAllSolutions のコメント参照）。診断用で、
/// D = 0 は 2 直線が平行になる点、すなわち型 2 特異点（文書 §5.2）を意味する。
struct AnkleElim
{
  double nc{0.0};   //!< N_c = W_1 V_2 - W_2 V_1。解では c5 = N_c / D
  double ns{0.0};   //!< N_s = U_1 W_2 - U_2 W_1。解では s5 = N_s / D
  double d{0.0};    //!< D   = 2 (U_1 V_2 - U_2 V_1)
  double g{0.0};    //!< G   = N_c² + N_s² - D²   (AP-16)。零点が解
};

inline AnkleElim ankleElim(
  const AnkleParams & prm, const Vec3 a[kAnkleChains], double c6, double s6)
{
  const AnkleFwdCoeff c0 = ankleFwdCoeff(prm, 0, a[0], c6, s6);
  const AnkleFwdCoeff c1 = ankleFwdCoeff(prm, 1, a[1], c6, s6);
  AnkleElim el;
  el.nc = c0.w * c1.v - c1.w * c0.v;
  el.ns = c0.u * c1.w - c1.u * c0.w;
  el.d = 2.0 * (c0.u * c1.v - c1.u * c0.v);
  el.g = el.nc * el.nc + el.ns * el.ns - el.d * el.d;
  return el;
}

struct AnkleFkSolution
{
  double th5{0.0}, th6{0.0};
  double det{0.0};   //!< det Jθ。0 に近い解は型 2 特異点の近く（文書 §5.2）
};

/// 順変換の解を全部拾う。起動時の初期化と、単体テストのオラクル用。
///
/// 4 通りの枝の組み合わせ (δ_1, δ_2) ∈ {±1}² それぞれについて、残差
///   Δ_{δ1δ2}(θ6) = ψ_1^{δ1}(θ6) - ψ_2^{δ2}(θ6)
/// を θ6 ∈ (-π, π] で走査し、符号変化を二分法で詰める。曲線の交点が解なので、
/// これで組める姿勢が全部出る（文書 §4.4 の「高々 8 通り」と同じ集合）。
///
/// 文書 §4.4 は θ5 を消去した G(θ6) = N_c² + N_s² - D² = 0 (AP-16)、さらに
/// t = tan(θ6/2) で 8 次式 (AP-20) に直す道筋を与えているが、実装には使わない。
///   - 8 次式は係数が 10^16 の桁になって根が悪条件（文書 §4.5 で往復誤差が 6 桁悪化）
///   - G のままでも、2 直線が平行になる D = 0 の近くで G が正になる区間が
///     極端に狭くなり、格子走査がそこを跨いで根を落とす（実測で 4000 姿勢中 11 件）
/// Δ は D → 0 でも滑らかなので、この 2 つの問題がどちらも起きない。
///
/// 返すのは書き込んだ解の数。maxOut を超えた分は捨てる。
inline int ankleFkAllSolutions(
  const AnkleParams & prm, const double q[kAnkleChains],
  AnkleFkSolution * out, int maxOut)
{
  const Vec3 a[kAnkleChains] = {ankleCrank(prm, 0, q[0]), ankleCrank(prm, 1, q[1])};

  // 枝 (d1,d2) の残差。曲線が両方存在するときだけ true。
  auto residual = [&](double t, int d1, int d2, double & delta, double & th5) {
      const double c6 = std::cos(t), s6 = std::sin(t);
      const AnkleCurve k1 = ankleCurve(prm, 0, a[0], c6, s6, d1);
      const AnkleCurve k2 = ankleCurve(prm, 1, a[1], c6, s6, d2);
      if (!k1.ok || !k2.ok) {return false;}
      delta = detail::wrapPi(k1.th5 - k2.th5);
      th5 = k1.th5;
      return true;
    };

  const int n = ankle_config::FK_SCAN_DIVISIONS;
  const double step = 2.0 * M_PI / n;
  int found = 0;

  for (int s1 = 0; s1 < 2 && found < maxOut; ++s1) {
    for (int s2 = 0; s2 < 2 && found < maxOut; ++s2) {
      const int d1 = s1 ? -1 : 1;
      const int d2 = s2 ? -1 : 1;

      double tPrev = -M_PI, rPrev = 0.0, dummy = 0.0;
      bool okPrev = residual(tPrev, d1, d2, rPrev, dummy);

      for (int j = 1; j <= n && found < maxOut; ++j) {
        const double tCur = -M_PI + step * j;
        double rCur = 0.0;
        const bool okCur = residual(tCur, d1, d2, rCur, dummy);

        // 符号変化があるか。|Δ| が π をまたいで折り返しただけの偽の符号変化は、
        // 隣り合う値が連続していない（差が π 以上）ことで弾く。
        const bool bracket = okPrev && okCur &&
          std::fabs(rCur - rPrev) < M_PI &&
          ((rPrev <= 0.0 && rCur > 0.0) || (rPrev >= 0.0 && rCur < 0.0));

        if (bracket) {
          double lo = tPrev, hi = tCur, rlo = rPrev;
          double th5 = 0.0, rm = 0.0;
          for (int k = 0; k < 80 && (hi - lo) > 1e-15; ++k) {
            const double mid = 0.5 * (lo + hi);
            if (!residual(mid, d1, d2, rm, th5)) {break;}
            if ((rlo < 0.0) == (rm < 0.0)) {
              lo = mid;
              rlo = rm;
            } else {
              hi = mid;
            }
          }
          const double th6 = 0.5 * (lo + hi);
          if (residual(th6, d1, d2, rm, th5)) {
            // 拘束を本当に満たしているか（枝の折り返しで拾った偽解を落とす）
            bool good = true;
            for (int i = 0; i < kAnkleChains; ++i) {
              good = good && std::fabs(ankleConstraint(prm, i, th5, th6, q[i])) < 1e-3;
            }
            // 別の枝の組で既に拾った解と同じなら足さない（曲線の折り返しでは
            // δ = +1 と -1 の枝が合流するので、同じ交点が 2 度出る）。
            for (int k = 0; good && k < found; ++k) {
              good = std::fabs(detail::wrapPi(out[k].th5 - th5)) > 1e-7 ||
                std::fabs(detail::wrapPi(out[k].th6 - th6)) > 1e-7;
            }
            if (good) {
              AnkleFkSolution sol;
              sol.th5 = detail::wrapPi(th5);
              sol.th6 = th6;
              sol.det = ankleJacobian(prm, sol.th5, sol.th6, q).det();
              out[found++] = sol;
            }
          }
        }
        tPrev = tCur;
        rPrev = rCur;
        okPrev = okCur;
      }
    }
  }
  return found;
}

// ---------------------------------------------------------------------------
// サーボ指令への換算   文書 §3.4 式 (1)
// ---------------------------------------------------------------------------
/// φ_i = φ_i0 + σ_i n_i q_i。q_i はクランク角であってサーボの指令値ではない。
inline double ankleServoFromCrank(const AnkleParams & prm, int i, double q)
{
  return prm.servoZero[i] + prm.servoSign[i] * prm.gear[i] * q;
}

inline double ankleCrankFromServo(const AnkleParams & prm, int i, double phi)
{
  return (phi - prm.servoZero[i]) / (prm.servoSign[i] * prm.gear[i]);
}

// ---------------------------------------------------------------------------
// パラメータの組み立て
// ---------------------------------------------------------------------------
inline void AnkleParams::finalize()
{
  p5 = {0.0, 0.0, -l5};
  for (int i = 0; i < kAnkleChains; ++i) {
    // ê を単位化し、û から ê 成分を抜いて正規直交対にする。
    const double en = e[i].norm();
    if (en > 0.0) {e[i] = e[i] * (1.0 / en);}
    Vec3 uu = u[i] - e[i] * u[i].dot(e[i]);
    const double un = uu.norm();
    if (un > 0.0) {uu = uu * (1.0 / un);}
    u[i] = uu;
    v[i] = e[i].cross(u[i]);
    bSq[i] = b[i].normSq();
  }

  // ロッド長を「中立姿勢 (θ5,θ6)=(0,0) が q_i=0」から決める（文書 §8）。
  // 実機でターンバックルを回して合わせる作業がそのまま式になっている。
  if (ankle_config::ROD_LEN_FROM_NEUTRAL) {
    for (int i = 0; i < kAnkleChains; ++i) {
      rod[i] = (ankleBall(*this, i, 1.0, 0.0, 1.0, 0.0) - (c[i] + u[i] * r[i])).norm();
    }
  }

  // 枝 ε_i, δ_i を「中立姿勢で q_i = 0 / ψ_i(0) = 0 になる方」に取る（文書 §3.3・§4.2）。
  if (!ankle_config::BRANCH_FROM_NEUTRAL) {
    return;
  }
  for (int i = 0; i < kAnkleChains; ++i) {
    const AnkleIkChain ch = ankleIkChain(*this, i, ankleBall(*this, i, 1.0, 0.0, 1.0, 0.0));
    const double base = std::atan2(ch.q, ch.p);
    const double arc = (ch.rho > 0.0) ? std::acos(detail::clampUnit(ch.s / ch.rho)) : 0.0;
    eps[i] = (std::fabs(detail::wrapPi(base + arc)) <=
      std::fabs(detail::wrapPi(base - arc))) ? +1 : -1;

    const Vec3 a = c[i] + u[i] * r[i];              // A_i(q_i = 0)
    const AnkleFwdCoeff co = ankleFwdCoeff(*this, i, a, 1.0, 0.0);
    const double rho = std::hypot(co.u, co.v);
    const double base2 = std::atan2(co.v, co.u);
    const double arc2 = (rho > 0.0) ? std::acos(detail::clampUnit(co.w / (2.0 * rho))) : 0.0;
    del[i] = (std::fabs(detail::wrapPi(base2 + arc2)) <=
      std::fabs(detail::wrapPi(base2 - arc2))) ? +1 : -1;
  }
}

/// ankle_config.hpp から左右脚のパラメータを組み立てる。
///
/// 左脚は右脚の鏡像なので、鏡映 M = diag(-1,1,1) を掛けた幾何になる（文書 §6.1）。
///   C_i, b_i, û_i は M を掛ける（x 成分の符号が反転）
///   ê_i は -M を掛ける。(Ma)×(Mb) = -M(a×b) なので、こうしないと
///   v̂_i = ê_i × û_i が鏡像にならず、同じ q_i が鏡像の姿勢を指さなくなる
/// θ5・θ6 の定義は左右で共通（leg_kinematics.hpp と同じ座標系のまま）なので、
/// 同じサーボ角に対して θ5 は左右で符号が逆になる。それが鏡像機構の正しい姿。
///
/// TODO(CAD/実機): 左右のサーボの向きと原点は SERVO_SIGN / SERVO_ZERO_DEG 側で
///                 吸収する。実機で 1 軸ずつ回して確かめること。
inline AnkleParams makeAnkleParams(Side side)
{
  namespace ac = ankle_config;
  const double lat = (side == Side::RIGHT) ? 1.0 : -1.0;

  AnkleParams prm;
  prm.l5 = config::L5;
  for (int i = 0; i < kAnkleChains; ++i) {
    prm.c[i] = {lat * ac::CRANK_CENTER[i][0], ac::CRANK_CENTER[i][1], ac::CRANK_CENTER[i][2]};
    prm.e[i] = {ac::CRANK_AXIS[i][0], lat * ac::CRANK_AXIS[i][1], lat * ac::CRANK_AXIS[i][2]};
    prm.u[i] = {lat * ac::CRANK_REF_DIR[i][0], ac::CRANK_REF_DIR[i][1], ac::CRANK_REF_DIR[i][2]};
    prm.r[i] = ac::CRANK_RADIUS[i];
    prm.b[i] = {lat * ac::FOOT_BALL[i][0], ac::FOOT_BALL[i][1], ac::FOOT_BALL[i][2]};
    prm.rod[i] = ac::ROD_LENGTH[i];
    prm.servoZero[i] = ac::SERVO_ZERO_DEG[i] * M_PI / 180.0;
    prm.servoSign[i] = static_cast<double>(ac::SERVO_SIGN[i]);
    prm.gear[i] = ac::GEAR_RATIO[i];
    prm.eps[i] = ac::BRANCH_EPS[i];
    prm.del[i] = ac::BRANCH_DELTA[i];
  }
  prm.finalize();
  return prm;
}

}  // namespace roboone_kinematics

#endif  // ROBOONE_KINEMATICS__ANKLE_PARALLEL_HPP_
