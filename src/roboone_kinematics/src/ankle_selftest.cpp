// 足首パラレルリンク FK/IK の自己検算。
//   ros2 run roboone_kinematics ankle_selftest [-n 姿勢数] [--seed S]
//
// docs/足首パラレルリンク導出.pdf §8 の検算をそのまま再現し、同文書が本文中に
// 数値で書いている量（中立まわりの線形近似 (AP-22)、静力学 (AP-21)、曲線の傾き
// dθ5/dθ6、死点余裕、|det Jθ|、必要サーボ可動範囲）と突き合わせる。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "roboone_kinematics/ankle_parallel.hpp"

using namespace roboone_kinematics;

namespace
{

int g_failures = 0;

void check(bool cond, const char * what)
{
  if (!cond) {
    std::printf("           *** 不一致: %s\n", what);
    ++g_failures;
  }
}

constexpr double kDeg = 180.0 / M_PI;

double angleDiff(double x, double y)
{
  return std::fabs(std::atan2(std::sin(x - y), std::cos(x - y)));
}

/// 設計可動域の一様乱数姿勢。
void randomPose(std::mt19937_64 & rng, double & th5, double & th6)
{
  std::uniform_real_distribution<double> d5(
    ankle_config::TH5_LIMIT_DEG[0] / kDeg, ankle_config::TH5_LIMIT_DEG[1] / kDeg);
  std::uniform_real_distribution<double> d6(
    ankle_config::TH6_LIMIT_DEG[0] / kDeg, ankle_config::TH6_LIMIT_DEG[1] / kDeg);
  th5 = d5(rng);
  th6 = d6(rng);
}

// ---------------------------------------------------------------------------
// 1. 中立姿勢とロッド長
// ---------------------------------------------------------------------------
void testNeutral(const AnkleParams & prm)
{
  std::printf("\n[1] 中立姿勢 (θ5,θ6)=(0,0)\n");

  const AnkleIkResult ik = ankleIk(prm, 0.0, 0.0);
  std::printf(
    "    逆変換 q = (%.3e, %.3e) deg   status=%d\n",
    ik.q[0] * kDeg, ik.q[1] * kDeg, static_cast<int>(ik.status));
  check(ik.status == AnkleIkStatus::Ok, "中立姿勢が到達不能");
  const double worstQ = std::max(std::fabs(ik.q[0]), std::fabs(ik.q[1])) * kDeg;
  check(worstQ < 1e-12, "中立姿勢で q = 0 にならない");

  // ロッド長は「中立姿勢が q=0」から決めている。表 1 の公称値と照合する。
  std::printf(
    "    ロッド長 L = (%.4f, %.4f) mm   表 1 の公称 (%.1f, %.1f)\n",
    prm.rod[0], prm.rod[1], ankle_config::ROD_LENGTH[0], ankle_config::ROD_LENGTH[1]);
  check(
    std::fabs(prm.rod[0] - ankle_config::ROD_LENGTH[0]) < 0.05 &&
    std::fabs(prm.rod[1] - ankle_config::ROD_LENGTH[1]) < 0.05,
    "ロッド長が表 1 の公称値とずれている");

  const double q0[2] = {0.0, 0.0};
  const AnkleFkResult fk = ankleFk(prm, q0, 0.0);
  std::printf(
    "    順変換 (θ5,θ6) = (%.3e, %.3e) deg   %d 反復\n",
    fk.th5 * kDeg, fk.th6 * kDeg, fk.iters);
  check(fk.status == AnkleFkStatus::Ok, "中立姿勢の順変換が収束しない");
  check(
    std::fabs(fk.th5) * kDeg < 1e-11 && std::fabs(fk.th6) * kDeg < 1e-11,
    "中立姿勢の順変換が 0 に戻らない");

  std::printf("    枝 ε = (%+d, %+d)   δ = (%+d, %+d)\n", prm.eps[0], prm.eps[1],
    prm.del[0], prm.del[1]);
}

// ---------------------------------------------------------------------------
// 2. 往復（文書 §8 の主検算）
// ---------------------------------------------------------------------------
void testRoundTrip(const AnkleParams & prm, int n, std::uint64_t seed)
{
  std::printf("\n[2] 往復 逆変換 -> 順変換（%d 姿勢, 可動域 ±35°）\n", n);
  std::mt19937_64 rng(seed);

  double worstCold = 0.0, worstWarm = 0.0, worstAll = 0.0;
  double minMargin = 1e300, minDet = 1e300;
  double worstF = 0.0;
  int unreachable = 0, notConverged = 0;
  int worstIters = 0;
  double prevTh6 = 0.0;

  for (int k = 0; k < n; ++k) {
    double th5, th6;
    randomPose(rng, th5, th6);

    const AnkleIkResult ik = ankleIk(prm, th5, th6);
    if (ik.status != AnkleIkStatus::Ok) {
      ++unreachable;
      continue;
    }
    minMargin = std::min({minMargin, ik.margin[0], ik.margin[1]});

    // 拘束式そのものが満たされているか（式の実装ミスはここで出る）
    for (int i = 0; i < kAnkleChains; ++i) {
      worstF = std::max(worstF, std::fabs(ankleConstraint(prm, i, th5, th6, ik.q[i])));
    }

    const AnkleJacobian J = ankleJacobian(prm, th5, th6, ik.q);
    minDet = std::min(minDet, std::fabs(J.det()));

    // 起動直後（前周期の値が無い）を模して θ6 = 0 から
    const AnkleFkResult cold = ankleFk(prm, ik.q, 0.0);
    // 制御ループを模して「前の姿勢の θ6」から
    const AnkleFkResult warm = ankleFk(prm, ik.q, prevTh6);
    prevTh6 = th6;

    if (cold.status != AnkleFkStatus::Ok || warm.status != AnkleFkStatus::Ok) {
      ++notConverged;
      continue;
    }
    worstIters = std::max(worstIters, std::max(cold.iters, warm.iters));

    const double ec = std::max(angleDiff(cold.th5, th5), angleDiff(cold.th6, th6)) * kDeg;
    const double ew = std::max(angleDiff(warm.th5, th5), angleDiff(warm.th6, th6)) * kDeg;
    worstCold = std::max(worstCold, ec);
    worstWarm = std::max(worstWarm, ew);
    worstAll = std::max(worstAll, std::max(ec, ew));
  }

  std::printf("    往復誤差   θ6=0 から %.2e deg / 前周期から %.2e deg\n", worstCold, worstWarm);
  std::printf("    反復回数   最大 %d（上限 %d）\n", worstIters, ankle_config::FK_MAX_ITER);
  std::printf("    死点余裕   min(ρ_i - |S_i|) = %.2f mm   文書 §5.2 は 8.0 mm\n", minMargin);
  std::printf("    型 2 特異  min|det Jθ| = %.2e            文書 §5.2 は 4.1e6\n", minDet);
  std::printf("    拘束残差   max|F_i| = %.2e mm²\n", worstF);
  std::printf("    到達不能 %d 件 / 収束せず %d 件\n", unreachable, notConverged);

  check(unreachable == 0, "可動域内に到達不能な姿勢がある");
  check(notConverged == 0, "順変換が収束しない姿勢がある");
  check(worstAll < 1e-9, "往復誤差が大きい");
  check(worstF < 1e-6, "拘束式が満たされていない");
  check(minMargin > ankle_config::DEAD_POINT_MARGIN_MM, "死点に近い姿勢がある");
  check(minDet > ankle_config::SINGULARITY_DET_MIN, "型 2 特異点に近い姿勢がある");
}

// ---------------------------------------------------------------------------
// 3. ヤコビアン（解析 vs 数値微分）
// ---------------------------------------------------------------------------
void testJacobian(const AnkleParams & prm, int n, std::uint64_t seed)
{
  std::printf("\n[3] ヤコビアン (AP-17) と逆変換の数値微分\n");
  std::mt19937_64 rng(seed + 1);
  const double h = 1e-6;
  double worst = 0.0;

  for (int k = 0; k < n; ++k) {
    double th5, th6;
    randomPose(rng, th5, th6);
    const AnkleIkResult ik = ankleIk(prm, th5, th6);
    if (ik.status != AnkleIkStatus::Ok) {continue;}

    const AnkleJacobian J = ankleJacobian(prm, th5, th6, ik.q);
    // Jq q̇ = Jθ θ̇ なので ∂q_i/∂θ_j = (Jθ)_{ij} / (Jq)_i
    const AnkleIkResult p5 = ankleIk(prm, th5 + h, th6);
    const AnkleIkResult m5 = ankleIk(prm, th5 - h, th6);
    const AnkleIkResult p6 = ankleIk(prm, th5, th6 + h);
    const AnkleIkResult m6 = ankleIk(prm, th5, th6 - h);

    for (int i = 0; i < kAnkleChains; ++i) {
      const double num[2] = {
        (p5.q[i] - m5.q[i]) / (2.0 * h), (p6.q[i] - m6.q[i]) / (2.0 * h)};
      for (int j = 0; j < 2; ++j) {
        worst = std::max(worst, std::fabs(num[j] - J.jt[i][j] / J.jq[i]));
      }
    }
  }
  std::printf("    max|∂q/∂θ の差| = %.2e   文書 §8 は 4.6e-11\n", worst);
  check(worst < 1e-7, "解析ヤコビアンが数値微分と合わない");
}

// ---------------------------------------------------------------------------
// 4. 中立姿勢まわりの数値（文書が本文で挙げている値との照合）
// ---------------------------------------------------------------------------
void testNeutralNumbers(const AnkleParams & prm)
{
  std::printf("\n[4] 中立姿勢まわり\n");
  const double q0[2] = {0.0, 0.0};
  const AnkleJacobian J = ankleJacobian(prm, 0.0, 0.0, q0);

  // 線形近似 (AP-22)  [q1;q2] ≈ M [θ5;θ6],  M = Jq⁻¹ Jθ
  double M[2][2];
  for (int i = 0; i < kAnkleChains; ++i) {
    for (int j = 0; j < 2; ++j) {M[i][j] = J.jt[i][j] / J.jq[i];}
  }
  std::printf("    (AP-22) M = [%+.3f %+.3f; %+.3f %+.3f]   文書は [-0.915 0.656; 0.907 0.658]\n",
    M[0][0], M[0][1], M[1][0], M[1][1]);
  check(
    std::fabs(M[0][0] + 0.915) < 0.002 && std::fabs(M[0][1] - 0.656) < 0.002 &&
    std::fabs(M[1][0] - 0.907) < 0.002 && std::fabs(M[1][1] - 0.658) < 0.002,
    "線形近似 (AP-22) が文書と違う");

  // 差がピッチ、和がロール。文書は θ5 ≈ (q2-q1)/1.82、θ6 ≈ (q1+q2)/1.31
  const double kPitch = M[1][0] - M[0][0];
  const double kRoll = M[0][1] + M[1][1];
  std::printf("    θ5 ≈ (q2-q1)/%.2f   θ6 ≈ (q1+q2)/%.2f   文書は 1.82, 1.31\n", kPitch, kRoll);
  check(std::fabs(kPitch - 1.82) < 0.01 && std::fabs(kRoll - 1.31) < 0.01,
    "差動の係数が文書と違う");

  // 静力学 (AP-21)  τ_q = Jqᵀ Jθ⁻ᵀ τ_θ = M⁻ᵀ τ_θ
  const double detM = M[0][0] * M[1][1] - M[0][1] * M[1][0];
  const double S[2][2] = {
    { M[1][1] / detM, -M[1][0] / detM},
    {-M[0][1] / detM,  M[0][0] / detM},
  };
  std::printf(
    "    (AP-21) JqᵀJθ⁻ᵀ = [%+.3f %+.3f; %+.3f %+.3f]   文書は [-0.550 0.758; 0.548 0.764]\n",
    S[0][0], S[0][1], S[1][0], S[1][1]);
  check(
    std::fabs(S[0][0] + 0.550) < 0.002 && std::fabs(S[0][1] - 0.758) < 0.002 &&
    std::fabs(S[1][0] - 0.548) < 0.002 && std::fabs(S[1][1] - 0.764) < 0.002,
    "静力学の行列が文書と違う");
  std::printf("      -> θ6 は 2 個が同符号 %.2f ずつ、θ5 は逆符号 %.2f ずつ分担（文書 §5.3 と同じ）\n",
    0.5 * (S[0][1] + S[1][1]), 0.5 * (std::fabs(S[0][0]) + std::fabs(S[1][0])));

  // 曲線の傾き dθ5/dθ6 = -(Jθ)_{i2}/(Jθ)_{i1} = -M_{i2}/M_{i1}
  const double slope0 = -J.jt[0][1] / J.jt[0][0];
  const double slope1 = -J.jt[1][1] / J.jt[1][0];
  std::printf("    曲線の傾き dθ5/dθ6 = %+.3f, %+.3f（中立姿勢）\n", slope0, slope1);
  check(
    std::fabs(slope0 + M[0][1] / M[0][0]) < 1e-9 &&
    std::fabs(slope1 + M[1][1] / M[1][0]) < 1e-9,
    "曲線の傾きが (AP-22) と整合しない");
  check(slope0 > 0.5 && slope1 < -0.5, "2 族の曲線が同じ向きに傾いている");
}

// ---------------------------------------------------------------------------
// 4b. 文書の例題（図 3・図 4 の (θ5,θ6) = (+12°, -8°)）
// ---------------------------------------------------------------------------
void testWorkedExample(const AnkleParams & prm)
{
  std::printf("\n[4b] 文書の例題 (θ5,θ6) = (+12°, -8°)\n");
  const double th5 = 12.0 / kDeg, th6 = -8.0 / kDeg;

  // 図 3 の太い 2 本は q1 = -15.6°、q2 = +6.1° の曲線。
  const AnkleIkResult ik = ankleIk(prm, th5, th6);
  std::printf("    逆変換 q = (%+.2f, %+.2f) deg   図 3 は (-15.6, +6.1)\n",
    ik.q[0] * kDeg, ik.q[1] * kDeg);
  check(ik.status == AnkleIkStatus::Ok, "例題が到達不能");
  check(
    std::fabs(ik.q[0] * kDeg + 15.6) < 0.05 && std::fabs(ik.q[1] * kDeg - 6.1) < 0.05,
    "例題のクランク角が図 3 と違う");

  // §4.2 が「傾きが +0.75 と -0.65」と書いている数値は、本文では中立姿勢と
  // されているが、実際に一致するのはこの例題姿勢の方。中立姿勢では +0.72/-0.73。
  const AnkleJacobian J = ankleJacobian(prm, th5, th6, ik.q);
  const double slope0 = -J.jt[0][1] / J.jt[0][0];
  const double slope1 = -J.jt[1][1] / J.jt[1][0];
  std::printf("    曲線の傾き dθ5/dθ6 = %+.3f, %+.3f   文書 §4.2 は +0.75, -0.65\n",
    slope0, slope1);
  check(std::fabs(slope0 - 0.75) < 0.005 && std::fabs(slope1 + 0.65) < 0.005,
    "例題姿勢の曲線の傾きが文書と違う");

  // 図 4 は (q1,q2) = (-15.6°, +6.1°) で組める姿勢が 4 通り、そのうち
  // θ6 = -8° と -4.8° の 2 根が図では重なる、と書いている。
  const double q[2] = {-15.6 / kDeg, 6.1 / kDeg};
  AnkleFkSolution sols[16];
  const int m = ankleFkAllSolutions(prm, q, sols, 16);
  std::printf("    組める姿勢 %d 通り（図 4 は 4 通り）:", m);
  for (int j = 0; j < m; ++j) {std::printf("  θ6=%+.1f°", sols[j].th6 * kDeg);}
  std::printf("\n");
  check(m == 4, "例題の組み方の数が図 4 と違う");

  int nearMinus8 = 0, nearMinus5 = 0;
  for (int j = 0; j < m; ++j) {
    const double t = sols[j].th6 * kDeg;
    if (std::fabs(t + 8.0) < 0.2) {++nearMinus8;}
    if (std::fabs(t + 4.8) < 0.2) {++nearMinus5;}
  }
  check(nearMinus8 == 1 && nearMinus5 == 1, "図 4 が言う -8° / -4.8° の 2 根が出ない");
}

// ---------------------------------------------------------------------------
// 5. 必要なサーボ可動範囲（文書 §8 の最後の項目）
// ---------------------------------------------------------------------------
void testServoRange(const AnkleParams & prm, int n, std::uint64_t seed)
{
  std::printf("\n[5] 設計可動域をカバーするのに必要なクランク角の範囲\n");

  // (a) 可動域の格子を全部なめた厳密な範囲。実機のリミットはこちらで決める。
  double lo[2] = {1e300, 1e300}, hi[2] = {-1e300, -1e300};
  const int g = 121;
  for (int i5 = 0; i5 < g; ++i5) {
    for (int i6 = 0; i6 < g; ++i6) {
      const double th5 = (ankle_config::TH5_LIMIT_DEG[0] +
        (ankle_config::TH5_LIMIT_DEG[1] - ankle_config::TH5_LIMIT_DEG[0]) * i5 / (g - 1)) / kDeg;
      const double th6 = (ankle_config::TH6_LIMIT_DEG[0] +
        (ankle_config::TH6_LIMIT_DEG[1] - ankle_config::TH6_LIMIT_DEG[0]) * i6 / (g - 1)) / kDeg;
      const AnkleIkResult ik = ankleIk(prm, th5, th6);
      if (ik.status != AnkleIkStatus::Ok) {continue;}
      for (int i = 0; i < kAnkleChains; ++i) {
        lo[i] = std::min(lo[i], ik.q[i] * kDeg);
        hi[i] = std::max(hi[i], ik.q[i] * kDeg);
      }
    }
  }

  // (b) 文書 §8 と同じ作り方（一様乱数 n 姿勢）。隅を踏まないぶん内側に出る。
  double rlo[2] = {1e300, 1e300}, rhi[2] = {-1e300, -1e300};
  std::mt19937_64 rng(seed + 6);
  for (int k = 0; k < n; ++k) {
    double th5, th6;
    randomPose(rng, th5, th6);
    const AnkleIkResult ik = ankleIk(prm, th5, th6);
    if (ik.status != AnkleIkStatus::Ok) {continue;}
    for (int i = 0; i < kAnkleChains; ++i) {
      rlo[i] = std::min(rlo[i], ik.q[i] * kDeg);
      rhi[i] = std::max(rhi[i], ik.q[i] * kDeg);
    }
  }

  std::printf("    格子(厳密)   q1 ∈ [%+.1f, %+.1f]   q2 ∈ [%+.1f, %+.1f]\n",
    lo[0], hi[0], lo[1], hi[1]);
  std::printf("    乱数 %d 姿勢 q1 ∈ [%+.1f, %+.1f]   q2 ∈ [%+.1f, %+.1f]\n",
    n, rlo[0], rhi[0], rlo[1], rhi[1]);
  std::printf("    文書 §8      q1 ∈ [-49.7, +62.4]   q2 ∈ [-50.8, +61.5]（乱数 4000 姿勢）\n");

  // 範囲の端は可動域の隅で取るので、乱数標本は seed ごとに ±1.5° ほどばらつく
  // （12 seed で実測）。文書の数値もその散らばりの中にある。厳密な判定は
  // 「格子で求めた範囲が文書の標本を含む」の方で行い、標本同士は緩く見る。
  check(
    lo[0] <= -49.7 && hi[0] >= 62.4 && lo[1] <= -50.8 && hi[1] >= 61.5,
    "厳密な範囲が文書の乱数標本を含んでいない");
  check(
    std::fabs(rlo[0] + 49.7) < 2.5 && std::fabs(rhi[0] - 62.4) < 2.5 &&
    std::fabs(rlo[1] + 50.8) < 2.5 && std::fabs(rhi[1] - 61.5) < 2.5,
    "乱数標本の範囲が文書と違う");
}

// ---------------------------------------------------------------------------
// 6. 全解（文書 §4.4）
// ---------------------------------------------------------------------------
void testAllSolutions(const AnkleParams & prm, int n, std::uint64_t seed)
{
  std::printf("\n[6] 順変換の全解（枝 4 通りの残差 Δ を走査）\n");
  std::mt19937_64 rng(seed + 2);
  int hist[12] = {0};
  int missed = 0;
  double worstResidual = 0.0;

  for (int k = 0; k < n; ++k) {
    double th5, th6;
    randomPose(rng, th5, th6);
    const AnkleIkResult ik = ankleIk(prm, th5, th6);
    if (ik.status != AnkleIkStatus::Ok) {continue;}

    AnkleFkSolution sols[16];
    const int m = ankleFkAllSolutions(prm, ik.q, sols, 16);
    ++hist[std::min(m, 11)];

    // 元の姿勢が全解の中にあるか（1 変数ニュートンの答えと同じものが居るか）
    bool found = false;
    for (int j = 0; j < m; ++j) {
      if (angleDiff(sols[j].th5, th5) < 1e-6 && angleDiff(sols[j].th6, th6) < 1e-6) {
        found = true;
      }
      for (int i = 0; i < kAnkleChains; ++i) {
        worstResidual = std::max(
          worstResidual,
          std::fabs(ankleConstraint(prm, i, sols[j].th5, sols[j].th6, ik.q[i])));
      }
    }
    if (!found) {++missed;}
  }

  std::printf("    解の個数:");
  for (int i = 0; i < 12; ++i) {
    if (hist[i]) {std::printf("  %d 個 = %d 件", i, hist[i]);}
  }
  std::printf("\n    （文書 §8 は 4000 姿勢で 2 個 1250 件 / 4 個 2733 件 / 6 個 17 件）\n");
  std::printf("    全解が拘束を満たす max|F_i| = %.2e mm²\n", worstResidual);
  std::printf("    元の姿勢を拾えなかった件数 = %d\n", missed);
  check(missed == 0, "全解探索が元の姿勢を取りこぼす");
  check(worstResidual < 1e-4, "全解が拘束式を満たしていない");
  check(hist[1] == 0 && hist[3] == 0 && hist[5] == 0, "解の個数が奇数になっている");
}

// ---------------------------------------------------------------------------
// 7. 左右対称に組んだ機構（文書 §6.1）
// ---------------------------------------------------------------------------
void testMirrorSymmetry(const AnkleParams & base)
{
  std::printf("\n[7] 左右対称に組んだ機構では θ5 = 0 ⟺ q1 = q2\n");
  // 鎖 2 を鎖 1 の鏡像 M = diag(-1,1,1) に置き直す（本機は 2 個を上下にずらして
  // 積んでいるので実際には対称でない。ここは式の検算のための仮想機構）。
  AnkleParams sym = base;
  sym.c[1] = {-base.c[0].x, base.c[0].y, base.c[0].z};
  sym.b[1] = {-base.b[0].x, base.b[0].y, base.b[0].z};
  sym.u[1] = {-base.u[0].x, base.u[0].y, base.u[0].z};
  sym.e[1] = {base.e[0].x, -base.e[0].y, -base.e[0].z};   // ê_2 = -M ê_1
  sym.r[1] = base.r[0];
  sym.finalize();
  check(sym.valid(), "対称機構のパラメータが前提を満たさない");
  check(std::fabs(sym.rod[0] - sym.rod[1]) < 1e-12, "対称機構でロッド長が揃わない");

  double worstQ = 0.0, worstTh5 = 0.0;
  for (int k = -30; k <= 30; ++k) {
    const double th6 = k / kDeg;
    const AnkleIkResult ik = ankleIk(sym, 0.0, th6);
    if (ik.status != AnkleIkStatus::Ok) {continue;}
    worstQ = std::max(worstQ, std::fabs(ik.q[0] - ik.q[1]));

    const AnkleFkResult fk = ankleFk(sym, ik.q, th6);
    if (fk.status == AnkleFkStatus::Ok) {
      worstTh5 = std::max(worstTh5, std::fabs(fk.th5) * kDeg);
    }
  }
  std::printf("    θ6 = ±30° で max|q1 - q2| = %.2e rad、戻した θ5 = %.2e deg\n",
    worstQ, worstTh5);
  std::printf("    （文書 §6.1 は厳密に 0 と 2.5e-14 deg）\n");
  check(worstQ < 1e-13, "対称機構で q1 = q2 にならない");
  check(worstTh5 < 1e-10, "対称機構で θ5 = 0 に戻らない");
}

// ---------------------------------------------------------------------------
// 8. ℓ5 = 0（足首 2 軸が交わる。文書 §6.2）
// ---------------------------------------------------------------------------
void testZeroL5(const AnkleParams & base, int n, std::uint64_t seed)
{
  std::printf("\n[8] ℓ5 = 0（足首 2 軸が交わる）\n");
  AnkleParams p = base;
  p.l5 = 0.0;
  p.finalize();
  check(p.valid(), "ℓ5=0 のパラメータが前提を満たさない");

  std::mt19937_64 rng(seed + 3);
  double worst = 0.0;
  int bad = 0;
  for (int k = 0; k < n; ++k) {
    double th5, th6;
    randomPose(rng, th5, th6);
    const AnkleIkResult ik = ankleIk(p, th5, th6);
    if (ik.status != AnkleIkStatus::Ok) {++bad; continue;}
    const AnkleFkResult fk = ankleFk(p, ik.q, th6);
    if (fk.status != AnkleFkStatus::Ok) {++bad; continue;}
    worst = std::max(
      worst, std::max(angleDiff(fk.th5, th5), angleDiff(fk.th6, th6)) * kDeg);
  }
  std::printf("    往復誤差 %.2e deg（文書 §8 は 3.0e-13）、解けない姿勢 %d 件\n", worst, bad);
  check(bad == 0, "ℓ5=0 で解けない姿勢がある");
  check(worst < 1e-9, "ℓ5=0 で往復しない");
}

// ---------------------------------------------------------------------------
// 9. 左脚（鏡像の幾何）
// ---------------------------------------------------------------------------
void testLeftLeg(const AnkleParams & right, int n, std::uint64_t seed)
{
  std::printf("\n[9] 左脚（鏡映 M = diag(-1,1,1) を掛けた幾何）\n");
  const AnkleParams left = makeAnkleParams(Side::LEFT);
  check(left.valid(), "左脚のパラメータが前提を満たさない");
  check(
    std::fabs(left.rod[0] - right.rod[0]) < 1e-12 &&
    std::fabs(left.rod[1] - right.rod[1]) < 1e-12,
    "左右でロッド長が変わってしまっている");

  // 文書 §6.1 の帰結: 同じ (q1,q2) に対して θ5 の符号だけが左右で入れ替わる。
  std::mt19937_64 rng(seed + 4);
  double worst = 0.0;
  int bad = 0;
  for (int k = 0; k < n; ++k) {
    double th5, th6;
    randomPose(rng, th5, th6);
    const AnkleIkResult ikR = ankleIk(right, th5, th6);
    const AnkleIkResult ikL = ankleIk(left, -th5, th6);
    if (ikR.status != AnkleIkStatus::Ok || ikL.status != AnkleIkStatus::Ok) {++bad; continue;}
    worst = std::max(
      worst, std::max(angleDiff(ikR.q[0], ikL.q[0]), angleDiff(ikR.q[1], ikL.q[1])));

    const AnkleFkResult fk = ankleFk(left, ikL.q, th6);
    if (fk.status != AnkleFkStatus::Ok) {++bad; continue;}
    worst = std::max(
      worst, std::max(angleDiff(fk.th5, -th5), angleDiff(fk.th6, th6)));
  }
  std::printf("    右脚 (θ5,θ6) と左脚 (-θ5,θ6) が同じ q を与える: 差 %.2e rad、失敗 %d 件\n",
    worst, bad);
  check(bad == 0, "左脚で解けない姿勢がある");
  check(worst < 1e-9, "左脚の鏡像関係が成り立たない");
}

// ---------------------------------------------------------------------------
// 10. サーボ角の往復
// ---------------------------------------------------------------------------
void testServoMapping(const AnkleParams & prm)
{
  std::printf("\n[10] サーボ角の換算 (1)\n");
  double worst = 0.0;
  for (int k = -60; k <= 60; k += 5) {
    const double q = k / kDeg;
    for (int i = 0; i < kAnkleChains; ++i) {
      worst = std::max(
        worst, std::fabs(ankleCrankFromServo(prm, i, ankleServoFromCrank(prm, i, q)) - q));
    }
  }
  std::printf("    φ <-> q の往復誤差 %.2e rad\n", worst);
  check(worst < 1e-15, "サーボ角の換算が往復しない");
}

// ---------------------------------------------------------------------------
// 11. 実行時間
// ---------------------------------------------------------------------------
void testTiming(const AnkleParams & prm, std::uint64_t seed)
{
  std::printf("\n[11] 実行時間（200Hz ループの見積もり用）\n");
  const int n = 20000;
  std::vector<double> th5(n), th6(n);
  std::mt19937_64 rng(seed + 5);
  for (int k = 0; k < n; ++k) {randomPose(rng, th5[k], th6[k]);}

  double sink = 0.0;
  auto t0 = std::chrono::steady_clock::now();
  for (int k = 0; k < n; ++k) {
    const AnkleIkResult r = ankleIk(prm, th5[k], th6[k]);
    sink += r.q[0] + r.q[1];
  }
  auto t1 = std::chrono::steady_clock::now();

  std::vector<double> q0(n), q1(n);
  for (int k = 0; k < n; ++k) {
    const AnkleIkResult r = ankleIk(prm, th5[k], th6[k]);
    q0[k] = r.q[0];
    q1[k] = r.q[1];
  }
  auto t2 = std::chrono::steady_clock::now();
  for (int k = 0; k < n; ++k) {
    const double q[2] = {q0[k], q1[k]};
    const AnkleFkResult r = ankleFk(prm, q, k ? th6[k - 1] : 0.0);
    sink += r.th5 + r.th6;
  }
  auto t3 = std::chrono::steady_clock::now();

  const double ikNs = std::chrono::duration<double, std::nano>(t1 - t0).count() / n;
  const double fkNs = std::chrono::duration<double, std::nano>(t3 - t2).count() / n;
  std::printf("    逆変換 %.0f ns/回、順変換 %.0f ns/回（前周期を初期値に）\n", ikNs, fkNs);
  std::printf("    200Hz = 5ms 周期に対して %.4f%% / %.4f%%\n",
    ikNs * 1e-6 / 5.0 * 100.0, fkNs * 1e-6 / 5.0 * 100.0);
  if (sink == 12345.678) {std::printf(" ");}   // 最適化除去よけ（sink を使う）
}

}  // namespace

int main(int argc, char ** argv)
{
  int n = 4000;
  std::uint64_t seed = 20260828;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "-n") == 0 && i + 1 < argc) {
      n = std::atoi(argv[++i]);
    } else if (std::strcmp(argv[i], "--seed") == 0 && i + 1 < argc) {
      seed = std::strtoull(argv[++i], nullptr, 10);
    } else {
      std::fprintf(stderr, "usage: %s [-n 姿勢数] [--seed S]\n", argv[0]);
      return 2;
    }
  }

  const AnkleParams prm = makeAnkleParams(Side::RIGHT);
  std::printf("足首パラレルリンク 自己検算（右脚、%d 姿勢、seed %llu）\n",
    n, static_cast<unsigned long long>(seed));
  std::printf("寸法は ankle_config.hpp（docs/足首パラレルリンク導出.pdf 表 1 の仮置き値）\n");
  check(prm.valid(), "パラメータが前提を満たしていない");

  testNeutral(prm);
  testRoundTrip(prm, n, seed);
  testJacobian(prm, std::min(n, 500), seed);
  testNeutralNumbers(prm);
  testWorkedExample(prm);
  testServoRange(prm, n, seed);
  testAllSolutions(prm, n, seed);
  testMirrorSymmetry(prm);
  testZeroL5(prm, std::min(n, 1000), seed);
  testLeftLeg(prm, std::min(n, 1000), seed);
  testServoMapping(prm);
  testTiming(prm, seed);

  std::printf("\n%s（不一致 %d 件）\n", g_failures ? "*** 失敗" : "全部一致", g_failures);
  return g_failures ? 1 : 0;
}
