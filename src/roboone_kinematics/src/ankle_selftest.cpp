// 足首パラレルリンクの検算。
//
//   ros2 run roboone_kinematics ankle_selftest [-n 往復の試行数]
//
// 期待値は「足首パラレルリンク：関節角 ↔ サーボ角 変換の実装仕様」§8。
// 仕様の数値は s = +1（左脚）・ℓ5 = 8.0 mm のもの。leg_config.hpp の L5 が
// 違う値なら、寸法に依る照合は飛ばして機構そのものの検算だけを走らせる。
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>

#include "roboone_kinematics/ankle_parallel.hpp"

using namespace roboone_kinematics;      // NOLINT(build/namespaces)

namespace
{
constexpr double kDeg = 180.0 / M_PI;
int g_fail = 0;

void check(bool cond, const char * what)
{
  std::printf("  [%s] %s\n", cond ? " OK " : "FAIL", what);
  if (!cond) {++g_fail;}
}

/// 仕様 §8 の数値は ℓ5 = 8.0 前提。leg_config.hpp が違えば照合はスキップする。
const bool kSpecGeometry = std::fabs(config::L5 - ankle_config::L5_SPEC) < 1e-9;

void checkSpec(bool cond, const char * what)
{
  if (kSpecGeometry) {
    check(cond, what);
  } else if (!cond) {
    std::printf("  [skip] %s（ℓ5 が仕様 §3 の %.1f と違う）\n",
      what, ankle_config::L5_SPEC);
  }
}

double angleDiff(double a, double b)
{
  return std::fabs(std::atan2(std::sin(a - b), std::cos(a - b)));
}

// ---------------------------------------------------------------------- §8.1
void testNeutral(const AnkleParams & prm)
{
  std::printf("\n§8.1 中立姿勢 (θ5, θ6) = (0, 0)\n");
  const AnkleIkResult ik = ankleIk(prm, 0.0, 0.0);
  std::printf("    q      = (%+.4f, %+.4f) deg   仕様 (-0.390, +1.202)\n",
    ik.q[0] * kDeg, ik.q[1] * kDeg);
  std::printf("    Δ      = (%.1f, %.1f) mm²      仕様 (323.7, 324.0)  最大 r² = %.0f\n",
    ik.delta[0], ik.delta[1], prm.r[0] * prm.r[0]);

  check(ik.status == AnkleIkStatus::Ok, "中立姿勢は到達可能");
  checkSpec(std::fabs(ik.q[0] * kDeg - (-0.390)) < 1e-3, "鎖1 のクランク角が -0.390 deg");
  checkSpec(std::fabs(ik.q[1] * kDeg - (+1.202)) < 1e-3, "鎖2 のクランク角が +1.202 deg");
  checkSpec(std::fabs(ik.delta[0] - 323.7) < 0.1, "鎖1 の Δ が 323.7 mm²");
  checkSpec(std::fabs(ik.delta[1] - 324.0) < 0.1, "鎖2 の Δ が 324.0 mm²");

  // 拘束残差。閉形式が本当に閉ループを満たしているか
  double worst = 0.0;
  for (int i = 0; i < kAnkleChains; ++i) {
    worst = std::max(worst, std::fabs(ankleConstraint(prm, i, 0.0, 0.0, ik.q[i])));
  }
  std::printf("    拘束残差 %.2e mm²\n", worst);
  check(worst < 1e-8, "閉ループ拘束を満たす");

  // T ポーズ原点: サーボ 0 が中立姿勢に対応する（§4.5）
  for (int i = 0; i < kAnkleChains; ++i) {
    const double phi = ankleServoFromCrank(prm, i, ik.q[i]);
    check(std::fabs(phi - prm.servoHome[i]) < 1e-12,
      i == 0 ? "T ポーズで鎖1 のサーボ値が home" : "T ポーズで鎖2 のサーボ値が home");
  }
}

// ---------------------------------------------------------------------- §4.4
void testBranches(const AnkleParams & prm)
{
  std::printf("\n§4.4 組み立ての枝\n");
  AnkleParams alt = prm;
  alt.eps[0] = alt.eps[1] = -1;
  alt.finalize();
  const AnkleIkResult ik = ankleIk(alt, 0.0, 0.0);
  std::printf("    ε = -1 : (%+.3f, %+.3f) deg   仕様 (-176.691, -179.183)\n",
    ik.q[0] * kDeg, ik.q[1] * kDeg);
  checkSpec(std::fabs(ik.q[0] * kDeg - (-176.691)) < 1e-2, "ε=-1 の鎖1 が -176.691 deg");
  checkSpec(std::fabs(ik.q[1] * kDeg - (-179.183)) < 1e-2, "ε=-1 の鎖2 が -179.183 deg");

  std::printf("    β（自動選択）= (%+d, %+d)   仕様 (+1, -1)\n", prm.del[0], prm.del[1]);
  checkSpec(prm.del[0] == +1 && prm.del[1] == -1, "順変換の枝 β が (+1, -1)");
}

// ---------------------------------------------------------------------- §8.2
void testRoundTrip(const AnkleParams & prm, int n)
{
  std::printf("\n§8.2 往復 ankleFk(ankleIk(θ)) == θ   (±15 deg 一様乱数 %d 姿勢)\n", n);
  std::mt19937 rng(20260828);
  std::uniform_real_distribution<double> ang(-15.0 / kDeg, 15.0 / kDeg);
  std::uniform_real_distribution<double> seed(-5.0 / kDeg, 5.0 / kDeg);

  double worst = 0.0;
  int unreachable = 0, notOk = 0, iters = 0, maxIter = 0;
  for (int k = 0; k < n; ++k) {
    const double t5 = ang(rng), t6 = ang(rng);
    const AnkleIkResult ik = ankleIk(prm, t5, t6);
    if (ik.status != AnkleIkStatus::Ok) {++unreachable; continue;}
    // 初期値を ±5 deg ずらしても落ちること
    const AnkleFkResult fk = ankleFk(prm, ik.q, t6 + seed(rng));
    if (fk.status != AnkleFkStatus::Ok) {++notOk; continue;}
    iters += fk.iters;
    maxIter = std::max(maxIter, fk.iters);
    worst = std::max(worst, std::max(angleDiff(fk.th5, t5), angleDiff(fk.th6, t6)) * kDeg);
  }
  std::printf("    最大誤差 %.2e deg   到達不能 %d 件   収束せず %d 件\n",
    worst, unreachable, notOk);
  std::printf("    ニュートン反復 平均 %.2f 回 / 最大 %d 回（上限 %d）\n",
    static_cast<double>(iters) / std::max(1, n - unreachable - notOk),
    maxIter, ankle_config::FK_MAX_ITER);
  check(unreachable == 0, "±15 deg の範囲に到達不能な姿勢が無い");
  check(notOk == 0, "順変換が全姿勢で収束する");
  check(worst < 1e-9, "往復誤差が 1e-9 deg 未満");
}

// ---------------------------------------------------------------------- §8.3
void testWorkspace(const AnkleParams & prm)
{
  std::printf("\n§8.3 可動域\n");

  // 単軸（0.1 deg 刻みで Δ が負になるまで）
  for (int axis = 0; axis < 2; ++axis) {
    double lim = 0.0;
    for (double a = 0.0; a <= 130.0; a += 0.1) {
      const double t = a / kDeg;
      const AnkleIkResult p = ankleIk(prm, axis == 0 ? t : 0.0, axis == 0 ? 0.0 : t, false);
      const AnkleIkResult m = ankleIk(prm, axis == 0 ? -t : 0.0, axis == 0 ? 0.0 : -t, false);
      if (p.status != AnkleIkStatus::Ok || m.status != AnkleIkStatus::Ok) {break;}
      lim = a;
    }
    if (axis == 0) {
      std::printf("    θ5（ロール）単軸 ±%.1f deg   仕様 ±35.8\n", lim);
      checkSpec(std::fabs(lim - 35.8) < 0.2, "θ5 の機構限界が ±35.8 deg");
    } else {
      std::printf("    θ6（ピッチ）単軸 ±%.1f deg 以上（130 deg まで走査。機構では止まらない）\n",
        lim);
      check(lim >= 79.8, "θ6 は少なくとも ±79.8 deg まで到達できる");
    }
  }

  // 同時
  for (const double lim : {15.0, 20.0, 25.0}) {
    double q1lo = 1e9, q1hi = -1e9, q2lo = 1e9, q2hi = -1e9, dmin = 1e9;
    int bad = 0;
    for (int i = 0; i <= 40; ++i) {
      for (int j = 0; j <= 40; ++j) {
        const double t5 = (-lim + 2.0 * lim * i / 40.0) / kDeg;
        const double t6 = (-lim + 2.0 * lim * j / 40.0) / kDeg;
        const AnkleIkResult r = ankleIk(prm, t5, t6, false);
        if (r.status != AnkleIkStatus::Ok) {++bad; continue;}
        q1lo = std::min(q1lo, r.q[0] * kDeg); q1hi = std::max(q1hi, r.q[0] * kDeg);
        q2lo = std::min(q2lo, r.q[1] * kDeg); q2hi = std::max(q2hi, r.q[1] * kDeg);
        dmin = std::min(dmin, std::min(r.delta[0], r.delta[1]));
      }
    }
    std::printf("    同時 ±%.0f : q1 [%+.1f, %+.1f]  q2 [%+.1f, %+.1f]  Δmin %.0f/324  到達不能 %d\n",
      lim, q1lo, q1hi, q2lo, q2hi, dmin, bad);
    if (lim == 15.0) {
      checkSpec(bad == 0 && std::fabs(dmin - 198.0) < 2.0, "同時 ±15 deg で Δmin = 198");
    } else if (lim == 25.0) {
      check(bad > 0, "同時 ±25 deg では到達不能な点が出る");
    }
  }
}

// ---------------------------------------------------------------------- §8.4
void testLinear(const AnkleParams & prm)
{
  std::printf("\n§8.4 中立まわりの線形近似  ∂q/∂θ\n");
  const double h = 1e-6;
  double J[2][2];
  for (int j = 0; j < 2; ++j) {
    const AnkleIkResult p = ankleIk(prm, j == 0 ? h : 0.0, j == 0 ? 0.0 : h);
    const AnkleIkResult m = ankleIk(prm, j == 0 ? -h : 0.0, j == 0 ? 0.0 : -h);
    for (int i = 0; i < 2; ++i) {J[i][j] = (p.q[i] - m.q[i]) / (2.0 * h);}
  }
  std::printf("        [ %+.3f  %+.3f ]      仕様 [ +1.474  -0.776 ]\n", J[0][0], J[0][1]);
  std::printf("        [ %+.3f  %+.3f ]           [ -1.480  -0.789 ]\n", J[1][0], J[1][1]);
  std::printf("    差がロール（θ5 ≈ (q1-q2)/%.3f）、和がピッチ（θ6 ≈ -(q1+q2)/%.3f）\n",
    J[0][0] - J[1][0], -(J[0][1] + J[1][1]));
  checkSpec(std::fabs(J[0][0] - 1.474) < 2e-3 && std::fabs(J[1][0] + 1.480) < 2e-3,
    "ロール列が (+1.474, -1.480)");
  checkSpec(std::fabs(J[0][1] + 0.776) < 2e-3 && std::fabs(J[1][1] + 0.789) < 2e-3,
    "ピッチ列が (-0.776, -0.789)");
}

// ------------------------------------------------------------------- 左右対称
void testMirror(int n)
{
  std::printf("\n左右の対称性（右脚は s = -1 を入れるだけ）\n");
  const AnkleParams l = makeAnkleParams(Side::LEFT);
  const AnkleParams r = makeAnkleParams(Side::RIGHT);
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> ang(-15.0 / kDeg, 15.0 / kDeg);
  double worst = 0.0;
  for (int k = 0; k < n; ++k) {
    const double t5 = ang(rng), t6 = ang(rng);
    // y 反転の鏡像ではロール θ5 だけ符号が変わり、ピッチ θ6 は変わらない。
    // クランク円は x-z 平面内なので y 反転の影響を受けず、クランク角もそのまま。
    // **鎖の添字は入れ替わらない**（取付高さ 73/108 が鎖を区別するため）。
    const AnkleIkResult a = ankleIk(l, t5, t6);
    const AnkleIkResult b = ankleIk(r, -t5, t6);
    for (int i = 0; i < kAnkleChains; ++i) {
      worst = std::max(worst, angleDiff(a.q[i], b.q[i]));
    }
  }
  std::printf("    q_left,i(θ5,θ6) と q_right,i(-θ5,θ6) の最大差 %.2e deg\n", worst * kDeg);
  check(worst * kDeg < 1e-9, "右脚が左脚の鏡像になっている");
}

// -------------------------------------------------------------------- 起動時
void testScan(const AnkleParams & prm)
{
  std::printf("\n§5.2 前周期の値が無いときの粗探し\n");
  const double t5 = 9.0 / kDeg, t6 = -11.0 / kDeg;
  const AnkleIkResult ik = ankleIk(prm, t5, t6);
  const AnkleFkResult fk = ankleFkScan(prm, ik.q);
  std::printf("    ankleFkScan → (%+.4f, %+.4f) deg   真値 (%+.4f, %+.4f)\n",
    fk.th5 * kDeg, fk.th6 * kDeg, t5 * kDeg, t6 * kDeg);
  check(fk.status == AnkleFkStatus::Ok, "種なしでも解が見つかる");
  check(angleDiff(fk.th5, t5) * kDeg < 1e-9 && angleDiff(fk.th6, t6) * kDeg < 1e-9,
    "粗探しの解が正しい");
}

// -------------------------------------------------------------------- 異常系
void testErrors(const AnkleParams & prm)
{
  std::printf("\n§8.6 異常系\n");
  const AnkleIkResult far = ankleIk(prm, 40.0 / kDeg, 0.0, false);
  std::printf("    θ5 = 40 deg → status=%d  Δ=(%.1f, %.1f)\n",
    static_cast<int>(far.status), far.delta[0], far.delta[1]);
  check(far.status == AnkleIkStatus::Unreachable, "θ5 = 40 deg は到達不能を返す");

  AnkleParams broken = prm;
  broken.rod[1] = 200.0;
  broken.finalize();
  const AnkleIkResult bad = ankleIk(broken, 0.0, 0.0, false);
  std::printf("    ロッド長 r22 = 200 → status=%d\n", static_cast<int>(bad.status));
  check(bad.status == AnkleIkStatus::Unreachable, "壊した幾何は中立でも到達不能");

  // ロッドが届かない幾何では、中立のクランク角でも曲線が存在しない
  const AnkleIkResult nk = ankleIk(prm, 0.0, 0.0);
  const AnkleFkResult fkBad = ankleFk(broken, nk.q, 0.0);
  std::printf("    壊した幾何の順変換 → status=%d\n", static_cast<int>(fkBad.status));
  check(fkBad.status != AnkleFkStatus::Ok, "解の無い (q1,q2) は Ok を返さない");

  // 逆に、Ok を返したときは必ず閉ループ拘束を満たしていること。
  // (2.5, -2.5) rad は「でたらめ」ではなく θ5 ≈ 24 deg の実在する姿勢なので、
  // 到達不能の例には使えない。不変条件はこちらで見る。
  const double qFar[kAnkleChains] = {2.5, -2.5};
  const AnkleFkResult fkFar = ankleFkScan(prm, qFar);
  if (fkFar.status == AnkleFkStatus::Ok) {
    double res = 0.0;
    for (int i = 0; i < kAnkleChains; ++i) {
      res = std::max(res,
        std::fabs(ankleConstraint(prm, i, fkFar.th5, fkFar.th6, qFar[i])));
    }
    std::printf("    q = (2.5, -2.5) rad → (%+.2f, %+.2f) deg  拘束残差 %.2e mm²\n",
      fkFar.th5 * kDeg, fkFar.th6 * kDeg, res);
    check(res < 1e-6, "Ok を返した順変換は閉ループ拘束を満たす");
  }
}

// ------------------------------------------------------------------ 整合確認
void testConsistency(const AnkleParams & prm)
{
  std::printf("\n上流との整合\n");
  std::printf("    leg_config::L5 = %.3f mm   仕様 §3 の ℓ5 = %.1f mm\n",
    config::L5, ankle_config::L5_SPEC);
  if (!kSpecGeometry) {
    // 失敗にはしない。ankle_parallel.hpp は config::L5 を読むので FK と IK は
    // 互いに整合しており、機構としては正しく解けている。ずれているのは
    // 「仕様 §8 の期待値を計算したときの寸法」だけ。ただし脚 IK 側と足首側で
    // 別々の ℓ5 を持つと足先位置がずれるので、どちらが正かは決めること（§9-6）。
    std::printf("  [warn] ℓ5 が仕様 §8 の期待値を出したときの値と違う。"
      "寸法に依る照合は飛ばす（機構の検算は寸法に依らず走る）\n");
  }
  check(prm.valid(), "パラメータが前提条件を満たす");
}
}  // namespace

int main(int argc, char ** argv)
{
  int n = 3000;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "-n") == 0 && i + 1 < argc) {n = std::atoi(argv[++i]);}
  }

  const AnkleParams prm = makeAnkleParams(Side::LEFT);
  std::printf("足首パラレルリンク 検算（左脚 s=+1、ℓ5 = %.3f mm）\n", prm.l5);
  std::printf("  クランク半径 %.1f / %.1f mm   ロッド長 %.1f / %.1f mm\n",
    prm.r[0], prm.r[1], prm.rod[0], prm.rod[1]);

  testConsistency(prm);
  testNeutral(prm);
  testBranches(prm);
  testRoundTrip(prm, n);
  testWorkspace(prm);
  testLinear(prm);
  testMirror(500);
  testScan(prm);
  testErrors(prm);

  std::printf("\n%s（失敗 %d 件）\n", g_fail == 0 ? "すべて通過" : "失敗あり", g_fail);
  return g_fail == 0 ? 0 : 1;
}
