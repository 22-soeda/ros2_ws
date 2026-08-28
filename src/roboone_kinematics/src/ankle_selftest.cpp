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
  std::printf("\n§5.2 前周期の値が無いとき（粗探しは窓ソルバに統合済み）\n");
  const double t5 = 9.0 / kDeg, t6 = -11.0 / kDeg;
  const AnkleIkResult ik = ankleIk(prm, t5, t6);
  const AnkleFkResult fk = ankleFkScan(prm, ik.q);
  std::printf("    ankleFkScan → (%+.4f, %+.4f) deg   真値 (%+.4f, %+.4f)\n",
    fk.th5 * kDeg, fk.th6 * kDeg, t5 * kDeg, t6 * kDeg);
  check(fk.status == AnkleFkStatus::Ok, "種なしでも解が見つかる");
  check(angleDiff(fk.th5, t5) * kDeg < 1e-9 && angleDiff(fk.th6, t6) * kDeg < 1e-9,
    "種なしの解が正しい");
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

  // クランクリミットの外を渡したら、丸めたことを申告すること。
  // (2.5, -2.5) rad = (143, -143) deg はサーボが出せない角度なので、
  // 通信化けや原点ずれで読めてしまったときにここで切る。
  const double qFar[kAnkleChains] = {2.5, -2.5};
  const AnkleFkResult fkFar = ankleFk(prm, qFar, 0.0);
  std::printf("    q = (%+.0f, %+.0f) deg（リミット外）→ (%+.2f, %+.2f) deg"
    "  crankClamped=%d\n", qFar[0] * kDeg, qFar[1] * kDeg,
    fkFar.th5 * kDeg, fkFar.th6 * kDeg, static_cast<int>(fkFar.crankClamped));
  check(fkFar.crankClamped, "クランクリミット外の入力は丸めたと申告する");
  check(std::fabs(fkFar.th6) * kDeg <= ankle_config::FK_WINDOW_DEG[1] + 1e-9,
    "リミット外の入力でも出力は窓の中に収まる");

  // 逆に、リミットの中で Ok を返したときは必ず閉ループ拘束を満たしていること。
  const double qIn[kAnkleChains] = {prm.qMax[0] * 0.9, prm.qMin[1] * 0.9};
  const AnkleFkResult fkIn = ankleFk(prm, qIn, 0.0);
  if (fkIn.status == AnkleFkStatus::Ok) {
    double res = 0.0;
    for (int i = 0; i < kAnkleChains; ++i) {
      res = std::max(res,
        std::fabs(ankleConstraint(prm, i, fkIn.th5, fkIn.th6, qIn[i])));
    }
    std::printf("    q = (%+.0f, %+.0f) deg → (%+.2f, %+.2f) deg  拘束残差 %.2e mm²\n",
      qIn[0] * kDeg, qIn[1] * kDeg, fkIn.th5 * kDeg, fkIn.th6 * kDeg, res);
    check(res < 1e-6, "Ok を返した順変換は閉ループ拘束を満たす");
  }
}

// -------------------------------------------- 特異点まわり（2026-08-28 に追加）
/// 「足裏を前後に傾けすぎると姿勢が暴れる」の再発防止。ここが本体。
///
/// 押さえるのは 4 つ。どれか 1 つでも落ちたら実機で姿勢が飛ぶ。
///   [S1] 特異点が config に書いてある場所にある
///   [S2] 窓の中で Φ(θ6) が狭義単調（クランク ±90 deg の全域で）
///   [S3] サーボリミットの箱の全域で、順変換が有界な姿勢を返す（発散しない）
///   [S4] その出力が連続（クランクを少し動かして姿勢が跳ばない）
void testSingularity(const AnkleParams & prm)
{
  using namespace ankle_config;
  std::printf("\n型 2 特異点と順変換の頑健性\n");

  // ---- [S1] 純ピッチ経路で det Jθ が符号を変える場所 ----------------------
  double found[2] = {0.0, 0.0};
  int nFound = 0;
  double prev = 0.0;
  for (double a = -120.0; a <= 120.0; a += 0.05) {
    const double t6 = a / kDeg;
    const AnkleIkResult ik = ankleIk(prm, 0.0, t6);
    const double det = ankleJacobian(prm, ik.q, 0.0, t6).det;
    if (a > -120.0 && prev * det < 0.0 && nFound < 2) {found[nFound++] = a;}
    prev = det;
  }
  std::printf("    det Jθ = 0 が θ6 = %+.2f / %+.2f deg   config %+.1f / %+.1f\n",
    found[0], found[1], TH6_SINGULAR_DEG[0], TH6_SINGULAR_DEG[1]);
  // 寸法に依る量なので、仕様との照合ではなく「config と実物が合っているか」を見る
  check(nFound == 2 && std::fabs(found[0] - TH6_SINGULAR_DEG[0]) < 1.0 &&
    std::fabs(found[1] - TH6_SINGULAR_DEG[1]) < 1.0,
    "型 2 特異点が TH6_SINGULAR_DEG の場所にある");

  // ロッドは届いたままなので、Δ の監視では捕まらないことを明示しておく
  const AnkleIkResult atSing = ankleIk(prm, 0.0, TH6_SINGULAR_DEG[0] / kDeg, false);
  std::printf("    その姿勢の Δ = (%.0f, %.0f) mm² … 正のまま（Δ 監視では捕まらない）\n",
    atSing.delta[0], atSing.delta[1]);
  check(atSing.status == AnkleIkStatus::Ok,
    "特異点では Δ > 0（ロッド長では止まらない）");

  // 窓が特異点の内側にあること
  check(FK_WINDOW_DEG[0] > TH6_SINGULAR_DEG[0] && FK_WINDOW_DEG[1] < TH6_SINGULAR_DEG[1],
    "順変換の窓が特異点の内側にある");

  // ---- [S2] 窓の中で Φ が狭義単調か（クランク ±90 deg の全域） -------------
  const double lo = FK_WINDOW_DEG[0] / kDeg, hi = FK_WINDOW_DEG[1] / kDeg;
  const int nq = 40, nt = 120;
  int nonMono = 0, noCurve = 0;
  for (int i = 0; i <= nq; ++i) {
    for (int j = 0; j <= nq; ++j) {
      const double q[kAnkleChains] = {
        (-90.0 + 180.0 * i / nq) / kDeg, (-90.0 + 180.0 * j / nq) / kDeg};
      double f0 = 0.0;
      int sgn = 0;
      for (int k = 0; k <= nt; ++k) {
        bool ok = false;
        const double f = apdetail::phi(prm, q, lo + (hi - lo) * k / nt, ok);
        if (!ok) {++noCurve; break;}
        if (k > 0 && f != f0) {
          const int t = (f > f0) ? 1 : -1;
          if (sgn != 0 && t != sgn) {++nonMono; break;}
          sgn = t;
        }
        f0 = f;
      }
    }
  }
  std::printf("    窓 [%+.0f, %+.0f] × クランク箱 ±90 deg の %d 点: "
    "非単調 %d 件 / 曲線欠け %d 件\n",
    FK_WINDOW_DEG[0], FK_WINDOW_DEG[1], (nq + 1) * (nq + 1), nonMono, noCurve);
  check(nonMono == 0, "窓の中で Φ(θ6) が狭義単調（根は高々 1 個）");
  check(noCurve == 0, "窓の中で曲線 Θ_i が途切れない");

  // ---- [S3] サーボリミットの箱の全域で発散しない --------------------------
  const int nb = 120;
  int bad = 0, clamped = 0;
  double max5 = 0.0, max6 = 0.0, worstRes = 0.0;
  for (int i = 0; i <= nb; ++i) {
    for (int j = 0; j <= nb; ++j) {
      const double q[kAnkleChains] = {
        prm.qMin[0] + (prm.qMax[0] - prm.qMin[0]) * i / nb,
        prm.qMin[1] + (prm.qMax[1] - prm.qMin[1]) * j / nb};
      const AnkleFkResult r = ankleFk(prm, q, 0.0);
      if (r.status == AnkleFkStatus::Clamped) {
        ++clamped;
      } else if (r.status != AnkleFkStatus::Ok) {
        ++bad;
        continue;
      }
      max5 = std::max(max5, std::fabs(r.th5) * kDeg);
      max6 = std::max(max6, std::fabs(r.th6) * kDeg);
      if (r.status == AnkleFkStatus::Ok) {
        for (int c = 0; c < kAnkleChains; ++c) {
          worstRes = std::max(worstRes,
            std::fabs(ankleConstraint(prm, c, r.th5, r.th6, q[c])));
        }
      }
    }
  }
  std::printf("    サーボリミット箱 %d 点: 解けない %d 件 / 窓の縁に張り付き %d 件\n",
    (nb + 1) * (nb + 1), bad, clamped);
  std::printf("    出力の大きさ max|θ5| %.2f / max|θ6| %.2f deg（窓 %.0f）"
    "   拘束残差 %.1e mm²\n", max5, max6, FK_WINDOW_DEG[1], worstRes);
  check(bad == 0, "サーボリミットの範囲では必ず姿勢が出る（NoCurve/発散なし）");
  check(max6 <= FK_WINDOW_DEG[1] + 1e-9, "θ6 が窓の外に出ない");
  check(max5 <= TH5_MECH_LIMIT_DEG, "θ5 が機構限界を超えない");
  check(worstRes < 1e-6, "Ok を返した姿勢は閉ループ拘束を満たす");

  // ---- [S4] 連続性: クランクを 1 刻み動かして姿勢が跳ばないか ---------------
  const double stepDeg = (prm.qMax[0] - prm.qMin[0]) * kDeg / nb;
  double jump = 0.0, jq0 = 0.0, jq1 = 0.0;
  for (int i = 0; i < nb; ++i) {
    for (int j = 0; j <= nb; ++j) {
      const double q0 = prm.qMin[0] + (prm.qMax[0] - prm.qMin[0]) * i / nb;
      const double q1 = prm.qMin[1] + (prm.qMax[1] - prm.qMin[1]) * j / nb;
      const double a[kAnkleChains] = {q0, q1};
      const double b[kAnkleChains] = {
        prm.qMin[0] + (prm.qMax[0] - prm.qMin[0]) * (i + 1) / nb, q1};
      const AnkleFkResult ra = ankleFk(prm, a, 0.0), rb = ankleFk(prm, b, 0.0);
      const double d = std::max(std::fabs(ra.th5 - rb.th5),
        std::fabs(ra.th6 - rb.th6)) * kDeg;
      if (d > jump) {jump = d; jq0 = q0 * kDeg; jq1 = q1 * kDeg;}
    }
  }
  std::printf("    連続性: クランクを %.2f deg 動かしたときの姿勢の変化 最大 %.2f deg"
    "  (q ≈ %+.0f, %+.0f)\n", stepDeg, jump, jq0, jq1);
  // 中立まわりの利得は ∂θ/∂q ≈ 0.7 で、縁でも 2.5 倍を超えない
  check(jump < 2.5 * stepDeg, "隣り合うクランク角で姿勢が跳ばない");

  // ---- 種に依らないこと ----------------------------------------------------
  const AnkleIkResult ik = ankleIk(prm, 7.0 / kDeg, -20.0 / kDeg);
  double spread = 0.0;
  const AnkleFkResult base = ankleFk(prm, ik.q, 0.0);
  for (const double seed : {-55.0, -30.0, 0.0, 30.0, 55.0}) {
    const AnkleFkResult r = ankleFk(prm, ik.q, seed / kDeg);
    spread = std::max(spread, std::max(angleDiff(r.th5, base.th5),
      angleDiff(r.th6, base.th6)) * kDeg);
  }
  std::printf("    種を窓の端から端まで振ったときの解のばらつき %.2e deg\n", spread);
  check(spread < 1e-9, "種（前周期の θ6）が答えを選ばない");

  // ---- 指令側のエンベロープ ------------------------------------------------
  const AnkleClampResult e1 = ankleClampJoints(0.0, -80.0 / kDeg);
  const AnkleClampResult e2 = ankleClampJoints(0.0, -10.0 / kDeg);
  std::printf("    ankleClampJoints: θ6 -80 → %+.1f (clamped %d) / -10 → %+.1f (clamped %d)\n",
    e1.th6 * kDeg, static_cast<int>(e1.clamped), e2.th6 * kDeg,
    static_cast<int>(e2.clamped));
  check(e1.clamped && std::fabs(e1.th6 * kDeg - FK_WINDOW_DEG[0]) < 1e-9,
    "特異点の向こうへの指令は窓の縁に丸める");
  check(!e2.clamped, "窓の中の指令はそのまま通す");
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

  // 足裏基準の実測との突き合わせ。b_i は o6 原点、実測は足裏原点なので p6 を挟む。
  // ここが合っていないと、可視化に描く足裏とロッドの取り付け位置がずれる。
  const double ballH = prm.b[0].z - config::P6_Z;
  const double ballX = prm.b[0].x - config::P6_X;
  std::printf("    足裏 -> ボール軸  高さ %.3f mm（実測 %.1f） / 前後 %.3f mm（実測 %.1f）\n",
    ballH, ankle_config::BALL_HEIGHT_ABOVE_SOLE,
    ballX, ankle_config::BALL_X_FROM_SOLE_CENTER);
  check(std::fabs(ballH - ankle_config::BALL_HEIGHT_ABOVE_SOLE) < 1e-9,
    "ボール軸の足裏からの高さが実測と合う");
  check(std::fabs(ballX - ankle_config::BALL_X_FROM_SOLE_CENTER) < 1e-9,
    "ボール軸の前後位置が実測と合う");
  // 足首ピッチ軸そのものは足裏中心の真上（ずれているのはボール軸のほう）
  check(config::P6_X == 0.0 && config::P6_Y == 0.0,
    "足首ピッチ軸 o6 が足裏中心の真上にある");
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
  testSingularity(prm);

  std::printf("\n%s（失敗 %d 件）\n", g_fail == 0 ? "すべて通過" : "失敗あり", g_fail);
  return g_fail == 0 ? 0 : 1;
}
