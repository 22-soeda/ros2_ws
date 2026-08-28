// 足首パラレルリンクの中間量を出す確認用ツール。CAD の値を入れ替えたあとの点検に使う。
//
//   ros2 run roboone_kinematics ankle_dump [--th5 deg] [--th6 deg] [--right]
//   ros2 run roboone_kinematics ankle_dump --limits [--home c0 c1] [--right]
//
// ankle_selftest が「合っているか」を判定するのに対し、こちらは「どうなっているか」を
// 並べるだけ。判定はしない。
//
// --limits は特異点・順変換の窓・クランクリミットを並べる。--home に
// servo_home.yaml の home カウント（鎖1 = ID6、鎖2 = ID5 の順）を渡すと、
// servo_limits.yaml にそのまま貼れる生カウントまで出す。
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>

#include "roboone_kinematics/ankle_parallel.hpp"

using namespace roboone_kinematics;      // NOLINT(build/namespaces)

namespace
{
constexpr double kDeg = 180.0 / M_PI;

void printVec(const char * name, const Vec3 & v)
{
  std::printf("    %-8s (%+9.3f, %+9.3f, %+9.3f)\n", name, v.x, v.y, v.z);
}
}  // namespace

int main(int argc, char ** argv)
{
  double th5 = 0.0, th6 = 0.0;
  Side side = Side::LEFT;
  bool limits = false;
  int home[kAnkleChains] = {-1, -1};
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--th5") == 0 && i + 1 < argc) {
      th5 = std::atof(argv[++i]) / kDeg;
    } else if (std::strcmp(argv[i], "--th6") == 0 && i + 1 < argc) {
      th6 = std::atof(argv[++i]) / kDeg;
    } else if (std::strcmp(argv[i], "--right") == 0) {
      side = Side::RIGHT;
    } else if (std::strcmp(argv[i], "--limits") == 0) {
      limits = true;
    } else if (std::strcmp(argv[i], "--home") == 0 && i + 2 < argc) {
      home[0] = std::atoi(argv[++i]);
      home[1] = std::atoi(argv[++i]);
      limits = true;
    }
  }

  const AnkleParams prm = makeAnkleParams(side);

  if (limits) {
    using namespace ankle_config;
    std::printf("=== リミットと特異点 (%s脚) ===\n", side == Side::LEFT ? "左" : "右");
    std::printf("  型 2 特異点（純ピッチ）  θ6 = %+.1f / %+.1f deg\n",
      TH6_SINGULAR_DEG[0], TH6_SINGULAR_DEG[1]);
    std::printf("  順変換の窓               θ6 ∈ [%+.1f, %+.1f] deg"
      "  … この中で Φ は単調\n", FK_WINDOW_DEG[0], FK_WINDOW_DEG[1]);
    std::printf("  ロールの機構限界         θ5 = ±%.1f deg (Δ = 0)\n", TH5_MECH_LIMIT_DEG);
    std::printf("\n  鎖   クランク [deg]      サーボ角 [deg]        生カウント\n");
    for (int i = 0; i < kAnkleChains; ++i) {
      double a = ankleServoFromCrank(prm, i, prm.qMin[i]) * kDeg;
      double b = ankleServoFromCrank(prm, i, prm.qMax[i]) * kDeg;
      if (a > b) {std::swap(a, b);}
      std::printf("  %d   %+6.1f .. %+6.1f   %+7.2f .. %+7.2f   ",
        i + 1, prm.qMin[i] * kDeg, prm.qMax[i] * kDeg, a, b);
      if (home[i] >= 0) {
        std::printf("[%4d, %4d]  (home %d)\n",
          static_cast<int>(std::lround(home[i] + a * 4096.0 / 360.0)),
          static_cast<int>(std::lround(home[i] + b * 4096.0 / 360.0)), home[i]);
      } else {
        std::printf("--home を渡すと出る\n");
      }
    }
    std::printf("\n  貼り先: feetech_servo/config/servo_limits.yaml"
      "（鎖1 = ID6、鎖2 = ID5）\n\n");
  }

  std::printf("=== 幾何 (%s脚, ℓ5 = %.3f mm) ===\n", side == Side::LEFT ? "左" : "右", prm.l5);
  for (int i = 0; i < kAnkleChains; ++i) {
    std::printf("  鎖 %d\n", i + 1);
    printVec("a_i0", prm.a0[i]);
    printVec("O_i", prm.c[i]);
    printVec("b_i", prm.b[i]);
    std::printf("    %-8s r=%.2f  L=%.2f  ε=%+d  β=%+d\n",
      "", prm.r[i], prm.rod[i], prm.eps[i], prm.del[i]);
    std::printf("    %-8s q_neutral = %+.4f deg （T ポーズでのクランク角）\n",
      "", prm.qNeutral[i] * kDeg);
  }

  std::printf("\n=== 逆変換  (θ5, θ6) = (%+.2f, %+.2f) deg ===\n", th5 * kDeg, th6 * kDeg);
  const AnkleIkResult ik = ankleIk(prm, th5, th6);
  for (int i = 0; i < kAnkleChains; ++i) {
    const Vec3 ball = ankleBall(prm, i, th5, th6);
    const AnkleIkChain ch = ankleIkChain(prm, i, ball);
    std::printf("  鎖 %d\n", i + 1);
    printVec("B_i", ball);
    std::printf("    %-8s w = (%+.3f, %+.3f, %+.3f)   |w| = %.3f\n",
      "", ch.p, ch.h, ch.q, std::sqrt(ch.p * ch.p + ch.h * ch.h + ch.q * ch.q));
    std::printf("    %-8s s = %+.3f   ρ = %.3f   Δ = %.2f mm² (最大 %.0f)   余裕 %.3f mm\n",
      "", ch.s, ch.rho, ch.delta, prm.r[i] * prm.r[i], ch.margin);
    std::printf("    %-8s q = %+.4f deg   サーボ %+.4f deg   拘束残差 %.2e mm²\n",
      "", ik.q[i] * kDeg, ankleServoFromCrank(prm, i, ik.q[i]) * kDeg,
      ankleConstraint(prm, i, th5, th6, ik.q[i]));
  }
  std::printf("  status = %d\n", static_cast<int>(ik.status));

  std::printf("\n=== 順変換 ===\n");
  const AnkleFkResult fk = ankleFk(prm, ik.q, 0.0);
  std::printf("  種 0 から     → (%+.6f, %+.6f) deg   反復 %d 回   status %d"
    "  crankClamped=%d atWindow=%d\n",
    fk.th5 * kDeg, fk.th6 * kDeg, fk.iters, static_cast<int>(fk.status),
    static_cast<int>(fk.crankClamped), static_cast<int>(fk.atWindow));
  const AnkleFkResult sc = ankleFk(prm, ik.q, th6 + 30.0 / kDeg);
  std::printf("  種を 30 deg ずらして → (%+.6f, %+.6f) deg   status %d"
    "  （窓の中で根は 1 個なので同じ解に落ちる）\n",
    sc.th5 * kDeg, sc.th6 * kDeg, static_cast<int>(sc.status));
  std::printf("  往復誤差      %.2e deg\n",
    std::max(std::fabs(fk.th5 - th5), std::fabs(fk.th6 - th6)) * kDeg);

  std::printf("\n  鎖ごとの曲線 Θ_i(θ6)（この θ6 の近傍）\n");
  for (double d = -4.0; d <= 4.01; d += 2.0) {
    const double t = th6 + d / kDeg;
    const AnkleCurve c1 = ankleCurve(prm, 0, t, ik.q[0]);
    const AnkleCurve c2 = ankleCurve(prm, 1, t, ik.q[1]);
    std::printf("    θ6 %+7.2f :  Θ1 %s  Θ2 %s  Φ %+9.5f deg\n", t * kDeg,
      c1.ok ? "" : "(無し)", c2.ok ? "" : "(無し)",
      (c1.ok && c2.ok) ? (c1.th5 - c2.th5) * kDeg : 0.0);
  }

  std::printf("\n=== ヤコビアン ===\n");
  const AnkleJacobian J = ankleJacobian(prm, ik.q, th5, th6);
  std::printf("  Jθ = [ %+11.1f  %+11.1f ]\n", J.jt[0][0], J.jt[0][1]);
  std::printf("       [ %+11.1f  %+11.1f ]   det = %+.4e\n", J.jt[1][0], J.jt[1][1], J.det);
  std::printf("  Jq = diag(%+.1f, %+.1f)\n", J.jq[0], J.jq[1]);

  // ∂q/∂θ（数値微分）。中立まわりなら仕様 §8.4 の行列になる
  const double h = 1e-6;
  std::printf("\n  ∂q/∂θ（数値微分）\n");
  double G[2][2];
  for (int j = 0; j < 2; ++j) {
    const AnkleIkResult p = ankleIk(prm, th5 + (j == 0 ? h : 0.0), th6 + (j == 0 ? 0.0 : h));
    const AnkleIkResult m = ankleIk(prm, th5 - (j == 0 ? h : 0.0), th6 - (j == 0 ? 0.0 : h));
    for (int i = 0; i < 2; ++i) {G[i][j] = (p.q[i] - m.q[i]) / (2.0 * h);}
  }
  std::printf("       [ %+.4f  %+.4f ]\n", G[0][0], G[0][1]);
  std::printf("       [ %+.4f  %+.4f ]\n", G[1][0], G[1][1]);
  return 0;
}
