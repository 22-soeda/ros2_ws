// 足首パラレルリンクの中間量を吐く確認用ツール。
//
//   ankle_dump --pose  <θ5deg> <θ6deg>   関節角を与えて逆変換の中身を出す
//   ankle_dump --crank <q1deg> <q2deg>   クランク角を与えて順変換の中身を出す
//   ankle_dump --csv   <n> <seed>        往復を CSV で吐く（外部実装との突き合わせ用）
//   ... [--side left|right]
//
// CAD の値を ankle_config.hpp に入れたあと、死点余裕と det Jθ が設計可動域で
// 正のままかを見るのに使う（docs/足首パラレルリンク導出.pdf §9 の確認手順）。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>

#include "roboone_kinematics/ankle_parallel.hpp"

using namespace roboone_kinematics;

namespace
{

constexpr double kDeg = 180.0 / M_PI;

void printVec(const char * name, const Vec3 & v)
{
  std::printf("  %-6s (%9.4f, %9.4f, %9.4f)\n", name, v.x, v.y, v.z);
}

void printGeometry(const AnkleParams & prm)
{
  std::printf("幾何（ankle_config.hpp）  ℓ5 = %.3f mm\n", prm.l5);
  for (int i = 0; i < kAnkleChains; ++i) {
    std::printf("鎖 %d:  r = %.3f  L = %.4f  ε = %+d  δ = %+d\n",
      i + 1, prm.r[i], prm.rod[i], prm.eps[i], prm.del[i]);
    printVec("C", prm.c[i]);
    printVec("e", prm.e[i]);
    printVec("u", prm.u[i]);
    printVec("v", prm.v[i]);
    printVec("b", prm.b[i]);
  }
}

void dumpPose(const AnkleParams & prm, double th5, double th6)
{
  std::printf("\n関節角 θ5 = %+.4f deg, θ6 = %+.4f deg\n", th5 * kDeg, th6 * kDeg);

  const AnkleIkResult ik = ankleIk(prm, th5, th6);
  for (int i = 0; i < kAnkleChains; ++i) {
    const Vec3 ball = ankleBall(prm, i, th5, th6);
    const AnkleIkChain ch = ankleIkChain(prm, i, ball);
    std::printf("\n鎖 %d  (AP-7)\n", i + 1);
    printVec("B", ball);
    printVec("w", ball - prm.c[i]);
    std::printf(
      "  P = %10.4f  Q = %10.4f  S = %10.4f  ρ = %10.4f  h = %10.4f\n",
      ch.p, ch.q, ch.s, ch.rho, ch.h);
    std::printf(
      "  死点余裕 ρ-|S| = %8.4f mm    q = %+9.4f deg   |F| = %.3e mm²\n",
      ch.margin, ik.q[i] * kDeg, std::fabs(ankleConstraint(prm, i, th5, th6, ik.q[i])));
    printVec("A", ankleCrank(prm, i, ik.q[i]));
  }
  std::printf("\n逆変換 status = %d\n", static_cast<int>(ik.status));

  const AnkleJacobian J = ankleJacobian(prm, th5, th6, ik.q);
  std::printf("\nヤコビアン (AP-17)\n");
  std::printf("  Jθ = [%12.4e %12.4e]\n       [%12.4e %12.4e]   det = %.4e\n",
    J.jt[0][0], J.jt[0][1], J.jt[1][0], J.jt[1][1], J.det());
  std::printf("  Jq = diag(%12.4e, %12.4e)\n", J.jq[0], J.jq[1]);
  std::printf("  曲線の傾き dθ5/dθ6 = %+.4f, %+.4f\n",
    -J.jt[0][1] / J.jt[0][0], -J.jt[1][1] / J.jt[1][0]);

  // (AP-14)(AP-16) の消去量。D = 0 が型 2 特異点（文書 §5.2）。
  const Vec3 a[kAnkleChains] = {
    ankleCrank(prm, 0, ik.q[0]), ankleCrank(prm, 1, ik.q[1])};
  const AnkleElim el = ankleElim(prm, a, std::cos(th6), std::sin(th6));
  std::printf("\n消去 (AP-14)(AP-16)\n");
  std::printf("  N_c = %+.4e  N_s = %+.4e  D = %+.4e  G = %+.4e\n", el.nc, el.ns, el.d, el.g);
  std::printf("  c5 = N_c/D = %+.6f  s5 = N_s/D = %+.6f  (cosθ5 = %+.6f, sinθ5 = %+.6f)\n",
    el.nc / el.d, el.ns / el.d, std::cos(th5), std::sin(th5));

  const AnkleFkResult fk = ankleFk(prm, ik.q, th6);
  std::printf(
    "\n順変換で戻す: θ5 = %+.6f deg, θ6 = %+.6f deg  (%d 反復, 残差 %.2e rad, status %d)\n",
    fk.th5 * kDeg, fk.th6 * kDeg, fk.iters, fk.residual, static_cast<int>(fk.status));
}

void dumpCrank(const AnkleParams & prm, double q0, double q1)
{
  const double q[2] = {q0, q1};
  std::printf("\nクランク角 q1 = %+.4f deg, q2 = %+.4f deg\n", q0 * kDeg, q1 * kDeg);
  for (int i = 0; i < kAnkleChains; ++i) {
    printVec("A", ankleCrank(prm, i, q[i]));
    std::printf("  サーボ指令 φ = %+.4f deg\n", ankleServoFromCrank(prm, i, q[i]) * kDeg);
  }

  const AnkleFkResult fk = ankleFk(prm, q, 0.0);
  std::printf(
    "\n順変換（θ6 = 0 から）: θ5 = %+.6f deg, θ6 = %+.6f deg  (%d 反復, status %d)\n",
    fk.th5 * kDeg, fk.th6 * kDeg, fk.iters, static_cast<int>(fk.status));

  AnkleFkSolution sols[16];
  const int m = ankleFkAllSolutions(prm, q, sols, 16);
  std::printf("\n組める姿勢は %d 通り（文書 §4.4 は高々 8 通り）\n", m);
  for (int j = 0; j < m; ++j) {
    double worst = 0.0;
    for (int i = 0; i < kAnkleChains; ++i) {
      worst = std::max(
        worst, std::fabs(ankleConstraint(prm, i, sols[j].th5, sols[j].th6, q[i])));
    }
    std::printf(
      "  %d: θ5 = %+9.4f  θ6 = %+9.4f deg   det Jθ = %+.4e   max|F| = %.2e\n",
      j + 1, sols[j].th5 * kDeg, sols[j].th6 * kDeg, sols[j].det, worst);
  }
}

void dumpCsv(const AnkleParams & prm, int n, std::uint64_t seed)
{
  // 1 行 = th5, th6, q1, q2, margin1, margin2, detJt, fk_th5, fk_th6, ik_status, fk_status
  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> d5(
    ankle_config::TH5_LIMIT_DEG[0] / kDeg, ankle_config::TH5_LIMIT_DEG[1] / kDeg);
  std::uniform_real_distribution<double> d6(
    ankle_config::TH6_LIMIT_DEG[0] / kDeg, ankle_config::TH6_LIMIT_DEG[1] / kDeg);

  for (int k = 0; k < n; ++k) {
    const double th5 = d5(rng), th6 = d6(rng);
    const AnkleIkResult ik = ankleIk(prm, th5, th6);
    const AnkleJacobian J = ankleJacobian(prm, th5, th6, ik.q);
    const AnkleFkResult fk = ankleFk(prm, ik.q, 0.0);
    std::printf(
      "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d,%d\n",
      th5, th6, ik.q[0], ik.q[1], ik.margin[0], ik.margin[1], J.det(),
      fk.th5, fk.th6, static_cast<int>(ik.status), static_cast<int>(fk.status));
  }
}

[[noreturn]] void usage(const char * argv0)
{
  std::fprintf(
    stderr,
    "usage: %s --pose <th5deg> <th6deg> | --crank <q1deg> <q2deg> | --csv <n> <seed>\n"
    "          [--side left|right]\n", argv0);
  std::exit(2);
}

}  // namespace

int main(int argc, char ** argv)
{
  Side side = Side::RIGHT;
  const char * mode = nullptr;
  double a0 = 0.0, a1 = 0.0;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--side" && i + 1 < argc) {
      const std::string s = argv[++i];
      if (s == "left") {
        side = Side::LEFT;
      } else if (s != "right") {
        usage(argv[0]);
      }
    } else if ((arg == "--pose" || arg == "--crank" || arg == "--csv") && i + 2 < argc) {
      mode = argv[i];
      a0 = std::atof(argv[i + 1]);
      a1 = std::atof(argv[i + 2]);
      i += 2;
    } else {
      usage(argv[0]);
    }
  }
  if (!mode) {usage(argv[0]);}

  const AnkleParams prm = makeAnkleParams(side);
  if (!prm.valid()) {
    std::fprintf(stderr, "ankle_config.hpp のパラメータが前提を満たしていない\n");
    return 1;
  }

  if (std::strcmp(mode, "--csv") == 0) {
    dumpCsv(prm, static_cast<int>(a0), static_cast<std::uint64_t>(a1));
    return 0;
  }

  std::printf("%s脚\n", side == Side::RIGHT ? "右" : "左");
  printGeometry(prm);
  if (std::strcmp(mode, "--pose") == 0) {
    dumpPose(prm, a0 / kDeg, a1 / kDeg);
  } else {
    dumpCrank(prm, a0 / kDeg, a1 / kDeg);
  }
  return 0;
}
