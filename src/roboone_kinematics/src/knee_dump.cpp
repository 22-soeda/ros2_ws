// C++ 実装の膝 4 節リンクの結果を CSV で吐く。
// scripts/crosscheck_knee.py が Python 参照実装 (scripts/knee_fourbar.py) と
// 突き合わせるために使う。独立に書いた 2 つの実装を比べるので、片方だけの
// 取り違えを拾える。
//
//   knee_dump <r1> <r2> <r3> <r4> <beta> <eps> <th4zero_deg> <sigma> <n> <lo_deg> <hi_deg>
//
// θ2 を lo..hi の間で n 点きざみに掃いて、1 行 =
//   t2, t3, t4, ratio, gamma, ik_t2, w3, w4, a3, a4, bend, servo, status
// 角はすべて rad。status は KneeStatus の値（0 = Ok）。
#include <cstdio>
#include <cstdlib>

#include "roboone_kinematics/knee_fourbar.hpp"

using namespace roboone_kinematics;

int main(int argc, char ** argv)
{
  if (argc != 12) {
    std::fprintf(
      stderr,
      "usage: %s <r1> <r2> <r3> <r4> <beta> <eps> <th4zero_deg> <sigma> "
      "<n> <lo_deg> <hi_deg>\n", argv[0]);
    return 2;
  }
  KneeParams prm;
  prm.r1 = std::atof(argv[1]);
  prm.r2 = std::atof(argv[2]);
  prm.r3 = std::atof(argv[3]);
  prm.r4 = std::atof(argv[4]);
  prm.beta = std::atoi(argv[5]);
  prm.eps = std::atoi(argv[6]);
  prm.theta4Zero = std::atof(argv[7]) * M_PI / 180.0;
  prm.sigmaJoint = std::atoi(argv[8]);
  const int n = std::atoi(argv[9]);
  const double lo = std::atof(argv[10]) * M_PI / 180.0;
  const double hi = std::atof(argv[11]) * M_PI / 180.0;

  for (int i = 0; i < n; ++i) {
    const double t2 = (n == 1) ? lo : lo + (hi - lo) * i / (n - 1);

    KneePose p;
    const KneeStatus st = kneeFk(prm, t2, p);
    if (st != KneeStatus::Ok) {
      std::printf("%.17g,0,0,0,0,0,0,0,0,0,0,0,%d\n", t2, static_cast<int>(st));
      continue;
    }
    double ratio = 0.0, w3 = 0.0, w4 = 0.0, a3 = 0.0, a4 = 0.0, servo = 0.0;
    const KneeStatus rst = kneeRatio(prm, p, ratio);
    kneeVelocity(prm, p, 1.0, w3, w4);
    kneeAcceleration(prm, p, 1.0, 1.0, a3, a4);

    KneePose back;
    const KneeStatus ist = kneeIk(prm, p.theta4, back);
    const double bend = kneeBendFromRocker(prm, p.theta4);
    const KneeStatus sst = kneeServoFromBend(prm, bend, servo);

    const int status = static_cast<int>(
      rst != KneeStatus::Ok ? rst : (ist != KneeStatus::Ok ? ist : sst));
    std::printf(
      "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%d\n",
      p.theta2, p.theta3, p.theta4, ratio, kneeTransmissionAngle(p), back.theta2,
      w3, w4, a3, a4, bend, servo, status);
  }
  return 0;
}
