// C++ 実装の FK/IK 結果を CSV で吐く。scripts/crosscheck_cpp.py が
// Python 参照実装と突き合わせるために使う。
//
//   leg_dump <a3> <a4> <b> <sigma> <flipmask> <n> <seed>
//
// 1 行 = th0..th5, px,py,pz, R00..R22, ik0..ik5, status
#include <cstdio>
#include <cstdlib>
#include <random>

#include "roboone_kinematics/leg_kinematics.hpp"

using namespace roboone_kinematics;

int main(int argc, char ** argv)
{
  if (argc != 8) {
    std::fprintf(stderr, "usage: %s <a3> <a4> <b> <sigma> <flipmask> <n> <seed>\n", argv[0]);
    return 2;
  }
  LegParams prm = makeLegParams(Side::RIGHT);
  prm.a3 = std::atof(argv[1]);
  prm.a4 = std::atof(argv[2]);
  prm.b = std::atof(argv[3]);
  prm.sigma = std::atoi(argv[4]);
  const int mask = std::atoi(argv[5]);
  for (std::size_t k = 0; k < kNumJoints; ++k) {
    prm.sign[k] = ((mask >> k) & 1) ? -1.0 : 1.0;
  }
  prm.finalize();
  const int n = std::atoi(argv[6]);
  std::mt19937_64 rng(std::strtoull(argv[7], nullptr, 10));

  for (int i = 0; i < n; ++i) {
    double internal[kNumJoints];
    for (std::size_t k = 0; k < kNumJoints; ++k) {
      std::uniform_real_distribution<double> d(
        config::JOINT_LIMIT_LO_DEG[k] * M_PI / 180.0,
        config::JOINT_LIMIT_HI_DEG[k] * M_PI / 180.0);
      internal[k] = d(rng);
    }
    internal[KNEE] = prm.sigma * internal[KNEE] + prm.phi;
    // 乱数は文書の符号 (Σ_S) で作り、公開符号 (Σ_B) に直して fk/ik に渡す
    double th[kNumJoints];
    toSolverAngles(prm, internal, th);

    Vec3 p; Mat3 R;
    fk(prm, th, p, R);
    double out[kNumJoints] = {0, 0, 0, 0, 0, 0};
    const IkStatus st = ik(prm, p, R, out, /*clamp=*/false);

    for (double v : th) {std::printf("%.17g,", v);}
    std::printf("%.17g,%.17g,%.17g,", p.x, p.y, p.z);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {std::printf("%.17g,", R(r, c));}
    }
    for (double v : out) {std::printf("%.17g,", v);}
    std::printf("%d\n", static_cast<int>(st));
  }
  return 0;
}
