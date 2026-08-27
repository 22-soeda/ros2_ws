// 歩行軌道を CSV で吐く。Python 版・JS 版との数値照合
// (tools/compare_walk_engines.py) と、後段 (IK・リプレイ) への受け渡しに使う。
//
// 使い方:
//   walk_dump <vx> <vy> [t_walk=4.5] [t_end=8.0] [dt=0.005]
// 指令プロファイルは 0.5 <= t < t_walk の間 (vx, vy)、それ以外 0 (Python 版の
// 既定シナリオと同じ形)。数値は %.17g (往復可能な精度) で出す。

#include <cstdio>
#include <cstdlib>

#include "roboone_walk_core/walk_engine.hpp"

using roboone_walk_core::GaitParams;
using roboone_walk_core::WalkEngine;
using roboone_walk_core::WalkOutputs;

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <vx> <vy> [t_walk=4.5] [t_end=8.0] [dt=0.005]\n", argv[0]);
    return 2;
  }
  const double vx = std::atof(argv[1]);
  const double vy = std::atof(argv[2]);
  const double t_walk = argc > 3 ? std::atof(argv[3]) : 4.5;
  const double t_end = argc > 4 ? std::atof(argv[4]) : 8.0;
  const double dt = argc > 5 ? std::atof(argv[5]) : 0.005;

  WalkEngine eng{GaitParams{}};
  const int n = static_cast<int>(t_end / dt + 0.5);
  std::printf("t,st,ph,sup,vx,vy,xix,xiy,comx,comy,zx,zy,lfx,lfy,lfz,rfx,rfy,rfz\n");
  for (int i = 0; i < n; ++i) {
    const double t = i * dt;
    const bool on = (t >= 0.5 && t < t_walk);
    const WalkOutputs o = eng.update(on ? vx : 0.0, on ? vy : 0.0, dt);
    std::printf(
      "%.17g,%d,%.17g,%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
      "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
      o.t, static_cast<int>(o.state), o.phase, o.support, o.v[0], o.v[1],
      o.xi[0], o.xi[1], o.com[0], o.com[1], o.zmp[0], o.zmp[1],
      o.left_foot[0], o.left_foot[1], o.left_foot[2],
      o.right_foot[0], o.right_foot[1], o.right_foot[2]);
  }
  return 0;
}
