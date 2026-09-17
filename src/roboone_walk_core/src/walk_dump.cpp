// 歩行軌道を CSV で吐く。Python 版・JS 版との数値照合
// (tools/compare_walk_engines.py) と、後段 (IK・リプレイ) への受け渡しに使う。
//
// 使い方:
//   walk_dump <vx> <vy> [t_walk=4.5] [t_end=8.0] [dt=0.005] [key=value ...]
// 指令プロファイルは 0.5 <= t < t_walk の間 (vx, vy)、それ以外 0 (Python 版の
// 既定シナリオと同じ形)。key=value は GaitParams の上書き (下の kKeys にある数値だけ)。
// 数値は %.17g (往復可能な精度) で出す。

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "roboone_walk_core/walk_engine.hpp"

using roboone_walk_core::GaitParams;
using roboone_walk_core::WalkEngine;
using roboone_walk_core::WalkOutputs;

static bool apply(GaitParams & p, const char * kv)
{
  const struct {const char * name; double * ptr;} kKeys[] = {
    {"ds_time", &p.ds_time}, {"t_step", &p.t_step}, {"foot_spacing", &p.foot_spacing},
    {"swing_height", &p.swing_height}, {"k_dcm", &p.k_dcm},
    {"a_max_x", &p.a_max[0]}, {"a_max_y", &p.a_max[1]},
  };
  const char * eq = std::strchr(kv, '=');
  if (!eq) {return false;}
  const std::string key(kv, eq - kv);
  for (const auto & k : kKeys) {
    if (key == k.name) {
      *k.ptr = std::atof(eq + 1);
      return true;
    }
  }
  return false;
}

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(
      stderr, "usage: %s <vx> <vy> [t_walk=4.5] [t_end=8.0] [dt=0.005] [key=value ...]\n",
      argv[0]);
    return 2;
  }
  const double vx = std::atof(argv[1]);
  const double vy = std::atof(argv[2]);
  // 位置引数は '=' を含まないものだけ数える
  int npos = 0;
  double pos[3] = {4.5, 8.0, 0.005};
  GaitParams p;
  for (int i = 3; i < argc; ++i) {
    if (std::strchr(argv[i], '=')) {
      if (!apply(p, argv[i])) {
        std::fprintf(stderr, "unknown key: %s\n", argv[i]);
        return 2;
      }
    } else if (npos < 3) {
      pos[npos++] = std::atof(argv[i]);
    }
  }
  const double t_walk = pos[0];
  const double t_end = pos[1];
  const double dt = pos[2];

  WalkEngine eng{p};
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
