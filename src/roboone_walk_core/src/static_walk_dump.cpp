// 静歩行の軌道を CSV で吐く。Python 版・JS 版との数値照合
// (tools/compare_walk_engines.py --engine static) に使う。
//
// 使い方:
//   static_walk_dump <vx> <vy> [t_walk=9.5] [t_end=20.0] [dt=0.005] [key=value ...]
//   static_walk_dump --params          # 既定値を key=value で出す (params.py との照合用)
// 指令プロファイルは 0.5 <= t < t_walk の間 (vx, vy)、それ以外 0。
// key=value は StaticGaitParams の上書き (v_max / a_max は "v_max=0.1,0.04")。
// 列は walk_dump と同じ。数値は %.17g (往復可能な精度) で出す。

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "roboone_walk_core/static_walk_engine.hpp"

using roboone_walk_core::StaticGaitParams;
using roboone_walk_core::StaticWalkEngine;
using roboone_walk_core::WalkOutputs;

static bool apply(StaticGaitParams & p, const char * kv)
{
  const char * eq = std::strchr(kv, '=');
  if (!eq) {return false;}
  const std::string key(kv, eq - kv);
  for (const auto & f : roboone_walk_core::staticGaitFields(p)) {
    if (key != f.name) {continue;}
    const char * s = eq + 1;
    for (int i = 0; i < f.n; ++i) {
      char * end = nullptr;
      f.ptr[i] = std::strtod(s, &end);
      if (end == s) {return false;}
      s = (*end == ',') ? end + 1 : end;
    }
    return true;
  }
  return false;
}

int main(int argc, char ** argv)
{
  StaticGaitParams p;
  if (argc == 2 && std::strcmp(argv[1], "--params") == 0) {
    for (const auto & f : roboone_walk_core::staticGaitFields(p)) {
      std::printf("%s=", f.name);
      for (int i = 0; i < f.n; ++i) {std::printf(i ? ",%.17g" : "%.17g", f.ptr[i]);}
      std::printf("\n");
    }
    return 0;
  }
  if (argc < 3) {
    std::fprintf(
      stderr, "usage: %s <vx> <vy> [t_walk=9.5] [t_end=20.0] [dt=0.005] [key=value ...]\n"
      "       %s --params\n", argv[0], argv[0]);
    return 2;
  }
  const double vx = std::atof(argv[1]);
  const double vy = std::atof(argv[2]);
  const double t_walk = argc > 3 ? std::atof(argv[3]) : 9.5;
  const double t_end = argc > 4 ? std::atof(argv[4]) : 20.0;
  const double dt = argc > 5 ? std::atof(argv[5]) : 0.005;
  for (int i = 6; i < argc; ++i) {
    if (!apply(p, argv[i])) {
      std::fprintf(stderr, "知らない設定か書式違い: %s\n", argv[i]);
      return 2;
    }
  }

  StaticWalkEngine eng{p};
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
