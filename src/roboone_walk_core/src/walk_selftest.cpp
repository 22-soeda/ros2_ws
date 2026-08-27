// walk_core C++ 版の自己検算。roboone_motion/test/test_walk_core.py の
// 主要な検査を C++ 側でも回す (数値の Python 一致は compare_walk_engines.py)。

#include <cmath>
#include <cstdio>
#include <vector>

#include "roboone_walk_core/walk_engine.hpp"

using namespace roboone_walk_core;  // NOLINT

static int g_failures = 0;

#define CHECK(cond, ...) \
  do { \
    if (!(cond)) { \
      ++g_failures; \
      std::printf("FAIL %s:%d  %s  ", __FILE__, __LINE__, #cond); \
      std::printf(__VA_ARGS__); \
      std::printf("\n"); \
    } \
  } while (0)

static const double DT = 0.005;

static std::vector<WalkOutputs> run(WalkEngine & e, double vx, double vy,
                                    double t_walk, double t_end)
{
  std::vector<WalkOutputs> outs;
  const int n = static_cast<int>(t_end / DT + 0.5);
  outs.reserve(n);
  for (int i = 0; i < n; ++i) {
    const double t = i * DT;
    const bool on = (t >= 0.5 && t < t_walk);
    outs.push_back(e.update(on ? vx : 0.0, on ? vy : 0.0, DT));
  }
  return outs;
}

// 全方向: 完走して足が揃い、ξ が中点に来る。足は交差しない
static void test_converges()
{
  const GaitParams p;
  const double cases[][2] = {
    {0.10, 0.0}, {0.15, 0.0}, {-0.10, 0.0},
    {0.0, 0.06}, {0.0, -0.06}, {0.08, 0.05}, {-0.08, -0.05},
  };
  for (const auto & c : cases) {
    WalkEngine e;
    const auto outs = run(e, c[0], c[1], 4.5, 8.5);
    const WalkOutputs & o = outs.back();
    CHECK(o.state == State::IDLE, "cmd=(%g,%g) state=%s", c[0], c[1], to_string(o.state));
    const double gap_y = o.left_foot[1] - o.right_foot[1];
    const double gap_x = o.left_foot[0] - o.right_foot[0];
    CHECK(std::abs(gap_y - p.foot_spacing) < 1e-3, "gap_y=%g", gap_y);
    CHECK(std::abs(gap_x) < 1e-3, "gap_x=%g", gap_x);
    const double mx = (o.left_foot[0] + o.right_foot[0]) / 2;
    const double my = (o.left_foot[1] + o.right_foot[1]) / 2;
    CHECK(std::abs(o.xi[0] - mx) < 2e-3 && std::abs(o.xi[1] - my) < 2e-3,
          "xi=(%g,%g) mid=(%g,%g)", o.xi[0], o.xi[1], mx, my);
    for (const auto & oo : outs) {
      CHECK(std::isfinite(oo.xi[0]) && std::isfinite(oo.xi[1]), "NaN at t=%g", oo.t);
      CHECK(oo.left_foot[1] - oo.right_foot[1] > 0.03,
            "feet crossed at t=%g (cmd %g,%g)", oo.t, c[0], c[1]);
    }
  }
}

// 決定性: 同じ入力列で同じ出力列
static void test_deterministic()
{
  WalkEngine a, b;
  for (int i = 0; i < 1200; ++i) {
    const double t = i * DT;
    const double vx = 0.08 * std::sin(t);
    const double vy = 0.04 * std::cos(1.3 * t);
    const auto oa = a.update(vx, vy, DT);
    const auto ob = b.update(vx, vy, DT);
    CHECK(oa.xi == ob.xi && oa.com == ob.com && oa.left_foot == ob.left_foot,
          "diverged at i=%d", i);
  }
}

// 着地点がクランプ域 (式 11) に収まる
static void test_clamp()
{
  const GaitParams p;
  WalkEngine e;
  for (int i = 0; i < 1600; ++i) {
    const double t = i * DT;
    const bool flip = (static_cast<int>(t * 2) % 2) == 0;
    e.update(flip ? 0.15 : -0.15, flip ? 0.08 : -0.08, DT);
  }
  int walked = 0;
  for (const auto & r : e.steps()) {
    if (!r.p_land || !r.p_nom) continue;
    ++walked;
    CHECK(std::abs((*r.p_land)[0] - (*r.p_nom)[0]) <= p.step_clamp_x + 1e-9,
          "step %d dx", r.step_idx);
    const double dy = (*r.p_land)[1] - (*r.p_nom)[1];
    CHECK(dy >= -p.step_clamp_out - 1e-9 && dy <= p.step_clamp_out + 1e-9,
          "step %d dy=%g", r.step_idx, dy);
  }
  CHECK(walked > 5, "walked=%d", walked);
}

// ESTOP: 凍結し、解除は reset のみ
static void test_estop()
{
  WalkEngine e;
  for (int i = 0; i < 200; ++i) e.update(0.1, 0.0, DT);
  const auto o1 = e.update(0.1, 0.0, DT, true);
  const auto o2 = e.update(0.1, 0.0, DT, false);
  CHECK(o1.state == State::ESTOP && o2.state == State::ESTOP, "estop latch");
  CHECK(o1.left_foot == o2.left_foot && o1.xi == o2.xi, "estop freeze");
}

int main()
{
  test_converges();
  test_deterministic();
  test_clamp();
  test_estop();
  if (g_failures == 0) {
    std::printf("walk_selftest: all OK\n");
    return 0;
  }
  std::printf("walk_selftest: %d failure(s)\n", g_failures);
  return 1;
}
