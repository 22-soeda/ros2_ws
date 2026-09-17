// static_walk C++ 版の自己検算。roboone_walk_ref/test/test_static_walk.py の
// 主要な検査を C++ 側でも回す (数値の Python 一致は compare_walk_engines.py --engine static)。

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

#include "roboone_walk_core/static_walk_engine.hpp"

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

using Cmd = std::function<Vec2(double)>;

struct Profile
{
  const char * name;
  Cmd cmd;
};

static Cmd stopAfter(double t_stop, double vx, double vy)
{
  return [=](double t) {
           return (t >= 0.5 && t < t_stop) ? Vec2{vx, vy} : Vec2{0.0, 0.0};
         };
}

static std::vector<Profile> profiles()
{
  return {
    {"fwd", stopAfter(9.0, 0.10, 0.0)},
    {"back", stopAfter(9.0, -0.10, 0.0)},
    {"left", stopAfter(9.0, 0.0, 0.04)},
    {"right", stopAfter(9.0, 0.0, -0.04)},
    {"diag", stopAfter(9.0, 0.08, -0.025)},
    {"diag_back", stopAfter(9.0, -0.08, 0.025)},
    {"rev", [](double t) {
        if (t >= 0.5 && t < 7.0) {return Vec2{0.10, 0.0};}
        return t < 14.0 && t >= 7.0 ? Vec2{-0.10, 0.0} : Vec2{0.0, 0.0};
      }},
    {"stick", [](double t) {
        if (t < 0.5 || t >= 24.0) {return Vec2{0.0, 0.0};}
        return Vec2{0.10 * std::sin(0.37 * t), 0.04 * std::cos(0.23 * t)};
      }},
  };
}

static std::vector<WalkOutputs> run(StaticWalkEngine & e, const Cmd & cmd, double t_end)
{
  std::vector<WalkOutputs> outs;
  const int n = static_cast<int>(t_end / DT + 0.5);
  outs.reserve(n);
  for (int i = 0; i < n; ++i) {
    const Vec2 c = cmd(i * DT);
    outs.push_back(e.update(c[0], c[1], DT));
  }
  return outs;
}

static void checkStopped(const WalkOutputs & o, const StaticGaitParams & p, const char * what)
{
  CHECK(o.state == State::IDLE, "%s: state=%s", what, to_string(o.state));
  CHECK(o.left_foot[2] == 0.0 && o.right_foot[2] == 0.0, "%s: 足が浮いている", what);
  CHECK(std::abs(o.left_foot[0] - o.right_foot[0]) < 1e-9, "%s: 足が前後にずれている", what);
  CHECK(std::abs(o.left_foot[1] - o.right_foot[1] - p.foot_spacing) < 1e-9,
        "%s: 足間隔 %g", what, o.left_foot[1] - o.right_foot[1]);
  CHECK(std::abs(o.com[0] - (o.left_foot[0] + o.right_foot[0]) / 2) < 1e-12 &&
        std::abs(o.com[1] - (o.left_foot[1] + o.right_foot[1]) / 2) < 1e-12,
        "%s: 重心が中点に無い", what);
}

// 既定値が静歩行として成り立つ
static void test_defaults()
{
  const StaticGaitParams p;
  const StaticGaitCheck r = checkStaticGait(p);
  CHECK(r.errors.empty(), "errors=%zu (%s)", r.errors.size(),
        r.errors.empty() ? "" : r.errors[0].c_str());
  CHECK(r.lands, "遊脚が着かない");
  CHECK(std::abs(r.t_shift_step - 1.466) < 1e-3, "t_shift_step=%g", r.t_shift_step);
  // 壊れた設定は弾く
  StaticGaitParams bad;
  bad.t_swing = 0.2;
  bad.swing_height = 0.05;
  bad.td_speed_max = 0.10;
  CHECK(!checkStaticGait(bad).lands && !checkStaticGait(bad).errors.empty(), "着地しない設定");
  StaticGaitParams off;
  off.com_offset_y = 0.035;
  CHECK(!checkStaticGait(off).errors.empty(), "足裏の縁まで外へずらす設定");
}

// 静歩行の定義: 全時刻で ZMP と重心が支持多角形の中。足は遊脚だけが動き、飛ばない
static void test_static_stability()
{
  for (double offset : {0.0, 0.01, -0.01}) {
    StaticGaitParams p;
    p.com_offset_y = offset;
    const double need = p.sole_width / 2.0 - std::abs(offset) - p.zmp_tol - 1e-9;
    for (const Profile & pr : profiles()) {
      StaticWalkEngine e{p};
      const auto outs = run(e, pr.cmd, 30.0);
      double worst = 1.0, worst_c = 1.0, dev = 0.0;
      for (const auto & o : outs) {
        worst = std::min(worst, supportMargin(o, p));
        worst_c = std::min(worst_c, supportMargin(o, p, &o.com));
        dev = std::max(dev, std::hypot(o.zmp[0] - o.com[0], o.zmp[1] - o.com[1]));
        if (o.state == State::SWING) {
          const Vec3 & f = o.support == LEFT ? o.left_foot : o.right_foot;
          CHECK(std::abs(o.com[0] - f[0]) < 1e-12 &&
                std::abs(o.com[1] - (f[1] + o.support * offset)) < 1e-12,
                "%s t=%g: 振り出し中の重心が支持足の上に無い", pr.name, o.t);
        }
      }
      CHECK(worst >= need, "%s offset=%g: ZMP の余裕 %.2fmm", pr.name, offset, worst * 1000);
      CHECK(worst_c >= p.sole_width / 2.0 - std::abs(offset) - 1e-9,
            "%s offset=%g: 重心の余裕 %.2fmm", pr.name, offset, worst_c * 1000);
      CHECK(dev <= p.zmp_tol + 1e-9 && dev > 0.9 * p.zmp_tol,
            "%s: |ZMP-重心| 最大 %.3fmm", pr.name, dev * 1000);
      checkStopped(outs.back(), p, pr.name);

      for (std::size_t i = 1; i < outs.size(); ++i) {
        const WalkOutputs & a = outs[i - 1];
        const WalkOutputs & b = outs[i];
        const WalkOutputs & sw = (b.state == State::SWING) ? b : a;
        CHECK(std::hypot(b.com[0] - a.com[0], b.com[1] - a.com[1]) < 0.5 * DT,
              "%s t=%g: 重心が飛んだ", pr.name, b.t);
        const Vec3 * fa[2] = {&a.left_foot, &a.right_foot};
        const Vec3 * fb[2] = {&b.left_foot, &b.right_foot};
        const int side[2] = {LEFT, RIGHT};
        for (int k = 0; k < 2; ++k) {
          const bool moved = (*fa[k])[0] != (*fb[k])[0] || (*fa[k])[1] != (*fb[k])[1];
          if (moved) {
            CHECK(sw.state == State::SWING && sw.support == -side[k],
                  "%s t=%g: 遊脚でない足が動いた", pr.name, b.t);
          }
          CHECK(std::abs((*fb[k])[2] - (*fa[k])[2]) <= p.td_overdrive + 1e-9,
                "%s t=%g: 足の高さが飛んだ", pr.name, b.t);
        }
      }
    }
  }
}

// どの時刻で指令を離しても、足を揃えて中点に立つ
static void test_stop_anytime()
{
  const StaticGaitParams p;
  const double cmds[][2] = {{0.10, 0.0}, {0.08, -0.025}, {0.0, 0.04}};
  for (const auto & c : cmds) {
    for (int k = 0; k < 24; ++k) {
      const double t_stop = 0.52 + 0.37 * k;
      StaticWalkEngine e{p};
      const auto outs = run(e, stopAfter(t_stop, c[0], c[1]), t_stop + 12.0);
      char what[64];
      std::snprintf(what, sizeof(what), "cmd=(%g,%g) t_stop=%.2f", c[0], c[1], t_stop);
      checkStopped(outs.back(), p, what);
      // 整形後の速度が v_stop_eps を下回ったあとに始まる歩は、足を揃える 1 歩だけ
      double t_quiet = 1e9;
      for (const auto & o : outs) {
        if (o.t > t_stop && std::hypot(o.v[0], o.v[1]) < p.v_stop_eps) {
          t_quiet = o.t;
          break;
        }
      }
      int after = 0;
      for (const auto & r : e.steps()) {
        if (r.t_start < t_quiet) {continue;}
        ++after;
        CHECK(r.mode == "stop", "%s: 止まる途中で歩いた (step %d)", what, r.step_idx);
      }
      CHECK(after <= 1, "%s: 離したあと %d 歩", what, after);
    }
  }
}

// 着地点は振り出しの開始で決まり、遊脚中は動かない
static void test_stride_latched()
{
  const StaticGaitParams p;
  StaticWalkEngine e{p};
  const auto outs = run(
    e, [](double t) {
      return (static_cast<int>(t * 3) % 2 == 0) ? Vec2{0.10, 0.0} : Vec2{-0.10, 0.03};
    }, 25.0);
  for (std::size_t i = 1; i < outs.size(); ++i) {
    if (outs[i - 1].state == State::SWING && outs[i].state == State::SWING) {
      CHECK(*outs[i - 1].p_land == *outs[i].p_land, "t=%g: 遊脚中に着地点が動いた", outs[i].t);
    }
  }
  int walked = 0;
  for (const auto & r : e.steps()) {
    if (r.mode != "walk") {continue;}
    ++walked;
    const int swing = -r.support;
    CHECK(std::abs((*r.p_land)[0] - (r.p_support[0] + r.v[0] * p.stride_time)) < 1e-12,
          "step %d x", r.step_idx);
    CHECK(std::abs((*r.p_land)[1] -
                   (r.p_support[1] + swing * p.foot_spacing + r.v[1] * p.stride_time)) < 1e-12,
          "step %d y", r.step_idx);
  }
  CHECK(walked >= 5, "walked=%d", walked);
}

// 決定性・ESTOP
static void test_deterministic_and_estop()
{
  StaticWalkEngine a, b;
  const Cmd c = profiles().back().cmd;
  for (int i = 0; i < 2400; ++i) {
    const Vec2 v = c(i * DT);
    const auto oa = a.update(v[0], v[1], DT);
    const auto ob = b.update(v[0], v[1], DT);
    CHECK(oa.com == ob.com && oa.left_foot == ob.left_foot && oa.state == ob.state,
          "diverged at i=%d", i);
  }

  StaticWalkEngine e;
  for (int i = 0; i < 600; ++i) {e.update(0.1, 0.0, DT);}
  const auto o1 = e.update(0.1, 0.0, DT, true);
  const auto o2 = e.update(0.1, 0.0, DT, false);
  CHECK(o1.state == State::ESTOP && o2.state == State::ESTOP, "estop latch");
  CHECK(o1.left_foot == o2.left_foot && o1.com == o2.com, "estop freeze");
  e.reset();
  CHECK(e.update(0.0, 0.0, DT).state == State::IDLE, "reset");
}

int main()
{
  test_defaults();
  test_static_stability();
  test_stop_anytime();
  test_stride_latched();
  test_deterministic_and_estop();
  if (g_failures == 0) {
    std::printf("static_walk_selftest: all OK\n");
    return 0;
  }
  std::printf("static_walk_selftest: %d failure(s)\n", g_failures);
  return 1;
}
