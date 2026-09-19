// 静歩行の計画エンジン static_walk の C++ 版。
//
// **roboone_walk_ref/static_walk/engine.py の機械移植で、Python 版が仕様の原本。**
// 設計判断と式の説明は engine.py の docstring にあるので、ここでは繰り返さない。
// ロジックを変えるときは Python / C++ / JS (roboone_viz/staticwalk.js) を揃え、
// tools/compare_walk_engines.py --engine static で数値一致 (許容 1e-6 m) を確かめること。
//
//   IDLE -> SHIFT (両足支持で重心を支持足の上へ) -> SWING (止めたまま反対の足を振り出す)
//        -> SHIFT -> ... -> 指令ゼロ: 足を揃える -> STOP (重心を中点へ) -> IDLE
//
// 出力は walk_core と同じ WalkOutputs。状態は State::SHIFT / SWING / STOP / IDLE / ESTOP。
// walk_core と同じく、時計も乱数も持たず update(vx, vy, dt) の入力列だけで決まる。
#ifndef ROBOONE_WALK_CORE__STATIC_WALK_ENGINE_HPP_
#define ROBOONE_WALK_CORE__STATIC_WALK_ENGINE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>
#include <string>
#include <vector>

#include "roboone_walk_core/walk_engine.hpp"

namespace roboone_walk_core
{

/// 静歩行の静的設定。roboone_walk_ref/static_walk/params.py の移植で、
/// 既定値は params.py と同じ (static_walk_dump --params で照合される)。実機が使う値は
/// motion ノードが読む config/static_gait.yaml で、調整ではそちらだけを変える。
/// 値の意味と根拠は params.py と static_gait.yaml の注記を見ること。
struct StaticGaitParams
{
  // --- 力学 -----------------------------------------------------------
  double z_c = 0.261;              // [m]
  double gravity = 9.81;           // [m/s^2]
  double foot_spacing = 0.140;     // [m]   home_pose.yaml の foot.y の 2 倍と揃える
  // --- 足裏 (IK の目標点が中心の長方形) --------------------------------
  double sole_length = 0.118;      // [m]
  double sole_width = 0.074;       // [m]
  // --- 重心移動 (SHIFT) -------------------------------------------------
  double zmp_tol = 0.010;          // [m]   重心の加速度で ZMP が重心の真下からずれてよい量
  double t_shift_min = 0.3;        // [s]
  double com_offset_y = 0.0;       // [m]   振り出し中の重心の横ずらし。+ で外側
  // --- 遊脚 (SWING) -----------------------------------------------------
  double t_swing = 0.6;            // [s]
  double swing_height = 0.025;     // [m]
  double td_overdrive = 0.004;     // [m]
  double td_speed_max = 0.20;      // [m/s]
  // --- 指令の解釈 -------------------------------------------------------
  double stride_time = 0.60;       // [s]   歩幅 = v·stride_time
  double v_max[2] = {0.10, 0.04};  // [m/s]
  double a_max[2] = {0.06, 0.03};  // [m/s^2]
  // --- 閾値 -------------------------------------------------------------
  double v_start_eps = 0.005;      // [m/s]
  double v_stop_eps = 0.010;       // [m/s]
  double loop_hz = 200.0;          // [Hz]

  double omega() const { return std::sqrt(gravity / z_c); }
};

/// StaticGaitParams の項目表。YAML の読み込み (motion ノード) と static_walk_dump の
/// key=value 上書きが使う。**項目を足したらここにも足すこと** (無いと読めない)。
struct StaticGaitField
{
  const char * name;
  double * ptr;
  int n;            //!< 要素数 (v_max / a_max は 2)
};

inline std::vector<StaticGaitField> staticGaitFields(StaticGaitParams & p)
{
  return {
    {"z_c", &p.z_c, 1}, {"gravity", &p.gravity, 1}, {"foot_spacing", &p.foot_spacing, 1},
    {"sole_length", &p.sole_length, 1}, {"sole_width", &p.sole_width, 1},
    {"zmp_tol", &p.zmp_tol, 1}, {"t_shift_min", &p.t_shift_min, 1},
    {"com_offset_y", &p.com_offset_y, 1},
    {"t_swing", &p.t_swing, 1}, {"swing_height", &p.swing_height, 1},
    {"td_overdrive", &p.td_overdrive, 1}, {"td_speed_max", &p.td_speed_max, 1},
    {"stride_time", &p.stride_time, 1}, {"v_max", p.v_max, 2}, {"a_max", p.a_max, 2},
    {"v_start_eps", &p.v_start_eps, 1}, {"v_stop_eps", &p.v_stop_eps, 1},
    {"loop_hz", &p.loop_hz, 1},
  };
}

namespace detail
{
constexpr double kStaticQuintic = 10.0 / 1.7320508075688772;  // 10/√3 (加速度ピーク係数)
constexpr double kSwingRise = 0.45;       // 振り出しのうち足を上げる区間 (walk_core と同じ)
// [m] 接地とみなす高さ。td_overdrive = 0 の「ちょうど 0 で着く」計画を、浮動小数の
// 丸め (+1e-18 のような値) で「届かない」と弾かないための許容 (Python 原本と同じ)
constexpr double kLandEps = 1e-9;

// 5 次多項式 s(u) と 1 階・2 階微分
inline void quintic3(double u, double & s, double & ds, double & dds)
{
  u = clamp(u, 0.0, 1.0);
  s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
  ds = 30.0 * u * u * (1.0 + u * (-2.0 + u));
  dds = 60.0 * u * (1.0 + u * (-3.0 + 2.0 * u));
}

// 振り出しの経過 tau での足の高さの基準 (飽和前) と、降下区間かどうか
inline double swingRef(const StaticGaitParams & p, double tau, bool & falling)
{
  if (tau < kSwingRise) {
    falling = false;
    return p.swing_height * quintic(tau / kSwingRise);
  }
  falling = true;
  const double su = quintic((tau - kSwingRise) / (1.0 - kSwingRise));
  return p.swing_height * (1.0 - su) - p.td_overdrive * su;
}
}  // namespace detail

/// 重心を dist [m] 動かすのにかける時間。ZMP のずれを zmp_tol に収める。
inline double staticShiftTime(const StaticGaitParams & p, double dist)
{
  const double w = p.omega();
  const double a_lim = w * w * p.zmp_tol;
  return std::max(p.t_shift_min, std::sqrt(detail::kStaticQuintic * dist / a_lim));
}

/// 振り出しの経過 tau での足の高さ。降下は td_speed_max で飽和する (前周期 z_prev から)。
inline double staticSwingHeight(const StaticGaitParams & p, double tau, double z_prev, double dt)
{
  bool falling = false;
  const double z_ref = detail::swingRef(p, tau, falling);
  return falling ? std::max(z_ref, z_prev - p.td_speed_max * dt) : z_ref;
}

/// check_static_gait() の結果。errors が空なら使える。
struct StaticGaitCheck
{
  bool lands = false;
  double touch_phase = 1.0;
  bool saturated = false;
  double z_end = 0.0;
  double v_need = 0.0;
  double t_shift_start = 0.0;     //!< 立位 -> 最初の支持足
  double t_shift_step = 0.0;      //!< 足 -> 足 (横移動なし)
  double t_cycle_fwd = 0.0;       //!< 全速前進の 1 歩
  double v_fwd_real = 0.0;        //!< 全速前進の実際の速さ
  double stride_max[2] = {0.0, 0.0};
  double margin_single[2] = {0.0, 0.0};
  std::vector<std::string> errors;
};

/// 設定が静歩行として成り立つかを調べる (Python の check_static_gait と同じ)。
inline StaticGaitCheck checkStaticGait(const StaticGaitParams & p)
{
  StaticGaitCheck r;
  const double dt = 1.0 / (p.loop_hz > 0.0 ? p.loop_hz : 200.0);
  r.v_need = (p.swing_height + p.td_overdrive) / ((1.0 - detail::kSwingRise) * p.t_swing);
  double z = 0.0, t = 0.0;
  while (t < p.t_swing - 1e-12) {
    t += dt;
    const double tau = std::min(t / p.t_swing, 1.0);
    bool falling = false;
    const double z_ref = detail::swingRef(p, tau, falling);
    if (falling && z - p.td_speed_max * dt > z_ref + 1e-12) {r.saturated = true;}
    z = staticSwingHeight(p, tau, z, dt);
    if (!r.lands && z <= detail::kLandEps) {
      r.lands = true;
      r.touch_phase = tau;
    }
  }
  r.z_end = z;

  const double w = p.foot_spacing;
  r.stride_max[0] = p.v_max[0] * p.stride_time;
  r.stride_max[1] = p.v_max[1] * p.stride_time;
  const double d_fwd = std::hypot(r.stride_max[0], w + 2.0 * p.com_offset_y);
  r.t_shift_start = staticShiftTime(p, std::abs(w / 2.0 + p.com_offset_y));
  r.t_shift_step = staticShiftTime(p, w + 2.0 * p.com_offset_y);
  r.t_cycle_fwd = staticShiftTime(p, d_fwd) + p.t_swing;
  r.v_fwd_real = r.stride_max[0] / r.t_cycle_fwd;
  r.margin_single[0] = p.sole_length / 2.0;
  r.margin_single[1] = p.sole_width / 2.0 - std::abs(p.com_offset_y);

  char buf[512];
  if (!r.lands) {
    std::snprintf(
      buf, sizeof(buf),
      "遊脚が床に届かないまま振り出しが終わる (+%.1fmm 浮いている)。td_speed_max を %.3f 以上に"
      "するか、swing_height を下げるか、t_swing を伸ばすこと", r.z_end * 1000.0, r.v_need);
    r.errors.emplace_back(buf);
  }
  if (r.margin_single[1] <= p.zmp_tol) {
    std::snprintf(
      buf, sizeof(buf),
      "com_offset_y %.1fmm では片足支持の重心が足裏の縁 (半幅 %.1fmm) から zmp_tol 以内に出る",
      p.com_offset_y * 1000.0, p.sole_width * 500.0);
    r.errors.emplace_back(buf);
  }
  if (p.zmp_tol <= 0.0 || p.t_swing <= 0.0 || p.stride_time <= 0.0) {
    r.errors.emplace_back("zmp_tol / t_swing / stride_time は正の値にすること");
  }
  if (p.foot_spacing <= p.sole_width) {
    std::snprintf(
      buf, sizeof(buf), "足間隔 %.0fmm が足裏の幅 %.0fmm 以下 (足が重なる)",
      w * 1000.0, p.sole_width * 1000.0);
    r.errors.emplace_back(buf);
  }
  return r;
}

namespace detail
{
inline double cross2(const Vec2 & o, const Vec2 & a, const Vec2 & b)
{
  return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
}

inline double segDist(const Vec2 & pt, const Vec2 & a, const Vec2 & b)
{
  const double ax = b[0] - a[0], ay = b[1] - a[1];
  const double l2 = ax * ax + ay * ay;
  const double t = l2 <= 0.0 ? 0.0 :
    clamp(((pt[0] - a[0]) * ax + (pt[1] - a[1]) * ay) / l2, 0.0, 1.0);
  return std::hypot(pt[0] - (a[0] + t * ax), pt[1] - (a[1] + t * ay));
}
}  // namespace detail

/// point (null なら ZMP) が支持多角形の縁からどれだけ内側にあるか [m]。負なら外。
/// 支持多角形は床に着いている足 (z <= 0) の足裏長方形の凸包。足のヨーは 0。
inline double supportMargin(
  const WalkOutputs & o, const StaticGaitParams & p, const Vec2 * point = nullptr)
{
  const Vec2 pt = point ? *point : o.zmp;
  const double hx = p.sole_length / 2.0, hy = p.sole_width / 2.0;
  std::vector<Vec2> pts;
  pts.reserve(8);
  for (const Vec3 * f : {&o.left_foot, &o.right_foot}) {
    if ((*f)[2] > 1e-9) {continue;}
    for (double sx : {-1.0, 1.0}) {
      for (double sy : {-1.0, 1.0}) {
        pts.push_back({(*f)[0] + sx * hx, (*f)[1] + sy * hy});
      }
    }
  }
  // 凸包 (monotone chain、反時計回り)
  std::sort(pts.begin(), pts.end());
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  if (pts.size() < 3) {return -std::numeric_limits<double>::infinity();}
  std::vector<Vec2> hull(2 * pts.size());
  std::size_t k = 0;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    while (k >= 2 && detail::cross2(hull[k - 2], hull[k - 1], pts[i]) <= 0.0) {--k;}
    hull[k++] = pts[i];
  }
  for (std::size_t i = pts.size() - 1, lo = k + 1; i-- > 0; ) {
    while (k >= lo && detail::cross2(hull[k - 2], hull[k - 1], pts[i]) <= 0.0) {--k;}
    hull[k++] = pts[i];
  }
  hull.resize(k - 1);
  if (hull.size() < 3) {return -std::numeric_limits<double>::infinity();}

  bool inside = true;
  double d_min = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const Vec2 & a = hull[i];
    const Vec2 & b = hull[(i + 1) % hull.size()];
    if (detail::cross2(a, b, pt) < 0.0) {inside = false;}
    d_min = std::min(d_min, detail::segDist(pt, a, b));
  }
  return inside ? d_min : -d_min;
}

class StaticWalkEngine
{
public:
  explicit StaticWalkEngine(const StaticGaitParams & params = StaticGaitParams{})
  : p_(params)
  {
    reset();
  }

  void reset()
  {
    const double w2 = p_.foot_spacing / 2.0;
    t_ = 0.0;
    state_ = State::IDLE;
    step_idx_ = 0;
    v_ = {0.0, 0.0};
    foot_left_ = {0.0, +w2};
    foot_right_ = {0.0, -w2};
    sup_ = LEFT;
    com_ = comv_ = coma_ = {0.0, 0.0};
    phase_ = t_local_ = dur_ = 0.0;
    c0_ = c1_ = swing_r0_ = {0.0, 0.0};
    swing_z_ = 0.0;
    p_land_ = {0.0, 0.0};
    mode_ = "walk";
    steps_.clear();
  }

  WalkOutputs update(double vx_cmd, double vy_cmd, double dt, bool estop = false)
  {
    t_ += dt;
    shape_cmd(vx_cmd, vy_cmd, dt);
    if (estop) {state_ = State::ESTOP;}
    switch (state_) {
      case State::IDLE: tick_idle(); break;
      case State::SHIFT:
      case State::STOP: tick_shift(dt); break;
      case State::SWING: tick_swing(dt); break;
      default: break;      // ESTOP: 脱力。復帰は reset() (実機では home 技) から
    }
    return outputs();
  }

  const StaticGaitParams & params() const { return p_; }

  /// 歩き出し・歩き続けのしきい値だけを差し替える（足踏みの包み用）。
  /// **幾何には一切触らない**ので、入れ切りしても軌道は跳ばない。負の値を入れると
  /// 「指令がどれだけ小さくても歩き続ける」= その場で歩を踏み続ける（歩幅は v·T の
  /// ままなので指令 0 で 0）。Python 版と JS 版は同じことを self.p / this.p の
  /// 差し替えでやっている（roboone_viz/record.py の MarchWalkEngine、
  /// template.html の MarchWalkEngineJS）。使うのは roboone_motion の WalkPlanner。
  void setStartStopEps(double start_eps, double stop_eps)
  {
    p_.v_start_eps = start_eps;
    p_.v_stop_eps = stop_eps;
  }
  const std::vector<StepRecord> & steps() const { return steps_; }

private:
  Vec2 & foot(int side) { return side == LEFT ? foot_left_ : foot_right_; }

  Vec2 midpoint() const
  {
    return {(foot_left_[0] + foot_right_[0]) / 2.0,
            (foot_left_[1] + foot_right_[1]) / 2.0};
  }

  bool moving(double eps) const { return std::hypot(v_[0], v_[1]) >= eps; }

  bool feet_aligned() const
  {
    return std::abs(foot_left_[0] - foot_right_[0]) < 1e-9 &&
           std::abs((foot_left_[1] - foot_right_[1]) - p_.foot_spacing) < 1e-9;
  }

  Vec2 over_support()
  {
    const Vec2 & f = foot(sup_);
    return {f[0], f[1] + sup_ * p_.com_offset_y};
  }

  // walk_core の shape_cmd と同じ (飽和・楕円・a_max)
  void shape_cmd(double vx, double vy, double dt)
  {
    vx = detail::clamp(vx, -p_.v_max[0], p_.v_max[0]);
    vy = detail::clamp(vy, -p_.v_max[1], p_.v_max[1]);
    const double s = std::hypot(vx / p_.v_max[0], vy / p_.v_max[1]);
    if (s > 1.0) {
      vx /= s;
      vy /= s;
    }
    const double vin[2] = {vx, vy};
    for (int k = 0; k < 2; ++k) {
      v_[k] += detail::clamp(vin[k] - v_[k], -p_.a_max[k] * dt, p_.a_max[k] * dt);
    }
  }

  void tick_idle()
  {
    com_ = midpoint();
    comv_ = coma_ = {0.0, 0.0};
    phase_ = 0.0;
    if (moving(p_.v_start_eps)) {
      // 横移動は進行方向側の足から踏み出す (walk_core と同じ)
      const double vy = v_[1];
      if (std::abs(vy) > 1e-6) {
        sup_ = vy > 0 ? RIGHT : LEFT;
      } else {
        sup_ = LEFT;
      }
      start_shift(over_support(), State::SHIFT);
    }
  }

  // ---------------------------------------------------------- 重心移動
  void start_shift(const Vec2 & target, State state)
  {
    state_ = state;
    c0_ = com_;
    c1_ = target;
    const double d = std::hypot(c1_[0] - c0_[0], c1_[1] - c0_[1]);
    dur_ = staticShiftTime(p_, d);
    t_local_ = 0.0;
    phase_ = 0.0;
  }

  void tick_shift(double dt)
  {
    t_local_ = std::min(t_local_ + dt, dur_);
    const double u = t_local_ / dur_;
    double s, ds, dds;
    detail::quintic3(u, s, ds, dds);
    for (int k = 0; k < 2; ++k) {
      const double dc = c1_[k] - c0_[k];
      com_[k] = c0_[k] + dc * s;
      comv_[k] = dc * ds / dur_;
      coma_[k] = dc * dds / (dur_ * dur_);
    }
    phase_ = u;
    if (t_local_ < dur_) {return;}
    com_ = c1_;
    comv_ = coma_ = {0.0, 0.0};
    if (state_ == State::STOP) {
      state_ = State::IDLE;
      phase_ = 0.0;
    } else if (!moving(p_.v_stop_eps) && feet_aligned()) {
      // 足が揃ったまま指令が消えた: 振り出さずに中点へ戻る
      start_shift(midpoint(), State::STOP);
    } else {
      start_swing();
    }
  }

  // ------------------------------------------------------------ 振り出し
  void start_swing()
  {
    state_ = State::SWING;
    ++step_idx_;
    t_local_ = 0.0;
    dur_ = p_.t_swing;
    phase_ = 0.0;
    const int swing = -sup_;
    swing_r0_ = foot(swing);
    swing_z_ = 0.0;
    const Vec2 ps = foot(sup_);
    if (moving(p_.v_stop_eps)) {
      mode_ = "walk";
      p_land_ = {ps[0] + v_[0] * p_.stride_time,
                 ps[1] + swing * p_.foot_spacing + v_[1] * p_.stride_time};
    } else {
      mode_ = "stop";     // 支持足の真横に揃える
      p_land_ = {ps[0], ps[1] + swing * p_.foot_spacing};
    }
    StepRecord r;
    r.step_idx = step_idx_;
    r.t_start = t_;
    r.support = sup_;
    r.mode = mode_;
    r.v = v_;
    r.p_support = ps;
    r.p_nom = p_land_;
    r.p_land = p_land_;
    steps_.push_back(r);
  }

  void tick_swing(double dt)
  {
    t_local_ = std::min(t_local_ + dt, dur_);
    phase_ = t_local_ / dur_;
    const double tau = phase_;
    const double s = detail::quintic(tau);
    const int swing = -sup_;
    Vec2 & f = foot(swing);
    f[0] = swing_r0_[0] + s * (p_land_[0] - swing_r0_[0]);
    f[1] = swing_r0_[1] + s * (p_land_[1] - swing_r0_[1]);
    swing_z_ = staticSwingHeight(p_, tau, swing_z_, dt);
    if (t_local_ < dur_) {return;}
    // 着地
    f = p_land_;
    swing_z_ = 0.0;
    steps_.back().t_end = t_;
    if (mode_ == "stop" || (!moving(p_.v_stop_eps) && feet_aligned())) {
      // 足が揃った: 重心を中点へ戻して止まる
      start_shift(midpoint(), State::STOP);
    } else {
      sup_ = swing;
      start_shift(over_support(), State::SHIFT);
    }
  }

  // -------------------------------------------------------------- 出力
  WalkOutputs outputs() const
  {
    WalkOutputs o;
    const double w = p_.omega();
    const bool swinging = state_ == State::SWING;
    const int swing = -sup_;
    o.t = t_;
    o.state = state_;
    o.step_idx = step_idx_;
    o.phase = phase_;
    o.support = swinging ? sup_ : 0;
    o.v = v_;
    for (int k = 0; k < 2; ++k) {
      o.xi[k] = com_[k] + comv_[k] / w;
      o.zmp[k] = com_[k] - coma_[k] / (w * w);
    }
    o.com = com_;
    o.left_foot = {foot_left_[0], foot_left_[1],
                   (swinging && swing == LEFT) ? swing_z_ : 0.0};
    o.right_foot = {foot_right_[0], foot_right_[1],
                    (swinging && swing == RIGHT) ? swing_z_ : 0.0};
    o.pelvis = {com_[0], com_[1], p_.z_c};
    if (swinging) {
      o.p_nom = p_land_;
      o.p_land = p_land_;
    }
    o.locked = swinging;
    o.stopping = (swinging && mode_ == "stop") || state_ == State::STOP;
    return o;
  }

  StaticGaitParams p_;
  double t_ = 0.0;
  State state_ = State::IDLE;
  int step_idx_ = 0;
  Vec2 v_{0.0, 0.0};
  Vec2 foot_left_{0.0, 0.0};
  Vec2 foot_right_{0.0, 0.0};
  int sup_ = LEFT;
  Vec2 com_{0.0, 0.0}, comv_{0.0, 0.0}, coma_{0.0, 0.0};
  double phase_ = 0.0;
  double t_local_ = 0.0;
  double dur_ = 0.0;
  Vec2 c0_{0.0, 0.0}, c1_{0.0, 0.0};
  Vec2 swing_r0_{0.0, 0.0};
  double swing_z_ = 0.0;
  Vec2 p_land_{0.0, 0.0};          // SWING 中だけ意味を持つ
  std::string mode_ = "walk";      // "walk" / "stop" (足を揃える歩)
  std::vector<StepRecord> steps_;
};

}  // namespace roboone_walk_core

#endif  // ROBOONE_WALK_CORE__STATIC_WALK_ENGINE_HPP_
