// 歩行計画エンジン walk_core の C++ 版。
//
// **roboone_motion/walk_core/engine.py の機械移植で、Python 版が仕様の原本。**
// 設計判断・式の導出・文書との対応は engine.py の docstring に全部書いてあるので
// ここでは繰り返さない。ロジックを変えるときは必ず両方を揃え、
// tools/compare_walk_engines.py で数値一致 (許容 1e-6 m) を確認すること。
//
// Python 版と同じく:
//   * 世界座標 (初期立位の骨盤直下が原点、x 前・y 左)。回転なしの平行移動のみ
//   * 時計も乱数も持たず update(vx, vy, dt) の入力列だけで決定的
//   * IMU・推定なしの純フィードフォワード (ξ は常に計画値)
#ifndef ROBOONE_WALK_CORE__WALK_ENGINE_HPP_
#define ROBOONE_WALK_CORE__WALK_ENGINE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "roboone_walk_core/gait_params.hpp"

namespace roboone_walk_core
{

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;

enum class State { IDLE, START, STEP, STOP, ESTOP };

inline const char * to_string(State s)
{
  switch (s) {
    case State::IDLE: return "IDLE";
    case State::START: return "START";
    case State::STEP: return "STEP";
    case State::STOP: return "STOP";
    default: return "ESTOP";
  }
}

constexpr int LEFT = +1;   // 足の符号。+1 左 / -1 右 (y は左が正)
constexpr int RIGHT = -1;

namespace detail
{
inline double clamp(double v, double lo, double hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

// 5 次多項式 s(τ) = 10τ³ - 15τ⁴ + 6τ⁵ (式 14)
inline double quintic(double tau)
{
  tau = clamp(tau, 0.0, 1.0);
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}
}  // namespace detail

// クランプ域 (xmin, xmax, ymin, ymax)。世界座標
using ClampBox = std::array<double, 4>;

struct WalkOutputs
{
  double t = 0.0;
  State state = State::IDLE;
  int step_idx = 0;
  double phase = 0.0;             // 歩の位相 φ (START では押し出し経過)
  int support = 0;                // +1 左足支持 / -1 右足支持 / 0 両足
  Vec2 v{0.0, 0.0};               // 整形後の指令
  Vec2 xi{0.0, 0.0};              // DCM ξ
  Vec2 com{0.0, 0.0};             // 重心 x_C (水平成分)
  Vec2 zmp{0.0, 0.0};             // ZMP 参照 p
  Vec3 left_foot{0.0, 0.0, 0.0};
  Vec3 right_foot{0.0, 0.0, 0.0};
  Vec3 pelvis{0.0, 0.0, 0.0};     // (x_C, z_c)
  std::optional<Vec2> p_nom;      // 名目着地点 (式 5)
  std::optional<Vec2> p_land;     // 補正・クランプ後の着地点 (式 10, 11)
  std::optional<Vec2> b_next;     // DCM オフセット (式 8 の一般形)
  std::optional<Vec2> xi_eos;     // 歩の終端の ξ 予測 (式 9)
  std::optional<ClampBox> clamp_box;
  bool locked = false;
  bool stopping = false;          // 停止シーケンス中 (prep または stop の歩)

  // 骨盤水平座標系 {L} での足先目標 (IK 接続用)
  Vec3 left_foot_in_pelvis() const
  {
    return {left_foot[0] - pelvis[0], left_foot[1] - pelvis[1], left_foot[2] - pelvis[2]};
  }
  Vec3 right_foot_in_pelvis() const
  {
    return {right_foot[0] - pelvis[0], right_foot[1] - pelvis[1], right_foot[2] - pelvis[2]};
  }
};

struct StepRecord
{
  int step_idx = 0;
  double t_start = 0.0;
  int support = 0;
  std::string mode;               // "walk" / "prep" / "stop"
  Vec2 v{0.0, 0.0};
  Vec2 p_support{0.0, 0.0};
  std::optional<Vec2> p_nom;
  std::optional<Vec2> p_land;
  std::optional<Vec2> b_next;
  bool clamped = false;
  std::optional<double> t_end;
};

class WalkEngine
{
public:
  explicit WalkEngine(const GaitParams & params = GaitParams{})
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
    xi_ = {0.0, 0.0};
    com_ = {0.0, 0.0};
    zmp_ = {0.0, 0.0};
    phase_ = 0.0;
    t_local_ = 0.0;
    xi_ini_ = {0.0, 0.0};
    swing_r0_ = {0.0, 0.0};
    swing_z_ = 0.0;
    p_nom_.reset();
    p_land_.reset();
    b_next_.reset();
    xi_eos_.reset();
    clamp_box_.reset();
    locked_ = false;
    stopping_ = false;
    stop_prep_ = false;
    steps_.clear();
  }

  WalkOutputs update(double vx_cmd, double vy_cmd, double dt, bool estop = false)
  {
    t_ += dt;
    shape_cmd(vx_cmd, vy_cmd, dt);

    if (estop) {
      state_ = State::ESTOP;
    }
    if (state_ == State::ESTOP) {
      return outputs();          // 脱力。復帰は reset() (実機では home 技) から
    }

    switch (state_) {
      case State::IDLE: tick_idle(); break;
      case State::START: tick_start(dt); break;
      case State::STEP: tick_step(dt); break;
      case State::STOP: tick_stop(dt); break;
      default: break;
    }
    return outputs();
  }

  const GaitParams & params() const { return p_; }
  const std::vector<StepRecord> & steps() const { return steps_; }

private:
  Vec2 & foot(int side) { return side == LEFT ? foot_left_ : foot_right_; }
  const Vec2 & foot(int side) const { return side == LEFT ? foot_left_ : foot_right_; }

  // ------------------------------------------------------------ 指令の整形
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

  // ------------------------------------------- 1 歩のパラメータ (engine.py 参照)
  // 戻り値: (p_nom, b_here, b_next)
  void step_params(Vec2 & p_nom, Vec2 & b_here, Vec2 & b_next) const
  {
    const double lx = v_[0] * p_.t_step;
    const double ly = v_[1] * p_.t_step;
    const double w = p_.foot_spacing;
    const int s_next = -sup_;
    const Vec2 & ps = foot(sup_);
    p_nom = {ps[0] + lx, ps[1] + s_next * w + ly};
    const double ewt = p_.e_wt();
    const double denom = ewt * ewt - 1.0;
    const double l_first[2] = {lx, s_next * w + ly};
    const double l_second[2] = {lx, sup_ * w + ly};
    for (int k = 0; k < 2; ++k) {
      b_here[k] = (l_first[k] * ewt + l_second[k]) / denom;
      b_next[k] = (l_second[k] * ewt + l_first[k]) / denom;
    }
  }

  Vec2 clamp_landing(const Vec2 & raw, const Vec2 & p_nom)
  {
    const int s_next = -sup_;
    const double xmin = p_nom[0] - p_.step_clamp_x;
    const double xmax = p_nom[0] + p_.step_clamp_x;
    double ymin, ymax;
    if (s_next == LEFT) {          // 左足が着く: 外側 = +y
      ymin = p_nom[1] - p_.step_clamp_in;
      ymax = p_nom[1] + p_.step_clamp_out;
    } else {                       // 右足が着く: 外側 = -y
      ymin = p_nom[1] - p_.step_clamp_out;
      ymax = p_nom[1] + p_.step_clamp_in;
    }
    clamp_box_ = ClampBox{xmin, xmax, ymin, ymax};
    return {detail::clamp(raw[0], xmin, xmax), detail::clamp(raw[1], ymin, ymax)};
  }

  Vec2 predict_xi_eos() const
  {
    const double e = std::exp(p_.omega() * (p_.t_step - t_local_));
    const Vec2 & ps = foot(sup_);
    return {ps[0] + (xi_[0] - ps[0]) * e, ps[1] + (xi_[1] - ps[1]) * e};
  }

  void update_landing()
  {
    Vec2 p_nom, b_here, b;
    step_params(p_nom, b_here, b);
    const Vec2 xi_eos = predict_xi_eos();
    Vec2 raw;
    for (int k = 0; k < 2; ++k) {
      raw[k] = p_nom[k] + p_.k_dcm * (xi_eos[k] - (p_nom[k] + b[k]));
    }
    p_nom_ = p_nom;
    b_next_ = b;
    xi_eos_ = xi_eos;
    p_land_ = clamp_landing(raw, p_nom);
  }

  // 停止準備歩 (式 21)。engine.py _update_prep_landing 参照
  void update_prep_landing()
  {
    const int s_next = -sup_;
    const Vec2 & ps = foot(sup_);
    const Vec2 p_nom = {ps[0], ps[1] + s_next * p_.foot_spacing};
    const Vec2 xi_eos = predict_xi_eos();
    const Vec2 b_stop = {0.0, sup_ * (p_.foot_spacing / 2.0) / p_.e_wt()};
    const Vec2 raw = {xi_eos[0] - b_stop[0], xi_eos[1] - b_stop[1]};
    p_nom_ = p_nom;
    b_next_ = b_stop;
    xi_eos_ = xi_eos;
    p_land_ = clamp_landing(raw, p_nom);
  }

  // 停止の最後の歩。engine.py _update_stop_landing 参照
  void update_stop_landing()
  {
    const int s_next = -sup_;
    const Vec2 & ps = foot(sup_);
    const Vec2 p_nom = {ps[0], ps[1] + s_next * p_.foot_spacing};
    const Vec2 xi_eos = predict_xi_eos();
    const Vec2 raw = {2.0 * xi_eos[0] - ps[0], 2.0 * xi_eos[1] - ps[1]};
    p_nom_ = p_nom;
    b_next_.reset();
    xi_eos_ = xi_eos;
    p_land_ = clamp_landing(raw, p_nom);
  }

  // ---------------------------------------------------------- 歩の境界処理
  void enter_step()
  {
    state_ = State::STEP;
    step_idx_ += 1;
    phase_ = 0.0;
    t_local_ = 0.0;
    locked_ = false;
    zmp_ = foot(sup_);
    xi_ini_ = xi_;
    const int swing = -sup_;
    swing_r0_ = foot(swing);
    swing_z_ = 0.0;
    const bool v_small = std::hypot(v_[0], v_[1]) < p_.v_stop_eps;
    std::string mode;
    if (stop_prep_) {
      // 準備歩の次 = 足を揃える最後の歩。指令が復活しても完了させる
      mode = "stop";
      stopping_ = true;
      stop_prep_ = false;
      update_stop_landing();
      locked_ = true;
    } else if (v_small) {
      mode = "prep";
      stopping_ = false;
      stop_prep_ = true;
      update_prep_landing();
      locked_ = true;
    } else {
      mode = "walk";
      stopping_ = false;
      update_landing();
    }
    StepRecord r;
    r.step_idx = step_idx_;
    r.t_start = t_;
    r.support = sup_;
    r.mode = mode;
    r.v = v_;
    r.p_support = foot(sup_);
    steps_.push_back(r);
  }

  void finish_step_record()
  {
    if (steps_.empty()) return;
    StepRecord & r = steps_.back();
    r.p_nom = p_nom_;
    r.p_land = p_land_;
    r.b_next = b_next_;
    if (clamp_box_ && p_land_) {
      const ClampBox & b = *clamp_box_;
      const Vec2 & pl = *p_land_;
      r.clamped = (pl[0] == b[0] || pl[0] == b[1] || pl[1] == b[2] || pl[1] == b[3]);
    }
    r.t_end = t_;
  }

  // -------------------------------------------------------- 遊脚 (式 14〜16)
  Vec3 swing_pos(double dt)
  {
    const double tau = detail::clamp(phase_, 0.0, 1.0);
    const double s = detail::quintic(tau);
    const Vec2 & pl = *p_land_;
    const double x = swing_r0_[0] + s * (pl[0] - swing_r0_[0]);
    const double y = swing_r0_[1] + s * (pl[1] - swing_r0_[1]);
    double z;
    if (tau < 0.45) {
      z = p_.swing_height * detail::quintic(tau / 0.45);
      swing_z_ = z;
    } else {
      const double u = (tau - 0.45) / 0.55;
      const double z_ref =
        p_.swing_height * (1.0 - detail::quintic(u)) - p_.td_overdrive * detail::quintic(u);
      z = std::max(z_ref, swing_z_ - p_.td_speed_max * dt);
      swing_z_ = z;
    }
    return {x, y, z};
  }

  // ------------------------------------------------------------- DCM 積分
  void advance_dcm(double dt)
  {
    const double w = p_.omega();
    const double e = std::exp(w * t_local_);
    for (int k = 0; k < 2; ++k) {
      xi_[k] = zmp_[k] + (xi_ini_[k] - zmp_[k]) * e;
      com_[k] += w * (xi_[k] - com_[k]) * dt;
    }
  }

  // ----------------------------------------------------------------- IDLE
  void tick_idle()
  {
    const Vec2 mid = midpoint();
    xi_ = mid;
    com_ = mid;
    zmp_ = mid;
    p_nom_.reset();
    p_land_.reset();
    b_next_.reset();
    xi_eos_.reset();
    clamp_box_.reset();
    stopping_ = false;
    if (std::hypot(v_[0], v_[1]) >= p_.v_start_eps) {
      enter_start();
    }
  }

  void enter_start()
  {
    // 横移動があるときは進行方向側の足から踏み出す (engine.py _enter_start 参照)
    const double vy = v_[1];
    if (std::abs(vy) > 1e-6) {
      sup_ = vy > 0 ? RIGHT : LEFT;
    } else {
      sup_ = LEFT;
    }
    state_ = State::START;
    t_local_ = 0.0;
    phase_ = 0.0;
    xi_ini_ = xi_;
    zmp_ = foot(-sup_);            // ZMP は押し出し足 (式 19)
    stopping_ = false;
    stop_prep_ = false;
  }

  // ---------------------------------------------------------------- START
  void tick_start(double dt)
  {
    t_local_ += dt;
    phase_ = t_local_ / p_.start_pushoff_max;
    advance_dcm(dt);
    Vec2 p_nom, b_here, b_next;
    step_params(p_nom, b_here, b_next);
    p_nom_ = p_nom;
    b_next_ = b_next;
    p_land_.reset();
    xi_eos_.reset();
    clamp_box_.reset();
    const double target_y = foot(sup_)[1] + b_here[1];
    if (sup_ * (xi_[1] - target_y) >= 0.0) {
      // ξ が支持足の上に乗った。5 ms 離散の行き過ぎは e^{ωT} 倍に増幅されるので
      // 交差時刻を閉形式で解き、ξ を交差点ちょうどに置いてから歩に入る
      const double zy = zmp_[1];
      const double y0 = xi_ini_[1];
      if (std::abs(y0 - zy) > 1e-12 && (target_y - zy) / (y0 - zy) > 0.0) {
        const double e_star = (target_y - zy) / (y0 - zy);
        xi_[0] = zmp_[0] + (xi_ini_[0] - zmp_[0]) * e_star;
        xi_[1] = target_y;
      }
      enter_step();
    } else if (t_local_ > p_.start_pushoff_max) {
      state_ = State::STOP;        // 押し出し切れず。静かに立位へ戻す
    } else if (std::hypot(v_[0], v_[1]) < p_.v_stop_eps) {
      state_ = State::STOP;        // 押し出し中に指令が消えた
    }
  }

  // ----------------------------------------------------------------- STEP
  void tick_step(double dt)
  {
    // 歩の境界も閉形式で T ちょうどに取る (START の交差と同じ増幅対策)
    t_local_ = std::min(t_local_ + dt, p_.t_step);
    phase_ = t_local_ / p_.t_step;
    advance_dcm(dt);
    if (!locked_) {
      if (phase_ < p_.swing_lock_phase) {
        update_landing();
      } else {
        locked_ = true;
      }
    }
    const int swing = -sup_;
    const Vec3 sw = swing_pos(dt);
    foot(swing)[0] = sw[0];
    foot(swing)[1] = sw[1];
    if (t_local_ >= p_.t_step) {
      land(swing);
    }
  }

  void land(int swing)
  {
    finish_step_record();
    foot(swing) = *p_land_;
    swing_z_ = 0.0;
    if (stopping_) {
      state_ = State::STOP;
      p_nom_.reset();
      p_land_.reset();
      b_next_.reset();
      xi_eos_.reset();
      clamp_box_.reset();
    } else {
      sup_ = swing;                // 支持脚の交代
      enter_step();
    }
  }

  // ----------------------------------------------------------------- STOP
  void tick_stop(double dt)
  {
    Vec2 proj;
    double dist;
    project_between_feet(xi_, proj, dist);
    if (dist > p_.stop_outside_eps) {
      // 収束できない。ξ に近い側を支持足にしてもう 1 歩 (計画では通常来ない)
      sup_ = nearer_foot(xi_);
      v_ = {0.0, 0.0};
      enter_step();
      return;
    }
    zmp_ = proj;                   // ZMP を ξ に置く → ξ は動かない
    xi_ini_ = xi_;
    t_local_ = 0.0;
    advance_dcm(dt);               // ξ は不動、重心だけ ξ へ収束
    phase_ = 0.0;
    if (std::abs(xi_[0] - com_[0]) < p_.settle_eps &&
        std::abs(xi_[1] - com_[1]) < p_.settle_eps)
    {
      state_ = State::IDLE;
    }
  }

  // ------------------------------------------------------------------ 補助
  Vec2 midpoint() const
  {
    return {(foot_left_[0] + foot_right_[0]) / 2.0,
            (foot_left_[1] + foot_right_[1]) / 2.0};
  }

  int nearer_foot(const Vec2 & pt) const
  {
    const double dl = std::hypot(pt[0] - foot_left_[0], pt[1] - foot_left_[1]);
    const double dr = std::hypot(pt[0] - foot_right_[0], pt[1] - foot_right_[1]);
    return dl <= dr ? LEFT : RIGHT;
  }

  void project_between_feet(const Vec2 & pt, Vec2 & proj, double & dist) const
  {
    const Vec2 & a = foot_left_;
    const Vec2 & b = foot_right_;
    const double abx = b[0] - a[0], aby = b[1] - a[1];
    const double den = abx * abx + aby * aby;
    const double u = den == 0.0 ? 0.0 :
      detail::clamp(((pt[0] - a[0]) * abx + (pt[1] - a[1]) * aby) / den, 0.0, 1.0);
    proj = {a[0] + u * abx, a[1] + u * aby};
    dist = std::hypot(pt[0] - proj[0], pt[1] - proj[1]);
  }

  WalkOutputs outputs() const
  {
    WalkOutputs o;
    const bool in_step = state_ == State::STEP;
    const int swing = -sup_;
    o.t = t_;
    o.state = state_;
    o.step_idx = step_idx_;
    o.phase = phase_;
    o.support = in_step ? sup_ : 0;
    o.v = v_;
    o.xi = xi_;
    o.com = com_;
    o.zmp = zmp_;
    o.left_foot = {foot_left_[0], foot_left_[1],
                   (in_step && swing == LEFT) ? swing_z_ : 0.0};
    o.right_foot = {foot_right_[0], foot_right_[1],
                    (in_step && swing == RIGHT) ? swing_z_ : 0.0};
    o.pelvis = {com_[0], com_[1], p_.z_c};
    o.p_nom = p_nom_;
    o.p_land = p_land_;
    o.b_next = b_next_;
    o.xi_eos = xi_eos_;
    o.clamp_box = clamp_box_;
    o.locked = locked_;
    o.stopping = stopping_ || stop_prep_;
    return o;
  }

  GaitParams p_;
  double t_ = 0.0;
  State state_ = State::IDLE;
  int step_idx_ = 0;
  Vec2 v_{0.0, 0.0};
  Vec2 foot_left_{0.0, 0.0};
  Vec2 foot_right_{0.0, 0.0};
  int sup_ = LEFT;
  Vec2 xi_{0.0, 0.0};
  Vec2 com_{0.0, 0.0};
  Vec2 zmp_{0.0, 0.0};
  double phase_ = 0.0;
  double t_local_ = 0.0;
  Vec2 xi_ini_{0.0, 0.0};
  Vec2 swing_r0_{0.0, 0.0};
  double swing_z_ = 0.0;
  std::optional<Vec2> p_nom_;
  std::optional<Vec2> p_land_;
  std::optional<Vec2> b_next_;
  std::optional<Vec2> xi_eos_;
  std::optional<ClampBox> clamp_box_;
  bool locked_ = false;
  bool stopping_ = false;
  bool stop_prep_ = false;
  std::vector<StepRecord> steps_;
};

}  // namespace roboone_walk_core

#endif  // ROBOONE_WALK_CORE__WALK_ENGINE_HPP_
