// 歩行計画の切り替え口 — 動歩行 (walk_core) と静歩行 (static_walk) を起動時に選ぶ。
//
// **モードは起動時に 1 回だけ決める**（motion ノードの walk_mode パラメータ。launch の
// walk_mode:=dynamic|static）。実行中には切り替えない。どちらの計画器も出力は同じ
// rwc::WalkOutputs なので、下流（足先の組み立て・IK・安定化・/motion/stab）は
// モードを知らずに済む。モードで違うのは次の 3 つだけで、全部ここに集めてある:
//
//   * どの計画器を回すか                     WalkPlanner
//   * 計画上の立位の足間隔と重心高さ         WalkSetup::planFootSpacing() / planZc()
//   * 遊脚の時間と接地の位相 (安定化のゲート) WalkSetup::swingTiming()
//
// 状態の番号 (rwc::State) は共通で、SHIFT / SWING は静歩行だけ、START / STEP は
// 動歩行だけが出す。片足で立っている周期は singleSupport() で見る。
//
// ROS を知らない（motion_selftest から叩ける）。
#ifndef ROBOONE_MOTION__WALK_PLANNER_HPP_
#define ROBOONE_MOTION__WALK_PLANNER_HPP_

#include <string>

#include "roboone_motion/body_pose.hpp"
#include "roboone_motion/side.hpp"
#include "roboone_walk_core/static_walk_engine.hpp"
#include "roboone_walk_core/walk_engine.hpp"

namespace roboone_motion
{

namespace rwc = roboone_walk_core;

enum class WalkMode { Dynamic, Static };

inline const char * walkModeName(WalkMode m)
{
  return m == WalkMode::Static ? "static" : "dynamic";
}

/// "dynamic" / "static" を読む。それ以外は false（黙って既定へ落とさない）。
inline bool parseWalkMode(const std::string & s, WalkMode & out)
{
  if (s == "dynamic") {
    out = WalkMode::Dynamic;
    return true;
  }
  if (s == "static") {
    out = WalkMode::Static;
    return true;
  }
  return false;
}

/// 遊脚を振っている時間と、その中で床に着く位相。安定化の着地ゲートが使う。
struct SwingTiming
{
  double duration = 0.6;       //!< [s] 遊脚の時間 (動歩行 t_step / 静歩行 t_swing)
  double touch_phase = 1.0;    //!< 床に着く位相 [0,1]。着かない設定なら 1
};

/// 歩行の設定一式。motion ノードが起動時に詰める。使わないほうの設定も持っていてよい。
struct WalkSetup
{
  WalkMode mode = WalkMode::Dynamic;
  rwc::GaitParams gait;            //!< 動歩行 (gait.yaml)
  rwc::StaticGaitParams stat;      //!< 静歩行 (static_gait.yaml)

  bool isStatic() const {return mode == WalkMode::Static;}
  /// 計画上の立位の足間隔 [m]。実機の足はここからホーム姿勢の足へ平行移動する
  double planFootSpacing() const {return isStatic() ? stat.foot_spacing : gait.foot_spacing;}
  /// 計画の重心高さ [m]
  double planZc() const {return isStatic() ? stat.z_c : gait.z_c;}
  double omega() const {return isStatic() ? stat.omega() : gait.omega();}

  SwingTiming swingTiming() const
  {
    SwingTiming t;
    if (isStatic()) {
      const rwc::StaticGaitCheck c = rwc::checkStaticGait(stat);
      t.duration = stat.t_swing;
      t.touch_phase = c.lands ? c.touch_phase : 1.0;
    } else {
      const rwc::SwingLanding l = rwc::checkSwingLanding(gait);
      t.duration = gait.t_step;
      t.touch_phase = l.lands ? l.touch_phase : 1.0;
    }
    return t;
  }
};

/// 選んだほうの計画器だけを回す。
class WalkPlanner
{
public:
  void configure(const WalkSetup & s)
  {
    mode_ = s.mode;
    dyn_ = rwc::WalkEngine(s.gait);
    stat_ = rwc::StaticWalkEngine(s.stat);
  }

  WalkMode mode() const {return mode_;}

  rwc::WalkOutputs update(double vx, double vy, double dt)
  {
    return mode_ == WalkMode::Static ? stat_.update(vx, vy, dt) : dyn_.update(vx, vy, dt);
  }

  /// 立位・原点へ戻す（技や脱力で歩行を打ち切ったとき）。
  void reset()
  {
    dyn_.reset();
    stat_.reset();
  }

private:
  WalkMode mode_ = WalkMode::Dynamic;
  rwc::WalkEngine dyn_{rwc::GaitParams{}};
  rwc::StaticWalkEngine stat_{rwc::StaticGaitParams{}};
};

/// 片足で立っている周期か（動歩行の STEP / 静歩行の SWING で、支持足が決まっている）。
inline bool singleSupport(const rwc::WalkOutputs & o)
{
  return (o.state == rwc::State::STEP || o.state == rwc::State::SWING) && o.support != 0;
}

/// 計画の立位 (0, ±W/2, -z_c) からホーム姿勢の足までのずれ [mm]。
///
/// **歩行の足はホーム姿勢の足に揃える。** 実機の足の位置は home_pose.yaml の foot だけが
/// 決め、計画上の足間隔 W は横の重心経路としてだけ効く (MotionController::configure())。
/// 歩行の足 = 計画の足 (骨盤から見た位置) + これ。
///
/// 静歩行では W を 2 × foot.y と揃える前提で、ずれた分だけ振り出し中の重心が支持足の
/// 中心から横へずれる (checkStaticStance() が起動時に言う)。
inline void walkStanceOffset(
  const WalkSetup & walk, const BodyPose & home, rk::Vec3 out[kNumSide])
{
  const double half = walk.planFootSpacing() * 500.0;
  const double zc = walk.planZc() * 1000.0;
  for (int s = 0; s < kNumSide; ++s) {
    const double lat = (s == kLeft) ? +1.0 : -1.0;
    const rk::Vec3 & hp = home.foot[s].p;
    out[s] = rk::Vec3{hp.x, hp.y - lat * half, hp.z + zc};
  }
}

/// 計画の出力 -> 両足の足裏の目標 (Σ_U [mm])。tickWalk と静歩行の到達域の門が共有する。
///
/// 世界座標 [m] -> 骨盤水平系 [m] -> Σ_U [mm] に直して walkStanceOffset() を足す。
/// 足裏の向きは**ホーム姿勢と同じ**（どちらの計画器も平行移動しか作らない）。
inline void walkFeet(
  const rwc::WalkOutputs & o, const rk::Vec3 stance_off[kNumSide], const BodyPose & home,
  FootPose out[kNumSide])
{
  const rwc::Vec3 fp[kNumSide] = {o.right_foot_in_pelvis(), o.left_foot_in_pelvis()};
  for (int s = 0; s < kNumSide; ++s) {
    out[s].p = rk::Vec3{
      fp[s][0] * 1000.0 + stance_off[s].x,
      fp[s][1] * 1000.0 + stance_off[s].y,
      fp[s][2] * 1000.0 + stance_off[s].z};
    for (int k = 0; k < 3; ++k) {
      out[s].rpy[k] = home.foot[s].rpy[k];
    }
  }
}

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__WALK_PLANNER_HPP_
