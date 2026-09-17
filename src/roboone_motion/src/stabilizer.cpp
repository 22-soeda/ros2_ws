#include "roboone_motion/stabilizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace roboone_motion
{

namespace
{

double clamp(double v, double lo, double hi) {return v < lo ? lo : (v > hi ? hi : v);}

double smoothstep(double x)
{
  x = clamp(x, 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

/// prev から target へ、1 周期に max_step までしか動かさない。
double rateLimit(double prev, double target, double max_step)
{
  return prev + clamp(target - prev, -max_step, max_step);
}

}  // namespace

bool Stabilizer::configure(
  const ServoMap * map, const BodyPose & home, double body_pitch,
  const rwc::GaitParams & gait)
{
  WalkSetup walk;
  walk.gait = gait;
  return configure(map, home, body_pitch, walk.swingTiming());
}

bool Stabilizer::configure(
  const ServoMap * map, const BodyPose & home, double body_pitch, const SwingTiming & swing)
{
  touch_phase_ = swing.touch_phase;
  swing_dur_ = swing.duration;
  ds_time_ = swing.ds_time;

  bool all = true;
  for (int s = 0; s < kNumSide; ++s) {
    ready_[s] = false;
    const rk::LegServoParams & prm = map->leg_params(s);
    FootPose f = home.foot[s];
    bodyPitchApply(f, body_pitch);
    double th[rk::kNumJoints]{};
    if (rk::ik(prm.leg, f.p, matFromRpy(f.rpy), th, /*clamp=*/false) != rk::IkStatus::Ok) {
      ev_.error(
        std::string("安定化: ") + kSideTag[s] +
        "脚のホーム姿勢が IK で解けないので、この脚には足首の補正を出さない");
      all = false;
      continue;
    }
    rk::Vec3 p0;
    rk::Mat3 r0;
    rk::fk(prm.leg, th, p0, r0);

    // 足首 1 軸を h だけ動かしたときの、胴体から見た足裏の回転（左から掛かる小回転）
    const double h = 1e-5;
    const std::size_t idx[2] = {rk::ANKLE_PITCH, rk::ANKLE_ROLL};
    double j[2][2]{};   // j[r][c]: r = (roll, pitch), c = (θ5, θ6)
    for (int c = 0; c < 2; ++c) {
      double tp[rk::kNumJoints];
      std::copy(th, th + rk::kNumJoints, tp);
      tp[idx[c]] += h;
      rk::Vec3 p1;
      rk::Mat3 r1;
      rk::fk(prm.leg, tp, p1, r1);
      // D = R1 · R0^T
      double d[3][3]{};
      for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
          for (int k = 0; k < 3; ++k) {
            d[a][b] += r1(a, k) * r0(b, k);
          }
        }
      }
      j[0][c] = 0.5 * (d[2][1] - d[1][2]) / h;   // x 軸まわり = ロール
      j[1][c] = 0.5 * (d[0][2] - d[2][0]) / h;   // y 軸まわり = ピッチ
    }
    const double det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
    char buf[256];
    std::snprintf(
      buf, sizeof(buf),
      "安定化: %s脚の足首 d(roll,pitch)/d(θ5,θ6) = [[%+.3f, %+.3f], [%+.3f, %+.3f]]",
      kSideTag[s], j[0][0], j[0][1], j[1][0], j[1][1]);
    if (std::abs(det) < 0.5) {
      ev_.error(std::string(buf) + " — 行列式が小さすぎる。この脚には足首の補正を出さない");
      all = false;
      continue;
    }
    ev_.info(buf);
    jinv_[s][0][0] = j[1][1] / det;
    jinv_[s][0][1] = -j[0][1] / det;
    jinv_[s][1][0] = -j[1][0] / det;
    jinv_[s][1][1] = j[0][0] / det;
    ready_[s] = true;
  }
  reset();
  return all;
}

void Stabilizer::ankleFromFootRotation(int s, double u_roll, double u_pitch, double out[2]) const
{
  if (!ready_[s]) {
    out[0] = out[1] = 0.0;
    return;
  }
  out[0] = jinv_[s][0][0] * u_roll + jinv_[s][0][1] * u_pitch;
  out[1] = jinv_[s][1][0] * u_roll + jinv_[s][1][1] * u_pitch;
}

void Stabilizer::reset()
{
  fade_ = 0.0;
  corr_ = PoseCorrection{};
  dbg_ = Debug{};
}

void Stabilizer::weightsAndGate(
  const rwc::WalkOutputs * w, double weight[kNumSide], bool & gate) const
{
  weight[kRight] = weight[kLeft] = 1.0;
  gate = false;
  // 動歩行の歩の頭の両足支持 (ds_time > 0): 両脚とも接地しているので重みは両脚 1。
  // 直前の着地からの経過が gate_post_td 以内ならゲインを弱める
  if (w && w->state == rwc::State::STEP && w->double_support) {
    const double T = std::max(1e-3, swing_dur_);
    gate = (1.0 - touch_phase_) * T + w->ds_elapsed <= g_.gate_post_td;
    return;
  }
  // 片足支持 = 動歩行の STEP / 静歩行の SWING。位相はどちらも遊脚の経過 [0,1]。
  // 静歩行の両足支持 (SHIFT / STOP) は立位と同じく両脚に効かせる。
  if (!w || !singleSupport(*w)) {return;}

  const int sup = (w->support == rwc::LEFT) ? kLeft : kRight;
  const int sw = (sup == kLeft) ? kRight : kLeft;
  const double ph = w->phase;
  const double T = std::max(1e-3, swing_dur_);

  // 遊脚: 離地で抜き、着地の少し前から歩の終わりまでに戻す
  const double lift = std::max(1e-3, g_.lift_phase);
  const double out = 1.0 - smoothstep(ph / lift);
  const double in_start = clamp(touch_phase_ - std::max(0.02, g_.gate_pre_td / T), 0.0, 0.98);
  const double in = smoothstep((ph - in_start) / (1.0 - in_start));
  weight[sup] = 1.0;
  weight[sw] = std::max(out, in);

  // 着地の前後はゲインを弱める。動歩行は歩が切れ目なく続くので、前の歩の着地の尾が
  // 次の歩の頭にかかる分も見る (静歩行は着地と次の振り出しの間に重心移動が挟まる)
  const double t = ph * T;
  const double td = touch_phase_ * T;
  // 両足支持を挟む (ds_time > 0) なら、着地の尾は上の両足支持の側で見る
  gate = (t >= td - g_.gate_pre_td && t <= td + g_.gate_post_td) ||
    (w->state == rwc::State::STEP && t <= td + g_.gate_post_td - T - ds_time_);
}

const PoseCorrection & Stabilizer::update(double dt, const Input & in)
{
  const bool imu_ok = in.att && in.att->valid && in.att_age <= g_.imu_timeout;
  if (in.relax) {
    reset();
    dbg_.imu_ok = imu_ok;     // 脱力中でも IMU が来ているかは見えるように
    return corr_;
  }

  const bool want = g_.enable && g_.anyGain() && in.layer_active;
  if (want && !imu_ok) {
    char buf[256];
    std::snprintf(
      buf, sizeof(buf),
      "IMU が来ていない (最後のサンプルから %.2fs)。足首・胴体の補正を抜いている",
      in.att ? in.att_age : -1.0);
    ev_.warn(buf, 2000, "stab_no_imu");
  }
  const bool active = want && imu_ok;
  const double fstep = (g_.fade_time > 1e-6) ? dt / g_.fade_time : 1.0;
  fade_ = clamp(fade_ + (active ? fstep : -fstep), 0.0, 1.0);

  double weight[kNumSide];
  bool gate = false;
  weightsAndGate(in.walk, weight, gate);

  double u_roll = 0.0, u_pitch = 0.0, torso = 0.0;
  if (imu_ok) {
    const double k = gate ? g_.gate_scale : 1.0;
    const Attitude & a = *in.att;
    u_pitch = k * (g_.kp_pitch * a.pitch + g_.kd_pitch * a.gyro[1]);
    u_roll = k * (g_.kp_roll * a.roll + g_.kd_roll * a.gyro[0]);
    torso = clamp(-g_.k_torso * a.pitch, -g_.torso_clamp, g_.torso_clamp);
  }

  const double max_step = std::max(0.0, g_.rate_limit) * dt;
  // 使っていないほうの方式は 0 へ戻す（実行中に stab.board を切り替えても跳ばない）。
  for (int s = 0; s < kNumSide; ++s) {
    double tgt[2]{0.0, 0.0};
    if (!g_.board) {
      ankleFromFootRotation(s, u_roll, u_pitch, tgt);
    }
    for (int c = 0; c < 2; ++c) {
      const double v = g_.board ? 0.0 :
        clamp(fade_ * weight[s] * tgt[c], -g_.ankle_clamp, g_.ankle_clamp);
      corr_.ankle[s][c] = rateLimit(corr_.ankle[s][c], v, max_step);
    }
  }
  // 板は脚ごとの重みを持たない（両足裏を 1 枚として回すので分けられない）。
  const double board_tgt[2] = {
    g_.board ? clamp(fade_ * u_roll, -g_.board_clamp, g_.board_clamp) : 0.0,
    g_.board ? clamp(fade_ * u_pitch, -g_.board_clamp, g_.board_clamp) : 0.0};
  for (int c = 0; c < 2; ++c) {
    corr_.board[c] = rateLimit(corr_.board[c], board_tgt[c], max_step);
  }
  corr_.body_pitch = rateLimit(corr_.body_pitch, fade_ * torso, max_step);

  dbg_.imu_ok = imu_ok;
  dbg_.active = active;
  dbg_.gate = gate;
  dbg_.fade = fade_;
  dbg_.weight[kRight] = weight[kRight];
  dbg_.weight[kLeft] = weight[kLeft];
  dbg_.u_roll = u_roll;
  dbg_.u_pitch = u_pitch;
  dbg_.torso_target = torso;
  return corr_;
}

}  // namespace roboone_motion
