#include "roboone_motion/pose_codec.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace roboone_motion
{

using feetech_servo::ServoState;

namespace
{
std::string tag(int side) {return std::string(kSideTag[side]);}
}  // namespace

PoseCodec::Encoded PoseCodec::encode(const BodyPose & pose)
{
  Encoded out;
  out.arm_deg.assign(map_->num_arm(), 0.0);

  for (int s = 0; s < kNumSide; ++s) {
    std::vector<int16_t> pos(map_->bus(s).ids.size(), 0);

    double servo[rk::kNumJoints];
    double theta[rk::kNumJoints]{};

    if (pose.leg_mode[s] == LegMode::Servo) {
      // 角度書きの脚 (motions.yaml の R_leg / L_leg)。**IK も FK も通らない。**
      // 届かない姿勢を書けるのがこの書き方の主旨なので、ここで運動学の都合で
      // 止めない。歯止めは servo_limits.yaml の窓だけ (下の clamped)。
      if (!pose.leg_servo_valid[s]) {
        // 引き継ぐ土台が IK で解けなかった枚 (MotionPlayer::start の警告)。
        // 埋まっていない軸があるので、足裏書きが解けないときと同じく送らない。
        ev_.warn(
          tag(s) + "脚: 角度書きのサーボ角が揃っていない。この周期の指令は送らない",
          1000, "servo_incomplete" + tag(s));
        continue;
      }
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
        servo[j] = pose.leg_servo[s][j];
      }
      bodyPitchApplyServo(map_->leg_params(s), servo, body_pitch_);
      // 関節角は /motion/joint_commands のために出すだけ。出せなくても指令は送る。
      const rk::LegServoStatus jst =
        rk::legJointsFromServo(map_->leg_params(s), servo, theta, th6_cmd_seed_[s]);
      if (jst == rk::LegServoStatus::Ok || jst == rk::LegServoStatus::AnkleClamped) {
        th6_cmd_seed_[s] = theta[rk::ANKLE_ROLL];
      } else {
        // 機構としてその角にはならない (膝の三角形が閉じない等)。指令は出るが、
        // リンクが突っ張るのでサーボは追従しきれない。config を疑うところ。
        ev_.warn(
          tag(s) + "脚: 角度書きの姿勢を関節角に戻せない (status=" +
          std::to_string(static_cast<int>(jst)) +
          ")。機構が成り立たない角かもしれない (指令はそのまま送る)",
          2000, "servo_no_joints" + tag(s));
      }
    } else {
      FootPose foot = pose.foot[s];
      bodyPitchApply(foot, body_pitch_);
      const LegSolve r = servoFromFootPose(map_->leg_params(s), foot, servo, theta);
      if (!r.ok()) {
        // 解けない目標は**送らない**。前周期の指令が生きたままになるので、
        // 機体は直前の姿勢で止まる。ゼロや中途半端な解を送るより安全。
        ev_.warn(
          tag(s) + "脚の IK が解けない (ik=" + std::to_string(static_cast<int>(r.ik_status)) +
          " servo=" + std::to_string(static_cast<int>(r.servo_status)) +
          ")。この周期の指令は送らない", 1000, "ik_unsolved" + tag(s));
        continue;
      }
      if (r.ankle_outside_envelope) {
        // 丸めていないので指令はこのまま出る。実測姿勢を読み戻せない帯に入った合図。
        ev_.warn(
          tag(s) + "脚: 足首がエンベロープの外 (指令はそのまま出す。実測姿勢が"
          "読み戻せない可能性)", 2000, "ankle_envelope" + tag(s));
      }
    }

    // どの軸が丸められたかを名前で出す。「丸められた」だけだと、原点が窓の外に
    // 1 カウントはみ出しているような無害なものと、可動域を超えた指令の区別が付かない。
    std::string clamped;
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
      bool c = false;
      pos[j] = static_cast<int16_t>(map_->leg_count_from_servo(s, j, servo[j], &c));
      if (c) {
        clamped += (clamped.empty() ? "" : ",");
        clamped += std::string("ID") + std::to_string(kLegServoId[j]) +
          "(" + kLegJointName[j] + ")";
      }
    }
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
      out.theta[s][j] = theta[j];
    }

    // 腕。運動学は無いので T ポーズ基準の deg をそのままカウントに直す。
    const auto & arms = map_->arms();
    std::size_t k = rk::kNumJoints;
    for (std::size_t a = 0; a < arms.size(); ++a) {
      if (arms[a].side != s) {continue;}
      const double deg = (a < pose.arm.size()) ? pose.arm[a] : 0.0;
      out.arm_deg[a] = deg;
      bool c = false;
      pos[k++] = static_cast<int16_t>(map_->arm_count_from_deg(a, deg, &c));
      if (c) {
        clamped += (clamped.empty() ? "" : ",");
        clamped += arms[a].name;
      }
    }
    if (!clamped.empty()) {
      ev_.warn(
        tag(s) + ": 指令が servo_limits.yaml の窓で丸められた: " + clamped,
        2000, "limit_clamp" + tag(s));
    }

    out.counts[s] = std::move(pos);
    out.send[s] = true;
  }
  return out;
}

PoseCodec::Decoded PoseCodec::decode(const std::vector<ServoState> st[kNumSide])
{
  Decoded out;
  out.pose.arm.assign(map_->num_arm(), 0.0);
  out.ok = true;

  for (int s = 0; s < kNumSide; ++s) {
    if (st[s].size() < map_->bus(s).ids.size()) {
      out.ok = false;
      out.why += tag(s) + "脚: まだ読めていない ";
      continue;
    }
    double servo[rk::kNumJoints];
    double theta[rk::kNumJoints]{};
    bool all = true;
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
      if (!st[s][j].valid) {
        all = false;
        out.why += tag(s) + "脚: ID" + std::to_string(kLegServoId[j]) + " が応答しない ";
        break;
      }
      servo[j] = map_->leg_servo_from_count(s, j, st[s][j].pos);
    }
    if (!all) {
      out.ok = false;
      continue;
    }
    // theta は 0 で初期化してある。legJointsFromServo は膝で失敗すると theta[KNEE]
    // を書かずに早期リターンするので、未初期化のまま fk() に入れると出鱈目な姿勢が
    // 出る (それを「実測」として補間の起点にすると事故る)。
    const rk::LegServoStatus lst = footPoseFromServo(
      map_->leg_params(s), servo, out.pose.foot[s], theta, th6_meas_seed_[s]);
    // 指令側 (encode) と対になる境界。実測は Σ_B で出てくるので Σ_U へ戻す。
    // これを忘れると、武装時の「実測姿勢 -> 保持姿勢」の補間の起点だけが別の系に
    // なり、トルクを入れた瞬間に前傾ぶんだけ跳ねる。
    // ★theta は Σ_B の関節角のまま置く。/joint_states は実機の値を出す場所なので。
    bodyPitchRemove(out.pose.foot[s], body_pitch_);
    out.status[s] = lst;
    if (lst != rk::LegServoStatus::Ok) {
      out.ok = false;
      out.why += tag(s) + "脚: 変換できない(status=" +
        std::to_string(static_cast<int>(lst)) + ", 膝サーボ " +
        std::to_string(
        static_cast<int>(
          map_->leg_tpose_deg_from_count(s, rk::KNEE, st[s][rk::KNEE].pos))) + "deg) ";
    }
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
      out.theta[s][j] = theta[j];
    }
    out.side_read[s] = true;

    const auto & arms = map_->arms();
    std::size_t k = rk::kNumJoints;
    for (std::size_t a = 0; a < arms.size(); ++a) {
      if (arms[a].side != s) {continue;}
      if (k < st[s].size() && st[s][k].valid) {
        out.pose.arm[a] = map_->arm_deg_from_count(a, st[s][k].pos);
      }
      ++k;
    }
  }
  return out;
}

}  // namespace roboone_motion
