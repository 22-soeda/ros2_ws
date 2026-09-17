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

PoseCodec::Encoded PoseCodec::encode(const BodyPose & pose, const PoseCorrection * corr)
{
  Encoded out;
  out.arm_deg.assign(map_->num_arm(), 0.0);
  if (corr && corr->zero()) {corr = nullptr;}
  const double psi = body_pitch_ + (corr ? corr->body_pitch : 0.0);

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
      bodyPitchApplyServo(map_->leg_params(s), servo, psi);
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
      bodyPitchApply(foot, psi);
      const double * off = (corr && !corr->ankleZero(s)) ? corr->ankle[s] : nullptr;
      LegSolve r = servoFromFootPose(map_->leg_params(s), foot, servo, theta, off);
      if (!r.ok() && corr) {
        // 補正のせいで解けないなら、補正を外して解き直す（ヘッダ「安定化の補正」）
        foot = pose.foot[s];
        bodyPitchApply(foot, body_pitch_);
        r = servoFromFootPose(map_->leg_params(s), foot, servo, theta);
        if (r.ok()) {
          out.corr_dropped[s] = true;
          ev_.warn(
            tag(s) + "脚: 安定化の補正を入れると解けないので、補正を外して出した",
            1000, "corr_dropped" + tag(s));
        }
      }
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
    std::string wrapped;
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
      if (!st[s][j].valid) {
        all = false;
        out.why += tag(s) + "脚: ID" + std::to_string(kLegServoId[j]) + " が応答しない ";
        break;
      }
      servo[j] = map_->leg_servo_from_count(s, j, st[s][j].pos);
      if (st[s][j].pos < 0 || st[s][j].pos > 4095) {
        wrapped += " ID" + std::to_string(kLegServoId[j]) + "=" + std::to_string(st[s][j].pos);
      }
    }
    if (!all) {
      out.ok = false;
      continue;
    }

    // --- 武装の起点: サーボ角そのもの（ヘッダ「実測姿勢」）-----------------
    // 指令側 (encode) の bodyPitchApplyServo と対。Σ_U へ戻しておけば、そのまま
    // encode に通すと実測と同じカウントに戻る（= 補間の初周期で動かない）。
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {out.pose.leg_servo[s][j] = servo[j];}
    bodyPitchRemoveServo(map_->leg_params(s), out.pose.leg_servo[s], body_pitch_);
    out.pose.leg_servo_valid[s] = true;
    out.pose.leg_mode[s] = LegMode::Servo;
    if (!wrapped.empty()) {
      // 多回転の軸（servo_limits.yaml が [0, 0]）で巻き数が乗った。サーボ角のまま
      // 補間すると、その軸を 1 回転させる指令になる
      out.ok = false;
      out.why += tag(s) + "脚: カウントが 0-4095 の外 (多回転の巻き数ずれ)" + wrapped + " ";
    } else {
      const rk::LegServoPathResult arm = rk::legServoArmable(map_->leg_params(s), servo);
      if (!arm.ok()) {
        out.ok = false;
        out.why += tag(s) + "脚: " + legPathWhy(arm) + " ";
      }
    }

    // --- 足裏の影: 表示と /joint_states 用 -----------------------------------
    // 足首が CRANK_LIMIT_DEG の箱の外にいると AnkleClamped（箱の縁で解いた値）に
    // なるが、もう武装を止める理由にはしない（起点はサーボ角のほうなので）。
    // theta は 0 で初期化してある。legJointsFromServo は膝で失敗すると theta[KNEE]
    // を書かずに早期リターンするので、未初期化のまま fk() に入れると出鱈目な姿勢が出る。
    const rk::LegServoStatus lst = footPoseFromServo(
      map_->leg_params(s), servo, out.pose.foot[s], theta, th6_meas_seed_[s]);
    // 実測は Σ_B で出てくるので Σ_U へ戻す（encode の bodyPitchApply と対）。
    // ★theta は Σ_B の関節角のまま置く。/joint_states は実機の値を出す場所なので。
    bodyPitchRemove(out.pose.foot[s], body_pitch_);
    out.status[s] = lst;
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
