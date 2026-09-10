// 変換層 — 機体の 1 姿勢 (BodyPose) と、バスへ流す生カウントの間。
//
// **Σ_U と Σ_B の境界はここだけ。** 上の層 (歩行計画・技の再生・補間・ホーム姿勢) は
// 胴体の前傾を知らないまま Σ_U で動き、この層が指令のときに掛け、観測のときに外す。
// 対になっているので、片方だけ直すと武装の瞬間に前傾ぶん跳ねる。
//
//     encode()  BodyPose [Σ_U] ─bodyPitchApply→ [Σ_B] ─IK→ サーボ角 ─→ 生カウント
//     decode()  生カウント ─→ サーボ角 ─FK→ [Σ_B] ─bodyPitchRemove→ BodyPose [Σ_U]
//
// ===========================================================================
// 脚の 2 つの書き方
// ===========================================================================
// LegMode::Foot   足裏の (p, R)。**IK を通る。**解けない目標は送らない
// LegMode::Servo  サーボ角。**IK も FK も通らない。**届かない姿勢を書けるのが主旨
//                 なので運動学の都合で止めない。歯止めは servo_limits.yaml の窓だけ
//
// ===========================================================================
// 解けない目標は「送らない」
// ===========================================================================
// encode() は解けなかった脚の send[] を false にして返す。呼び側がその脚の指令を
// 出さなければ、前周期の指令が生きたままになって機体は直前の姿勢で止まる。
// ゼロや中途半端な解を送るより安全。
#ifndef ROBOONE_MOTION__POSE_CODEC_HPP_
#define ROBOONE_MOTION__POSE_CODEC_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "feetech_servo/feetech_bus.hpp"
#include "roboone_motion/body_pose.hpp"
#include "roboone_motion/event.hpp"
#include "roboone_motion/servo_map.hpp"
#include "roboone_motion/side.hpp"

namespace roboone_motion
{

class PoseCodec
{
public:
  /// map は寿命を通じて生きていること（ノードが持つ ServoMap を指す）。
  void configure(const ServoMap * map, double body_pitch)
  {
    map_ = map;
    body_pitch_ = body_pitch;
  }

  double bodyPitch() const {return body_pitch_;}

  // --- 指令側 -----------------------------------------------------------
  struct Encoded
  {
    //! その脚の指令を出してよいか。false = 解けなかったので前周期の指令を保つ
    bool send[kNumSide]{false, false};
    std::vector<int16_t> counts[kNumSide];              //!< バス 1 本ぶんの生カウント
    //! 指令の関節角 [rad]（/motion/joint_commands 用）。send[s] のときだけ有効
    double theta[kNumSide][rk::kNumJoints]{};
    std::vector<double> arm_deg;                        //!< 指令の腕角 [deg]
  };
  Encoded encode(const BodyPose & pose);

  // --- 観測側 -----------------------------------------------------------
  struct Decoded
  {
    //! 姿勢としてそのまま使えるか（両脚が読めて、かつ変換も成功した）
    bool ok = false;
    //! 生カウントが全軸読めた側。theta / arm はこの側だけ更新されている
    bool side_read[kNumSide]{false, false};
    BodyPose pose;
    //! 実測の関節角 [rad]（/joint_states 用）。**Σ_B のまま**置く（実機の値なので）
    double theta[kNumSide][rk::kNumJoints]{};
    //! サーボ角 -> 関節角 の変換結果。side_read[s] のときだけ意味がある
    rk::LegServoStatus status[kNumSide]{rk::LegServoStatus::Ok, rk::LegServoStatus::Ok};
    std::string why;                                    //!< ok = false のときの理由
  };
  Decoded decode(const std::vector<feetech_servo::ServoState> st[kNumSide]);

  bool popEvent(Event & e) {return ev_.pop(e);}

private:
  const ServoMap * map_ = nullptr;
  //! 胴体の前傾 [rad]。+ で前へ倒れる。股ピッチ θ1 に -body_pitch_ を足すのと等価で、
  //! 膝・足首の関節角は変わらない（body_pose.hpp の「胴体の前傾」節）。
  double body_pitch_ = 0.0;
  //! 足首の順変換 (1 変数ニュートン法) の種。指令側と観測側で別々に持つ
  double th6_cmd_seed_[kNumSide]{0.0, 0.0};
  double th6_meas_seed_[kNumSide]{0.0, 0.0};
  EventQueue ev_;
};

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__POSE_CODEC_HPP_
