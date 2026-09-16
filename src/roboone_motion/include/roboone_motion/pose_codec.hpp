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
//
// ===========================================================================
// 安定化の補正（PoseCorrection）
// ===========================================================================
// IMU の安定化（stabilizer.hpp）が出す補正も、この境界でだけ掛ける。上の層の
// 姿勢（cur_pose_ / hold_pose_ / 技の補間）には一切混ぜない。混ぜると補正が
// 保持姿勢や技の起点に焼き付いて、補正を切っても戻らなくなる。
//
//   body_pitch   胴体の前傾に足す。body_pitch_ と同じ経路（Σ_U -> Σ_B）
//   ankle        足裏書きの脚だけ、IK の後・足首リンク変換の前に関節角へ足す
//
// **補正を入れると解けない周期は、補正を外して解き直す。** 補正のせいで脚の
// 指令が止まる（= 前周期で固まる）のは、補正が無いより悪いので。
//
// ★decode() は補正を外さない（静的な body_pitch_ だけ外す）。実測姿勢を使うのは
//   脱力中と武装の起点だけで、そこでは補正が 0 なので食い違わない。
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

/// サーボとの境界でだけ掛ける補正（安定化の出力）。全部 0 なら何もしない。
struct PoseCorrection
{
  //! 足首の関節角に足す量 [rad]。[s][0] = θ5（ピッチ）, [s][1] = θ6（ロール）。
  //! 足裏書きの脚にだけ効く（角度書きの脚は IK を通らないので触らない）
  double ankle[kNumSide][2]{{0.0, 0.0}, {0.0, 0.0}};
  //! 胴体の前傾に足す量 [rad]。body_pitch と同じ向き（+ で前へ倒す）
  double body_pitch = 0.0;

  bool ankleZero(int s) const {return ankle[s][0] == 0.0 && ankle[s][1] == 0.0;}
  bool zero() const {return body_pitch == 0.0 && ankleZero(kRight) && ankleZero(kLeft);}
};

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
    //! 補正を入れると解けなかったので、補正を外して出した脚
    bool corr_dropped[kNumSide]{false, false};
  };
  /// corr が null なら補正なし（ティーチ・起動時の門と同じ変換）。
  Encoded encode(const BodyPose & pose, const PoseCorrection * corr = nullptr);

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
