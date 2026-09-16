// IMU による歩行・立位の安定化 — 足首戦略と、胴体を起こす股の補正。
//
// **ROS を知らない。** 入力は IMU の姿勢（imu_attitude.hpp）と歩行計画の支持脚・位相、
// 出力はサーボとの境界で掛ける補正（pose_codec.hpp の PoseCorrection）だけ。
// motion_selftest から実機なしで符号まで確かめられる。
//
// 設計の原本は docs/imu_biped_walking.pdf §4.3〜4.4 と
// docs/ros2_walk_implementation.pdf §5（手順 9, 11）・§9（段 1〜3）。
//
// ===========================================================================
// 足首戦略（kp / kd）
// ===========================================================================
// 支持脚の足首に、胴体の傾きと角速度に比例した関節角を足す。
//
//     u = kp·傾き + kd·角速度                （ピッチ・ロール別々。式 (42) の符号を反転した量）
//     足裏を胴体から見て u だけ回す          （ピッチ: + でつま先が下がる / ロール: + で左縁が上がる）
//
// 足裏が床に平らに着いていれば、足裏を胴体に対して u 回すことは胴体を床に対して
// -u 回すことと同じ（剛性が高い見方）。サーボの剛性が有限なら、同じ指令がつま先
// （前へ倒れているとき）を床へ押し付けるトルクになり、ZMP が倒れる側へ出て
// 倒れを減速させる（剛性が有限な見方）。どちらの見方でも「倒れている向きと逆へ戻す」。
//
//   * kd（ジャイロの減衰）が主砲。フィルタ遅れもドリフトも無い。**先に入れる**
//   * kp（傾きの比例）はサーボの剛性に足される。傾いたまま戻らない分を減らす
//
// 「足裏を u 回す」を足首 2 軸（θ5 ピッチ・θ6 ロール）の角に直すのは、ホーム姿勢で
// 数値微分したヤコビアンの逆行列（configure()）。AXIS_FLIP や足首の組み方の符号を
// 手で追わないため。左右それぞれで持つ。
//
// ===========================================================================
// 胴体の補正（k_torso。股戦略）
// ===========================================================================
// 胴体の前傾に body_pitch_add = -k_torso·ピッチ を足す（PoseCorrection::body_pitch）。
// 胴体が前へ倒れていたら股を伸ばして起こす向き。k_torso = 1 で測った傾きの分だけ
// 逆に回す（imu_biped_walking §4.4 の「世界座標で水平を保つ」）。
//
// ★文書の式 (47) R_BL = (Rx(kφ)Ry(kθ))^T をそのまま body_pitch に読み替えると
//   body_pitch_add = +kθ になり、**足裏を床に平らに保つ変換**にはなるが、足裏が
//   着いている間は胴体の傾きを指令と一致させて位置サーボの復元力を消す向きになる
//   （k = 1 で剛性ゼロ）。ここでは文書の文章のほう（胴体を逆に回す）に合わせた。
//   motion_selftest [8] が向きを検算している。
//
// ===========================================================================
// どの脚に効かせるか・いつ弱めるか
// ===========================================================================
//   * 両脚が着いているとき（IDLE / START / STOP・HOLD）は両脚
//   * STEP では支持脚だけ。遊脚は離地（位相 0〜lift_phase）で 1 -> 0 に抜き、
//     着地の少し前から歩の終わりまでに 0 -> 1 へ戻す（次の歩で支持脚になるので）
//   * 予定の着地の gate_pre_td 前から gate_post_td 後まではゲインを gate_scale 倍にする。
//     着地の衝撃をジャイロが拾って減衰項が暴れるため（文書 §4.3）
//   * HOLD / WALK 以外（技の再生・その場保持・武装中）は fade_time かけて 0 へ抜く。
//     RELAX では即座に 0
//   * IMU が imu_timeout より古ければ 0 へ抜く（警告を出す）
//
// 出力は ankle_clamp / torso_clamp で飽和させ、rate_limit で変化の速さを抑える。
// 可動域の外に出る補正は PoseCodec が外して出す。
#ifndef ROBOONE_MOTION__STABILIZER_HPP_
#define ROBOONE_MOTION__STABILIZER_HPP_

#include "roboone_motion/body_pose.hpp"
#include "roboone_motion/event.hpp"
#include "roboone_motion/imu_attitude.hpp"
#include "roboone_motion/pose_codec.hpp"
#include "roboone_motion/servo_map.hpp"
#include "roboone_motion/side.hpp"
#include "roboone_walk_core/walk_engine.hpp"

namespace roboone_motion
{

namespace rwc = roboone_walk_core;

/// 実行中に変えられるゲイン。**既定は全部 0**（入れても今までと同じ動き）。
struct StabGains
{
  bool enable = true;
  double kp_pitch = 0.0;      //!< [rad/rad]
  double kp_roll = 0.0;       //!< [rad/rad]
  double kd_pitch = 0.0;      //!< [rad/(rad/s)]。文書の目安 0.05
  double kd_roll = 0.0;       //!< [rad/(rad/s)]
  double k_torso = 0.0;       //!< [rad/rad]。1 で測った傾きの分だけ胴体を逆に回す
  double ankle_clamp = 0.12;  //!< [rad] 足首 1 軸あたりの上限
  double torso_clamp = 0.15;  //!< [rad] 胴体の補正の上限
  double rate_limit = 4.0;    //!< [rad/s] 補正の変化の速さの上限
  double gate_pre_td = 0.03;  //!< [s] 予定の着地のこれだけ前からゲインを弱める
  double gate_post_td = 0.02; //!< [s] 予定の着地のこれだけ後まで弱める
  double gate_scale = 0.5;    //!< 弱める倍率
  double fade_time = 0.5;     //!< [s] 補正を入れる・抜くのにかける時間
  double imu_timeout = 0.1;   //!< [s] これより古い IMU は使わない
  double lift_phase = 0.10;   //!< 遊脚の補正を抜き切る位相

  bool anyGain() const
  {
    return kp_pitch != 0.0 || kp_roll != 0.0 || kd_pitch != 0.0 || kd_roll != 0.0 ||
           k_torso != 0.0;
  }
};

class Stabilizer
{
public:
  /// ホーム姿勢で足首のヤコビアンを取り、歩行計画から着地の位相を取る。
  /// 解けない脚があれば false（その脚には補正を出さない）。
  bool configure(
    const ServoMap * map, const BodyPose & home, double body_pitch,
    const rwc::GaitParams & gait);

  void setGains(const StabGains & g) {g_ = g;}
  const StabGains & gains() const {return g_;}

  struct Input
  {
    bool layer_active = false;              //!< HOLD / WALK（補正を効かせる状態）
    bool relax = false;                     //!< 脱力中（即座に 0）
    const Attitude * att = nullptr;         //!< null = IMU がまだ来ていない
    double att_age = 1e9;                   //!< [s] 最後の IMU サンプルからの経過
    const rwc::WalkOutputs * walk = nullptr;   //!< この周期の歩行計画（無ければ null）
  };
  const PoseCorrection & update(double dt, const Input & in);
  void reset();

  const PoseCorrection & correction() const {return corr_;}

  /// 記録用（/motion/stab）。
  struct Debug
  {
    bool imu_ok = false;
    bool active = false;
    bool gate = false;
    double fade = 0.0;
    double weight[kNumSide]{1.0, 1.0};
    double u_roll = 0.0;       //!< 足裏を胴体に対して回したい量 [rad]
    double u_pitch = 0.0;
    double torso_target = 0.0;
  };
  const Debug & debug() const {return dbg_;}

  double touchPhase() const {return touch_phase_;}
  bool legReady(int s) const {return ready_[s];}
  /// 足裏を胴体に対して (u_roll, u_pitch) 回すための足首の角 [θ5, θ6]。テスト用にも出す。
  void ankleFromFootRotation(int s, double u_roll, double u_pitch, double out[2]) const;

  bool popEvent(Event & e) {return ev_.pop(e);}

private:
  void weightsAndGate(const rwc::WalkOutputs * w, double weight[kNumSide], bool & gate) const;

  StabGains g_;
  bool ready_[kNumSide]{false, false};
  //! [s][r][c]: (u_roll, u_pitch) -> (θ5, θ6) の行列
  double jinv_[kNumSide][2][2]{};
  double touch_phase_ = 1.0;
  double t_step_ = 0.6;
  double fade_ = 0.0;
  PoseCorrection corr_;
  Debug dbg_;
  EventQueue ev_;
};

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__STABILIZER_HPP_
