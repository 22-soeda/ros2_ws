// 生成層 — 状態機械。「今この瞬間、機体はどの姿勢でいるべきか」だけを決める。
//
// **ROS もサーボも知らない。** 入力は実測姿勢と外からの指令、出力は目標姿勢 1 つ。
// おかげで motion_selftest から実機なしで順序を検証できる（下の 4 つは全部
// 実機でしか踏めなかったバグの修正で、テストが無いと再発しても気付けない）。
//
// ===========================================================================
// 状態
// ===========================================================================
//   RELAX   脱力。目標を作らず、実測を追いかける（復帰時の補間の起点になる）
//   ARMING  実測姿勢 -> 保持姿勢へ torque_on_time 秒かけて補間中
//   HOLD    立位保持。/cmd_walk を待つ
//   WALK    歩行中
//   MOTION  技を再生中
//   STAY    その場保持。目標を作らず cur_pose_ をそのまま出し続ける
//
// ===========================================================================
// 順序の約束 — 崩すと実機で事故る 4 つ
// ===========================================================================
// [1] **技の要求を武装判定より先に捌く。** teleop のホームポジション操作は
//     /cmd_motion "home" -> (0.1s) -> /estop false の順に来るので、先に home を
//     捌いておけば hold_pose_ がホーム姿勢になった状態で武装に入れる。逆順に
//     すると、最初の 1 本が「立ち上げ中なので出さない」で捨てられる。
//
// [2] **補間の起点は「その周期の」実測サーボ角。取れない・経路が通らないなら
//     武装しない。** 立ち上げは「実測姿勢 -> 保持姿勢」の補間なので、起点が
//     分からないまま始めると補間にならない。以前ここで hold_pose_ を起点に代用して
//     いたが、起点と終点が同じ = 補間が無いのと同じで、**トルクが入るだけで動かない**
//     (2026-08-28 実機)。代用すると今度は保持姿勢へ一気に飛ぶので、代用はしない。
//     起点はサーボ角で持ち、補間もサーボ角の空間で回す (pose_codec.hpp
//     「実測姿勢」)。足首は閉ループなので、トルクを入れる前と補間を始める前に
//     「起点 -> 保持姿勢」がサーボ角の直線で組めたまま通るかを確かめる
//     (rk::legServoPath)。2026-09-18 までは「FK が 1 度でも解けたら以後ずっと
//     取れた扱い」で、脱力中に手で足首を動かすと古い起点のまま武装していた。
//
// [3] **WALK -> HOLD は少し待ってから。** 歩き始めは指令がレート制限で立ち上がる
//     ので、歩行エンジンの状態が IDLE と START の間を数十 ms 単位で往復する
//     (2026-08-28 実機で 55ms 周期のばたつきを確認)。そのたびに状態が変わると
//     /motion/state が荒れ、hold_pose_ も上書きされ続ける。
//
// [4] **その場保持の武装後は HOLD ではなく STAY。** HOLD は tickWalk が足先を
//     立位のスタンスへ上書きするので、寝た姿勢からだと跳ねる。転倒 -> 脱力のあと、
//     寝た姿勢で武装して起き上がりに繋ぐ経路 (docs/無線操縦_不足項目レビュー.md §4.3)。
//
// ===========================================================================
// 歩行と旋回
// ===========================================================================
// 歩行計画は roboone_walk_core。動歩行 (WalkEngine) か静歩行 (StaticWalkEngine) を
// 起動時に選ぶ (walk_planner.hpp。実行中には切り替えない)。どちらも**平行移動のみ**で、
// 機体は向きを変えない。/cmd_walk の angular.z は使わない (旋回はキーフレーム
// モーション turn_l / turn_r の担当)。angular.z が乗っていたら起動後 1 回だけ警告する。
#ifndef ROBOONE_MOTION__MOTION_CONTROL_HPP_
#define ROBOONE_MOTION__MOTION_CONTROL_HPP_

#include <atomic>
#include <mutex>
#include <string>

#include "roboone_motion/body_pose.hpp"
#include "roboone_motion/event.hpp"
#include "roboone_motion/load_ff.hpp"
#include "roboone_motion/motion_library.hpp"
#include "roboone_motion/servo_map.hpp"
#include "roboone_motion/side.hpp"
#include "roboone_motion/walk_planner.hpp"
#include "roboone_walk_core/walk_engine.hpp"

namespace roboone_motion
{

namespace rwc = roboone_walk_core;

enum class State { RELAX, ARMING, HOLD, WALK, MOTION, STAY };

const char * stateName(State s);

//! その場保持の技名。teleop の hold_motion と合わせる (motions.yaml には書かない)
constexpr char kHoldMotion[] = "hold";

class MotionController
{
public:
  struct Options
  {
    double cmd_timeout = 0.5;         //!< /cmd_walk がこれだけ途切れたら指令ゼロ
    double torque_on_time = 2.0;      //!< 実測姿勢 -> 保持姿勢の補間時間
    double home_move_time = 1.5;      //!< /cmd_motion "home" でホームへ移る時間
    double hold_arm_time = 0.5;       //!< その場保持で武装するときの補間時間
    double walk_idle_hold = 0.25;     //!< IDLE がこれだけ続いたら HOLD（ばたつき止め）
    double loop_hz = 200.0;           //!< 到達域の見張りの間引きに使う
    bool walk_enable = true;
    bool motion_interrupts_walk = true;
    bool require_home_before_arm = true;
  };

  /// map / lib は寿命を通じて生きていること（ノードが持つ実体を指す）。
  /// walk.mode で動歩行か静歩行かが決まる（以後変えない）。
  void configure(
    const ServoMap * map, const MotionLibrary * lib, const WalkSetup & walk,
    const BodyPose & home, double body_pitch, const Options & opt);
  /// 動歩行で組む（従来の呼び方）。
  void configure(
    const ServoMap * map, const MotionLibrary * lib, const rwc::GaitParams & gait,
    const BodyPose & home, double body_pitch, const Options & opt);

  /// 荷重の前送り（load_ff.hpp）。実行中に変えられるので、step() を呼ぶ側が
  /// 毎周期渡す。**既定は sink = 0 で、入れても今までと同じ動き。**
  void setLoadFf(const LoadFfParams & p) {load_ff_ = p;}
  const LoadFfParams & loadFf() const {return load_ff_;}
  /// 直近の周期で各脚に配った荷重の割合 [0,1]（記録用。sink = 0 でも入る）。
  const double * loadShareLast() const {return load_share_;}
  /// 直近の周期で実際に出している伸ばし量 [mm]（レート制限のあと）。
  const double * loadFfOffset() const {return load_ff_state_.offset();}

  /// 足踏み（walk_planner.hpp の WalkPlanner::setMarch）。実行中に変えられるので、
  /// step() を呼ぶ側が毎周期渡す。**既定 false。true にした周期からその場で歩き出す。**
  void setMarch(bool on) {walk_.setMarch(on);}
  bool march() const {return walk_.march();}

  // --- 外からの指令（購読スレッドから呼ばれる。ロックを持つ）-------------
  void setEstop(bool v) {estop_.store(v);}
  void setWalkCmd(double vx, double vy, double wz, double stamp);
  void requestMotion(const std::string & name);

  // --- 1 周期 -----------------------------------------------------------
  struct Tick
  {
    State state = State::RELAX;
    bool state_changed = false;
    //! この周期の目標姿勢。**null なら指令を出さない**（RELAX）
    const BodyPose * target = nullptr;
    bool want_torque = false;
  };

  /// measured が null なら「実測姿勢が取れなかった」。why はその理由。
  Tick step(
    double now, double dt, const BodyPose * measured, const std::string & why,
    bool torque_ready);

  // --- 素性 -------------------------------------------------------------
  State state() const {return state_;}
  WalkMode walkMode() const {return walk_.mode();}

  /// /motion/state の書式。**behavior が読み方をテストで固定しているので変えない。**
  ///
  ///     <状態>              RELAX / ARMING / HOLD / WALK / STAY
  ///                         (STAY = その場保持。behavior の ready_states に無いので
  ///                          「歩けない」扱いになる。寝ている間はそれで正しい)
  ///     MOTION:<技名>       再生中だけ。技名はコロンの後ろ ("MOTION:punch_r")
  ///     ... walk=static      静歩行で起動したときだけ末尾に付く ("HOLD walk=static")。
  ///                         behavior は先頭の語だけを状態として読み、key=value は
  ///                         motion= / playing= 以外を無視する
  ///
  /// 先頭語が状態で、コロン区切りの後置は MOTION のときの技名だけ。将来ここに
  /// 支持脚や位相を足すなら **空白区切りで後ろに足す** こと (behavior 側は
  /// 空白以降を無視するように作られている)。先頭語の意味を変える・コロンの
  /// 使い方を増やす変更は、behavior (roboone_behavior) と同時に直す。
  ///
  /// 転倒 (FALL 相当) はまだ無い。姿勢の推定は motion_node の中にある
  /// (imu_attitude.hpp。/camera/imu から胴体のロール・ピッチを出している) ので、
  /// 入れるならその出力を step() へ渡す形になる。
  std::string stateText() const;

  const BodyPose & currentPose() const {return cur_pose_;}
  /// この周期に回した歩行計画の出力。回していない周期 (HOLD / WALK 以外) は null。
  /// IMU の安定化が支持脚と位相を知るために読む (stabilizer.hpp)。
  const rwc::WalkOutputs * walkOutputs() const {return walk_ticked_ ? &walk_out_ : nullptr;}
  const BodyPose & holdPose() const {return hold_pose_;}

  bool popEvent(Event & e) {return ev_.pop(e);}

private:
  //! 武装の経路が通らなかったとき、次に確かめ直すまでの間 [s]（検査が数 ms かかる）
  static constexpr double kArmRecheck = 0.25;

  bool canArm() const {return !opt_.require_home_before_arm || seen_motion_;}
  /// from（実測。サーボ角の本体を持つこと）から hold_pose_ へ、サーボ角の直線で
  /// 脚が組めたまま動けるか。ダメなら why に理由を足して false（ヘッダ [2]）。
  bool armPathClear(const BodyPose & from, std::string & why) const;
  void setState(State s);
  /// 今の姿勢 (cur_pose_) から to へ time 秒で移る 1 区間の補間を仕込む。
  void startBlend(const BodyPose & to, double time, double now, const char * what);
  void handleMotionRequest(const std::string & name, double now);
  /// 歩行計画を 1 周期進めて、足先目標を cur_pose_ に書く。
  void tickWalk(double now, double dt);
  /// 歩行を畳む（計画と荷重の前送りを一緒に戻す）。
  void resetWalk();
  void reportPlayerWarning();

  const ServoMap * map_ = nullptr;
  const MotionLibrary * lib_ = nullptr;
  Options opt_;
  double body_pitch_ = 0.0;

  WalkPlanner walk_;
  //! 歩行計画の立位の足 (0, ±W/2, -z_c) からホーム姿勢の足までのずれ [mm]。
  //! 歩行の足 = 計画の足 + これ、なので計画の立位はホーム姿勢の足そのものになる
  //! (configure() の注記)
  rk::Vec3 stance_off_[kNumSide]{};
  rwc::WalkOutputs walk_out_;
  bool walk_ticked_ = false;       //!< この周期に tickWalk を回したか
  LoadFfParams load_ff_;
  LoadFf load_ff_state_;
  double load_share_[kNumSide]{0.0, 0.0};
  MotionPlayer player_;
  Motion blend_motion_;
  BodyPose home_pose_, hold_pose_, cur_pose_;

  std::atomic<bool> estop_{false};
  std::mutex walk_mtx_, req_mtx_;
  double walk_cmd_[2]{0.0, 0.0};
  double walk_stamp_ = 0.0;
  std::string motion_req_;
  bool got_motion_ = false, seen_motion_ = false, warned_yaw_ = false;

  //! サーボ層へ伝える「トルクを入れてよいか」。step() だけが書く
  bool want_torque_ = false;
  bool arm_in_place_ = false;      //!< hold を受けた: 次の武装は実測姿勢のまま
  bool stay_after_arm_ = false;    //!< その武装が終わったら HOLD ではなく STAY へ
  double arm_check_at_ = -1.0;     //!< 武装の経路をこの時刻まで確かめ直さない
  double idle_since_ = -1.0;
  int reach_tick_ = 0;
  State state_ = State::RELAX;
  EventQueue ev_;
};

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__MOTION_CONTROL_HPP_
