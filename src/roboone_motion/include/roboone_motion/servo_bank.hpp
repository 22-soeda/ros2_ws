// サーボ層 — 2 本のバスを「1 つの機体」として束ねる。
//
// この層より下は feetech_servo (1 ポート 1 バスのドライバ)、上は生カウントの世界。
// **運動学も ROS も知らない。** 必要なのはポート名と ID の並びだけで、ServoMap を
// 引数に取らないのはそのため（後から feetech_servo へ移せるようにしておく）。
//
// ===========================================================================
// 上との境界
// ===========================================================================
// 制御ループとバススレッドは、生カウントの配列 2 本だけでやり取りする。
//
//     control ──[ setTargets(side, counts) ]──→ bus     指令
//     control ←──[ states(side, out)       ]── bus      実測
//
// **制御ループはシリアルを触らない。** 読み出しは 1 往復で数 ms かかり、応答が
// 欠けると最大 timeout_ms (20ms) 待つので、同じスレッドに置くと 200Hz の周期が
// 読み出しの都合で崩れる。書き込みは送りっぱなし (TX のみ) なので速い。
//
// ===========================================================================
// トルクの入れ方 — ここが事故の起きる場所
// ===========================================================================
// Feetech は**目標角レジスタが生きたままトルクが入る**。前回の目標角が残っている
// ところへトルクを入れると、そこへ全速で飛ぶ。だからトルクオンは必ず 3 段:
//
//   1) 実測位置を読む
//   2) その実測位置を目標として書く    ← これで「今いる場所」が目標になる
//   3) トルクを入れる                  ← 動かない
//
// 4 段目 (実測姿勢から保持姿勢への補間) は上の層 (MotionController) の仕事。
//
// **実測が揃わないうちは入れない。** 読めなかった軸だけが古い目標角へ飛ぶため。
// ここで止まるときはまず電源を疑う (低電圧で応答が間欠的に欠ける実機の癖)。
//
// ===========================================================================
// allow_torque と dry_run
// ===========================================================================
//   allow_torque=false  バスは開いて読むが、enable_torque(true) と位置指令を
//                       送らない。「入ったことにして」上の層は最後まで回る
//   dry_run=true        バスも開かない。torqueReady() は即 true
#ifndef ROBOONE_MOTION__SERVO_BANK_HPP_
#define ROBOONE_MOTION__SERVO_BANK_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "feetech_servo/feetech_bus.hpp"
#include "roboone_motion/event.hpp"
#include "roboone_motion/side.hpp"

namespace roboone_motion
{

/// 1 本のバスに載っている軸（ServoMap::BusAxes から必要なぶんだけ写したもの）。
struct BankPort
{
  std::string dev;                //!< udev の固定名。空なら「そのバスは無い」
  std::vector<uint8_t> ids;       //!< 読み書きの並び。1 パケットの中身
};

struct BankOptions
{
  int baud = 1000000;
  //! HLS 系は位置指令パケットの 44/45 が GOAL_TORQUE。**0 だと全軸まったく動かない。**
  int goal_torque = 1000;
  //! 位置指令の速度 (reg46/47)。★0 にしないこと。0 は「無制限」ではなく「動かない」
  int move_speed = 2000;
  //! 位置指令の加速度 (reg41)。単位 100 step/s^2、0-254。★0 は使わない（motion_node.yaml）
  int move_acc = 254;
  double loop_hz = 200.0;         //!< 書き込み周期
  double read_hz = 50.0;          //!< 読み出し周期（1 往復かかるので書き込みより遅く）
  bool dry_run = false;
  bool allow_torque = true;
};

/// 電流の区間集計。読み出し (read_hz) で積み、上が吸い出すたびに 0 に戻る。
///
/// publish のたびの瞬時値だけを出すと、読み 50Hz / 出し 10Hz では 5 回に 4 回捨てる
/// ことになり、**踏ん張った瞬間や接触時の突入電流をそのまま取りこぼす。**
struct BankCurrent
{
  std::vector<int> peak;          //!< 区間ピーク [mA]（絶対値）
  std::vector<long> sum;          //!< 区間合計 [mA]
  std::vector<int> n;             //!< 有効サンプル数。ピークの信用度がこれで分かる
};

class ServoBank
{
public:
  ~ServoBank() {stop();}

  /// バスを開く。**スレッドはまだ動かない**（start() で動く）。
  ///
  /// どちらも開けなければ false + err。片方だけ開けたときは true を返すので、
  /// 歩行を止めるかどうかは呼び側が numOpened() を見て決める（単脚では歩けない）。
  bool open(const BankPort port[kNumSide], const BankOptions & opt, std::string & err);

  void start();                   //!< バススレッドを 1 バスにつき 1 本回す
  /// スレッドを止め、**脱力を置いてから**閉じる。
  ///
  /// トルクを入れたまま消えると、機体は最後の目標角で固まったまま誰も止められない。
  void stop();

  // --- トルク ---------------------------------------------------------
  void setWantTorque(bool v) {want_torque_.store(v);}
  /// 全軸のトルクをその場で切る。**開いた直後に「まず脱力」したいとき用**
  /// （motion_teach は手で構えるので、読み始める前にこれを 1 回だけ呼ぶ）。
  /// start() の前に呼ぶこと（バススレッドと同時にバスを触らない）。
  void relax();
  /// 開いている全バスで 3 段のトルクオンが終わったか。dry_run では即 true。
  bool torqueReady() const;
  bool torqueOn(int side) const {return torque_on_[side].load();}

  // --- 生カウントの授受 -------------------------------------------------
  void setTargets(int side, std::vector<int16_t> counts);
  /// 今バスへ流している目標。まだ無ければ false（/motion/servo_states 用）。
  bool targets(int side, std::vector<int16_t> & out) const;
  void states(int side, std::vector<feetech_servo::ServoState> & out) const;
  /// 電流の区間集計を吸い出して 0 に戻す。
  BankCurrent takeCurrent(int side);

  // --- 素性 -------------------------------------------------------------
  bool has(int side) const {return bus_[side] != nullptr;}
  int numOpened() const;
  const std::string & dev(int side) const {return port_[side].dev;}
  const std::vector<uint8_t> & ids(int side) const {return port_[side].ids;}
  bool allowTorque() const {return opt_.allow_torque;}
  bool dryRun() const {return opt_.dry_run;}

  bool popEvent(Event & e) {return ev_.pop(e);}
  std::size_t takeDroppedEvents() {return ev_.takeDropped();}

private:
  void busLoop(int side);
  /// トルクオンの 3 段。入れられたら true。
  bool armSide(int side, const std::vector<uint16_t> & speeds, const std::vector<uint8_t> & accs);
  void accumCurrent(int side, const std::vector<feetech_servo::ServoState> & st);

  BankPort port_[kNumSide];
  BankOptions opt_;

  std::unique_ptr<feetech_servo::FeetechBus> bus_[kNumSide];
  std::thread thread_[kNumSide];
  std::atomic<bool> running_{false};
  std::atomic<bool> want_torque_{false};
  std::atomic<bool> torque_on_[kNumSide];

  mutable std::mutex cmd_mtx_[kNumSide];
  std::vector<int16_t> cmd_[kNumSide];
  bool cmd_valid_[kNumSide]{false, false};

  mutable std::mutex st_mtx_[kNumSide];
  std::vector<feetech_servo::ServoState> state_[kNumSide];
  BankCurrent cur_[kNumSide];      //!< st_mtx_ の中でだけ触る

  EventQueue ev_;
};

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__SERVO_BANK_HPP_
