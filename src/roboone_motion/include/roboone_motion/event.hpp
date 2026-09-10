// 層から上へ渡す「出来事」。**ログの代わり。**
//
// servo_bank / pose_codec / motion_config / motion_control は ROS を知らない。
// ログを出したい場面は多いが、そのために rclcpp へ依存させると「ログのためだけに
// ROS が要る」コードになり、実機なしで回せなくなる（motion_selftest から叩けない）。
//
// そこで各層は出来事をキューに積むだけにして、**ROS の殻 (motion_node) が取り出して
// RCLCPP_* へ流す**。throttle_ms は RCLCPP_*_THROTTLE の間隔で、0 なら毎回出す。
//
//     Event e;
//     while (bank_.popEvent(e)) {logEvent(e);}
//
// キューは上限で頭から捨てる。**取り出し側が止まってもメモリを食い潰さない**ほうが、
// 200Hz で回る層としては正しい（捨てたことは dropped で分かる）。
#ifndef ROBOONE_MOTION__EVENT_HPP_
#define ROBOONE_MOTION__EVENT_HPP_

#include <deque>
#include <mutex>
#include <string>
#include <utility>

namespace roboone_motion
{

enum class EventLevel { Info = 0, Warn, Error };

struct Event
{
  EventLevel level = EventLevel::Info;
  int throttle_ms = 0;        //!< 0 なら毎回出す。> 0 は RCLCPP_*_THROTTLE の間隔
  //! 間引きの単位。**同じ出来事に同じキーを付ける。** 空なら text をキーにする。
  //!
  //! text をそのままキーにすると、座標や status を含む文が毎回別物になって
  //! 間引きが効かない（「足先が到達域の外」は毎周期 p が変わる）。逆に呼び出し箇所
  //! ごとに束ねると左右が潰れるので、キーには側の別も入れる。
  std::string key;
  std::string text;
};

/// 出来事の置き場。**200Hz のスレッドから積まれるのでロックを持つ。**
class EventQueue
{
public:
  explicit EventQueue(std::size_t cap = 256)
  : cap_(cap) {}

  void info(std::string t, int throttle_ms = 0, std::string key = {})
  {
    push(Event{EventLevel::Info, throttle_ms, std::move(key), std::move(t)});
  }
  void warn(std::string t, int throttle_ms = 0, std::string key = {})
  {
    push(Event{EventLevel::Warn, throttle_ms, std::move(key), std::move(t)});
  }
  void error(std::string t, int throttle_ms = 0, std::string key = {})
  {
    push(Event{EventLevel::Error, throttle_ms, std::move(key), std::move(t)});
  }

  void push(Event e)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (q_.size() >= cap_) {
      q_.pop_front();
      ++dropped_;
    }
    q_.push_back(std::move(e));
  }

  /// 1 件取り出す。空なら false。
  bool pop(Event & out)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (q_.empty()) {return false;}
    out = std::move(q_.front());
    q_.pop_front();
    return true;
  }

  /// 溢れて捨てた件数を吸い出して 0 に戻す。
  std::size_t takeDropped()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return std::exchange(dropped_, static_cast<std::size_t>(0));
  }

private:
  mutable std::mutex mtx_;
  std::deque<Event> q_;
  std::size_t cap_;
  std::size_t dropped_ = 0;
};

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__EVENT_HPP_
