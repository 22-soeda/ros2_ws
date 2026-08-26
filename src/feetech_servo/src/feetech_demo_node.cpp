// feetech_demo_node: 2本のシリアルバス（コントローラ2台）で閉ループ制御を回すデモ。
//
// 各バスをそれぞれ独立したコールバックグループ + タイマで駆動し、
// MultiThreadedExecutor 上で 2本を並列に回す。1サイクルの中身は:
//   1) 書き込み: sync_write_position で全軸の目標位置を1パケット送信
//   2) 読み出し: sync_read_states で全軸の状態を1往復で取得
//   3) publish : sensor_msgs/JointState を /feetech/bus<i>/joint_states へ
//
// 安全のため enable_motion 既定 false。false のときはトルクを入れず、書き込みは
// 現在位置をそのまま目標にする（機体は動かない）。true のとき起点位置まわりに
// サイン波で往復させる。
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "feetech_servo/feetech_manager.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::Mode;
using feetech_servo::ServoState;

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kStepsPerRev = 4096.0;

double steps_to_rad(int steps) { return steps / kStepsPerRev * kTwoPi; }
int deg_to_steps(double deg) { return static_cast<int>(std::lround(deg / 360.0 * kStepsPerRev)); }
}  // namespace

class FeetechDemoNode : public rclcpp::Node
{
public:
  FeetechDemoNode()
  : Node("feetech_demo")
  {
    // --- パラメータ ---
    ports_ = declare_parameter<std::vector<std::string>>(
      "ports", {"/dev/ttyACM0", "/dev/ttyACM1"});
    baud_ = declare_parameter<int>("baud", 1000000);
    const auto ids_shared = declare_parameter<std::vector<int64_t>>("ids", {1});
    rate_hz_ = declare_parameter<double>("rate_hz", 50.0);
    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    amplitude_deg_ = declare_parameter<double>("amplitude_deg", 20.0);
    period_s_ = declare_parameter<double>("period_s", 4.0);
    move_speed_ = declare_parameter<int>("move_speed", 2000);
    move_acc_ = declare_parameter<int>("move_acc", 50);

    RCLCPP_INFO(
      get_logger(), "起動: %zuバス, baud=%d, rate=%.0fHz, enable_motion=%s",
      ports_.size(), baud_, rate_hz_, enable_motion_ ? "true" : "false");

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);

    for (size_t i = 0; i < ports_.size(); ++i) {
      // 各バス個別の ids を許可（ids_<i>）。無ければ共有 ids を使う。
      std::vector<int64_t> ids_i =
        declare_parameter<std::vector<int64_t>>("ids_" + std::to_string(i), ids_shared);

      auto ctx = std::make_shared<BusCtx>();
      ctx->index = i;
      ctx->port = ports_[i];
      for (int64_t v : ids_i) {
        ctx->ids.push_back(static_cast<uint8_t>(v));
      }

      FeetechBus * bus = mgr_.add_bus(ports_[i], baud_);
      if (!bus) {
        RCLCPP_ERROR(get_logger(), "バス%zu %s を開けない。スキップ", i, ports_[i].c_str());
        continue;
      }
      ctx->bus = bus;

      auto alive = bus->scan(ctx->ids);
      if (alive.size() != ctx->ids.size()) {
        RCLCPP_WARN(
          get_logger(), "バス%zu %s: 応答 %zu/%zu軸", i, ports_[i].c_str(),
          alive.size(), ctx->ids.size());
      }
      ctx->ids = alive;
      if (ctx->ids.empty()) {
        RCLCPP_ERROR(get_logger(), "バス%zu %s: 応答軸なし。スキップ", i, ports_[i].c_str());
        continue;
      }

      // 起点位置を1回読んで保持（サイン波の中心 / hold 目標）。
      std::vector<ServoState> st;
      bus->sync_read_states(ctx->ids, st);
      ctx->home.assign(ctx->ids.size(), 2048);
      for (size_t k = 0; k < st.size(); ++k) {
        if (st[k].valid) {
          ctx->home[k] = st[k].pos;
        }
      }

      if (enable_motion_) {
        for (uint8_t id : ctx->ids) {
          bus->init_motor(id, Mode::kPosition, /*enable_torque=*/true);
        }
        RCLCPP_INFO(get_logger(), "バス%zu: トルクON, サイン波駆動", i);
      } else {
        RCLCPP_INFO(get_logger(), "バス%zu: 読み取りのみ（トルクOFF, 機体は動かない）", i);
      }

      ctx->pub = create_publisher<sensor_msgs::msg::JointState>(
        "feetech/bus" + std::to_string(i) + "/joint_states", 10);

      // バスごとに専用の MutuallyExclusive グループ → 別バスと並列に回る。
      ctx->cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      auto ctx_ptr = ctx;  // shared_ptr をタイマにキャプチャ
      ctx->timer = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this, ctx_ptr]() { this->cycle(*ctx_ptr); },
        ctx->cb_group);

      buses_.push_back(ctx);
    }

    if (buses_.empty()) {
      RCLCPP_FATAL(get_logger(), "有効なバスが無い。ポート/配線/権限(dialout)を確認して");
    }
    start_ = now();
  }

  ~FeetechDemoNode() override
  {
    // 動かしていたらトルクを切って安全に停止。
    if (enable_motion_) {
      for (auto & ctx : buses_) {
        for (uint8_t id : ctx->ids) {
          ctx->bus->enable_torque(id, false);
        }
      }
    }
    mgr_.close_all();
  }

private:
  struct BusCtx
  {
    size_t index = 0;
    std::string port;
    FeetechBus * bus = nullptr;
    std::vector<uint8_t> ids;
    std::vector<int> home;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub;
    rclcpp::CallbackGroup::SharedPtr cb_group;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<ServoState> states;  // 再利用バッファ
    uint64_t cycles = 0;
  };

  // 1バスぶんの閉ループ 1サイクル。
  void cycle(BusCtx & ctx)
  {
    const double t = (now() - start_).seconds();

    // --- 1) 書き込み（目標位置を作る）---
    std::vector<int16_t> goals(ctx.ids.size());
    for (size_t k = 0; k < ctx.ids.size(); ++k) {
      int target = ctx.home[k];
      if (enable_motion_) {
        const double phase = kTwoPi * t / period_s_;
        target += deg_to_steps(amplitude_deg_ * std::sin(phase));
      }
      target = std::clamp(target, 0, 4095);
      goals[k] = static_cast<int16_t>(target);
    }
    if (enable_motion_) {
      std::vector<uint16_t> spd(ctx.ids.size(), static_cast<uint16_t>(move_speed_));
      std::vector<uint8_t> acc(ctx.ids.size(), static_cast<uint8_t>(move_acc_));
      ctx.bus->sync_write_position(ctx.ids, goals, spd, acc);
    }

    // --- 2) 読み出し（全軸1往復）---
    const int ok = ctx.bus->sync_read_states(ctx.ids, ctx.states);

    // --- 3) publish ---
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.reserve(ctx.states.size());
    msg.position.reserve(ctx.states.size());
    msg.velocity.reserve(ctx.states.size());
    msg.effort.reserve(ctx.states.size());
    for (const auto & s : ctx.states) {
      if (!s.valid) {
        continue;  // このサイクルで読めなかった軸は載せない（0を誤って流さない）
      }
      msg.name.push_back("bus" + std::to_string(ctx.index) + "_id" + std::to_string(s.id));
      msg.position.push_back(steps_to_rad(s.pos));   // rad
      msg.velocity.push_back(steps_to_rad(s.speed)); // rad/s 相当（step/s→rad/s）
      msg.effort.push_back(static_cast<double>(s.current)); // mA
    }
    ctx.pub->publish(msg);

    // 1秒ごとに1回、通信品質をログ。
    if (++ctx.cycles % static_cast<uint64_t>(std::max(1.0, rate_hz_)) == 0) {
      RCLCPP_INFO(
        get_logger(), "バス%zu %s: 読取 %d/%zu軸, tx=%lu rxfail=%lu",
        ctx.index, ctx.port.c_str(), ok, ctx.ids.size(),
        static_cast<unsigned long>(ctx.bus->tx_count()),
        static_cast<unsigned long>(ctx.bus->rx_fail()));
    }
  }

  feetech_servo::FeetechManager mgr_;
  std::vector<std::shared_ptr<BusCtx>> buses_;

  std::vector<std::string> ports_;
  int baud_ = 1000000;
  double rate_hz_ = 50.0;
  bool enable_motion_ = false;
  double amplitude_deg_ = 20.0;
  double period_s_ = 4.0;
  int move_speed_ = 2000;
  int move_acc_ = 50;
  rclcpp::Time start_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeetechDemoNode>();
  // バスを並列に回すため MultiThreaded。
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
