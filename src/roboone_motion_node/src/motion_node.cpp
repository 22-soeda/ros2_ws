// motion ノード — ros-architecture §3 の「200Hz ループ」の実体。
//
//   受け取る: /cmd_walk (geometry_msgs/Twist)  歩行指令。teleop / behavior から 20Hz
//             /cmd_motion (std_msgs/String)    技名。イベント時
//             /estop (std_msgs/Bool)           脱力 / トルクオン。latched
//   出す:     /motion/state (std_msgs/String)  状態。変化時
//             /joint_states (sensor_msgs/JointState)  10Hz
//
// ===========================================================================
// スレッドの分け方
// ===========================================================================
//   main        rclcpp::spin。購読コールバックだけ
//   control     200Hz。状態機械 -> 目標姿勢 -> IK -> 生カウント。**シリアルを触らない**
//   bus x2      200Hz。生カウントを 1 パケットで書く + 一定周期で読む
//
// 制御ループをシリアルから切り離してあるのは、読み出しが 1 往復で数 ms かかり、
// 応答が欠けると最大 timeout_ms (20ms) 待つため。同じスレッドに置くと 200Hz の
// 周期が読み出しの都合で崩れる。書き込みは送りっぱなし (TX のみ) なので速い。
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
//   4) 実測姿勢からホーム姿勢へ torque_on_time 秒かけて補間する (control 側)
//
// teleop は「/cmd_motion home を送り、home_torque_delay 秒あけて /estop false」の
// 2 段で来る (teleop_node.py の docstring)。こちらはその順序に依存しない:
// home は「保持したい姿勢」を差し替えるだけで、動き出すのは 4) から。
//
// **起動直後の /estop false では武装しない。** teleop は起動時に latched で
// estop=false を撒くので、そのまま従うと launch しただけでサーボに電気が入って
// ホーム姿勢へ動き出す (機体が転がっているかもしれない)。require_home_before_arm
// が立っている間は、/cmd_motion を 1 回受けるまで武装しない。teleop の
// ホームポジション操作 (Options 長押し) は home -> estop false の順に来るので、
// 通常の操作手順はそのまま通る。
//
// **その場保持 (hold)。** /cmd_motion "hold" は「今の実測姿勢のまま」トルクを入れる
// 予約で、home と同じ 2 段 (hold -> estop false) で来る。武装時に保持姿勢を実測姿勢に
// 差し替えるので補間距離がゼロになり、その場で固まるだけ。武装が終わったら HOLD では
// なく STAY に入る (HOLD は tickWalk が足先を立位のスタンスへ上書きするので、寝た
// 姿勢からだと跳ねる)。転倒 -> 脱力のあと、寝た姿勢で武装して起き上がりに繋ぐ経路
// (docs/無線操縦_不足項目レビュー.md §4.3)。既にトルクが入っていれば、今の目標姿勢で
// 止まる (歩行も技も打ち切り)。
//
// ★寝た姿勢が IK の到達域の外だと、実測姿勢を IK で往復した目標がクランプされて
//   武装の瞬間にその軸が跳ねる。既存の武装 (ホームへの補間の起点) と同じ性質だが、
//   寝た姿勢で初めて効いてくる。初回は allow_torque:=false で STAY に入れ、
//   /motion/joint_commands と /joint_states を見比べてから使うこと。
//
// ===========================================================================
// 歩行と旋回
// ===========================================================================
// 歩行計画は roboone_walk_core (WalkEngine)。**平行移動のみ**で、機体は向きを
// 変えない。/cmd_walk の angular.z は使わない (旋回はキーフレームモーション
// turn_l / turn_r の担当)。angular.z が乗っていたら起動後 1 回だけ警告する。
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "feetech_servo/feetech_bus.hpp"
#include "roboone_motion_node/body_pose.hpp"
#include "roboone_motion_node/motion_library.hpp"
#include "roboone_motion_node/servo_map.hpp"
#include "roboone_walk_core/walk_engine.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;
namespace rmn = roboone_motion_node;
namespace rwc = roboone_walk_core;

namespace
{

constexpr double kR2D = 180.0 / M_PI;

/// /estop と /autonomy は teleop 側が latched で出す。購読側も合わせないと
/// マッチしない (VOLATILE な subscriber は TRANSIENT_LOCAL な publisher と繋がるが、
/// 後から上がったときに直前の値を受け取れない)。
rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

enum class State { RELAX, ARMING, HOLD, WALK, MOTION, STAY };

//! その場保持の技名。teleop の hold_motion と合わせる (motions.yaml には書かない)
constexpr char kHoldMotion[] = "hold";

const char * stateName(State s)
{
  switch (s) {
    case State::RELAX: return "RELAX";
    case State::ARMING: return "ARMING";
    case State::HOLD: return "HOLD";
    case State::WALK: return "WALK";
    case State::STAY: return "STAY";
    default: return "MOTION";
  }
}

}  // namespace

class MotionNode : public rclcpp::Node
{
public:
  MotionNode()
  : Node("motion")
  {
    // --- パラメータ -----------------------------------------------------
    port_[rmn::kRight] = declare_parameter<std::string>("port_right", "/dev/feetech_right");
    port_[rmn::kLeft] = declare_parameter<std::string>("port_left", "/dev/feetech_left");
    baud_ = declare_parameter<int>("baud", 1000000);
    goal_torque_ = declare_parameter<int>("goal_torque", 1000);
    // ★0 にしないこと。位置指令パケットの速度 (reg46/47) に 0 を書くと、この実機
    //   (HLS 系) は目標位置を受け取っても動かない。同じパケットの 44/45 = GOAL_TORQUE
    //   に 0 を書くと全軸まったく動かないのと同じ性質で、0 は「無制限」ではない。
    //   200Hz で目標を流すので、軌道が速度で頭打ちにならない値にしておく
    //   (feetech_demo_node が 2000/50 で流せている)。
    move_speed_ = declare_parameter<int>("move_speed", 2000);
    move_acc_ = declare_parameter<int>("move_acc", 50);

    const std::string feetech_share =
      ament_index_cpp::get_package_share_directory("feetech_servo");
    const std::string motion_share =
      ament_index_cpp::get_package_share_directory("roboone_motion_node");
    home_yaml_ = declare_parameter<std::string>(
      "home_yaml", feetech_share + "/config/servo_home.yaml");
    limits_yaml_ = declare_parameter<std::string>(
      "limits_yaml", feetech_share + "/config/servo_limits.yaml");
    motions_yaml_ = declare_parameter<std::string>(
      "motions_yaml", motion_share + "/config/motions.yaml");
    // gait.yaml は roboone_motion (Python 側・歩行計画の仕様原本) が持っている。
    // 見つからなくても gait_params.hpp の既定値で走れるので、ここでは落とさない。
    std::string gait_default;
    try {
      gait_default =
        ament_index_cpp::get_package_share_directory("roboone_motion") + "/config/gait.yaml";
    } catch (const std::exception &) {
      gait_default = motion_share + "/config/gait.yaml";
    }
    gait_yaml_ = declare_parameter<std::string>("gait_yaml", gait_default);
    std::string home_pose_default;
    try {
      home_pose_default =
        ament_index_cpp::get_package_share_directory("roboone_motion") + "/config/home_pose.yaml";
    } catch (const std::exception &) {
      home_pose_default = motion_share + "/config/home_pose.yaml";
    }
    home_pose_yaml_ = declare_parameter<std::string>("home_pose_yaml", home_pose_default);

    loop_hz_ = declare_parameter<double>("loop_hz", 200.0);
    read_hz_ = declare_parameter<double>("read_hz", 50.0);
    joint_state_hz_ = declare_parameter<double>("joint_state_hz", 10.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    torque_on_time_ = declare_parameter<double>("torque_on_time", 2.0);
    home_move_time_ = declare_parameter<double>("home_move_time", 1.5);
    // その場保持 (hold) で武装するときの補間時間。距離ゼロなので短くてよい
    hold_arm_time_ = declare_parameter<double>("hold_arm_time", 0.5);
    stance_y_offset_ = declare_parameter<double>("stance_y_offset", 0.0);
    walk_enable_ = declare_parameter<bool>("walk_enable", true);
    // 歩行エンジンが IDLE になってから、これだけ続いたら HOLD へ落とす（ばたつき止め）。
    walk_idle_hold_ = declare_parameter<double>("walk_idle_hold", 0.25);
    motion_interrupts_walk_ = declare_parameter<bool>("motion_interrupts_walk", true);
    require_home_before_arm_ = declare_parameter<bool>("require_home_before_arm", true);
    dry_run_ = declare_parameter<bool>("dry_run", false);
    // サーボにトルクを入れてよいか。**通常運用の既定は true。**
    //   false にすると、バスは開いて読むが enable_torque(true) と位置指令の送信だけを
    //   行わない。歩行計画・IK・モーション再生・/joint_states は実測値を使って全部
    //   回るので、機体を動かさずに操縦系・config・軌道の通し確認ができる。
    //   実装中の検証で走らせるときはこれを落とす (CLAUDE.md「実装中の検証では
    //   トルクを入れない」)。バスも開きたくないときは dry_run。
    allow_torque_ = declare_parameter<bool>("allow_torque", true);

    // 実機のサーボが逆に回る腕軸。脚は leg_config.hpp の AXIS_FLIP が同じ役目を
    // 持つが、腕は運動学が無いのでここで持つ。
    // 2026-08-28 実機で R8 / L9 / R10 が逆に回るのを確認。
    arm_invert_ = declare_parameter<std::vector<std::string>>(
      "arm_invert", std::vector<std::string>{"R8", "L9", "R10"});

    for (int s = 0; s < rmn::kNumSide; ++s) {torque_on_[s].store(false);}
  }

  ~MotionNode() override {shutdown();}

  /// 設定の読み込みとバスの初期化。false なら起動を諦める。
  bool init()
  {
    std::string err;
    if (!map_.load(
        home_yaml_, limits_yaml_, port_[rmn::kRight], port_[rmn::kLeft], arm_invert_, err))
    {
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s", map_.summary().c_str());

    if (!lib_.load(motions_yaml_, map_, err)) {
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
      return false;
    }
    for (const auto & w : lib_.warnings()) {RCLCPP_WARN(get_logger(), "%s", w.c_str());}
    RCLCPP_INFO(get_logger(), "モーション: %s", lib_.summary().c_str());

    loadGait();

    if (!loadHomePose(err)) {
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
      return false;
    }
    hold_pose_ = home_pose_;
    cur_pose_ = home_pose_;
    checkPoseReachable(home_pose_, "ホーム姿勢");

    // 歩行の左右間隔と実機の股間隔の食い違いは黙って埋めない (leg_config.hpp の TODO)。
    const double walk_half_mm = gait_.foot_spacing * 500.0;
    const double hip_half_mm = -roboone_kinematics::config::HIP_Y;
    if (std::abs(walk_half_mm + stance_y_offset_ - hip_half_mm) > 1.0) {
      RCLCPP_WARN(
        get_logger(),
        "歩行の足間隔 ±%.1fmm (+ stance_y_offset %.1f) が股間隔 ±%.1fmm と違う。"
        "横方向の力学は walk_core の値で計画されるので、実機で横揺れが合わない場合は"
        "gait.yaml の foot_spacing か stance_y_offset を詰めること",
        walk_half_mm, stance_y_offset_, hip_half_mm);
    }

    checkWalkEnvelope();

    if (!openBuses()) {return false;}

    // --- 通信 -----------------------------------------------------------
    pub_state_ = create_publisher<std_msgs::msg::String>("/motion/state", latchedQos());
    pub_joints_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    // ★あとから追えるようにするための 3 本（ros2 bag に残す）。
    //   /joint_states だけだと「実測」しか残らず、沈み込み (指令と実測の差) を
    //   後から見られない。指令・実測・負荷・電圧を揃えて初めて原因を切り分けられる。
    pub_cmd_ = create_publisher<sensor_msgs::msg::JointState>(
      "/motion/joint_commands", 10);
    pub_servo_ = create_publisher<sensor_msgs::msg::JointState>("/motion/servo_states", 10);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/motion/diagnostics", 10);
    sub_estop_ = create_subscription<std_msgs::msg::Bool>(
      "/estop", latchedQos(), [this](std_msgs::msg::Bool::SharedPtr m) {onEstop(*m);});
    sub_walk_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_walk", 10, [this](geometry_msgs::msg::Twist::SharedPtr m) {onWalk(*m);});
    sub_motion_ = create_subscription<std_msgs::msg::String>(
      "/cmd_motion", 10, [this](std_msgs::msg::String::SharedPtr m) {onMotion(*m);});

    publishState();
    RCLCPP_INFO(
      get_logger(),
      "motion 起動。%.0fHz / 読み %.0fHz / 指令途絶 %.2fs / トルクオン補間 %.1fs%s",
      loop_hz_, read_hz_, cmd_timeout_, torque_on_time_,
      dry_run_ ? " ★dry_run: サーボへ書かない" : "");
    if (allow_torque_) {
      RCLCPP_WARN(
        get_logger(),
        "★このノードはサーボにトルクを入れる。機体を支えておくこと。"
        "動き出すのは /cmd_motion を受けて /estop false になってからで、"
        "実測姿勢から %.1fs かけて保持姿勢へ移る。止めるのは /estop true (L1)",
        torque_on_time_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "allow_torque:=false — サーボにトルクを入れず、位置指令も送らない。"
        "歩行計画・IK・モーション再生・/joint_states は動く (通し確認用)");
    }
    if (require_home_before_arm_) {
      RCLCPP_INFO(
        get_logger(),
        "起動直後の /estop false では武装しない。/cmd_motion を 1 回受けるまで脱力のまま"
        " (teleop の Options 長押しで home が飛んでくる)");
    }

    running_ = true;
    for (int s = 0; s < rmn::kNumSide; ++s) {
      if (bus_[s]) {bus_thread_[s] = std::thread([this, s] {busLoop(s);});}
    }
    control_thread_ = std::thread([this] {controlLoop();});
    return true;
  }

  void shutdown()
  {
    if (!running_.exchange(false)) {return;}
    if (control_thread_.joinable()) {control_thread_.join();}
    for (int s = 0; s < rmn::kNumSide; ++s) {
      if (bus_thread_[s].joinable()) {bus_thread_[s].join();}
    }
    // 落ちるときは必ず脱力を置いていく。トルクを入れたまま消えると、機体は
    // 最後の目標角で固まったまま誰も止められなくなる。
    for (int s = 0; s < rmn::kNumSide; ++s) {
      if (!bus_[s] || dry_run_) {continue;}
      for (uint8_t id : map_.bus(s).ids) {bus_[s]->enable_torque(id, false);}
      bus_[s]->close();
    }
  }

private:
  // =====================================================================
  // 設定
  // =====================================================================
  void loadGait()
  {
    YAML::Node y;
    try {
      y = YAML::LoadFile(gait_yaml_);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "gait.yaml を読めない (%s)。gait_params.hpp の既定値で走る", e.what());
      return;
    }
    // 打ち間違いを黙って既定値に落とさないため、知らないキーは警告する
    // (Python 版 GaitParams.from_yaml が KeyError で落ちるのと同じ意図)。
    std::vector<std::string> known;
    auto pick = [&](const char * k, double & dst) {
        known.emplace_back(k);
        if (y[k]) {dst = y[k].as<double>();}
      };
    pick("z_c", gait_.z_c);
    pick("gravity", gait_.gravity);
    pick("t_step", gait_.t_step);
    pick("foot_spacing", gait_.foot_spacing);
    pick("swing_height", gait_.swing_height);
    pick("swing_lock_phase", gait_.swing_lock_phase);
    pick("td_overdrive", gait_.td_overdrive);
    pick("td_speed_max", gait_.td_speed_max);
    pick("step_clamp_x", gait_.step_clamp_x);
    pick("step_clamp_out", gait_.step_clamp_out);
    pick("step_clamp_in", gait_.step_clamp_in);
    pick("start_pushoff_max", gait_.start_pushoff_max);
    pick("k_dcm", gait_.k_dcm);
    pick("cmd_timeout", gait_.cmd_timeout);
    pick("loop_hz", gait_.loop_hz);
    for (const char * k : {"v_max", "a_max"}) {
      known.emplace_back(k);
      if (!y[k] || !y[k].IsSequence() || y[k].size() != 2) {continue;}
      double * dst = (std::string(k) == "v_max") ? gait_.v_max : gait_.a_max;
      dst[0] = y[k][0].as<double>();
      dst[1] = y[k][1].as<double>();
    }
    for (const auto & kv : y) {
      const std::string k = kv.first.as<std::string>();
      if (std::find(known.begin(), known.end(), k) == known.end()) {
        RCLCPP_WARN(get_logger(), "gait.yaml の知らないキー \"%s\" は無視した", k.c_str());
      }
    }
    walk_ = rwc::WalkEngine(gait_);
    RCLCPP_INFO(
      get_logger(), "歩行 z_c=%.3fm T=%.2fs W=%.3fm v_max=(%.2f, %.2f)",
      gait_.z_c, gait_.t_step, gait_.foot_spacing, gait_.v_max[0], gait_.v_max[1]);
  }

  /// ホーム姿勢を home_pose.yaml から読む。
  ///
  /// **足裏の位置姿勢で書かれている**（関節角ではない）。狙いは「胴体が直立・
  /// 足裏が水平・骨盤が所定の高さ」なので、そこに至る関節角は IK が出せばよい。
  /// 関節角で書くと AXIS_FLIP（サーボの回転方向）が絡んで左右で同じ数値が同じ姿勢に
  /// ならないが、足裏の位置姿勢なら鏡像は y の符号だけで済む。
  bool loadHomePose(std::string & err)
  {
    YAML::Node y;
    try {
      y = YAML::LoadFile(home_pose_yaml_);
    } catch (const std::exception & e) {
      err = "ホーム姿勢を読めない: " + home_pose_yaml_ + " (" + e.what() + ")";
      return false;
    }
    if (!y["foot"]) {
      err = home_pose_yaml_ + ": foot: が無い";
      if (y["legs"]) {
        err += "（書式が変わった。legs: の関節角ではなく foot: の "
          "height / x / y / rpy で書く）";
      }
      return false;
    }
    const YAML::Node & f = y["foot"];
    const double h = f["height"] ? f["height"].as<double>() : 282.0;
    const double x = f["x"] ? f["x"].as<double>() : 0.0;
    const double half = f["y"] ? f["y"].as<double>() : -roboone_kinematics::config::HIP_Y;
    double rpy[3]{0.0, 0.0, 0.0};
    if (f["rpy"] && f["rpy"].IsSequence() && f["rpy"].size() == 3) {
      for (int k = 0; k < 3; ++k) {rpy[k] = f["rpy"][k].as<double>() * M_PI / 180.0;}
    }
    if (!(h > 0.0)) {
      err = home_pose_yaml_ + ": foot.height は正の mm";
      return false;
    }

    home_pose_.arm.assign(map_.num_arm(), 0.0);
    for (int s = 0; s < rmn::kNumSide; ++s) {
      // 左右は鏡像。y と、roll・yaw の符号だけが反転する（pitch は左右同じ）。
      const double lat = (s == rmn::kLeft) ? +1.0 : -1.0;
      home_pose_.foot[s].p = roboone_kinematics::Vec3{x, lat * half, -h};
      home_pose_.foot[s].rpy[0] = rpy[0] * lat;
      home_pose_.foot[s].rpy[1] = rpy[1];
      home_pose_.foot[s].rpy[2] = rpy[2] * lat;
    }

    // 腕。ID をキーにすると左右共通、"R8" / "L8" と書くと片側だけ。
    if (y["arms"]) {
      const auto & arms = map_.arms();
      for (std::size_t a = 0; a < arms.size(); ++a) {
        if (const YAML::Node n = y["arms"][arms[a].id]) {
          home_pose_.arm[a] = n.as<double>();
        }
        if (const YAML::Node n = y["arms"][arms[a].name]) {   // 片側指定が優先
          home_pose_.arm[a] = n.as<double>();
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "ホーム姿勢 (%s): 足裏 高さ %.1fmm / 前後 %+.1fmm / 半間隔 %.1fmm / "
      "姿勢 rpy [%.1f, %.1f, %.1f] deg",
      home_pose_yaml_.c_str(), h, x, half,
      rpy[0] * kR2D, rpy[1] * kR2D, rpy[2] * kR2D);

    // 骨盤高さ (z_c) が gait.yaml と食い違っていたら言う。歩行の計画高さと実際の
    // 立位高さがずれると、LIPM の ω が実機と合わない。
    if (std::abs(h / 1000.0 - gait_.z_c) > 0.005) {
      RCLCPP_WARN(
        get_logger(),
        "ホーム姿勢の骨盤高さ %.3fm が gait.yaml の z_c %.3fm と違う",
        h / 1000.0, gait_.z_c);
    }
    // 足間隔も同様。歩行は gait.yaml の foot_spacing で計画される。
    if (std::abs(2.0 * half / 1000.0 - gait_.foot_spacing) > 0.005) {
      RCLCPP_WARN(
        get_logger(),
        "ホーム姿勢の足間隔 %.4fm が gait.yaml の foot_spacing %.4fm と違う",
        2.0 * half / 1000.0, gait_.foot_spacing);
    }
    return true;
  }

  /// 姿勢が IK で解けるかを起動時に確かめる。解けない config を実機で踏むと、
  /// 「動かない」のか「解けていない」のかが現場で切り分けられない。
  void checkPoseReachable(const rmn::BodyPose & p, const char * what) const
  {
    for (int s = 0; s < rmn::kNumSide; ++s) {
      double servo[roboone_kinematics::kNumJoints], theta[roboone_kinematics::kNumJoints];
      const rmn::LegSolve r = rmn::servoFromFootPose(map_.leg_params(s), p.foot[s], servo, theta);
      if (!r.ok()) {
        RCLCPP_ERROR(
          get_logger(), "%s の %s脚が解けない (ik=%d servo=%d)。config を直すこと",
          what, rmn::kSideTag[s], static_cast<int>(r.ik_status),
          static_cast<int>(r.servo_status));
      } else if (r.ankle_clamped) {
        RCLCPP_WARN(get_logger(), "%s の %s脚: 足首が可動域で丸められた", what, rmn::kSideTag[s]);
      }
    }
  }

  /// 歩行が実際に指令しうる足先の範囲が、脚 IK の到達域に収まっているかを起動時に見る。
  ///
  /// gait.yaml の v_max / step_clamp_* / swing_height は
  /// roboone_walk_core/src/gait_from_kinematics.cpp が **IK の到達域から逆算**した値
  /// (design 基準)。ここではその逆をやって、入っている値で本当に届くかを確かめる。
  /// 片方だけ手で書き換えたときに気付けるようにするための照合で、歩かせてから
  /// 「片脚だけ動かない」で悩まないための門。
  ///
  /// 見る箱は、名目立位 (股の真下・z = -z_c) からの
  ///   x  ±step_clamp_x            前後の着地点クランプ
  ///   y  +step_clamp_out / -step_clamp_in   外側 / 内側 (右脚基準)
  ///   z  0 .. +swing_height       遊脚の高さ
  /// の 8 隅 + 中心。足裏は水平のまま (walk_core は平行移動のみ)。
  void checkWalkEnvelope()
  {
    const double zc = gait_.z_c * 1000.0;
    const double cx = gait_.step_clamp_x * 1000.0;
    const double cout = gait_.step_clamp_out * 1000.0;
    const double cin = gait_.step_clamp_in * 1000.0;
    const double hsw = gait_.swing_height * 1000.0;
    const double half = gait_.foot_spacing * 500.0;

    int worst = static_cast<int>(rmn::ReachLevel::Design);
    rmn::FootPose worst_pose;
    int side_worst = rmn::kRight;

    for (int s = 0; s < rmn::kNumSide; ++s) {
      const double lat = (s == rmn::kLeft) ? +1.0 : -1.0;
      for (double dx : {-cx, 0.0, cx}) {
        for (double dy : {-cin, 0.0, cout}) {
          for (double dz : {0.0, hsw}) {
            rmn::FootPose f;
            // 外側 / 内側は脚ごとに向きが逆。dy > 0 を「外側」として左右に配る。
            f.p = roboone_kinematics::Vec3{
              dx, lat * (half + dy) + stance_y_offset_ * lat, -zc + dz};
            // 姿勢はホームと同じ（歩行中もその向きで出すので、同じ条件で見る）。
            for (int k = 0; k < 3; ++k) {f.rpy[k] = home_pose_.foot[s].rpy[k];}
            const int lv = static_cast<int>(rmn::reachLevel(map_.leg_params(s), f));
            if (lv < worst) {
              worst = lv;
              worst_pose = f;
              side_worst = s;
            }
          }
        }
      }
    }

    const auto lvl = static_cast<rmn::ReachLevel>(worst);
    const char * box = "歩行の足先の箱 (x±%.0f / 外%.0f 内%.0f / 高さ%.0f mm, z_c=%.0f)";
    if (lvl == rmn::ReachLevel::Design) {
      RCLCPP_INFO(get_logger(), (std::string(box) + " はすべて design 域の内側").c_str(),
        cx, cout, cin, hsw, zc);
      return;
    }
    if (lvl == rmn::ReachLevel::Mech) {
      // **これは想定内。** home_pose.yaml が「leg_bend 30 deg だと足首ピッチが常時
      // -30 deg で、足首パラレルリンクの設計可動域 (同時 ±15 deg) の外・機構限界の
      // 内で歩くことになる」と断ってある。届かないわけではないので情報に留める。
      RCLCPP_INFO(
        get_logger(),
        "歩行の足先の箱は mech 域には収まるが design 域は出る"
        " (最悪 %s脚 p=[%.1f, %.1f, %.1f])。立位で足首ピッチを使っているぶん"
        " 設計可動域 (同時 ±15 deg の菱形) の外で歩く。"
        " 内側に入れたいなら home_pose.yaml の foot.height を上げる",
        rmn::kSideTag[side_worst], worst_pose.p.x, worst_pose.p.y, worst_pose.p.z);
      return;
    }
    RCLCPP_ERROR(
      get_logger(),
      "歩行の足先の箱の隅に **届かない** (%s止まり: %s脚 p=[%.1f, %.1f, %.1f])。"
      "そのまま歩かせるとその位相で IK が解けず脚が止まる。gait.yaml の"
      " step_clamp_* / swing_height / z_c を"
      " `ros2 run roboone_walk_core gait_from_kinematics` で出し直すこと",
      rmn::reachLevelName(lvl), rmn::kSideTag[side_worst],
      worst_pose.p.x, worst_pose.p.y, worst_pose.p.z);
  }

  bool openBuses()
  {
    int opened = 0;
    for (int s = 0; s < rmn::kNumSide; ++s) {
      if (dry_run_) {
        RCLCPP_WARN(get_logger(), "dry_run: %s を開かない", port_[s].c_str());
        continue;
      }
      auto b = std::make_unique<FeetechBus>(
        port_[s], baud_, 0, 20, feetech_servo::Family::kHls);
      if (!b->open()) {
        RCLCPP_ERROR(
          get_logger(), "%s を開けない (udev 固定名・電源・dialout 権限を確認)",
          port_[s].c_str());
        continue;
      }
      // HLS 系は位置指令の 44/45 が GOAL_TORQUE。0 のままだと全軸まったく動かない。
      b->set_goal_torque(static_cast<uint16_t>(goal_torque_));
      const auto alive = b->scan(map_.bus(s).ids);
      if (alive.size() != map_.bus(s).ids.size()) {
        RCLCPP_WARN(
          get_logger(), "%s: 応答 %zu/%zu 軸。欠けた軸は指令が届かない",
          rmn::kSideTag[s], alive.size(), map_.bus(s).ids.size());
      }
      bus_[s] = std::move(b);
      ++opened;
    }
    if (dry_run_) {return true;}
    if (opened == 0) {
      RCLCPP_ERROR(get_logger(), "どちらのバスも開けなかった");
      return false;
    }
    if (opened < rmn::kNumSide) {
      walk_enable_ = false;
      RCLCPP_ERROR(get_logger(), "片側のバスしか無いので歩行を無効にした (単脚では歩けない)");
    }
    return true;
  }

  // =====================================================================
  // 購読
  // =====================================================================
  /// teleop は脱力中 2 秒おきに /estop true を再送してくる (latched の取りこぼし保険。
  /// teleop_node.py の _estop_beat)。**変化したときだけ**言う。毎回書くと、
  /// コントローラが切れているあいだログが 2 秒おきに埋まって他が読めなくなる。
  void onEstop(const std_msgs::msg::Bool & m)
  {
    const bool prev = estop_.exchange(m.data);
    if (m.data == prev) {return;}
    if (m.data) {
      RCLCPP_WARN(get_logger(), "/estop true — 脱力する");
    } else {
      RCLCPP_INFO(get_logger(), "/estop false — トルクを入れてよい");
    }
  }

  void onWalk(const geometry_msgs::msg::Twist & m)
  {
    std::lock_guard<std::mutex> lk(walk_mtx_);
    walk_cmd_[0] = m.linear.x;
    walk_cmd_[1] = m.linear.y;
    walk_stamp_ = nowSec();
    if (std::abs(m.angular.z) > 1e-3 && !warned_yaw_) {
      warned_yaw_ = true;
      RCLCPP_WARN(
        get_logger(),
        "/cmd_walk の angular.z は使わない。歩行は平行移動のみで、旋回は"
        " キーフレームモーション (turn_l / turn_r) の担当");
    }
  }

  void onMotion(const std_msgs::msg::String & m)
  {
    std::lock_guard<std::mutex> lk(req_mtx_);
    motion_req_ = m.data;
    got_motion_ = true;
  }

  // =====================================================================
  // 制御ループ (200Hz)。シリアルには触らない
  // =====================================================================
  void controlLoop()
  {
    const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    auto next = std::chrono::steady_clock::now();
    const int js_div = std::max(1, static_cast<int>(loop_hz_ / std::max(1.0, joint_state_hz_)));
    int tick = 0;

    while (running_) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      const double now = nowSec();
      const double dt = 1.0 / loop_hz_;

      // --- 1) 技の要求を取り出す ---------------------------------------
      std::string req;
      {
        std::lock_guard<std::mutex> lk(req_mtx_);
        if (got_motion_) {
          req = motion_req_;
          got_motion_ = false;
          seen_motion_ = true;
        }
      }

      // --- 2) 技の要求を捌く -------------------------------------------
      //
      // ★武装の判定より **先** に置く。teleop のホームポジション操作は
      //   /cmd_motion "home" -> (0.1s) -> /estop false の順に来るので、先に home を
      //   捌いておけば hold_pose_ がホーム姿勢になった状態で武装に入れる。
      //   逆順にすると、最初の 1 本が「立ち上げ中なので出さない」で捨てられる。
      if (!req.empty()) {handleMotionRequest(req, now);}

      // --- 3) 脱力 (最優先。どの状態からでも即座に落ちる) ----------------
      if (estop_.load()) {
        if (state_ != State::RELAX) {
          setState(State::RELAX);
          player_.stop();
          walk_.reset();
          want_torque_.store(false);
          arm_in_place_ = false;
          stay_after_arm_ = false;
        }
      } else if (state_ == State::RELAX && canArm()) {
        // ★実測姿勢が 1 度も取れていないうちは武装しない。
        //
        // 立ち上げは「実測姿勢 -> 保持姿勢」の補間なので、起点が分からないまま
        // 始めると補間にならない。以前ここで hold_pose_ を起点に代用していたが、
        // 起点と終点が同じ = 補間が無いのと同じで、**トルクが入るだけで動かない**
        // (2026-08-28 実機で発生)。代用すると今度は保持姿勢へ一気に飛ぶので、
        // どちらにしても代用してはいけない。取れるまで待って理由を出す。
        if (!have_measured_) {
          std::string why;
          bool ok = false;
          measuredPose(ok, &why);
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "実測姿勢が取れないので武装しない: %s", why.empty() ? "(理由不明)" : why.c_str());
        } else {
          want_torque_.store(true);
          if (torqueReady()) {
            // cur_pose_ は RELAX 中に毎周期更新している「最後に取れた実測姿勢」。
            bool ok = false;
            const rmn::BodyPose meas = measuredPose(ok);
            if (ok) {cur_pose_ = meas;}
            if (arm_in_place_) {
              // hold: 実測姿勢をそのまま保持姿勢にする。補間距離ゼロ = その場で固まる
              arm_in_place_ = false;
              stay_after_arm_ = true;
              hold_pose_ = cur_pose_;
              startBlend(hold_pose_, hold_arm_time_, now, "hold");
              setState(State::ARMING);
              RCLCPP_INFO(
                get_logger(),
                "トルクオン (その場保持)。実測姿勢 R[%.1f, %.1f, %.1f] のまま動かない",
                cur_pose_.foot[rmn::kRight].p.x, cur_pose_.foot[rmn::kRight].p.y,
                cur_pose_.foot[rmn::kRight].p.z);
            } else {
              startBlend(hold_pose_, torque_on_time_, now, "arming");
              setState(State::ARMING);
              RCLCPP_INFO(
                get_logger(),
                "トルクオン。実測姿勢 R[%.1f, %.1f, %.1f] から %.1fs かけて保持姿勢へ移る",
                cur_pose_.foot[rmn::kRight].p.x, cur_pose_.foot[rmn::kRight].p.y,
                cur_pose_.foot[rmn::kRight].p.z, torque_on_time_);
            }
          }
        }
      }

      // --- 4) 状態ごとに目標姿勢を作る ----------------------------------
      switch (state_) {
        case State::RELAX:
          // 脱力中は目標を作らない。手で動かされるので、実測を追いかけて
          // おくと復帰時の補間の起点がそのまま使える。
          {
            std::string why;
            bool ok = false;
            const rmn::BodyPose meas = measuredPose(ok, &why);
            if (ok) {
              cur_pose_ = meas;
            } else {
              RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "実測姿勢が取れない: %s", why.c_str());
            }
          }
          break;

        case State::ARMING:
        case State::MOTION:
          if (!player_.sample(now, cur_pose_)) {
            hold_pose_ = cur_pose_;
            if (stay_after_arm_) {
              // その場保持の武装が終わった。HOLD にすると tickWalk が足先を立位の
              // スタンスへ上書きして跳ねるので、目標を作らない STAY で止める
              stay_after_arm_ = false;
              setState(State::STAY);
            } else {
              setState(State::HOLD);
            }
          }
          break;

        case State::HOLD:
        case State::WALK:
          tickWalk(now, dt);
          break;

        case State::STAY:
          // その場保持。目標は作らず cur_pose_ をそのまま出し続ける。
          // 抜けるのは技 (起き上がり) か home か脱力。/cmd_walk は効かない
          break;
      }

      // --- 5) IK を通してバスへ渡す -------------------------------------
      if (state_ != State::RELAX) {writeTargets(cur_pose_);}

      // --- 6) 状態通知と /joint_states ----------------------------------
      if (++tick % js_div == 0) {publishJointStates();}

      std::this_thread::sleep_until(next);
      // 何かで大きく遅れたら次の周期に合わせ直す (取り戻そうとして暴走させない)
      const auto t = std::chrono::steady_clock::now();
      if (t > next + std::chrono::milliseconds(50)) {next = t;}
    }
  }

  /// 起動直後の latched な /estop false で勝手に武装しないための門。
  bool canArm() const {return !require_home_before_arm_ || seen_motion_;}

  bool torqueReady() const
  {
    for (int s = 0; s < rmn::kNumSide; ++s) {
      if (bus_[s] && !torque_on_[s].load()) {return false;}
    }
    return true;      // dry_run はバスが無いので即 true
  }

  /// 今の姿勢 (cur_pose_) から to へ time 秒で移る 1 区間の補間を仕込む。
  /// what は /motion/state に出る名前 ("home" など)。
  void startBlend(const rmn::BodyPose & to, double time, double now, const char * what)
  {
    blend_motion_.name = what;
    blend_motion_.return_home = false;
    blend_motion_.frames.assign(1, rmn::KeyFrame{});
    rmn::KeyFrame & f = blend_motion_.frames[0];
    f.dt = std::max(0.05, time);
    f.linear = false;
    f.has_arm.assign(map_.num_arm(), 1);
    f.arm = to.arm;
    for (int s = 0; s < rmn::kNumSide; ++s) {
      f.has_foot[s] = true;
      f.foot[s] = to.foot[s];
    }
    player_.start(blend_motion_, cur_pose_, home_pose_, now);
  }

  void handleMotionRequest(const std::string & name, double now)
  {
    if (name == kHoldMotion) {
      // その場保持 (冒頭の説明)。脱力中なら「次の武装は実測姿勢のまま」の予約、
      // トルクが入っていれば今の目標姿勢で止まる
      if (state_ == State::RELAX) {
        arm_in_place_ = true;
        RCLCPP_INFO(get_logger(), "その場保持を予約した (トルクが入っても動かず、実測姿勢を保持する)");
        return;
      }
      walk_.reset();
      player_.stop();
      hold_pose_ = cur_pose_;
      setState(State::STAY);
      RCLCPP_INFO(get_logger(), "その場保持 (今の目標姿勢で止まる)");
      return;
    }
    if (name != "home" && !lib_.find(name)) {
      RCLCPP_WARN(get_logger(), "知らない技 \"%s\" (motions.yaml に無い)", name.c_str());
      return;
    }

    if (name == "home") {
      // ホームは「保持したい姿勢」を差し替えるのが本体。脱力中ならそれだけ。
      // トルクが入っていれば、そこへ home_move_time 秒かけて移る。
      hold_pose_ = home_pose_;
      arm_in_place_ = false;   // hold の予約は home で上書き
      if (state_ == State::RELAX) {
        RCLCPP_INFO(get_logger(), "ホーム姿勢を予約した (トルクが入ったらそこへ移る)");
        return;
      }
      walk_.reset();
      startBlend(home_pose_, home_move_time_, now, "home");
      setState(State::MOTION);
      RCLCPP_INFO(get_logger(), "ホームポジションへ (%.1fs)", home_move_time_);
      return;
    }

    if (state_ == State::RELAX) {
      RCLCPP_WARN(get_logger(), "脱力中なので技 \"%s\" は出さない", name.c_str());
      return;
    }
    if (state_ == State::ARMING) {
      RCLCPP_WARN(get_logger(), "立ち上げ中なので技 \"%s\" は出さない", name.c_str());
      return;
    }
    if (state_ == State::WALK) {
      if (!motion_interrupts_walk_) {
        RCLCPP_WARN(get_logger(), "歩行中なので技 \"%s\" は出さない", name.c_str());
        return;
      }
      RCLCPP_WARN(get_logger(), "歩行を打ち切って技 \"%s\" に入る", name.c_str());
    }
    walk_.reset();
    player_.start(*lib_.find(name), cur_pose_, home_pose_, now);
    setState(State::MOTION);
    RCLCPP_INFO(get_logger(), "技 \"%s\" 再生 (%.2fs)", name.c_str(), player_.duration());
  }

  /// 歩行計画を 1 周期進めて、足先目標を cur_pose_ に書く。
  void tickWalk(double now, double dt)
  {
    double vx = 0.0, vy = 0.0;
    {
      std::lock_guard<std::mutex> lk(walk_mtx_);
      // 指令が途絶えたらゼロを入れる。最後の指令を保持しない
      // (無線が切れたまま歩き続けるのがいちばん困る)。
      if (walk_stamp_ > 0.0 && now - walk_stamp_ <= cmd_timeout_) {
        vx = walk_cmd_[0];
        vy = walk_cmd_[1];
      }
    }
    if (!walk_enable_) {vx = vy = 0.0;}

    const rwc::WalkOutputs o = walk_.update(vx, vy, dt);

    // 世界座標 [m] -> 骨盤水平系 [m] -> Σ_B [mm]
    const rwc::Vec3 fp[rmn::kNumSide] = {o.right_foot_in_pelvis(), o.left_foot_in_pelvis()};
    for (int s = 0; s < rmn::kNumSide; ++s) {
      const double lat = (s == rmn::kLeft) ? +1.0 : -1.0;
      cur_pose_.foot[s].p = roboone_kinematics::Vec3{
        fp[s][0] * 1000.0,
        fp[s][1] * 1000.0 + lat * stance_y_offset_,
        fp[s][2] * 1000.0};
      // 足裏の向きは**ホーム姿勢と同じ**にする。walk_core は平行移動のみで機体は
      // 向きを変えないので、足裏の姿勢は歩行中も変わらない。ここを水平に固定すると、
      // home_pose.yaml の rpy（膝のしなりを補正するつま先上げ）が立位でしか効かず、
      // 歩き出した瞬間に機体が後傾する。立位と歩行で別々の値を持たない。
      for (int k = 0; k < 3; ++k) {
        cur_pose_.foot[s].rpy[k] = home_pose_.foot[s].rpy[k];
      }
    }
    // 腕は歩行では動かさない。直前の保持値をそのまま持ち越す。
    cur_pose_.arm = hold_pose_.arm;

    // 実際に出している足先が設計可動域を出ていないか見張る。gait.yaml が
    // 到達域から逆算されている前提が崩れる (v_max を手で上げた、z_c がずれた) と
    // ここに出る。指令自体は送る — 止めるほうが危ないので、記録だけ残す。
    if (++reach_tick_ >= static_cast<int>(loop_hz_ / 10.0)) {
      reach_tick_ = 0;
      for (int s = 0; s < rmn::kNumSide; ++s) {
        // design 域の外は起動時に断ってあるので騒がない。**機構として届かない**
        // ところへ行ったときだけ言う (そこは IK が解けず脚が止まる位相)。
        const rmn::ReachLevel lv = rmn::reachLevel(map_.leg_params(s), cur_pose_.foot[s]);
        if (lv < rmn::ReachLevel::Mech) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "%s脚の足先が機構の到達域の外 (%s): p=[%.1f, %.1f, %.1f]",
            rmn::kSideTag[s], rmn::reachLevelName(lv),
            cur_pose_.foot[s].p.x, cur_pose_.foot[s].p.y, cur_pose_.foot[s].p.z);
        }
      }
    }

    // 歩き始めは指令がレート制限で立ち上がるので、歩行エンジンの状態が
    // IDLE と START の間を数十 ms 単位で往復する（2026-08-28 実機で 55ms 周期の
    // ばたつきを確認）。そのたびに状態を publish すると /motion/state が荒れ、
    // hold_pose_ も上書きされ続ける。**止まったと見なすのは少し待ってから。**
    const bool moving = (o.state != rwc::State::IDLE);
    if (moving) {
      idle_since_ = -1.0;
      if (state_ != State::WALK) {setState(State::WALK);}
    } else if (state_ == State::WALK) {
      if (idle_since_ < 0.0) {idle_since_ = now;}
      if (now - idle_since_ >= walk_idle_hold_) {
        hold_pose_ = cur_pose_;
        setState(State::HOLD);
      }
    }
  }

  // =====================================================================
  // 目標姿勢 -> サーボ指令
  // =====================================================================
  void writeTargets(const rmn::BodyPose & pose)
  {
    for (int s = 0; s < rmn::kNumSide; ++s) {
      std::vector<int16_t> pos(map_.bus(s).ids.size(), 0);

      double servo[roboone_kinematics::kNumJoints], theta[roboone_kinematics::kNumJoints];
      const rmn::LegSolve r = rmn::servoFromFootPose(map_.leg_params(s), pose.foot[s], servo, theta);
      if (!r.ok()) {
        // 解けない目標は**送らない**。前周期の指令が生きたままになるので、
        // 機体は直前の姿勢で止まる。ゼロや中途半端な解を送るより安全。
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "%s脚の IK が解けない (ik=%d servo=%d)。この周期の指令は送らない",
          rmn::kSideTag[s], static_cast<int>(r.ik_status), static_cast<int>(r.servo_status));
        continue;
      }
      if (r.ankle_clamped) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "%s脚: 足首が可動域で丸められた", rmn::kSideTag[s]);
      }

      // どの軸が丸められたかを名前で出す。「丸められた」だけだと、原点が窓の外に
      // 1 カウントはみ出しているような無害なものと、可動域を超えた指令の区別が付かない。
      std::string clamped_axes;
      for (std::size_t j = 0; j < roboone_kinematics::kNumJoints; ++j) {
        bool c = false;
        pos[j] = static_cast<int16_t>(map_.leg_count_from_servo(s, j, servo[j], &c));
        if (c) {
          clamped_axes += (clamped_axes.empty() ? "" : ",");
          clamped_axes += std::string("ID") + std::to_string(rmn::kLegServoId[j]) +
            "(" + rmn::kLegJointName[j] + ")";
        }
      }
      {
        std::lock_guard<std::mutex> lk(theta_mtx_);
        for (std::size_t j = 0; j < roboone_kinematics::kNumJoints; ++j) {
          theta_cmd_[s][j] = theta[j];
        }
      }

      // 腕。運動学は無いので T ポーズ基準の deg をそのままカウントに直す。
      const auto & arms = map_.arms();
      std::size_t k = roboone_kinematics::kNumJoints;
      for (std::size_t a = 0; a < arms.size(); ++a) {
        if (arms[a].side != s) {continue;}
        const double deg = (a < pose.arm.size()) ? pose.arm[a] : 0.0;
        {
          std::lock_guard<std::mutex> lk(arm_mtx_);
          if (arm_cmd_deg_.size() != arms.size()) {arm_cmd_deg_.assign(arms.size(), 0.0);}
          arm_cmd_deg_[a] = deg;
        }
        bool c = false;
        pos[k++] = static_cast<int16_t>(map_.arm_count_from_deg(a, deg, &c));
        if (c) {
          clamped_axes += (clamped_axes.empty() ? "" : ",");
          clamped_axes += arms[a].name;
        }
      }
      if (!clamped_axes.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "%s: 指令が servo_limits.yaml の窓で丸められた: %s",
          rmn::kSideTag[s], clamped_axes.c_str());
      }

      std::lock_guard<std::mutex> lk(cmd_mtx_[s]);
      bus_pos_[s] = std::move(pos);
      bus_pos_valid_[s] = true;
    }
  }

  /// 実測のサーボ角から今の姿勢を組み立てる。読めなければ ok = false。
  /// why が非 null なら、失敗したときにどの脚がどう駄目だったかを書く。
  rmn::BodyPose measuredPose(bool & ok, std::string * why = nullptr)
  {
    rmn::BodyPose p;
    p.arm.assign(map_.num_arm(), 0.0);
    ok = true;
    for (int s = 0; s < rmn::kNumSide; ++s) {
      std::vector<ServoState> st;
      {
        std::lock_guard<std::mutex> lk(st_mtx_[s]);
        st = bus_state_[s];
      }
      if (st.size() < map_.bus(s).ids.size()) {
        ok = false;
        if (why) {*why += std::string(rmn::kSideTag[s]) + "脚: まだ読めていない ";}
        continue;
      }
      double servo[roboone_kinematics::kNumJoints];
      double theta[roboone_kinematics::kNumJoints]{};
      bool all = true;
      for (std::size_t j = 0; j < roboone_kinematics::kNumJoints; ++j) {
        if (!st[j].valid) {
          all = false;
          if (why) {
            *why += std::string(rmn::kSideTag[s]) + "脚: ID" +
              std::to_string(kLegServoIdOf(j)) + " が応答しない ";
          }
          break;
        }
        servo[j] = map_.leg_servo_from_count(s, j, st[j].pos);
      }
      if (!all) {
        ok = false;
        continue;
      }
      // theta は 0 で初期化してある。legJointsFromServo は膝で失敗すると
      // theta[KNEE] を書かずに早期リターンするので、未初期化のまま fk() に入れると
      // 出鱈目な姿勢が出る (それを「実測」として補間の起点にすると事故る)。
      const roboone_kinematics::LegServoStatus lst =
        rmn::footPoseFromServo(map_.leg_params(s), servo, p.foot[s], theta, th6_seed_[s]);
      if (lst != roboone_kinematics::LegServoStatus::Ok) {
        ok = false;
        if (why) {
          *why += std::string(rmn::kSideTag[s]) + "脚: 変換できない(status=" +
            std::to_string(static_cast<int>(lst)) + ", 膝サーボ " +
            std::to_string(static_cast<int>(map_.leg_tpose_deg_from_count(
                s, roboone_kinematics::KNEE, st[roboone_kinematics::KNEE].pos))) + "deg) ";
        }
      }
      {
        std::lock_guard<std::mutex> lk(theta_mtx_);
        for (std::size_t j = 0; j < roboone_kinematics::kNumJoints; ++j) {
          theta_meas_[s][j] = theta[j];
        }
      }

      const auto & arms = map_.arms();
      std::size_t k = roboone_kinematics::kNumJoints;
      for (std::size_t a = 0; a < arms.size(); ++a) {
        if (arms[a].side != s) {continue;}
        if (k < st.size() && st[k].valid) {p.arm[a] = map_.arm_deg_from_count(a, st[k].pos);}
        ++k;
      }
    }
    {
      std::lock_guard<std::mutex> lk(arm_mtx_);
      arm_meas_deg_ = p.arm;
    }
    if (ok) {have_measured_ = true;}
    return p;
  }

  // =====================================================================
  // バススレッド (1 本ずつ。書き込み 200Hz / 読み出し read_hz)
  // =====================================================================
  void busLoop(int s)
  {
    const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    auto next = std::chrono::steady_clock::now();
    const int read_div = std::max(1, static_cast<int>(loop_hz_ / std::max(1.0, read_hz_)));
    const auto & ids = map_.bus(s).ids;
    const std::vector<uint16_t> speeds(ids.size(), static_cast<uint16_t>(move_speed_));
    const std::vector<uint8_t> accs(ids.size(), static_cast<uint8_t>(move_acc_));
    std::vector<ServoState> st;
    int tick = 0;

    while (running_) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

      // --- 読み出し (先にやる。トルクを入れる前に実測が要るため) ----------
      if (tick++ % read_div == 0) {
        st.clear();
        bus_[s]->sync_read_states(ids, st);
        std::lock_guard<std::mutex> lk(st_mtx_[s]);
        bus_state_[s] = st;
      }

      const bool want = want_torque_.load();

      // --- トルクを入れる (docstring「トルクの入れ方」の 1)-3)) ------------
      if (want && !torque_on_[s].load() && !allow_torque_) {
        // 禁止中。「入ったことにして」制御ループだけ進める。位置指令も送らない
        // ので機体は動かないが、状態機械・IK・モーション再生は実機の実測値を
        // 使って最後まで通る。
        torque_on_[s].store(true);
        RCLCPP_WARN(
          get_logger(),
          "%s: allow_torque:=false のためトルクを入れない (指令は計算するが送らない)",
          rmn::kSideTag[s]);
      } else if (want && !torque_on_[s].load()) {
        std::vector<ServoState> now;
        bus_[s]->sync_read_states(ids, now);
        std::vector<int16_t> hold(ids.size(), 0);
        bool all = true;
        for (std::size_t k = 0; k < ids.size(); ++k) {
          if (k < now.size() && now[k].valid) {
            hold[k] = static_cast<int16_t>(now[k].pos);
          } else {
            all = false;
          }
        }
        if (!all) {
          // 実測が揃わないままトルクを入れると、読めなかった軸だけが古い目標角へ
          // 飛ぶ。**揃うまで入れない。** 低電圧だと応答が間欠的に欠ける実機の癖が
          // あるので、ここで止まるときはまず電源を疑う。
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "%s: 実測が揃わないのでトルクを入れない (電圧を確認)", rmn::kSideTag[s]);
        } else {
          // 1) 実測位置を目標として書き 2) トルクを入れる。この順でないと、
          //    レジスタに残っている古い目標角へ全速で飛ぶ。
          bus_[s]->sync_write_position(ids, hold, speeds, accs);
          int n = 0;
          for (uint8_t id : ids) {n += bus_[s]->enable_torque(id, true) ? 1 : 0;}
          {
            std::lock_guard<std::mutex> lk(cmd_mtx_[s]);
            bus_pos_[s] = hold;              // control 側が上書きするまでの当座の目標
            bus_pos_valid_[s] = true;
          }
          torque_on_[s].store(true);
          RCLCPP_INFO(
            get_logger(), "%s: 実測位置を目標にしてトルクオン (%d/%zu 軸)",
            rmn::kSideTag[s], n, ids.size());
        }
      } else if (!want && torque_on_[s].load()) {
        // 禁止中でも「切る」側は必ず通す (入っていないものを切っても害はない)。
        for (uint8_t id : ids) {bus_[s]->enable_torque(id, false);}
        torque_on_[s].store(false);
        {
          std::lock_guard<std::mutex> lk(cmd_mtx_[s]);
          bus_pos_valid_[s] = false;
        }
        RCLCPP_WARN(get_logger(), "%s: トルクオフ (脱力)", rmn::kSideTag[s]);
      }

      // --- 位置指令 -------------------------------------------------------
      if (torque_on_[s].load() && allow_torque_) {
        std::vector<int16_t> pos;
        {
          std::lock_guard<std::mutex> lk(cmd_mtx_[s]);
          if (bus_pos_valid_[s]) {pos = bus_pos_[s];}
        }
        if (pos.size() == ids.size()) {
          bus_[s]->sync_write_position(ids, pos, speeds, accs);
        }
      }

      std::this_thread::sleep_until(next);
      const auto t = std::chrono::steady_clock::now();
      if (t > next + std::chrono::milliseconds(50)) {next = t;}
    }
  }

  // =====================================================================
  // 通知
  // =====================================================================
  void setState(State s)
  {
    if (state_ == s) {return;}
    state_ = s;
    publishState();
  }

  /// /motion/state の書式。**behavior が読み方をテストで固定しているので変えない。**
  ///
  ///     <状態>              RELAX / ARMING / HOLD / WALK / STAY
  ///                         (STAY = その場保持。behavior の ready_states に無いので
  ///                          「歩けない」扱いになる。寝ている間はそれで正しい)
  ///     MOTION:<技名>       再生中だけ。技名はコロンの後ろ ("MOTION:punch_r")
  ///
  /// 先頭語が状態で、コロン区切りの後置は MOTION のときの技名だけ。将来ここに
  /// 支持脚や位相を足すなら **空白区切りで後ろに足す** こと (behavior 側は
  /// 空白以降を無視するように作られている)。先頭語の意味を変える・コロンの
  /// 使い方を増やす変更は、behavior (roboone_behavior) と同時に直す。
  ///
  /// 転倒 (FALL 相当) はまだ無い。姿勢を知る手段が機体に無く、/imu/data を出す
  /// ノードが存在しないため (RealSense が出すのは生の /camera/imu で、姿勢は
  /// 入っていない)。入れるなら IMU フィルタが先。
  void publishState()
  {
    std_msgs::msg::String m;
    m.data = stateName(state_);
    if (state_ == State::MOTION && player_.active()) {m.data += ":" + player_.name();}
    pub_state_->publish(m);
    RCLCPP_INFO(get_logger(), "/motion/state -> %s", m.data.c_str());
  }

  /// 記録用の 4 本を出す。**あとから ros2 bag で追えることが目的。**
  ///
  ///   /joint_states           実測の関節角 [rad]（標準の型なので rviz などでも読める）
  ///   /motion/joint_commands  指令の関節角 [rad]。同じ並び・同じ名前
  ///   /motion/servo_states    サーボ空間。position = 実測カウント /
  ///                           velocity = 目標カウント / effort = 負荷 (-1..1)
  ///   /motion/diagnostics     バスごとの電圧・最高温度・応答した軸数・欠損累計
  ///
  /// 「荷重で沈む」を後から見るには **指令と実測の差**が要る。/joint_states だけだと
  /// 実測しか残らず、沈んでいたのか指令がそもそもそこだったのか区別が付かない。
  /// 関節空間とサーボ空間の両方を出すのは、間に膝 4 節リンクの変換が挟まっていて、
  /// 「サーボは追従しているがリンクがたわんでいる」と「サーボが追従していない」を
  /// 分けたいから。前者は関節空間だけに差が出て、後者は両方に出る。
  ///
  /// ★servo_states の velocity は速度ではなく目標カウント。JointState に
  ///   「目標」の場所が無いので velocity を借りている（record_topics.yaml にも明記）。
  void publishJointStates()
  {
    const auto stamp = get_clock()->now();
    sensor_msgs::msg::JointState meas, cmd;
    meas.header.stamp = stamp;
    cmd.header.stamp = stamp;

    {
      std::lock_guard<std::mutex> lk(theta_mtx_);
      for (int s = 0; s < rmn::kNumSide; ++s) {
        for (std::size_t j = 0; j < roboone_kinematics::kNumJoints; ++j) {
          const std::string n = std::string(rmn::kSideTag[s]) + "_" + rmn::kLegJointName[j];
          meas.name.push_back(n);
          meas.position.push_back(theta_meas_[s][j]);
          cmd.name.push_back(n);
          cmd.position.push_back(theta_cmd_[s][j]);
        }
      }
    }
    {
      // 腕は運動学を持たないので、T ポーズ基準の実測 deg を rad に直して入れる。
      // 脚の position が「関節角」なのに対し、腕は「サーボ角」であることに注意。
      std::lock_guard<std::mutex> lk(arm_mtx_);
      const auto & arms = map_.arms();
      for (std::size_t a = 0; a < arms.size(); ++a) {
        meas.name.push_back(arms[a].name);
        meas.position.push_back(
          (a < arm_meas_deg_.size() ? arm_meas_deg_[a] : 0.0) / kR2D);
        cmd.name.push_back(arms[a].name);
        cmd.position.push_back(
          (a < arm_cmd_deg_.size() ? arm_cmd_deg_[a] : 0.0) / kR2D);
      }
    }
    pub_joints_->publish(meas);
    pub_cmd_->publish(cmd);

    // --- サーボ空間（生カウント）と診断 --------------------------------
    sensor_msgs::msg::JointState sv;
    sv.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticArray da;
    da.header.stamp = stamp;

    for (int s = 0; s < rmn::kNumSide; ++s) {
      std::vector<ServoState> st;
      std::vector<int16_t> goal;
      {
        std::lock_guard<std::mutex> lk(st_mtx_[s]);
        st = bus_state_[s];
      }
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_[s]);
        if (bus_pos_valid_[s]) {goal = bus_pos_[s];}
      }
      const auto & ids = map_.bus(s).ids;

      double volt = 0.0;
      int nvalid = 0, tmax = 0;
      uint8_t errbits = 0;
      for (std::size_t k = 0; k < ids.size(); ++k) {
        const ServoState e = (k < st.size()) ? st[k] : ServoState{};
        sv.name.push_back(std::string(rmn::kSideTag[s]) + "_ID" + std::to_string(ids[k]));
        sv.position.push_back(e.valid ? e.pos : std::numeric_limits<double>::quiet_NaN());
        sv.velocity.push_back(k < goal.size() ? goal[k] : 0.0);
        // 負荷は 0.1% 単位で符号ビット付き。-1..1 に正規化して入れる。
        sv.effort.push_back(e.valid ? e.load / 1000.0 : 0.0);
        if (e.valid) {
          volt += e.volt;
          ++nvalid;
          tmax = std::max(tmax, e.temp);
          errbits |= e.err;
        }
      }

      diagnostic_msgs::msg::DiagnosticStatus ds;
      ds.name = std::string("motion/bus_") + rmn::kSideTag[s];
      ds.hardware_id = map_.bus(s).port;
      const double v = nvalid ? volt / nvalid : 0.0;
      // 低電圧だとサーボの応答が間欠的に欠ける実機の癖があるので、そこを段の基準にする。
      ds.level = (nvalid == 0) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR
        : (nvalid < static_cast<int>(ids.size()) || errbits || v < 10.5)
        ? diagnostic_msgs::msg::DiagnosticStatus::WARN
        : diagnostic_msgs::msg::DiagnosticStatus::OK;
      ds.message = std::to_string(nvalid) + "/" + std::to_string(ids.size()) + " 軸が応答";
      auto kv = [&ds](const char * k, const std::string & v2) {
          diagnostic_msgs::msg::KeyValue e;
          e.key = k;
          e.value = v2;
          ds.values.push_back(e);
        };
      kv("電圧[V]", std::to_string(v));
      kv("最高温度[C]", std::to_string(tmax));
      kv("応答軸数", std::to_string(nvalid));
      kv("エラービット", std::to_string(static_cast<int>(errbits)));
      kv("トルク", torque_on_[s].load() ? (allow_torque_ ? "on" : "禁止中") : "off");
      da.status.push_back(ds);
    }
    pub_servo_->publish(sv);
    pub_diag_->publish(da);
  }

  static int kLegServoIdOf(std::size_t j) {return rmn::kLegServoId[j];}

  double nowSec() const {return get_clock()->now().nanoseconds() * 1e-9;}

  // --- 設定 -------------------------------------------------------------
  std::string port_[rmn::kNumSide];
  int baud_ = 1000000, goal_torque_ = 1000, move_speed_ = 0, move_acc_ = 0;
  std::string home_yaml_, limits_yaml_, motions_yaml_, gait_yaml_, home_pose_yaml_;
  std::vector<std::string> arm_invert_;
  double loop_hz_ = 200.0, read_hz_ = 50.0, joint_state_hz_ = 10.0;
  double cmd_timeout_ = 0.5, torque_on_time_ = 2.0, home_move_time_ = 1.5;
  double hold_arm_time_ = 0.5;
  bool arm_in_place_ = false;    // hold を受けた: 次の武装は実測姿勢のまま
  bool stay_after_arm_ = false;  // その武装が終わったら HOLD ではなく STAY へ
  double stance_y_offset_ = 0.0;
  double walk_idle_hold_ = 0.25;
  bool walk_enable_ = true, motion_interrupts_walk_ = true;
  bool require_home_before_arm_ = true, dry_run_ = false, allow_torque_ = false;

  // --- 実体 -------------------------------------------------------------
  rmn::ServoMap map_;
  rmn::MotionLibrary lib_;
  rwc::GaitParams gait_;
  rwc::WalkEngine walk_{rwc::GaitParams{}};
  rmn::MotionPlayer player_;
  rmn::Motion blend_motion_;
  rmn::BodyPose home_pose_, hold_pose_, cur_pose_;
  double th6_seed_[rmn::kNumSide]{0.0, 0.0};

  std::unique_ptr<FeetechBus> bus_[rmn::kNumSide];
  std::thread bus_thread_[rmn::kNumSide], control_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> estop_{false};
  std::atomic<bool> want_torque_{false};
  std::atomic<bool> torque_on_[rmn::kNumSide];

  std::mutex cmd_mtx_[rmn::kNumSide];
  std::vector<int16_t> bus_pos_[rmn::kNumSide];
  bool bus_pos_valid_[rmn::kNumSide]{false, false};

  std::mutex st_mtx_[rmn::kNumSide];
  std::vector<ServoState> bus_state_[rmn::kNumSide];

  std::mutex theta_mtx_, arm_mtx_;
  double theta_cmd_[rmn::kNumSide][roboone_kinematics::kNumJoints]{};
  double theta_meas_[rmn::kNumSide][roboone_kinematics::kNumJoints]{};
  std::vector<double> arm_meas_deg_, arm_cmd_deg_;

  std::mutex walk_mtx_, req_mtx_;
  double walk_cmd_[2]{0.0, 0.0};
  double walk_stamp_ = 0.0;
  std::string motion_req_;
  bool got_motion_ = false, seen_motion_ = false, warned_yaw_ = false;
  int reach_tick_ = 0;
  double idle_since_ = -1.0;
  bool have_measured_ = false;   //!< 実測姿勢が 1 度でも取れたか

  State state_ = State::RELAX;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_servo_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_walk_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_motion_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionNode>();
  if (!node->init()) {
    RCLCPP_FATAL(node->get_logger(), "初期化に失敗した。起動しない");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  node->shutdown();          // 脱力を置いていく (デストラクタ任せにしない)
  rclcpp::shutdown();
  return 0;
}
