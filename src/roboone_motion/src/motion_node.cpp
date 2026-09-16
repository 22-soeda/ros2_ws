// motion ノード — ros-architecture §3 の「200Hz ループ」の実体。
//
//   受け取る: /cmd_walk (geometry_msgs/Twist)  歩行指令。teleop / behavior から 20Hz
//             /cmd_motion (std_msgs/String)    技名。イベント時
//             /estop (std_msgs/Bool)           脱力 / トルクオン。latched
//             /camera/imu (sensor_msgs/Imu)    RealSense 内蔵 IMU の生値。200Hz
//   出す:     /motion/state (std_msgs/String)  状態。変化時
//             /joint_states 他 (10Hz)          記録用の 5 本 (publishTelemetry を見よ)
//             /motion/stab (Float64MultiArray) 安定化の中身 (publishStab を見よ)
//   サービス: /motion/imu_zero (std_srvs/Trigger)  立位で IMU の零点を取る
//
// ===========================================================================
// このファイルは「ROS の殻」
// ===========================================================================
// 中身は 4 つの層に分けてある。**どれも ROS を知らない**ので、実機なしで
// motion_selftest から叩ける。このファイルがやるのは、パラメータを読んで層を
// 組み立て、200Hz で 3 行を回し、各層が積んだ出来事をログへ流すことだけ。
//
//   servo_bank.hpp     サーボ層  2 バスの開閉・スレッド・トルク・生カウントの授受
//   pose_codec.hpp     変換層    BodyPose <-> 生カウント（Σ_U / Σ_B の境界）
//   motion_config.hpp  設定層    YAML の読み込みと起動時の門
//   motion_control.hpp 生成層    状態機械・歩行・技の再生
//   imu_attitude.hpp / stabilizer.hpp   IMU の姿勢推定と、それを使う安定化
//
// 1 周期はこれだけ:
//
//     bank_.states()  ->  codec_.decode()  ->  ctrl_.step()  ->  stab_.update()
//                     ->  codec_.encode(目標, 補正)  ->  bank_.setTargets()
//
// ===========================================================================
// スレッドの分け方
// ===========================================================================
//   main        rclcpp::spin。購読コールバックだけ（値を controller へ置く）。
//               /camera/imu は姿勢推定まで回す（C++ で数 µs。ロックは imu_mtx_）
//   control     200Hz。上の 1 周期。**シリアルを触らない**
//   bus x2      ServoBank が持つ。生カウントを 1 パケットで書く + 一定周期で読む
//
// 制御ループをシリアルから切り離してあるのは、読み出しが 1 往復で数 ms かかり、
// 応答が欠けると最大 timeout_ms (20ms) 待つため。同じスレッドに置くと 200Hz の
// 周期が読み出しの都合で崩れる。書き込みは送りっぱなし (TX のみ) なので速い。
//
// ★トルクの入れ方（事故の起きる場所）は servo_bank.hpp、状態遷移の順序の約束は
//   motion_control.hpp の冒頭にある。実機で踏んだ話はそちらに書いてある。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "roboone_motion/imu_attitude.hpp"
#include "roboone_motion/motion_config.hpp"
#include "roboone_motion/motion_control.hpp"
#include "roboone_motion/pose_codec.hpp"
#include "roboone_motion/servo_bank.hpp"
#include "roboone_motion/stabilizer.hpp"

using feetech_servo::ServoState;
namespace rm = roboone_motion;
namespace rk = roboone_kinematics;

namespace
{

constexpr double kR2D = 180.0 / M_PI;

/// /motion/stab の並び。**順番を変えたら bag を読む側も直す**（名前は layout にも載せる）。
constexpr const char * kStabFields[] = {
  "t",            //  0 [s] ノードの時計
  "imu_ok",       //  1 IMU が新しいか (0/1)
  "imu_age",      //  2 [s] 最後の IMU サンプルからの経過
  "imu_rx_lag",   //  3 [s] 受信時刻 - header.stamp。RealSense は機器の時計を換算して
                  //    stamp を付けるので**絶対の遅れにはならない**（実測で -22ms 前後）。
                  //    値そのものではなく、途中で跳ねていないかを見る
  "roll",         //  4 [rad] + で右へ倒れている
  "pitch",        //  5 [rad] + で前へ倒れている
  "gyro_x",       //  6 [rad/s] Σ_B。ロールの速さ
  "gyro_y",       //  7 [rad/s] Σ_B。ピッチの速さ
  "gyro_z",       //  8 [rad/s]
  "accel_weight", //  9 加速度をどれだけ信じたか [0,1]
  "at_rest",      // 10 静止判定 (0/1)
  "active",       // 11 補正を入れる条件が揃っているか (0/1)
  "fade",         // 12 補正の出し入れ [0,1]
  "gate",         // 13 着地の前後でゲインを弱めているか (0/1)
  "w_R",          // 14 右脚に効かせる割合
  "w_L",          // 15 左脚に効かせる割合
  "u_roll",       // 16 [rad] 足裏を胴体に対して回したい量
  "u_pitch",      // 17 [rad]
  "ank_R_th5",    // 18 [rad] 右足首 θ5（ピッチ）に足した量
  "ank_R_th6",    // 19 [rad] 右足首 θ6（ロール）
  "ank_L_th5",    // 20 [rad]
  "ank_L_th6",    // 21 [rad]
  "torso",        // 22 [rad] 胴体の前傾に足した量
  "walk_state",   // 23 0 IDLE / 1 START / 2 STEP / 3 STOP / 4 ESTOP / -1 歩行計画を回していない
  "support",      // 24 +1 左足支持 / -1 右足支持 / 0 両足
  "phase",        // 25 歩の位相
  "corr_dropped", // 26 補正を外して出した脚の数（この周期）
  "ff_ax",        // 27 [m/s^2] IMU に教えた計画上の重心加速度（前）
  "ff_ay",        // 28 [m/s^2] 同（左）
};
constexpr std::size_t kStabN = sizeof(kStabFields) / sizeof(kStabFields[0]);

/// 実行中に変えられる double パラメータ（安定化と IMU）。範囲は descriptor で縛るので、
/// 範囲外の ros2 param set は rclcpp が弾く。
struct DoubleParam
{
  const char * name;
  double lo, hi;
  double rm::StabGains::* gain;     //!< どちらか片方だけ
  double rm::ImuOptions::* imu;
  const char * desc;
};
const DoubleParam kDoubleParams[] = {
  {"stab.kd_pitch", 0.0, 0.5, &rm::StabGains::kd_pitch, nullptr,
    "足首ピッチのジャイロ減衰 [rad/(rad/s)]。文書の目安 0.05。最初に上げる"},
  {"stab.kd_roll", 0.0, 0.5, &rm::StabGains::kd_roll, nullptr,
    "足首ロールのジャイロ減衰 [rad/(rad/s)]"},
  {"stab.kp_pitch", 0.0, 3.0, &rm::StabGains::kp_pitch, nullptr,
    "足首ピッチの傾き比例 [rad/rad]。kd の後に上げる"},
  {"stab.kp_roll", 0.0, 3.0, &rm::StabGains::kp_roll, nullptr,
    "足首ロールの傾き比例 [rad/rad]"},
  {"stab.k_torso", 0.0, 1.5, &rm::StabGains::k_torso, nullptr,
    "胴体を起こす股の補正 [rad/rad]。1 で測った前後の傾きの分だけ逆に回す"},
  {"stab.ankle_clamp", 0.0, 0.3, &rm::StabGains::ankle_clamp, nullptr,
    "足首の補正の上限 [rad]（1 軸あたり）"},
  {"stab.torso_clamp", 0.0, 0.35, &rm::StabGains::torso_clamp, nullptr,
    "胴体の補正の上限 [rad]"},
  {"stab.rate_limit", 0.01, 50.0, &rm::StabGains::rate_limit, nullptr,
    "補正の変化の速さの上限 [rad/s]"},
  {"stab.gate_pre_td", 0.0, 0.2, &rm::StabGains::gate_pre_td, nullptr,
    "予定の着地のこれだけ前からゲインを弱める [s]"},
  {"stab.gate_post_td", 0.0, 0.2, &rm::StabGains::gate_post_td, nullptr,
    "予定の着地のこれだけ後までゲインを弱める [s]"},
  {"stab.gate_scale", 0.0, 1.0, &rm::StabGains::gate_scale, nullptr,
    "着地の前後でゲインに掛ける倍率"},
  {"stab.fade_time", 0.0, 5.0, &rm::StabGains::fade_time, nullptr,
    "補正を入れる・抜くのにかける時間 [s]"},
  {"stab.imu_timeout", 0.01, 1.0, &rm::StabGains::imu_timeout, nullptr,
    "これより古い IMU は使わない [s]"},
  {"stab.lift_phase", 0.01, 0.5, &rm::StabGains::lift_phase, nullptr,
    "遊脚の補正を抜き切る歩の位相"},
  {"imu.tau_c", 0.05, 20.0, nullptr, &rm::ImuOptions::tau_c,
    "加速度で傾きを引き戻す時定数 [s]。長いほどジャイロを信じる"},
  {"imu.gyro_lpf_hz", 0.0, 100.0, nullptr, &rm::ImuOptions::gyro_lpf_hz,
    "角速度の 1 次 LPF [Hz]。0 で素通し"},
  {"imu.accel_band", 0.01, 1.0, nullptr, &rm::ImuOptions::accel_band,
    "|f| が g からこの割合だけ外れたら加速度を使わない"},
};

double steadySec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// /estop と /autonomy は teleop 側が latched で出す。購読側も合わせないと
/// マッチしない (VOLATILE な subscriber は TRANSIENT_LOCAL な publisher と繋がるが、
/// 後から上がったときに直前の値を受け取れない)。
rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).reliable().transient_local();
}

/// share を引く。無いパッケージなら空。
std::string shareOf(const char * pkg)
{
  try {
    return ament_index_cpp::get_package_share_directory(pkg);
  } catch (const std::exception &) {
    return {};
  }
}

}  // namespace

class MotionNode : public rclcpp::Node
{
public:
  MotionNode()
  : Node("motion")
  {
    // --- バス -----------------------------------------------------------
    port_[rm::kRight] = declare_parameter<std::string>("port_right", "/dev/feetech_right");
    port_[rm::kLeft] = declare_parameter<std::string>("port_left", "/dev/feetech_left");
    bank_opt_.baud = declare_parameter<int>("baud", 1000000);
    bank_opt_.goal_torque = declare_parameter<int>("goal_torque", 2047);
    // ★0 にしないこと。位置指令パケットの速度 (reg46/47) に 0 を書くと、この実機
    //   (HLS 系) は目標位置を受け取っても動かない。同じパケットの 44/45 = GOAL_TORQUE
    //   に 0 を書くと全軸まったく動かないのと同じ性質で、0 は「無制限」ではない。
    bank_opt_.move_speed = declare_parameter<int>("move_speed", 2000);
    // 加速度 (reg41)。単位 100 step/s^2、0-254。254 = 25400 step/s^2 ≒ 39 rad/s^2。
    // 以前の 50 (≒ 7.7 rad/s^2) では遊脚の膝が最高速度に乗るまで 0.24s かかり、
    // 足首の安定化の補正も追従しなかった。**0 は使わない**（ベンダ資料では「最大」
    // だが、この HLS 系は速度 0・トルク 0 を「無効」として扱うので、確かめるまで避ける）。
    bank_opt_.move_acc = declare_parameter<int>("move_acc", 254);

    // --- config のパス ---------------------------------------------------
    const std::string feetech = shareOf("feetech_servo");
    const std::string motion = shareOf("roboone_motion");
    // gait.yaml / home_pose.yaml は roboone_walk_ref (歩行計画の仕様原本) が持っている。
    const std::string ref = shareOf("roboone_walk_ref");
    home_yaml_ = declare_parameter<std::string>("home_yaml", feetech + "/config/servo_home.yaml");
    limits_yaml_ =
      declare_parameter<std::string>("limits_yaml", feetech + "/config/servo_limits.yaml");
    motions_yaml_ =
      declare_parameter<std::string>("motions_yaml", motion + "/config/motions.yaml");
    gait_yaml_ = declare_parameter<std::string>(
      "gait_yaml", (ref.empty() ? motion : ref) + "/config/gait.yaml");
    home_pose_yaml_ = declare_parameter<std::string>(
      "home_pose_yaml", (ref.empty() ? motion : ref) + "/config/home_pose.yaml");

    // --- 周期 -------------------------------------------------------------
    loop_hz_ = declare_parameter<double>("loop_hz", 200.0);
    read_hz_ = declare_parameter<double>("read_hz", 50.0);
    joint_state_hz_ = declare_parameter<double>("joint_state_hz", 10.0);
    bank_opt_.loop_hz = loop_hz_;
    bank_opt_.read_hz = read_hz_;
    ctrl_opt_.loop_hz = loop_hz_;

    // --- 状態機械 ---------------------------------------------------------
    ctrl_opt_.cmd_timeout = declare_parameter<double>("cmd_timeout", 0.5);
    ctrl_opt_.torque_on_time = declare_parameter<double>("torque_on_time", 2.0);
    ctrl_opt_.home_move_time = declare_parameter<double>("home_move_time", 1.5);
    ctrl_opt_.hold_arm_time = declare_parameter<double>("hold_arm_time", 0.5);
    ctrl_opt_.stance_y_offset = declare_parameter<double>("stance_y_offset", 0.0);
    ctrl_opt_.walk_enable = declare_parameter<bool>("walk_enable", true);
    ctrl_opt_.walk_idle_hold = declare_parameter<double>("walk_idle_hold", 0.25);
    ctrl_opt_.motion_interrupts_walk = declare_parameter<bool>("motion_interrupts_walk", true);
    ctrl_opt_.require_home_before_arm =
      declare_parameter<bool>("require_home_before_arm", true);

    // --- 安全 -------------------------------------------------------------
    bank_opt_.dry_run = declare_parameter<bool>("dry_run", false);
    // サーボにトルクを入れてよいか。**通常運用の既定は true。**
    //   false にすると、バスは開いて読むが enable_torque(true) と位置指令の送信だけを
    //   行わない。歩行計画・IK・モーション再生・/joint_states は実測値を使って全部
    //   回るので、機体を動かさずに操縦系・config・軌道の通し確認ができる。
    //   実装中の検証で走らせるときはこれを落とす (CLAUDE.md「実装中の検証では
    //   トルクを入れない」)。バスも開きたくないときは dry_run。
    bank_opt_.allow_torque = declare_parameter<bool>("allow_torque", true);

    // 実機のサーボが逆に回る腕軸。脚は leg_config.hpp の AXIS_FLIP が同じ役目を
    // 持つが、腕は運動学が無いのでここで持つ。
    // 2026-08-28 実機で R8 / L9 / R10 が逆に回るのを確認。
    arm_invert_ = declare_parameter<std::vector<std::string>>(
      "arm_invert", std::vector<std::string>{"R8", "L9", "R10"});

    declareStabParams();
  }

  ~MotionNode() override {shutdown();}

  /// 設定の読み込みとバスの初期化。false なら起動を諦める。
  bool init()
  {
    const std::string motion = shareOf("roboone_motion");
    const std::string feetech =
      shareOf("feetech_servo").empty() ? motion : shareOf("feetech_servo");
    const std::string ref =
      shareOf("roboone_walk_ref").empty() ? motion : shareOf("roboone_walk_ref");
    rm::fallbackPath(motions_yaml_, motion + "/config/motions.yaml", "motions_yaml", boot_);
    rm::fallbackPath(home_yaml_, feetech + "/config/servo_home.yaml", "home_yaml", boot_);
    rm::fallbackPath(limits_yaml_, feetech + "/config/servo_limits.yaml", "limits_yaml", boot_);
    rm::fallbackPath(home_pose_yaml_, ref + "/config/home_pose.yaml", "home_pose_yaml", boot_);
    rm::fallbackPath(gait_yaml_, ref + "/config/gait.yaml", "gait_yaml", boot_);
    drain();

    std::string err;
    if (!map_.load(
        home_yaml_, limits_yaml_, port_[rm::kRight], port_[rm::kLeft], arm_invert_, err))
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

    // --- 設定と門（全部 motion_config。ここは実機に触らない）---------------
    rm::checkMotionLegServo(map_, lib_, boot_);
    rm::loadGait(gait_yaml_, gait_, boot_);
    rm::checkGait(gait_, boot_);
    if (!rm::loadHomePose(home_pose_yaml_, map_, gait_, home_pose_, body_pitch_, boot_, err)) {
      drain();
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
      return false;
    }
    rm::checkPoseReachable(map_, home_pose_, "ホーム姿勢", boot_);
    rm::checkStance(gait_, ctrl_opt_.stance_y_offset, boot_);
    rm::checkWalkEnvelope(
      map_, gait_, home_pose_, body_pitch_, ctrl_opt_.stance_y_offset, boot_);
    drain();

    // --- 層を組む ---------------------------------------------------------
    rm::BankPort bp[rm::kNumSide];
    for (int s = 0; s < rm::kNumSide; ++s) {
      bp[s].dev = port_[s];
      bp[s].ids = map_.bus(s).ids;
    }
    if (!bank_.open(bp, bank_opt_, err)) {
      drain();
      RCLCPP_ERROR(get_logger(), "%s", err.c_str());
      return false;
    }
    if (!bank_opt_.dry_run && bank_.numOpened() < rm::kNumSide) {
      ctrl_opt_.walk_enable = false;
      RCLCPP_ERROR(get_logger(), "片側のバスしか無いので歩行を無効にした (単脚では歩けない)");
    }
    codec_.configure(&map_, body_pitch_);
    ctrl_.configure(&map_, &lib_, gait_, home_pose_, body_pitch_, ctrl_opt_);
    stab_.configure(&map_, home_pose_, body_pitch_, gait_);
    drain();

    // --- 通信 -------------------------------------------------------------
    pub_state_ = create_publisher<std_msgs::msg::String>("/motion/state", latchedQos());
    pub_joints_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    pub_cmd_ = create_publisher<sensor_msgs::msg::JointState>("/motion/joint_commands", 10);
    pub_servo_ = create_publisher<sensor_msgs::msg::JointState>("/motion/servo_states", 10);
    pub_cur_ = create_publisher<sensor_msgs::msg::JointState>("/motion/servo_current", 10);
    pub_diag_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/motion/diagnostics", 10);
    sub_estop_ = create_subscription<std_msgs::msg::Bool>(
      "/estop", latchedQos(), [this](std_msgs::msg::Bool::SharedPtr m) {onEstop(*m);});
    sub_walk_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_walk", 10, [this](geometry_msgs::msg::Twist::SharedPtr m) {
        ctrl_.setWalkCmd(m->linear.x, m->linear.y, m->angular.z, nowSec());
      });
    sub_motion_ = create_subscription<std_msgs::msg::String>(
      "/cmd_motion", 10, [this](std_msgs::msg::String::SharedPtr m) {
        ctrl_.requestMotion(m->data);
      });

    // --- IMU と安定化 -------------------------------------------------------
    // RealSense は /camera/imu を SENSOR_DATA (BEST_EFFORT) で出す。合わせないと繋がらない。
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::SharedPtr m) {onImu(*m);});
    pub_stab_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/motion/stab", rclcpp::SensorDataQoS());
    {
      std_msgs::msg::MultiArrayDimension dim;
      for (std::size_t i = 0; i < kStabN; ++i) {
        dim.label += (i ? "," : "");
        dim.label += kStabFields[i];
      }
      dim.size = kStabN;
      dim.stride = kStabN;
      stab_layout_.dim.push_back(dim);
    }
    srv_zero_ = create_service<std_srvs::srv::Trigger>(
      "/motion/imu_zero",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {onImuZero(*res);});
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & ps) {return onSetParams(ps);});
    RCLCPP_INFO(get_logger(), "安定化: %s", stabSummary().c_str());

    publishState();
    RCLCPP_INFO(
      get_logger(),
      "motion 起動。%.0fHz / 読み %.0fHz / 指令途絶 %.2fs / トルクオン補間 %.1fs%s",
      loop_hz_, read_hz_, ctrl_opt_.cmd_timeout, ctrl_opt_.torque_on_time,
      bank_opt_.dry_run ? " ★dry_run: サーボへ書かない" : "");
    if (bank_opt_.allow_torque) {
      RCLCPP_WARN(
        get_logger(),
        "★このノードはサーボにトルクを入れる。機体を支えておくこと。"
        "動き出すのは /cmd_motion を受けて /estop false になってからで、"
        "実測姿勢から %.1fs かけて保持姿勢へ移る。止めるのは /estop true (L1)",
        ctrl_opt_.torque_on_time);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "allow_torque:=false — サーボにトルクを入れず、位置指令も送らない。"
        "歩行計画・IK・モーション再生・/joint_states は動く (通し確認用)");
    }
    if (ctrl_opt_.require_home_before_arm) {
      RCLCPP_INFO(
        get_logger(),
        "起動直後の /estop false では武装しない。/cmd_motion を 1 回受けるまで脱力のまま"
        " (teleop の Options 長押しで home が飛んでくる)");
    }

    running_ = true;
    bank_.start();
    control_thread_ = std::thread([this] {controlLoop();});
    return true;
  }

  void shutdown()
  {
    if (!running_.exchange(false)) {return;}
    if (control_thread_.joinable()) {control_thread_.join();}
    bank_.stop();      // 脱力を置いて閉じる
    drain();
  }

private:
  // =====================================================================
  // 制御ループ (200Hz)。シリアルには触らない
  // =====================================================================
  void controlLoop()
  {
    const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    auto next = std::chrono::steady_clock::now();
    const int js_div = std::max(1, static_cast<int>(loop_hz_ / std::max(1.0, joint_state_hz_)));
    const int stab_div = std::max(1, static_cast<int>(loop_hz_ / std::max(1.0, stab_debug_hz_)));
    start_steady_ = steadySec();
    int tick = 0;
    std::vector<ServoState> st[rm::kNumSide];

    while (running_) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
      const double now = nowSec();
      const double dt = 1.0 / loop_hz_;

      // 1) 実測を読む -> 姿勢へ直す
      for (int s = 0; s < rm::kNumSide; ++s) {bank_.states(s, st[s]);}
      const rm::PoseCodec::Decoded meas = codec_.decode(st);
      for (int s = 0; s < rm::kNumSide; ++s) {
        if (!meas.side_read[s]) {continue;}
        for (std::size_t j = 0; j < rk::kNumJoints; ++j) {theta_meas_[s][j] = meas.theta[s][j];}
      }
      if (meas.side_read[rm::kRight] || meas.side_read[rm::kLeft]) {
        arm_meas_deg_ = meas.pose.arm;
      }

      // 2) 状態機械を 1 周期
      const rm::MotionController::Tick t =
        ctrl_.step(now, dt, meas.ok ? &meas.pose : nullptr, meas.why, bank_.torqueReady());
      bank_.setWantTorque(t.want_torque);
      if (t.state_changed) {publishState();}

      // 3) IMU の安定化。補正は目標姿勢に混ぜず、サーボとの境界で掛ける
      // 歩行計画の重心加速度 ω²(x_C − p)。IMU の比力から引いて傾きを出す
      const rm::rwc::WalkOutputs * wo = ctrl_.walkOutputs();
      double ff[2]{0.0, 0.0};
      if (accel_ff_.load() && wo && wo->state != rm::rwc::State::IDLE) {
        const double w2 = gait_.omega() * gait_.omega();
        ff[0] = w2 * (wo->com[0] - wo->zmp[0]);
        ff[1] = w2 * (wo->com[1] - wo->zmp[1]);
      }
      rm::Attitude att;
      double imu_age = 1e9, imu_rx_lag = 0.0;
      {
        std::lock_guard<std::mutex> lk(imu_mtx_);
        imu_.setAccelFeedforward(ff[0], ff[1]);
        att = imu_.attitude();
        if (imu_rx_ > 0.0) {imu_age = steadySec() - imu_rx_;}
        imu_rx_lag = imu_rx_lag_;
      }
      {
        std::lock_guard<std::mutex> lk(param_mtx_);
        stab_.setGains(gains_);
      }
      rm::Stabilizer::Input sin;
      sin.relax = (t.state == rm::State::RELAX);
      sin.layer_active = (t.state == rm::State::HOLD || t.state == rm::State::WALK);
      sin.att = att.valid ? &att : nullptr;
      sin.att_age = imu_age;
      sin.walk = wo;
      const rm::PoseCorrection & corr = stab_.update(dt, sin);
      checkImuAlive(att, imu_age);

      // 4) 目標姿勢 -> 生カウント -> バス
      int dropped = 0;
      if (t.target) {
        const rm::PoseCodec::Encoded enc = codec_.encode(*t.target, &corr);
        for (int s = 0; s < rm::kNumSide; ++s) {
          dropped += enc.corr_dropped[s] ? 1 : 0;
          if (!enc.send[s]) {continue;}
          bank_.setTargets(s, enc.counts[s]);
          for (std::size_t j = 0; j < rk::kNumJoints; ++j) {theta_cmd_[s][j] = enc.theta[s][j];}
        }
        arm_cmd_deg_ = enc.arm_deg;
      }

      drain();
      if (++tick % js_div == 0) {publishTelemetry();}
      if (tick % stab_div == 0) {
        publishStab(now, att, imu_age, imu_rx_lag, sin.walk, dropped, ff);
      }

      std::this_thread::sleep_until(next);
      // 何かで大きく遅れたら次の周期に合わせ直す (取り戻そうとして暴走させない)
      const auto t2 = std::chrono::steady_clock::now();
      if (t2 > next + std::chrono::milliseconds(50)) {next = t2;}
    }
  }

  // =====================================================================
  // IMU と安定化
  // =====================================================================
  /// 安定化と IMU のパラメータ。**stab.* と imu.* の大半は実行中に ros2 param set で
  /// 変えられる**（onSetParams）。読み取り専用は imu.topic と stab.debug_hz だけ。
  void declareStabParams()
  {
    rcl_interfaces::msg::ParameterDescriptor ro;
    ro.read_only = true;
    ro.description = "IMU の生値のトピック (sensor_msgs/Imu)。起動時にだけ読む";
    imu_topic_ = declare_parameter<std::string>("imu.topic", "/camera/imu", ro);
    ro.description = "/motion/stab を出す周期 [Hz]。起動時にだけ読む";
    stab_debug_hz_ = declare_parameter<double>("stab.debug_hz", 100.0, ro);

    rcl_interfaces::msg::ParameterDescriptor en;
    en.description = "安定化を使うか。false でゲインに関わらず補正を抜く";
    gains_.enable = declare_parameter<bool>("stab.enable", gains_.enable, en);
    en.description =
      "歩行計画の重心加速度 ω²(x_C − p) を IMU の比力から引いてから傾きを出すか";
    accel_ff_ = declare_parameter<bool>("imu.accel_ff", accel_ff_, en);

    rcl_interfaces::msg::ParameterDescriptor mount;
    mount.description =
      "カメラの取り付けの傾き (roll, pitch, yaw) [deg]。pitch + でカメラが下を向く。"
      "/motion/imu_zero で立位から取り直せる";
    const std::vector<double> m = declare_parameter<std::vector<double>>(
      "imu.mount_rpy_deg",
      {imu_opt_.mount_rpy[0] * kR2D, imu_opt_.mount_rpy[1] * kR2D,
        imu_opt_.mount_rpy[2] * kR2D}, mount);
    std::string err;
    if (!mountFromDeg(m, imu_opt_, err)) {
      throw std::invalid_argument("imu.mount_rpy_deg: " + err);
    }

    for (const DoubleParam & d : kDoubleParams) {
      rcl_interfaces::msg::ParameterDescriptor desc;
      desc.description = d.desc;
      rcl_interfaces::msg::FloatingPointRange r;
      r.from_value = d.lo;
      r.to_value = d.hi;
      r.step = 0.0;
      desc.floating_point_range.push_back(r);
      const double def = d.gain ? gains_.*d.gain : imu_opt_.*d.imu;
      const double v = declare_parameter<double>(d.name, def, desc);
      if (d.gain) {
        gains_.*d.gain = v;
      } else {
        imu_opt_.*d.imu = v;
      }
    }
    imu_.configure(imu_opt_);
  }

  static bool mountFromDeg(const std::vector<double> & v, rm::ImuOptions & o, std::string & err)
  {
    if (v.size() != 3) {
      err = "要素は 3 つ (roll, pitch, yaw) [deg]";
      return false;
    }
    for (int k = 0; k < 3; ++k) {
      if (!std::isfinite(v[k]) || std::abs(v[k]) > 180.0) {
        err = "各要素は -180..180 [deg]";
        return false;
      }
      o.mount_rpy[k] = v[k] / kR2D;
    }
    return true;
  }

  rcl_interfaces::msg::SetParametersResult onSetParams(
    const std::vector<rclcpp::Parameter> & ps)
  {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;
    rm::StabGains g;
    rm::ImuOptions o;
    {
      std::lock_guard<std::mutex> lk(param_mtx_);
      g = gains_;
      o = imu_opt_;
    }
    bool touched = false;
    for (const auto & p : ps) {
      const std::string & n = p.get_name();
      if (n == "stab.enable") {
        g.enable = p.as_bool();
        touched = true;
        continue;
      }
      if (n == "imu.accel_ff") {
        accel_ff_ = p.as_bool();
        touched = true;
        continue;
      }
      if (n == "imu.mount_rpy_deg") {
        std::string err;
        if (!mountFromDeg(p.as_double_array(), o, err)) {
          res.successful = false;
          res.reason = n + ": " + err;
          return res;
        }
        touched = true;
        continue;
      }
      for (const DoubleParam & d : kDoubleParams) {
        if (n != d.name) {continue;}
        if (d.gain) {
          g.*d.gain = p.as_double();
        } else {
          o.*d.imu = p.as_double();
        }
        touched = true;
      }
    }
    if (!touched) {return res;}
    {
      std::lock_guard<std::mutex> lk(param_mtx_);
      gains_ = g;
      imu_opt_ = o;
    }
    {
      std::lock_guard<std::mutex> lk(imu_mtx_);
      imu_.configure(o);     // 取り付けが変わったときだけ推定をやり直す
    }
    RCLCPP_INFO(get_logger(), "安定化: %s", stabSummary().c_str());
    return res;
  }

  std::string stabSummary()
  {
    rm::StabGains g;
    rm::ImuOptions o;
    {
      std::lock_guard<std::mutex> lk(param_mtx_);
      g = gains_;
      o = imu_opt_;
    }
    char buf[512];
    std::snprintf(
      buf, sizeof(buf),
      "%s kd=(p %.3f, r %.3f) kp=(p %.3f, r %.3f) k_torso=%.2f 上限 足首 %.3f / 胴体 %.3f rad"
      " | IMU %s 取り付け [%.2f, %.2f, %.2f] deg tau_c %.2fs LPF %.0fHz 加速度の差し引き %s",
      g.enable ? (g.anyGain() ? "有効" : "有効 (ゲイン 0 なので補正なし)") : "無効",
      g.kd_pitch, g.kd_roll, g.kp_pitch, g.kp_roll, g.k_torso, g.ankle_clamp, g.torso_clamp,
      imu_topic_.c_str(), o.mount_rpy[0] * kR2D, o.mount_rpy[1] * kR2D, o.mount_rpy[2] * kR2D,
      o.tau_c, o.gyro_lpf_hz, accel_ff_.load() ? "あり" : "なし");
    return buf;
  }

  void onImu(const sensor_msgs::msg::Imu & m)
  {
    const double g[3] = {m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z};
    const double a[3] = {
      m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z};
    const double stamp = rclcpp::Time(m.header.stamp).seconds();
    const double rx_lag = nowSec() - stamp;
    std::lock_guard<std::mutex> lk(imu_mtx_);
    imu_.update(stamp, g, a);
    imu_rx_ = steadySec();
    imu_rx_lag_ = rx_lag;
  }

  /// 立位で呼ぶ。今の胴体をロール・ピッチ 0 にするよう取り付けの傾きを取り直す。
  void onImuZero(std_srvs::srv::Trigger::Response & res)
  {
    double mount[3]{};
    std::string msg;
    bool ok = false;
    {
      std::lock_guard<std::mutex> lk(imu_mtx_);
      ok = imu_.zero(mount, msg);
    }
    res.success = ok;
    res.message = msg;
    if (!ok) {
      RCLCPP_WARN(get_logger(), "IMU の零点: %s", msg.c_str());
      return;
    }
    {
      std::lock_guard<std::mutex> lk(param_mtx_);
      for (int k = 0; k < 3; ++k) {imu_opt_.mount_rpy[k] = mount[k];}
    }
    // パラメータにも反映する（ros2 param get で見える）。値は推定側と同じなので
    // onSetParams の configure は推定をやり直さない。
    set_parameter(
      rclcpp::Parameter(
        "imu.mount_rpy_deg",
        std::vector<double>{mount[0] * kR2D, mount[1] * kR2D, mount[2] * kR2D}));
    RCLCPP_INFO(get_logger(), "IMU の %s", msg.c_str());
    RCLCPP_INFO(
      get_logger(),
      "残すなら motion_node.yaml の imu: に書く →  mount_rpy_deg: [%.3f, %.3f, %.3f]",
      mount[0] * kR2D, mount[1] * kR2D, mount[2] * kR2D);
  }

  /// IMU が来ているかを 1 度だけ言う（control スレッド）。
  void checkImuAlive(const rm::Attitude & att, double age)
  {
    if (att.valid && age < 0.5) {
      if (!imu_seen_) {
        imu_seen_ = true;
        RCLCPP_INFO(
          get_logger(), "IMU (%s) を受け始めた。roll %.2f / pitch %.2f deg",
          imu_topic_.c_str(), att.roll * kR2D, att.pitch * kR2D);
      }
      return;
    }
    if (!imu_warned_ && steadySec() - start_steady_ > 10.0) {
      imu_warned_ = true;
      RCLCPP_WARN(
        get_logger(),
        "IMU (%s) が来ていない。安定化は効かない (補正 0 のまま動く)。"
        "roboone.launch.py の imu:=true か camera:=true で RealSense を上げること",
        imu_topic_.c_str());
    }
  }

  /// /motion/stab。並びは kStabFields。
  void publishStab(
    double now, const rm::Attitude & att, double age, double rx_lag,
    const rm::rwc::WalkOutputs * w, int dropped, const double ff[2])
  {
    const rm::Stabilizer::Debug & d = stab_.debug();
    const rm::PoseCorrection & c = stab_.correction();
    std_msgs::msg::Float64MultiArray m;
    m.layout = stab_layout_;
    m.data = {
      now, d.imu_ok ? 1.0 : 0.0, std::min(age, 1e3), rx_lag,
      att.roll, att.pitch, att.gyro[0], att.gyro[1], att.gyro[2],
      att.accel_weight, att.at_rest ? 1.0 : 0.0,
      d.active ? 1.0 : 0.0, d.fade, d.gate ? 1.0 : 0.0,
      d.weight[rm::kRight], d.weight[rm::kLeft], d.u_roll, d.u_pitch,
      c.ankle[rm::kRight][0], c.ankle[rm::kRight][1],
      c.ankle[rm::kLeft][0], c.ankle[rm::kLeft][1],
      c.body_pitch,
      w ? static_cast<double>(static_cast<int>(w->state)) : -1.0,
      w ? static_cast<double>(w->support) : 0.0,
      w ? w->phase : 0.0,
      static_cast<double>(dropped), ff[0], ff[1]};
    pub_stab_->publish(m);
  }

  void onEstop(const std_msgs::msg::Bool & m)
  {
    // teleop は脱力中 2 秒おきに /estop true を再送してくる (latched の取りこぼし保険。
    // teleop_node.py の _estop_beat)。**変化したときだけ**言う。毎回書くと、
    // コントローラが切れているあいだログが 2 秒おきに埋まって他が読めなくなる。
    const bool prev = estop_seen_.exchange(m.data);
    ctrl_.setEstop(m.data);
    if (m.data == prev && seen_estop_once_) {return;}
    seen_estop_once_ = true;
    if (m.data) {
      RCLCPP_WARN(get_logger(), "/estop true — 脱力する");
    } else {
      RCLCPP_INFO(get_logger(), "/estop false — トルクを入れてよい");
    }
  }

  // =====================================================================
  // 各層が積んだ出来事をログへ
  // =====================================================================
  void drain()
  {
    rm::Event e;
    while (boot_.pop(e)) {logEvent(e);}
    while (bank_.popEvent(e)) {logEvent(e);}
    while (codec_.popEvent(e)) {logEvent(e);}
    while (ctrl_.popEvent(e)) {logEvent(e);}
    while (stab_.popEvent(e)) {logEvent(e);}
  }

  /// throttle_ms > 0 の出来事は key ごとに間引く。
  ///
  /// RCLCPP_*_THROTTLE は呼び出し箇所ごとに状態を持つので、キューから流す形だと
  /// 全部の警告が 1 つの間引きに束ねられてしまう。**キーで間引くのは自前でやる。**
  void logEvent(const rm::Event & e)
  {
    if (e.throttle_ms > 0) {
      const std::string & k = e.key.empty() ? e.text : e.key;
      const double now = nowSec();
      auto it = throttle_.find(k);
      if (it != throttle_.end() && now - it->second < e.throttle_ms / 1000.0) {return;}
      throttle_[k] = now;
    }
    switch (e.level) {
      case rm::EventLevel::Error: RCLCPP_ERROR(get_logger(), "%s", e.text.c_str()); break;
      case rm::EventLevel::Warn: RCLCPP_WARN(get_logger(), "%s", e.text.c_str()); break;
      default: RCLCPP_INFO(get_logger(), "%s", e.text.c_str()); break;
    }
  }

  void publishState()
  {
    std_msgs::msg::String m;
    m.data = ctrl_.stateText();
    pub_state_->publish(m);
    RCLCPP_INFO(get_logger(), "/motion/state -> %s", m.data.c_str());
  }

  /// 記録用の 5 本を出す。**あとから ros2 bag で追えることが目的。**
  ///
  ///   /joint_states           実測の関節角 [rad]（標準の型なので rviz などでも読める）
  ///   /motion/joint_commands  指令の関節角 [rad]。同じ並び・同じ名前
  ///   /motion/servo_states    サーボ空間。position = 実測カウント /
  ///                           velocity = 目標カウント / effort = 負荷 (-1..1)
  ///   /motion/servo_current   電流 [mA]。position = 区間の有効サンプル数 /
  ///                           velocity = 区間平均 / effort = 区間ピーク
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
  ///   servo_current も同じ借り方をしている。**サンプル数を一緒に出すのは、ピークが
  ///   何回の読みから出た値かで信用度が変わるため。** 低電圧で応答が欠けると
  ///   サンプル数が落ちるので、そこを見ればピークの取りこぼしに気付ける。
  void publishTelemetry()
  {
    const auto stamp = get_clock()->now();
    sensor_msgs::msg::JointState meas, cmd;
    meas.header.stamp = stamp;
    cmd.header.stamp = stamp;

    for (int s = 0; s < rm::kNumSide; ++s) {
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
        const std::string n = std::string(rm::kSideTag[s]) + "_" + rm::kLegJointName[j];
        meas.name.push_back(n);
        meas.position.push_back(theta_meas_[s][j]);
        cmd.name.push_back(n);
        cmd.position.push_back(theta_cmd_[s][j]);
      }
    }
    // 腕は運動学を持たないので、T ポーズ基準の実測 deg を rad に直して入れる。
    // 脚の position が「関節角」なのに対し、腕は「サーボ角」であることに注意。
    const auto & arms = map_.arms();
    for (std::size_t a = 0; a < arms.size(); ++a) {
      meas.name.push_back(arms[a].name);
      meas.position.push_back((a < arm_meas_deg_.size() ? arm_meas_deg_[a] : 0.0) / kR2D);
      cmd.name.push_back(arms[a].name);
      cmd.position.push_back((a < arm_cmd_deg_.size() ? arm_cmd_deg_[a] : 0.0) / kR2D);
    }
    pub_joints_->publish(meas);
    pub_cmd_->publish(cmd);

    // --- サーボ空間（生カウント）と診断 --------------------------------
    sensor_msgs::msg::JointState sv, cv;
    sv.header.stamp = stamp;
    cv.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticArray da;
    da.header.stamp = stamp;

    for (int s = 0; s < rm::kNumSide; ++s) {
      std::vector<ServoState> st;
      std::vector<int16_t> goal;
      bank_.states(s, st);
      bank_.targets(s, goal);
      const rm::BankCurrent cur = bank_.takeCurrent(s);
      const auto & ids = map_.bus(s).ids;

      double volt = 0.0;
      int nvalid = 0, tmax = 0, imax = 0;
      uint8_t errbits = 0;
      for (std::size_t k = 0; k < ids.size(); ++k) {
        const ServoState e = (k < st.size()) ? st[k] : ServoState{};
        const std::string nm = std::string(rm::kSideTag[s]) + "_ID" + std::to_string(ids[k]);
        sv.name.push_back(nm);
        sv.position.push_back(e.valid ? e.pos : std::numeric_limits<double>::quiet_NaN());
        sv.velocity.push_back(k < goal.size() ? goal[k] : 0.0);
        // 負荷は 0.1% 単位で符号ビット付き。-1..1 に正規化して入れる。
        sv.effort.push_back(e.valid ? e.load / 1000.0 : 0.0);

        const int ns = (k < cur.n.size()) ? cur.n[k] : 0;
        const int pk = (k < cur.peak.size()) ? cur.peak[k] : 0;
        cv.name.push_back(nm);
        cv.position.push_back(ns);
        cv.velocity.push_back(ns ? static_cast<double>(cur.sum[k]) / ns : 0.0);
        cv.effort.push_back(pk);
        imax = std::max(imax, pk);

        if (e.valid) {
          volt += e.volt;
          ++nvalid;
          tmax = std::max(tmax, e.temp);
          errbits |= e.err;
        }
      }

      diagnostic_msgs::msg::DiagnosticStatus ds;
      ds.name = std::string("motion/bus_") + rm::kSideTag[s];
      ds.hardware_id = bank_.dev(s);
      const double v = nvalid ? volt / nvalid : 0.0;
      // 低電圧だとサーボの応答が間欠的に欠ける実機の癖があるので、そこを段の基準にする。
      ds.level = (nvalid == 0) ? diagnostic_msgs::msg::DiagnosticStatus::ERROR :
        (nvalid < static_cast<int>(ids.size()) || errbits || v < 10.5) ?
        diagnostic_msgs::msg::DiagnosticStatus::WARN :
        diagnostic_msgs::msg::DiagnosticStatus::OK;
      ds.message = std::to_string(nvalid) + "/" + std::to_string(ids.size()) + " 軸が応答";
      auto kv = [&ds](const char * k, const std::string & v2) {
          diagnostic_msgs::msg::KeyValue e;
          e.key = k;
          e.value = v2;
          ds.values.push_back(e);
        };
      kv("電圧[V]", std::to_string(v));
      kv("最高温度[C]", std::to_string(tmax));
      // このバスで一番食っている軸のピーク。過電流保護の手前にいるかの目安。
      kv("最大電流[mA]", std::to_string(imax));
      kv("応答軸数", std::to_string(nvalid));
      kv("エラービット", std::to_string(static_cast<int>(errbits)));
      kv("トルク", bank_.torqueOn(s) ? (bank_.allowTorque() ? "on" : "禁止中") : "off");
      da.status.push_back(ds);
    }
    pub_servo_->publish(sv);
    pub_cur_->publish(cv);
    pub_diag_->publish(da);
  }

  double nowSec() const {return get_clock()->now().nanoseconds() * 1e-9;}

  // --- 設定 -------------------------------------------------------------
  std::string port_[rm::kNumSide];
  std::string home_yaml_, limits_yaml_, motions_yaml_, gait_yaml_, home_pose_yaml_;
  std::vector<std::string> arm_invert_;
  double loop_hz_ = 200.0, read_hz_ = 50.0, joint_state_hz_ = 10.0;
  double body_pitch_ = 0.0;
  rm::BankOptions bank_opt_;
  rm::MotionController::Options ctrl_opt_;

  // --- 層 ---------------------------------------------------------------
  rm::ServoMap map_;
  rm::MotionLibrary lib_;
  rm::rwc::GaitParams gait_;
  rm::BodyPose home_pose_;
  rm::ServoBank bank_;
  rm::PoseCodec codec_;
  rm::MotionController ctrl_;
  rm::EventQueue boot_;          //!< 起動時の門が積む出来事

  std::thread control_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> estop_seen_{false};
  bool seen_estop_once_ = false;

  // --- IMU と安定化 --------------------------------------------------------
  std::string imu_topic_ = "/camera/imu";
  double stab_debug_hz_ = 100.0;
  std::mutex imu_mtx_;            //!< imu_ / imu_rx_ / imu_rx_lag_（IMU コールバックと control）
  rm::ImuAttitude imu_;
  double imu_rx_ = 0.0;           //!< 最後に受けた時刻 [s]（steady）
  double imu_rx_lag_ = 0.0;       //!< 受信時刻 - header.stamp [s]
  std::mutex param_mtx_;          //!< gains_ / imu_opt_（パラメータの callback と control）
  rm::StabGains gains_;
  rm::ImuOptions imu_opt_;
  rm::Stabilizer stab_;           //!< control スレッドだけが触る
  std::atomic<bool> accel_ff_{true};
  double start_steady_ = 0.0;
  bool imu_seen_ = false, imu_warned_ = false;
  std_msgs::msg::MultiArrayLayout stab_layout_;

  // --- 記録用のバッファ（control スレッドだけが触る）----------------------
  double theta_cmd_[rm::kNumSide][rk::kNumJoints]{};
  double theta_meas_[rm::kNumSide][rk::kNumJoints]{};
  std::vector<double> arm_meas_deg_, arm_cmd_deg_;
  std::unordered_map<std::string, double> throttle_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joints_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cmd_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_servo_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_cur_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diag_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_estop_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_walk_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_motion_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_stab_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_zero_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MotionNode> node;
  try {
    node = std::make_shared<MotionNode>();
  } catch (const std::exception & e) {
    // パラメータの型違い・範囲外はここに来る (例: 0.0 と書くところを 0 と書いた)
    RCLCPP_FATAL(rclcpp::get_logger("motion"), "パラメータが読めない: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
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
