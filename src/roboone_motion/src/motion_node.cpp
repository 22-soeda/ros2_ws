// motion ノード — ros-architecture §3 の「200Hz ループ」の実体。
//
//   受け取る: /cmd_walk (geometry_msgs/Twist)  歩行指令。teleop / behavior から 20Hz
//             /cmd_motion (std_msgs/String)    技名。イベント時
//             /estop (std_msgs/Bool)           脱力 / トルクオン。latched
//   出す:     /motion/state (std_msgs/String)  状態。変化時
//             /joint_states 他 (10Hz)          記録用の 5 本 (publishTelemetry を見よ)
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
//
// 1 周期はこれだけ:
//
//     bank_.states()  ->  codec_.decode()  ->  ctrl_.step()  ->  codec_.encode()
//                                                            ->  bank_.setTargets()
//
// ===========================================================================
// スレッドの分け方
// ===========================================================================
//   main        rclcpp::spin。購読コールバックだけ（値を controller へ置く）
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
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "roboone_motion/motion_config.hpp"
#include "roboone_motion/motion_control.hpp"
#include "roboone_motion/pose_codec.hpp"
#include "roboone_motion/servo_bank.hpp"

using feetech_servo::ServoState;
namespace rm = roboone_motion;
namespace rk = roboone_kinematics;

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
    bank_opt_.goal_torque = declare_parameter<int>("goal_torque", 1000);
    // ★0 にしないこと。位置指令パケットの速度 (reg46/47) に 0 を書くと、この実機
    //   (HLS 系) は目標位置を受け取っても動かない。同じパケットの 44/45 = GOAL_TORQUE
    //   に 0 を書くと全軸まったく動かないのと同じ性質で、0 は「無制限」ではない。
    bank_opt_.move_speed = declare_parameter<int>("move_speed", 2000);
    bank_opt_.move_acc = declare_parameter<int>("move_acc", 50);

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

      // 3) 目標姿勢 -> 生カウント -> バス
      if (t.target) {
        const rm::PoseCodec::Encoded enc = codec_.encode(*t.target);
        for (int s = 0; s < rm::kNumSide; ++s) {
          if (!enc.send[s]) {continue;}
          bank_.setTargets(s, enc.counts[s]);
          for (std::size_t j = 0; j < rk::kNumJoints; ++j) {theta_cmd_[s][j] = enc.theta[s][j];}
        }
        arm_cmd_deg_ = enc.arm_deg;
      }

      drain();
      if (++tick % js_div == 0) {publishTelemetry();}

      std::this_thread::sleep_until(next);
      // 何かで大きく遅れたら次の周期に合わせ直す (取り戻そうとして暴走させない)
      const auto t2 = std::chrono::steady_clock::now();
      if (t2 > next + std::chrono::milliseconds(50)) {next = t2;}
    }
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
