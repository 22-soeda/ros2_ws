// rs_imu_test — /camera/imu を一定時間ためて、IMU が「使える状態か」を判定する。
//
// rs_stream_test が周期の健全性を見るのに対し、こちらは中身（重力の向き、
// ジャイロのバイアス、ノイズの大きさ）を見る。imu_filter_madgwick に渡す前の
// 素の値がまともかどうか、と、共分散パラメータを決めるための実測値を出す。
//
//   # 別端末で: ros2 launch realsense_bringup realsense.launch.py
//   # 機体を静止させた状態で
//   ros2 run realsense_bringup rs_imu_test
//   ros2 run realsense_bringup rs_imu_test --duration 30
//
// 静止していることが前提。動かしながら走らせると「静止していない」と警告を出す。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace
{

// D435if の IMU 光学フレーム (camera_imu_optical_frame) の軸:
//   X = 右, Y = 下, Z = 前
// 加速度計は「重力への反力」を測るので、カメラを正立・水平に置くと
// 上向き(= -Y)に約 +9.8 が出る → (0, -9.8, 0)。
constexpr double kGravity = 9.80665;
const char * const kAxisName[3] = {"X(右)", "Y(下)", "Z(前)"};

// 判定のしきい値。実測に基づいて緩めてよい。
constexpr double kAccelNormTolerance = 0.8;   // |a| が 9.8±0.8 から外れたら NG
constexpr double kGyroBiasLimit = 0.05;       // 静止時のジャイロ平均 [rad/s]
constexpr double kStillAccelStd = 0.35;       // これを超えたら「静止していない」
constexpr double kStillGyroStd = 0.05;

// printf 書式で std::string を作る小道具
template<typename ... Args>
std::string fmt(const char * f, Args ... args)
{
  char buf[192];
  std::snprintf(buf, sizeof(buf), f, args ...);
  return std::string(buf);
}

double steadyNowSec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Series
{
  std::vector<double> v;
  double mean() const
  {
    if (v.empty()) {return 0.0;}
    double s = 0.0;
    for (double x : v) {s += x;}
    return s / v.size();
  }
  double stddev() const
  {
    if (v.size() < 2) {return 0.0;}
    const double m = mean();
    double s = 0.0;
    for (double x : v) {s += (x - m) * (x - m);}
    return std::sqrt(s / v.size());
  }
  double min() const {return v.empty() ? 0.0 : *std::min_element(v.begin(), v.end());}
  double max() const {return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());}
};

void usage(const char * argv0)
{
  std::printf(
    "使い方: %s [オプション]   ※機体を静止させて走らせること\n"
    "  --duration SEC   計測時間（既定 10）\n"
    "  --wait SEC       トピックが現れるまでの待ち時間（既定 15）\n"
    "  --topic NAME     購読するトピック（既定 /camera/imu）\n"
    "  --expect-hz HZ   期待周波数（既定 200。0 で周期判定をしない）\n"
    "  -h, --help       このヘルプ\n", argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  double duration = 10.0, wait = 15.0, expect_hz = 200.0;
  std::string topic = "/camera/imu";

  std::vector<char *> ros_args{argv[0]};
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {return (i + 1 < argc) ? argv[++i] : std::string();};
    if (a == "--duration") {
      duration = std::stod(next());
    } else if (a == "--wait") {
      wait = std::stod(next());
    } else if (a == "--topic") {
      topic = next();
    } else if (a == "--expect-hz") {
      expect_hz = std::stod(next());
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      // 知らない引数は rclcpp に渡す（--ros-args 以降など）
      ros_args.push_back(argv[i]);
    }
  }

  rclcpp::init(static_cast<int>(ros_args.size()), ros_args.data());
  auto node = rclcpp::Node::make_shared("rs_imu_test");

  std::printf("%s を待っています（最大 %.0f 秒）...\n", topic.c_str(), wait);
  const double t_wait_end = steadyNowSec() + wait;
  while (rclcpp::ok() && node->count_publishers(topic) == 0) {
    if (steadyNowSec() > t_wait_end) {
      std::printf("NG: %s の publisher が見つかりません。launch は起動していますか？\n",
        topic.c_str());
      rclcpp::shutdown();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  Series ax, ay, az, gx, gy, gz, anorm;
  std::vector<double> recv;          // 受信時刻（周期の実測用）
  std::vector<double> stamp;         // header.stamp（時刻の単調性チェック用）
  size_t nonfinite = 0, backwards = 0;
  std::string frame_id;

  // IMU は SENSOR_DATA (BEST_EFFORT) で publish されるので、購読側も合わせる。
  // RELIABLE で購読すると QoS が非互換になり、1つも受け取れない。
  auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    topic, rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::Imu::SharedPtr m) {
      frame_id = m->header.frame_id;
      const double t = rclcpp::Time(m->header.stamp).seconds();
      if (!stamp.empty() && t < stamp.back()) {++backwards;}
      stamp.push_back(t);
      recv.push_back(steadyNowSec());

      const double a[3] = {m->linear_acceleration.x, m->linear_acceleration.y,
        m->linear_acceleration.z};
      const double g[3] = {m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z};
      for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(a[i]) || !std::isfinite(g[i])) {++nonfinite; return;}
      }
      ax.v.push_back(a[0]); ay.v.push_back(a[1]); az.v.push_back(a[2]);
      gx.v.push_back(g[0]); gy.v.push_back(g[1]); gz.v.push_back(g[2]);
      anorm.v.push_back(std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]));
    });

  std::printf("%.0f 秒間 計測します（動かさないでください）...\n", duration);
  const double t_end = steadyNowSec() + duration;
  while (rclcpp::ok() && steadyNowSec() < t_end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }

  if (ax.v.size() < 2) {
    std::printf("NG: サンプルが %zu 個しか取れませんでした。\n", ax.v.size());
    rclcpp::shutdown();
    return 1;
  }

  const double hz = (recv.size() - 1) / (recv.back() - recv.front());
  const double amean[3] = {ax.mean(), ay.mean(), az.mean()};
  const double gmean[3] = {gx.mean(), gy.mean(), gz.mean()};
  const double astd[3] = {ax.stddev(), ay.stddev(), az.stddev()};
  const double gstd[3] = {gx.stddev(), gy.stddev(), gz.stddev()};

  // 重力がどの軸に乗っているか（= カメラの取り付け姿勢の目安）
  int grav_axis = 0;
  for (int i = 1; i < 3; ++i) {
    if (std::fabs(amean[i]) > std::fabs(amean[grav_axis])) {grav_axis = i;}
  }

  std::printf("\n--- 受信 ---------------------------------------------------\n");
  std::printf("  frame_id      : %s\n", frame_id.c_str());
  std::printf("  サンプル数    : %zu\n", ax.v.size());
  std::printf("  実測周波数    : %.2f Hz (期待 %.0f Hz)\n", hz, expect_hz);
  std::printf("  時刻の逆行    : %zu 回\n", backwards);
  std::printf("  NaN/Inf       : %zu 個\n", nonfinite);

  std::printf("\n--- 加速度 [m/s^2] -----------------------------------------\n");
  for (int i = 0; i < 3; ++i) {
    std::printf("  %-7s 平均 %+8.4f   標準偏差 %.4f\n", kAxisName[i], amean[i], astd[i]);
  }
  std::printf("  |a| 平均 %.4f  (重力 %.4f との差 %+.4f)\n",
    anorm.mean(), kGravity, anorm.mean() - kGravity);
  std::printf("  重力が乗っている軸: %s (%+.2f)\n", kAxisName[grav_axis], amean[grav_axis]);
  std::printf("    ※ 正立・水平なら Y(下) に約 -9.8。別の軸に出ているなら傾いて置かれている\n");

  std::printf("\n--- 角速度 [rad/s] -----------------------------------------\n");
  for (int i = 0; i < 3; ++i) {
    std::printf("  %-7s 平均(バイアス) %+8.5f   標準偏差 %.5f\n",
      kAxisName[i], gmean[i], gstd[i]);
  }

  std::printf("\n--- 共分散パラメータの参考値 -------------------------------\n");
  // imu_filter に渡す共分散は「分散 = 標準偏差^2」。実測から出したものを
  // config/realsense.yaml の linear_accel_cov / angular_velocity_cov に入れられる。
  const double acc_var = std::max({astd[0], astd[1], astd[2]});
  const double gyr_var = std::max({gstd[0], gstd[1], gstd[2]});
  std::printf("  linear_accel_cov    : %.6f  (実測の最大標準偏差 %.4f の2乗)\n",
    acc_var * acc_var, acc_var);
  std::printf("  angular_velocity_cov: %.6f  (実測の最大標準偏差 %.5f の2乗)\n",
    gyr_var * gyr_var, gyr_var);

  // --- 判定 ------------------------------------------------------------------
  std::printf("\n--- 判定 ---------------------------------------------------\n");
  bool ok = true;
  auto judge = [&ok](bool cond, const char * name, const std::string & detail) {
      std::printf("  [%s] %-24s %s\n", cond ? "OK" : "NG", name, detail.c_str());
      if (!cond) {ok = false;}
    };
  judge(nonfinite == 0, "NaN/Inf なし", fmt("%zu 個", nonfinite));
  judge(backwards == 0, "タイムスタンプ単調", fmt("逆行 %zu 回", backwards));
  if (expect_hz > 0.0) {
    judge(std::fabs(hz - expect_hz) / expect_hz < 0.1, "周波数 (±10%)",
      fmt("%.2f Hz / 期待 %.0f Hz", hz, expect_hz));
  }
  judge(std::fabs(anorm.mean() - kGravity) < kAccelNormTolerance, "重力の大きさ",
    fmt("|a| = %.3f m/s^2 (許容 %.3f ± %.1f)", anorm.mean(), kGravity, kAccelNormTolerance));
  const double gbias = std::max({std::fabs(gmean[0]), std::fabs(gmean[1]), std::fabs(gmean[2])});
  judge(gbias < kGyroBiasLimit, "ジャイロのバイアス",
    fmt("最大 %.5f rad/s (許容 < %.2f)", gbias, kGyroBiasLimit));

  // 静止判定は合否に含めない（動かしながら測っても値そのものは出したいので警告扱い）
  const double amax_std = std::max({astd[0], astd[1], astd[2]});
  const double gmax_std = std::max({gstd[0], gstd[1], gstd[2]});
  if (amax_std > kStillAccelStd || gmax_std > kStillGyroStd) {
    std::printf(
      "  [警告] 計測中に機体が動いていた可能性があります"
      " (加速度 std %.3f, 角速度 std %.4f)。\n"
      "         バイアスと共分散の値は静止状態で測り直してください。\n", amax_std, gmax_std);
  }

  std::printf("\n総合: %s\n", ok ? "OK" : "NG");
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
