// rs_stream_test — realsense2_camera が実際に出しているトピックを一定時間購読し、
// 「何Hzで、どれだけ揺れて、中身は妥当か」を1つの表にまとめる実機テスト。
//
// これは gtest ではなく素の CLI。実機が繋がっていないと意味がないテストなので、
// ament のテストには載せず、手で走らせる（feetech_servo の test/ と同じ方針）。
//
//   # 別端末で: ros2 launch realsense_bringup realsense.launch.py
//   ros2 run realsense_bringup rs_stream_test
//   ros2 run realsense_bringup rs_stream_test --duration 30 --tolerance 5
//
// 判定に通れば終了コード 0、どれか NG なら 1 を返すので、そのまま起動確認の
// ゲートとして使える。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace
{

// D435 の depth は 16bit 整数で、1 = 1mm（depth_module.depth_units の既定値）。
constexpr double kDepthUnitM = 0.001;

double steadyNowSec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 到着時刻の列から周期・ジッタを、header.stamp との差から遅延を出す。
class Rate
{
public:
  void add(double recv_s, double latency_s)
  {
    recv_.push_back(recv_s);
    if (std::isfinite(latency_s)) {
      lat_sum_ += latency_s;
      lat_max_ = std::max(lat_max_, latency_s);
      ++lat_n_;
    }
  }

  size_t count() const {return recv_.size();}

  // 実測周波数。間隔の平均ではなく「(n-1) / 全体の経過時間」で出す
  // （取りこぼしがあれば正直に下がる）。
  double hz() const
  {
    if (recv_.size() < 2) {return 0.0;}
    const double span = recv_.back() - recv_.front();
    return span > 0.0 ? static_cast<double>(recv_.size() - 1) / span : 0.0;
  }

  double jitterMs() const
  {
    const auto d = intervals();
    if (d.size() < 2) {return 0.0;}
    const double mean = std::accumulate(d.begin(), d.end(), 0.0) / d.size();
    double var = 0.0;
    for (double x : d) {var += (x - mean) * (x - mean);}
    return std::sqrt(var / d.size()) * 1e3;
  }

  // 最大間隔。ここが周期の数倍になっていたら「たまに固まる」ということ。
  double maxGapMs() const
  {
    const auto d = intervals();
    if (d.empty()) {return 0.0;}
    return *std::max_element(d.begin(), d.end()) * 1e3;
  }

  double meanLatencyMs() const {return lat_n_ ? lat_sum_ / lat_n_ * 1e3 : 0.0;}
  double maxLatencyMs() const {return lat_n_ ? lat_max_ * 1e3 : 0.0;}

private:
  std::vector<double> intervals() const
  {
    std::vector<double> d;
    for (size_t i = 1; i < recv_.size(); ++i) {d.push_back(recv_[i] - recv_[i - 1]);}
    return d;
  }

  std::vector<double> recv_;
  double lat_sum_{0.0};
  double lat_max_{-std::numeric_limits<double>::infinity()};
  size_t lat_n_{0};
};

// 1トピック分の監視結果。
struct Monitor
{
  std::string label;        // 表に出す短い名前
  std::string topic;
  double expect_hz{0.0};    // 0 なら周期の合否判定をしない
  bool required{false};     // true のトピックが出ていなければ即 NG
  bool has_publisher{false};
  Rate rate;
  std::vector<std::string> problems;          // 中身の異常（同じ文言は1回だけ）。NG になる
  std::vector<std::string> warnings;          // 知っておくべきだが実害はないもの。合否には効かない
  std::function<std::string()> detail;        // 表の下に出す中身のサマリ

  void note(const std::string & msg)
  {
    if (std::find(problems.begin(), problems.end(), msg) == problems.end()) {
      problems.push_back(msg);
    }
  }

  void warn(const std::string & msg)
  {
    if (std::find(warnings.begin(), warnings.end(), msg) == warnings.end()) {
      warnings.push_back(msg);
    }
  }
};

// publisher の QoS に合わせて購読する。
// realsense は画像を RELIABLE、IMU を SENSOR_DATA(BEST_EFFORT) で出すので、
// 決め打ちにすると IMU だけ繋がらない、という事故が起きる。
rclcpp::QoS matchPublisherQos(rclcpp::Node & node, const std::string & topic, size_t depth)
{
  rclcpp::QoS qos(depth);
  const auto infos = node.get_publishers_info_by_topic(topic);
  if (!infos.empty() &&
    infos.front().qos_profile().reliability() == rclcpp::ReliabilityPolicy::BestEffort)
  {
    qos.best_effort();
  }
  return qos;
}

// ---------------------------------------------------------------------------
// 中身の検査
// ---------------------------------------------------------------------------

struct DepthAcc
{
  double valid_ratio_sum{0.0};   // 距離が取れている画素の割合
  double center_sum{0.0};        // 画像中央の距離[m]
  size_t center_n{0};
  size_t n{0};
  uint32_t width{0}, height{0};
  std::string encoding;
};

void checkDepth(const sensor_msgs::msg::Image & m, Monitor & mon, DepthAcc & acc)
{
  acc.encoding = m.encoding;
  acc.width = m.width;
  acc.height = m.height;

  if (m.encoding != "16UC1") {
    mon.note("encoding が 16UC1 ではない: " + m.encoding);
    return;
  }
  if (m.step != m.width * 2 || m.data.size() != static_cast<size_t>(m.step) * m.height) {
    mon.note("data のサイズが width/height/step と合わない");
    return;
  }
  if (m.header.frame_id.empty()) {mon.note("frame_id が空");}

  const auto * px = reinterpret_cast<const uint16_t *>(m.data.data());

  // 全画素舐めると 30Hz では重いので、16画素おきのサンプリングで有効率を見る。
  // （厳密な値が要るときは rs_depth_test を使う。あちらは全画素見る）
  // 0 = 測距できなかった画素、0xFFFF = 遠すぎて飽和した画素。どちらも距離として使えない。
  size_t seen = 0, valid = 0;
  for (size_t i = 0; i < static_cast<size_t>(m.width) * m.height; i += 16, ++seen) {
    const uint16_t v = px[i];
    if (v != 0 && v != 0xFFFF) {++valid;}
  }
  if (seen) {acc.valid_ratio_sum += static_cast<double>(valid) / seen;}

  // 中央 32x32 の平均距離。壁やロボットに向けた時に妥当な値が出るかの目安。
  const int cx = static_cast<int>(m.width) / 2, cy = static_cast<int>(m.height) / 2;
  double sum = 0.0;
  size_t cnt = 0;
  for (int y = cy - 16; y < cy + 16; ++y) {
    for (int x = cx - 16; x < cx + 16; ++x) {
      if (y < 0 || x < 0 || y >= static_cast<int>(m.height) || x >= static_cast<int>(m.width)) {
        continue;
      }
      const uint16_t v = px[static_cast<size_t>(y) * m.width + x];
      if (v != 0 && v != 0xFFFF) {sum += v * kDepthUnitM; ++cnt;}
    }
  }
  if (cnt) {acc.center_sum += sum / cnt; ++acc.center_n;}
  ++acc.n;
}

struct ImuAcc
{
  double accel_norm_sum{0.0};
  double gyro_norm_sum{0.0};
  size_t n{0};
  std::string frame_id;
};

void checkImu(const sensor_msgs::msg::Imu & m, Monitor & mon, ImuAcc & acc)
{
  acc.frame_id = m.header.frame_id;
  if (m.header.frame_id.empty()) {mon.note("frame_id が空");}

  const double ax = m.linear_acceleration.x, ay = m.linear_acceleration.y,
    az = m.linear_acceleration.z;
  const double gx = m.angular_velocity.x, gy = m.angular_velocity.y, gz = m.angular_velocity.z;

  if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(az) ||
    !std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz))
  {
    mon.note("NaN/Inf が入っている");
    return;
  }

  const double an = std::sqrt(ax * ax + ay * ay + az * az);
  // 静止していれば重力ぶんの 9.8 前後。動いていても 5〜20 を大きく外れることはない。
  // これを外れたら accel が来ていない（0埋め）か、単位がおかしい。
  if (an < 1.0) {mon.note("加速度がほぼ 0（accel ストリームが乗っていない疑い）");}
  if (an > 30.0) {mon.note("加速度が大きすぎる（>30 m/s^2）");}

  // orientation は realsense からは出ない（imu_filter が埋める）。
  // 「無い」ことが covariance[0] = -1 で示されているのが正しい状態。
  if (m.orientation_covariance[0] >= 0.0) {
    mon.note("orientation_covariance[0] が -1 でない（姿勢なしの表明が壊れている）");
  }
  if (m.angular_velocity_covariance[0] <= 0.0) {
    mon.note("angular_velocity_covariance が 0 以下（imu_filter が使えない）");
  }

  acc.accel_norm_sum += an;
  acc.gyro_norm_sum += std::sqrt(gx * gx + gy * gy + gz * gz);
  ++acc.n;
}

struct CloudAcc
{
  double points_sum{0.0};
  size_t points_min{std::numeric_limits<size_t>::max()};
  size_t points_max{0};
  size_t n{0};
  std::string fields;
  uint32_t point_step{0}, row_step{0};
};

void checkCloud(const sensor_msgs::msg::PointCloud2 & m, Monitor & mon, CloudAcc & acc)
{
  std::string names;
  bool has_x = false, has_y = false, has_z = false;
  for (const auto & f : m.fields) {
    names += (names.empty() ? "" : ",") + f.name;
    if (f.name == "x") {has_x = true;}
    if (f.name == "y") {has_y = true;}
    if (f.name == "z") {has_z = true;}
  }
  acc.fields = names;
  acc.point_step = m.point_step;
  acc.row_step = m.row_step;

  if (!has_x || !has_y || !has_z) {mon.note("x/y/z フィールドが揃っていない: " + names);}
  if (m.header.frame_id.empty()) {mon.note("frame_id が空");}
  // 実際に何点入っているかは width*height*point_step で決まる。ここが合わなければ致命的。
  if (m.data.size() != static_cast<size_t>(m.width) * m.height * m.point_step) {
    mon.note("data のサイズが width*height*point_step と合わない");
  }
  // 一方 row_step は realsense-ros 側が更新し忘れる（無効画素を捨てて width を
  // 縮めた後も、縮める前の値が残る）。PointCloud2Iterator も pcl_conversions も
  // width/height/point_step しか見ないので実害はないが、row_step を信じて
  // ポインタを進める自作コードを書くとバッファ外を読むので、警告として出す。
  if (m.data.size() != static_cast<size_t>(m.row_step) * m.height) {
    mon.warn("row_step が data サイズと不整合（realsense-ros 側の既知の癖。"
      "点数は width*height から取ること）");
  }

  const size_t pts = static_cast<size_t>(m.width) * m.height;
  if (pts == 0) {mon.note("点数 0 のフレームがある（何も映っていない/距離が近すぎる）");}

  acc.points_sum += static_cast<double>(pts);
  acc.points_min = std::min(acc.points_min, pts);
  acc.points_max = std::max(acc.points_max, pts);
  ++acc.n;
}

struct ColorAcc
{
  size_t n{0};
  uint32_t width{0}, height{0};
  std::string encoding;
};

void checkColor(const sensor_msgs::msg::Image & m, Monitor & mon, ColorAcc & acc)
{
  acc.encoding = m.encoding;
  acc.width = m.width;
  acc.height = m.height;
  if (m.encoding != "rgb8") {mon.note("encoding が rgb8 ではない: " + m.encoding);}
  if (m.data.size() != static_cast<size_t>(m.step) * m.height) {
    mon.note("data のサイズが step*height と合わない");
  }
  ++acc.n;
}

// ---------------------------------------------------------------------------

void usage(const char * argv0)
{
  std::printf(
    "使い方: %s [オプション]\n"
    "  --duration SEC    計測時間（既定 10）\n"
    "  --wait SEC        トピックが現れるまでの待ち時間（既定 15）\n"
    "  --tolerance PCT   期待周波数からのずれの許容％（既定 10）\n"
    "  --ns PREFIX       トピックの接頭辞（既定 /camera）\n"
    "  --depth-hz HZ     depth の期待周波数（既定 30）\n"
    "  --imu-hz HZ       imu の期待周波数（既定 200）\n"
    "  --points-hz HZ    点群の期待周波数（既定 30）\n"
    "  --color-hz HZ     color の期待周波数（既定 30）\n"
    "  -h, --help        このヘルプ\n"
    "\n"
    "depth と imu は必須。points と color は出ていれば測り、無ければスキップする。\n", argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  double duration = 10.0, wait = 15.0, tolerance = 10.0;
  double depth_hz = 30.0, imu_hz = 200.0, points_hz = 30.0, color_hz = 30.0;
  std::string ns = "/camera";

  // rclcpp に渡す前に自前の引数を抜き取る（--ros-args 以降は rclcpp に任せる）
  std::vector<char *> ros_args{argv[0]};
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {return (i + 1 < argc) ? argv[++i] : std::string();};
    if (a == "--duration") {
      duration = std::stod(next());
    } else if (a == "--wait") {
      wait = std::stod(next());
    } else if (a == "--tolerance") {
      tolerance = std::stod(next());
    } else if (a == "--ns") {
      ns = next();
    } else if (a == "--depth-hz") {
      depth_hz = std::stod(next());
    } else if (a == "--imu-hz") {
      imu_hz = std::stod(next());
    } else if (a == "--points-hz") {
      points_hz = std::stod(next());
    } else if (a == "--color-hz") {
      color_hz = std::stod(next());
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      ros_args.push_back(argv[i]);
    }
  }

  rclcpp::init(static_cast<int>(ros_args.size()), ros_args.data());
  auto node = rclcpp::Node::make_shared("rs_stream_test");

  // {表示名, トピック, 期待周波数, 必須か}。残りのメンバは既定値のまま。
  // depth と imu はこのパッケージの目的そのものなので必須。points と color は
  // launch の設定次第で存在しないので、出ていれば測る・無ければスキップ。
  Monitor depth{"depth", ns + "/depth/image_rect_raw", depth_hz, true};
  Monitor imu{"imu", ns + "/imu", imu_hz, true};
  Monitor points{"points", ns + "/depth/color/points", points_hz, false};
  Monitor color{"color", ns + "/color/image_raw", color_hz, false};
  std::vector<Monitor *> mons{&depth, &imu, &points, &color};

  // --- publisher が現れるのを待つ -------------------------------------------
  // required なものが全部揃った時点で先へ進む。optional は「その時点で出ていれば測る」。
  std::printf("トピックを待っています（最大 %.0f 秒）...\n", wait);
  const double t_wait_end = steadyNowSec() + wait;
  while (rclcpp::ok()) {
    bool required_ready = true;
    for (auto * m : mons) {
      m->has_publisher = node->count_publishers(m->topic) > 0;
      if (m->required && !m->has_publisher) {required_ready = false;}
    }
    if (required_ready) {break;}
    if (steadyNowSec() > t_wait_end) {break;}
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  for (auto * m : mons) {
    std::printf("  %-8s %-34s %s\n", m->label.c_str(), m->topic.c_str(),
      m->has_publisher ? "見つかった" : (m->required ? "見つからない (NG)" : "無効 (スキップ)"));
  }
  std::printf("\n%.0f 秒間 計測します...\n", duration);

  // --- 購読 -----------------------------------------------------------------
  DepthAcc depth_acc;
  ImuAcc imu_acc;
  CloudAcc cloud_acc;
  ColorAcc color_acc;

  auto latency = [&node](const std_msgs::msg::Header & h) {
      if (h.stamp.sec == 0 && h.stamp.nanosec == 0) {
        return std::numeric_limits<double>::quiet_NaN();
      }
      return (node->now() - rclcpp::Time(h.stamp)).seconds();
    };

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;

  if (depth.has_publisher) {
    subs.push_back(node->create_subscription<sensor_msgs::msg::Image>(
        depth.topic, matchPublisherQos(*node, depth.topic, 10),
        [&](const sensor_msgs::msg::Image::SharedPtr m) {
          depth.rate.add(steadyNowSec(), latency(m->header));
          checkDepth(*m, depth, depth_acc);
        }));
    depth.detail = [&depth_acc]() {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%ux%u %s / 測距できた画素 %.1f%% / 中央32x32の距離 %.3f m",
          depth_acc.width, depth_acc.height, depth_acc.encoding.c_str(),
          depth_acc.n ? depth_acc.valid_ratio_sum / depth_acc.n * 100.0 : 0.0,
          depth_acc.center_n ? depth_acc.center_sum / depth_acc.center_n : 0.0);
        return std::string(buf);
      };
  }

  if (imu.has_publisher) {
    subs.push_back(node->create_subscription<sensor_msgs::msg::Imu>(
        imu.topic, matchPublisherQos(*node, imu.topic, 100),
        [&](const sensor_msgs::msg::Imu::SharedPtr m) {
          imu.rate.add(steadyNowSec(), latency(m->header));
          checkImu(*m, imu, imu_acc);
        }));
    imu.detail = [&imu_acc]() {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "frame=%s / |加速度| 平均 %.2f m/s^2 / |角速度| 平均 %.4f rad/s",
          imu_acc.frame_id.c_str(),
          imu_acc.n ? imu_acc.accel_norm_sum / imu_acc.n : 0.0,
          imu_acc.n ? imu_acc.gyro_norm_sum / imu_acc.n : 0.0);
        return std::string(buf);
      };
  }

  if (points.has_publisher) {
    subs.push_back(node->create_subscription<sensor_msgs::msg::PointCloud2>(
        points.topic, matchPublisherQos(*node, points.topic, 5),
        [&](const sensor_msgs::msg::PointCloud2::SharedPtr m) {
          points.rate.add(steadyNowSec(), latency(m->header));
          checkCloud(*m, points, cloud_acc);
        }));
    points.detail = [&cloud_acc]() {
        char buf[256];
        std::snprintf(buf, sizeof(buf),
          "フィールド[%s] point_step=%u / 点数 平均 %.0f (最小 %zu, 最大 %zu)",
          cloud_acc.fields.c_str(), cloud_acc.point_step,
          cloud_acc.n ? cloud_acc.points_sum / cloud_acc.n : 0.0,
          cloud_acc.n ? cloud_acc.points_min : size_t{0}, cloud_acc.points_max);
        return std::string(buf);
      };
  }

  if (color.has_publisher) {
    subs.push_back(node->create_subscription<sensor_msgs::msg::Image>(
        color.topic, matchPublisherQos(*node, color.topic, 5),
        [&](const sensor_msgs::msg::Image::SharedPtr m) {
          color.rate.add(steadyNowSec(), latency(m->header));
          checkColor(*m, color, color_acc);
        }));
    color.detail = [&color_acc]() {
        char buf[256];
        std::snprintf(buf, sizeof(buf), "%ux%u %s",
          color_acc.width, color_acc.height, color_acc.encoding.c_str());
        return std::string(buf);
      };
  }

  const double t_end = steadyNowSec() + duration;
  while (rclcpp::ok() && steadyNowSec() < t_end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::microseconds(200));
  }

  // --- 結果 -----------------------------------------------------------------
  std::printf(
    "\n%-8s %-32s %7s %8s %8s %8s %9s %8s  %s\n",
    "stream", "topic", "count", "Hz", "expect", "jitter", "max gap", "delay", "判定");
  std::printf("%s\n", std::string(110, '-').c_str());

  bool all_ok = true;
  for (auto * m : mons) {
    std::string verdict;
    if (!m->has_publisher) {
      verdict = m->required ? "NG 未publish" : "-- 無効";
      if (m->required) {all_ok = false;}
    } else if (m->rate.count() < 2) {
      verdict = "NG 受信なし";
      all_ok = false;
    } else if (!m->problems.empty()) {
      verdict = "NG 中身";
      all_ok = false;
    } else if (m->expect_hz > 0.0 &&
      std::fabs(m->rate.hz() - m->expect_hz) / m->expect_hz * 100.0 > tolerance)
    {
      verdict = "NG 周期";
      all_ok = false;
    } else {
      verdict = "OK";
    }

    std::printf(
      "%-8s %-32s %7zu %8.2f %8.1f %7.2fms %7.1fms %6.1fms  %s\n",
      m->label.c_str(), m->topic.c_str(), m->rate.count(), m->rate.hz(), m->expect_hz,
      m->rate.jitterMs(), m->rate.maxGapMs(), m->rate.meanLatencyMs(), verdict.c_str());
  }

  std::printf("\n中身:\n");
  for (auto * m : mons) {
    if (!m->has_publisher || m->rate.count() == 0) {continue;}
    std::printf("  %-8s %s\n", m->label.c_str(), m->detail ? m->detail().c_str() : "");
    for (const auto & p : m->problems) {std::printf("           ! %s\n", p.c_str());}
    for (const auto & w : m->warnings) {std::printf("           ? %s\n", w.c_str());}
  }

  std::printf(
    "\n  ! = NG の原因 / ? = 知っておくべきだが合否には効かないもの\n");
  std::printf(
    "\n凡例: jitter=受信間隔の標準偏差 / max gap=最大の受信間隔（周期の数倍なら取りこぼし）\n"
    "      delay=header.stamp から受信までの平均（値が桁違いならタイムスタンプ源がずれている）\n");
  std::printf("\n総合: %s\n", all_ok ? "OK" : "NG");
  rclcpp::shutdown();
  return all_ok ? 0 : 1;
}
