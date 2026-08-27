// rs_depth_test — depth 画像を数フレームためて、距離が本当に取れているかを見る。
//
// rs_stream_test は「30Hz で届いているか」までしか見ない。空の（全画素 0 の）
// 画像でも 30Hz は出るので、depth が使い物になるかは別に確かめる必要がある。
// このツールは全画素を走査して有効画素率と距離の分布を出し、さらに画面を
// 16x9 のセルに割った距離マップを表示する。壁やロボットに向けて走らせて、
// 数字が見た目と合っていれば depth は正常。
//
//   # 別端末で: ros2 launch realsense_bringup realsense.launch.py
//   ros2 run realsense_bringup rs_depth_test
//   ros2 run realsense_bringup rs_depth_test --frames 60 --min-valid 40
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{

// 距離マップのセル数。端末幅に収まる範囲で、画角の見当がつく粗さ。
constexpr int kGridCols = 16;
constexpr int kGridRows = 9;

// depth 画素は3種類に分けて数える。ひとまとめに「有効/無効」で見ると、
// カメラが壊れているのか、ただ何も無い方を向いているだけなのかが区別できない。
//   0                 : ステレオマッチングが失敗した画素（テクスチャ無し・遮蔽・近すぎ）
//   max_range より遠い : 視差がほぼ 0 の画素。D435 の測距レンジ(〜10m)の外なので
//                        数十mという値が出るが、これは距離ではなく「遠い」という意味しかない
//                        （0xFFFF = 65.535m は 16bit の上限に張り付いた極端な例）
//   それ以外           : 実際に測れた距離
constexpr uint16_t kDepthInvalid = 0;

double steadyNowSec()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void usage(const char * argv0)
{
  std::printf(
    "使い方: %s [オプション]\n"
    "  --frames N        解析するフレーム数（既定 30）\n"
    "  --duration SEC    それまでに集まらなければ打ち切る秒数（既定 10）\n"
    "  --wait SEC        トピックが現れるまでの待ち時間（既定 15）\n"
    "  --topic NAME      購読するトピック（既定 /camera/depth/image_rect_raw）\n"
    "  --scale M         1 カウントあたりの距離[m]（既定 0.001 = 1mm）\n"
    "  --min-valid PCT   測距できた画素率の合格ライン％（既定 20）\n"
    "  --max-range M     これより遠い画素は「レンジ外」として別に数える（既定 10）\n"
    "  -h, --help        このヘルプ\n", argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  int want_frames = 30;
  double duration = 10.0, wait = 15.0, scale = 0.001, min_valid = 20.0, max_range = 10.0;
  std::string topic = "/camera/depth/image_rect_raw";

  std::vector<char *> ros_args{argv[0]};
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&]() -> std::string {return (i + 1 < argc) ? argv[++i] : std::string();};
    if (a == "--frames") {
      want_frames = std::stoi(next());
    } else if (a == "--duration") {
      duration = std::stod(next());
    } else if (a == "--wait") {
      wait = std::stod(next());
    } else if (a == "--topic") {
      topic = next();
    } else if (a == "--scale") {
      scale = std::stod(next());
    } else if (a == "--min-valid") {
      min_valid = std::stod(next());
    } else if (a == "--max-range") {
      max_range = std::stod(next());
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      // 知らない引数は rclcpp に渡す（--ros-args 以降など）
      ros_args.push_back(argv[i]);
    }
  }

  rclcpp::init(static_cast<int>(ros_args.size()), ros_args.data());
  auto node = rclcpp::Node::make_shared("rs_depth_test");

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

  int frames = 0;
  uint32_t width = 0, height = 0;
  std::string encoding, frame_id;
  bool bad_layout = false;

  double valid_ratio_sum = 0.0;       // 測距レンジ内で距離が取れた画素の割合
  double invalid_ratio_sum = 0.0;     // 0 の画素の割合
  double far_ratio_sum = 0.0;         // 測距レンジ外（遠すぎ）の画素の割合
  double depth_sum = 0.0;             // 距離が取れた画素の総和（全フレーム通算）
  size_t depth_n = 0;
  double dmin = std::numeric_limits<double>::infinity();
  double dmax = 0.0;
  std::vector<double> center_per_frame;             // 中央8x8の平均距離（時間ノイズ用）
  std::vector<double> grid_sum(kGridCols * kGridRows, 0.0);   // セルごとの距離の総和
  std::vector<size_t> grid_n(kGridCols * kGridRows, 0);
  std::vector<size_t> grid_far(kGridCols * kGridRows, 0);     // セルごとのレンジ外画素数

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    topic, rclcpp::QoS(5),
    [&](const sensor_msgs::msg::Image::SharedPtr m) {
      if (frames >= want_frames) {return;}
      width = m->width;
      height = m->height;
      encoding = m->encoding;
      frame_id = m->header.frame_id;

      if (m->encoding != "16UC1" || m->step != m->width * 2 ||
        m->data.size() != static_cast<size_t>(m->step) * m->height)
      {
        bad_layout = true;
        ++frames;
        return;
      }

      const auto * px = reinterpret_cast<const uint16_t *>(m->data.data());
      size_t valid = 0, invalid = 0, far = 0;
      double center_sum = 0.0;
      size_t center_n = 0;
      const int cx = static_cast<int>(m->width) / 2, cy = static_cast<int>(m->height) / 2;

      for (uint32_t y = 0; y < m->height; ++y) {
        const int gr = static_cast<int>(y * kGridRows / m->height);
        for (uint32_t x = 0; x < m->width; ++x) {
          const uint16_t raw = px[static_cast<size_t>(y) * m->width + x];
          const int gc = static_cast<int>(x * kGridCols / m->width);
          if (raw == kDepthInvalid) {++invalid; continue;}
          const double d = raw * scale;
          if (d > max_range) {
            ++far;
            ++grid_far[gr * kGridCols + gc];
            continue;
          }
          ++valid;
          depth_sum += d;
          ++depth_n;
          dmin = std::min(dmin, d);
          dmax = std::max(dmax, d);

          grid_sum[gr * kGridCols + gc] += d;
          ++grid_n[gr * kGridCols + gc];

          if (std::abs(static_cast<int>(y) - cy) < 4 && std::abs(static_cast<int>(x) - cx) < 4) {
            center_sum += d;
            ++center_n;
          }
        }
      }

      const size_t total = static_cast<size_t>(m->width) * m->height;
      if (total) {
        valid_ratio_sum += static_cast<double>(valid) / total;
        invalid_ratio_sum += static_cast<double>(invalid) / total;
        far_ratio_sum += static_cast<double>(far) / total;
      }
      if (center_n) {center_per_frame.push_back(center_sum / center_n);}
      ++frames;
    });

  std::printf("%d フレーム集めます（最大 %.0f 秒）...\n", want_frames, duration);
  const double t_end = steadyNowSec() + duration;
  while (rclcpp::ok() && frames < want_frames && steadyNowSec() < t_end) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::microseconds(500));
  }

  if (frames == 0) {
    std::printf("NG: 1 フレームも受信できませんでした。\n");
    rclcpp::shutdown();
    return 1;
  }

  const double valid_ratio = valid_ratio_sum / frames * 100.0;
  const double invalid_ratio = invalid_ratio_sum / frames * 100.0;
  const double far_ratio = far_ratio_sum / frames * 100.0;
  const double dmean = depth_n ? depth_sum / depth_n : 0.0;

  std::printf("\n--- 画像 ---------------------------------------------------\n");
  std::printf("  frame_id     : %s\n", frame_id.c_str());
  std::printf("  解像度/形式  : %ux%u %s\n", width, height, encoding.c_str());
  std::printf("  フレーム数   : %d\n", frames);
  std::printf("  1カウント    : %.4f m\n", scale);

  std::printf("\n--- 画素の内訳 ---------------------------------------------\n");
  std::printf("  測距できた   : %.1f %%  (合格ライン %.0f %%)\n", valid_ratio, min_valid);
  std::printf("  測距失敗(0)  : %.1f %%  ← テクスチャ無し・遮蔽・近すぎ\n", invalid_ratio);
  std::printf("  レンジ外     : %.1f %%  ← %.0f m より遠い（視差がほぼ 0 の画素）\n",
    far_ratio, max_range);
  std::printf("\n--- 距離（測距できた画素のみ） -----------------------------\n");
  std::printf("  最小/平均/最大 : %.3f / %.3f / %.3f m\n",
    std::isfinite(dmin) ? dmin : 0.0, dmean, dmax);

  if (center_per_frame.size() >= 2) {
    double m = 0.0;
    for (double x : center_per_frame) {m += x;}
    m /= center_per_frame.size();
    double s = 0.0;
    for (double x : center_per_frame) {s += (x - m) * (x - m);}
    s = std::sqrt(s / center_per_frame.size());
    // 同じ場所を見続けたときの値のばらつき = 時間方向のノイズ。
    // 距離が遠いほど大きくなる（D435 はおおむね距離の2乗に比例）。
    std::printf("  中央8x8 の距離 : %.3f m  (フレーム間の標準偏差 %.4f m)\n", m, s);
  }

  std::printf(
    "\n--- 距離マップ [m] (%dx%d セルの平均。 far=レンジ外 / ---- =測距できず) ---\n",
    kGridCols, kGridRows);
  for (int r = 0; r < kGridRows; ++r) {
    std::printf("  ");
    for (int c = 0; c < kGridCols; ++c) {
      const size_t n = grid_n[r * kGridCols + c];
      if (n > 0) {
        std::printf("%5.2f ", grid_sum[r * kGridCols + c] / n);
      } else if (grid_far[r * kGridCols + c] > 0) {
        std::printf("  far ");
      } else {
        std::printf(" ---- ");
      }
    }
    std::printf("\n");
  }

  std::printf("\n--- 判定 ---------------------------------------------------\n");
  bool ok = true;
  auto judge = [&ok](bool cond, const char * name, const std::string & detail) {
      std::printf("  [%s] %-22s %s\n", cond ? "OK" : "NG", name, detail.c_str());
      if (!cond) {ok = false;}
    };
  char buf[160];

  std::snprintf(buf, sizeof(buf), "%s", encoding.c_str());
  judge(!bad_layout, "画像レイアウト", bad_layout ? "encoding/step/data が不整合" : buf);

  std::snprintf(buf, sizeof(buf), "%.1f %% (>= %.0f %%)", valid_ratio, min_valid);
  judge(valid_ratio >= min_valid, "測距できた画素率", buf);

  // D435 の測距レンジはおおむね 0.1〜10m。飽和画素は別枠で数えているので、
  // ここを外れた値が出るなら scale が違うか、depth ではないものを見ている。
  std::snprintf(buf, sizeof(buf), "%.3f 〜 %.3f m", std::isfinite(dmin) ? dmin : 0.0, dmax);
  judge(depth_n > 0 && std::isfinite(dmin) && dmin > 0.05, "距離の範囲", buf);

  if (far_ratio > 20.0) {
    std::printf(
      "  [警告] レンジ外の画素が %.1f %% あります。カメラの前が開けすぎている"
      "（%.0f m 以上先しかない）状態です。\n", far_ratio, max_range);
  }

  std::printf("\n総合: %s\n", ok ? "OK" : "NG");
  std::printf(
    "\nヒント: 測距できた画素率が低いときは、被写体が近すぎる（0.2m 未満）か、\n"
    "        真っ白な壁でテクスチャが無いか、屋外で赤外が飽和している。\n"
    "        リングに見立てて 0.5〜3m 先に物がある向きで測るのが本来の使い方。\n");
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
