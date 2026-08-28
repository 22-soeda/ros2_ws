// motion_teach — 脱力させた機体を手で構えて、その姿勢をモーション config の
// キーフレームとして捕まえるツール。
//
//   ros2 run roboone_motion_node motion_teach
//   ros2 run roboone_motion_node motion_teach --out /tmp/punch_r.yaml
//
// ===========================================================================
// このツールがサーボに書くのは起動時のトルク OFF ただ 1 回だけ
// ===========================================================================
// 手で関節を動かせるようにするためで、これ以外の書き込み経路を持たない。
//   * 位置指令を書かない（sync_write_position / write_position を呼ばない）
//   * トルクを入れない（enable_torque(id, true) を呼ばない）
//   * EEPROM を触らない / init_motor() も呼ばない
// --keep-torque を付けるとトルク OFF も省く。どちらでも読むだけ。
//
// ===========================================================================
// 使い方
// ===========================================================================
// 1. 機体を安全な高さで支える（脱力するので、そのままだと崩れる）
// 2. 起動する。全軸のトルクが切れて、画面に今の姿勢が出続ける
// 3. 取りたい姿勢に手で構える
// 4. **スペース**を押す。その瞬間の値が YAML のキーフレームとして出る
// 5. 出た行を config/motions.yaml に貼り、``t:`` を狙いの時間に直す
//
//   キー   スペース / c  今の姿勢を捕まえる
//          n             新しい技の見出し（motions: の下に貼る枠）を出す
//          + / -         次に出す t: の値を 0.05s ずつ増減する
//          q / Ctrl-C    終了
//
// 画面は標準エラーへ、捕まえた YAML は標準出力へ出す。だから
//
//   ros2 run roboone_motion_node motion_teach > punch_r.yaml
//
// とすると、画面を見ながら捕まえたぶんだけが綺麗にファイルへ溜まる
// （--out を使えばリダイレクトなしで同じことができる）。
//
// ===========================================================================
// 捕まえた姿勢は「IK で戻せるか」まで確かめる
// ===========================================================================
// 手で作った姿勢は、実測サーボ角 -> 順変換 で足裏の (p, R) にしている。この (p, R) を
// config に書くと、再生時は逆に IK を通ることになる。**順変換で出せても IK で
// 戻せるとは限らない**（可動域の縁、足首の特異点の近く）ので、捕まえるたびに
// (p, R) -> IK -> FK の往復誤差を出す。ここが大きい姿勢を config に入れると、
// 実機では「そこだけ動かない」という形で出て、原因が非常に追いにくい。
#include <yaml-cpp/yaml.h>

#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "feetech_servo/feetech_bus.hpp"
#include "roboone_motion_node/body_pose.hpp"
#include "roboone_motion_node/servo_map.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;
namespace rmn = roboone_motion_node;
namespace rk = roboone_kinematics;

namespace
{

constexpr double kR2D = 180.0 / M_PI;

std::atomic<bool> g_stop{false};
termios g_tty_saved{};
bool g_tty_raw = false;

void restoreTty()
{
  if (g_tty_raw) {
    tcsetattr(STDIN_FILENO, TCSANOW, &g_tty_saved);
    g_tty_raw = false;
  }
}

void onSigint(int) {g_stop = true;}

/// 1 キーずつ拾えるようにする。端末でなければ何もしない（パイプ実行を許す）。
void rawTty()
{
  if (!isatty(STDIN_FILENO)) {return;}
  if (tcgetattr(STDIN_FILENO, &g_tty_saved) != 0) {return;}
  termios raw = g_tty_saved;
  raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 0;
  if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
    g_tty_raw = true;
    std::atexit(restoreTty);
  }
}

/// 押されていれば 1 文字、無ければ 0。
char pollKey()
{
  if (!g_tty_raw) {return 0;}
  char c = 0;
  return read(STDIN_FILENO, &c, 1) == 1 ? c : 0;
}

void usage()
{
  std::fprintf(
    stderr,
    "使い方: motion_teach [オプション]\n"
    "  --right PORT     右半身のポート（既定 /dev/feetech_right）\n"
    "  --left PORT      左半身のポート（既定 /dev/feetech_left）\n"
    "  --home PATH      servo_home.yaml（既定 share/feetech_servo/config/）\n"
    "  --limits PATH    servo_limits.yaml（既定 同上。表示にのみ使う）\n"
    "  --out PATH       捕まえた YAML をこのファイルにも追記する\n"
    "  --hz N           画面の更新周期（既定 20）\n"
    "  --t SEC          最初に出す t: の値（既定 0.30）\n"
    "  --keep-torque    起動時のトルク OFF も行わない（読むだけなのは変わらない）\n"
    "  --baud N         ボーレート（既定 1000000）\n"
    "\n"
    "サーボへの書き込みは起動時のトルク OFF 1 回だけ。\n");
}

/// 捕まえた 1 姿勢を motions.yaml のキーフレームとして書き出す。
std::string keyframeYaml(const rmn::ServoMap & map, const rmn::BodyPose & p, double t)
{
  char buf[512];
  std::string out;
  std::snprintf(buf, sizeof(buf), "      - t: %.2f\n", t);
  out += buf;
  const char * key[rmn::kNumSide] = {"R_foot", "L_foot"};
  for (int s = 0; s < rmn::kNumSide; ++s) {
    std::snprintf(
      buf, sizeof(buf),
      "        %s: {p: [%8.3f, %8.3f, %9.3f], rpy: [%7.2f, %7.2f, %7.2f]}\n",
      key[s], p.foot[s].p.x, p.foot[s].p.y, p.foot[s].p.z,
      p.foot[s].rpy[0] * kR2D, p.foot[s].rpy[1] * kR2D, p.foot[s].rpy[2] * kR2D);
    out += buf;
  }
  out += "        arms:   {";
  const auto & arms = map.arms();
  for (std::size_t a = 0; a < arms.size(); ++a) {
    std::snprintf(
      buf, sizeof(buf), "%s%s: %7.2f", a ? ", " : "", arms[a].name.c_str(),
      a < p.arm.size() ? p.arm[a] : 0.0);
    out += buf;
  }
  out += "}\n";
  return out;
}

/// 1 本のバス。開けなかった側は「無い」ものとして片側だけで続ける。
struct Half
{
  std::unique_ptr<FeetechBus> bus;
  std::vector<ServoState> st;
  bool ok = false;                 //!< 直近の読みで脚 6 軸が全部揃ったか
  rk::LegServoStatus status = rk::LegServoStatus::Ok;
  double th6_seed = 0.0;
  double theta[rk::kNumJoints]{};
  double servo_tpose_deg[rk::kNumJoints]{};
};

}  // namespace

int main(int argc, char ** argv)
{
  std::string right = "/dev/feetech_right", left = "/dev/feetech_left";
  std::string home_path, limits_path, out_path;
  double hz = 20.0, t_next = 0.30;
  int baud = 1000000;
  bool torque_off = true;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char * what) -> std::string {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "%s の値が無い\n", what);
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "--right") {right = next("--right");} else if (a == "--left") {
      left = next("--left");
    } else if (a == "--home") {home_path = next("--home");} else if (a == "--limits") {
      limits_path = next("--limits");
    } else if (a == "--out") {out_path = next("--out");} else if (a == "--hz") {
      hz = std::atof(next("--hz").c_str());
    } else if (a == "--t") {t_next = std::atof(next("--t").c_str());} else if (a == "--baud") {
      baud = std::atoi(next("--baud").c_str());
    } else if (a == "--keep-torque") {torque_off = false;} else if (a == "-h" || a == "--help") {
      usage();
      return 0;
    } else {
      std::fprintf(stderr, "知らないオプション: %s\n", a.c_str());
      usage();
      return 2;
    }
  }

  std::string share;
  try {
    share = ament_index_cpp::get_package_share_directory("feetech_servo");
  } catch (const std::exception & e) {
    std::fprintf(stderr, "feetech_servo の share が見つからない: %s\n", e.what());
    return 2;
  }
  if (home_path.empty()) {home_path = share + "/config/servo_home.yaml";}
  if (limits_path.empty()) {limits_path = share + "/config/servo_limits.yaml";}

  rmn::ServoMap map;
  std::string err;
  // 腕の回転方向はティーチ側でも同じにしないと、捕まえた値を config へ貼ったときに
  // 符号が合わない。既定は motion_node.yaml の arm_invert と同じ。
  const std::vector<std::string> arm_invert = {"R8", "L9", "R10"};
  if (!map.load(home_path, limits_path, right, left, arm_invert, err)) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 2;
  }
  std::fprintf(stderr, "%s\n", map.summary().c_str());

  std::signal(SIGINT, onSigint);
  std::signal(SIGTERM, onSigint);

  Half half[rmn::kNumSide];
  const std::string port[rmn::kNumSide] = {right, left};
  int opened = 0;
  for (int s = 0; s < rmn::kNumSide; ++s) {
    half[s].bus = std::make_unique<FeetechBus>(port[s], baud, 0, 20, feetech_servo::Family::kHls);
    if (!half[s].bus->open()) {
      std::fprintf(
        stderr, "%s が開けない（udev 固定名と電源を確認）。この側は飛ばす。\n",
        port[s].c_str());
      half[s].bus.reset();
      continue;
    }
    ++opened;
    // ここが唯一の書き込み。手で構えられるように脱力させる。
    if (torque_off) {
      int n = 0;
      for (uint8_t id : map.bus(s).ids) {n += half[s].bus->enable_torque(id, false) ? 1 : 0;}
      std::fprintf(
        stderr, "%s: %d/%zu 軸をトルク OFF（脱力）。手で動かせる。\n",
        rmn::kSideTag[s], n, map.bus(s).ids.size());
    }
  }
  if (opened == 0) {
    std::fprintf(stderr, "どちらのバスも開けなかった。\n");
    return 1;
  }

  std::FILE * out_file = nullptr;
  if (!out_path.empty()) {
    out_file = std::fopen(out_path.c_str(), "a");
    if (!out_file) {
      std::fprintf(stderr, "--out %s を開けない\n", out_path.c_str());
      return 2;
    }
    std::fprintf(stderr, "捕まえた YAML は %s にも追記する\n", out_path.c_str());
  }

  rawTty();
  if (!g_tty_raw) {
    std::fprintf(stderr, "★端末ではないのでキー入力は使えない（表示だけ）。\n");
  }

  const auto period = std::chrono::duration<double>(1.0 / (hz < 1.0 ? 1.0 : hz));
  bool paused = false;
  int captured = 0;

  while (!g_stop) {
    const auto tick = std::chrono::steady_clock::now();

    // --- 読み取り ------------------------------------------------------
    rmn::BodyPose pose;
    pose.arm.assign(map.num_arm(), 0.0);
    for (int s = 0; s < rmn::kNumSide; ++s) {
      if (!half[s].bus) {continue;}
      half[s].st.clear();
      half[s].bus->sync_read_states(map.bus(s).ids, half[s].st);

      double servo[rk::kNumJoints];
      half[s].ok = true;
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
        if (j >= half[s].st.size() || !half[s].st[j].valid) {half[s].ok = false; break;}
        servo[j] = map.leg_servo_from_count(s, j, half[s].st[j].pos);
        half[s].servo_tpose_deg[j] = map.leg_tpose_deg_from_count(s, j, half[s].st[j].pos);
      }
      if (half[s].ok) {
        half[s].status = rmn::footPoseFromServo(
          map.leg_params(s), servo, pose.foot[s], half[s].theta, half[s].th6_seed);
      }
      const auto & arms = map.arms();
      std::size_t k = rk::kNumJoints;
      for (std::size_t a = 0; a < arms.size(); ++a) {
        if (arms[a].side != s) {continue;}
        if (k < half[s].st.size() && half[s].st[k].valid) {
          pose.arm[a] = map.arm_deg_from_count(a, half[s].st[k].pos);
        }
        ++k;
      }
    }

    // --- キー ----------------------------------------------------------
    const char key = pollKey();
    if (key == 'q' || key == 3) {break;}
    if (paused) {
      if (key != 0) {paused = false;}
    } else if (key == ' ' || key == 'c') {
      // IK で戻せるかを確かめてから出す（docstring 参照）
      std::string note;
      for (int s = 0; s < rmn::kNumSide; ++s) {
        if (!half[s].bus) {continue;}
        if (!half[s].ok) {
          note += std::string(rmn::kSideTag[s]) + "脚の実測が欠けている ";
          continue;
        }
        double sv[rk::kNumJoints], th[rk::kNumJoints];
        const rmn::LegSolve r =
          rmn::servoFromFootPose(map.leg_params(s), pose.foot[s], sv, th);
        if (!r.ok()) {
          char b[128];
          std::snprintf(
            b, sizeof(b), "%s脚が IK で戻せない(ik=%d servo=%d) ", rmn::kSideTag[s],
            static_cast<int>(r.ik_status), static_cast<int>(r.servo_status));
          note += b;
          continue;
        }
        rk::Vec3 p2;
        rk::Mat3 R2;
        rk::fk(map.leg_params(s).leg, th, p2, R2);
        const double e = std::sqrt(
          (p2.x - pose.foot[s].p.x) * (p2.x - pose.foot[s].p.x) +
          (p2.y - pose.foot[s].p.y) * (p2.y - pose.foot[s].p.y) +
          (p2.z - pose.foot[s].p.z) * (p2.z - pose.foot[s].p.z));
        if (e > 1.0) {
          char b[128];
          std::snprintf(b, sizeof(b), "%s脚の往復誤差 %.2fmm ", rmn::kSideTag[s], e);
          note += b;
        }
        if (r.ankle_clamped) {note += std::string(rmn::kSideTag[s]) + "脚の足首が可動域の外 ";}
      }

      const std::string block = keyframeYaml(map, pose, t_next);
      std::fputs(block.c_str(), stdout);
      std::fflush(stdout);
      if (out_file) {
        std::fputs(block.c_str(), out_file);
        std::fflush(out_file);
      }
      ++captured;
      paused = true;
      std::fprintf(stderr, "\033[H\033[J");
      std::fprintf(stderr, "--- %d 枚目を捕まえた (t: %.2f) ---\n\n%s\n", captured, t_next,
        block.c_str());
      if (!note.empty()) {
        std::fprintf(stderr, "★注意: %s\n  この姿勢は再生時に狙いどおりにならない。\n\n",
          note.c_str());
      }
      std::fprintf(stderr, "任意のキーで表示に戻る（q で終了）\n");
      std::fflush(stderr);
    } else if (key == '+' || key == '=') {
      t_next += 0.05;
    } else if (key == '-') {
      t_next = t_next > 0.05 ? t_next - 0.05 : 0.05;
    } else if (key == 'n') {
      std::fputs("\n  <技名>:\n    return_home: true\n    keyframes:\n", stdout);
      std::fflush(stdout);
      if (out_file) {
        std::fputs("\n  <技名>:\n    return_home: true\n    keyframes:\n", out_file);
        std::fflush(out_file);
      }
    }

    // --- 表示 ----------------------------------------------------------
    if (!paused) {
      std::fprintf(stderr, "\033[H\033[J");
      std::fprintf(
        stderr,
        "motion_teach — 脱力中（書き込みは起動時のトルク OFF だけ）\n"
        "  スペース/c 捕まえる   +/- 次の t: を増減   n 技の見出し   q 終了\n"
        "  次に出す t: %.2f s        捕まえた枚数: %d\n\n", t_next, captured);

      static const char * kName[rk::kNumJoints] = {
        "股ピッチ", "股ロール", "股ヨー  ", "膝      ", "足首鎖0 ", "足首鎖1 "};
      for (int s = 0; s < rmn::kNumSide; ++s) {
        if (!half[s].bus) {continue;}
        std::fprintf(
          stderr, "[%s脚] %s\n", rmn::kSideTag[s],
          half[s].ok ? (half[s].status == rk::LegServoStatus::Ok ? "" : "★変換に失敗")
          : "★応答が欠けている");
        std::fprintf(stderr, "   関節      ID   サーボ[deg]  関節角[deg]\n");
        for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
          std::fprintf(
            stderr, "   %s  %-3d  %+10.2f   %+10.2f\n", kName[j], rmn::kLegServoId[j],
            half[s].servo_tpose_deg[j], half[s].theta[j] * kR2D);
        }
        std::fprintf(
          stderr,
          "   足裏 p = [%8.2f, %8.2f, %9.2f] mm   rpy = [%6.2f, %6.2f, %6.2f] deg\n\n",
          pose.foot[s].p.x, pose.foot[s].p.y, pose.foot[s].p.z,
          pose.foot[s].rpy[0] * kR2D, pose.foot[s].rpy[1] * kR2D, pose.foot[s].rpy[2] * kR2D);
      }

      std::fprintf(stderr, "[腕] T ポーズ基準 [deg]\n  ");
      const auto & arms = map.arms();
      for (std::size_t a = 0; a < arms.size(); ++a) {
        std::fprintf(stderr, "%s %+7.2f   ", arms[a].name.c_str(), pose.arm[a]);
      }
      std::fprintf(stderr, "\n");

      double volt = 0.0;
      int nv = 0;
      for (int s = 0; s < rmn::kNumSide; ++s) {
        for (const auto & st : half[s].st) {
          if (st.valid) {volt += st.volt; ++nv;}
        }
      }
      // 低電圧だとサーボの応答が間欠的に欠ける実機の癖があるので、常に出しておく。
      std::fprintf(stderr, "\n電圧 %.2f V (%d 軸から)\n", nv ? volt / nv : 0.0, nv);
      std::fflush(stderr);
    }

    std::this_thread::sleep_until(
      tick + std::chrono::duration_cast<std::chrono::steady_clock::duration>(period));
  }

  restoreTty();
  if (out_file) {std::fclose(out_file);}
  std::fprintf(stderr, "\n終了。%d 枚捕まえた（サーボへの書き込みはトルク OFF だけ）\n", captured);
  return 0;
}
