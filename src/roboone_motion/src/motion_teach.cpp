// motion_teach — 脱力させた機体を手で構えて、その姿勢をモーション config の
// キーフレームとして捕まえるツール。
//
//   ros2 run roboone_motion motion_teach
//   ros2 run roboone_motion motion_teach --out /tmp/punch_r.yaml
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
//          f             出す書式を切り替える（ik -> angle -> both）
//          q / Ctrl-C    終了
//
// ===========================================================================
// 2 つの書式 — 足裏 (IK) とサーボ角
// ===========================================================================
//   ik     R_foot / L_foot。足裏の (p, R)。再生時に IK を通る（既定）
//   angle  R_leg / L_leg。T ポーズ基準のサーボ角 [deg]。再生時は IK を通らない
//   both   両方出す。ik の行が生きていて、angle の行は # でコメントアウトして出す
//
// 手で構えた姿勢は**順変換では必ず出せるが、IK で戻せるとは限らない**（可動域の縁、
// 足首の特異点の近く、寝ている姿勢）。ik で捕まえて往復誤差が出るような姿勢は、
// angle で捕まえればそのまま再生できる。逆に angle は機体を組み替えると付いて
// こないので、届く姿勢は ik のまま置いておくほうがよい。both はその判断を後回しに
// する書式で、貼ってから # を付け替えれば切り替えられる。
//
// 画面は標準エラーへ、捕まえた YAML は標準出力へ出す。だから
//
//   ros2 run roboone_motion motion_teach > punch_r.yaml
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

#include "roboone_motion/pose_codec.hpp"
#include "roboone_motion/servo_bank.hpp"
#include "roboone_motion/body_pose.hpp"
#include "roboone_motion/servo_map.hpp"

using feetech_servo::ServoState;
namespace rm = roboone_motion;
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
    "  --limits PATH    servo_limits.yaml（既定 同上。窓の外を捕まえたら言う）\n"
    "  --out PATH       捕まえた YAML をこのファイルにも追記する\n"
    "  --hz N           画面の更新周期（既定 20）\n"
    "  --t SEC          最初に出す t: の値（既定 0.30）\n"
    "  --format FMT     出す書式 ik / angle / both（既定 ik。f キーでも切り替わる）\n"
    "  --keep-torque    起動時のトルク OFF も行わない（読むだけなのは変わらない）\n"
    "  --baud N         ボーレート（既定 1000000）\n"
    "\n"
    "サーボへの書き込みは起動時のトルク OFF 1 回だけ。\n");
}

/// 出す書式。冒頭の「2 つの書式」を参照。
/// ServoBank が積んだ出来事を画面（stderr）へ流す。
///
/// 捕まえた YAML は stdout へ出すので、ログは必ず stderr。混ぜると
/// `motion_teach > punch_r.yaml` が汚れる。
void drainEvents(rm::ServoBank & bank)
{
  rm::Event e;
  while (bank.popEvent(e)) {
    std::fprintf(
      stderr, "%s%s\n",
      e.level == rm::EventLevel::Error ? "エラー: " :
      (e.level == rm::EventLevel::Warn ? "警告: " : ""), e.text.c_str());
  }
}

enum class Format { kIk, kAngle, kBoth };

const char * formatName(Format f)
{
  switch (f) {
    case Format::kAngle: return "angle (R_leg / L_leg・サーボ角)";
    case Format::kBoth: return "both (ik の行 + # を付けた angle の行)";
    default: return "ik (R_foot / L_foot・足裏)";
  }
}

/// 片側の直近の読み。**バスは持たない**（ServoBank が 2 本まとめて面倒を見る）。
///
/// 姿勢そのものは PoseCodec が組み立てる。ここに残すのは、ティーチの画面と
/// 角度書きの行に要る「サーボ角そのもの」だけ。
struct Half
{
  bool ok = false;                 //!< 直近の読みで脚 6 軸が全部揃ったか
  rk::LegServoStatus status = rk::LegServoStatus::Ok;
  double theta[rk::kNumJoints]{};
  double servo[rk::kNumJoints]{};           //!< 絶対サーボ角 [rad]
  double servo_tpose_deg[rk::kNumJoints]{}; //!< 同じものを T ポーズ基準 [deg] で
};

/// 足裏書きの 1 行（``R_foot: {p: [...], rpy: [...]}``）。
std::string footLine(const char * indent, const char * key, const rm::FootPose & f)
{
  char buf[256];
  std::snprintf(
    buf, sizeof(buf),
    "%s%s: {p: [%8.3f, %8.3f, %9.3f], rpy: [%7.2f, %7.2f, %7.2f]}\n",
    indent, key, f.p.x, f.p.y, f.p.z, f.rpy[0] * kR2D, f.rpy[1] * kR2D, f.rpy[2] * kR2D);
  return buf;
}

/// 角度書きの 1 行（``R_leg: {ID1: ..., ...}``）。値は T ポーズ基準 [deg]。
///
/// 並びは Joint enum のまま（足首が ID6 -> ID5 の順）。読み込み側は ID で引くので
/// 順番に意味は無いが、画面の並びと揃えておくと突き合わせやすい。
std::string legLine(const char * indent, const char * key, const Half & h)
{
  char buf[512];
  std::string out = std::string(indent) + key + ": {";
  for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
    std::snprintf(
      buf, sizeof(buf), "%sID%d: %8.2f", j ? ", " : "", rm::kLegServoId[j],
      h.servo_tpose_deg[j]);
    out += buf;
  }
  return out + "}\n";
}

/// 捕まえた 1 姿勢を motions.yaml のキーフレームとして書き出す。
std::string keyframeYaml(
  const rm::ServoMap & map, const rm::BodyPose & p, const Half half[rm::kNumSide],
  double t, Format fmt)
{
  char buf[512];
  std::string out;
  std::snprintf(buf, sizeof(buf), "      - t: %.2f\n", t);
  out += buf;
  const char * fkey[rm::kNumSide] = {"R_foot", "L_foot"};
  const char * lkey[rm::kNumSide] = {"R_leg", "L_leg"};
  for (int s = 0; s < rm::kNumSide; ++s) {
    if (fmt != Format::kAngle) {out += footLine("        ", fkey[s], p.foot[s]);}
    if (fmt == Format::kAngle) {out += legLine("        ", lkey[s], half[s]);}
    // both では角度側を # で殺して出す。貼ってから # を付け替えれば切り替わる
    // （両方生かすと「足裏で決めてから軸を上書き」になって別の意味になる）。
    if (fmt == Format::kBoth) {out += legLine("#       ", lkey[s], half[s]);}
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

}  // namespace

int main(int argc, char ** argv)
{
  std::string right = "/dev/feetech_right", left = "/dev/feetech_left";
  std::string home_path, limits_path, out_path;
  double hz = 20.0, t_next = 0.30;
  int baud = 1000000;
  bool torque_off = true;
  Format fmt = Format::kIk;

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
    } else if (a == "--format") {
      const std::string v = next("--format");
      if (v == "ik") {
        fmt = Format::kIk;
      } else if (v == "angle") {
        fmt = Format::kAngle;
      } else if (v == "both") {
        fmt = Format::kBoth;
      } else {
        std::fprintf(stderr, "--format は ik / angle / both のどれか（%s）\n", v.c_str());
        return 2;
      }
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

  rm::ServoMap map;
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

  Half half[rm::kNumSide];

  // サーボ層。**トルクは絶対に入れない** (allow_torque = false)。読むだけの道具なので、
  // 位置指令も送らない。唯一の書き込みは下の relax() = 起動時のトルク OFF 1 回だけ。
  rm::ServoBank bank;
  rm::BankPort bp[rm::kNumSide];
  const std::string port[rm::kNumSide] = {right, left};
  for (int s = 0; s < rm::kNumSide; ++s) {
    bp[s].dev = port[s];
    bp[s].ids = map.bus(s).ids;
  }
  rm::BankOptions bopt;
  bopt.baud = baud;
  bopt.loop_hz = (hz < 1.0 ? 1.0 : hz);
  bopt.read_hz = bopt.loop_hz;          // 書かないので、周期 = 読みの周期
  bopt.allow_torque = false;
  if (!bank.open(bp, bopt, err)) {
    drainEvents(bank);
    std::fprintf(stderr, "%s\n", err.c_str());
    return 1;
  }
  if (torque_off) {bank.relax();}       // 手で構えられるように脱力させる
  drainEvents(bank);
  bank.start();

  // 姿勢の組み立て。**前傾は掛けない** (body_pitch = 0)。捕まえた姿勢は Σ_B
  // (実機そのもの) で出す約束なので、ここで Σ_U へ戻すと二重に傾く。
  rm::PoseCodec codec;
  codec.configure(&map, 0.0);
  std::vector<feetech_servo::ServoState> st[rm::kNumSide];

  std::FILE * out_file = nullptr;
  if (!out_path.empty()) {
    out_file = std::fopen(out_path.c_str(), "a");
    if (!out_file) {
      std::fprintf(stderr, "--out %s を開けない\n", out_path.c_str());
      return 2;
    }
    std::fprintf(stderr, "捕まえた YAML は %s にも追記する\n", out_path.c_str());
  }

  std::fprintf(stderr, "出す書式: %s\n", formatName(fmt));

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
    // 生カウント -> 姿勢 は PoseCodec の仕事。ここに残すのは、画面と角度書きの行に
    // 要る「サーボ角そのもの」を写すところだけ。
    for (int s = 0; s < rm::kNumSide; ++s) {
      bank.states(s, st[s]);
    }
    const rm::PoseCodec::Decoded d = codec.decode(st);
    const rm::BodyPose & pose = d.pose;
    for (int s = 0; s < rm::kNumSide; ++s) {
      half[s].ok = d.side_read[s];
      half[s].status = d.status[s];
      if (!half[s].ok) {continue;}
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
        half[s].theta[j] = d.theta[s][j];
        half[s].servo[j] = map.leg_servo_from_count(s, j, st[s][j].pos);
        half[s].servo_tpose_deg[j] = map.leg_tpose_deg_from_count(s, j, st[s][j].pos);
      }
    }
    drainEvents(bank);

    // --- キー ----------------------------------------------------------
    const char key = pollKey();
    if (key == 'q' || key == 3) {break;}
    if (paused) {
      if (key != 0) {paused = false;}
    } else if (key == ' ' || key == 'c') {
      // 出す書式ごとに「その行が再生時に狙いどおりになるか」を確かめてから出す。
      //   ik    IK で戻せるか（往復誤差）。docstring の「IK で戻せるか」の節
      //   angle servo_limits.yaml の窓に収まっているか（角度書きの唯一の歯止め）
      std::string note;
      for (int s = 0; s < rm::kNumSide; ++s) {
        if (!bank.has(s)) {continue;}
        if (!half[s].ok) {
          note += std::string(rm::kSideTag[s]) + "脚の実測が欠けている ";
          continue;
        }
        if (fmt != Format::kIk) {
          // 角度書きは IK も FK も通らずにサーボへ出るので、窓の外を書くと
          // 再生時に黙って丸められる。捕まえた時点で言う。
          for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
            bool clamped = false;
            map.leg_count_from_servo(s, j, half[s].servo[j], &clamped);
            if (clamped) {
              char b[128];
              std::snprintf(
                b, sizeof(b), "%s脚 ID%d が servo_limits の窓の外(%.1fdeg) ",
                rm::kSideTag[s], rm::kLegServoId[j], half[s].servo_tpose_deg[j]);
              note += b;
            }
          }
        }
        if (fmt == Format::kAngle) {continue;}
        double sv[rk::kNumJoints], th[rk::kNumJoints];
        const rm::LegSolve r =
          rm::servoFromFootPose(map.leg_params(s), pose.foot[s], sv, th);
        if (!r.ok()) {
          char b[160];
          std::snprintf(
            b, sizeof(b),
            "%s脚が IK で戻せない(ik=%d servo=%d。--format angle なら出せる) ",
            rm::kSideTag[s],
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
          char b[160];
          std::snprintf(
            b, sizeof(b), "%s脚の往復誤差 %.2fmm(--format angle なら誤差なく出せる) ",
            rm::kSideTag[s], e);
          note += b;
        }
        if (r.ankle_clamped) {note += std::string(rm::kSideTag[s]) + "脚の足首が可動域の外 ";}
      }

      const std::string block = keyframeYaml(map, pose, half, t_next, fmt);
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
    } else if (key == 'f') {
      // ik -> angle -> both -> ik
      if (fmt == Format::kIk) {
        fmt = Format::kAngle;
      } else if (fmt == Format::kAngle) {
        fmt = Format::kBoth;
      } else {
        fmt = Format::kIk;
      }
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
        "  スペース/c 捕まえる   +/- 次の t: を増減   f 書式   n 技の見出し   q 終了\n"
        "  次に出す t: %.2f s   書式: %s   捕まえた枚数: %d\n\n",
        t_next, formatName(fmt), captured);

      static const char * kName[rk::kNumJoints] = {
        "股ピッチ", "股ロール", "股ヨー  ", "膝      ", "足首鎖0 ", "足首鎖1 "};
      for (int s = 0; s < rm::kNumSide; ++s) {
        if (!bank.has(s)) {continue;}
        std::fprintf(
          stderr, "[%s脚] %s\n", rm::kSideTag[s],
          half[s].ok ? (half[s].status == rk::LegServoStatus::Ok ? "" : "★変換に失敗") :
            "★応答が欠けている");
        std::fprintf(stderr, "   関節      ID   サーボ[deg]  関節角[deg]\n");
        for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
          std::fprintf(
            stderr, "   %s  %-3d  %+10.2f   %+10.2f\n", kName[j], rm::kLegServoId[j],
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
      for (int s = 0; s < rm::kNumSide; ++s) {
        for (const auto & e : st[s]) {
          if (e.valid) {volt += e.volt; ++nv;}
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
