// feetech_calibrate_home: 各軸の「初期位置（原点）」を手で合わせて記録する対話ツール。
//
// サーボの生カウント 2047 が機構上の 90deg とは限らないので、目で見て合わせた姿勢の
// 生カウントを軸ごとに記録し、YAML に保存する。EEPROM は一切書き換えない（OFS(31) も触らない）。
//
//   ros2 run feetech_servo feetech_calibrate_home                 # 全軸を1軸ずつ
//   ros2 run feetech_servo feetech_calibrate_home --bus 0 --id 3  # bus0 の ID3 だけやり直す
//   ros2 run feetech_servo feetech_calibrate_home -o ~/home.yaml  # 保存先を変える
//
// 進め方（1軸ずつ・前の軸はトルクONで保持されるので姿勢が崩れない）:
//   1. その軸のトルクを切る → 手で動かせる
//   2. 現在の生カウント/角度を画面に**リアルタイム表示**（20Hz）しながら目で合わせる
//   3. Enter で確定 → その位置を保持したまま次の軸へ
//   微調整は矢印キー（トルクを自動でONにして 1/10 ステップずつ動かす）
//
// 安全のため:
//   - 起動時に「軸を脱力させる＝機体が落ちる」ことを確認する（--yes で省略）
//   - 現在位置が読めない軸にはトルクを入れない／確定もしない
//   - 既存の保存内容は読み込んでマージする（飛ばした軸の値は消えない）
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <scservo/SMS_STS.h>

#include "feetech_servo/feetech_manager.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;

namespace
{

constexpr int kPosMax = 4095;
constexpr int kCenter = 2048;       // 0-4095 の中央（= 180deg）
constexpr int kReadAttempts = 5;    // 低電圧時は応答が間欠的に欠けるので数回試す

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop = true; }

float deg_of(int steps) { return steps * 360.0f / 4096.0f; }

// --- 端末を raw にして「押した瞬間」にキーを拾う（矢印キーも取れる）---
struct RawTty
{
  termios saved{};
  bool active = false;
  int saved_flags = 0;
  bool nonblock = false;
  // stdin をノンブロッキングにする。端末でなくても（パイプ入力でも）表示ループが止まらない。
  void enable_nonblock()
  {
    saved_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (saved_flags >= 0 && fcntl(STDIN_FILENO, F_SETFL, saved_flags | O_NONBLOCK) == 0) {
      nonblock = true;
    }
  }
  void enable()
  {
    enable_nonblock();
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved) != 0) {
      return;
    }
    termios raw = saved;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;   // read は待たずに返る（表示ループを回し続けるため）
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      active = true;
    }
  }
  void restore()
  {
    if (active) {
      tcsetattr(STDIN_FILENO, TCSANOW, &saved);
      active = false;
    }
    if (nonblock) {
      fcntl(STDIN_FILENO, F_SETFL, saved_flags);
      nonblock = false;
    }
  }
  ~RawTty() { restore(); }
};

// 押されたキーの意味（矢印キーのエスケープ列は Key に畳んでから扱う）
enum class Key { kNone, kEnter, kSkip, kBack, kTorque, kQuit, kHelp, kJog };

struct KeyEvent
{
  Key key = Key::kNone;
  int jog = 0;  // kJog のときの移動量（ステップ）
};

// 入力バッファから 1 イベント取り出す。取れなければ kNone。
KeyEvent take_key(std::string & buf)
{
  while (!buf.empty()) {
    const unsigned char c = static_cast<unsigned char>(buf[0]);
    if (c == 0x1b) {  // ESC [ A/B/C/D = 矢印
      if (buf.size() < 3) {
        return {};      // まだ続きが来ていない
      }
      if (buf[1] == '[') {
        const char d = buf[2];
        buf.erase(0, 3);
        switch (d) {
          case 'A': return {Key::kJog, +10};
          case 'B': return {Key::kJog, -10};
          case 'C': return {Key::kJog, +1};
          case 'D': return {Key::kJog, -1};
          default: continue;
        }
      }
      buf.erase(0, 1);
      continue;
    }
    buf.erase(0, 1);
    switch (c) {
      case '\r': case '\n': return {Key::kEnter};
      case 's': case 'S': return {Key::kSkip};
      case 'b': case 'B': return {Key::kBack};
      case 't': case 'T': return {Key::kTorque};
      case 'q': case 'Q': case 0x03 /*Ctrl-C*/: return {Key::kQuit};
      case '?': case 'h': return {Key::kHelp};
      case '+': case '=': return {Key::kJog, +1};
      case '-': case '_': return {Key::kJog, -1};
      case ']': return {Key::kJog, +10};
      case '[': return {Key::kJog, -10};
      default: continue;
    }
  }
  return {};
}

struct Axis
{
  int id = 0;
  int lim_min = 0;      // 0,0 = 制限なし
  int lim_max = 0;
  bool alive = false;
  bool has_prev = false;  // 前回の保存値があるか
  int prev_home = 0;
  bool done = false;      // 今回確定したか
  int home = 0;
};

struct BusCal
{
  std::string port;
  std::vector<Axis> axes;
  FeetechBus * bus = nullptr;
};

// 軸一覧は servo_limits.yaml から取る（ポートとIDの一覧はここが唯一の正）。
std::vector<BusCal> load_axes(const std::string & path, int & baud)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "軸一覧を読めない: %s\n  %s\n", path.c_str(), e.what());
    std::exit(2);
  }
  if (root["baud"]) {
    baud = root["baud"].as<int>();
  }
  if (!root["buses"] || !root["buses"].IsSequence()) {
    std::fprintf(stderr, "%s: `buses:` が無い\n", path.c_str());
    std::exit(2);
  }
  std::vector<BusCal> out;
  for (const auto & b : root["buses"]) {
    BusCal cal;
    if (!b["port"] || !b["servos"] || !b["servos"].IsMap()) {
      std::fprintf(stderr, "%s: buses の要素に port/servos が無い\n", path.c_str());
      std::exit(2);
    }
    cal.port = b["port"].as<std::string>();
    for (const auto & kv : b["servos"]) {
      Axis a;
      a.id = kv.first.as<int>();
      const YAML::Node & v = kv.second;
      if (v.IsSequence() && v.size() == 2) {
        a.lim_min = v[0].as<int>();
        a.lim_max = v[1].as<int>();
      } else if (v.IsMap() && v["min"] && v["max"]) {
        a.lim_min = v["min"].as<int>();
        a.lim_max = v["max"].as<int>();
      }
      cal.axes.push_back(a);
    }
    std::sort(
      cal.axes.begin(), cal.axes.end(), [](const Axis & x, const Axis & y) {return x.id < y.id;});
    out.push_back(std::move(cal));
  }
  return out;
}

// 既存の保存内容を読んでマージ元にする（無ければ何もしない）。
void load_previous(const std::string & path, std::vector<BusCal> & cals, float & home_deg)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception &) {
    return;  // 初回は存在しなくて当たり前
  }
  if (root["home_deg"]) {
    home_deg = root["home_deg"].as<float>();
  }
  if (!root["buses"] || !root["buses"].IsSequence()) {
    return;
  }
  for (const auto & b : root["buses"]) {
    if (!b["port"] || !b["servos"]) {
      continue;
    }
    const std::string port = b["port"].as<std::string>();
    for (BusCal & cal : cals) {
      if (cal.port != port) {
        continue;
      }
      for (const auto & kv : b["servos"]) {
        const int id = kv.first.as<int>();
        int home = -1;
        if (kv.second.IsMap() && kv.second["home"]) {
          home = kv.second["home"].as<int>();
        } else if (kv.second.IsScalar()) {
          home = kv.second.as<int>();
        }
        if (home < 0) {
          continue;
        }
        for (Axis & a : cal.axes) {
          if (a.id == id) {
            a.has_prev = true;
            a.prev_home = home;
          }
        }
      }
    }
  }
}

ServoState read_retry(FeetechBus * bus, int id)
{
  ServoState st;
  for (int i = 0; i < kReadAttempts && !st.valid; ++i) {
    st = bus->read_state(static_cast<uint8_t>(id));
  }
  return st;
}

bool save_yaml(
  const std::string & path, const std::vector<BusCal> & cals, float home_deg, const char * date)
{
  FILE * f = std::fopen(path.c_str(), "w");
  if (!f) {
    std::fprintf(stderr, "保存できない: %s (%s)\n", path.c_str(), std::strerror(errno));
    return false;
  }
  std::fprintf(
    f,
    "# サーボの初期位置（原点）オフセット。feetech_calibrate_home が生成（%s）。\n"
    "#\n"
    "#   home   … その軸を狙いの姿勢に合わせたときの生カウント (0-4095)\n"
    "#   offset … home - %d（中央からのずれ。参考値）\n"
    "#\n"
    "# 使い方（狙いの姿勢を %.1f deg と定義した場合）:\n"
    "#   指令カウント = home + (狙い角[deg] - %.1f) * 4096 / 360\n"
    "#   現在角[deg]  = %.1f + (現在カウント - home) * 360 / 4096\n"
    "#\n"
    "# 手で合わせ直したくなったら: ros2 run feetech_servo feetech_calibrate_home\n"
    "# （--bus/--id で1軸だけやり直せる。このファイルは軸ごとにマージされる）\n\n",
    date, kCenter, home_deg, home_deg, home_deg);
  std::fprintf(f, "center: %d\n", kCenter);
  std::fprintf(f, "home_deg: %.1f\n\n", home_deg);
  std::fprintf(f, "buses:\n");
  for (const BusCal & cal : cals) {
    std::fprintf(f, "  - port: %s\n", cal.port.c_str());
    std::fprintf(f, "    servos:\n");
    bool any = false;
    for (const Axis & a : cal.axes) {
      const bool have = a.done || a.has_prev;
      if (!have) {
        continue;
      }
      const int home = a.done ? a.home : a.prev_home;
      char key[8];
      std::snprintf(key, sizeof(key), "%d:", a.id);
      std::fprintf(
        f, "      %-4s {home: %4d, offset: %5d}   # %.1f deg%s\n",
        key, home, home - kCenter, deg_of(home), a.done ? "" : "  (前回の値)");
      any = true;
    }
    if (!any) {
      std::fprintf(f, "      {}  # 未校正\n");
    }
  }
  std::fclose(f);
  return true;
}

void print_keys()
{
  std::printf(
    "  キー: [Enter]確定して次へ  [s]この軸は飛ばす  [b]前の軸に戻る  [t]トルクON/OFF\n"
    "        [←][→] 1ステップ動かす  [↑][↓] 10ステップ  [q]保存して終了  [?]このヘルプ\n");
}

void usage(const char * argv0)
{
  std::printf(
    "使い方: %s [-c FILE] [-o FILE] [--bus N] [--id N] [--home-deg D] [--yes]\n"
    "         [--baud N] [--speed N] [--acc N] [--torque N] [--timeout MS] [--no-hold]\n"
    "  -c, --config   軸一覧（既定: share/feetech_servo/config/servo_limits.yaml）\n"
    "  -o, --out      保存先（既定: src/feetech_servo/config/servo_home.yaml があればそこ、\n"
    "                 無ければ ./servo_home.yaml）\n"
    "  --bus N        このバス（0始まり）だけ校正する\n"
    "  --id N         このIDだけ校正する（--bus と併用可）\n"
    "  --home-deg D   合わせる姿勢を何 deg と呼ぶか（既定 90。保存ファイルに記録するだけ）\n"
    "  --no-hold      確定後にトルクで保持しない（既定は保持して姿勢が崩れないようにする）\n"
    "  --no-release   軸に来たとき自動で脱力しない（矢印キーだけで合わせる場合）\n"
    "  --yes, -y      起動時の確認を省略する\n"
    "  --speed/--acc  微調整で動かすときの速度・加速度（既定 300 / 20）\n"
    "  --torque N     HLS系の目標トルク 0-1000（既定 1000）。0 だと駆動しない\n"
    "  --timeout MS   1トランザクションの受信タイムアウト（既定 20）\n",
    argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string config_path;
  std::string out_path;
  int baud_override = 0;
  int timeout_ms = 20;
  int only_bus = -1;
  int only_id = -1;
  float home_deg = 90.0f;
  int speed = 300;
  int acc = 20;
  int goal_torque = 1000;
  bool assume_yes = false;
  bool hold_after = true;
  bool release_on_enter = true;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&]() -> const char * {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "%s に値がない\n", a.c_str());
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "-c" || a == "--config") {config_path = need();} else if (a == "-o" || a == "--out") {
      out_path = need();
    } else if (a == "--bus") {only_bus = std::atoi(need());} else if (a == "--id") {
      only_id = std::atoi(need());
    } else if (a == "--home-deg") {
      home_deg = static_cast<float>(std::atof(need()));
    } else if (a == "--baud") {baud_override = std::atoi(need());} else if (a == "--speed") {
      speed = std::atoi(need());
    } else if (a == "--acc") {acc = std::atoi(need());} else if (a == "--torque") {
      goal_torque = std::atoi(need());
    } else if (a == "--timeout") {timeout_ms = std::atoi(need());} else if (a == "--no-hold") {
      hold_after = false;
    } else if (a == "--no-release") {release_on_enter = false;} else if (a == "-y" || a == "--yes") {
      assume_yes = true;
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "不明な引数: %s\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
  }

  if (config_path.empty()) {
    try {
      config_path = ament_index_cpp::get_package_share_directory("feetech_servo") +
        "/config/servo_limits.yaml";
    } catch (const std::exception & e) {
      std::fprintf(stderr, "既定の軸一覧の場所を解決できない: %s\n", e.what());
      return 2;
    }
  }
  if (out_path.empty()) {
    // ワークスペース直下で実行しているなら、そのままソースツリーの config/ に置く。
    struct stat sb;
    if (stat("src/feetech_servo/config", &sb) == 0 && S_ISDIR(sb.st_mode)) {
      out_path = "src/feetech_servo/config/servo_home.yaml";
    } else {
      out_path = "servo_home.yaml";
    }
  }

  int baud = 1000000;
  std::vector<BusCal> cals = load_axes(config_path, baud);
  if (baud_override > 0) {
    baud = baud_override;
  }
  load_previous(out_path, cals, home_deg);

  // --bus / --id は「今回まわる軸」だけを絞る。cals からは消さない
  // （消すと、その軸の前回値まで保存ファイルから落ちてしまう）。
  if (only_bus >= 0 && only_bus >= static_cast<int>(cals.size())) {
    std::fprintf(stderr, "bus %d は無い（0..%zu）\n", only_bus, cals.size() - 1);
    return 2;
  }
  struct Ref { size_t bus; size_t axis; };
  std::vector<Ref> order;
  for (size_t b = 0; b < cals.size(); ++b) {
    if (only_bus >= 0 && static_cast<size_t>(only_bus) != b) {
      continue;
    }
    for (size_t a = 0; a < cals[b].axes.size(); ++a) {
      if (only_id > 0 && cals[b].axes[a].id != only_id) {
        continue;
      }
      order.push_back({b, a});
    }
  }
  const int total_axes = static_cast<int>(order.size());
  if (total_axes == 0) {
    std::fprintf(stderr, "対象の軸が無い（--bus/--id の指定を確認）\n");
    return 2;
  }

  std::printf("軸一覧: %s\n保存先: %s\n", config_path.c_str(), out_path.c_str());
  std::printf(
    "対象 %d 軸 / 合わせる姿勢を %.1f deg と呼ぶ / baud=%d\n", total_axes, home_deg, baud);
  std::printf(
    "\n1軸ずつ、その軸だけ脱力させて手で合わせる。確定した軸は%s。\n",
    hold_after ? "トルクで保持するので姿勢は崩れない" : "脱力したまま（--no-hold）");
  std::printf("EEPROM は書き換えない（保存は YAML だけ）。\n");
  if (!assume_yes) {
    std::printf("\n脱力した軸は自重で落ちる。支えられる準備ができたら yes と入力: ");
    std::fflush(stdout);
    std::string ans;
    if (!std::getline(std::cin, ans) || (ans != "yes" && ans != "y")) {
      std::printf("中止した\n");
      return 1;
    }
  }

  // --- 接続 ---
  feetech_servo::FeetechManager mgr;
  for (BusCal & cal : cals) {
    cal.bus = mgr.add_bus(cal.port, baud, 0, timeout_ms);
    if (!cal.bus) {
      std::fprintf(stderr, "%s を開けない（未接続 / dialout権限なし）。中止\n", cal.port.c_str());
      return 1;
    }
    cal.bus->set_goal_torque(static_cast<uint16_t>(goal_torque));
  }
  if (goal_torque == 0) {
    std::printf("※ 目標トルクが0なので矢印キーで動かない（--torque 1000）\n");
  }

  std::signal(SIGINT, on_sigint);
  const bool interactive = isatty(STDIN_FILENO) != 0;
  RawTty tty;
  tty.enable();

  bool quit = false;
  std::string keybuf;   // 軸をまたいで持ち越す（まとめて届いたキーを捨てない）
  for (size_t idx = 0; idx < order.size() && !quit && !g_stop; ++idx) {
    BusCal & cal = cals[order[idx].bus];
    Axis & ax = cal.axes[order[idx].axis];
    const uint8_t id = static_cast<uint8_t>(ax.id);

    // --- この軸の準備 ---
    ax.alive = false;
    for (int i = 0; i < kReadAttempts && !ax.alive; ++i) {
      ax.alive = cal.bus->ping(id);
    }
    std::printf("\n────────────────────────────────────────────────────────\n");
    std::printf(
      "[%zu/%zu] %s  ID %d", idx + 1, order.size(), cal.port.c_str(), ax.id);
    if (ax.lim_min != 0 || ax.lim_max != 0) {
      std::printf("   リミット %d..%d", ax.lim_min, ax.lim_max);
    }
    if (ax.has_prev) {
      std::printf("   前回 %d", ax.prev_home);
    }
    std::printf("\n");
    if (!ax.alive) {
      std::printf("  応答なし → 飛ばす（電源電圧と配線を確認）\n");
      continue;
    }

    ServoState st = read_retry(cal.bus, ax.id);
    if (!st.valid) {
      std::printf("  現在位置が読めない → 飛ばす\n");
      continue;
    }
    bool torque_on = cal.bus->read_byte(id, SMS_STS_TORQUE_ENABLE) == 1;
    int goal = st.pos;  // トルクON中の目標位置（矢印キーで動かす）
    if (release_on_enter && torque_on) {
      torque_on = !cal.bus->enable_torque(id, false);
    }
    std::printf(
      "  %s。目で見て合わせたら Enter\n",
      torque_on ? "トルクON中（[t]で脱力 / 矢印キーで微調整）" : "脱力した。手で動かせる");
    print_keys();

    // --- リアルタイム表示 + キー待ち ---
    bool next_axis = false;
    ServoState last = st;   // 最後に読めた状態（表示用）
    bool last_valid = true;
    int lost = 0;
    while (!next_axis && !g_stop) {
      st = cal.bus->read_state(id);
      if (st.valid) {
        last = st;
        last_valid = true;
        lost = 0;
      } else if (++lost > 10) {
        last_valid = false;  // 10回連続で欠けたら「読めていない」と出す
      }

      char prev_col[48] = "";
      if (ax.has_prev) {
        std::snprintf(prev_col, sizeof(prev_col), "  前回比 %+d", last.pos - ax.prev_home);
      }
      if (last_valid) {
        std::printf(
          "\r  ▶ value %4d   %6.1f deg   中央から %+5d   %s  %.1fV %d℃%s%s        ",
          last.pos, deg_of(last.pos), last.pos - kCenter,
          torque_on ? "トルクON" : "脱力中  ", last.volt, last.temp, prev_col,
          last.volt < 9.0f ? "  ※低電圧" : "");
      } else {
        std::printf("\r  ▶ 応答が欠けている（電源電圧と配線を確認）                    ");
      }
      std::fflush(stdout);

      // キー入力（ノンブロッキング）
      char rb[32];
      const ssize_t n = ::read(STDIN_FILENO, rb, sizeof(rb));
      if (n > 0) {
        keybuf.append(rb, static_cast<size_t>(n));
      } else if (n == 0 && !interactive && keybuf.empty()) {
        // パイプ入力が尽きた = これ以上キーは来ない
        std::printf("\n  入力が尽きたので終了する\n");
        quit = true;
        break;
      }
      for (KeyEvent ev = take_key(keybuf); ev.key != Key::kNone; ev = take_key(keybuf)) {
        switch (ev.key) {
          case Key::kEnter: {
            const ServoState fix = read_retry(cal.bus, ax.id);
            if (!fix.valid) {
              std::printf("\n  位置が読めない。もう一度 Enter\n");
              break;
            }
            ax.done = true;
            ax.home = fix.pos;
            std::printf(
              "\n  確定: ID %d の原点 = %d (%.1f deg / 中央から %+d)\n",
              ax.id, ax.home, deg_of(ax.home), ax.home - kCenter);
            if (ax.lim_min != ax.lim_max &&
              (ax.home < ax.lim_min || ax.home > ax.lim_max))
            {
              std::printf(
                "  ※ 角度リミット %d..%d の外。トルクを入れると端まで動く\n",
                ax.lim_min, ax.lim_max);
            }
            if (hold_after) {
              cal.bus->write_position(
                id, static_cast<int16_t>(ax.home), static_cast<uint16_t>(speed),
                static_cast<uint8_t>(acc));
              std::printf(
                "  この位置で保持: %s\n",
                cal.bus->enable_torque(id, true) ? "トルクON" : "失敗（応答なし）");
            }
            next_axis = true;
            break;
          }
          case Key::kSkip:
            std::printf(
              "\n  飛ばした%s\n", ax.has_prev ? "（前回の値を残す）" : "（未校正のまま）");
            next_axis = true;
            break;
          case Key::kBack:
            if (idx == 0) {
              std::printf("\n  ここが最初の軸\n");
              break;
            }
            idx -= 2;      // ループ末尾の ++idx と合わせて1つ前へ
            next_axis = true;
            std::printf("\n  前の軸に戻る\n");
            break;
          case Key::kTorque: {
            if (torque_on) {
              torque_on = !cal.bus->enable_torque(id, false);
              std::printf("\n  %s\n", torque_on ? "脱力に失敗" : "脱力した。手で動かせる");
            } else {
              const ServoState cur = read_retry(cal.bus, ax.id);
              if (!cur.valid) {
                std::printf("\n  位置が読めないのでトルクは入れない（飛び出し防止）\n");
                break;
              }
              goal = cur.pos;
              cal.bus->write_position(
                id, static_cast<int16_t>(goal), static_cast<uint16_t>(speed),
                static_cast<uint8_t>(acc));
              torque_on = cal.bus->enable_torque(id, true);
              std::printf(
                "\n  %s\n", torque_on ? "トルクON（現在位置で保持）" : "トルクONに失敗");
            }
            print_keys();
            break;
          }
          case Key::kJog: {
            if (!torque_on) {
              // 微調整はトルクが要る。現在位置を目標にしてから入れる（飛び出し防止）
              const ServoState cur = read_retry(cal.bus, ax.id);
              if (!cur.valid) {
                std::printf("\n  位置が読めないのでトルクは入れない\n");
                break;
              }
              goal = cur.pos;
              cal.bus->write_position(
                id, static_cast<int16_t>(goal), static_cast<uint16_t>(speed),
                static_cast<uint8_t>(acc));
              torque_on = cal.bus->enable_torque(id, true);
              if (!torque_on) {
                std::printf("\n  トルクONに失敗（応答なし）\n");
                break;
              }
            }
            goal += ev.jog;
            int lo = 0, hi = kPosMax;
            if (ax.lim_min != 0 || ax.lim_max != 0) {   // 0,0 は制限なし
              lo = ax.lim_min;
              hi = ax.lim_max;
            }
            goal = std::max(lo, std::min(hi, goal));
            cal.bus->write_position(
              id, static_cast<int16_t>(goal), static_cast<uint16_t>(speed),
              static_cast<uint8_t>(acc));
            break;
          }
          case Key::kHelp:
            std::printf("\n");
            print_keys();
            break;
          case Key::kQuit:
            std::printf("\n  中断して、ここまでの結果を保存する\n");
            quit = true;
            next_axis = true;
            break;
          case Key::kNone:
            break;
        }
        if (next_axis) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz
    }
  }
  tty.restore();

  // --- 結果 ---
  std::printf("\n────────────────────────────────────────────────────────\n結果:\n");
  int done = 0, kept = 0, missing = 0;
  for (const BusCal & cal : cals) {
    std::printf("\n[%s]\n  %-4s %-8s %-10s %-8s %s\n", cal.port.c_str(), "ID", "原点", "角度",
      "中央から", "");
    for (const Axis & a : cal.axes) {
      if (a.done) {
        ++done;
        std::printf(
          "  %-4d %-8d %-10.1f %+-8d %s\n", a.id, a.home, deg_of(a.home), a.home - kCenter,
          a.has_prev ? "更新" : "新規");
      } else if (a.has_prev) {
        ++kept;
        std::printf(
          "  %-4d %-8d %-10.1f %+-8d %s\n", a.id, a.prev_home, deg_of(a.prev_home),
          a.prev_home - kCenter, "前回の値");
      } else {
        ++missing;
        std::printf("  %-4d %-8s %-10s %-8s %s\n", a.id, "-", "-", "-", "未校正");
      }
    }
  }
  std::printf("\n確定 %d 軸 / 前回の値を維持 %d 軸 / 未校正 %d 軸\n", done, kept, missing);

  if (done == 0 && kept == 0) {
    std::printf("保存する値が無い。ファイルは書き換えない\n");
    return 1;
  }
  char date[32] = "";
  {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::strftime(date, sizeof(date), "%Y-%m-%d %H:%M", &tm);
  }
  if (!save_yaml(out_path, cals, home_deg, date)) {
    return 1;
  }
  std::printf("保存した: %s\n", out_path.c_str());
  if (out_path.rfind("src/", 0) == 0) {
    // ツール類が読むのは install 側のコピーなので、ソースに書いただけでは反映されない。
    std::printf(
      "  ※ 反映には `colcon build --packages-select feetech_servo` が必要\n"
      "     （すぐ使うなら feetech_goto_test --home %s）\n", out_path.c_str());
  }
  if (missing > 0) {
    std::printf("  ※ 未校正の軸が %d 個。同じコマンドで続きから合わせられる\n", missing);
  }
  if (hold_after) {
    std::printf("  ※ 確定した軸はトルクONのまま。脱力するなら feetech_shell の off\n");
  }
  return 0;
}
