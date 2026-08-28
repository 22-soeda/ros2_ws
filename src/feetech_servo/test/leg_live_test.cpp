// 実機のサーボ角を読み取り、脚の姿勢を JSON で流し続ける読み取り専用ツール。
//
//   ros2 run feetech_servo leg_live_test --scan            # 全軸の生角度を表で見る
//   ros2 run feetech_servo leg_live_test --ids 1,2,3,4,5,6 # 関節に割り当てて姿勢を出す
//   ros2 run feetech_servo leg_live_test --ids ... --json  # 1 行 1 JSON（可視化用）
//
// ★このツールは **一切書き込まない**。トルクにも目標位置にも触れないので、機体は
//   脱力したまま。手で膝を曲げれば、その角度がそのまま画面に出る。
//   （書き込み API を呼んでいる箇所が 1 つも無いことをレビューで確認すること）
//
// 使い方の順番:
//   1. --scan で全軸を表示し、動かしたい関節を手で振って **どの ID が動くか**を見る。
//      軸 ID と関節の対応は機体の組み立てで決まり、リポジトリには記録が無い。
//   2. 分かった順（股ピッチ, 股ロール, 股ヨー, 膝, 足首上, 足首下）で --ids に渡す。
//   3. --json を付けると roboone_kinematics の leg_servo.hpp を通した関節角と
//      足裏の姿勢が 1 行 1 JSON で出る。viz/serve_leg_live.py がこれを読む。
//
// 角度の基準は servo_home.yaml の原点（T ポーズ = 脚がまっすぐ真上）。
// つまり全軸 0 deg が「脚が垂直に伸びた姿勢」になる。
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "feetech_servo/feetech_bus.hpp"
#include "roboone_kinematics/leg_servo.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;
namespace rk = roboone_kinematics;

namespace
{
volatile std::sig_atomic_t g_stop = 0;
void on_sigint(int) {g_stop = 1;}

constexpr double kStepsPerDeg = 4096.0 / 360.0;
constexpr double kDeg = 180.0 / M_PI;

struct HomeConfig
{
  double home_deg = 90.0;
  std::map<std::string, std::map<int, int>> home;   // port -> (id -> 原点カウント)
  std::vector<std::string> ports;
};

HomeConfig loadHome(const std::string & path)
{
  HomeConfig cfg;
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "原点ファイルを読めない: %s\n  %s\n", path.c_str(), e.what());
    std::exit(2);
  }
  if (root["home_deg"]) {cfg.home_deg = root["home_deg"].as<double>();}
  for (const auto & b : root["buses"]) {
    if (!b["port"] || !b["servos"]) {continue;}
    const std::string port = b["port"].as<std::string>();
    cfg.ports.push_back(port);
    for (const auto & kv : b["servos"]) {
      const int id = kv.first.as<int>();
      if (kv.second.IsMap() && kv.second["home"]) {
        cfg.home[port][id] = kv.second["home"].as<int>();
      }
    }
  }
  if (cfg.home.empty()) {
    std::fprintf(stderr, "%s: 原点が 1 軸も入っていない\n", path.c_str());
    std::exit(2);
  }
  return cfg;
}

std::vector<int> parseIds(const std::string & s)
{
  std::vector<int> out;
  size_t p = 0;
  while (p < s.size()) {
    size_t c = s.find(',', p);
    if (c == std::string::npos) {c = s.size();}
    out.push_back(std::atoi(s.substr(p, c - p).c_str()));
    p = c + 1;
  }
  return out;
}

/// 生カウント -> 原点基準の角度 [deg]。T ポーズが 0 deg。
double countToDeg(int pos, int home)
{
  return (pos - home) / kStepsPerDeg;
}

void usage()
{
  std::printf(
    "使い方: leg_live_test [--scan] [--ids a,b,c,d,e,f] [--json] [--leg L|R]\n"
    "  --scan     全軸の角度を表で出し続ける（ID と関節の対応を探すため）\n"
    "  --ids      股ピッチ,股ロール,股ヨー,膝,足首上,足首下 の順に軸 ID を並べる\n"
    "  --leg      どちらの脚か（既定 L）。バスは L=左半身 / R=右半身 を選ぶ\n"
    "  --json     1 行 1 JSON で出す（可視化用）\n"
    "  --hz       読み取り周期（既定 30）\n"
    "  --home     原点ファイル（既定 share/feetech_servo/config/servo_home.yaml）\n"
    "  --relax    起動時に **トルクを切る**（手で動かせるようにする）。入れることはしない\n"
    "\n"
    "★書き込みは一切しない。トルクは入らないので、手で関節を動かして確かめられる。\n");
}
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, on_sigint);

  bool scan = false, json = false, relax = false;
  std::string idsArg, leg = "L", homePath;
  double hz = 30.0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--scan") {scan = true;} else if (a == "--json") {json = true;} else if (
      a == "--relax") {relax = true;} else if (
      a == "--ids" && i + 1 < argc) {idsArg = argv[++i];} else if (
      a == "--leg" && i + 1 < argc) {leg = argv[++i];} else if (
      a == "--hz" && i + 1 < argc) {hz = std::atof(argv[++i]);} else if (
      a == "--home" && i + 1 < argc) {homePath = argv[++i];} else {
      usage();
      return a == "--help" || a == "-h" ? 0 : 2;
    }
  }
  if (!scan && idsArg.empty()) {usage(); return 2;}

  if (homePath.empty()) {
    const char * share = std::getenv("FEETECH_SHARE");
    homePath = share ? std::string(share) + "/config/servo_home.yaml"
      : std::string("/home/auto/ros2_ws/install/feetech_servo/share/feetech_servo"
        "/config/servo_home.yaml");
  }
  const HomeConfig home = loadHome(homePath);

  const bool right = (leg == "R" || leg == "r");
  const std::string port = right ? "/dev/feetech_right" : "/dev/feetech_left";
  if (home.home.find(port) == home.home.end()) {
    std::fprintf(stderr, "原点ファイルに %s が無い\n", port.c_str());
    return 2;
  }
  const auto & hmap = home.home.at(port);

  FeetechBus bus(port, 1000000, 0, 20, feetech_servo::Family::kHls);
  if (!bus.open()) {
    std::fprintf(stderr, "%s を開けない（dialout グループか、udev ルールを確認）\n",
      port.c_str());
    return 2;
  }
  std::fprintf(stderr, "%s を読み取り専用で開いた（書き込みはしない）\n", port.c_str());

  // 読む軸を決める
  std::vector<uint8_t> ids;
  std::vector<int> jointIds;
  if (scan) {
    for (const auto & kv : hmap) {ids.push_back(static_cast<uint8_t>(kv.first));}
  } else {
    jointIds = parseIds(idsArg);
    if (jointIds.size() != static_cast<size_t>(rk::kNumJoints)) {
      std::fprintf(stderr, "--ids は %d 個必要（渡された数 %zu）\n",
        rk::kNumJoints, jointIds.size());
      return 2;
    }
    for (int id : jointIds) {
      if (hmap.find(id) == hmap.end()) {
        std::fprintf(stderr, "ID %d の原点が %s に無い\n", id, port.c_str());
        return 2;
      }
      ids.push_back(static_cast<uint8_t>(id));
    }
  }

  // --relax のときだけ、起動時に 1 回だけトルクを切る。**入れる経路は持たない**
  // （enable_torque(id, true) も init_motor() もこのファイルには無い）。
  if (relax) {
    int n = 0;
    for (uint8_t id : ids) {n += bus.enable_torque(id, false) ? 1 : 0;}
    std::fprintf(stderr, "トルクを切った（%d / %zu 軸）。手で動かせる\n", n, ids.size());
  } else {
    std::fprintf(stderr,
      "書き込みは一切しない。関節が固ければ --relax を付けてトルクを切ること\n");
  }

  const rk::LegServoParams prm =
    rk::makeLegServoParams(right ? rk::Side::RIGHT : rk::Side::LEFT);
  const long periodUs = static_cast<long>(1e6 / std::max(1.0, hz));
  double th6Seed = 0.0;
  std::vector<ServoState> st;
  long frame = 0;

  if (scan && !json) {
    std::printf("\n手で関節を動かすと、その軸の角度だけが変わる。Ctrl-C で終了。\n");
    std::printf("角度は servo_home.yaml の原点（T ポーズ）を 0 deg とした値。\n\n");
  }

  while (!g_stop) {
    const int n = bus.sync_read_states(ids, st);
    (void)n;

    if (scan) {
      std::printf("\033[H\033[J%s  読めた軸 %d / %zu\n\n", port.c_str(), n, ids.size());
      std::printf("  ID   生カウント   角度[deg]   電圧    err\n");
      for (size_t i = 0; i < ids.size(); ++i) {
        const int id = ids[i];
        if (i >= st.size() || !st[i].valid) {
          std::printf("  %-4d      ----        ----     ----   応答なし\n", id);
          continue;
        }
        std::printf("  %-4d   %6d   %+9.2f   %5.1fV   %s\n", id, st[i].pos,
          countToDeg(st[i].pos, hmap.at(id)), st[i].volt,
          st[i].err ? feetech_servo::err_str(st[i].err).c_str() : "-");
      }
      std::fflush(stdout);
    } else {
      double servo[rk::kNumJoints] = {0};
      bool allOk = true;
      for (int j = 0; j < rk::kNumJoints; ++j) {
        if (static_cast<size_t>(j) >= st.size() || !st[j].valid) {allOk = false; continue;}
        servo[j] = countToDeg(st[j].pos, hmap.at(jointIds[j])) / kDeg;
      }
      double theta[rk::kNumJoints] = {0};
      const rk::LegServoStatus lst =
        rk::legJointsFromServo(prm, servo, theta, th6Seed);
      if (lst == rk::LegServoStatus::Ok) {th6Seed = theta[rk::ANKLE_ROLL];}

      // 足首パラレルリンクのクランク角（表示用）
      const double q[rk::kAnkleChains] = {
        rk::ankleCrankFromServo(prm.ankle, 0, servo[rk::ANKLE_PITCH]),
        rk::ankleCrankFromServo(prm.ankle, 1, servo[rk::ANKLE_ROLL])};

      if (json) {
        std::printf("{\"frame\":%ld,\"ok\":%s,\"status\":%d,\"leg\":\"%s\"",
          frame, allOk ? "true" : "false", static_cast<int>(lst), right ? "R" : "L");
        std::printf(",\"servo_deg\":[");
        for (int j = 0; j < rk::kNumJoints; ++j) {
          std::printf("%s%.4f", j ? "," : "", servo[j] * kDeg);
        }
        std::printf("],\"theta_deg\":[");
        for (int j = 0; j < rk::kNumJoints; ++j) {
          std::printf("%s%.4f", j ? "," : "", theta[j] * kDeg);
        }
        std::printf("],\"crank_deg\":[%.4f,%.4f]", q[0] * kDeg, q[1] * kDeg);

        // 描画用の 3D 点。**すべて ankle_parallel.hpp が計算したもの**で、
        // ブラウザ側は線を引くだけ。可視化のために式を書き直さない。
        // 座標は Σ_s（原点 o5、x 前 / y 左 / z 上）、単位 mm。
        {
          const double t5 = theta[rk::ANKLE_PITCH], t6 = theta[rk::ANKLE_ROLL];
          const rk::AnkleParams & ap = prm.ankle;
          const double c5 = std::cos(t5), s5 = std::sin(t5);
          const double c6 = std::cos(t6), s6 = std::sin(t6);
          std::printf(",\"pts\":{\"o5\":[0,0,0],\"o6\":[0,%.3f,%.3f]",
            -s5 * ap.p5.z, c5 * ap.p5.z);
          std::printf(",\"chain\":[");
          for (int i = 0; i < rk::kAnkleChains; ++i) {
            const rk::Vec3 O = ap.c[i];
            const rk::Vec3 K = rk::ankleCrank(ap, i, q[i]);
            const rk::Vec3 B = rk::ankleBall(ap, i, t5, t6);
            std::printf("%s{\"O\":[%.3f,%.3f,%.3f],\"K\":[%.3f,%.3f,%.3f],"
              "\"B\":[%.3f,%.3f,%.3f]}", i ? "," : "",
              O.x, O.y, O.z, K.x, K.y, K.z, B.x, B.y, B.z);
          }
          // 足裏の四隅。★寸法は仮置き（CAD 確定後に差し替える）。Σ_6 に置いて
          //   足の姿勢 R5·R6 を掛ける。
          std::printf("],\"sole\":[");
          const double sx = 60.0, sy = 35.0, sz = -34.0;
          const double corner[4][3] = {{sx, sy, sz}, {sx, -sy, sz},
            {-sx, -sy, sz}, {-sx, sy, sz}};
          for (int k = 0; k < 4; ++k) {
            const double cx = corner[k][0], cy = corner[k][1], cz = corner[k][2];
            const double rx = c6 * cx + s6 * cz;
            const double ry = cy;
            const double rz = -s6 * cx + c6 * cz + ap.p5.z;
            std::printf("%s[%.3f,%.3f,%.3f]", k ? "," : "",
              rx, c5 * ry - s5 * rz, s5 * ry + c5 * rz);
          }
          std::printf("]}");
        }
        std::printf(",\"volt\":%.2f}\n", st.empty() ? 0.0 : st[0].volt);
        std::fflush(stdout);
      } else {
        std::printf("\033[H\033[Jstatus=%d  %s\n\n", static_cast<int>(lst),
          allOk ? "" : "★一部の軸が応答していない");
        static const char * kName[] = {"股ピッチ", "股ロール", "股ヨー ", "膝    ",
          "足首上 ", "足首下 "};
        std::printf("  関節       ID   サーボ[deg]   関節角[deg]\n");
        for (int j = 0; j < rk::kNumJoints; ++j) {
          std::printf("  %s  %-4d  %+9.2f    %+9.2f\n", kName[j], jointIds[j],
            servo[j] * kDeg, theta[j] * kDeg);
        }
        std::printf("\n  足首クランク q = (%+.2f, %+.2f) deg\n", q[0] * kDeg, q[1] * kDeg);
        std::fflush(stdout);
      }
    }
    ++frame;
    usleep(periodUs);
  }
  std::printf("\n終了（書き込みは一度も行っていない）\n");
  return 0;
}
