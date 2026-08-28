// 脚 FK/IK を 1 行 1 リクエストの JSON サービスとして出す。
//   ros2 run roboone_kinematics leg_service
//
// 可視化 (roboone_motion/viz/serve_leg3d.py) から起動され、ブラウザに出る形は
// すべてここを通る。つまり画面に見えているのは leg_kinematics.hpp そのものの
// 出力で、可視化用に書き直した別実装ではない。
//
// 標準入力（1 行 1 コマンド。長さの単位は mm、角度は deg、座標は Σ_B）:
//   params                                 現在の寸法を返す
//   set <key> <value>                      寸法を差し替える（下の kKeys）
//   fk  <L|R> t1 t2 t3 t4 t5 t6            関節角 -> 関節位置
//   ik  <L|R> x y z                        足裏中心の目標 -> 関節角（足姿勢は水平）
//   ikpose <L|R> x y z roll pitch yaw      足姿勢も指定する版（Σ_B の RPY）
// 標準出力は 1 行 1 JSON。エラーも JSON で返し、プロセスは落とさない。
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "roboone_kinematics/leg_kinematics.hpp"

using namespace roboone_kinematics;

namespace
{

/// set で差し替えられる寸法。Σ_B の値で、右脚のものとして扱う。
struct Dims
{
  double l3 = config::L3, l4 = config::L4, l5 = config::L5, l6 = config::L6;
  double p3x = config::P3_X, p3y = config::P3_Y, p4y = config::P4_Y;
  double hipx = config::HIP_X, hipy = config::HIP_Y, hipz = config::HIP_Z;
  double p6x = config::P6_X, p6y = config::P6_Y, p6z = config::P6_Z;
  int knee = config::KNEE_FORWARD;
};

Dims g_dims;

/// Σ_B の寸法から左右脚の LegParams（Σ_S 成分）を組み立てる。
/// makeLegParams() と同じ手順を、config 定数のかわりに Dims から行う。
LegParams makeParams(Side side)
{
  const double lat = (side == Side::RIGHT) ? 1.0 : -1.0;
  LegParams prm = makeLegParams(side);       // AXIS_FLIP はここから引き継ぐ
  prm.l3 = g_dims.l3;
  prm.l4 = g_dims.l4;
  prm.l5 = g_dims.l5;
  prm.l6 = g_dims.l6;
  prm.a3 = -g_dims.p3y * lat;
  prm.a4 = -g_dims.p4y * lat;
  prm.b = g_dims.p3x;
  prm.p0 = {-g_dims.hipy * lat, g_dims.hipx, g_dims.hipz};
  prm.p6 = {-g_dims.p6y * lat, g_dims.p6x, g_dims.p6z};
  prm.sigma = -g_dims.knee;
  prm.finalize();
  return prm;
}

const char * statusName(IkStatus s)
{
  switch (s) {
    case IkStatus::Ok: return "ok";
    case IkStatus::AnkleOutOfRange: return "ankle_out_of_range";
    case IkStatus::KneeOutOfRange: return "knee_out_of_range";
    default: return "no_branch";
  }
}

void printVec(const Vec3 & v)
{
  std::printf("[%.4f,%.4f,%.4f]", v.x, v.y, v.z);
}

/// 関節位置 5 点と足姿勢を JSON で吐く。
void printPose(const LegParams & prm, const double th[kNumJoints])
{
  Vec3 o[5];
  jointOrigins(prm, th, o);
  Vec3 p;
  Mat3 R;
  fk(prm, th, p, R);

  std::printf("\"theta\":[");
  for (std::size_t k = 0; k < kNumJoints; ++k) {
    std::printf("%s%.6f", k ? "," : "", th[k] * 180.0 / M_PI);
  }
  std::printf("],\"origins\":[");
  for (int k = 0; k < 5; ++k) {
    if (k) {std::printf(",");}
    printVec(o[k]);
  }
  std::printf("],\"foot\":");
  printVec(p);
  std::printf(",\"R\":[");
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {std::printf("%s%.6f", (r || c) ? "," : "", R(r, c));}
  }
  std::printf("]");
}

bool setKey(const std::string & k, double v)
{
  if (k == "l3") {g_dims.l3 = v;} else if (k == "l4") {g_dims.l4 = v;}
  else if (k == "l5") {g_dims.l5 = v;} else if (k == "l6") {g_dims.l6 = v;}
  else if (k == "p3x") {g_dims.p3x = v;} else if (k == "p3y") {g_dims.p3y = v;}
  else if (k == "p4y") {g_dims.p4y = v;} else if (k == "hipx") {g_dims.hipx = v;}
  else if (k == "hipy") {g_dims.hipy = v;} else if (k == "hipz") {g_dims.hipz = v;}
  else if (k == "p6x") {g_dims.p6x = v;} else if (k == "p6y") {g_dims.p6y = v;}
  else if (k == "p6z") {g_dims.p6z = v;}
  else if (k == "knee") {g_dims.knee = (v < 0.0) ? -1 : +1;}
  else {return false;}
  return true;
}

}  // namespace

int main()
{
  std::ios::sync_with_stdio(false);
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream is(line);
    std::string cmd;
    if (!(is >> cmd)) {continue;}

    if (cmd == "params") {
      const LegParams r = makeParams(Side::RIGHT);
      std::printf(
        "{\"ok\":1,\"l3\":%g,\"l4\":%g,\"l5\":%g,\"l6\":%g,\"p3x\":%g,\"p3y\":%g,"
        "\"p4y\":%g,\"hipx\":%g,\"hipy\":%g,\"hipz\":%g,\"knee\":%d,\"reach\":%g,"
        "\"frame\":\"x=front,y=left,z=up\"}\n",
        g_dims.l3, g_dims.l4, g_dims.l5, g_dims.l6, g_dims.p3x, g_dims.p3y,
        g_dims.p4y, g_dims.hipx, g_dims.hipy, g_dims.hipz, g_dims.knee,
        r.l3 + r.l4 + r.l5 + r.l6);
      std::fflush(stdout);
      continue;
    }

    if (cmd == "set") {
      std::string key;
      double v = 0.0;
      if (!(is >> key >> v) || !setKey(key, v)) {
        std::printf("{\"ok\":0,\"error\":\"bad set\"}\n");
      } else {
        std::printf("{\"ok\":1}\n");
      }
      std::fflush(stdout);
      continue;
    }

    std::string sideStr;
    if (!(is >> sideStr)) {
      std::printf("{\"ok\":0,\"error\":\"no side\"}\n");
      std::fflush(stdout);
      continue;
    }
    const Side side = (sideStr == "L" || sideStr == "l") ? Side::LEFT : Side::RIGHT;
    const LegParams prm = makeParams(side);

    if (cmd == "fk") {
      double th[kNumJoints]{};
      bool ok = true;
      for (std::size_t k = 0; k < kNumJoints; ++k) {
        double d = 0.0;
        ok = ok && static_cast<bool>(is >> d);
        th[k] = d * M_PI / 180.0;
      }
      if (!ok) {
        std::printf("{\"ok\":0,\"error\":\"fk needs 6 angles\"}\n");
      } else {
        std::printf("{\"ok\":1,\"status\":\"ok\",");
        printPose(prm, th);
        std::printf("}\n");
      }
      std::fflush(stdout);
      continue;
    }

    if (cmd == "ik" || cmd == "ikpose") {
      double x = 0.0, y = 0.0, z = 0.0, roll = 0.0, pitch = 0.0, yaw = 0.0;
      bool ok = static_cast<bool>(is >> x >> y >> z);
      if (cmd == "ikpose") {ok = ok && static_cast<bool>(is >> roll >> pitch >> yaw);}
      if (!ok) {
        std::printf("{\"ok\":0,\"error\":\"ik needs x y z\"}\n");
        std::fflush(stdout);
        continue;
      }
      const double d2r = M_PI / 180.0;
      // Σ_B の RPY（z-y-x 順）で足姿勢を作る
      const Mat3 R = rotZ(yaw * d2r) * rotY(pitch * d2r) * rotX(roll * d2r);
      double th[kNumJoints]{};
      const IkStatus st = ik(prm, Vec3{x, y, z}, R, th, /*clamp=*/true);
      std::printf("{\"ok\":%d,\"status\":\"%s\",", st == IkStatus::Ok ? 1 : 0,
        statusName(st));
      if (st != IkStatus::NoBranch) {
        printPose(prm, th);
      } else {
        std::printf("\"theta\":null,\"origins\":null");
      }
      std::printf("}\n");
      std::fflush(stdout);
      continue;
    }

    std::printf("{\"ok\":0,\"error\":\"unknown command\"}\n");
    std::fflush(stdout);
  }
  return 0;
}
