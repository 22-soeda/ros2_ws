// motion_selftest — 姿勢の表し方まわりの自己検算。実機もサーボも要らない。
//
//   ros2 run roboone_motion motion_selftest
//   ros2 run roboone_motion motion_selftest --motions /tmp/draft.yaml
//
// 見るのは 8 つ。どれも「実機で踏むと原因が非常に追いにくい」種類の食い違い。
//
//   [1] T ポーズ基準角 [deg] <-> 絶対サーボ角 [rad] <-> 生カウント が一巡すること
//       (servo_map.hpp の [1][2][3]。config に書く角と機体を流れる角の橋)
//   [2] 胴体の前傾を「足裏を回して IK」と「股ピッチのサーボ角に足す」の 2 通りで
//       掛けた結果が一致すること (body_pose.hpp の等価性。**左脚は AXIS_FLIP で
//       サーボの向きが逆**なので、符号を落とすとここで開く)
//   [3] 実機の motions.yaml が読めること
//   [4] 脚の角度書き (R_leg / L_leg) の読み込み・引き継ぎ・書き方が混ざる区間
//   [5] 状態機械の順序 (motion_control.hpp「順序の約束」)。**全部、実機でしか
//       踏めなかったバグの修正**なので、テストが無いと再発しても気付けない。
//       武装の起点 (pose_codec.hpp「実測姿勢」) は生カウントから decode() を通して
//       確かめる。脱力して垂れた足首 (2026-09-17 の bag の値) から跳ばずに立つこと
//   [6] 設定の門 (motion_config.hpp)。gait.yaml / home_pose.yaml を書き換えたあと、
//       実機を起こす前にここで確かめられる。**既定では報告だけで落とさない。**
//       [1]-[5] はコードの検算なので赤いままにしてはいけないが、[6] が見ているのは
//       その時どきの config（調整の途中なら当然エラーが出る）。混ぜると
//       「テストはいつも赤いもの」になって [1]-[5] の赤に気付けなくなる。
//       config を詰める側の作業では --strict を付けて落とす。
//   [7] IMU の姿勢推定 (imu_attitude.hpp)。合成したサンプルで、光学座標系からの
//       載せ替え・傾きと角速度の符号・歩行中の揺れへの強さ・零点合わせを見る
//   [8] 安定化 (stabilizer.hpp)。**補正の向き**を順運動学で確かめる。符号を
//       落とすと安定化が倒れる向きに効くので、実機に出す前にここで止める
//   [9] 静歩行 (walk_mode:=static)。静歩行の門が食い違いを拾うこと、状態機械が
//       静歩行の計画で歩いて止まること、振り出し中の重心が門の言うとおりの位置に
//       あること、安定化が SWING を片足支持として扱うこと。static_gait.yaml そのものの
//       門の結果は [6] と同じく --strict のときだけ落とす
//
// 落ちたら戻り値 1。config を書き換えたあとに 1 回通しておくところ。
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "roboone_motion/imu_attitude.hpp"
#include "roboone_motion/motion_config.hpp"
#include "roboone_motion/motion_control.hpp"
#include "roboone_motion/motion_library.hpp"
#include "roboone_motion/pose_codec.hpp"
#include "roboone_motion/stabilizer.hpp"

namespace rm = roboone_motion;
namespace rk = roboone_kinematics;

namespace
{

constexpr double kR2D = 180.0 / M_PI;
int g_fail = 0;

void check(bool ok, const std::string & what)
{
  std::printf("  %s %s\n", ok ? "ok  " : "★NG", what.c_str());
  if (!ok) {++g_fail;}
}

std::string fmt(const char * f, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}

/// [4] で使う一時ファイル。実機の motions.yaml を汚さずに書式だけを試す。
const char * kAngleYaml =
  "motions:\n"
  "  angle_only:\n"
  "    return_home: false\n"
  "    keyframes:\n"
  "      - t: 0.20\n"
  "        R_leg: {ID1: -10.0, ID2: 0.0, ID3: 0.0, ID4: 30.0, ID6: 0.0, ID5: 0.0}\n"
  "        L_leg: {ID1:  10.0, ID2: 0.0, ID3: 0.0, ID4: 30.0, ID6: 0.0, ID5: 0.0}\n"
  "  mixed:\n"
  "    return_home: false\n"
  "    keyframes:\n"
  "      - t: 0.20\n"
  "        R_foot: {p: [-20.0, -89.3, -261.0], rpy: [0, 0, 0]}\n"
  "        L_foot: {p: [-20.0,  89.3, -261.0], rpy: [0, 0, 0]}\n"
  "      - t: 0.20\n"
  "        R_leg: {ID4: 40.0}\n"
  "      - t: 0.20\n"
  "        R_foot: {p: [-20.0, -89.3, -261.0], rpy: [0, 0, 0]}\n"
  "  unknown_axis:\n"
  "    keyframes:\n"
  "      - t: 0.20\n"
  "        R_leg: {ID9: 10.0}\n"
  "  no_base:\n"
  "    return_home: false\n"
  "    keyframes:\n"
  "      - t: 0.20\n"
  "        R_foot: {p: [0.0, -89.3, -600.0], rpy: [0, 0, 0]}\n"
  "      - t: 0.20\n"
  "        R_leg: {ID4: 40.0}\n";

/// [7] 機体の姿勢と角速度から {O} の IMU サンプルを作って推定器へ流す。
struct ImuSim
{
  rm::ImuAttitude f;
  //! 本当の取り付け。実機は 0° だが、自明でない回転でも戻せるかを見るため 30° 下向きで作る
  double mount[3]{0.0, 30.0 / kR2D, 0.0};
  double t = 0.0;
  double roll = 0.0, pitch = 0.0;           //!< 本当の姿勢 [rad]

  ImuSim()
  {
    rm::ImuOptions o;
    for (int k = 0; k < 3; ++k) {o.mount_rpy[k] = mount[k];}
    f.configure(o);
  }

  /// 角速度 (Σ_B) で本当の姿勢を進めながら 1 サンプル流す。acc_w は世界座標の加速度。
  /// ff を立てると、その水平成分を計画上の加速度として推定器にも教える。
  void step(double wx, double wy, const rk::Vec3 & acc_w = rk::Vec3{}, bool ff = false)
  {
    f.setAccelFeedforward(ff ? acc_w.x : 0.0, ff ? acc_w.y : 0.0);
    const double dt = 0.005;
    // 小さい傾きの範囲なので、ロール・ピッチの速さ ≒ Σ_B の角速度でよい
    roll += wx * dt;
    pitch += wy * dt;
    const double rpy[3] = {roll, pitch, 0.0};
    const rk::Mat3 r_wb = rm::matFromRpy(rpy);
    const rk::Vec3 f_b = r_wb.mulT(acc_w + rk::Vec3{0.0, 0.0, rm::ImuAttitude::kGravity});
    const rk::Vec3 wo = rm::ImuAttitude::opticalFromBody(mount, rk::Vec3{wx, wy, 0.0});
    const rk::Vec3 fo = rm::ImuAttitude::opticalFromBody(mount, f_b);
    const double g[3] = {wo.x, wo.y, wo.z};
    const double a[3] = {fo.x, fo.y, fo.z};
    t += dt;
    f.update(t, g, a);
  }
  void hold(double sec) {for (int i = 0; i < static_cast<int>(sec / 0.005); ++i) {step(0.0, 0.0);}}
};

/// [8] 胴体から見た足裏の小回転 D = R1 · R0^T の (roll, pitch) 成分。
void footRotation(const rk::Mat3 & r1, const rk::Mat3 & r0, double & roll, double & pitch)
{
  double d[3][3]{};
  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      for (int k = 0; k < 3; ++k) {
        d[a][b] += r1(a, k) * r0(b, k);
      }
    }
  }
  roll = std::atan2(d[2][1] - d[1][2], d[1][1] + d[2][2]);
  pitch = std::atan2(d[0][2] - d[2][0], d[0][0] + d[2][2]);
}

/// decode() が出すのと同じ形の実測姿勢（本体 = サーボ角、足裏は影）。
rm::BodyPose asMeasured(const rm::ServoMap & map, const rm::BodyPose & p)
{
  rm::BodyPose m = p;
  for (int s = 0; s < rm::kNumSide; ++s) {
    rm::fillLegServoFromFoot(map.leg_params(s), m, s);
    m.leg_mode[s] = rm::LegMode::Servo;
  }
  return m;
}

/// 生カウントを「全軸読めた」ServoState の列にする（decode() の入力）。
std::vector<feetech_servo::ServoState> statesFromCounts(const std::vector<int16_t> & counts)
{
  std::vector<feetech_servo::ServoState> st(counts.size());
  for (std::size_t k = 0; k < counts.size(); ++k) {
    st[k].pos = counts[k];
    st[k].valid = true;
  }
  return st;
}

/// 立位に近い、左右対称な足裏の目標（点検の土台）。
rm::BodyPose stancePose(std::size_t num_arm)
{
  rm::BodyPose p;
  p.arm.assign(num_arm, 0.0);
  for (int s = 0; s < rm::kNumSide; ++s) {
    const double lat = (s == rm::kLeft) ? +1.0 : -1.0;
    p.foot[s].p = rk::Vec3{-20.0, lat * 89.3, -261.0};
  }
  return p;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string home_path, limits_path, motions_path, gait_path, home_pose_path, static_gait_path;
  bool strict = false;   //!< [6] の門のエラーも失敗として扱う
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char * what) -> std::string {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "%s の値が無い\n", what);
          std::exit(2);
        }
        return argv[++i];
      };
    if (a == "--home") {
      home_path = next("--home");
    } else if (a == "--limits") {
      limits_path = next("--limits");
    } else if (a == "--motions") {
      motions_path = next("--motions");
    } else if (a == "--gait") {
      gait_path = next("--gait");
    } else if (a == "--home-pose") {
      home_pose_path = next("--home-pose");
    } else if (a == "--static-gait") {
      static_gait_path = next("--static-gait");
    } else if (a == "--strict") {
      strict = true;
    } else {
      std::fprintf(
        stderr,
        "使い方: motion_selftest [--home servo_home.yaml] [--limits servo_limits.yaml]"
        " [--motions motions.yaml] [--gait gait.yaml] [--home-pose home_pose.yaml]"
        " [--static-gait static_gait.yaml] [--strict]\n"
        "  --strict  [6] 設定の門のエラーも失敗にする (config を詰めるときはこちら)\n");
      return 2;
    }
  }
  try {
    const std::string fs = ament_index_cpp::get_package_share_directory("feetech_servo");
    if (home_path.empty()) {home_path = fs + "/config/servo_home.yaml";}
    if (limits_path.empty()) {limits_path = fs + "/config/servo_limits.yaml";}
    if (motions_path.empty()) {
      motions_path =
        ament_index_cpp::get_package_share_directory("roboone_motion") +
        "/config/motions.yaml";
    }
    // gait.yaml / home_pose.yaml の原本は roboone_walk_ref。
    if (gait_path.empty() || home_pose_path.empty() || static_gait_path.empty()) {
      std::string ref;
      try {
        ref = ament_index_cpp::get_package_share_directory("roboone_walk_ref");
      } catch (const std::exception &) {
        ref = ament_index_cpp::get_package_share_directory("roboone_motion");
      }
      if (gait_path.empty()) {gait_path = ref + "/config/gait.yaml";}
      if (home_pose_path.empty()) {home_pose_path = ref + "/config/home_pose.yaml";}
      if (static_gait_path.empty()) {static_gait_path = ref + "/config/static_gait.yaml";}
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "share が見つからない: %s\n", e.what());
    return 2;
  }

  rm::ServoMap map;
  std::string err;
  // ポート名は原点ファイルの中の見出しとしてしか使わない（バスは開かない）。
  if (!map.load(home_path, limits_path, "/dev/feetech_right", "/dev/feetech_left", {}, err)) {
    std::fprintf(stderr, "%s\n", err.c_str());
    return 1;
  }

  // ---- [1] 角の 3 つの表し方が一巡するか -------------------------------
  std::printf("[1] T ポーズ基準 deg <-> 絶対サーボ角 rad <-> 生カウント\n");
  {
    double worst_deg = 0.0, worst_cnt = 0.0;
    for (int s = 0; s < rm::kNumSide; ++s) {
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
        for (double deg : {-40.0, -5.0, 0.0, 12.5, 60.0}) {
          const double back =
            map.leg_tpose_deg_from_servo(s, j, map.leg_servo_from_tpose_deg(s, j, deg));
          worst_deg = std::max(worst_deg, std::abs(back - deg));
        }
        for (int cnt : {1500, 2048, 2600}) {
          const double a = map.leg_tpose_deg_from_count(s, j, cnt);
          const double b = map.leg_tpose_deg_from_servo(s, j, map.leg_servo_from_count(s, j, cnt));
          worst_cnt = std::max(worst_cnt, std::abs(a - b));
        }
      }
    }
    check(worst_deg < 1e-9, fmt("deg -> rad -> deg の往復 (最大 %.3g deg)", worst_deg));
    check(worst_cnt < 1e-9, fmt("カウント経由でも同じ deg (最大 %.3g deg)", worst_cnt));
  }

  // ---- [2] 胴体の前傾: 足裏を回す vs 股ピッチのサーボ角に足す ------------
  std::printf("[2] body_pitch の 2 通りの掛け方が一致するか\n");
  {
    const rm::BodyPose ref = stancePose(map.num_arm());
    for (double psi_deg : {5.0, -7.5, 12.0}) {
      const double psi = psi_deg / kR2D;
      for (int s = 0; s < rm::kNumSide; ++s) {
        double sv_u[rk::kNumJoints], th[rk::kNumJoints];
        const rm::LegSolve r0 = rm::servoFromFootPose(map.leg_params(s), ref.foot[s], sv_u, th);
        rm::FootPose fb = ref.foot[s];
        rm::bodyPitchApply(fb, psi);
        double sv_b[rk::kNumJoints], th2[rk::kNumJoints];
        const rm::LegSolve r1 = rm::servoFromFootPose(map.leg_params(s), fb, sv_b, th2);
        if (!r0.ok() || !r1.ok()) {
          check(false, fmt("%s脚 psi=%+.1f: 点検に使う立位が解けない", rm::kSideTag[s], psi_deg));
          continue;
        }
        rm::bodyPitchApplyServo(map.leg_params(s), sv_u, psi);
        double worst = 0.0;
        std::size_t worst_j = 0;
        for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
          const double d = std::abs(sv_u[j] - sv_b[j]) * kR2D;
          if (d > worst) {
            worst = d;
            worst_j = j;
          }
        }
        check(
          worst < 1e-6,
          fmt(
            "%s脚 psi=%+.1fdeg: 最大差 %.2g deg (ID%d)", rm::kSideTag[s], psi_deg, worst,
            rm::kLegServoId[worst_j]));
      }
    }
  }

  // ---- [3] 実機の motions.yaml -----------------------------------------
  std::printf("[3] %s\n", motions_path.c_str());
  {
    rm::MotionLibrary lib;
    if (!lib.load(motions_path, map, err)) {
      check(false, err);
    } else {
      check(true, lib.summary());
      for (const auto & w : lib.warnings()) {
        std::printf("      警告: %s\n", w.c_str());
      }
    }
  }

  // ---- [4] 脚の角度書き -------------------------------------------------
  std::printf("[4] 脚の角度書き (R_leg / L_leg)\n");
  {
    const std::string tmp = "/tmp/roboone_motion_selftest.yaml";
    std::FILE * fp = std::fopen(tmp.c_str(), "w");
    if (!fp) {
      check(false, "一時ファイルを作れない");
      return 1;
    }
    std::fputs(kAngleYaml, fp);
    std::fclose(fp);

    rm::MotionLibrary lib;
    if (!lib.load(tmp, map, err)) {
      check(false, "角度書きの config を読めない: " + err);
      return 1;
    }
    check(lib.find("angle_only") != nullptr, "角度書きだけの技を読めた");
    check(lib.warnings().size() == 1, "脚に無い軸 (ID9) は警告に留めて捨てる");

    const rm::BodyPose from = stancePose(map.num_arm());

    // [4-1] 角度書きだけ: 書いた値がそのまま終端になる（IK を通らない）
    {
      rm::MotionPlayer pl;
      pl.start(*lib.find("angle_only"), from, from, 0.0, map);
      check(pl.warning().empty(), "警告なしで仕込めた");
      rm::BodyPose out;
      pl.sample(1.0, out);        // 再生し切ったので out は最終姿勢
      const double d4 = map.leg_tpose_deg_from_servo(
        rm::kRight, rk::KNEE, out.leg_servo[rm::kRight][rk::KNEE]);
      check(std::abs(d4 - 30.0) < 1e-6, fmt("R 膝 ID4 が書いたとおり 30 deg (%.4f)", d4));
      const double d1 = map.leg_tpose_deg_from_servo(
        rm::kLeft, rk::HIP_PITCH, out.leg_servo[rm::kLeft][rk::HIP_PITCH]);
      check(std::abs(d1 - 10.0) < 1e-6, fmt("L 股 ID1 が書いたとおり 10 deg (%.4f)", d1));
    }

    // [4-2] 混在: 足裏 -> 角度(膝だけ) -> 足裏
    {
      rm::MotionPlayer pl;
      pl.start(*lib.find("mixed"), from, from, 0.0, map);
      check(pl.warning().empty(), "書き方が混ざっていても警告なし");
      rm::BodyPose out;

      pl.sample(0.10, out);       // 1 枚目の区間: 足裏 -> 足裏
      check(
        out.leg_mode[rm::kRight] == rm::LegMode::Foot,
        "足裏 -> 足裏 の区間は今までどおり足裏空間");

      pl.sample(0.30, out);       // 2 枚目の区間: 足裏 -> 角度
      check(out.leg_mode[rm::kRight] == rm::LegMode::Servo, "足裏 -> 角度 の区間は角度空間");
      check(out.leg_mode[rm::kLeft] == rm::LegMode::Foot, "触っていない左脚は足裏書きのまま");

      pl.sample(0.50, out);       // 3 枚目の区間: 角度 -> 足裏
      check(out.leg_mode[rm::kRight] == rm::LegMode::Servo, "角度 -> 足裏 の区間も角度空間");

      pl.sample(0.3999, out);     // 2 枚目の終端
      const double d4 = map.leg_tpose_deg_from_servo(
        rm::kRight, rk::KNEE, out.leg_servo[rm::kRight][rk::KNEE]);
      check(std::abs(d4 - 40.0) < 1e-3, fmt("書いた ID4 は 40 deg に届く (%.4f)", d4));

      // 書かなかった軸は「ひとつ前の足裏姿勢を IK で直した値」を引き継ぐ
      double sv[rk::kNumJoints], th[rk::kNumJoints];
      rm::servoFromFootPose(map.leg_params(rm::kRight), from.foot[rm::kRight], sv, th);
      double worst = 0.0;
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
        if (j == rk::KNEE) {continue;}
        worst = std::max(worst, std::abs(out.leg_servo[rm::kRight][j] - sv[j]) * kR2D);
      }
      check(worst < 1e-6, fmt("書かなかった軸は足裏姿勢の IK 値のまま (最大差 %.2g deg)", worst));

      pl.sample(1.0, out);
      check(out.leg_mode[rm::kRight] == rm::LegMode::Foot, "足裏書きの枚で書き方が戻る");
      check(
        std::abs(out.foot[rm::kRight].p.z + 261.0) < 1e-6,
        fmt("戻り先の足裏 z = %.3f mm", out.foot[rm::kRight].p.z));
    }
    // [4-3] 引き継ぐ土台が IK で解けない: 指令を出さない側へ倒す
    {
      rm::MotionPlayer pl;
      pl.start(*lib.find("no_base"), from, from, 0.0, map);
      check(!pl.warning().empty(), "土台が解けない枚は警告を出す");
      rm::BodyPose out;
      pl.sample(0.30, out);     // 足裏(解けない) -> 角度(膝だけ) の区間
      check(
        out.leg_mode[rm::kRight] == rm::LegMode::Servo &&
        !out.leg_servo_valid[rm::kRight],
        "揃っていない側は「指令を出さない」に倒す (古い足裏を IK で送らない)");
    }
    std::remove(tmp.c_str());
  }

  // =======================================================================
  // [6] 設定の門 — gait.yaml / home_pose.yaml を実機なしで通す
  // =======================================================================
  rm::rwc::GaitParams gait;
  rm::BodyPose home;
  double body_pitch = 0.0;
  bool home_ok = false;
  {
    std::printf("\n[6] 設定の門 (gait.yaml / home_pose.yaml)\n");
    rm::EventQueue ev;
    rm::loadGait(gait_path, gait, ev);
    rm::checkGait(gait, ev);
    std::string e6;
    home_ok = rm::loadHomePose(home_pose_path, map, gait, home, body_pitch, ev, e6);
    if (!home_ok) {
      check(false, "ホーム姿勢を読めない: " + e6);
    } else {
      rm::checkPoseReachable(map, home, "ホーム姿勢", ev);
      rm::checkStance(gait, home, ev);
      rm::checkWalkEnvelope(map, gait, home, body_pitch, ev);
    }
    // Error が 1 つでも出たら落とす。**実機を起こす前にここで止める**のが目的。
    int nerr = 0;
    rm::Event e;
    while (ev.pop(e)) {
      const char * mark = e.level == rm::EventLevel::Error ? "  ★ " :
        (e.level == rm::EventLevel::Warn ? "  ! " : "    ");
      std::printf("%s%s\n", mark, e.text.c_str());
      if (e.level == rm::EventLevel::Error) {++nerr;}
    }
    if (strict) {
      check(nerr == 0, fmt("門が出したエラー %d 件", nerr));
    } else if (nerr > 0) {
      std::printf(
        "  -- 門のエラー %d 件。**この実行では失敗にしない** "
        "(config の話なので --strict を付けたときだけ落とす)\n", nerr);
    } else {
      check(true, "門はエラーなし");
    }
  }

  // =======================================================================
  // [5] 状態機械の順序 — motion_control.hpp「順序の約束」の 4 つ + 脱力の取りこぼし
  // =======================================================================
  rm::MotionLibrary lib5;
  {
    std::string e5;
    if (!lib5.load(motions_path, map, e5)) {
      check(false, "状態機械の検算に使う motions.yaml を読めない: " + e5);
      home_ok = false;
    }
  }
  if (home_ok) {
    std::printf("\n[5] 状態機械の順序\n");
    const double dt = 1.0 / 200.0;
    rm::MotionController::Options copt;   // 既定のまま (require_home_before_arm = true)

    // 実測姿勢として使う「取れている姿勢」。ホーム姿勢のサーボ角（必ず IK で解ける）。
    const rm::BodyPose meas = asMeasured(map, home);
    const std::string why = "(テスト)";

    auto make = [&]() {
        auto c = std::make_unique<rm::MotionController>();
        c->configure(&map, &lib5, gait, home, body_pitch, copt);
        return c;
      };
    auto drop = [](rm::MotionController & c) {
        rm::Event e;
        while (c.popEvent(e)) {}
      };

    // [5-1] 技の要求を武装判定より **先** に捌く。
    //   teleop は home -> (0.1s) -> /estop false の順に来る。逆順に捌くと、
    //   その周期はまだ seen_motion_ が false なので武装に入れず 1 本落とす。
    {
      auto c = make();
      c->setEstop(false);
      auto t = c->step(0.0, dt, &meas, why, /*torque_ready=*/true);
      check(
        t.state == rm::State::RELAX && !t.want_torque,
        "/cmd_motion を受けるまでは /estop false でも武装しない");
      c->requestMotion("home");
      t = c->step(dt, dt, &meas, why, true);
      check(
        t.state == rm::State::ARMING,
        "home を受けた **その周期で** 武装に入る (要求を武装判定より先に捌く)");
      drop(*c);
    }

    // [5-2] その周期の実測姿勢が取れないうちは武装しない。
    //   起点が無いまま始めると補間にならず、「トルクが入るだけで動かない」
    //   (2026-08-28 実機)。代用すると今度は保持姿勢へ一気に飛ぶ。
    {
      auto c = make();
      c->setEstop(false);
      c->requestMotion("home");
      auto t = c->step(0.0, dt, nullptr, why, true);
      check(
        t.state == rm::State::RELAX && !t.want_torque,
        "実測姿勢が取れないうちは武装せず、トルクも要求しない");
      t = c->step(dt, dt, &meas, why, true);
      check(t.state == rm::State::ARMING, "実測姿勢が取れたら武装する");
      drop(*c);
    }
    //   脱力中に 1 度取れても、武装する周期に取れていなければ武装しない。
    //   2026-09-18 までは「1 度取れたら以後ずっと取れた扱い」で、手で足首を動かして
    //   読めなくなった後も、古い実測を起点にして武装していた。
    {
      auto c = make();
      c->setEstop(true);
      c->requestMotion("home");
      auto t = c->step(0.0, dt, &meas, why, true);
      c->setEstop(false);
      t = c->step(dt, dt, nullptr, why, true);
      check(
        t.state == rm::State::RELAX && !t.want_torque,
        "★前に取れた実測があっても、その周期に取れなければ武装しない");
      drop(*c);
    }

    // [5-3] 脱力の取りこぼし。RELAX のまま武装待ち (want_torque = true だが
    //   torque_ready がまだ) の最中に /estop true が来たら、トルクの要求を必ず下ろす。
    {
      auto c = make();
      c->setEstop(false);
      c->requestMotion("home");
      auto t = c->step(0.0, dt, &meas, why, /*torque_ready=*/false);
      check(
        t.state == rm::State::RELAX && t.want_torque,
        "武装待ち: RELAX のままトルクを要求する");
      c->setEstop(true);
      t = c->step(dt, dt, &meas, why, false);
      check(
        !t.want_torque,
        "★武装待ちの最中に /estop true が来たらトルクの要求を下ろす");
      drop(*c);
    }

    // [5-4] その場保持 (hold) の武装後は HOLD ではなく STAY。
    //   HOLD にすると tickWalk が足先を立位のスタンスへ上書きするので、寝た姿勢
    //   からだと跳ねる (転倒 -> 脱力 -> その場保持 -> 起き上がり の経路)。
    {
      auto c = make();
      // teleop は起動時に /estop true を撒く。hold もその状態で来る
      // (hold -> hold_torque_delay -> /estop false の 2 段)。
      c->setEstop(true);
      c->requestMotion(rm::kHoldMotion);     // 脱力中は「予約」
      auto t = c->step(0.0, dt, &meas, why, true);
      check(
        t.state == rm::State::RELAX && !t.want_torque,
        "hold は脱力中なら予約だけ (トルクも要求しない)");
      c->setEstop(false);
      t = c->step(dt, dt, &meas, why, true);
      check(t.state == rm::State::ARMING, "その場保持で武装に入る");
      double now = dt;
      for (int i = 0; i < 400 && t.state == rm::State::ARMING; ++i) {
        now += dt;
        t = c->step(now, dt, &meas, why, true);
      }
      check(t.state == rm::State::STAY, "その場保持の武装が終わったら HOLD ではなく STAY");
      // STAY は /cmd_walk を受けない (寝ている間に歩き出さない)
      c->setWalkCmd(0.05, 0.0, 0.0, now);
      for (int i = 0; i < 40; ++i) {
        now += dt;
        t = c->step(now, dt, &meas, why, true);
      }
      check(t.state == rm::State::STAY, "STAY 中は /cmd_walk で歩き出さない");
      drop(*c);
    }

    // [5-5] その場保持の武装中に home が来たら、終わりは STAY ではなく HOLD。
    //   stay_after_arm_ が立ったまま home の補間に入ると、終わった時点で STAY に
    //   落ちて立位のまま固まる (以後 /cmd_walk が効かず歩けない)。
    {
      auto c = make();
      c->setEstop(true);
      c->requestMotion(rm::kHoldMotion);
      auto t = c->step(0.0, dt, &meas, why, true);
      c->setEstop(false);
      double now = dt;
      t = c->step(now, dt, &meas, why, true);
      check(t.state == rm::State::ARMING, "その場保持で武装に入る");
      c->requestMotion("home");              // 武装が終わる前に home
      for (int i = 0; i < 800 && t.state != rm::State::HOLD; ++i) {
        now += dt;
        t = c->step(now, dt, &meas, why, true);
        if (t.state == rm::State::STAY) {break;}
      }
      check(
        t.state == rm::State::HOLD,
        "★その場保持の武装中に home が来たら、終わりは STAY ではなく HOLD");
      drop(*c);
    }

    // [5-6] WALK -> HOLD のばたつき止め。
    //   歩き始めは指令がレート制限で立ち上がるので、歩行エンジンが IDLE と START の
    //   間を数十 ms 単位で往復する (2026-08-28 実機で 55ms 周期)。**状態が何度も
    //   変わらないこと**が要点なので、遷移の回数を数える。
    {
      auto c = make();
      c->setEstop(false);
      c->requestMotion("home");
      double now = 0.0;
      auto t = c->step(now, dt, &meas, why, true);
      for (int i = 0; i < 800 && t.state != rm::State::HOLD; ++i) {
        now += dt;
        t = c->step(now, dt, &meas, why, true);
      }
      check(t.state == rm::State::HOLD, "ホームへの補間が終わって HOLD");

      int changes = 0;
      bool saw_walk = false;
      // 2 秒歩かせる
      for (int i = 0; i < 400; ++i) {
        now += dt;
        c->setWalkCmd(0.05, 0.0, 0.0, now);
        t = c->step(now, dt, &meas, why, true);
        changes += t.state_changed ? 1 : 0;
        saw_walk = saw_walk || t.state == rm::State::WALK;
      }
      // 指令を止めて 3 秒。停止シーケンス -> IDLE -> (walk_idle_hold) -> HOLD
      for (int i = 0; i < 600; ++i) {
        now += dt;
        c->setWalkCmd(0.0, 0.0, 0.0, now);
        t = c->step(now, dt, &meas, why, true);
        changes += t.state_changed ? 1 : 0;
      }
      check(saw_walk, "/cmd_walk で WALK に入る");
      check(t.state == rm::State::HOLD, "指令を止めたら HOLD に戻る");
      check(changes == 2, fmt("状態が変わったのは HOLD->WALK->HOLD の 2 回だけ (%d 回)", changes));
      drop(*c);
    }

    // [5-8] 歩行と HOLD の足はホーム姿勢の足に揃う。
    //   2026-09-18 までは歩行の立位を stance_y_offset で別に持っていて、ホーム姿勢
    //   (±89.3) と食い違い、home の補間が終わって HOLD に入った次の周期に足が
    //   19.3mm 跳んでいた（[5-7] は HOLD に入ったその周期しか見ていない）。
    //   計画上の足間隔からも前後の原点からもずらしたホーム姿勢で確かめる。
    {
      rm::BodyPose home8 = home;
      for (int s = 0; s < rm::kNumSide; ++s) {
        const double lat = (s == rm::kLeft) ? +1.0 : -1.0;
        home8.foot[s].p.x = -10.0;
        home8.foot[s].p.y = lat * (gait.foot_spacing * 500.0 - 12.0);
      }
      const rm::BodyPose meas8 = asMeasured(map, home8);
      rm::MotionController c;
      c.configure(&map, &lib5, gait, home8, body_pitch, copt);
      auto dev = [&home8](const rm::BodyPose & p) {
          double d = 0.0;
          for (int s = 0; s < rm::kNumSide; ++s) {
            d = std::max(
              {d, std::abs(p.foot[s].p.x - home8.foot[s].p.x),
                std::abs(p.foot[s].p.y - home8.foot[s].p.y),
                std::abs(p.foot[s].p.z - home8.foot[s].p.z)});
          }
          return d;
        };
      c.setEstop(false);
      c.requestMotion("home");
      double now = 0.0;
      auto t = c.step(now, dt, &meas8, why, true);
      for (int i = 0; i < 800 && t.state != rm::State::HOLD; ++i) {
        now += dt;
        t = c.step(now, dt, &meas8, why, true);
      }
      double hold_dev = 0.0;
      for (int i = 0; i < 100; ++i) {
        now += dt;
        t = c.step(now, dt, &meas8, why, true);
        hold_dev = std::max(hold_dev, dev(c.currentPose()));
      }
      check(
        t.state == rm::State::HOLD && hold_dev < 1e-6,
        fmt("★HOLD の足はホーム姿勢の足のまま (0.5s の最大ずれ %.2g mm)", hold_dev));

      // 歩いて止まると、ホーム姿勢の足へ戻る。残るのは計画の停止位置のずれ
      // （重心の静止判定 settle_eps と、最後の歩の着地補正）だけ
      for (int i = 0; i < 400; ++i) {
        now += dt;
        c.setWalkCmd(0.05, 0.0, 0.0, now);
        t = c.step(now, dt, &meas8, why, true);
      }
      for (int i = 0; i < 800; ++i) {
        now += dt;
        c.setWalkCmd(0.0, 0.0, 0.0, now);
        t = c.step(now, dt, &meas8, why, true);
      }
      const double walk_dev = dev(c.currentPose());
      check(
        t.state == rm::State::HOLD && walk_dev < 3.0,
        fmt("歩いて止まった後の足もホーム姿勢の足 (ずれ %.2f mm)", walk_dev));
      drop(c);
    }

    // [5-7] 武装の起点はサーボ角。脱力して垂れた足首から跳ばずに立つ。
    //   2026-09-17 の bag: 起動直後の脱力で両脚とも足首クランク ≈ (+94.5°, +94.0°)。
    //   CRANK_LIMIT_DEG の箱の外なので、FK で足裏を出す旧方式では武装しなかった
    //   （毎回手で戻していた）。ロッドが死点を越えた姿勢で、FK の足裏を IK で戻すと
    //   別のクランク角になる = 足裏で補間すると初周期に跳ぶ。
    {
      rm::PoseCodec codec;
      codec.configure(&map, body_pitch);
      const rm::PoseCodec::Encoded eh = codec.encode(home);
      std::vector<int16_t> cnt[rm::kNumSide];
      std::vector<feetech_servo::ServoState> st[rm::kNumSide];
      const double q_droop[2] = {94.5 / kR2D, 94.0 / kR2D};
      for (int s = 0; s < rm::kNumSide; ++s) {
        const rk::AnkleParams & ap = map.leg_params(s).ankle;
        cnt[s] = eh.counts[s];
        cnt[s][rk::ANKLE_PITCH] = static_cast<int16_t>(
          map.leg_count_from_servo(s, rk::ANKLE_PITCH, rk::ankleServoFromCrank(ap, 0, q_droop[0])));
        cnt[s][rk::ANKLE_ROLL] = static_cast<int16_t>(
          map.leg_count_from_servo(s, rk::ANKLE_ROLL, rk::ankleServoFromCrank(ap, 1, q_droop[1])));
        st[s] = statesFromCounts(cnt[s]);
      }
      const rm::PoseCodec::Decoded d = codec.decode(st);
      check(
        d.status[rm::kRight] == rk::LegServoStatus::AnkleClamped &&
        d.status[rm::kLeft] == rk::LegServoStatus::AnkleClamped,
        "垂れた足首は FK の箱の外 (AnkleClamped。旧方式ではここで武装しなかった)");
      check(d.ok, d.ok ? "それでも武装の起点としては使える" : "武装の起点として使えない: " + d.why);
      bool servo_body = true;
      for (int s = 0; s < rm::kNumSide; ++s) {
        servo_body = servo_body && d.pose.leg_mode[s] == rm::LegMode::Servo &&
          d.pose.leg_servo_valid[s];
      }
      check(servo_body, "実測姿勢の本体はサーボ角");

      // 逆変換で戻すと別のクランク角になる（足裏で補間しない理由）
      {
        const rk::AnkleParams & ap = map.leg_params(rm::kRight).ankle;
        rk::AnkleParams wide = ap;
        for (int i = 0; i < rk::kAnkleChains; ++i) {
          wide.qMin[i] = rk::ankle_config::ARM_CRANK_LIMIT_DEG[0] / kR2D;
          wide.qMax[i] = rk::ankle_config::ARM_CRANK_LIMIT_DEG[1] / kR2D;
        }
        const rk::AnkleFkResult f = rk::ankleFk(wide, q_droop, 0.0);
        const rk::AnkleIkResult b = rk::ankleIk(ap, f.th5, f.th6, false);
        check(
          f.status == rk::AnkleFkStatus::Ok &&
          std::max(std::abs(b.q[0] - q_droop[0]), std::abs(b.q[1] - q_droop[1])) * kR2D > 5.0,
          fmt(
            "垂れた足首 θ5 %.1f deg を逆変換で戻すとクランク (%.1f, %.1f) deg (実測 94.5, 94.0)",
            f.th5 * kR2D, b.q[0] * kR2D, b.q[1] * kR2D));
      }

      // decode -> encode で実測と同じカウントに戻る（前傾も往復する）
      {
        const rm::PoseCodec::Encoded e = codec.encode(d.pose);
        int worst = 0;
        for (int s = 0; s < rm::kNumSide; ++s) {
          for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
            worst = std::max(worst, std::abs(e.counts[s][j] - cnt[s][j]));
          }
        }
        check(
          e.send[rm::kRight] && e.send[rm::kLeft] && worst == 0,
          fmt("実測姿勢をそのまま指令にすると実測のカウントに戻る (最大ずれ %d)", worst));
      }

      // 武装: 初周期は実測のまま、補間中は滑らかに、終わりはホーム姿勢
      {
        auto c = make();
        c->setEstop(false);
        c->requestMotion("home");
        double now = 0.0;
        auto t = c->step(now, dt, &d.pose, why, true);
        check(t.state == rm::State::ARMING, "垂れた足首のまま武装に入る");
        std::vector<int16_t> prev[rm::kNumSide];
        int first = 0, step_max = 0;
        bool all_sent = true;
        for (int i = 0; i < 1000 && t.state == rm::State::ARMING; ++i) {
          const rm::PoseCodec::Encoded e = codec.encode(*t.target);
          for (int s = 0; s < rm::kNumSide; ++s) {
            all_sent = all_sent && e.send[s];
            if (!e.send[s]) {continue;}
            for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
              const int v = e.counts[s][j];
              if (i == 0) {first = std::max(first, std::abs(v - cnt[s][j]));}
              if (!prev[s].empty()) {step_max = std::max(step_max, std::abs(v - prev[s][j]));}
            }
            prev[s] = e.counts[s];
          }
          now += dt;
          t = c->step(now, dt, &d.pose, why, true);
        }
        check(first <= 1, fmt("補間の初周期は実測のカウント (最大ずれ %d)", first));
        check(all_sent, "補間中は毎周期両脚の指令が出る (IK で詰まらない)");
        // 足首 ~66 deg ≈ 750 カウントを 2 秒の 5 次補間で: 最大 ~3.5 カウント/周期
        check(step_max <= 15, fmt("補間中に跳ばない (1 周期の最大 %d カウント)", step_max));
        check(t.state == rm::State::HOLD, "補間が終わって HOLD");
        const rm::PoseCodec::Encoded e = codec.encode(c->currentPose());
        int to_home = 0;
        for (int s = 0; s < rm::kNumSide; ++s) {
          for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
            to_home = std::max(to_home, std::abs(e.counts[s][j] - eh.counts[s][j]));
          }
        }
        check(to_home <= 1, fmt("行き着く先はホーム姿勢 (最大ずれ %d カウント)", to_home));
        drop(*c);
      }

      // 起点として使えない実測: 巻き数ずれ / 武装の箱の外
      {
        std::vector<feetech_servo::ServoState> bad[rm::kNumSide] = {st[0], st[1]};
        bad[rm::kLeft][rk::ANKLE_PITCH].pos += 4096;
        const rm::PoseCodec::Decoded w = codec.decode(bad);
        check(
          !w.ok && w.why.find("巻き数") != std::string::npos,
          "カウントが 0-4095 の外なら使わない (多回転の巻き数ずれ): " + w.why);

        bad[rm::kLeft] = st[rm::kLeft];
        const rk::AnkleParams & ap = map.leg_params(rm::kLeft).ankle;
        for (int i = 0; i < rk::kAnkleChains; ++i) {
          const std::size_t j = (i == 0) ? rk::ANKLE_PITCH : rk::ANKLE_ROLL;
          bad[rm::kLeft][j].pos = map.leg_count_from_servo(
            rm::kLeft, j, rk::ankleServoFromCrank(ap, i, -60.0 / kR2D));
        }
        const rm::PoseCodec::Decoded o = codec.decode(bad);
        check(
          !o.ok && o.why.find("武装の箱") != std::string::npos,
          "両クランク -60 deg (型 2 特異点の側) は使わない: " + o.why);

        // decode の判定を素通りしても、状態機械の経路検査で止まる
        auto c = make();
        c->setEstop(false);
        c->requestMotion("home");
        const auto t = c->step(0.0, dt, &o.pose, why, true);
        check(
          t.state == rm::State::RELAX && !t.want_torque,
          "その実測を起点に渡されても、状態機械はトルクを要求しない");
        drop(*c);
      }
    }
  }


  // =======================================================================
  // [7] IMU の姿勢推定 — 合成したサンプルで符号と収束を見る
  // =======================================================================
  {
    std::printf("\n[7] IMU の姿勢推定 (imu_attitude.hpp)\n");
    {
      ImuSim sim;
      sim.hold(3.0);
      const rm::Attitude & a = sim.f.attitude();
      check(
        a.valid && std::abs(a.roll) * kR2D < 0.05 && std::abs(a.pitch) * kR2D < 0.05,
        fmt(
          "水平に立っていれば 0 (30° 下向きの取り付けを戻せている): roll %.3f / pitch %.3f deg",
          a.roll * kR2D, a.pitch * kR2D));
    }
    {
      ImuSim sim;
      sim.pitch = 10.0 / kR2D;
      sim.hold(2.0);
      const rm::Attitude & a = sim.f.attitude();
      check(
        std::abs(a.pitch * kR2D - 10.0) < 0.1 && std::abs(a.roll) * kR2D < 0.1,
        fmt("前へ 10° 倒すと pitch = +10: %.3f deg (roll %.3f)", a.pitch * kR2D, a.roll * kR2D));
    }
    {
      ImuSim sim;
      sim.roll = 8.0 / kR2D;
      sim.hold(2.0);
      const rm::Attitude & a = sim.f.attitude();
      check(
        std::abs(a.roll * kR2D - 8.0) < 0.1 && std::abs(a.pitch) * kR2D < 0.1,
        fmt("右へ 8° 倒すと roll = +8: %.3f deg (pitch %.3f)", a.roll * kR2D, a.pitch * kR2D));
    }
    {
      // 角速度の符号と積分。0.2s で 0.1 rad 前へ回す
      ImuSim sim;
      sim.hold(1.0);
      for (int i = 0; i < 40; ++i) {
        sim.step(0.0, 0.5);
      }
      const rm::Attitude & a = sim.f.attitude();
      check(
        std::abs(a.gyro[1] - 0.5) < 0.01 && std::abs(a.gyro[0]) < 0.01,
        fmt("前へ回っているとき gyro_y = +0.5: %.4f (gyro_x %.4f)", a.gyro[1], a.gyro[0]));
      check(
        std::abs(a.pitch - sim.pitch) * kR2D < 0.2,
        fmt("ジャイロで追う: 本当 %.2f / 推定 %.2f deg", sim.pitch * kR2D, a.pitch * kR2D));
      for (int i = 0; i < 40; ++i) {
        sim.step(0.4, -0.5);
      }
      check(
        std::abs(a.gyro[0] - 0.4) < 0.01 && std::abs(a.roll - sim.roll) * kR2D < 0.2,
        fmt(
          "右へ回っているとき gyro_x = +0.4: %.4f / roll 本当 %.2f 推定 %.2f deg",
          a.gyro[0], sim.roll * kR2D, a.roll * kR2D));
    }
    {
      // 歩行の揺れ: 横に ±2.6 m/s^2 の矩形（周期 2T = 1.2s。LIPM の横の重心加速度の形）。
      // 加速度だけで傾きを出すと 15° ずれる量。計画上の加速度を教えれば消える
      for (int use_ff = 0; use_ff < 2; ++use_ff) {
        ImuSim sim;
        sim.hold(1.0);
        double worst = 0.0;
        for (int i = 0; i < 2400; ++i) {
          const double ay = (std::fmod(i * 0.005, 1.2) < 0.6) ? 2.6 : -2.6;
          sim.step(0.0, 0.0, rk::Vec3{0.0, ay, 0.0}, use_ff != 0);
          worst = std::max(worst, std::abs(sim.f.attitude().roll) * kR2D);
        }
        if (use_ff) {
          check(
            worst < 0.05,
            fmt("横揺れ ±2.6 m/s^2 でも、計画上の加速度を引けば roll はずれない: 最大 %.3f deg",
            worst));
        } else {
          // 引かない場合の大きさを残しておく（設計の根拠。落とす条件ではない）
          std::printf(
            "    (参考) 計画上の加速度を引かないと roll が最大 %.2f deg ずれる\n", worst);
        }
      }
    }
    {
      // 零点: 取り付けを 25° と思い込ませた推定器を、本当は 30° の機体で水平に立たせる
      ImuSim sim;
      rm::ImuOptions o;
      o.mount_rpy[1] = 25.0 / kR2D;
      sim.f.configure(o);
      std::string msg;
      double m[3]{};
      sim.hold(0.5);
      check(!sim.f.zero(m, msg), "サンプルが足りないうちは零点を取らない: " + msg);
      sim.hold(2.5);
      const double before = sim.f.attitude().pitch * kR2D;
      const bool ok = sim.f.zero(m, msg);
      check(
        ok && std::abs(m[1] * kR2D - 30.0) < 0.05 && std::abs(m[0]) * kR2D < 0.05,
        fmt(
          "零点で取り付けを取り直す: 前 pitch %.2f deg -> mount [%.3f, %.3f, %.3f]",
          before, m[0] * kR2D, m[1] * kR2D, m[2] * kR2D));
      check(
        std::abs(sim.f.attitude().pitch) * kR2D < 0.05,
        fmt("零点の直後は 0: %.3f deg", sim.f.attitude().pitch * kR2D));
      // 0.35s で 10° 前へ回す
      for (int i = 0; i < 70; ++i) {
        sim.step(0.0, 0.5);
      }
      sim.hold(0.5);
      check(
        std::abs(sim.f.attitude().pitch - sim.pitch) * kR2D < 0.1,
        fmt(
          "零点のあと前へ倒すと同じだけ読む: 本当 %.3f / 推定 %.3f deg",
          sim.pitch * kR2D, sim.f.attitude().pitch * kR2D));
      for (int i = 0; i < 600; ++i) {
        sim.step(0.0, 0.3);
      }
      check(!sim.f.zero(m, msg), "動いている間は零点を取らない: " + msg);
      // 寝ている (大きく傾いた) まま零点を取ると取り付けが壊れる。断る
      ImuSim lying;
      lying.pitch = 60.0 / kR2D;
      lying.hold(3.0);
      const bool z = lying.f.zero(m, msg);
      check(
        !z && std::abs(lying.f.attitude().pitch * kR2D - 60.0) < 0.1,
        "60° 傾いたままでは零点を取らない: " + msg);
    }
  }

  // =======================================================================
  // [8] 安定化 — 補正の向きを順運動学で確かめる
  // =======================================================================
  if (home_ok) {
    std::printf("\n[8] 安定化 (stabilizer.hpp)\n");
    const double dt = 1.0 / 200.0;
    auto drain = [](rm::Stabilizer & st, bool print) {
        rm::Event e;
        int nwarn = 0;
        while (st.popEvent(e)) {
          if (print) {std::printf("    %s\n", e.text.c_str());}
          nwarn += (e.level != rm::EventLevel::Info) ? 1 : 0;
        }
        return nwarn;
      };
    rm::Stabilizer st;
    const bool cfg = st.configure(&map, home, body_pitch, gait);
    const int cfg_warn = drain(st, true);
    check(
      cfg && cfg_warn == 0 && st.legReady(rm::kRight) && st.legReady(rm::kLeft),
      "両脚の足首ヤコビアンが取れる");

    // (a) 足裏を胴体に対して回す向き。ホーム姿勢の関節角に補正を足して FK で見る
    for (int s = 0; s < rm::kNumSide; ++s) {
      const rk::LegServoParams & prm = map.leg_params(s);
      rm::FootPose f = home.foot[s];
      rm::bodyPitchApply(f, body_pitch);
      double th[rk::kNumJoints]{};
      if (rk::ik(prm.leg, f.p, rm::matFromRpy(f.rpy), th, false) != rk::IkStatus::Ok) {
        check(false, fmt("%s脚: ホーム姿勢が解けない", rm::kSideTag[s]));
        continue;
      }
      rk::Vec3 p0;
      rk::Mat3 r0;
      rk::fk(prm.leg, th, p0, r0);
      const double u = 0.05;
      for (int k = 0; k < 2; ++k) {
        const double ur = (k == 1) ? u : 0.0, up = (k == 0) ? u : 0.0;
        double off[2];
        st.ankleFromFootRotation(s, ur, up, off);
        double th2[rk::kNumJoints];
        std::copy(th, th + rk::kNumJoints, th2);
        th2[rk::ANKLE_PITCH] += off[0];
        th2[rk::ANKLE_ROLL] += off[1];
        rk::Vec3 p1;
        rk::Mat3 r1;
        rk::fk(prm.leg, th2, p1, r1);
        double gr, gp;
        footRotation(r1, r0, gr, gp);
        const bool close = std::abs(gr - ur) < 0.05 * u && std::abs(gp - up) < 0.05 * u;
        if (k == 0) {
          // 前へ倒れている -> 胴体から見てつま先を下げる（足裏の x 軸の z 成分が減る）
          check(
            close && r1(2, 0) < r0(2, 0),
            fmt(
              "%s脚: 前へ倒れているときはつま先を下げる (θ5 %+.4f θ6 %+.4f rad -> "
              "roll %+.4f pitch %+.4f)", rm::kSideTag[s], off[0], off[1], gr, gp));
        } else {
          // 右へ倒れている -> 胴体から見て足裏の左縁を上げる（y 軸の z 成分が増える）
          check(
            close && r1(2, 1) > r0(2, 1),
            fmt(
              "%s脚: 右へ倒れているときは左縁を上げる (θ5 %+.4f θ6 %+.4f rad -> "
              "roll %+.4f pitch %+.4f)", rm::kSideTag[s], off[0], off[1], gr, gp));
        }
      }
    }

    // (b) 変換層: 補正は IK の後の関節角に足される。解けない補正は外して出す
    {
      rm::PoseCodec codec;
      codec.configure(&map, body_pitch);
      const rm::PoseCodec::Encoded e0 = codec.encode(home);
      rm::PoseCorrection c;
      c.ankle[rm::kRight][0] = 0.03;
      c.ankle[rm::kLeft][1] = -0.02;
      const rm::PoseCodec::Encoded e1 = codec.encode(home, &c);
      const double d_r = e1.theta[rm::kRight][rk::ANKLE_PITCH] -
        e0.theta[rm::kRight][rk::ANKLE_PITCH];
      const double d_l = e1.theta[rm::kLeft][rk::ANKLE_ROLL] - e0.theta[rm::kLeft][rk::ANKLE_ROLL];
      const double d_hip = e1.theta[rm::kRight][rk::HIP_PITCH] -
        e0.theta[rm::kRight][rk::HIP_PITCH];
      check(
        e1.send[rm::kRight] && e1.send[rm::kLeft] && std::abs(d_r - 0.03) < 1e-12 &&
        std::abs(d_l + 0.02) < 1e-12 && std::abs(d_hip) < 1e-12 &&
        e1.counts[rm::kRight] != e0.counts[rm::kRight],
        fmt("足首の補正が関節角にそのまま乗る (R θ5 %+.4f / L θ6 %+.4f)", d_r, d_l));
      rm::PoseCorrection big;
      big.ankle[rm::kRight][0] = -1.4;   // θ5 がピッチの機構限界の外
      const rm::PoseCodec::Encoded e2 = codec.encode(home, &big);
      rm::Event ev;
      while (codec.popEvent(ev)) {}
      check(
        e2.send[rm::kRight] && e2.corr_dropped[rm::kRight] && !e2.corr_dropped[rm::kLeft] &&
        std::abs(e2.theta[rm::kRight][rk::ANKLE_PITCH] - e0.theta[rm::kRight][rk::ANKLE_PITCH]) <
        1e-12,
        fmt(
          "解けない補正 (-1.4 rad) は外して、補正なしの指令を出す (send %d dropped %d/%d)",
          e2.send[rm::kRight], e2.corr_dropped[rm::kRight], e2.corr_dropped[rm::kLeft]));
    }

    rm::Attitude att;
    att.valid = true;
    rm::Stabilizer::Input in;
    in.layer_active = true;
    in.att = &att;
    in.att_age = 0.005;
    auto run = [&](double sec) {
        for (int i = 0; i < static_cast<int>(sec / dt); ++i) {
          st.update(dt, in);
        }
        return st.correction();
      };

    // (c) ゲイン 0 なら何もしない
    {
      att.pitch = 0.1;
      att.gyro[1] = 0.5;
      st.setGains(rm::StabGains{});
      const rm::PoseCorrection c = run(1.0);
      check(c.zero(), "ゲインが 0 なら補正は 0");
    }

    // (d) 胴体の補正: 前へ倒れていたら前傾を減らす（胴体を起こす）向き
    {
      st.reset();
      rm::StabGains g;
      g.k_torso = 0.5;
      st.setGains(g);
      att = rm::Attitude{};
      att.valid = true;
      att.pitch = 0.1;
      const rm::PoseCorrection c = run(1.0);
      check(
        std::abs(c.body_pitch + 0.05) < 1e-9,
        fmt("前へ 0.1 rad 倒れていたら胴体の前傾を -0.05 rad (k_torso 0.5): %+.4f", c.body_pitch));
      // body_pitch を減らすことが胴体を起こすことになっているか: 足裏を胴体から見て
      // 前へ回す（= 股が伸びる）向きか
      rm::FootPose f = home.foot[rm::kRight];
      rm::FootPose f2 = f;
      rm::bodyPitchApply(f, body_pitch);
      rm::bodyPitchApply(f2, body_pitch + c.body_pitch);
      check(
        rm::matFromRpy(f2.rpy)(2, 0) < rm::matFromRpy(f.rpy)(2, 0),
        "胴体を起こす補正は、胴体から見て足裏のつま先を下げる向き (足首戦略と同じ向き)");
    }

    // (e) 入れ方: fade で立ち上がり、上限で止まる
    {
      st.reset();
      rm::StabGains g;
      g.kp_pitch = 0.5;
      st.setGains(g);
      att = rm::Attitude{};
      att.valid = true;
      att.pitch = 0.1;
      double full[2];
      st.ankleFromFootRotation(rm::kRight, 0.0, 0.05, full);
      const double a1 = std::abs(run(0.1).ankle[rm::kRight][0]);
      const double a2 = std::abs(run(0.9).ankle[rm::kRight][0]);
      check(
        a1 > 0.0 && a1 < 0.3 * std::abs(full[0]) && std::abs(a2 - std::abs(full[0])) < 1e-9,
        fmt("fade で立ち上がる: 0.1s %.4f -> 1.0s %.4f (目標 %.4f)", a1, a2, std::abs(full[0])));
      att.pitch = 1.0;
      const rm::PoseCorrection c = run(1.0);
      check(
        std::abs(std::abs(c.ankle[rm::kRight][0]) - g.ankle_clamp) < 1e-9,
        fmt("上限で止まる: %.4f (上限 %.3f)", c.ankle[rm::kRight][0], g.ankle_clamp));
      const double j0 = c.ankle[rm::kRight][0];
      att.pitch = -1.0;
      st.update(dt, in);
      const double j1 = st.correction().ankle[rm::kRight][0];
      check(
        std::abs(j1 - j0) <= g.rate_limit * dt + 1e-12,
        fmt("1 周期に動くのは rate_limit まで: %.4f rad", std::abs(j1 - j0)));
    }

    // (f) ジャイロの減衰の向き: 前へ回っている (gyro_y > 0) なら傾き (pitch > 0) と同じ向き
    {
      st.reset();
      rm::StabGains g;
      g.kd_pitch = 0.05;
      g.kd_roll = 0.05;
      st.setGains(g);
      att = rm::Attitude{};
      att.valid = true;
      att.gyro[0] = -0.4;
      att.gyro[1] = 0.4;
      run(1.0);
      check(
        std::abs(st.debug().u_pitch - 0.02) < 1e-12 && std::abs(st.debug().u_roll + 0.02) < 1e-12,
        fmt(
          "u = kd·ω: pitch %+.4f / roll %+.4f", st.debug().u_pitch, st.debug().u_roll));
    }

    // (g) 抜き方: IMU が古い / HOLD・WALK 以外 / 脱力
    {
      att.gyro[0] = 0.0;
      att.gyro[1] = 0.0;
      att.pitch = 0.1;
      rm::StabGains g;
      g.kp_pitch = 0.5;
      st.setGains(g);
      st.reset();
      run(1.0);
      in.att_age = 0.5;
      drain(st, false);
      const rm::PoseCorrection c1 = run(1.0);
      const int nw = drain(st, false);
      check(c1.zero() && nw > 0, "IMU が古くなったら補正を抜いて警告する");
      in.att_age = 0.005;
      run(1.0);
      in.layer_active = false;
      const double b0 = std::abs(st.correction().ankle[rm::kRight][0]);
      const double b1 = std::abs(run(0.2).ankle[rm::kRight][0]);
      const rm::PoseCorrection c2 = run(0.5);
      check(
        b0 > 0.0 && (b1 < b0) && (b1 > 0.0) && c2.zero(),
        fmt("HOLD / WALK 以外では fade で抜く (%.4f -> %.4f -> 0)", b0, b1));
      in.layer_active = true;
      run(1.0);
      in.relax = true;
      const rm::PoseCorrection c3 = run(dt);
      check(c3.zero(), "脱力したら即座に 0");
      in.relax = false;
    }

    // (h) 歩行中: 遊脚には効かせない。着地の前後はゲインを弱める
    {
      st.reset();
      rm::StabGains g;
      g.kp_pitch = 0.5;
      st.setGains(g);
      att = rm::Attitude{};
      att.valid = true;
      att.pitch = 0.1;
      rm::rwc::WalkOutputs w;
      w.state = rm::rwc::State::STEP;
      w.support = rm::rwc::LEFT;
      w.phase = 0.5;
      in.walk = &w;
      const rm::PoseCorrection c = run(1.0);
      check(
        c.ankleZero(rm::kRight) && !c.ankleZero(rm::kLeft),
        "左足支持の歩の中ほどでは、右 (遊脚) の足首に補正を出さない");
      w.phase = 0.01;
      st.update(dt, in);
      check(st.debug().weight[rm::kRight] > 0.9, "離地の直後は遊脚にもまだ効いている");
      w.phase = st.touchPhase();
      st.update(dt, in);
      const bool gate_td = st.debug().gate;
      const double u_td = st.debug().u_pitch;
      w.phase = 0.5;
      st.update(dt, in);
      check(
        gate_td && !st.debug().gate && std::abs(u_td - 0.5 * st.debug().u_pitch) < 1e-12,
        fmt("予定の着地 (位相 %.3f) の前後はゲインを半分にする", st.touchPhase()));
      w.phase = 0.999;
      st.update(dt, in);
      check(
        st.debug().weight[rm::kRight] > 0.95,
        "歩の終わりには遊脚にも戻っている (次の歩で支持脚になる)");
      w.state = rm::rwc::State::IDLE;
      w.support = 0;
      st.update(dt, in);
      check(
        st.debug().weight[rm::kRight] == 1.0 && st.debug().weight[rm::kLeft] == 1.0,
        "立っているときは両脚に効かせる");
      in.walk = nullptr;
    }

    // (i) 状態機械は HOLD / WALK の周期にだけ歩行計画の出力を出す
    {
      rm::MotionController::Options copt;
      auto c = std::make_unique<rm::MotionController>();
      c->configure(&map, &lib5, gait, home, body_pitch, copt);
      const std::string why = "(テスト)";
      const rm::BodyPose meas = asMeasured(map, home);
      c->setEstop(false);
      c->requestMotion("home");
      double now = 0.0;
      auto t = c->step(now, dt, &meas, why, true);
      for (int i = 0; i < 1000 && t.state != rm::State::HOLD; ++i) {
        now += dt;
        t = c->step(now, dt, &meas, why, true);
      }
      // HOLD に入ったその周期はまだ歩行計画を回していないので、もう 1 周期進める
      now += dt;
      t = c->step(now, dt, &meas, why, true);
      const bool in_hold = (t.state == rm::State::HOLD) && c->walkOutputs() != nullptr;
      c->requestMotion(rm::kHoldMotion);
      now += dt;
      t = c->step(now, dt, &meas, why, true);
      check(
        in_hold && t.state == rm::State::STAY && c->walkOutputs() == nullptr,
        "歩行計画の出力は HOLD で出て、STAY では出ない");
    }
  }

  // =======================================================================
  // [9] 静歩行 (walk_mode:=static)
  // =======================================================================
  if (home_ok) {
    std::printf("\n[9] 静歩行 (walk_planner.hpp / static_gait.yaml)\n");
    const double dt = 1.0 / 200.0;
    // 出来事を数える。print なら中身も出す
    auto count = [](rm::EventQueue & ev, rm::EventLevel lv, bool print) {
        int n = 0;
        rm::Event e;
        while (ev.pop(e)) {
          if (print) {
            const char * mark = e.level == rm::EventLevel::Error ? "  ★ " :
              (e.level == rm::EventLevel::Warn ? "  ! " : "    ");
            std::printf("%s%s\n", mark, e.text.c_str());
          }
          n += (e.level == lv) ? 1 : 0;
        }
        return n;
      };

    rm::WalkSetup walk;
    walk.mode = rm::WalkMode::Static;
    walk.gait = gait;

    // (a) static_gait.yaml そのものの門。config の話なので --strict のときだけ落とす
    {
      rm::EventQueue ev;
      rm::loadStaticGait(static_gait_path, walk.stat, ev);
      rm::checkStaticGait(walk.stat, ev);
      rm::checkStaticStance(walk.stat, home, ev);
      rm::checkStaticWalkEnvelope(map, walk, home, body_pitch, ev);
      int nerr = 0;
      rm::Event e;
      while (ev.pop(e)) {
        const char * mark = e.level == rm::EventLevel::Error ? "  ★ " :
          (e.level == rm::EventLevel::Warn ? "  ! " : "    ");
        std::printf("%s%s\n", mark, e.text.c_str());
        nerr += (e.level == rm::EventLevel::Error) ? 1 : 0;
      }
      if (strict) {
        check(nerr == 0, fmt("静歩行の門が出したエラー %d 件", nerr));
      } else if (nerr > 0) {
        std::printf("  -- 静歩行の門のエラー %d 件。**この実行では失敗にしない** (--strict で落とす)\n", nerr);
      } else {
        check(true, "静歩行の門はエラーなし");
      }
    }

    // (b) 門が食い違いを拾うこと (コードの検算。config に依らず落とす)
    {
      const rm::rwc::StaticGaitParams base;       // 既定値 = static_gait.yaml
      rm::BodyPose home70 = home;
      for (int s = 0; s < rm::kNumSide; ++s) {
        const double lat = (s == rm::kLeft) ? +1.0 : -1.0;
        home70.foot[s].p = rk::Vec3{0.0, lat * base.foot_spacing * 500.0, -base.z_c * 1000.0};
      }
      rm::EventQueue ev;
      rm::checkStaticStance(base, home70, ev);
      check(
        count(ev, rm::EventLevel::Warn, false) == 0,
        "計画の足間隔とホーム姿勢の足が揃っていれば、立位の門は黙っている");
      rm::rwc::StaticGaitParams wide = base;
      wide.foot_spacing = 0.170;
      rm::checkStaticStance(wide, home70, ev);
      check(
        count(ev, rm::EventLevel::Warn, false) == 1,
        "static_gait.yaml の foot_spacing が 2 x foot.y と違えば警告する");
      rm::rwc::StaticGaitParams edge = base;
      edge.com_offset_y = 0.030;                  // 静的余裕 37 - 30 = 7mm <= zmp_tol 10mm
      rm::checkStaticStance(edge, home70, ev);
      check(
        count(ev, rm::EventLevel::Error, false) == 1,
        "重心の横ずらしで静的余裕が zmp_tol 以下になればエラー");

      rm::WalkSetup hi = walk;
      hi.stat = base;
      hi.stat.swing_height = 0.050;              // 外へ開いた遊脚が足首で届かない
      rm::checkStaticWalkEnvelope(map, hi, home70, 0.0, ev);
      check(
        count(ev, rm::EventLevel::Error, false) == 1,
        "足上げ 50mm は届かない時刻があるのでエラー (static_gait.yaml の表)");

      // yaml の読み込み: 列も読め、知らないキーは警告
      const std::string tmp = "/tmp/motion_selftest_static_gait.yaml";
      if (FILE * fp = std::fopen(tmp.c_str(), "w")) {
        std::fputs("zmp_tol: 0.012\nv_max: [0.08, 0.03]\nt_step: 0.6\n", fp);
        std::fclose(fp);
      }
      rm::rwc::StaticGaitParams got;
      rm::loadStaticGait(tmp, got, ev);
      const int nw = count(ev, rm::EventLevel::Warn, false);
      check(
        got.zmp_tol == 0.012 && got.v_max[0] == 0.08 && got.v_max[1] == 0.03 && nw == 1,
        fmt(
          "static_gait.yaml の読み込み: 列も読み、知らないキー (t_step) は警告 %d 件", nw));
      std::remove(tmp.c_str());
    }

    // (c) 状態機械が静歩行で歩いて止まる。振り出し中の重心は門の言うとおりの位置
    //     (ホーム姿勢の足を計画の足間隔から 5mm 外へずらし、重心が内へ 5mm ずれる形で見る)
    {
      rm::BodyPose home5 = home;
      const double half = walk.stat.foot_spacing * 500.0;
      for (int s = 0; s < rm::kNumSide; ++s) {
        const double lat = (s == rm::kLeft) ? +1.0 : -1.0;
        home5.foot[s].p.x = 0.0;
        home5.foot[s].p.y = lat * (half + 5.0);
      }
      const rm::BodyPose meas5 = asMeasured(map, home5);
      rm::MotionController::Options copt;
      rm::MotionController c;
      c.configure(&map, &lib5, walk, home5, body_pitch, copt);
      const std::string why = "(テスト)";
      c.setEstop(false);
      c.requestMotion("home");
      double now = 0.0;
      auto t = c.step(now, dt, &meas5, why, true);
      for (int i = 0; i < 800 && t.state != rm::State::HOLD; ++i) {
        now += dt;
        t = c.step(now, dt, &meas5, why, true);
      }
      check(
        t.state == rm::State::HOLD && c.walkMode() == rm::WalkMode::Static &&
        c.stateText() == "HOLD walk=static",
        "静歩行で HOLD に入り、/motion/state は \"HOLD walk=static\" (" + c.stateText() + ")");

      bool saw_shift = false, saw_swing = false, saw_start = false;
      int changes = 0;
      double lat_dev = 0.0, x_dev = 0.0, z_max = -1e9;
      for (int i = 0; i < static_cast<int>(12.0 / dt); ++i) {
        now += dt;
        c.setWalkCmd(0.10, 0.0, 0.0, now);
        t = c.step(now, dt, &meas5, why, true);
        changes += t.state_changed ? 1 : 0;
        const rm::rwc::WalkOutputs * w = c.walkOutputs();
        if (!w) {continue;}
        saw_shift = saw_shift || w->state == rm::rwc::State::SHIFT;
        saw_start = saw_start || w->state == rm::rwc::State::START ||
          w->state == rm::rwc::State::STEP;
        if (w->state != rm::rwc::State::SWING) {continue;}
        saw_swing = true;
        // 支持足は骨盤から見て外へ 5mm (= 重心が支持足の中心から内へ 5mm)。前後は 0
        const int sup = (w->support == rm::rwc::LEFT) ? rm::kLeft : rm::kRight;
        const int sw = (sup == rm::kLeft) ? rm::kRight : rm::kLeft;
        const double lat = (sup == rm::kLeft) ? +1.0 : -1.0;
        const rk::Vec3 & f = c.currentPose().foot[sup].p;
        lat_dev = std::max(lat_dev, std::abs(f.y - lat * 5.0));
        x_dev = std::max(x_dev, std::abs(f.x));
        z_max = std::max(z_max, c.currentPose().foot[sw].p.z);
      }
      check(saw_shift && saw_swing && !saw_start, "静歩行の計画 (SHIFT / SWING) で歩く");
      check(
        lat_dev < 1e-6 && x_dev < 1e-6,
        fmt(
          "振り出し中の支持足は骨盤から見て外へ 5.0mm・前後 0 (ずれ %.2g / %.2g mm)。"
          "門 (checkStaticStance) の「重心が内へ 5mm」と同じ", lat_dev, x_dev));
      check(
        std::abs(z_max - (home5.foot[rm::kLeft].p.z + walk.stat.swing_height * 1000.0)) < 0.5,
        fmt("遊脚はホーム姿勢の高さから足上げ %.0fmm まで上がる (最高 %.1fmm)",
        walk.stat.swing_height * 1000.0, z_max));

      // 止まるまで: 指令の減速 (a_max で約 1.7s) の間の歩 + 揃える歩 + 中点へ戻す移動
      for (int i = 0; i < static_cast<int>(12.0 / dt); ++i) {
        now += dt;
        c.setWalkCmd(0.0, 0.0, 0.0, now);
        t = c.step(now, dt, &meas5, why, true);
        changes += t.state_changed ? 1 : 0;
      }
      double dev = 0.0;
      for (int s = 0; s < rm::kNumSide; ++s) {
        dev = std::max(
          {dev, std::abs(c.currentPose().foot[s].p.x - home5.foot[s].p.x),
            std::abs(c.currentPose().foot[s].p.y - home5.foot[s].p.y),
            std::abs(c.currentPose().foot[s].p.z - home5.foot[s].p.z)});
      }
      check(
        t.state == rm::State::HOLD && changes == 2,
        fmt("指令を止めたら HOLD に戻る。状態の変化は HOLD->WALK->HOLD の 2 回 (%d 回)", changes));
      check(dev < 1e-6, fmt("止まった足はホーム姿勢の足 (ずれ %.2g mm)", dev));

      // 振り出しの途中で脱力が来たら、すぐ RELAX
      for (int i = 0; i < static_cast<int>(10.0 / dt); ++i) {
        now += dt;
        c.setWalkCmd(0.10, 0.0, 0.0, now);
        t = c.step(now, dt, &meas5, why, true);
        if (c.walkOutputs() && c.walkOutputs()->state == rm::rwc::State::SWING) {break;}
      }
      const bool swinging = c.walkOutputs() && c.walkOutputs()->state == rm::rwc::State::SWING;
      c.setEstop(true);
      now += dt;
      t = c.step(now, dt, &meas5, why, true);
      check(
        swinging && t.state == rm::State::RELAX && !t.want_torque && t.target == nullptr,
        "振り出しの途中で /estop true が来たら、その周期に脱力する");
      rm::Event e;
      while (c.popEvent(e)) {}

      // 動歩行で組んだら、状態の文字列は今までと同じ
      rm::MotionController d;
      d.configure(&map, &lib5, gait, home, body_pitch, copt);
      check(
        d.walkMode() == rm::WalkMode::Dynamic && d.stateText() == "RELAX",
        "動歩行では /motion/state に何も足さない (" + d.stateText() + ")");
    }

    // (d) 安定化: SWING を片足支持として扱い、着地の時刻は t_swing で見る
    {
      rm::Stabilizer st;
      const rm::SwingTiming sw = walk.swingTiming();
      st.configure(&map, home, body_pitch, sw);
      {
        rm::Event e;
        while (st.popEvent(e)) {}
      }
      rm::StabGains g;
      g.kp_pitch = 0.5;
      st.setGains(g);
      rm::Attitude att;
      att.valid = true;
      att.pitch = 0.1;
      rm::rwc::WalkOutputs w;
      rm::Stabilizer::Input in;
      in.layer_active = true;
      in.att = &att;
      in.att_age = 0.005;
      in.walk = &w;
      w.state = rm::rwc::State::SWING;
      w.support = rm::rwc::LEFT;
      w.phase = 0.5;
      for (int i = 0; i < 200; ++i) {st.update(dt, in);}
      check(
        st.correction().ankleZero(rm::kRight) && !st.correction().ankleZero(rm::kLeft),
        "静歩行の振り出しの中ほどでは、右 (遊脚) の足首に補正を出さない");
      w.phase = sw.touch_phase;
      st.update(dt, in);
      const bool gate_td = st.debug().gate;
      w.phase = 0.01;
      st.update(dt, in);
      check(
        gate_td && !st.debug().gate && std::abs(sw.duration - walk.stat.t_swing) < 1e-12 &&
        sw.touch_phase < 1.0,
        fmt(
          "着地 (振り出しの %.0f%%、t_swing %.2fs) の前後だけゲインを弱め、振り出しの頭は弱めない",
          sw.touch_phase * 100.0, sw.duration));
      w.state = rm::rwc::State::SHIFT;
      w.support = 0;
      w.phase = 0.5;
      st.update(dt, in);
      check(
        st.debug().weight[rm::kRight] == 1.0 && st.debug().weight[rm::kLeft] == 1.0 &&
        !st.debug().gate,
        "重心移動 (SHIFT) の両足支持では両脚に効かせる");
    }
  }

  std::printf("\n%s\n", g_fail ? "★通らなかった項目がある" : "全部通った");
  return g_fail ? 1 : 0;
}
