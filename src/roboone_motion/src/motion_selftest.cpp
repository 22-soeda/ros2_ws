// motion_selftest — 姿勢の表し方まわりの自己検算。実機もサーボも要らない。
//
//   ros2 run roboone_motion motion_selftest
//   ros2 run roboone_motion motion_selftest --motions /tmp/draft.yaml
//
// 見るのは 6 つ。どれも「実機で踏むと原因が非常に追いにくい」種類の食い違い。
//
//   [1] T ポーズ基準角 [deg] <-> 絶対サーボ角 [rad] <-> 生カウント が一巡すること
//       (servo_map.hpp の [1][2][3]。config に書く角と機体を流れる角の橋)
//   [2] 胴体の前傾を「足裏を回して IK」と「股ピッチのサーボ角に足す」の 2 通りで
//       掛けた結果が一致すること (body_pose.hpp の等価性。**左脚は AXIS_FLIP で
//       サーボの向きが逆**なので、符号を落とすとここで開く)
//   [3] 実機の motions.yaml が読めること
//   [4] 脚の角度書き (R_leg / L_leg) の読み込み・引き継ぎ・書き方が混ざる区間
//   [5] 状態機械の順序 (motion_control.hpp「順序の約束」)。**全部、実機でしか
//       踏めなかったバグの修正**なので、テストが無いと再発しても気付けない
//   [6] 設定の門 (motion_config.hpp)。gait.yaml / home_pose.yaml を書き換えたあと、
//       実機を起こす前にここで確かめられる。**既定では報告だけで落とさない。**
//       [1]-[5] はコードの検算なので赤いままにしてはいけないが、[6] が見ているのは
//       その時どきの config（調整の途中なら当然エラーが出る）。混ぜると
//       「テストはいつも赤いもの」になって [1]-[5] の赤に気付けなくなる。
//       config を詰める側の作業では --strict を付けて落とす。
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

#include "roboone_motion/motion_config.hpp"
#include "roboone_motion/motion_control.hpp"
#include "roboone_motion/motion_library.hpp"

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
  std::string home_path, limits_path, motions_path, gait_path, home_pose_path;
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
    } else if (a == "--strict") {
      strict = true;
    } else {
      std::fprintf(
        stderr,
        "使い方: motion_selftest [--home servo_home.yaml] [--limits servo_limits.yaml]"
        " [--motions motions.yaml] [--gait gait.yaml] [--home-pose home_pose.yaml]"
        " [--strict]\n"
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
    if (gait_path.empty() || home_pose_path.empty()) {
      std::string ref;
      try {
        ref = ament_index_cpp::get_package_share_directory("roboone_walk_ref");
      } catch (const std::exception &) {
        ref = ament_index_cpp::get_package_share_directory("roboone_motion");
      }
      if (gait_path.empty()) {gait_path = ref + "/config/gait.yaml";}
      if (home_pose_path.empty()) {home_pose_path = ref + "/config/home_pose.yaml";}
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
      rm::checkStance(gait, 0.0, ev);
      rm::checkWalkEnvelope(map, gait, home, body_pitch, 0.0, ev);
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

    // 実測姿勢として使う「取れている姿勢」。ホーム姿勢なら必ず IK で解ける。
    const rm::BodyPose meas = home;
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

    // [5-2] 実測姿勢が 1 度も取れていないうちは武装しない。
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
  }

  std::printf("\n%s\n", g_fail ? "★通らなかった項目がある" : "全部通った");
  return g_fail ? 1 : 0;
}
