#include "roboone_motion/motion_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#include "roboone_walk_core/walk_engine.hpp"

namespace roboone_motion
{

namespace
{

constexpr double kR2D = 180.0 / M_PI;

/// printf 書式で 1 行こしらえる（EventQueue は文字列しか受けない）。
std::string fmt(const char * f, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return std::string(buf);
}

}  // namespace

void fallbackPath(std::string & path, const std::string & def, const char * what, EventQueue & ev)
{
  if (!path.empty()) {return;}
  path = def;
  ev.warn(
    std::string(what) + " のパスが空だったので既定を使う: " + def +
    " (別のノード向けの params ファイルが流れ込んでいないか確認すること)");
}

void loadGait(const std::string & path, rwc::GaitParams & out, EventQueue & ev)
{
  YAML::Node y;
  try {
    y = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    ev.warn(
      std::string("gait.yaml を読めない (") + e.what() + ")。gait_params.hpp の既定値で走る");
    return;
  }
  std::vector<std::string> known;
  auto pick = [&](const char * k, double & dst) {
      known.emplace_back(k);
      if (y[k]) {dst = y[k].as<double>();}
    };
  pick("z_c", out.z_c);
  pick("gravity", out.gravity);
  pick("t_step", out.t_step);
  pick("foot_spacing", out.foot_spacing);
  pick("swing_height", out.swing_height);
  pick("swing_lock_phase", out.swing_lock_phase);
  pick("td_overdrive", out.td_overdrive);
  pick("td_speed_max", out.td_speed_max);
  pick("swing_ratio", out.swing_ratio);
  pick("step_clamp_x", out.step_clamp_x);
  pick("step_clamp_out", out.step_clamp_out);
  pick("step_clamp_in", out.step_clamp_in);
  pick("start_pushoff_max", out.start_pushoff_max);
  pick("k_dcm", out.k_dcm);
  pick("cmd_timeout", out.cmd_timeout);
  pick("loop_hz", out.loop_hz);
  for (const char * k : {"v_max", "a_max"}) {
    known.emplace_back(k);
    if (!y[k] || !y[k].IsSequence() || y[k].size() != 2) {continue;}
    double * dst = (std::string(k) == "v_max") ? out.v_max : out.a_max;
    dst[0] = y[k][0].as<double>();
    dst[1] = y[k][1].as<double>();
  }
  for (const auto & kv : y) {
    const std::string k = kv.first.as<std::string>();
    if (std::find(known.begin(), known.end(), k) == known.end()) {
      ev.warn("gait.yaml の知らないキー \"" + k + "\" は無視した");
    }
  }
  ev.info(
    fmt(
      "歩行 z_c=%.3fm T=%.2fs W=%.3fm v_max=(%.2f, %.2f)",
      out.z_c, out.t_step, out.foot_spacing, out.v_max[0], out.v_max[1]));
}

void checkGait(const rwc::GaitParams & gait, EventQueue & ev)
{
  const rwc::SwingLanding sl = rwc::checkSwingLanding(gait);
  if (!sl.lands) {
    ev.error(
      fmt(
        "遊脚が床に届かないまま歩が終わる (φ=1 で +%.1fmm 浮いている)。"
        "空中で支持脚が入れ替わる。td_speed_max を %.3f 以上へ上げるか、"
        "swing_height を下げるか、t_step を伸ばすこと "
        "(降下区間 0.55·swing_ratio·T = %.3fs に %.1fmm 降ろす必要がある)",
        sl.z_end * 1000.0, sl.v_need, 0.55 * gait.swing_ratio * gait.t_step,
        (gait.swing_height + gait.td_overdrive) * 1000.0));
  } else {
    ev.info(
      fmt(
        "遊脚 swing_ratio=%.2f (狙いの両足支持 %.1f%%) -> 実際は位相 %.3f で接地し"
        " 両足支持 %.1f%% (%.0f ms)%s",
        gait.swing_ratio, (1.0 - gait.swing_ratio) * 100.0, sl.touch_phase,
        sl.double_support * 100.0, sl.double_support * gait.t_step * 1000.0,
        sl.saturated ? " ※降下が td_speed_max に張り付いている (狙いより必ず小さく出る)" : ""));
  }
  if (sl.lands && sl.touch_phase < gait.swing_lock_phase) {
    ev.warn(
      fmt(
        "接地の位相 %.3f が着地点の凍結位相 swing_lock_phase %.2f より早い。"
        "着地点がまだ動いている最中に足が床へ着く。swing_lock_phase を下げること",
        sl.touch_phase, gait.swing_lock_phase));
  }
}

void checkStance(const rwc::GaitParams & gait, const BodyPose & home, EventQueue & ev)
{
  const double plan_half = gait.foot_spacing * 500.0;
  const double real_half = home.foot[kLeft].p.y;
  const double hip_half = -rk::config::HIP_Y;
  ev.info(
    fmt(
      "歩行の足 = ホーム姿勢の足 (±%.1fmm。股の真下から %+.1fmm)。計画上の足間隔は ±%.1fmm"
      " (gait.yaml の foot_spacing) で、実機の足より %+.1fmm 外側に取っている",
      real_half, real_half - hip_half, plan_half, plan_half - real_half));
  if (plan_half < real_half) {
    ev.warn(
      fmt(
        "計画上の足間隔 ±%.1fmm が実機の足 ±%.1fmm より狭い。骨盤が支持足の上まで"
        "来ないので、単脚支持で遊脚側へ倒れやすい (gait.yaml の foot_spacing の注記)",
        plan_half, real_half));
  }
}

bool loadHomePose(
  const std::string & path, const ServoMap & map, const rwc::GaitParams & gait,
  BodyPose & out, double & body_pitch, EventQueue & ev, std::string & err)
{
  YAML::Node y;
  try {
    y = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    err = "ホーム姿勢を読めない: " + path + " (" + e.what() + ")";
    return false;
  }
  if (!y["foot"]) {
    err = path + ": foot: が無い";
    if (y["legs"]) {
      err += "（書式が変わった。legs: の関節角ではなく foot: の height / x / y / rpy で書く）";
    }
    return false;
  }
  const YAML::Node & f = y["foot"];
  const double h = f["height"] ? f["height"].as<double>() : 282.0;
  const double x = f["x"] ? f["x"].as<double>() : 0.0;
  const double half = f["y"] ? f["y"].as<double>() : -rk::config::HIP_Y;
  double rpy[3]{0.0, 0.0, 0.0};
  if (f["rpy"] && f["rpy"].IsSequence() && f["rpy"].size() == 3) {
    for (int k = 0; k < 3; ++k) {
      rpy[k] = f["rpy"][k].as<double>() * M_PI / 180.0;
    }
  }
  if (!(h > 0.0)) {
    err = path + ": foot.height は正の mm";
    return false;
  }

  const double bp_deg = y["body_pitch"] ? y["body_pitch"].as<double>() : 0.0;
  if (std::abs(bp_deg) > 45.0) {
    err = path + ": body_pitch が大きすぎる (|deg| <= 45)";
    return false;
  }
  body_pitch = bp_deg * M_PI / 180.0;

  out.arm.assign(map.num_arm(), 0.0);
  for (int s = 0; s < kNumSide; ++s) {
    // 左右は鏡像。y と、roll・yaw の符号だけが反転する（pitch は左右同じ）。
    const double lat = (s == kLeft) ? +1.0 : -1.0;
    out.foot[s].p = rk::Vec3{x, lat * half, -h};
    out.foot[s].rpy[0] = rpy[0] * lat;
    out.foot[s].rpy[1] = rpy[1];
    out.foot[s].rpy[2] = rpy[2] * lat;
  }

  // 腕。ID をキーにすると左右共通、"R8" / "L8" と書くと片側だけ。
  if (y["arms"]) {
    const auto & arms = map.arms();
    for (std::size_t a = 0; a < arms.size(); ++a) {
      if (const YAML::Node n = y["arms"][arms[a].id]) {out.arm[a] = n.as<double>();}
      if (const YAML::Node n = y["arms"][arms[a].name]) {out.arm[a] = n.as<double>();}
    }
  }

  ev.info(
    fmt(
      "ホーム姿勢 (%s): 足裏 高さ %.1fmm / 前後 %+.1fmm / 半間隔 %.1fmm / "
      "姿勢 rpy [%.1f, %.1f, %.1f] deg / 胴体の前傾 %+.1f deg",
      path.c_str(), h, x, half, rpy[0] * kR2D, rpy[1] * kR2D, rpy[2] * kR2D, bp_deg));
  if (bp_deg != 0.0) {
    ev.info(
      fmt(
        "胴体を %+.1f deg 前傾させる (股ピッチに %+.1f deg を足すのと等価。"
        "膝・足首の関節角は変わらない)", bp_deg, -bp_deg));
  }

  // 骨盤高さ (z_c) が gait.yaml と食い違っていたら言う。歩行の計画高さと実際の
  // 立位高さがずれると、LIPM の ω が実機と合わない。
  if (std::abs(h / 1000.0 - gait.z_c) > 0.005) {
    ev.warn(
      fmt(
        "ホーム姿勢の骨盤高さ %.3fm が gait.yaml の z_c %.3fm と違う", h / 1000.0, gait.z_c));
  }
  // 足間隔は gait.yaml の foot_spacing と違っていてよい (歩行の足はこちらに揃える。
  // checkStance が関係を言う)
  return true;
}

void checkPoseReachable(
  const ServoMap & map, const BodyPose & pose, const char * what, EventQueue & ev)
{
  for (int s = 0; s < kNumSide; ++s) {
    double servo[rk::kNumJoints], theta[rk::kNumJoints];
    const LegSolve r = servoFromFootPose(map.leg_params(s), pose.foot[s], servo, theta);
    if (!r.ok()) {
      ev.error(
        fmt(
          "%s の %s脚が解けない (ik=%d servo=%d)。config を直すこと",
          what, kSideTag[s], static_cast<int>(r.ik_status), static_cast<int>(r.servo_status)));
    } else if (r.ankle_outside_envelope) {
      ev.warn(fmt("%s の %s脚: 足首がエンベロープの外（解けてはいる）", what, kSideTag[s]));
    }
  }
}

void checkMotionLegServo(const ServoMap & map, const MotionLibrary & lib, EventQueue & ev)
{
  int frames = 0;
  for (const auto & m : lib.motions()) {
    for (std::size_t i = 0; i < m.frames.size(); ++i) {
      const KeyFrame & f = m.frames[i];
      for (int s = 0; s < kNumSide; ++s) {
        if (!f.anyLegServo(s)) {continue;}
        ++frames;
        for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
          if (!f.has_leg[s][j]) {continue;}
          bool clamped = false;
          map.leg_count_from_servo(s, j, f.leg_servo[s][j], &clamped);
          if (clamped) {
            ev.error(
              fmt(
                "技 \"%s\" の %zu 枚目 %s_leg ID%d (%s) が servo_limits.yaml の窓の外"
                " (%.2f deg)。再生時に丸められてその軸だけ動かない",
                m.name.c_str(), i + 1, kSideTag[s], kLegServoId[j], kLegJointName[j],
                map.leg_tpose_deg_from_servo(s, j, f.leg_servo[s][j])));
          }
        }
      }
    }
  }
  if (frames > 0) {
    ev.info(
      fmt("脚をサーボ角で書いたキーフレームが %d 枚ある (その脚は IK を通らずに出る)", frames));
  }
}

void checkWalkEnvelope(
  const ServoMap & map, const rwc::GaitParams & gait, const BodyPose & home,
  double body_pitch, EventQueue & ev)
{
  const double zc = gait.z_c * 1000.0;
  const double cx = gait.step_clamp_x * 1000.0;
  const double cout = gait.step_clamp_out * 1000.0;
  const double cin = gait.step_clamp_in * 1000.0;
  const double hsw = gait.swing_height * 1000.0;

  int worst = static_cast<int>(ReachLevel::Design);
  FootPose worst_pose;
  int side_worst = kRight;

  for (int s = 0; s < kNumSide; ++s) {
    const double lat = (s == kLeft) ? +1.0 : -1.0;
    for (double dx : {-cx, 0.0, cx}) {
      for (double dy : {-cin, 0.0, cout}) {
        for (double dz : {0.0, hsw}) {
          FootPose f;
          // 立位はホーム姿勢の足 (歩行の足はそこに揃えてある)。
          // 外側 / 内側は脚ごとに向きが逆。dy > 0 を「外側」として左右に配る。
          const rk::Vec3 & base = home.foot[s].p;
          f.p = rk::Vec3{base.x + dx, base.y + lat * dy, base.z + dz};
          // 姿勢はホームと同じ（歩行中もその向きで出すので、同じ条件で見る）。
          for (int k = 0; k < 3; ++k) {
            f.rpy[k] = home.foot[s].rpy[k];
          }
          // 判定は指令と同じ系 (Σ_B) で行う。箱そのものは Σ_U で組んであるので
          // 前傾を掛けてから見る。表示は Σ_U のまま (gait.yaml と突き合わせるため)。
          FootPose fb = f;
          bodyPitchApply(fb, body_pitch);
          const int lv = static_cast<int>(reachLevel(map.leg_params(s), fb));
          if (lv < worst) {
            worst = lv;
            worst_pose = f;
            side_worst = s;
          }
        }
      }
    }
  }

  const auto lvl = static_cast<ReachLevel>(worst);
  if (lvl == ReachLevel::Design) {
    ev.info(
      fmt(
        "歩行の足先の箱 (x±%.0f / 外%.0f 内%.0f / 高さ%.0f mm, z_c=%.0f) はすべて design 域の内側",
        cx, cout, cin, hsw, zc));
    return;
  }
  if (lvl == ReachLevel::Mech) {
    // **これは想定内。** home_pose.yaml が「leg_bend 30 deg だと足首ピッチが常時
    // -30 deg で、足首パラレルリンクの設計可動域 (同時 ±15 deg) の外・機構限界の
    // 内で歩くことになる」と断ってある。届かないわけではないので情報に留める。
    ev.info(
      fmt(
        "歩行の足先の箱は mech 域には収まるが design 域は出る"
        " (最悪 %s脚 p=[%.1f, %.1f, %.1f])。立位で足首ピッチを使っているぶん"
        " 設計可動域 (同時 ±15 deg の菱形) の外で歩く。"
        " 内側に入れたいなら home_pose.yaml の foot.height を上げる",
        kSideTag[side_worst], worst_pose.p.x, worst_pose.p.y, worst_pose.p.z));
    return;
  }
  ev.error(
    fmt(
      "歩行の足先の箱の隅に **届かない** (%s止まり: %s脚 p=[%.1f, %.1f, %.1f])。"
      "そのまま歩かせるとその位相で IK が解けず脚が止まる。gait.yaml の"
      " step_clamp_* / swing_height / z_c を"
      " `ros2 run roboone_walk_core gait_from_kinematics` で出し直すこと",
      reachLevelName(lvl), kSideTag[side_worst],
      worst_pose.p.x, worst_pose.p.y, worst_pose.p.z));
}

}  // namespace roboone_motion
