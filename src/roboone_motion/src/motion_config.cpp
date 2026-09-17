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
  char buf[1024];   // 日本語は 1 文字 3 バイト。長い門の文が切れないように
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
  pick("ds_time", out.ds_time);
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
      "歩行 z_c=%.3fm T=%.2fs 両足支持 %.2fs W=%.3fm v_max=(%.2f, %.2f)",
      out.z_c, out.t_step, out.ds_time, out.foot_spacing, out.v_max[0], out.v_max[1]));
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
  if (gait.ds_time < 0.0) {
    ev.warn(fmt("ds_time %.3f は負。0 (両足支持なし) として扱われる", gait.ds_time));
  } else if (gait.ds_time > 0.0) {
    const rwc::DsConsts c = gait.ds_consts();
    const double step = gait.ds_time + gait.t_step;
    ev.info(
      fmt(
        "両足支持 %.2fs + 単脚支持 %.2fs = 1 歩 %.2fs。歩く速さは指令の %.0f%% (歩幅は v·t_step)。"
        " 1 歩の増幅 e^{ωT} = %.0f、ずれの吸収に要る着地点のずらし %.1f 倍",
        gait.ds_time, gait.t_step, step, 100.0 * gait.t_step / step, c.e, c.ed / c.kappa));
    if (gait.ds_time > 0.4 + 1e-9) {
      // 2026-09-18 の走査 (gait.yaml の ds_time の注記): a_max (0.06, 0.03) では
      // 0.4s は通り、0.5s は全速前進からの停止でも計画が発散した
      ev.warn(
        fmt(
          "ds_time %.2fs は長い。a_max (%.2f, %.2f) のままだと指令の変化を着地点で吸収しきれず"
          "計画が発散しうる (0.5s で全速前進からの停止が発散した)。a_max を下げること",
          gait.ds_time, gait.a_max[0], gait.a_max[1]));
    }
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

// ===========================================================================
// 静歩行
// ===========================================================================

void loadStaticGait(const std::string & path, rwc::StaticGaitParams & out, EventQueue & ev)
{
  YAML::Node y;
  try {
    y = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    ev.warn(
      std::string("static_gait.yaml を読めない (") + e.what() +
      ")。static_walk_engine.hpp の既定値で走る");
    return;
  }
  if (!y.IsMap()) {
    ev.warn("static_gait.yaml が空か書式違い (" + path + ")。static_walk_engine.hpp の既定値で走る");
    return;
  }
  const std::vector<rwc::StaticGaitField> fields = rwc::staticGaitFields(out);
  for (const auto & kv : y) {
    const std::string k = kv.first.as<std::string>();
    const auto it = std::find_if(
      fields.begin(), fields.end(), [&k](const rwc::StaticGaitField & f) {return k == f.name;});
    if (it == fields.end()) {
      ev.warn("static_gait.yaml の知らないキー \"" + k + "\" は無視した");
      continue;
    }
    const YAML::Node & n = kv.second;
    if (it->n == 1) {
      *it->ptr = n.as<double>();
    } else if (n.IsSequence() && static_cast<int>(n.size()) == it->n) {
      for (int i = 0; i < it->n; ++i) {it->ptr[i] = n[i].as<double>();}
    } else {
      ev.warn(fmt("static_gait.yaml の %s は %d 要素の列で書く (既定値のまま)", k.c_str(), it->n));
    }
  }
  ev.info(
    fmt(
      "静歩行 (%s): W=%.3fm / zmp_tol %.1fmm / 振り出し %.2fs / 足上げ %.0fmm / "
      "重心の横ずらし %+.1fmm / 歩幅 = v x %.2fs / v_max=(%.2f, %.2f)",
      path.c_str(), out.foot_spacing, out.zmp_tol * 1000.0, out.t_swing,
      out.swing_height * 1000.0, out.com_offset_y * 1000.0, out.stride_time,
      out.v_max[0], out.v_max[1]));
}

void checkStaticGait(const rwc::StaticGaitParams & stat, EventQueue & ev)
{
  const rwc::StaticGaitCheck r = rwc::checkStaticGait(stat);
  for (const auto & e : r.errors) {
    ev.error("静歩行の設定: " + e);
  }
  ev.info(
    fmt(
      "静歩行: 重心移動 %.2fs (足間隔ぶん) + 振り出し %.2fs。全速前進で 1 歩 %.2fs・%.3f m/s"
      " (歩幅 %.0fmm)。遊脚は振り出しの %.0f%% で接地%s",
      r.t_shift_step, stat.t_swing, r.t_cycle_fwd, r.v_fwd_real, r.stride_max[0] * 1000.0,
      r.touch_phase * 100.0, r.saturated ? " (降下が td_speed_max に張り付いている)" : ""));
}

void checkStaticStance(
  const rwc::StaticGaitParams & stat, const BodyPose & home, EventQueue & ev)
{
  const double half = stat.foot_spacing * 500.0;
  const double zc = stat.z_c * 1000.0;
  const double off = stat.com_offset_y * 1000.0;
  const double hl = stat.sole_length * 500.0, hw = stat.sole_width * 500.0;
  // 振り出し中、計画の重心は「支持足の中心 + 外へ off」。実機の足はそこから
  // walkStanceOffset() だけ動くので、骨盤 (= 重心) から見た支持足の位置もずれる。
  double worst = 1e9, c_eff_show = 0.0, x_eff_show = 0.0;
  for (int s = 0; s < kNumSide; ++s) {
    const double lat = (s == kLeft) ? +1.0 : -1.0;
    const rk::Vec3 & hp = home.foot[s].p;
    const double c_eff = off + half - lat * hp.y;   // 支持足の中心から外へ (+)
    const double x_eff = 0.0 - hp.x;                // 支持足の中心から前へ (+。-0 を出さない)
    const double m = std::min(hl - std::abs(x_eff), hw - std::abs(c_eff));
    if (m < worst) {
      worst = m;
      c_eff_show = c_eff;
      x_eff_show = x_eff;
    }
  }
  const rk::Vec3 & hl_p = home.foot[kLeft].p;
  ev.info(
    fmt(
      "静歩行の立位: 計画の足間隔 ±%.1fmm / ホーム姿勢の足 ±%.1fmm (前後 %+.1fmm)。"
      "振り出し中の重心は支持足の中心から外へ %+.1fmm・前へ %+.1fmm"
      " (うち com_offset_y %+.1fmm)。片足支持の静的余裕 %.1fmm",
      half, hl_p.y, hl_p.x, c_eff_show, x_eff_show, off, worst));
  if (std::abs(half - hl_p.y) > 0.5) {
    ev.warn(
      fmt(
        "static_gait.yaml の foot_spacing (±%.1fmm) がホーム姿勢の足 (home_pose.yaml の"
        " foot.y ±%.1fmm) と違う。差の %+.1fmm だけ振り出し中の重心が支持足の中心から"
        "横へずれる。foot_spacing を 2 x foot.y に揃えること"
        " (重心をずらしたいなら com_offset_y で持つ)",
        half, hl_p.y, half - hl_p.y));
  }
  if (std::abs(hl_p.x) > 0.5) {
    ev.warn(
      fmt(
        "ホーム姿勢の足が前後に %+.1fmm ずれている。静歩行では重心が支持足の中心から"
        "前後へ %+.1fmm ずれたまま足を上げる", hl_p.x, -hl_p.x));
  }
  if (std::abs(-hl_p.z - zc) > 5.0) {
    ev.warn(
      fmt(
        "ホーム姿勢の骨盤高さ %.1fmm が static_gait.yaml の z_c %.1fmm と違う"
        " (重心移動の時間の計算がずれる)", -hl_p.z, zc));
  }
  if (worst <= stat.zmp_tol * 1000.0) {
    ev.error(
      fmt(
        "静歩行の片足支持の静的余裕 %.1fmm が zmp_tol %.1fmm 以下。重心移動の途中で"
        " ZMP が足裏から出る", worst, stat.zmp_tol * 1000.0));
  }
}

namespace
{

/// 計画に与える指令の組。roboone_viz/static_reach.py の profiles() と同じ 11 通り。
/// 変えたら両方を揃えること。
struct WalkProfile
{
  const char * name;
  double a[2];
  double b[2];
  bool two;
};

/// 11 通りの指令で歩行計画を回し、一定間隔で両足の足先 (Σ_U) を fn(名前, 出力, 足先) へ渡す。
///
/// 時刻は k 倍する (静歩行は k = 全速前進の 1 歩 / 2.13s)。歩きを遅くしても同じ歩の
/// 並びを見るため。
///   two = false: 0.5k <= t < 9.5k の間 a
///   two = true : 0.5k <= t < 8k の間 a、8k <= t < 16k の間 b
///   26k まで回し、2k·thin 周期に 1 回見る
/// 戻り値は渡した点の数 (脚の数は数えない)。
template<typename F>
int runWalkProfiles(const WalkSetup & walk, const BodyPose & home, int thin, F fn)
{
  const bool st = walk.isStatic();
  const double vx = st ? walk.stat.v_max[0] : walk.gait.v_max[0];
  const double vy = st ? walk.stat.v_max[1] : walk.gait.v_max[1];
  const double dx = 0.8 * vx, dy = 0.625 * vy;     // 斜めは楕円制限の内側
  const WalkProfile profs[] = {
    {"前進", {vx, 0.0}, {0.0, 0.0}, false},
    {"後進", {-vx, 0.0}, {0.0, 0.0}, false},
    {"左", {0.0, vy}, {0.0, 0.0}, false},
    {"右", {0.0, -vy}, {0.0, 0.0}, false},
    {"斜め左前", {dx, dy}, {0.0, 0.0}, false},
    {"斜め右前", {dx, -dy}, {0.0, 0.0}, false},
    {"斜め左後ろ", {-dx, dy}, {0.0, 0.0}, false},
    {"斜め左前 (横多め)", {0.7 * vx, 0.75 * vy}, {0.0, 0.0}, false},
    {"前後の切り返し", {vx, 0.0}, {-vx, 0.0}, true},
    {"左右の切り返し", {0.0, vy}, {0.0, -vy}, true},
    {"前進 -> 斜め右前", {vx, 0.0}, {0.7 * vx, -0.75 * vy}, true},
  };
  const double k = st ? rwc::checkStaticGait(walk.stat).t_cycle_fwd / 2.13 : 1.0;
  const double dt = 0.005, t_end = 26.0 * k;
  const int every = std::max(1, thin * static_cast<int>(std::lround(2.0 * k)));
  rk::Vec3 off[kNumSide];
  walkStanceOffset(walk, home, off);

  int points = 0;
  for (const WalkProfile & pr : profs) {
    WalkPlanner pl;
    pl.configure(walk);
    const int n = static_cast<int>(t_end / dt + 0.5);
    for (int i = 0; i < n; ++i) {
      const double t = i * dt;
      const double * c = nullptr;
      if (t >= 0.5 * k && t < (pr.two ? 8.0 : 9.5) * k) {
        c = pr.a;
      } else if (pr.two && t >= 8.0 * k && t < 16.0 * k) {
        c = pr.b;
      }
      const rwc::WalkOutputs o = pl.update(c ? c[0] : 0.0, c ? c[1] : 0.0, dt);
      if (i % every) {continue;}
      FootPose f[kNumSide];
      walkFeet(o, off, home, f);
      ++points;
      fn(pr.name, o, f);
    }
  }
  return points;
}

}  // namespace

void checkStaticWalkEnvelope(
  const ServoMap & map, const WalkSetup & walk, const BodyPose & home,
  double body_pitch, EventQueue & ev)
{
  int total = 0, bad = 0;
  std::string bad_names, first, last_bad_name;
  runWalkProfiles(
    walk, home, 1,
    [&](const char * name, const rwc::WalkOutputs & o, const FootPose f[kNumSide]) {
      for (int s = 0; s < kNumSide; ++s) {
        FootPose fb = f[s];
        bodyPitchApply(fb, body_pitch);
        ++total;
        const ReachLevel lv = reachLevel(map.leg_params(s), fb);
        if (lv >= ReachLevel::Mech) {continue;}
        ++bad;
        if (last_bad_name != name) {
          last_bad_name = name;
          bad_names += (bad_names.empty() ? "" : " / ");
          bad_names += name;
        }
        if (first.empty()) {
          first = fmt(
            "%s t=%.2fs %s %s脚 p=[%.1f, %.1f, %.1f] (%s)", name, o.t,
            rwc::to_string(o.state), kSideTag[s], f[s].p.x, f[s].p.y, f[s].p.z,
            reachLevelName(lv));
        }
      }
    });
  if (bad == 0) {
    ev.info(
      fmt(
        "静歩行の足先は、前後・左右・斜め・切り返しの 11 通りの指令の全時刻 (%d 点) で"
        " 機構の到達域の内側", total));
    return;
  }
  ev.error(
    fmt(
      "静歩行の足先が機構の到達域の外に出る時刻がある (%d / %d 点。%s)。最初: %s。"
      "その位相で IK が解けず脚が止まる。static_gait.yaml の swing_height を下げるか"
      " com_offset_y を内側 (-) へ寄せること (表は roboone_viz/static_reach.py で作れる)",
      bad, total, bad_names.c_str(), first.c_str()));
}

void checkBoardEnvelope(
  const ServoMap & map, const WalkSetup & walk, const BodyPose & home,
  double body_pitch, double clamp, EventQueue & ev)
{
  if (clamp <= 0.0) {
    ev.info("板の補正は上限 0 (効かない)");
    return;
  }
  // 板を掛けると脚が伸び縮みするので、足首リンクが先に尽きる。1 軸ずつと、
  // ロールとピッチが同時に入る隅の 8 通りを見る (隅がいちばん厳しい)。
  const double kCorner[8][2] = {
    {+1, 0}, {-1, 0}, {0, +1}, {0, -1}, {+1, +1}, {+1, -1}, {-1, +1}, {-1, -1}};
  // 上限をそのまま試して駄目なら、どこまでなら通るかを言う (board_clamp を下げる目安)。
  const double kTry[] = {1.0, 0.75, 0.5, 0.25};
  double ok_at = 0.0;
  int bad_at_clamp = 0, total = 0;   // total は「板を掛けなければ届く」点だけ数える
  std::string first;
  for (const double frac : kTry) {
    const double c = clamp * frac;
    int bad = 0, counted = 0;
    std::string first_here;
    runWalkProfiles(
      walk, home, 8,
      [&](const char * name, const rwc::WalkOutputs & o, const FootPose f[kNumSide]) {
        // 板を掛けない状態で既に届かない脚は数えない。それは計画そのものの話で、
        // checkWalkEnvelope / checkStaticWalkEnvelope がもう言っている。
        bool plain_ok[kNumSide];
        for (int s = 0; s < kNumSide; ++s) {
          FootPose f0 = f[s];
          bodyPitchApply(f0, body_pitch);
          plain_ok[s] = reachLevel(map.leg_params(s), f0) >= ReachLevel::Mech;
        }
        for (const auto & cn : kCorner) {
          FootPose fb[kNumSide] = {f[kRight], f[kLeft]};
          boardApply(fb, cn[0] * c, cn[1] * c);
          for (int s = 0; s < kNumSide; ++s) {
            if (!plain_ok[s]) {continue;}
            ++counted;
            bodyPitchApply(fb[s], body_pitch);
            if (reachLevel(map.leg_params(s), fb[s]) >= ReachLevel::Mech) {continue;}
            ++bad;
            if (first_here.empty()) {
              first_here = fmt(
                "%s t=%.2fs %s %s脚 (ロール %+.1f / ピッチ %+.1f deg)", name, o.t,
                rwc::to_string(o.state), kSideTag[s], cn[0] * c * kR2D, cn[1] * c * kR2D);
            }
          }
        }
      });
    if (frac == 1.0) {
      bad_at_clamp = bad;
      total = counted;
      first = first_here;
    }
    if (bad == 0) {
      ok_at = c;
      break;
    }
  }
  if (bad_at_clamp == 0) {
    ev.info(
      fmt(
        "板の補正 ±%.1f deg は、11 通りの指令の全時刻 (%d 点) で脚が届く",
        clamp * kR2D, total));
    return;
  }
  const std::string how = (ok_at > 0.0) ?
    fmt("全時刻で出し切りたいなら stab.board_clamp を %.3f (±%.1f deg) まで下げること",
      ok_at, ok_at * kR2D) :
    fmt(
      "±%.1f deg まで下げても届かない時刻が残る (計画そのものが到達域の縁にある)。"
      "swing_height / foot_spacing / z_c のほうを見直すこと", 0.25 * clamp * kR2D);
  ev.warn(
    fmt(
      "板の補正 ±%.1f deg は届かない時刻がある (%d / %d 点 = %.3f%%)。最初: %s。"
      "その周期は PoseCodec が板を縮めて出す (指令は止まらない)。%s",
      clamp * kR2D, bad_at_clamp, total, 100.0 * bad_at_clamp / std::max(1, total),
      first.c_str(), how.c_str()));
}

}  // namespace roboone_motion
