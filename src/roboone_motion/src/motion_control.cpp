#include "roboone_motion/motion_control.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>

namespace roboone_motion
{

namespace
{

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

const char * stateName(State s)
{
  switch (s) {
    case State::RELAX: return "RELAX";
    case State::ARMING: return "ARMING";
    case State::HOLD: return "HOLD";
    case State::WALK: return "WALK";
    case State::STAY: return "STAY";
    default: return "MOTION";
  }
}

void MotionController::configure(
  const ServoMap * map, const MotionLibrary * lib, const WalkSetup & walk,
  const BodyPose & home, double body_pitch, const Options & opt)
{
  map_ = map;
  lib_ = lib;
  opt_ = opt;
  body_pitch_ = body_pitch;
  walk_.configure(walk);
  // **歩行の足はホーム姿勢の足に揃える。** 実機の足の位置は home_pose.yaml の foot だけが
  // 決め、計画上の足間隔 (gait.yaml / static_gait.yaml の foot_spacing) は横の重心経路と
  // してだけ効く。計画を実機の足より広く取ると、足の位置はそのままで骨盤の横振りだけが増える。
  // 以前は motion_node.yaml の stance_y_offset で別に持っていて、ホーム姿勢 (±89.3) と
  // 歩行の立位 (±70) が食い違い、武装や home の直後に HOLD へ入った周期で足が
  // 19.3mm 跳んでいた (2026-09-18 の bag で股ロールの目標が 43 カウント跳ぶ)。
  walkStanceOffset(walk, home, stance_off_);
  home_pose_ = home;
  hold_pose_ = home;
  cur_pose_ = home;
}

void MotionController::configure(
  const ServoMap * map, const MotionLibrary * lib, const rwc::GaitParams & gait,
  const BodyPose & home, double body_pitch, const Options & opt)
{
  WalkSetup walk;
  walk.gait = gait;
  configure(map, lib, walk, home, body_pitch, opt);
}

std::string MotionController::stateText() const
{
  std::string s = stateName(state_);
  if (state_ == State::MOTION && player_.active()) {s += ":" + player_.name();}
  // 静歩行のときだけ足す。動歩行の文字列は今までと同じにしておく（behavior のテスト）
  if (walk_.mode() == WalkMode::Static) {s += " walk=static";}
  return s;
}

void MotionController::setWalkCmd(double vx, double vy, double wz, double stamp)
{
  std::lock_guard<std::mutex> lk(walk_mtx_);
  walk_cmd_[0] = vx;
  walk_cmd_[1] = vy;
  walk_stamp_ = stamp;
  if (std::abs(wz) > 1e-3 && !warned_yaw_) {
    warned_yaw_ = true;
    ev_.warn(
      "/cmd_walk の angular.z は使わない。歩行は平行移動のみで、旋回は"
      " キーフレームモーション (turn_l / turn_r) の担当");
  }
}

void MotionController::requestMotion(const std::string & name)
{
  std::lock_guard<std::mutex> lk(req_mtx_);
  motion_req_ = name;
  got_motion_ = true;
}

void MotionController::setState(State s)
{
  if (state_ == s) {return;}
  state_ = s;
}

void MotionController::reportPlayerWarning()
{
  if (player_.warning().empty()) {return;}
  ev_.warn(player_.warning(), 2000, "player_warning");
}

bool MotionController::armPathClear(const BodyPose & from, std::string & why) const
{
  bool ok = true;
  for (int s = 0; s < kNumSide; ++s) {
    const rk::LegServoParams & prm = map_->leg_params(s);
    if (from.leg_mode[s] != LegMode::Servo || !from.leg_servo_valid[s]) {
      why += std::string(kSideTag[s]) + "脚: 実測にサーボ角が無い ";
      ok = false;
      continue;
    }
    // 行き先のサーボ角。足裏書きなら MotionPlayer::start() と同じく補正なしの IK の影
    double to[rk::kNumJoints];
    if (hold_pose_.leg_mode[s] == LegMode::Servo && hold_pose_.leg_servo_valid[s]) {
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {to[j] = hold_pose_.leg_servo[s][j];}
    } else {
      double theta[rk::kNumJoints];
      if (!servoFromFootPose(prm, hold_pose_.foot[s], to, theta).ok()) {
        why += std::string(kSideTag[s]) + "脚: 保持姿勢が IK で解けない ";
        ok = false;
        continue;
      }
    }
    const rk::LegServoPathResult r = rk::legServoPath(prm, from.leg_servo[s], to);
    if (!r.ok()) {
      why += std::string(kSideTag[s]) + "脚: 保持姿勢までの途中で" + legPathWhy(r) + " ";
      ok = false;
    }
  }
  return ok;
}

void MotionController::startBlend(
  const BodyPose & to, double time, double now, const char * what)
{
  blend_motion_.name = what;
  blend_motion_.return_home = false;
  blend_motion_.frames.assign(1, KeyFrame{});
  KeyFrame & f = blend_motion_.frames[0];
  f.dt = std::max(0.05, time);
  f.linear = false;
  f.has_arm.assign(map_->num_arm(), 1);
  f.arm = to.arm;
  for (int s = 0; s < kNumSide; ++s) {
    // 行き先が角度書きの脚なら、補間もそのまま角度で通す (足裏に落とすと
    // IK を通ることになって、角度で書いた意味が消える)。
    if (to.leg_mode[s] == LegMode::Servo && to.leg_servo_valid[s]) {
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
        f.has_leg[s][j] = 1;
        f.leg_servo[s][j] = to.leg_servo[s][j];
      }
    } else {
      f.has_foot[s] = true;
      f.foot[s] = to.foot[s];
    }
  }
  player_.start(blend_motion_, cur_pose_, home_pose_, now, *map_);
  reportPlayerWarning();
}

void MotionController::handleMotionRequest(const std::string & name, double now)
{
  if (name == kHoldMotion) {
    // その場保持。脱力中なら「次の武装は実測姿勢のまま」の予約、
    // トルクが入っていれば今の目標姿勢で止まる
    if (state_ == State::RELAX) {
      arm_in_place_ = true;
      ev_.info("その場保持を予約した (トルクが入っても動かず、実測姿勢を保持する)");
      return;
    }
    resetWalk();
    player_.stop();
    hold_pose_ = cur_pose_;
    setState(State::STAY);
    ev_.info("その場保持 (今の目標姿勢で止まる)");
    return;
  }
  if (name != "home" && !lib_->find(name)) {
    ev_.warn("知らない技 \"" + name + "\" (motions.yaml に無い)");
    return;
  }

  if (name == "home") {
    // ホームは「保持したい姿勢」を差し替えるのが本体。脱力中ならそれだけ。
    // トルクが入っていれば、そこへ home_move_time 秒かけて移る。
    hold_pose_ = home_pose_;
    arm_in_place_ = false;   // hold の予約は home で上書き
    if (state_ == State::RELAX) {
      ev_.info("ホーム姿勢を予約した (トルクが入ったらそこへ移る)");
      return;
    }
    resetWalk();
    // ★hold の武装中 (ARMING) に home が来ることがある。落とさずに補間へ入ると、
    //   終わった時点で HOLD ではなく STAY に落ちて立位のまま固まる (歩けなくなる)。
    stay_after_arm_ = false;
    startBlend(home_pose_, opt_.home_move_time, now, "home");
    setState(State::MOTION);
    ev_.info(fmt("ホームポジションへ (%.1fs)", opt_.home_move_time));
    return;
  }

  if (state_ == State::RELAX) {
    ev_.warn("脱力中なので技 \"" + name + "\" は出さない");
    return;
  }
  if (state_ == State::ARMING) {
    ev_.warn("立ち上げ中なので技 \"" + name + "\" は出さない");
    return;
  }
  if (state_ == State::WALK) {
    if (!opt_.motion_interrupts_walk) {
      ev_.warn("歩行中なので技 \"" + name + "\" は出さない");
      return;
    }
    ev_.warn("歩行を打ち切って技 \"" + name + "\" に入る");
  }
  resetWalk();
  // 技のあとは HOLD。いまは上の ARMING 弾きで stay_after_arm_ が立ったまま
  // ここへ来ることはないが、弾きを緩めたときに STAY へ落ちないよう対にしておく。
  stay_after_arm_ = false;
  player_.start(*lib_->find(name), cur_pose_, home_pose_, now, *map_);
  reportPlayerWarning();
  setState(State::MOTION);
  ev_.info(fmt("技 \"%s\" 再生 (%.2fs)", name.c_str(), player_.duration()));
}

void MotionController::resetWalk()
{
  walk_.reset();
  // 荷重の前送りも戻す。残したまま次に歩き出すと、その分だけ足先が跳ぶ。
  load_ff_state_.reset();
  for (int s = 0; s < kNumSide; ++s) {load_share_[s] = 0.0;}
}

void MotionController::tickWalk(double now, double dt)
{
  double vx = 0.0, vy = 0.0;
  {
    std::lock_guard<std::mutex> lk(walk_mtx_);
    // 指令が途絶えたらゼロを入れる。最後の指令を保持しない
    // (無線が切れたまま歩き続けるのがいちばん困る)。
    if (walk_stamp_ > 0.0 && now - walk_stamp_ <= opt_.cmd_timeout) {
      vx = walk_cmd_[0];
      vy = walk_cmd_[1];
    }
  }
  if (!opt_.walk_enable) {vx = vy = 0.0;}

  const rwc::WalkOutputs o = walk_.update(vx, vy, dt);
  walk_out_ = o;
  walk_ticked_ = true;

  // 世界座標 [m] -> 骨盤水平系 [m] -> Σ_U [mm]。計画の立位がホーム姿勢の足に
  // 重なるよう stance_off_ を足す (configure())。
  // 足裏の向きは**ホーム姿勢と同じ**にする。計画器は平行移動のみで機体は
  // 向きを変えないので、足裏の姿勢は歩行中も変わらない。ここを水平に固定すると、
  // home_pose.yaml の rpy（膝のしなりを補正するつま先上げ）が立位でしか効かず、
  // 歩き出した瞬間に機体が後傾する。立位と歩行で別々の値を持たない。
  // 組み立ては静歩行の到達域の門 (checkStaticWalkEnvelope) と共有する (walkFeet)。
  walkFeet(o, stance_off_, home_pose_, cur_pose_.foot);
  // 荷重で縮む分を先に伸ばして返す (load_ff.hpp)。既定 (sink = 0) では何もしない。
  // **門の後に足す**ので、伸ばした足先は下の 10Hz の見張りが見る。
  load_ff_state_.update(o, load_ff_, dt, cur_pose_.foot, load_share_);
  for (int s = 0; s < kNumSide; ++s) {
    // 歩行は足裏書きしか作らない。直前の技が角度書きで終わっていたら、ここで
    // 書き方を戻す (戻さないと足先を書き換えても指令は古いサーボ角のまま出る)。
    cur_pose_.leg_mode[s] = LegMode::Foot;
    cur_pose_.leg_servo_valid[s] = false;
  }
  // 腕は歩行では動かさない。直前の保持値をそのまま持ち越す。
  cur_pose_.arm = hold_pose_.arm;

  // 実際に出している足先が設計可動域を出ていないか見張る。gait.yaml が
  // 到達域から逆算されている前提が崩れる (v_max を手で上げた、z_c がずれた) と
  // ここに出る。指令自体は送る — 止めるほうが危ないので、記録だけ残す。
  if (++reach_tick_ >= static_cast<int>(opt_.loop_hz / 10.0)) {
    reach_tick_ = 0;
    for (int s = 0; s < kNumSide; ++s) {
      // design 域の外は起動時に断ってあるので騒がない。**機構として届かない**
      // ところへ行ったときだけ言う (そこは IK が解けず脚が止まる位相)。
      FootPose fb = cur_pose_.foot[s];
      bodyPitchApply(fb, body_pitch_);
      const ReachLevel lv = reachLevel(map_->leg_params(s), fb);
      if (lv < ReachLevel::Mech) {
        ev_.warn(
          fmt(
            "%s脚の足先が機構の到達域の外 (%s): p=[%.1f, %.1f, %.1f]",
            kSideTag[s], reachLevelName(lv), cur_pose_.foot[s].p.x,
            cur_pose_.foot[s].p.y, cur_pose_.foot[s].p.z), 2000,
            std::string("reach") + kSideTag[s]);
      }
    }
  }

  // [3] 歩き始めのばたつき止め（ヘッダ「順序の約束」）。
  const bool moving = (o.state != rwc::State::IDLE);
  if (moving) {
    idle_since_ = -1.0;
    if (state_ != State::WALK) {setState(State::WALK);}
  } else if (state_ == State::WALK) {
    if (idle_since_ < 0.0) {idle_since_ = now;}
    if (now - idle_since_ >= opt_.walk_idle_hold) {
      hold_pose_ = cur_pose_;
      setState(State::HOLD);
    }
  }
}

MotionController::Tick MotionController::step(
  double now, double dt, const BodyPose * measured, const std::string & why, bool torque_ready)
{
  const State entry = state_;
  Tick tick;
  walk_ticked_ = false;

  // --- 1) 技の要求を取り出す ---------------------------------------
  std::string req;
  {
    std::lock_guard<std::mutex> lk(req_mtx_);
    if (got_motion_) {
      req = motion_req_;
      got_motion_ = false;
      seen_motion_ = true;
    }
  }

  // --- 2) 技の要求を捌く。★武装の判定より 先（ヘッダ [1]）------------
  if (!req.empty()) {handleMotionRequest(req, now);}

  // --- 3) 脱力 (最優先。どの状態からでも即座に落ちる) ----------------
  if (estop_.load()) {
    if (state_ != State::RELAX) {
      setState(State::RELAX);
      player_.stop();
      resetWalk();
      arm_in_place_ = false;
      stay_after_arm_ = false;
    }
    // ★if の外に置く。RELAX のまま武装待ち (want_torque_ = true だが torque_ready が
    //   まだ) の最中に /estop true が来ると、内側に置いた版では state_ が既に RELAX
    //   なので落とし損ね、**脱力を指示しているのにサーボへ電気が入る。**
    //   実測が揃わずに武装待ちが長引くほど踏みやすい。
    want_torque_ = false;
  } else if (state_ == State::RELAX && canArm()) {
    // ★起点はこの周期の実測。取れない・経路が通らないなら武装しない（ヘッダ [2]）。
    bool checked = false;   //!< この周期にもう経路を確かめた（検査は数 ms かかる）
    if (!want_torque_) {
      if (!measured) {
        ev_.warn(
          "実測姿勢が取れないので武装しない: " + (why.empty() ? std::string("(理由不明)") : why),
          2000, "arm_no_measure");
      } else if (now >= arm_check_at_) {
        // hold は起点 = 行き先なので経路は無い（起点として使えるかは decode が見た）
        std::string pwhy;
        if (arm_in_place_ || armPathClear(*measured, pwhy)) {
          want_torque_ = true;
          checked = true;
        } else {
          arm_check_at_ = now + kArmRecheck;
          ev_.warn("武装しない: " + pwhy, 2000, "arm_path");
        }
      }
    }
    if (want_torque_ && torque_ready) {
      if (!measured) {
        // トルクは実測位置を目標にして入っている（servo_bank）ので、待っても動かない
        ev_.warn(
          "トルクは入ったが実測姿勢が取れないので補間を待つ: " + why, 1000, "arm_wait_measure");
      } else {
        cur_pose_ = *measured;
        std::string pwhy;
        if (arm_in_place_) {
          // hold: 実測姿勢をそのまま保持姿勢にする。補間距離ゼロ = その場で固まる
          arm_in_place_ = false;
          stay_after_arm_ = true;
          hold_pose_ = cur_pose_;
          startBlend(hold_pose_, opt_.hold_arm_time, now, "hold");
          setState(State::ARMING);
          ev_.info(
            fmt(
              "トルクオン (その場保持)。実測姿勢 R[%.1f, %.1f, %.1f] のまま動かない",
              cur_pose_.foot[kRight].p.x, cur_pose_.foot[kRight].p.y,
              cur_pose_.foot[kRight].p.z));
        } else if (!checked && !armPathClear(cur_pose_, pwhy)) {
          // 検査からトルクが入るまでの数 ms に姿勢が変わった。脱力して出直す
          want_torque_ = false;
          arm_check_at_ = now + kArmRecheck;
          ev_.warn("トルクは入ったが補間の経路が通らないので脱力に戻す: " + pwhy);
        } else {
          startBlend(hold_pose_, opt_.torque_on_time, now, "arming");
          setState(State::ARMING);
          ev_.info(
            fmt(
              "トルクオン。実測姿勢 R[%.1f, %.1f, %.1f] から %.1fs かけて保持姿勢へ"
              "（サーボ角で補間）", cur_pose_.foot[kRight].p.x, cur_pose_.foot[kRight].p.y,
              cur_pose_.foot[kRight].p.z, opt_.torque_on_time));
        }
      }
    }
  }
  // ここで want_torque_ を触らない場合は前周期の値を保つ。武装済み
  // (ARMING/HOLD/WALK/MOTION/STAY) は true のまま、まだ /cmd_motion を
  // 受けていない RELAX は false のまま、が正しい。

  // --- 4) 状態ごとに目標姿勢を作る ----------------------------------
  switch (state_) {
    case State::RELAX:
      // 脱力中は目標を作らない。手で動かされるので実測を追いかけておく
      // （表示用。武装の起点はその周期の実測を使う。ヘッダ [2]）。
      if (measured) {
        cur_pose_ = *measured;
      } else {
        ev_.warn("実測姿勢が取れない: " + why, 5000, "no_measure");
      }
      break;

    case State::ARMING:
    case State::MOTION:
      if (!player_.sample(now, cur_pose_)) {
        hold_pose_ = cur_pose_;
        if (stay_after_arm_) {
          // [4] その場保持の武装が終わった。HOLD にすると tickWalk が足先を立位の
          // スタンスへ上書きして跳ねるので、目標を作らない STAY で止める
          stay_after_arm_ = false;
          setState(State::STAY);
        } else {
          setState(State::HOLD);
        }
      }
      break;

    case State::HOLD:
    case State::WALK:
      tickWalk(now, dt);
      break;

    case State::STAY:
      // その場保持。目標は作らず cur_pose_ をそのまま出し続ける。
      // 抜けるのは技 (起き上がり) か home か脱力。/cmd_walk は効かない
      break;
  }

  tick.state = state_;
  tick.state_changed = (state_ != entry);
  tick.want_torque = want_torque_;
  // --- 5) RELAX では指令を出さない -----------------------------------
  tick.target = (state_ == State::RELAX) ? nullptr : &cur_pose_;
  return tick;
}

}  // namespace roboone_motion
