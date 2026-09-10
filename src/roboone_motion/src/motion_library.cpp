#include "roboone_motion/motion_library.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace roboone_motion
{

namespace
{

constexpr double kD2R = M_PI / 180.0;

/// 5 次多項式 s(τ) = 10τ³ - 15τ⁴ + 6τ⁵。両端で速度・加速度がゼロ。
/// walk_engine.hpp の detail::quintic と同じ式（あちらは遊脚の時間整形に使う）。
double quintic(double tau)
{
  tau = tau < 0.0 ? 0.0 : (tau > 1.0 ? 1.0 : tau);
  return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau));
}

bool readFoot(const YAML::Node & n, FootPose & out, std::string & err)
{
  if (n["p"]) {
    const YAML::Node & p = n["p"];
    if (!p.IsSequence() || p.size() != 3) {
      err = "p は [x, y, z] の 3 要素 [mm]";
      return false;
    }
    out.p = rk::Vec3{p[0].as<double>(), p[1].as<double>(), p[2].as<double>()};
  }
  if (n["rpy"]) {
    const YAML::Node & r = n["rpy"];
    if (!r.IsSequence() || r.size() != 3) {
      err = "rpy は [roll, pitch, yaw] の 3 要素 [deg]";
      return false;
    }
    for (int k = 0; k < 3; ++k) {
      out.rpy[k] = r[k].as<double>() * kD2R;
    }
  }
  return true;
}

/// ``R_leg: {ID4: 41.2}`` を読む。値は T ポーズ基準 [deg]、持つのは絶対サーボ角 [rad]。
///
/// 知らない軸名は落とさずに warn へ回す。腕（readArms 相当）と同じ扱いで、
/// config の書き損じ 1 つで全モーションが死ぬのを避ける。
bool readLegServo(
  const YAML::Node & n, const ServoMap & map, int side, const std::string & motion,
  KeyFrame & f, std::vector<std::string> & warn, std::string & err)
{
  if (!n.IsMap()) {
    err = "サーボ角の並び {ID1: deg, ...} で書く（ID1-ID4 と足首の ID6 / ID5）";
    return false;
  }
  for (const auto & kv : n) {
    const std::string key = kv.first.as<std::string>();
    // "ID4" でも "4" でも受ける。ティーチが吐くのは "ID4"
    std::string digits = key;
    if (digits.size() > 2 && (digits[0] == 'I' || digits[0] == 'i') &&
      (digits[1] == 'D' || digits[1] == 'd'))
    {
      digits = digits.substr(2);
    }
    int id = 0;
    try {
      id = std::stoi(digits);
    } catch (const std::exception &) {
      id = 0;
    }
    const int j = (id > 0) ? legIndexOfServoId(id) : -1;
    if (j < 0) {
      warn.push_back(
        "技 \"" + motion + "\": 脚に無い軸 \"" + key +
        "\" を無視した（脚は ID1-ID4 と足首の ID6 / ID5。腕は arms: へ）");
      continue;
    }
    const std::size_t jj = static_cast<std::size_t>(j);
    f.has_leg[side][jj] = 1;
    f.leg_servo[side][jj] = map.leg_servo_from_tpose_deg(side, jj, kv.second.as<double>());
  }
  return true;
}

}  // namespace

bool MotionLibrary::load(const std::string & path, const ServoMap & map, std::string & err)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    err = "モーション定義を読めない: " + path + " (" + e.what() + ")";
    return false;
  }
  if (!root["motions"] || !root["motions"].IsMap()) {
    err = path + ": トップレベルに motions: が要る";
    return false;
  }

  bool def_return = true;
  double def_time = 0.6;
  if (root["defaults"]) {
    const YAML::Node & d = root["defaults"];
    if (d["return_home"]) {def_return = d["return_home"].as<bool>();}
    if (d["return_time"]) {def_time = d["return_time"].as<double>();}
  }

  const std::size_t na = map.num_arm();

  for (const auto & kv : root["motions"]) {
    Motion m;
    m.name = kv.first.as<std::string>();
    const YAML::Node & mn = kv.second;
    m.return_home = mn["return_home"] ? mn["return_home"].as<bool>() : def_return;
    m.return_time = mn["return_time"] ? mn["return_time"].as<double>() : def_time;

    if (!mn["keyframes"] || !mn["keyframes"].IsSequence() || mn["keyframes"].size() == 0) {
      err = path + ": 技 \"" + m.name + "\" に keyframes: が無い（1 枚以上要る）";
      return false;
    }

    for (const auto & fn : mn["keyframes"]) {
      KeyFrame f;
      f.has_arm.assign(na, 0);
      f.arm.assign(na, 0.0);
      if (!fn["t"]) {
        err = path + ": 技 \"" + m.name + "\" のキーフレームに t: が無い";
        return false;
      }
      f.dt = fn["t"].as<double>();
      if (!(f.dt > 0.0)) {
        err = path + ": 技 \"" + m.name + "\" の t は正の秒数（0 は不可）";
        return false;
      }
      if (fn["ease"]) {f.linear = fn["ease"].as<std::string>() == "linear";}

      const char * key[kNumSide] = {"R_foot", "L_foot"};
      for (int s = 0; s < kNumSide; ++s) {
        if (!fn[key[s]]) {continue;}
        std::string e;
        if (!readFoot(fn[key[s]], f.foot[s], e)) {
          err = path + ": 技 \"" + m.name + "\" の " + key[s] + ": " + e;
          return false;
        }
        f.has_foot[s] = true;
      }

      // 脚をサーボ角で書いた側（ヘッダの「脚をサーボ角で書く」）。R_foot と同居
      // していたら、足裏で決めた姿勢の上から書いた軸だけ上書きする（start() 側）。
      const char * legkey[kNumSide] = {"R_leg", "L_leg"};
      for (int s = 0; s < kNumSide; ++s) {
        if (!fn[legkey[s]]) {continue;}
        std::string e;
        if (!readLegServo(fn[legkey[s]], map, s, m.name, f, warnings_, e)) {
          err = path + ": 技 \"" + m.name + "\" の " + legkey[s] + ": " + e;
          return false;
        }
      }

      if (fn["arms"]) {
        for (const auto & an : fn["arms"]) {
          const std::string name = an.first.as<std::string>();
          const int k = map.arm_index(name);
          if (k < 0) {
            // 落とさずに警告に留める。servo_home.yaml に無い軸を config に書いた
            // だけで全モーションが死ぬと、実機で 1 軸抜いたときに何も動かせなくなる。
            warnings_.push_back(
              "技 \"" + m.name + "\": 知らない腕軸 \"" + name + "\" を無視した");
            continue;
          }
          f.has_arm[k] = 1;
          f.arm[k] = an.second.as<double>();
        }
      }
      m.frames.push_back(std::move(f));
    }
    motions_.push_back(std::move(m));
  }

  // ホーム姿勢の原本は home_pose.yaml。ここに書かれていたら二重定義
  // なので捨てる。片方だけ直して食い違うのがいちばん困る。
  const auto it = std::remove_if(
    motions_.begin(), motions_.end(), [](const Motion & m) {return m.name == "home";});
  if (it != motions_.end()) {
    motions_.erase(it, motions_.end());
    warnings_.push_back(
      "motions.yaml の \"home\" は無視した（ホーム姿勢の原本は home_pose.yaml）");
  }

  summary_.clear();
  for (const auto & m : motions_) {
    if (!summary_.empty()) {summary_ += ", ";}
    summary_ += m.name + "(" + std::to_string(m.frames.size()) + ")";
  }
  return true;
}

const Motion * MotionLibrary::find(const std::string & name) const
{
  for (const auto & m : motions_) {
    if (m.name == name) {return &m;}
  }
  return nullptr;
}

// ---------------------------------------------------------------------------

BodyPose blendPose(const BodyPose & a, const BodyPose & b, double u, bool linear)
{
  const double s = linear ? (u < 0.0 ? 0.0 : (u > 1.0 ? 1.0 : u)) : quintic(u);
  BodyPose out = a;
  for (int k = 0; k < kNumSide; ++k) {
    // 足裏は常に混ぜておく。角度空間で回す区間ではこれは指令に使われない影だが、
    // ログと「技のあとの起点」がそれらしい値であってほしいので置いていく。
    out.foot[k].p = a.foot[k].p + (b.foot[k].p - a.foot[k].p) * s;
    for (int i = 0; i < 3; ++i) {
      out.foot[k].rpy[i] = a.foot[k].rpy[i] + (b.foot[k].rpy[i] - a.foot[k].rpy[i]) * s;
    }

    // 片端でも角度書きなら角度空間で混ぜる（ヘッダ「書き方が混ざる区間」）。
    const bool want_servo =
      (a.leg_mode[k] == LegMode::Servo || b.leg_mode[k] == LegMode::Servo);
    if (want_servo && a.leg_servo_valid[k] && b.leg_servo_valid[k]) {
      out.leg_mode[k] = LegMode::Servo;
      out.leg_servo_valid[k] = true;
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
        out.leg_servo[k][j] =
          a.leg_servo[k][j] + (b.leg_servo[k][j] - a.leg_servo[k][j]) * s;
      }
    } else if (want_servo) {
      // 角度書きなのに影が揃っていない（土台を IK で解けなかった枚。start() が
      // 警告を出している）。**足裏へ落としてはいけない**——引き継いだ古い (p, R) を
      // IK で送ることになって、その瞬間に跳ねる。指令を出さない側へ倒す
      // （writeTargets が前周期の指令を保つ）。
      out.leg_mode[k] = LegMode::Servo;
      out.leg_servo_valid[k] = false;
    } else {
      out.leg_mode[k] = LegMode::Foot;
      out.leg_servo_valid[k] = false;
    }
  }
  const std::size_t n = std::min(a.arm.size(), b.arm.size());
  out.arm.assign(a.arm.begin(), a.arm.end());
  for (std::size_t i = 0; i < n; ++i) {
    out.arm[i] = a.arm[i] + (b.arm[i] - a.arm[i]) * s;
  }
  return out;
}

void MotionPlayer::start(
  const Motion & m, const BodyPose & from, const BodyPose & home, double now,
  const ServoMap & map)
{
  pose_.clear();
  t_.clear();
  linear_.clear();
  warning_.clear();

  pose_.push_back(from);
  t_.push_back(0.0);

  // 「書かなかった項目は引き継ぐ」をここで畳む。起点は再生開始時点の実際の姿勢。
  BodyPose cur = from;
  for (const auto & f : m.frames) {
    for (int s = 0; s < kNumSide; ++s) {
      if (f.has_foot[s]) {
        cur.foot[s] = f.foot[s];
        cur.leg_mode[s] = LegMode::Foot;
        cur.leg_servo_valid[s] = false;      // 影は下の一括で作り直す
      }
      if (!f.anyLegServo(s)) {continue;}

      // 角度書き。引き継ぐ土台がまだ足裏書きなら、まず IK でサーボ角に直す。
      // 同じ枚に R_foot と R_leg を両方書いたときもここを通るので、「足裏で
      // 姿勢を決めてから、書いた軸だけ上書きする」になる。
      bool base_ok = (cur.leg_mode[s] == LegMode::Servo && cur.leg_servo_valid[s]);
      if (!base_ok) {base_ok = fillLegServoFromFoot(map.leg_params(s), cur, s);}
      bool all_axes = true;
      for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
        if (f.has_leg[s][j]) {
          cur.leg_servo[s][j] = f.leg_servo[s][j];
        } else {
          all_axes = false;
        }
      }
      cur.leg_mode[s] = LegMode::Servo;
      // 6 軸そろって書いてあれば土台は要らない。欠けている軸を引き継げないときだけ困る。
      cur.leg_servo_valid[s] = base_ok || all_axes;
      if (!cur.leg_servo_valid[s]) {
        warning_ += std::string(kSideTag[s]) +
          "脚: 角度書きが引き継ぐ土台の姿勢を IK で解けない（欠けている軸を埋められない）。"
          "その脚は指令を出さない。6 軸すべて書くか、ひとつ前を届く姿勢にすること ";
      } else {
        // 足裏側を FK で置き直す（表示・次の技の起点用。指令には使わない）。
        fillFootFromLegServo(map.leg_params(s), cur, s);
      }
    }
    for (std::size_t k = 0; k < cur.arm.size() && k < f.has_arm.size(); ++k) {
      if (f.has_arm[k]) {cur.arm[k] = f.arm[k];}
    }
    pose_.push_back(cur);
    t_.push_back(t_.back() + f.dt);
    linear_.push_back(f.linear ? 1 : 0);
  }

  if (m.return_home && m.return_time > 0.0) {
    pose_.push_back(home);
    t_.push_back(t_.back() + m.return_time);
    linear_.push_back(0);
  }

  // 足裏書きの枚にもサーボ角の影を作っておく。書き方が混ざる区間を角度空間で
  // 回すのに要る値で、**ここで作らないと再生中に IK を解くことになる**
  // （200Hz の中で解かない）。枚数はたかだか数枚なので、隣を見ずに全部作る。
  for (auto & p : pose_) {
    for (int s = 0; s < kNumSide; ++s) {
      if (p.leg_mode[s] == LegMode::Foot && !p.leg_servo_valid[s]) {
        fillLegServoFromFoot(map.leg_params(s), p, s);
      }
    }
  }

  name_ = m.name;
  t0_ = now;
  active_ = true;
}

bool MotionPlayer::sample(double now, BodyPose & out)
{
  if (pose_.size() < 2) {
    active_ = false;
    if (!pose_.empty()) {out = pose_.back();}
    return false;
  }
  const double t = now - t0_;
  if (t >= t_.back()) {
    out = pose_.back();
    active_ = false;
    return false;
  }
  // 区間を線形に探す。キーフレームは数枚なので二分探索の値打ちが無い。
  std::size_t k = 0;
  while (k + 2 < t_.size() && t >= t_[k + 1]) {++k;}
  const double span = t_[k + 1] - t_[k];
  const double u = span > 0.0 ? (t - t_[k]) / span : 1.0;
  out = blendPose(pose_[k], pose_[k + 1], u, linear_[k] != 0);
  return true;
}

}  // namespace roboone_motion
