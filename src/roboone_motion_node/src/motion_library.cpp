#include "roboone_motion_node/motion_library.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace roboone_motion_node
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
    for (int k = 0; k < 3; ++k) {out.rpy[k] = r[k].as<double>() * kD2R;}
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
    out.foot[k].p = a.foot[k].p + (b.foot[k].p - a.foot[k].p) * s;
    for (int i = 0; i < 3; ++i) {
      out.foot[k].rpy[i] = a.foot[k].rpy[i] + (b.foot[k].rpy[i] - a.foot[k].rpy[i]) * s;
    }
  }
  const std::size_t n = std::min(a.arm.size(), b.arm.size());
  out.arm.assign(a.arm.begin(), a.arm.end());
  for (std::size_t i = 0; i < n; ++i) {out.arm[i] = a.arm[i] + (b.arm[i] - a.arm[i]) * s;}
  return out;
}

void MotionPlayer::start(
  const Motion & m, const BodyPose & from, const BodyPose & home, double now)
{
  pose_.clear();
  t_.clear();
  linear_.clear();

  pose_.push_back(from);
  t_.push_back(0.0);

  // 「書かなかった項目は引き継ぐ」をここで畳む。起点は再生開始時点の実際の姿勢。
  BodyPose cur = from;
  for (const auto & f : m.frames) {
    for (int s = 0; s < kNumSide; ++s) {
      if (f.has_foot[s]) {cur.foot[s] = f.foot[s];}
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

}  // namespace roboone_motion_node
