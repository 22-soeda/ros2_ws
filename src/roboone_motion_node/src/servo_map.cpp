#include "roboone_motion_node/servo_map.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

namespace roboone_motion_node
{

namespace
{

/// 伸び切り（曲げ量 0 = T ポーズ）に対応するクランク角 θ2_ext。
/// leg_service.cpp の kneeCrankAtExtension() と同じもの。片方だけ直さないこと。
double kneeCrankAtExtension(const rk::KneeParams & knee, bool & ok)
{
  rk::KneePose pose;
  ok = rk::kneeIk(knee, rk::kneeRockerFromBend(knee, 0.0), pose) == rk::KneeStatus::Ok;
  return pose.theta2;
}

}  // namespace

bool ServoMap::load(
  const std::string & home_path, const std::string & limits_path,
  const std::string & port_right, const std::string & port_left,
  const std::vector<std::string> & invert, std::string & err)
{
  const std::string port[kNumSide] = {port_right, port_left};

  // --- servo_home.yaml -------------------------------------------------
  YAML::Node root;
  try {
    root = YAML::LoadFile(home_path);
  } catch (const std::exception & e) {
    err = "原点ファイルを読めない: " + home_path + " (" + e.what() + ")";
    return false;
  }

  std::map<std::string, std::map<int, int>> home;   // port -> (id -> home count)
  for (const auto & b : root["buses"]) {
    if (!b["port"] || !b["servos"]) {continue;}
    const std::string p = b["port"].as<std::string>();
    for (const auto & kv : b["servos"]) {
      const YAML::Node & v = kv.second;
      if (v.IsMap() && v["home"]) {home[p][kv.first.as<int>()] = v["home"].as<int>();}
    }
  }

  // --- 脚 6 軸 ---------------------------------------------------------
  for (int s = 0; s < kNumSide; ++s) {
    bus_[s].port = port[s];
    const auto it = home.find(port[s]);
    if (it == home.end()) {
      err = home_path + " に " + port[s] + " が無い";
      return false;
    }
    const auto & hmap = it->second;
    for (std::size_t j = 0; j < rk::kNumJoints; ++j) {
      const auto h = hmap.find(kLegServoId[j]);
      if (h == hmap.end()) {
        err = port[s] + " の ID " + std::to_string(kLegServoId[j]) + " の原点が無い";
        return false;
      }
      leg_home_[s][j] = h->second;
      bus_[s].ids.push_back(static_cast<uint8_t>(kLegServoId[j]));
    }
    bus_[s].num_leg = static_cast<int>(rk::kNumJoints);

    leg_[s] = rk::makeLegServoParams(s == kRight ? rk::Side::RIGHT : rk::Side::LEFT);

    // T ポーズ基準角 [2] と絶対サーボ角 [3] のずれ。leg_service.cpp と同じ表。
    bool ok = true;
    const double t2ext = kneeCrankAtExtension(leg_[s].knee, ok);
    if (!ok) {
      err = std::string(kSideTag[s]) + ": 膝の伸び切り姿勢が解けない (knee_config を確認)";
      return false;
    }
    tpose_ref_[s][rk::HIP_PITCH] = 0.0;
    tpose_ref_[s][rk::HIP_ROLL] = 0.0;
    tpose_ref_[s][rk::HIP_YAW] = 0.0;
    tpose_ref_[s][rk::KNEE] = rk::kneeServoFromCrank(leg_[s].knee, t2ext);
    tpose_ref_[s][rk::ANKLE_PITCH] = leg_[s].ankle.servoHome[0];
    tpose_ref_[s][rk::ANKLE_ROLL] = leg_[s].ankle.servoHome[1];
  }

  // --- 腕（脚以外の全軸）-----------------------------------------------
  // servo_home.yaml に載っている ID のうち 1-6 以外を拾う。ID 順に並べる。
  for (int s = 0; s < kNumSide; ++s) {
    for (const auto & kv : home.at(port[s])) {
      const int id = kv.first;
      if (id >= 1 && id <= 6) {continue;}
      ArmAxis a;
      a.side = s;
      a.id = id;
      a.name = std::string(kSideTag[s]) + std::to_string(id);
      for (const auto & inv : invert) {
        if (inv == a.name) {a.sign = -1.0;}
      }
      arms_.push_back(a);
      arm_home_.push_back(kv.second);
      bus_[s].ids.push_back(static_cast<uint8_t>(id));
    }
  }

  // --- servo_limits.yaml（無くても続ける）-------------------------------
  int nlimit = 0;
  for (int s = 0; s < kNumSide; ++s) {limit_[s].assign(bus_[s].ids.size(), Limit{});}
  if (!limits_path.empty()) {
    try {
      const YAML::Node lroot = YAML::LoadFile(limits_path);
      std::map<std::string, std::map<int, Limit>> lim;
      for (const auto & b : lroot["buses"]) {
        if (!b["port"] || !b["servos"]) {continue;}
        const std::string p = b["port"].as<std::string>();
        for (const auto & kv : b["servos"]) {
          const YAML::Node & v = kv.second;
          Limit L;
          if (v.IsSequence() && v.size() == 2) {
            L.lo = v[0].as<int>();
            L.hi = v[1].as<int>();
          } else if (v.IsMap() && v["min"] && v["max"]) {
            L.lo = v["min"].as<int>();
            L.hi = v["max"].as<int>();
          } else {
            continue;
          }
          // 下限 > 上限 は不正。ファームが目標位置をその窓に丸めて軸が張り付く
          // （feetech_angle_limit_clamp の実機事例）。信じずに「制限なし」に落とす。
          if (L.lo > L.hi) {continue;}
          lim[p][kv.first.as<int>()] = L;
        }
      }
      for (int s = 0; s < kNumSide; ++s) {
        const auto it = lim.find(port[s]);
        if (it == lim.end()) {continue;}
        for (std::size_t k = 0; k < bus_[s].ids.size(); ++k) {
          const auto e = it->second.find(bus_[s].ids[k]);
          if (e != it->second.end()) {
            limit_[s][k] = e->second;
            if (e->second.active()) {++nlimit;}
          }
        }
      }
    } catch (const std::exception &) {
      // リミットが読めなくても動かせる。summary に出して呼び側に判断させる。
    }
  }

  std::string inv_list;
  for (const auto & a : arms_) {
    if (a.sign < 0.0) {inv_list += (inv_list.empty() ? "" : ",") + a.name;}
  }
  char buf[320];
  std::snprintf(
    buf, sizeof(buf),
    "原点 %s / 脚 %d軸x2 + 腕 %zu軸 (リミット有効 %d軸, 回転方向を反転した腕軸 %s)",
    home_path.c_str(), static_cast<int>(rk::kNumJoints), arms_.size(), nlimit,
    inv_list.empty() ? "なし" : inv_list.c_str());
  summary_ = buf;
  return true;
}

int ServoMap::arm_index(const std::string & name) const
{
  for (std::size_t k = 0; k < arms_.size(); ++k) {
    if (arms_[k].name == name) {return static_cast<int>(k);}
  }
  return -1;
}

double ServoMap::leg_servo_from_count(int side, std::size_t j, int count) const
{
  const double tpose_deg = (count - leg_home_[side][j]) * kDegPerCount;
  return tpose_ref_[side][j] + tpose_deg * M_PI / 180.0;
}

int ServoMap::leg_count_from_servo(int side, std::size_t j, double servo, bool * clamped) const
{
  const double tpose_deg = (servo - tpose_ref_[side][j]) * 180.0 / M_PI;
  const int raw = leg_home_[side][j] + static_cast<int>(std::lround(tpose_deg * kCountPerDeg));
  return clamp_count(side, kLegServoId[j], raw, clamped);
}

double ServoMap::arm_deg_from_count(std::size_t k, int count) const
{
  return (count - arm_home_[k]) * kDegPerCount * arms_[k].sign;
}

int ServoMap::arm_count_from_deg(std::size_t k, double deg, bool * clamped) const
{
  const int raw =
    arm_home_[k] + static_cast<int>(std::lround(deg * arms_[k].sign * kCountPerDeg));
  return clamp_count(arms_[k].side, arms_[k].id, raw, clamped);
}

int ServoMap::clamp_count(int side, int id, int count, bool * clamped) const
{
  for (std::size_t k = 0; k < bus_[side].ids.size(); ++k) {
    if (bus_[side].ids[k] != id) {continue;}
    const Limit & L = limit_[side][k];
    if (!L.active()) {break;}
    if (count < L.lo) {
      if (clamped) {*clamped = true;}
      return L.lo;
    }
    if (count > L.hi) {
      if (clamped) {*clamped = true;}
      return L.hi;
    }
    break;
  }
  return count;
}

}  // namespace roboone_motion_node
