// 生カウント <-> サーボ角の対応を 1 か所にまとめる層。
//
// この機体には「角度」が 3 種類あって、混ぜると必ず事故る。
//
//   [1] 生カウント     0-4095。サーボが実際にやり取りする唯一の量
//   [2] T ポーズ基準角  servo_home.yaml の home を 0 deg とした角度。
//                      人間が読み書きする角度（feetech_calibrate_home の出力、
//                      leg_service の servo コマンド、モーション config の腕の角度）
//   [3] 絶対サーボ角    leg_servo.hpp の legServoFromJoints / legJointsFromServo が
//                      やり取りする角度。膝と足首は機構の原点が T ポーズと
//                      ずれているので [2] とは定数だけずれる
//
// [1] <-> [2] は素直:
//
//     T ポーズ基準角[deg] = (生カウント - home) * 360 / 4096
//
// [2] <-> [3] のずれ（このファイルの主眼）は軸ごとに次の定数:
//
//     J1-J3 股      0                                    （サーボ直結）
//     J4    膝      kneeServoFromCrank(knee, θ2_ext)     （伸び切り = T ポーズ）
//     J5    足首鎖0 ankle.servoHome[0]                    （ID6・短ロッド 80mm）
//     J6    足首鎖1 ankle.servoHome[1]                    （ID5・長ロッド 115mm）
//
// この表を tpose_ref[] と呼ぶ。leg_service.cpp の servo コマンドが同じ足し算を
// しているので、あちらと数値が食い違ったらどちらかが壊れている。
//
// ===========================================================================
// ID の並び
// ===========================================================================
// 脚 6 軸は Joint enum の順に {1, 2, 3, 4, 6, 5}。**足首は 6, 5 の順**で、
// ankle_parallel.hpp の鎖の添字（鎖 0 = 短ロッド = ID6）に合わせてある。
// ここを 5, 6 にすると足首のロールとピッチが入れ替わる。
//
// 腕（上半身）は運動学を持たない。servo_home.yaml に載っている ID のうち脚の
// 1-6 以外を全部そのまま拾う。今の実機は右バスに 7,8,9,10・左バスに 8,9,10 で
// 計 7 軸（ID7 は右半身にしか無い）。左に ID7 を足したら servo_home.yaml に
// 1 行足すだけでこのクラスは追従する。
#ifndef ROBOONE_MOTION_NODE__SERVO_MAP_HPP_
#define ROBOONE_MOTION_NODE__SERVO_MAP_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "roboone_kinematics/leg_servo.hpp"

namespace roboone_motion_node
{

namespace rk = roboone_kinematics;

//: バスの添字。0 = 右半身 / 1 = 左半身。配列の添字にそのまま使う。
constexpr int kRight = 0;
constexpr int kLeft = 1;
constexpr int kNumSide = 2;
inline constexpr const char * kSideTag[kNumSide] = {"R", "L"};

//: 脚 6 軸のサーボ ID。並びは rk::Joint enum（足首は 鎖0=ID6 / 鎖1=ID5）。
inline constexpr int kLegServoId[rk::kNumJoints] = {1, 2, 3, 4, 6, 5};

//: /joint_states に出す脚の関節名（kLegServoId と同順）。
inline constexpr const char * kLegJointName[rk::kNumJoints] = {
  "hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch", "ankle_roll"};

/// 腕（運動学を持たない上半身の軸）1 本。
struct ArmAxis
{
  int side = kRight;      //!< kRight / kLeft
  int id = 0;             //!< サーボ ID
  std::string name;       //!< "R7" のような表記。config と /joint_states で使う
  double sign = 1.0;      //!< 実機のサーボが逆に回る軸は -1（脚の AXIS_FLIP に相当）
};

/// 1 本のバスに載っている軸の一覧（書き込み・読み出しの単位）。
struct BusAxes
{
  std::string port;
  std::vector<uint8_t> ids;   //!< 脚 6 軸 → 腕の順。読み書きはこの順で 1 パケット
  int num_leg = 0;            //!< 先頭から何本が脚か（= rk::kNumJoints、脚が無ければ 0）
};

class ServoMap
{
public:
  /// servo_home.yaml と servo_limits.yaml を読む。失敗したら false + err に理由。
  ///
  /// limits_path が空、またはファイルが無い場合はリミット無しとして続ける
  /// （リミットは「保険」であって、これが無いと動かせないものではない）。
  /// invert には「実機のサーボが逆に回る」腕軸の名前を並べる（例 {"L9", "R10"}）。
  /// 脚は leg_config.hpp の AXIS_FLIP が同じ役目を持つが、腕は運動学が無いので
  /// ここで持つ。左右対称に組んだぶん出力軸の向きが反転する軸がある。
  bool load(
    const std::string & home_path, const std::string & limits_path,
    const std::string & port_right, const std::string & port_left,
    const std::vector<std::string> & invert, std::string & err);

  // --- 軸の一覧 -------------------------------------------------------
  const BusAxes & bus(int side) const {return bus_[side];}
  const std::vector<ArmAxis> & arms() const {return arms_;}
  std::size_t num_arm() const {return arms_.size();}
  /// 名前（"R7" 等）から arms() の添字を引く。無ければ -1。
  int arm_index(const std::string & name) const;

  // --- 脚: 生カウント <-> 絶対サーボ角 --------------------------------
  /// 生カウント -> 絶対サーボ角 [rad]（legJointsFromServo に渡せる値）。
  double leg_servo_from_count(int side, std::size_t j, int count) const;
  /// 絶対サーボ角 [rad] -> 生カウント。servo_limits.yaml の窓で丸める。
  /// clamped が非 null なら、丸めたときに true を書く。
  int leg_count_from_servo(int side, std::size_t j, double servo, bool * clamped = nullptr) const;

  // --- 腕: 生カウント <-> T ポーズ基準角 ------------------------------
  double arm_deg_from_count(std::size_t k, int count) const;
  int arm_count_from_deg(std::size_t k, double deg, bool * clamped = nullptr) const;

  /// 脚の T ポーズ基準角 [deg]（表示・ティーチ用）。
  double leg_tpose_deg_from_count(int side, std::size_t j, int count) const
  {
    return (count - leg_home_[side][j]) * kDegPerCount;
  }

  const rk::LegServoParams & leg_params(int side) const {return leg_[side];}

  /// 起動時のログに出す 1 行（原点ファイルの出どころと軸数）。
  const std::string & summary() const {return summary_;}

private:
  static constexpr double kCountPerDeg = 4096.0 / 360.0;
  static constexpr double kDegPerCount = 360.0 / 4096.0;

  struct Limit
  {
    int lo = 0, hi = 0;                 //!< lo == hi のときは「制限なし」
    bool active() const {return lo != hi;}
  };

  int clamp_count(int side, int id, int count, bool * clamped) const;

  rk::LegServoParams leg_[kNumSide]{};
  double tpose_ref_[kNumSide][rk::kNumJoints]{};   //!< [2] と [3] のずれ [rad]
  int leg_home_[kNumSide][rk::kNumJoints]{};       //!< 脚 6 軸の home カウント
  std::vector<ArmAxis> arms_;
  std::vector<int> arm_home_;                      //!< arms_ と同順
  BusAxes bus_[kNumSide];
  std::vector<Limit> limit_[kNumSide];             //!< bus_[side].ids と同順
  std::string summary_;
};

}  // namespace roboone_motion_node

#endif  // ROBOONE_MOTION_NODE__SERVO_MAP_HPP_
