// 左右の添字。**この 4 つだけの最小のヘッダ。**
//
// サーボ層 (servo_bank) は運動学も ServoMap も知らないが、左右の区別だけは要る。
// servo_map.hpp に置いたままだと、サーボ層が運動学ヘッダを引きずることになる
// （servo_map.hpp -> leg_servo.hpp -> roboone_kinematics 一式）。かといって
// サーボ層に別名で持たせると、値がずれたときに左右が入れ替わる。
#ifndef ROBOONE_MOTION__SIDE_HPP_
#define ROBOONE_MOTION__SIDE_HPP_

namespace roboone_motion
{

//: バスの添字。0 = 右半身 / 1 = 左半身。配列の添字にそのまま使う。
constexpr int kRight = 0;
constexpr int kLeft = 1;
constexpr int kNumSide = 2;
inline constexpr const char * kSideTag[kNumSide] = {"R", "L"};

}  // namespace roboone_motion

#endif  // ROBOONE_MOTION__SIDE_HPP_
