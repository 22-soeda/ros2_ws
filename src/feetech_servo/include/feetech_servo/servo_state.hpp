// Feetech SMS/STS サーボ 1軸ぶんの状態。
// 生カウント値と物理量の両方を持つ（feetech.py の ServoState と同一の意味づけ）。
#ifndef FEETECH_SERVO__SERVO_STATE_HPP_
#define FEETECH_SERVO__SERVO_STATE_HPP_

#include <cstdint>
#include <string>

namespace feetech_servo
{

// PRESENT_POSITION(56) から一括で読む状態ブロックのサイズ（56..70）
inline constexpr int kStateBlockSize = 15;

struct ServoState
{
  int id = 0;
  int pos = 0;        // 0-4095 = 0-360deg（符号付き: 多回転オフセット含む場合あり）
  int speed = 0;      // step/s
  int load = 0;       // 0.1% 単位（±1000 = ±100%）
  float volt = 0.0f;  // V
  int temp = 0;       // degC
  int current = 0;    // mA
  bool moving = false;
  uint8_t err = 0;    // エラービット（下記 ERR_* 参照）
  bool valid = false; // このサイクルで正しく読めたか

  // 0-4095 → 0-360deg
  float deg() const { return pos * 360.0f / 4096.0f; }
};

// エラービット（SMS/STS ステータス）
inline constexpr uint8_t kErrVoltage = 0x01;
inline constexpr uint8_t kErrAngle = 0x02;
inline constexpr uint8_t kErrOverheat = 0x04;
inline constexpr uint8_t kErrOvercurrent = 0x08;
inline constexpr uint8_t kErrOverload = 0x20;

// エラービットを人間可読な文字列にする（例 "過熱/過負荷"）
std::string err_str(uint8_t err);

}  // namespace feetech_servo

#endif  // FEETECH_SERVO__SERVO_STATE_HPP_
