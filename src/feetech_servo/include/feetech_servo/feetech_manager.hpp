// FeetechManager: 複数のシリアルバス（=複数コントローラ）を束ねる。
//
// 本構成ではコントローラ2台 = バス2本（例 /dev/ttyACM0, /dev/ttyACM1）。
// 各バスは独立オブジェクトなので、呼び出し側でバスごとに別スレッドを割り当てれば
// 2本を真に並列で回せる（デモ node がその形）。
#ifndef FEETECH_SERVO__FEETECH_MANAGER_HPP_
#define FEETECH_SERVO__FEETECH_MANAGER_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "feetech_servo/feetech_bus.hpp"

namespace feetech_servo
{

class FeetechManager
{
public:
  FeetechManager() = default;

  // バスを追加して開く。成功した FeetechBus* を返す（失敗時は nullptr）。
  FeetechBus * add_bus(
    const std::string & port, int baud = 1000000, uint8_t proto_end = 0, int timeout_ms = 20,
    Family family = Family::kHls);

  size_t bus_count() const { return buses_.size(); }
  FeetechBus * bus(size_t i) { return i < buses_.size() ? buses_[i].get() : nullptr; }

  // 全バスを閉じる。
  void close_all();

private:
  std::vector<std::unique_ptr<FeetechBus>> buses_;
};

}  // namespace feetech_servo

#endif  // FEETECH_SERVO__FEETECH_MANAGER_HPP_
