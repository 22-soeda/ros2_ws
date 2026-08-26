#include "feetech_servo/feetech_manager.hpp"

namespace feetech_servo
{

FeetechBus * FeetechManager::add_bus(
  const std::string & port, int baud, uint8_t proto_end, int timeout_ms, Family family)
{
  auto bus = std::make_unique<FeetechBus>(port, baud, proto_end, timeout_ms, family);
  if (!bus->open()) {
    return nullptr;
  }
  FeetechBus * raw = bus.get();
  buses_.push_back(std::move(bus));
  return raw;
}

void FeetechManager::close_all()
{
  for (auto & b : buses_) {
    b->close();
  }
}

}  // namespace feetech_servo
