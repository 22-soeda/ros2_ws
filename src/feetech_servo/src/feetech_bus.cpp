#include "feetech_servo/feetech_bus.hpp"

#include <algorithm>
#include <cmath>

#include <scservo/SCServo.h>

namespace feetech_servo
{

namespace
{
// Feetech は符号を最上位ビットに置く「符号絶対値」表現（2の補数ではない）。
int sign_mag(int v, int sign_bit)
{
  const int mask = 1 << sign_bit;
  if (v & mask) {
    return -(v & ~mask);
  }
  return v;
}

// 符号絶対値へエンコード（sign_mag の逆）
uint16_t to_sign_mag(int16_t v, int sign_bit)
{
  if (v < 0) {
    return static_cast<uint16_t>(-v) | static_cast<uint16_t>(1u << sign_bit);
  }
  return static_cast<uint16_t>(v);
}

// 16bit をプロトコルのバイト順で 2バイトに置く（SCS::Host2SCS は protected なので自前）
void put16(uint8_t * p, uint16_t v, uint8_t proto_end)
{
  if (proto_end) {
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v & 0xFF);
  } else {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>(v >> 8);
  }
}
}  // namespace

std::string err_str(uint8_t err)
{
  if (err == 0) {
    return "";
  }
  std::string s;
  const std::pair<uint8_t, const char *> bits[] = {
    {kErrVoltage, "電圧"}, {kErrAngle, "角度"}, {kErrOverheat, "過熱"},
    {kErrOvercurrent, "過電流"}, {kErrOverload, "過負荷"},
  };
  for (const auto & [bit, name] : bits) {
    if (err & bit) {
      if (!s.empty()) {
        s += "/";
      }
      s += name;
    }
  }
  return s;
}

FeetechBus::FeetechBus(
  std::string port, int baud, uint8_t proto_end, int timeout_ms, Family family)
: port_(std::move(port)), baud_(baud), proto_end_(proto_end), timeout_ms_(timeout_ms),
  family_(family), sm_(std::make_unique<SMS_STS>(proto_end))
{
}

// 位置指令の 7バイトブロック（reg41 ACC から連続）。
// SDK の SMS_STS::WritePosEx は 44/45 に必ず 0 を書くが、HLS 系ではそこが GOAL_TORQUE
// なので目標トルク0＝無出力になる（実機で位置指令が一切効かない症状の原因）。
// そのため SDK の高レベル関数を使わず、ここでブロックを組んで genWrite/syncWrite する。
void FeetechBus::build_pos_block(
  uint8_t * out, int16_t position, uint16_t speed, uint8_t acc) const
{
  out[0] = acc;
  put16(out + 1, to_sign_mag(position, 15), proto_end_);
  put16(out + 3, family_ == Family::kHls ? goal_torque_ : 0, proto_end_);  // GOAL_TORQUE / GOAL_TIME
  put16(out + 5, speed, proto_end_);
}

FeetechBus::~FeetechBus()
{
  close();
}

bool FeetechBus::open()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (is_open_) {
    return true;
  }
  if (!sm_->begin(baud_, port_.c_str())) {
    return false;
  }
  sm_->IOTimeOut = timeout_ms_;  // begin() が 100ms に設定するので上書き
  is_open_ = true;
  return true;
}

void FeetechBus::close()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (!is_open_) {
    return;
  }
  if (sync_read_ids_ > 0) {
    sm_->syncReadEnd();
    sync_read_ids_ = 0;
  }
  sm_->end();
  is_open_ = false;
}

bool FeetechBus::ping(uint8_t id)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ++tx_count_;
  const bool ok = sm_->Ping(id) != -1;
  if (!ok) {
    ++rx_fail_;
  }
  return ok;
}

std::vector<uint8_t> FeetechBus::scan(const std::vector<uint8_t> & ids)
{
  std::vector<uint8_t> alive;
  for (uint8_t id : ids) {
    if (ping(id)) {
      alive.push_back(id);
    }
  }
  return alive;
}

bool FeetechBus::init_motor(uint8_t id, Mode mode, bool enable_torque)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ++tx_count_;
  return sm_->InitMotor(id, static_cast<uint8_t>(mode), enable_torque ? 1 : 0) == 1;
}

bool FeetechBus::enable_torque(uint8_t id, bool enable)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ++tx_count_;
  return sm_->EnableTorque(id, enable ? 1 : 0) == 1;
}

void FeetechBus::sync_write_position(
  const std::vector<uint8_t> & ids,
  const std::vector<int16_t> & positions,
  const std::vector<uint16_t> & speeds,
  const std::vector<uint8_t> & accs)
{
  if (ids.empty() || ids.size() != positions.size()) {
    return;
  }
  const uint8_t n = static_cast<uint8_t>(ids.size());

  // SDK は非const配列を要求するのでローカルにコピー。speeds/accs が空なら 0 で埋める。
  std::vector<uint8_t> id_buf(ids.begin(), ids.end());
  std::vector<int16_t> pos_buf(positions.begin(), positions.end());
  std::vector<uint16_t> spd_buf(n, 0);
  std::vector<uint8_t> acc_buf(n, 0);
  if (speeds.size() == ids.size()) {
    spd_buf.assign(speeds.begin(), speeds.end());
  }
  if (accs.size() == ids.size()) {
    acc_buf.assign(accs.begin(), accs.end());
  }

  std::lock_guard<std::mutex> lk(mtx_);
  std::vector<uint8_t> block(static_cast<size_t>(n) * 7);
  for (uint8_t i = 0; i < n; ++i) {
    build_pos_block(block.data() + static_cast<size_t>(i) * 7, pos_buf[i], spd_buf[i], acc_buf[i]);
  }
  ++tx_count_;
  sm_->syncWrite(id_buf.data(), n, SMS_STS_ACC, block.data(), 7);
}

bool FeetechBus::write_position(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
{
  std::lock_guard<std::mutex> lk(mtx_);
  uint8_t block[7];
  build_pos_block(block, position, speed, acc);
  ++tx_count_;
  const bool ok = sm_->genWrite(id, SMS_STS_ACC, block, 7) == 1;
  if (!ok) {
    ++rx_fail_;
  }
  return ok;
}

int FeetechBus::sync_read_states(const std::vector<uint8_t> & ids, std::vector<ServoState> & out)
{
  out.assign(ids.size(), ServoState{});
  for (size_t i = 0; i < ids.size(); ++i) {
    out[i].id = ids[i];
  }
  if (ids.empty()) {
    return 0;
  }

  std::lock_guard<std::mutex> lk(mtx_);

  // 軸数が変わったら syncRead バッファを確保し直す。
  const uint8_t n = static_cast<uint8_t>(ids.size());
  if (sync_read_ids_ != n) {
    if (sync_read_ids_ > 0) {
      sm_->syncReadEnd();
    }
    sm_->syncReadBegin(n, kStateBlockSize);
    sync_read_ids_ = n;
  }

  // 1往復で全軸へ要求 → 各応答を受信バッファに取り込む。
  std::vector<uint8_t> id_buf(ids.begin(), ids.end());
  sm_->syncReadPacketTx(id_buf.data(), n, SMS_STS_PRESENT_POSITION_L, kStateBlockSize);
  ++tx_count_;

  int ok = 0;
  uint8_t block[kStateBlockSize];
  for (size_t i = 0; i < ids.size(); ++i) {
    if (sm_->syncReadPacketRx(ids[i], block) == 0) {
      ++rx_fail_;
      continue;  // out[i].valid は false のまま
    }
    ServoState & s = out[i];
    // ブロック配置は feetech.py の ServoState と同一（addr56 起点）。
    s.pos = sign_mag(block[0] | (block[1] << 8), 15);
    s.speed = sign_mag(block[2] | (block[3] << 8), 15);
    s.load = sign_mag(block[4] | (block[5] << 8), 10);
    s.volt = block[6] * 0.1f;
    s.temp = block[7];
    s.err = block[9];
    s.moving = block[10] != 0;
    s.current = static_cast<int>(std::lround(sign_mag(block[13] | (block[14] << 8), 15) * 6.5));
    s.valid = true;
    ++ok;
  }
  return ok;
}

ServoState FeetechBus::read_state(uint8_t id)
{
  ServoState s;
  s.id = id;
  std::lock_guard<std::mutex> lk(mtx_);
  ++tx_count_;
  if (sm_->FeedBack(id) != 1) {
    ++rx_fail_;
    return s;
  }
  s.pos = sm_->ReadPos(-1);
  s.speed = sm_->ReadSpeed(-1);
  s.load = sm_->ReadLoad(-1);
  s.volt = sm_->ReadVoltage(-1) * 0.1f;
  s.temp = sm_->ReadTemper(-1);
  s.moving = sm_->ReadMove(-1) == 1;
  s.current = sm_->ReadCurrent(-1);
  s.valid = true;
  return s;
}

int FeetechBus::read_byte(uint8_t id, uint8_t addr)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ++tx_count_;
  const int v = sm_->readByte(id, addr);
  if (v == -1) {
    ++rx_fail_;
  }
  return v;
}

int FeetechBus::read_word(uint8_t id, uint8_t addr)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ++tx_count_;
  const int v = sm_->readWord(id, addr);
  if (v == -1) {
    ++rx_fail_;
  }
  return v;
}

bool FeetechBus::write_byte(uint8_t id, uint8_t addr, uint8_t value)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ++tx_count_;
  const bool ok = sm_->writeByte(id, addr, value) == 1;
  if (!ok) {
    ++rx_fail_;
  }
  return ok;
}

bool FeetechBus::write_word(uint8_t id, uint8_t addr, uint16_t value)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ++tx_count_;
  const bool ok = sm_->writeWord(id, addr, value) == 1;
  if (!ok) {
    ++rx_fail_;
  }
  return ok;
}

bool FeetechBus::unlock_eeprom(uint8_t id, bool unlock)
{
  std::lock_guard<std::mutex> lk(mtx_);
  ++tx_count_;
  const bool ok = (unlock ? sm_->unLockEeprom(id) : sm_->LockEeprom(id)) == 1;
  if (!ok) {
    ++rx_fail_;
  }
  return ok;
}

}  // namespace feetech_servo
