#include "roboone_motion/servo_bank.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <utility>
#include <vector>

namespace roboone_motion
{

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;

namespace
{
std::string tag(int side) {return std::string(kSideTag[side]);}
}  // namespace

bool ServoBank::open(const BankPort port[kNumSide], const BankOptions & opt, std::string & err)
{
  opt_ = opt;
  int opened = 0;
  for (int s = 0; s < kNumSide; ++s) {
    port_[s] = port[s];
    torque_on_[s].store(false);
    if (opt_.dry_run) {
      ev_.warn("dry_run: " + port_[s].dev + " を開かない");
      continue;
    }
    auto b = std::make_unique<FeetechBus>(
      port_[s].dev, opt_.baud, 0, 20, feetech_servo::Family::kHls);
    if (!b->open()) {
      ev_.error(port_[s].dev + " を開けない (udev 固定名・電源・dialout 権限を確認)");
      continue;
    }
    // HLS 系は位置指令の 44/45 が GOAL_TORQUE。0 のままだと全軸まったく動かない。
    b->set_goal_torque(static_cast<uint16_t>(opt_.goal_torque));
    const auto alive = b->scan(port_[s].ids);
    if (alive.size() != port_[s].ids.size()) {
      ev_.warn(
        tag(s) + ": 応答 " + std::to_string(alive.size()) + "/" +
        std::to_string(port_[s].ids.size()) + " 軸。欠けた軸は指令が届かない");
    }
    bus_[s] = std::move(b);
    ++opened;
  }
  if (opt_.dry_run) {return true;}
  if (opened == 0) {
    err = "どちらのバスも開けなかった";
    return false;
  }
  return true;
}

int ServoBank::numOpened() const
{
  int n = 0;
  for (int s = 0; s < kNumSide; ++s) {
    n += bus_[s] ? 1 : 0;
  }
  return n;
}

void ServoBank::start()
{
  if (running_.exchange(true)) {return;}
  for (int s = 0; s < kNumSide; ++s) {
    if (bus_[s]) {thread_[s] = std::thread([this, s] {busLoop(s);});}
  }
}

void ServoBank::stop()
{
  if (!running_.exchange(false)) {return;}
  for (int s = 0; s < kNumSide; ++s) {
    if (thread_[s].joinable()) {thread_[s].join();}
  }
  // 落ちるときは必ず脱力を置いていく。トルクを入れたまま消えると、機体は
  // 最後の目標角で固まったまま誰も止められなくなる。
  for (int s = 0; s < kNumSide; ++s) {
    if (!bus_[s] || opt_.dry_run) {continue;}
    for (uint8_t id : port_[s].ids) {
      bus_[s]->enable_torque(id, false);
    }
    bus_[s]->close();
  }
}

void ServoBank::relax()
{
  for (int s = 0; s < kNumSide; ++s) {
    if (!bus_[s] || opt_.dry_run) {continue;}
    int n = 0;
    for (uint8_t id : port_[s].ids) {
      n += bus_[s]->enable_torque(id, false) ? 1 : 0;
    }
    torque_on_[s].store(false);
    ev_.info(
      tag(s) + ": " + std::to_string(n) + "/" + std::to_string(port_[s].ids.size()) +
      " 軸をトルク OFF（脱力）。手で動かせる。");
  }
}

bool ServoBank::torqueReady() const
{
  for (int s = 0; s < kNumSide; ++s) {
    if (bus_[s] && !torque_on_[s].load()) {return false;}
  }
  return true;      // dry_run はバスが無いので即 true
}

void ServoBank::setTargets(int side, std::vector<int16_t> counts)
{
  std::lock_guard<std::mutex> lk(cmd_mtx_[side]);
  cmd_[side] = std::move(counts);
  cmd_valid_[side] = true;
}

bool ServoBank::targets(int side, std::vector<int16_t> & out) const
{
  std::lock_guard<std::mutex> lk(cmd_mtx_[side]);
  if (!cmd_valid_[side]) {return false;}
  out = cmd_[side];
  return true;
}

void ServoBank::states(int side, std::vector<ServoState> & out) const
{
  std::lock_guard<std::mutex> lk(st_mtx_[side]);
  out = state_[side];
}

BankCurrent ServoBank::takeCurrent(int side)
{
  std::lock_guard<std::mutex> lk(st_mtx_[side]);
  BankCurrent out = cur_[side];
  std::fill(cur_[side].peak.begin(), cur_[side].peak.end(), 0);
  std::fill(cur_[side].sum.begin(), cur_[side].sum.end(), 0);
  std::fill(cur_[side].n.begin(), cur_[side].n.end(), 0);
  return out;
}

/// 電流を軸ごとに積む。**st_mtx_[side] を持ったまま呼ぶこと。**
///
/// 読み出しは read_hz なのに publish はもっと遅い。publish した瞬間の値だけを出すと
/// 大半を捨てることになり、**踏ん張った瞬間や接触時の突入電流をそのまま取りこぼす。**
/// 区間の最大と平均を持ち回して出す。押し引きの向きは servo_states の負荷 (符号付き)
/// で見られるので、ここは絶対値。
void ServoBank::accumCurrent(int side, const std::vector<ServoState> & st)
{
  const std::size_t n = port_[side].ids.size();
  if (cur_[side].peak.size() != n) {
    cur_[side].peak.assign(n, 0);
    cur_[side].sum.assign(n, 0);
    cur_[side].n.assign(n, 0);
  }
  for (std::size_t k = 0; k < n && k < st.size(); ++k) {
    if (!st[k].valid) {continue;}
    const int a = st[k].current < 0 ? -st[k].current : st[k].current;
    cur_[side].peak[k] = std::max(cur_[side].peak[k], a);
    cur_[side].sum[k] += a;
    ++cur_[side].n[k];
  }
}

/// トルクオンの 3 段（ヘッダ「トルクの入れ方」）。入れられたら true。
bool ServoBank::armSide(
  int side, const std::vector<uint16_t> & speeds, const std::vector<uint8_t> & accs)
{
  const auto & ids = port_[side].ids;
  std::vector<ServoState> now;
  bus_[side]->sync_read_states(ids, now);            // 1) 実測位置を読む
  std::vector<int16_t> hold(ids.size(), 0);
  bool all = true;
  for (std::size_t k = 0; k < ids.size(); ++k) {
    if (k < now.size() && now[k].valid) {
      hold[k] = static_cast<int16_t>(now[k].pos);
    } else {
      all = false;
    }
  }
  if (!all) {
    // 実測が揃わないままトルクを入れると、読めなかった軸だけが古い目標角へ飛ぶ。
    // **揃うまで入れない。** 低電圧だと応答が間欠的に欠ける実機の癖があるので、
    // ここで止まるときはまず電源を疑う。
    ev_.warn(
      tag(side) + ": 実測が揃わないのでトルクを入れない (電圧を確認)", 1000, "arm_wait" + tag(side));
    return false;
  }
  // 2) その実測位置を目標として書く → 3) トルクを入れる。この順でないと、
  //    レジスタに残っている古い目標角へ全速で飛ぶ。
  bus_[side]->sync_write_position(ids, hold, speeds, accs);
  int n = 0;
  for (uint8_t id : ids) {
    n += bus_[side]->enable_torque(id, true) ? 1 : 0;
  }
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_[side]);
    cmd_[side] = hold;              // control 側が上書きするまでの当座の目標
    cmd_valid_[side] = true;
  }
  torque_on_[side].store(true);
  ev_.info(
    tag(side) + ": 実測位置を目標にしてトルクオン (" + std::to_string(n) + "/" +
    std::to_string(ids.size()) + " 軸)");
  return true;
}

// =====================================================================
// バススレッド (1 本ずつ。書き込み loop_hz / 読み出し read_hz)
// =====================================================================
void ServoBank::busLoop(int s)
{
  const auto period = std::chrono::duration<double>(1.0 / opt_.loop_hz);
  auto next = std::chrono::steady_clock::now();
  const int read_div =
    std::max(1, static_cast<int>(opt_.loop_hz / std::max(1.0, opt_.read_hz)));
  const auto & ids = port_[s].ids;
  const std::vector<uint16_t> speeds(ids.size(), static_cast<uint16_t>(opt_.move_speed));
  const std::vector<uint8_t> accs(ids.size(), static_cast<uint8_t>(opt_.move_acc));
  std::vector<ServoState> st;
  int tick = 0;

  while (running_) {
    next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);

    // --- 読み出し (先にやる。トルクを入れる前に実測が要るため) ----------
    if (tick++ % read_div == 0) {
      st.clear();
      bus_[s]->sync_read_states(ids, st);
      std::lock_guard<std::mutex> lk(st_mtx_[s]);
      state_[s] = st;
      accumCurrent(s, st);
    }

    const bool want = want_torque_.load();

    // --- トルクを入れる / 切る ------------------------------------------
    if (want && !torque_on_[s].load() && !opt_.allow_torque) {
      // 禁止中。「入ったことにして」制御ループだけ進める。位置指令も送らないので
      // 機体は動かないが、状態機械・IK・モーション再生は実機の実測値を使って
      // 最後まで通る。
      torque_on_[s].store(true);
      ev_.warn(
        tag(s) + ": allow_torque:=false のためトルクを入れない (指令は計算するが送らない)");
    } else if (want && !torque_on_[s].load()) {
      armSide(s, speeds, accs);
    } else if (!want && torque_on_[s].load()) {
      // 禁止中でも「切る」側は必ず通す (入っていないものを切っても害はない)。
      for (uint8_t id : ids) {
        bus_[s]->enable_torque(id, false);
      }
      torque_on_[s].store(false);
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_[s]);
        cmd_valid_[s] = false;
      }
      ev_.warn(tag(s) + ": トルクオフ (脱力)");
    }

    // --- 位置指令 -------------------------------------------------------
    if (torque_on_[s].load() && opt_.allow_torque) {
      std::vector<int16_t> pos;
      {
        std::lock_guard<std::mutex> lk(cmd_mtx_[s]);
        if (cmd_valid_[s]) {pos = cmd_[s];}
      }
      if (pos.size() == ids.size()) {
        bus_[s]->sync_write_position(ids, pos, speeds, accs);
      }
    }

    std::this_thread::sleep_until(next);
    // 何かで大きく遅れたら次の周期に合わせ直す (取り戻そうとして暴走させない)
    const auto t = std::chrono::steady_clock::now();
    if (t > next + std::chrono::milliseconds(50)) {next = t;}
  }
}

}  // namespace roboone_motion
