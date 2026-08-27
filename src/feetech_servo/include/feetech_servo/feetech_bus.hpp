// FeetechBus: 1本のシリアルバス（=コントローラ1台）を担当するラッパ。
//
// 公式 SCServo SDK (SMS_STS) を内部に持ち、閉ループ制御の2つの要
//   - 書き込み: SyncWritePosEx で全軸を1パケット送信
//   - 読み出し: syncReadPacketTx/Rx で全軸の状態を1往復で取得
// を提供する。1インスタンス=1ポートなので、複数バスは複数インスタンスで扱う。
//
// スレッド安全性: SMS_STS は内部にTX/RXバッファと同期読みバッファを持つため、
// 1つのバスへの同時アクセスは不可。本クラスは内部 mutex で各操作を直列化する。
// 別ポートの FeetechBus 同士は独立オブジェクトなので並列に回してよい。
#ifndef FEETECH_SERVO__FEETECH_BUS_HPP_
#define FEETECH_SERVO__FEETECH_BUS_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "feetech_servo/servo_state.hpp"

// 前方宣言（SDK ヘッダを公開ヘッダに漏らさない = pimpl 的に隠蔽）
class SMS_STS;

namespace feetech_servo
{

// サーボ系列。位置指令パケット(reg41..47)の 44/45 の意味が系列で違うため区別が必要。
//   SMS/STS : 44/45 = GOAL_TIME   → 0 を書く（速度指定で動く）
//   HLS     : 44/45 = GOAL_TORQUE → 0 を書くと目標トルク0＝まったく駆動しない
// 本構成の実機（model 4618 / 5130, FW 3.43）は HLS 系なので既定を kHls にしている。
// SMS/STS を繋ぐ場合は kSmsSts を指定すること（HLS 用の値を書くと GOAL_TIME になり誤動作する）。
enum class Family : uint8_t
{
  kSmsSts,
  kHls,
};

// 動作モード（SMS_STS のモードと一致）
enum class Mode : uint8_t
{
  kPosition = 0,     // サーボ（位置制御）
  kWheelClosed = 1,  // ホイール閉ループ（速度制御）
  kWheelOpen = 2,    // ホイール開ループ（PWM）
};

class FeetechBus
{
public:
  // port 例: "/dev/ttyACM0"、baud 例: 1000000。End=0 は SMS/STS のリトルエンディアン。
  // timeout_ms: 1トランザクションの受信タイムアウト。軸が欠けたとき最大この時間だけ待つ。
  //   SDK既定は100msだが、応答欠損時のループ停滞を避けるため既定20ms（Python版と同値）。
  FeetechBus(
    std::string port, int baud = 1000000, uint8_t proto_end = 0, int timeout_ms = 20,
    Family family = Family::kHls);
  ~FeetechBus();

  FeetechBus(const FeetechBus &) = delete;
  FeetechBus & operator=(const FeetechBus &) = delete;

  // シリアルポートを開く。成功で true。以降の操作の前に呼ぶこと。
  bool open();
  void close();
  bool is_open() const { return is_open_; }

  const std::string & port() const { return port_; }
  Family family() const { return family_; }

  // HLS 系のときに位置指令へ載せる目標トルク（0-1000）。0 だと駆動しない。
  // SMS/STS 系では無視され、常に GOAL_TIME=0 が書かれる。
  void set_goal_torque(uint16_t t) { goal_torque_ = t; }
  uint16_t goal_torque() const { return goal_torque_; }

  // --- 探索 ---
  bool ping(uint8_t id);
  std::vector<uint8_t> scan(const std::vector<uint8_t> & ids);

  // --- 初期化 ---
  // EEPROMアンロック→モード設定→ロック→トルクON/OFF（SDKのInitMotor）。成功で true。
  bool init_motor(uint8_t id, Mode mode = Mode::kPosition, bool enable_torque = true);
  bool enable_torque(uint8_t id, bool enable);

  // --- 書き込み（閉ループの前半）---
  // 全軸の目標位置を1パケットで送る。speeds/accs は空なら 0 として扱う。
  // ids と positions は同数。speeds/accs は空 or ids と同数。
  void sync_write_position(
    const std::vector<uint8_t> & ids,
    const std::vector<int16_t> & positions,
    const std::vector<uint16_t> & speeds = {},
    const std::vector<uint8_t> & accs = {});

  // 単軸書き込み（即時）。成功で true。
  bool write_position(uint8_t id, int16_t position, uint16_t speed = 0, uint8_t acc = 0);

  // --- 読み出し（閉ループの後半）---
  // 全軸の状態を1往復で読む（SyncRead, 15バイトブロック）。
  // out は ids と同順・同数で埋まる（読めなかった軸は valid=false）。
  // 戻り値: 正しく読めた軸数。
  int sync_read_states(const std::vector<uint8_t> & ids, std::vector<ServoState> & out);

  // 単軸の状態を読む（FeedBack 経由）。読めなければ valid=false。
  ServoState read_state(uint8_t id);

  // --- 生レジスタ読み（EEPROM の型番・モード等を覗く用）---
  // 失敗時は -1。addr は SMS_STS_* の定義値（vendor/scservo/include/scservo/SMS_STS.h）。
  int read_byte(uint8_t id, uint8_t addr);
  int read_word(uint8_t id, uint8_t addr);

  // --- 生レジスタ書き（設定の修正用。EEPROM 領域は unlock_eeprom で挟むこと）---
  // EEPROM(addr < 40) は書き込み回数に上限があるので、ループの中で呼ばないこと。
  bool write_byte(uint8_t id, uint8_t addr, uint8_t value);
  bool write_word(uint8_t id, uint8_t addr, uint16_t value);

  // EEPROM のロック解除/再ロック（LOCK レジスタ 55）。unlock=true で書き込み可になる。
  // 設定変更後は必ず false に戻すこと（意図しない書き換えを防ぐ）。
  bool unlock_eeprom(uint8_t id, bool unlock);

  // --- 統計（通信品質の把握用）---
  uint64_t tx_count() const { return tx_count_; }
  uint64_t rx_fail() const { return rx_fail_; }

private:
  // 位置指令ブロック(ACC,POS_L,POS_H,X_L,X_H,SPD_L,SPD_H)を組んで reg41 から書く。
  // X は系列で意味が変わる（Family 参照）。呼び出し側は mtx_ を保持していること。
  void build_pos_block(uint8_t * out, int16_t position, uint16_t speed, uint8_t acc) const;

  std::string port_;
  int baud_;
  uint8_t proto_end_;
  int timeout_ms_;
  Family family_;
  uint16_t goal_torque_ = 1000;
  bool is_open_ = false;

  std::unique_ptr<SMS_STS> sm_;       // 公式 SDK 本体
  mutable std::mutex mtx_;            // このバスの操作を直列化
  uint8_t sync_read_ids_ = 0;        // syncReadBegin で確保済みの軸数（再確保判定用）

  uint64_t tx_count_ = 0;
  uint64_t rx_fail_ = 0;
};

}  // namespace feetech_servo

#endif  // FEETECH_SERVO__FEETECH_BUS_HPP_
