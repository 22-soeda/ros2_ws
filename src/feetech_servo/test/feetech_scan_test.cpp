// feetech_scan_test: 1Mbps のバス上にいるサーボを総当たりで探し、見つかったIDの情報を全部出す。
//
// ROS を使わない素のCLIツール（実機がないと意味がないので ament のテストには載せない）。
//
//   ros2 run feetech_servo feetech_scan_test                      # 既定2ポート, ID 0..253
//   ros2 run feetech_servo feetech_scan_test --port /dev/ttyACM0  # ポート指定（複数可）
//   ros2 run feetech_servo feetech_scan_test --id-max 30          # 探索範囲を狭めて高速化
//
// 各IDについて EEPROM（型番/ファーム/モード/ボーレート/角度リミット/オフセット等）と
// 現在状態（位置・速度・負荷・電圧・温度・電流・エラー）を読んで表にする。
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include <scservo/SMS_STS.h>

#include "feetech_servo/feetech_bus.hpp"

using feetech_servo::FeetechBus;
using feetech_servo::ServoState;

namespace
{

// EEPROM の BAUD_RATE レジスタ値 → 実ボーレート
const char * baud_name(int reg)
{
  switch (reg) {
    case SMS_STS_1M: return "1M";
    case SMS_STS_0_5M: return "500k";
    case SMS_STS_250K: return "250k";
    case SMS_STS_128K: return "128k";
    case SMS_STS_115200: return "115200";
    case SMS_STS_76800: return "76800";
    case SMS_STS_57600: return "57600";
    case SMS_STS_38400: return "38400";
    default: return "?";
  }
}

const char * mode_name(int mode)
{
  switch (mode) {
    case SMS_STS_MODE_SERVO: return "位置";
    case SMS_STS_MODE_WHEEL_CLOSED: return "速度(閉)";
    case SMS_STS_MODE_WHEEL_OPEN: return "PWM(開)";
    case SMS_STS_MODE_STEPPER: return "ステッパ";
    default: return "?";
  }
}

// 1軸ぶんの読み取り結果。-1 は読めなかったことを示す。
struct ServoInfo
{
  int id = 0;
  int model = -1;
  int fw_major = -1;
  int fw_minor = -1;
  int mode = -1;
  int baud_reg = -1;
  int resp_level = -1;
  int min_angle = -1;
  int max_angle = -1;
  int ofs = -1;
  int torque = -1;
  int goal = -1;
  int max_temp = -1;
  int goal_torque = -1;   // HLS系では reg44/45 = GOAL_TORQUE（SMS/STS では GOAL_TIME）
  int torque_limit = -1;
  ServoState st;
};

ServoInfo read_info(FeetechBus & bus, uint8_t id)
{
  ServoInfo info;
  info.id = id;
  info.model = bus.read_word(id, SMS_STS_MODEL_L);
  info.fw_major = bus.read_byte(id, SMS_STS_FIRMWARE_VER_L);
  info.fw_minor = bus.read_byte(id, SMS_STS_FIRMWARE_VER_H);
  info.mode = bus.read_byte(id, SMS_STS_MODE);
  info.baud_reg = bus.read_byte(id, SMS_STS_BAUD_RATE);
  info.resp_level = bus.read_byte(id, SMS_STS_RESPONSE_STATUS_LEVEL);
  info.min_angle = bus.read_word(id, SMS_STS_MIN_ANGLE_LIMIT_L);
  info.max_angle = bus.read_word(id, SMS_STS_MAX_ANGLE_LIMIT_L);
  info.ofs = bus.read_word(id, SMS_STS_OFS_L);
  info.torque = bus.read_byte(id, SMS_STS_TORQUE_ENABLE);
  info.goal = bus.read_word(id, SMS_STS_GOAL_POSITION_L);
  info.max_temp = bus.read_byte(id, SMS_STS_MAX_TEMPERATURE_LIMIT);
  info.goal_torque = bus.read_word(id, SMS_STS_GOAL_TIME_L);  // HLS系なら GOAL_TORQUE
  info.torque_limit = bus.read_word(id, SMS_STS_TORQUE_LIMIT_L);
  info.st = bus.read_state(id);
  return info;
}

void print_field(const char * label, int v, const char * suffix = "")
{
  if (v < 0) {
    std::printf("  %-14s : (読めず)\n", label);
  } else {
    std::printf("  %-14s : %d%s\n", label, v, suffix);
  }
}

// 1ポートを走査して、見つかった軸数を返す。
int scan_port(const std::string & port, int baud, int id_min, int id_max, int timeout_ms)
{
  std::printf("\n================ %s (baud=%d) ================\n", port.c_str(), baud);

  FeetechBus bus(port, baud, /*proto_end=*/0, timeout_ms);
  if (!bus.open()) {
    std::printf("  ポートを開けない（未接続 / dialout権限なし）。スキップ\n");
    return 0;
  }

  // --- 総当たり ping ---
  std::printf("  ID %d..%d を ping 中", id_min, id_max);
  std::fflush(stdout);
  std::vector<uint8_t> found;
  for (int id = id_min; id <= id_max; ++id) {
    if (id == SMS_STS_BROADCAST_ID) {
      continue;  // 0xFE はブロードキャスト。応答しない
    }
    if (bus.ping(static_cast<uint8_t>(id))) {
      found.push_back(static_cast<uint8_t>(id));
      std::printf("\n    → ID %d 応答あり", id);
      std::fflush(stdout);
    }
  }
  std::printf("\n  検出 %zu 軸: ", found.size());
  for (size_t i = 0; i < found.size(); ++i) {
    std::printf("%s%d", i ? ", " : "", found[i]);
  }
  std::printf("\n");

  if (found.empty()) {
    std::printf("  （このポートには %d bps で応答する軸がない）\n", baud);
    return 0;
  }

  // --- 一覧表 ---
  std::printf(
    "\n  %-4s %-7s %-7s %-9s %-7s %-6s %-8s %-6s %-6s %-6s %-6s %s\n",
    "ID", "型番", "FW", "モード", "ボーレート", "トルク", "位置", "角度", "電圧", "温度", "電流", "エラー");
  std::printf("  %s\n", std::string(96, '-').c_str());
  std::vector<ServoInfo> infos;
  for (uint8_t id : found) {
    ServoInfo in = read_info(bus, id);
    infos.push_back(in);
    char pos[16], deg[16], volt[16], temp[16], cur[16];
    if (in.st.valid) {
      std::snprintf(pos, sizeof(pos), "%d", in.st.pos);
      std::snprintf(deg, sizeof(deg), "%.1f", in.st.deg());
      std::snprintf(volt, sizeof(volt), "%.1f", in.st.volt);
      std::snprintf(temp, sizeof(temp), "%d", in.st.temp);
      std::snprintf(cur, sizeof(cur), "%d", in.st.current);
    } else {
      std::snprintf(pos, sizeof(pos), "-");
      std::snprintf(deg, sizeof(deg), "-");
      std::snprintf(volt, sizeof(volt), "-");
      std::snprintf(temp, sizeof(temp), "-");
      std::snprintf(cur, sizeof(cur), "-");
    }
    char fw[16];
    std::snprintf(fw, sizeof(fw), "%d.%d", in.fw_major, in.fw_minor);
    std::printf(
      "  %-4d %-7d %-7s %-9s %-7s %-6s %-8s %-6s %-6s %-6s %-6s %s\n",
      in.id, in.model, fw, mode_name(in.mode), baud_name(in.baud_reg),
      in.torque < 0 ? "?" : (in.torque ? "ON" : "OFF"),
      pos, deg, volt, temp, cur,
      in.st.valid ? feetech_servo::err_str(in.st.err).c_str() : "読めず");
  }

  // --- 軸ごとの詳細 ---
  for (const ServoInfo & in : infos) {
    std::printf("\n  --- ID %d 詳細 ---\n", in.id);
    print_field("型番(model)", in.model);
    std::printf("  %-14s : %d.%d\n", "ファーム", in.fw_major, in.fw_minor);
    std::printf("  %-14s : %d (%s)\n", "動作モード", in.mode, mode_name(in.mode));
    std::printf("  %-14s : %d (%s)\n", "ボーレート設定", in.baud_reg, baud_name(in.baud_reg));
    print_field("応答レベル", in.resp_level);
    std::printf("  %-14s : %d .. %d\n", "角度リミット", in.min_angle, in.max_angle);
    print_field("位置オフセット", in.ofs);
    print_field("温度上限", in.max_temp, " degC");
    std::printf("  %-14s : %s\n", "トルク", in.torque < 0 ? "?" : (in.torque ? "ON" : "OFF"));
    print_field("目標位置", in.goal);
    print_field("目標トルク", in.goal_torque);   // HLS系: 0 だと駆動しない
    print_field("トルク上限", in.torque_limit);
    if (in.st.valid) {
      std::printf("  %-14s : %d (%.2f deg)\n", "現在位置", in.st.pos, in.st.deg());
      print_field("現在速度", in.st.speed, " step/s");
      std::printf("  %-14s : %.1f %%\n", "負荷", in.st.load / 10.0f);
      std::printf("  %-14s : %.1f V\n", "電圧", in.st.volt);
      print_field("温度", in.st.temp, " degC");
      print_field("電流", in.st.current, " mA");
      std::printf("  %-14s : %s\n", "動作中", in.st.moving ? "yes" : "no");
      std::printf(
        "  %-14s : 0x%02X %s\n", "エラービット", in.st.err, feetech_servo::err_str(in.st.err).c_str());
    } else {
      std::printf("  %-14s : (現在状態を読めず)\n", "状態");
    }
  }

  std::printf(
    "\n  通信統計: tx=%llu, rx_fail=%llu\n",
    static_cast<unsigned long long>(bus.tx_count()),
    static_cast<unsigned long long>(bus.rx_fail()));
  return static_cast<int>(found.size());
}

void usage(const char * argv0)
{
  std::printf(
    "使い方: %s [--port PORT]... [--baud N] [--id-min N] [--id-max N] [--timeout MS]\n"
    "  --port     走査するシリアルポート（複数指定可。既定: /dev/ttyACM0, /dev/ttyACM1）\n"
    "  --baud     ボーレート（既定 1000000）\n"
    "  --id-min   探索開始ID（既定 0）\n"
    "  --id-max   探索終了ID（既定 253）\n"
    "  --timeout  1トランザクションの受信タイムアウト ms（既定 20）\n",
    argv0);
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> ports;
  int baud = 1000000;
  int id_min = 0;
  int id_max = 253;
  int timeout_ms = 20;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](int & out) {
        if (i + 1 >= argc) {
          std::fprintf(stderr, "%s に値がない\n", a.c_str());
          std::exit(2);
        }
        out = std::atoi(argv[++i]);
      };
    if (a == "--port" && i + 1 < argc) {
      ports.push_back(argv[++i]);
    } else if (a == "--baud") {
      next(baud);
    } else if (a == "--id-min") {
      next(id_min);
    } else if (a == "--id-max") {
      next(id_max);
    } else if (a == "--timeout") {
      next(timeout_ms);
    } else if (a == "-h" || a == "--help") {
      usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "不明な引数: %s\n", a.c_str());
      usage(argv[0]);
      return 2;
    }
  }
  if (ports.empty()) {
    ports = {"/dev/ttyACM0", "/dev/ttyACM1"};
  }
  if (id_min < 0) {
    id_min = 0;
  }
  if (id_max > 253) {
    id_max = 253;
  }

  std::printf("Feetech サーボ列挙テスト (baud=%d, ID %d..%d)\n", baud, id_min, id_max);
  int total = 0;
  for (const std::string & port : ports) {
    total += scan_port(port, baud, id_min, id_max, timeout_ms);
  }
  std::printf("\n合計 %d 軸を検出（%zu ポート）\n", total, ports.size());
  return total > 0 ? 0 : 1;
}
