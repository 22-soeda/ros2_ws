// feetech_limits: 全軸のトルク上限・電流上限まわりを「読むだけ」のツール。
//
//   ros2 run feetech_servo feetech_limits              # 両バス、ID 1..20 を総当たり
//   ros2 run feetech_servo feetech_limits --ids 1,2,3  # ID を指定
//   ros2 run feetech_servo feetech_limits --only left  # 片側だけ
//   ros2 run feetech_servo feetech_limits --csv        # 表ではなく CSV で出す
//
// ===========================================================================
// このツールは絶対に書き込まない
// ===========================================================================
// 呼ぶのは open() / ping() / read_byte() / read_word() だけ。enable_torque() も
// write_* も unlock_eeprom() も一切呼ばないので、**トルクが切れたまま実行できる**。
// トルクが入っている状態で実行しても、状態を変えない（40番は読むだけ）。
//
// ===========================================================================
// 何を読むか
// ===========================================================================
// トルク上限は 2 段構えになっている:
//   16/17 MAX_TORQUE       EEPROM。トルクON時に 48/49 へコピーされる「電源投入時の値」
//   48/49 TORQUE_LIMIT     SRAM。実際に効いている上限（0-1000 = 0-100%）
//   44/45 GOAL_TORQUE      HLS 系のみ。位置指令に載る目標トルク（0 だと動かない）
// 電流・保護系は EEPROM 側にある:
//   28/29 PROTECTION_CURRENT          過電流保護のしきい値（LSB = 6.5mA）
//   38    OVER_CURRENT_PROTECTION_TIME 上を超えてよい時間（LSB = 10ms）
//   34    PROTECTIVE_TORQUE           保護に入った後に出し続けるトルク(%)
//   35    PROTECTION_TIME             過負荷を許容する時間（LSB = 10ms）
//   36    OVERLOAD_TORQUE             過負荷と判定するトルク（実機は 255 が入っており
//                                     HLS で % かどうか不明。生値のまま出す）
//   24/25 MINIMUM_STARTUP_FORCE       起動に必要な最小出力
//   13/14/15 温度上限・入力電圧の上下限
//
// 注意: アドレス定義は SMS/STS 系のもの（SMS_STS.h）。実機は HLS 系で、
// 44/45 の意味だけは違う（GOAL_TIME ではなく GOAL_TORQUE）ことが確認済み。
// 28/29・34・35・36・38 が HLS でも同じ位置かはベンダのメモリテーブルで
// 突き合わせること。読めない/明らかにおかしい値は "-" と出る。
#include <scservo/SMS_STS.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "feetech_servo/feetech_bus.hpp"

using feetech_servo::FeetechBus;

namespace
{

std::vector<int> parse_ids(const std::string & s)
{
  std::vector<int> out;
  std::size_t p = 0;
  while (p < s.size()) {
    std::size_t c = s.find(',', p);
    if (c == std::string::npos) {c = s.size();}
    if (c > p) {out.push_back(std::atoi(s.substr(p, c - p).c_str()));}
    p = c + 1;
  }
  return out;
}

// 1軸ぶんの読み取り結果。-1 は読めなかったことを示す。
struct Limits
{
  int id = 0;
  int model = -1;           // 3/4   型番（4618 / 5130 など）
  int torque_enable = -1;   // 40    0=OFF 1=ON
  int max_torque = -1;      // 16/17 0-1000
  int torque_limit = -1;    // 48/49 0-1000
  int goal_torque = -1;     // 44/45 HLS のみ意味を持つ
  int prot_current = -1;    // 28/29 LSB=6.5mA
  int overcur_time = -1;    // 38    LSB=10ms
  int prot_torque = -1;     // 34    %
  int prot_time = -1;       // 35    LSB=10ms
  int overload_torque = -1; // 36    %
  int startup_force = -1;   // 24/25
  int temp_limit = -1;      // 13    ℃
  int volt_max = -1;        // 14    LSB=0.1V
  int volt_min = -1;        // 15    LSB=0.1V
  int now_volt = -1;        // 62    LSB=0.1V
  int now_temp = -1;        // 63    ℃
  int now_current = -1;     // 69/70 LSB=6.5mA
};

Limits read_one(FeetechBus & bus, int id)
{
  const uint8_t u = static_cast<uint8_t>(id);
  Limits r;
  r.id = id;
  r.model = bus.read_word(u, SMS_STS_MODEL_L);
  r.torque_enable = bus.read_byte(u, SMS_STS_TORQUE_ENABLE);
  r.max_torque = bus.read_word(u, SMS_STS_MAX_TORQUE_L);
  r.torque_limit = bus.read_word(u, SMS_STS_TORQUE_LIMIT_L);
  r.goal_torque = bus.read_word(u, SMS_STS_GOAL_TIME_L);  // HLS では GOAL_TORQUE
  r.prot_current = bus.read_word(u, SMS_STS_PROTECTION_CURRENT_L);
  r.overcur_time = bus.read_byte(u, SMS_STS_OVER_CURRENT_PROTECTION_TIME);
  r.prot_torque = bus.read_byte(u, SMS_STS_PROTECTIVE_TORQUE);
  r.prot_time = bus.read_byte(u, SMS_STS_PROTECTION_TIME);
  r.overload_torque = bus.read_byte(u, SMS_STS_OVERLOAD_TORQUE);
  r.startup_force = bus.read_word(u, SMS_STS_MINIMUM_STARTUP_FORCE_L);
  r.temp_limit = bus.read_byte(u, SMS_STS_MAX_TEMPERATURE_LIMIT);
  r.volt_max = bus.read_byte(u, SMS_STS_MAX_INPUT_VOLT);
  r.volt_min = bus.read_byte(u, SMS_STS_MIN_INPUT_VOLT);
  r.now_volt = bus.read_byte(u, SMS_STS_PRESENT_VOLTAGE);
  r.now_temp = bus.read_byte(u, SMS_STS_PRESENT_TEMPERATURE);
  r.now_current = bus.read_word(u, SMS_STS_PRESENT_CURRENT_L);
  return r;
}

// -1（読めず）を "-" に、そうでなければ書式化して返す小道具。
std::string fmt(int v, const char * suffix = "")
{
  if (v < 0) {return "-";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%d%s", v, suffix);
  return buf;
}

std::string fmt_pct1000(int v)  // 0-1000 を % 付きで
{
  if (v < 0) {return "-";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%d(%.1f%%)", v, v / 10.0);
  return buf;
}

std::string fmt_ma(int v)  // LSB=6.5mA
{
  if (v < 0) {return "-";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%d(%.0fmA)", v, v * 6.5);
  return buf;
}

std::string fmt_ms(int v)  // LSB=10ms
{
  if (v < 0) {return "-";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%d(%dms)", v, v * 10);
  return buf;
}

std::string fmt_volt(int v)  // LSB=0.1V
{
  if (v < 0) {return "-";}
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.1fV", v / 10.0);
  return buf;
}

void print_torque_table(const char * tag, const std::vector<Limits> & rows)
{
  std::printf("\n--- %s トルク上限 ---\n", tag);
  std::printf(
    "%4s %6s %6s %14s %14s %14s %10s %10s\n",
    "ID", "型番", "TrqEN", "最大トルク(16)", "トルク上限(48)", "目標トルク(44)",
    "過負荷(36)", "保護トルク(34)");
  for (const Limits & r : rows) {
    std::printf(
      "%4d %6s %6s %14s %14s %14s %10s %10s\n",
      r.id,
      fmt(r.model).c_str(),
      r.torque_enable < 0 ? "-" : (r.torque_enable ? "ON" : "OFF"),
      fmt_pct1000(r.max_torque).c_str(),
      fmt_pct1000(r.torque_limit).c_str(),
      fmt_pct1000(r.goal_torque).c_str(),
      fmt(r.overload_torque).c_str(),
      fmt(r.prot_torque, "%").c_str());
  }
}

void print_current_table(const char * tag, const std::vector<Limits> & rows)
{
  std::printf("\n--- %s 電流・保護 ---\n", tag);
  std::printf(
    "%4s %14s %12s %12s %12s %8s %14s %10s %8s %12s\n",
    "ID", "保護電流(28)", "過電流時間(38)", "保護時間(35)", "起動最小力(24)",
    "温度上限", "入力電圧範囲", "現在電圧", "現在温度", "現在電流(69)");
  for (const Limits & r : rows) {
    std::string vrange = "-";
    if (r.volt_min >= 0 && r.volt_max >= 0) {
      char buf[64];
      std::snprintf(buf, sizeof(buf), "%.1f-%.1fV", r.volt_min / 10.0, r.volt_max / 10.0);
      vrange = buf;
    }
    std::printf(
      "%4d %14s %12s %12s %12s %8s %14s %10s %8s %12s\n",
      r.id,
      fmt_ma(r.prot_current).c_str(),
      fmt_ms(r.overcur_time).c_str(),
      fmt_ms(r.prot_time).c_str(),
      fmt(r.startup_force).c_str(),
      fmt(r.temp_limit, "℃").c_str(),
      vrange.c_str(),
      fmt_volt(r.now_volt).c_str(),
      fmt(r.now_temp, "℃").c_str(),
      fmt_ma(r.now_current).c_str());
  }
}

void print_csv(const char * tag, const std::vector<Limits> & rows)
{
  for (const Limits & r : rows) {
    std::printf(
      "%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
      tag, r.id, r.model, r.torque_enable, r.max_torque, r.torque_limit, r.goal_torque,
      r.prot_current, r.overcur_time, r.prot_torque, r.prot_time, r.overload_torque,
      r.startup_force, r.temp_limit, r.volt_max, r.volt_min, r.now_volt, r.now_temp,
      r.now_current);
  }
}

void usage()
{
  std::printf(
    "使い方: feetech_limits [オプション]\n"
    "  --ids LIST   見る ID（既定 1..20 を総当たり）\n"
    "  --only SIDE  right / left / both（既定 both）\n"
    "  --csv        CSV で出す（bus,id,model,trq_en,max_trq,trq_lim,goal_trq,prot_cur,\n"
    "               overcur_time,prot_trq,prot_time,overload_trq,startup,temp_lim,\n"
    "               volt_max,volt_min,now_volt,now_temp,now_current。-1 は読めず）\n"
    "\n"
    "読み取り専用。サーボへは一切書き込まないので、トルクが切れたまま実行できる。\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string idstr, only = "both";
  bool csv = false;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char * w) -> std::string {
        if (i + 1 >= argc) {std::fprintf(stderr, "%s の値が無い\n", w); std::exit(2);}
        return argv[++i];
      };
    if (a == "--ids") {idstr = next("--ids");
    } else if (a == "--only") {only = next("--only");
    } else if (a == "--csv") {csv = true;
    } else if (a == "-h" || a == "--help") {usage(); return 0;
    } else {
      std::fprintf(stderr, "知らないオプション: %s\n", a.c_str());
      usage();
      return 2;
    }
  }

  std::vector<int> ids;
  if (idstr.empty()) {
    for (int i = 1; i <= 20; ++i) {ids.push_back(i);}
  } else {
    ids = parse_ids(idstr);
  }

  const char * ports[2] = {"/dev/feetech_right", "/dev/feetech_left"};
  const char * tags[2] = {"R", "L"};
  const bool want[2] = {only != "left", only != "right"};

  if (csv) {
    std::printf(
      "bus,id,model,trq_en,max_trq,trq_lim,goal_trq,prot_cur,overcur_time,prot_trq,"
      "prot_time,overload_trq,startup,temp_lim,volt_max,volt_min,now_volt,now_temp\n");
  }

  int found_total = 0;
  for (int s = 0; s < 2; ++s) {
    if (!want[s]) {continue;}
    FeetechBus bus(ports[s], 1000000, 0, 20, feetech_servo::Family::kHls);
    if (!bus.open()) {
      std::fprintf(stderr, "%s を開けない。この側は飛ばす\n", ports[s]);
      continue;
    }
    std::vector<Limits> rows;
    for (int id : ids) {
      if (!bus.ping(static_cast<uint8_t>(id))) {continue;}
      rows.push_back(read_one(bus, id));
    }
    found_total += static_cast<int>(rows.size());
    if (csv) {
      print_csv(tags[s], rows);
    } else {
      std::printf("\n=== %s (%s) : %zu 軸 ===\n", tags[s], ports[s], rows.size());
      if (rows.empty()) {
        std::printf("（応答なし）\n");
      } else {
        print_torque_table(tags[s], rows);
        print_current_table(tags[s], rows);
      }
    }
    bus.close();
  }

  if (!csv) {
    std::printf(
      "\n合計 %d 軸。トルク上限(48)は 0-1000 = 0-100%%、保護電流(28)は LSB=6.5mA、\n"
      "時間系は LSB=10ms。書き込みは一切していない。\n", found_total);
  }
  return found_total > 0 ? 0 : 1;
}
