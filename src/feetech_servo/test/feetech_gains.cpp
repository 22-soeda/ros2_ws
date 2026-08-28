// feetech_gains: 位置制御のゲインとトルク上限を「読む」ツール。
//
//   ros2 run feetech_servo feetech_gains                    # 全軸の現在値を表で出す
//   ros2 run feetech_servo feetech_gains --ids 4            # 膝だけ見る
//   ros2 run feetech_servo feetech_gains --scale-p 2 --write   # ★EEPROM を書き換える
//   ros2 run feetech_servo feetech_gains --set-p 64 --write    # ★値を直接指定して書く
//   ros2 run feetech_servo feetech_gains --follow              # 追従誤差と負荷を流し見る
//
// ===========================================================================
// 既定は読み取り専用。--write を付けたときだけ書く
// ===========================================================================
// --write が無ければ「何を書くか」を表示するだけで、サーボには一切書き込まない。
// --write を付けても、実行前に確認を求める。
//
// 書くのは **addr 21 (MODE0_P_COEF) だけ**。D・I・トルク上限は表示するが変えない
// （まず 1 つだけ動かして効果を見るため。同時に複数変えると原因が切り分けられない）。
//
// ===========================================================================
// 何のためのツールか
// ===========================================================================
// 荷重で関節が「しなる」＝ 指令位置と実位置の差（追従誤差）が残る、という症状に対して
// 位置制御の P ゲインを上げると誤差が減る。ただし上げすぎると発振する（軸が唸る・
// 細かく振動する）。**2 倍ずつ試して、唸り出す手前で止めるのが定石。**
//
// P ゲインで直るのは「サーボが指令位置を保持しきれていない」ぶんだけ。リンクや
// フレーム自体がたわんでいるぶんは直らない。切り分けは、トルクを入れた状態で
// 指令位置と実位置の差（feetech_shell の pos）を見ればよい:
//   差が大きい  -> サーボの追従不足。P ゲインが効く
//   差が小さい  -> 機構のたわみ。P ゲインでは直らない
//
// ===========================================================================
// --follow : 「トルクが足りないのか、追従が甘いのか」を切り分ける
// ===========================================================================
// 目標位置 (42/43) と実位置 (56/57) の差、そのときの負荷 (60/61) と電流 (69/70) を
// 並べて出す。**トルクを入れた状態で**（motion ノードを上げたまま）実行する。
//
//   誤差が大きい & 負荷が上限に張り付いている -> 本当にトルクが足りない
//   誤差が大きい & 負荷に余裕がある           -> ゲイン不足。トルクを上げても直らない
//
// 後者になるのが普通で、理由は **I ゲインが 0** だから。位置制御が P（と D）だけだと、
// 一定の荷重に対して定常偏差が必ず残る:
//
//     偏差 = 荷重トルク / (P ゲイン x 定数)
//
// サーボは「偏差 x P」ぶんのトルクしか出そうとしないので、上限まで余裕があっても
// そこまで出さない。**上限を上げても偏差は変わらない。** 偏差を消すには P を上げるか、
// I を入れて時間積分で押し切るかのどちらか。浮かせたとき（無荷重）と接地したとき
// （荷重あり）で姿勢が変わるのは、この定常偏差そのもの。
//
// ===========================================================================
// EEPROM について
// ===========================================================================
// addr 21 は EEPROM 領域なので、書き込み回数に上限がある（ループの中で呼ばない）。
// 書く前に LOCK(55) を解除し、書いたら必ず戻す。トルクは切ってから書く。
#include <scservo/SMS_STS.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "feetech_servo/feetech_manager.hpp"

using feetech_servo::FeetechBus;

namespace
{

std::atomic<bool> g_stop{false};
void on_sigint(int) {g_stop = true;}

/// 見たいレジスタ。HLS 系は 44/45 の意味が SMS/STS と違うが、ここで読む
/// 16/17・21・22・23・48/49 は同じ位置にある（FW 3.43 実機で確認すること）。
struct Reg
{
  const char * name;
  int addr;
  bool word;
};

/// --follow で見るレジスタ。
struct Follow
{
  int goal = 0, present = 0, load = 0, current = 0;
};

const Reg kRegs[] = {
  {"P(21)", SMS_STS_MODE0_P_COEF, false},
  {"D(22)", SMS_STS_MODE0_D_COEF, false},
  {"I(23)", SMS_STS_MODE0_I_COEF, false},
  {"最大トルク(16)", SMS_STS_MAX_TORQUE_L, true},
  {"トルク上限(48)", SMS_STS_TORQUE_LIMIT_L, true},
};

std::vector<int> parseIds(const std::string & s)
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

void usage()
{
  std::printf(
    "使い方: feetech_gains [オプション]\n"
    "  --ids LIST     見る ID（既定 1,2,3,4,5,6,7,8,9,10）\n"
    "  --only SIDE    right / left / both（既定 both）\n"
    "  --scale-p X    P ゲインを X 倍にする（255 で頭打ち）\n"
    "  --set-p N      P ゲインを N にする（--scale-p と排他）\n"
    "  --set-i N      I ゲイン(23)を N にする。定常偏差（荷重で沈むぶん）を消す\n"
    "  --follow       目標と実位置の差・負荷・電流を流し見る（トルクを入れた状態で）\n"
    "  --seconds S    --follow を S 秒で止める（既定 0 = Ctrl-C まで）\n"
    "  --write        ★実際に EEPROM へ書く。無ければ表示のみ\n"
    "  --yes          確認プロンプトを飛ばす\n"
    "\n"
    "既定は読み取り専用。--write を付けたときだけ書き込む。\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string idstr = "1,2,3,4,5,6,7,8,9,10", only = "both";
  double scale_p = 0.0;
  int set_p = -1;
  bool write = false, yes = false, follow = false;
  int set_i = -1;
  double seconds = 0.0;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](const char * w) -> std::string {
        if (i + 1 >= argc) {std::fprintf(stderr, "%s の値が無い\n", w); std::exit(2);}
        return argv[++i];
      };
    if (a == "--ids") {idstr = next("--ids");} else if (a == "--only") {
      only = next("--only");
    } else if (a == "--scale-p") {scale_p = std::atof(next("--scale-p").c_str());
    } else if (a == "--set-p") {set_p = std::atoi(next("--set-p").c_str());
    } else if (a == "--set-i") {set_i = std::atoi(next("--set-i").c_str());
    } else if (a == "--follow") {follow = true;
    } else if (a == "--seconds") {seconds = std::atof(next("--seconds").c_str());
    } else if (a == "--write") {write = true;} else if (a == "--yes") {yes = true;
    } else if (a == "-h" || a == "--help") {usage(); return 0;} else {
      std::fprintf(stderr, "知らないオプション: %s\n", a.c_str());
      usage();
      return 2;
    }
  }
  if (scale_p > 0.0 && set_p >= 0) {
    std::fprintf(stderr, "--scale-p と --set-p は同時に使えない\n");
    return 2;
  }
  const bool changing = (scale_p > 0.0 || set_p >= 0 || set_i >= 0);
  std::signal(SIGINT, on_sigint);
  std::signal(SIGTERM, on_sigint);

  const std::vector<int> ids = parseIds(idstr);
  const char * ports[2] = {"/dev/feetech_right", "/dev/feetech_left"};
  const char * tags[2] = {"R", "L"};
  const bool want[2] = {only != "left", only != "right"};

  struct Plan { int side; int id; int addr; const char * what; int from; int to; };
  std::vector<Plan> plan;
  std::vector<std::unique_ptr<FeetechBus>> buses(2);

  for (int s = 0; s < 2; ++s) {
    if (!want[s]) {continue;}
    buses[s] = std::make_unique<FeetechBus>(ports[s], 1000000, 0, 20,
      feetech_servo::Family::kHls);
    if (!buses[s]->open()) {
      std::fprintf(stderr, "%s を開けない。この側は飛ばす\n", ports[s]);
      buses[s].reset();
      continue;
    }
    std::printf("\n=== %s (%s) ===\n", tags[s], ports[s]);
    std::printf("%4s", "ID");
    for (const Reg & r : kRegs) {std::printf(" %16s", r.name);}
    std::printf("\n");

    for (int id : ids) {
      if (!buses[s]->ping(static_cast<uint8_t>(id))) {continue;}
      std::printf("%4d", id);
      int pval = -1, ival = -1;
      for (const Reg & r : kRegs) {
        const int v = r.word ? buses[s]->read_word(static_cast<uint8_t>(id), r.addr)
          : buses[s]->read_byte(static_cast<uint8_t>(id), r.addr);
        std::printf(" %16d", v);
        if (r.addr == SMS_STS_MODE0_P_COEF) {pval = v;}
        if (r.addr == SMS_STS_MODE0_I_COEF) {ival = v;}
      }
      std::printf("\n");
      if (pval >= 0 && (scale_p > 0.0 || set_p >= 0)) {
        int target = (set_p >= 0) ? set_p : static_cast<int>(pval * scale_p + 0.5);
        target = std::max(1, std::min(255, target));
        if (target != pval) {
          plan.push_back({s, id, SMS_STS_MODE0_P_COEF, "P", pval, target});
        }
      }
      if (ival >= 0 && set_i >= 0) {
        const int target = std::max(0, std::min(255, set_i));
        if (target != ival) {
          plan.push_back({s, id, SMS_STS_MODE0_I_COEF, "I", ival, target});
        }
      }
    }
  }

  // ---------------------------------------------------------------- follow
  if (follow) {
    std::printf(
      "\n目標位置と実位置の差を見る。**トルクが入っていないと差は出ない**\n"
      "（motion ノードを上げたまま実行すること）。Ctrl-C で終了。\n\n");
    const auto t0 = std::chrono::steady_clock::now();
    while (!g_stop) {
      const double t =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      if (seconds > 0.0 && t > seconds) {break;}
      std::printf("\033[H\033[Jt=%.1fs   誤差 = 実位置 - 目標位置\n\n", t);
      for (int s2 = 0; s2 < 2; ++s2) {
        if (!buses[s2]) {continue;}
        std::printf("=== %s ===\n", tags[s2]);
        std::printf("%4s %9s %9s %9s %9s %9s %9s\n",
          "ID", "目標", "実位置", "誤差", "誤差[deg]", "負荷[%]", "電流[mA]");
        for (int id : ids) {
          const uint8_t u = static_cast<uint8_t>(id);
          const int goal = buses[s2]->read_word(u, SMS_STS_GOAL_POSITION_L);
          if (goal < 0) {continue;}
          const int pres = buses[s2]->read_word(u, SMS_STS_PRESENT_POSITION_L);
          int load = buses[s2]->read_word(u, SMS_STS_PRESENT_LOAD_L);
          const int cur = buses[s2]->read_word(u, SMS_STS_PRESENT_CURRENT_L);
          // 負荷は符号ビット付き（10 bit 目が向き）。大きさだけ見る
          if (load >= 0) {load &= 0x3FF;}
          const int err = pres - goal;
          std::printf("%4d %9d %9d %+9d %+9.2f %9.1f %9d\n",
            id, goal, pres, err, err * 360.0 / 4096.0, load / 10.0, cur);
        }
        std::printf("\n");
      }
      std::printf(
        "負荷が上限(98%%)に張り付いていなければ、トルクではなくゲインの問題。\n");
      std::fflush(stdout);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    return 0;
  }

  if (!changing) {
    std::printf(
      "\n読み取りのみ。P ゲインを変えるなら --scale-p 2 を付けて何が書かれるか見る。\n"
      "荷重で沈むぶん（定常偏差）を見るなら --follow。\n");
    return 0;
  }

  std::printf("\n--- 書き込み予定 ---\n");
  if (plan.empty()) {
    std::printf("変わる軸なし（既に狙いの値、または 255 で頭打ち）\n");
    return 0;
  }
  for (const Plan & p : plan) {
    std::printf("  %s ID%-3d  %s(%d) %3d -> %3d\n",
      tags[p.side], p.id, p.what, p.addr, p.from, p.to);
  }
  if (!write) {
    std::printf("\n表示のみ。実際に書くには --write を付ける。\n");
    return 0;
  }

  if (!yes) {
    std::printf(
      "\n★EEPROM に書き込む（書き込み回数に上限がある）。トルクは切ってから実行する。\n"
      "  上げすぎると軸が唸る・発振する。まず 2 倍で様子を見ること。\n"
      "続行するなら yes と入力: ");
    std::fflush(stdout);
    std::string ans;
    std::getline(std::cin, ans);
    if (ans != "yes") {
      std::printf("中止した。何も書いていない。\n");
      return 1;
    }
  }

  int ok = 0;
  for (const Plan & p : plan) {
    FeetechBus * b = buses[p.side].get();
    const uint8_t id = static_cast<uint8_t>(p.id);
    b->enable_torque(id, false);              // 書く前にトルクを切る
    if (!b->unlock_eeprom(id, true)) {
      std::fprintf(stderr, "%s ID%d: EEPROM のロックを解除できない\n", tags[p.side], p.id);
      continue;
    }
    const bool w = b->write_byte(id, static_cast<uint8_t>(p.addr),
      static_cast<uint8_t>(p.to));
    b->unlock_eeprom(id, false);              // 必ず戻す
    const int rb = b->read_byte(id, static_cast<uint8_t>(p.addr));
    std::printf("  %s ID%-3d  %s %3d -> %3d  書込%s 読戻し %d %s\n",
      tags[p.side], p.id, p.what, p.from, p.to, w ? "OK" : "NG", rb,
      rb == p.to ? "" : "★一致しない");
    if (w && rb == p.to) {++ok;}
  }
  std::printf("\n%d/%zu 軸を書き換えた。\n", ok, plan.size());
  std::printf("トルクは切ったままなので、動かすには motion ノードを上げ直すこと。\n");
  return 0;
}
