# feetech_servo

Feetech SMS/STS シリアルバスサーボの C++ 制御ヘルパ（ROS2 Jazzy / ament_cmake）。
公式 **SCServo SDK を同梱**し、バス単位で「一括書き込み → 一括読み出し」の**閉ループ**を提供する。
コントローラ2台 = **2本のシリアルバス**を並列に回す構成。

## 構成

```
feetech_servo/
├── include/feetech_servo/
│   ├── servo_state.hpp     # 1軸の状態（生カウント値＋物理量）
│   ├── feetech_bus.hpp     # 1バス=1コントローラのラッパ（閉ループの中身）
│   └── feetech_manager.hpp # 複数バスを束ねる
├── src/
│   ├── feetech_bus.cpp
│   ├── feetech_manager.cpp
│   └── feetech_demo_node.cpp   # 2バス並列で閉ループを回すデモ node
├── vendor/scservo/         # 同梱した Feetech 公式 SCServo SDK (MIT)
├── test/
│   ├── feetech_scan_test.cpp   # サーボ列挙CLI
│   ├── feetech_goto_test.cpp   # 全軸一斉移動CLI
│   └── feetech_shell.cpp       # 1軸ずつ手で操作する対話CLI
├── config/feetech_demo.yaml
└── launch/feetech_demo.launch.py
```

- ライブラリ `feetech_servo` は **ROS 非依存の純粋 C++**（SDK は内部にだけ使い、公開ヘッダには漏らさない）。
  他パッケージから `find_package(feetech_servo)` → `target_link_libraries(... feetech_servo)` で使える。

## 閉ループの流れ（1バス1サイクル）

```cpp
#include <feetech_servo/feetech_bus.hpp>
using namespace feetech_servo;

FeetechBus bus("/dev/ttyACM0", 1000000);
bus.open();
std::vector<uint8_t> ids = bus.scan({1, 2, 3});
for (auto id : ids) bus.init_motor(id, Mode::kPosition, /*torque=*/true);

std::vector<ServoState> states;
while (running) {
  // 1) 書き込み: 全軸の目標位置を1パケット (SyncWritePosEx)
  bus.sync_write_position(ids, goals, speeds, accs);
  // 2) 読み出し: 全軸の状態を1往復で取得 (SyncRead, 15byte)
  bus.sync_read_states(ids, states);
  // states[k].valid が true の軸だけ使う
}
```

- **書き込み**: `sync_write_position()` … `SyncWritePosEx` で全軸を1パケット送信。
- **読み出し**: `sync_read_states()` … `syncReadPacketTx/Rx` で全軸ぶんを1往復で取得し、
  位置・速度・負荷・電圧・温度・電流・moving・err を復号（`feetech.py` と同一の意味づけ）。
- 単軸版 `write_position()` / `read_state()`（FeedBack経由）も用意。
- 各バスは独立オブジェクト。**バスごとに別スレッド**を割り当てれば2本を真に並列で回せる
  （デモ node は MultiThreadedExecutor + バス毎コールバックグループでそれを行う）。

## ビルド

```bash
cd ~/ros2_ws
colcon build --packages-select feetech_servo --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## デモ node

```bash
# 読み取りのみ（安全: トルクOFF, 機体は動かない）
ros2 launch feetech_servo feetech_demo.launch.py

# 実機を動かす（トルクON + 起点まわりにサイン波）
ros2 launch feetech_servo feetech_demo.launch.py enable_motion:=true
```

- パラメータは `config/feetech_demo.yaml`。`ports` / `ids`（`ids_0`,`ids_1` でバス別上書き）/
  `rate_hz` / `enable_motion` / `amplitude_deg` / `period_s` など。
- 出力: `/feetech/bus0/joint_states`, `/feetech/bus1/joint_states`
  （position=rad, velocity=rad/s相当, effort=電流mA）。有効に読めた軸のみ載せる。

## 実機テストツール

`test/` の3本は ROS を使わない素のCLI（実機がないと意味がないので ament のテストには載せていない）。
手で1軸ずつ触りたいときは **`feetech_shell`**、全軸の一斉動作確認は `feetech_goto_test`。

### 1. サーボ列挙 `feetech_scan_test`

1Mbps のバスを総当たり ping して、応答したIDの情報を全部出す。

```bash
ros2 run feetech_servo feetech_scan_test                      # 既定2ポート, ID 0..253
ros2 run feetech_servo feetech_scan_test --port /dev/ttyACM0  # ポート指定（複数可）
ros2 run feetech_servo feetech_scan_test --id-max 20          # 探索範囲を狭めて高速化
```

出力: 一覧表（型番/FW/モード/ボーレート/トルク/位置/角度/電圧/温度/電流/エラー）＋
軸ごとの詳細（角度リミット・位置オフセット・温度上限・目標位置・負荷・moving 等）＋ `tx/rx_fail`。

### 2. 一斉移動 `feetech_goto_test`

定義済みの全軸を**一斉に**、約30秒かけて位置 **2047**（中央）へ動かす。

```bash
ros2 run feetech_servo feetech_goto_test              # 既定の定義（両ポート ID 1..10）を30秒で2047へ
ros2 run feetech_servo feetech_goto_test --no-torque  # トルクを入れずに手順だけ確認（動かない）
ros2 run feetech_servo feetech_goto_test --scan       # 定義ではなく応答した軸すべて
ros2 run feetech_servo feetech_goto_test --motors /dev/ttyACM0:1,5,6 --duration 10
ros2 run feetech_servo feetech_goto_test --torque 500     # HLS系の目標トルクを下げる
ros2 run feetech_servo feetech_goto_test --family sms     # SMS/STS サーボの場合
```

処理順（バスごとに1スレッド、共通の開始時刻に `sleep_until` して同時に動き出す）:

1. `ping` で定義軸の生存確認（応答しない軸は対象外）
2. `sync_read_states` で**現在の角度を読む**（数回リトライ。読めた軸だけが軌道の始点を持つ）
3. **目標位置に現在位置を書いてからトルクON** — この順序でないとトルク投入時に古い目標へ飛び出す
4. `rate_hz`（既定50Hz）で始点→2047 を smoothstep 補間し `SyncWritePosEx` で送信、同時に状態を読む
5. 到達誤差と `tx/rx_fail`・平均取得率を表示

- 対象の定義は `test/feetech_goto_test.cpp` の `kDefaultMotors`（`--motors` で上書き）。
- **`--torque 0` にすると HLS 系はまったく動かない**（上の「サーボ系列」節を参照）。
- Ctrl-C で中断するとその時点の指令位置で保持したまま終了する（トルクは切らない）。
- 終了後もトルクは入ったまま（保持）。

### 3. 手動操作シェル `feetech_shell`

**1軸だけ**を手で操作する対話CLI。コマンドを打つとその場でシリアルに信号が出る。

```bash
ros2 run feetech_servo feetech_shell                 # 既定2ポートを開いて対話開始
ros2 run feetech_servo feetech_shell --bus 1 --id 5  # 最初から bus1 の ID5 を選択
echo -e "id 5\npos\ngo 2047" | ros2 run feetech_servo feetech_shell   # パイプでスクリプト実行
```

操作対象は常に「選択中の **bus + ID** の1軸」。一括操作はしない。

| コマンド | 内容 |
|---|---|
| `bus [N\|PORT]` | コントローラを選ぶ（`bus 1` / `bus /dev/ttyACM0`）。未接続なら開き直しを試す |
| `buses` | バス一覧と接続状態 |
| `id [N]` | 操作するIDを選ぶ。**ping で存在確認できたときだけ**選択が変わる |
| `ping [N]` | そのIDが応答するか |
| `scan [min max]` | このバスのIDを総当たり ping（既定 1..20） |
| `pos` | 現在位置を生値で表示（0-4095 と deg） |
| `state` | 位置/速度/負荷/電圧/温度/電流/moving/エラー |
| `info` | EEPROM（型番・FW・モード・トルク・角度リミット・目標位置・目標トルク等） |
| `on` / `off` | トルクON / OFF |
| `go POS` | 指定位置へ移動（0-4095。範囲外は丸める） |
| `jog D` | 現在位置から D ステップ相対移動（`jog -100`） |
| `watch [秒]` | 位置を連続表示（既定5秒、Ctrl-C で中断） |
| `speed` / `acc` / `torque` | 移動速度 step/s・加速度・HLS系の目標トルクの設定/表示 |
| `stats` | このバスの `tx` / `rx_fail` |
| `quit` | 終了（**トルクの状態はそのまま**） |

- 行のどこかに `@ID` を付けると、その行だけ別のIDを対象にできる（`pos @7`, `go 2047 @3`）。
- `on` は**現在位置を目標に書いてからトルクを入れる**ので、投入時に古い目標へ飛び出さない。
  現在位置が読めない軸には**トルクを入れない**（危険なため）。
- `go` の前にトルクOFF／目標トルク0を検出したら警告する（指令自体は送る）。
- `go` は送信後 0.3 秒待って実測位置と誤差・負荷・電流を表示する。
- 起動時オプション: `--port`(複数可) `--baud` `--bus` `--id` `--speed` `--acc` `--torque` `--family` `--timeout`。

## サーボ系列（HLS / SMS・STS）— 動かないときの最重要ポイント

位置指令は `reg41(ACC), 42-43(GOAL_POSITION), 44-45(?), 46-47(GOAL_SPEED)` の7バイトを一括で書くが、
**44/45 の意味が系列で違う**:

| 系列 | reg44/45 | 0 を書くと |
|---|---|---|
| SMS/STS | `GOAL_TIME` | 問題なし（速度指定で動く） |
| **HLS** | **`GOAL_TORQUE`** | **目標トルク0 = まったく駆動しない** |

同梱SDKの `SMS_STS::WritePosEx` / `SyncWritePosEx` は 44/45 に**必ず0を書く**ため、HLS系サーボでは
「トルクON・目標位置も正しく入っているのに、負荷も電流もほぼ0のまま1ステップも動かない」という症状になる。

**本構成の実機（model 4618 / 5130, FW 3.43, 全19軸）は HLS 系**。そのため `FeetechBus` は
SDK の高レベル関数を使わず自前で7バイトブロックを組み、`Family::kHls`（既定）のときは
44/45 に `goal_torque`（既定 1000, `set_goal_torque()` で変更）を載せる。

```cpp
FeetechBus bus("/dev/ttyACM1", 1000000, 0, 20, Family::kHls);  // 既定が kHls
bus.set_goal_torque(1000);   // 0 にすると動かなくなるので注意
```

SMS/STS を繋ぐ場合は `Family::kSmsSts` を指定する（HLS用の値を書くと GOAL_TIME に化ける）。
`feetech_goto_test` は `--family sms` / `--torque N` で切り替えられる。

## 注意

- CH343 は `cdc_acm` ドライバなので **`/dev/ttyACM*`**（`ttyUSB` ではない）。
  安定させたい場合は `/dev/serial/by-id/...` を `ports` に指定。
- ポートを開くにはユーザーが **`dialout` グループ**に入っている必要がある。
- **供給電圧が低い**とサーボの通信が間欠的に欠け、`sync_read_states` の取得軸数が減り
  `rx_fail` が増える。読み取り信頼性が低いときはまず電源電圧を定格（STS3215 なら ~12V）に戻す。

## 同梱 SDK

`vendor/scservo/` は Feetech 公式 SCServo Linux SDK（MIT, `vendor/scservo/LICENSE`）。
`SMS_STS` クラスを静的リンクして利用している。
