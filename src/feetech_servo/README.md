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
│   ├── feetech_shell.cpp       # 1軸ずつ手で操作する対話CLI
│   └── feetech_set_limits.cpp  # 角度リミットの一括書き込み
├── docs/servo-registers.md     # 実機18軸のレジスタ設定（実測値）
├── config/servo_limits.yaml    # 角度リミットの設定ファイル
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

`test/` の5本は ROS を使わない素のCLI（実機がないと意味がないので ament のテストには載せていない）。
手で1軸ずつ触りたいときは **`feetech_shell`**、全軸をまとめて角度指定で動かすのは `feetech_goto_test`、
原点（初期位置）を軸ごとに決めるときは **`feetech_calibrate_home`**。

### 1. サーボ列挙 `feetech_scan_test`

1Mbps のバスを総当たり ping して、応答したIDの情報を全部出す。

```bash
ros2 run feetech_servo feetech_scan_test                      # 既定2ポート, ID 0..253
ros2 run feetech_servo feetech_scan_test --port /dev/ttyACM0  # ポート指定（複数可）
ros2 run feetech_servo feetech_scan_test --id-max 20          # 探索範囲を狭めて高速化
```

出力: 一覧表（型番/FW/モード/ボーレート/トルク/位置/角度/電圧/温度/電流/エラー）＋
軸ごとの詳細（角度リミット・位置オフセット・温度上限・目標位置・負荷・moving 等）＋ `tx/rx_fail`。

### 2. 一斉移動（角度指定）`feetech_goto_test`

全軸を**一斉に**、約30秒かけて**角度で指定した姿勢**へ動かす。目標は生カウントではなく
**deg** で与え、各軸の原点（[config/servo_home.yaml](config/servo_home.yaml)）から

```
指令カウント = home + (狙い角[deg] - home_deg) * 4096 / 360
```

で軸ごとに換算する。**`--angle` を省くと `home_deg`（=90deg）＝校正した姿勢そのもの**へ戻る
（＝Tポーズ）。原点が無い軸は角度に換算できないので動かさない（応答するのに未校正なら警告する）。

```bash
ros2 run feetech_servo feetech_goto_test                  # 30秒かけて校正した姿勢(90deg)へ
ros2 run feetech_servo feetech_goto_test --angle 120      # 全軸 120deg へ
ros2 run feetech_servo feetech_goto_test --dry-run        # 換算した目標を見るだけ（動かさない）
ros2 run feetech_servo feetech_goto_test --motors /dev/ttyACM0:1,5,6 --duration 10
ros2 run feetech_servo feetech_goto_test --torque 500     # HLS系の目標トルクを下げる
ros2 run feetech_servo feetech_goto_test --family sms     # SMS/STS サーボの場合
```

処理順（バスごとに1スレッド、共通の開始時刻に `sleep_until` して同時に動き出す）:

1. `ping` で対象軸の生存確認（応答しない軸は対象外。**応答するのに原点が無い軸は警告して除外**）
2. `sync_read_states` で**現在位置を読む**（数回リトライ。読めた軸だけが軌道の始点を持つ）
3. **目標位置に現在位置を書いてからトルクON** — この順序でないとトルク投入時に古い目標へ飛び出す
4. `rate_hz`（既定50Hz）で始点→目標を smoothstep 補間し `SyncWritePosEx` で送信、同時に状態を読む
5. 到達誤差（deg）と `tx/rx_fail`・平均取得率を表示

| オプション | 内容 |
|---|---|
| `--angle DEG` | 全軸の狙い角（既定 = 原点ファイルの `home_deg`） |
| `--home FILE` | 原点ファイル（既定 `share/feetech_servo/config/servo_home.yaml`） |
| `--motors PORT:ID,...` | 対象を絞る（複数指定可）。既定は原点ファイルの全軸 |
| `--dry-run, -n` | 換算結果だけ表示して終了（指令もトルクも出さない） |
| `--duration S` / `--rate HZ` | 到達までの秒数（既定30）/ 送信周波数（既定50） |
| `--speed N` / `--acc N` | `SyncWritePosEx` の速度・加速度（既定 600 / 20） |
| `--torque N` | HLS系の目標トルク 0-1000（既定 1000） |
| `--no-torque` | トルクを入れずに指令だけ流す（**既にトルクONの軸は動く**ので注意） |
| `--yes, -y` | 開始前の3秒カウントダウンを省略 |

- 原点は先に `feetech_calibrate_home` で取る（未校正だと起動時に「原点ファイルを読めない」で止まる）。
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
| `getb A` / `getw A` | 生レジスタを読む（`getw 9` = 角度リミット下限） |
| `setb A V` / `setw A V` | 生レジスタに書く。EEPROM(addr<40) は自動で unlock→書き→lock |
| `limits [Lo Hi]` | 角度リミットの表示 / 設定（既定は全軸 `0 0` = 制限なし） |
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

### 4. 角度リミットの一括書き込み `feetech_set_limits`

設定ファイル [config/servo_limits.yaml](config/servo_limits.yaml) に書いた角度リミットを、
**全軸まとめてサーボの EEPROM に書き込む**（可動域をソフト側ではなくモーター側に持たせる場合に使う）。

```bash
ros2 run feetech_servo feetech_set_limits --dry-run    # 何を変更するかだけ表示（書き込まない）
ros2 run feetech_servo feetech_set_limits              # 確認プロンプトのあと一括書き込み
ros2 run feetech_servo feetech_set_limits -c ./my.yaml --yes
```

**`config/servo_limits.yaml` を編集したら `colcon build` が必要**（ツールが読むのは
`install/.../share/feetech_servo/config/` に入ったコピーのため）。ソースを直接読ませるなら `-c` で渡す。

設定ファイルの書式（値は生カウント 0-4095。`[下限, 上限]` か `{min: , max: }`）:

```yaml
baud: 1000000
buses:
  - port: /dev/ttyACM0
    servos:
      1:  [1024, 3072]     # 90deg .. 270deg
      2:  [0, 0]           # 0,0 = 制限なし（多回転可）
```

書き込むのは `MIN_ANGLE_LIMIT(9)` / `MAX_ANGLE_LIMIT(11)` の2つだけ。安全のため:

- **設定ファイル全体を検証してから接続する** — 下限>上限・範囲外・可動域ゼロがあれば**1軸も書かずに中止**
- **現在値と同じ軸は書かない** — EEPROM の書き換え回数を無駄にしない（全軸そのままなら「変更なし」で終了）
- **応答しない軸・現在値を読めない軸は飛ばす**（低電圧時に誤った値を書かないため。ping と読みは3回まで再試行）
- **トルクONの軸があれば中止**する。`--torque-off` を付けたときだけ先に脱力させる（機体が落ちるので支えてから）
- **新しい範囲の外に現在位置がある軸は警告**する（次にトルクを入れた時点で範囲の端まで動くため）
- 書き込みは `unlock → 書き → lock → 読み戻し検証` の順で、必ず EEPROM を再ロックする

| オプション | 内容 |
|---|---|
| `-c, --config FILE` | 設定ファイル（既定 `share/feetech_servo/config/servo_limits.yaml`） |
| `--dry-run, -n` | 書き込まず、変更内容だけ表示 |
| `--yes, -y` | 確認プロンプトを省略 |
| `--torque-off` | トルクONの軸を先に脱力させる |
| `--baud N` | 設定ファイルの `baud` を上書き |

### 5. 原点（初期位置）合わせ `feetech_calibrate_home`

**生カウント 2047 が機構上の 90deg とは限らない**ので、目で見て合わせた姿勢の生カウントを
軸ごとに記録して [config/servo_home.yaml](config/servo_home.yaml) に保存する。
**EEPROM は一切書き換えない**（`OFS(31)` も触らない）。保存されるのは YAML だけ。

```bash
ros2 run feetech_servo feetech_calibrate_home                 # 全軸を1軸ずつ
ros2 run feetech_servo feetech_calibrate_home --bus 0 --id 3  # bus0 の ID3 だけやり直す
ros2 run feetech_servo feetech_calibrate_home -o ~/home.yaml  # 保存先を変える
```

進め方は**1軸ずつ**。ある軸に来ると、

1. その軸だけトルクを切る → 手で動かせる
2. **現在の生カウント / 角度を 20Hz でリアルタイム表示**しながら目で合わせる
   （`value 2100  184.6 deg  中央から +52  脱力中  11.9V 31℃  前回比 +15`）
3. `Enter` でその位置を確定 → **その位置をトルクで保持したまま次の軸へ**（姿勢が崩れない）

| キー | 動作 |
|---|---|
| `Enter` | 今の位置を原点として確定し、次の軸へ |
| `←` `→` | 1ステップ動かす（トルクが切れていれば自動でONにしてから動かす） |
| `↑` `↓` | 10ステップ動かす |
| `t` | トルク ON / OFF（ONは現在位置を目標に書いてから入れる＝飛び出さない） |
| `s` | この軸は飛ばす（前回の値は残る） |
| `b` | 前の軸に戻ってやり直す |
| `q` | ここまでの結果を保存して終了 |

安全のため:

- 起動時に「軸を脱力させる＝機体が落ちる」ことを確認する（`--yes` で省略）
- **現在位置が読めない軸にはトルクを入れない／確定もしない**（飛び出し防止。読みは5回まで再試行）
- 矢印キーでの移動は [config/servo_limits.yaml](config/servo_limits.yaml) の角度リミット内に丸める
- 確定した位置がリミットの外なら警告する
- **保存は軸ごとにマージ**される。`--bus/--id` で1軸だけやり直しても、他の軸の値は消えない
- 電圧が 9V を切っているときは表示に `※低電圧` を出す（値が当てにならないので電源を直してから）

保存されるファイル:

```yaml
center: 2048
home_deg: 90.0      # 合わせた姿勢を何 deg と呼ぶか（--home-deg。記録するだけ）
buses:
  - port: /dev/ttyACM0
    servos:
      1:   {home: 2100, offset:    52}   # 184.6 deg
```

**校正したら `colcon build` が必要**（`feetech_goto_test` が読むのは
`install/.../share/feetech_servo/config/` に入ったコピーのため）。ビルドせずに使うなら
`feetech_goto_test --home src/feetech_servo/config/servo_home.yaml`。

`home` が原点の生カウント、`offset` は `home - 2048`（中央からのずれ。参考値）。
使う側は `指令カウント = home + (狙い角[deg] - home_deg) * 4096 / 360`。

| オプション | 内容 |
|---|---|
| `-c, --config FILE` | 軸一覧（既定 `share/feetech_servo/config/servo_limits.yaml`） |
| `-o, --out FILE` | 保存先（既定 `src/feetech_servo/config/servo_home.yaml`、無ければ `./servo_home.yaml`） |
| `--bus N` / `--id N` | そのバス / そのIDだけ校正する |
| `--home-deg D` | 合わせる姿勢を何 deg と呼ぶか（既定 90） |
| `--no-hold` | 確定後にトルクで保持しない |
| `--no-release` | 軸に来たとき自動で脱力しない（矢印キーだけで合わせる） |
| `--yes, -y` | 起動時の確認を省略 |
| `--speed N` / `--acc N` | 微調整で動かす速度・加速度（既定 300 / 20） |
| `--torque N` | HLS系の目標トルク 0-1000（既定 1000）。0 だと動かない |

## 実機の設定値

18軸すべてのレジスタ実測値は [docs/servo-registers.md](docs/servo-registers.md) にまとめてある
（バス0/バス1は同一。差は型番 4618 / 5130 の間だけ）。

## 角度リミット（1軸だけ動かない・0付近に張り付くとき）

EEPROM の `MIN_ANGLE_LIMIT(9)` / `MAX_ANGLE_LIMIT(11)` は**全軸 `0 .. 0`（制限なし）が既定**。
ここが **下限 > 上限** の不正な値になっていると、ファームが目標位置をその窓に丸めるため
**GOAL_POSITION には指令値が正しく入るのに、実位置は 0 付近から動かない**。

- 2026-08-26 に bus1 の ID2 がこの状態（`1 .. 0`）で、`go 1500` でも `go 2000` でも
  pos=4 に張り付いていた。他17軸はすべて `0 .. 0`。`limits 0 0` で復旧。
- 見分け方: `feetech_shell` の `info` で「角度リミット」を見る（不正なら警告が出る）。
  トルクON・目標トルク1000・err=0x00 なのに動かない、が典型症状。
- 直し方（トルクOFFが必要。EEPROM は書き込み回数に上限があるので設定修正のときだけ）:

```
[bus1 id2]> off
[bus1 id2]> limits 0 0
[bus1 id2]> on
```

## 注意

- CH343 は `cdc_acm` ドライバなので **`/dev/ttyACM*`**（`ttyUSB` ではない）。
  安定させたい場合は `/dev/serial/by-id/...` を `ports` に指定。
- ポートを開くにはユーザーが **`dialout` グループ**に入っている必要がある。
- **供給電圧が低い**とサーボの通信が間欠的に欠け、`sync_read_states` の取得軸数が減り
  `rx_fail` が増える。読み取り信頼性が低いときはまず電源電圧を定格（STS3215 なら ~12V）に戻す。

## 同梱 SDK

`vendor/scservo/` は Feetech 公式 SCServo Linux SDK（MIT, `vendor/scservo/LICENSE`）。
`SMS_STS` クラスを静的リンクして利用している。

配布物に対する変更（`vendor/scservo/include/scservo/SCS.h`、`NOTE(feetech_servo)` コメント付き）:

- **`syncReadRxBuff` などのメンバを既定値で初期化**した。元の SDK はコンストラクタで初期化しておらず、
  不定値のまま `syncReadBegin()` に入ると `if (syncReadRxBuff) delete[] syncReadRxBuff;` が
  ゴミポインタを解放して落ちる（`free(): invalid pointer` / SIGSEGV）。**起動直後の
  最初の `sync_read_states` でスタック内容次第で落ちる**という再現性の低い形で出る。
- `~SCS()` で `syncReadRxBuff` を解放するようにした（元は空実装でリークしていた）。
