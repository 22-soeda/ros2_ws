# roboone_teleop

PS5 (DualSense) コントローラによる無線操縦。`/joy` を `/estop`・`/cmd_walk`・`/cmd_motion`
に変換する。ros-architecture §2 の teleop ノード、作る順番 §5 の 2 にあたる。

本番 (ROBO-ONE Auto) での役割は**非常停止だけ**。手動操作は開発中に behavior を止めて
歩行だけを試すためのもの。

```
DualSense --BT--> joy (既製: game_controller_node) --/joy--> teleop --> /estop
                                                                    --> /cmd_walk
                                                                    --> /cmd_motion
                                                                    --> /autonomy   → behavior
                                                                    --> /ui/oled/text     → ui
                                                                        /ui/led/pattern
                                                                        /ui/buzzer
```

**値を変えたい人は [docs/teleop_tuning.md](../../docs/teleop_tuning.md) を読む。**
項目の一覧・意味・単位・範囲は `roboone_teleop/params.py` の表 1 か所にあり、
`config/ps5_dualsense.yaml` はその全項目を並べたもの。走らせたまま
`ros2 param set /teleop scale.x 0.08` で試せる。編集した YAML は
`ros2 run roboone_teleop teleop_params --check <yaml>` で検査できる
(打ち間違えたキーは ROS が黙って捨てるので、起動時にも警告が出る)。
「そもそも何が動かないか」の洗い出しは `docs/無線操縦_不足項目レビュー.md`。

## 1. コントローラを繋ぐ

```bash
./scripts/ps5_pair.sh
```

コントローラの **PS + Create** (十字キーの上、左上の小さいボタン) を 5 秒ほど同時押しして、
ライトバーが速く点滅したらペアリングモード。一度 `trust` すれば以後は PS ボタンだけで
自動接続する。

繋ぎ直したいとき・別のコントローラに替えるときは `./scripts/ps5_pair.sh --forget`。

USB ケーブルでも同じように使える (`/dev/input/js0` が出れば ROS 側は区別しない)。
ペアリングがうまくいかないときの切り分けに使うと早い。

## 2. 起動

```bash
ros2 launch roboone_teleop teleop.launch.py
```

引数:

| 引数 | 既定 | 意味 |
|---|---|---|
| `joy_backend` | `game_controller` | `game_controller` = SDL GameController API。SDL が DualSense の対応表を内蔵しており、ボタン/軸の並びが機種非依存になる。`joy` = 生の SDL Joystick (並びはドライバ任せ) |
| `config` | `config/ps5_dualsense.yaml` | 割り当て YAML |
| `device_id` | `0` | コントローラが複数あるとき |
| `autorepeat_rate` | `20.0` | **0 にしないこと**。0 だとスティックを動かさない限り `/joy` が来ず、無通信ウォッチドッグが「静止」と「Bluetooth 断」を区別できなくなる |
| `overrides` | (なし) | 自分用の差分 YAML。`config` の上に重ねる (後のファイルが勝つ)。変えたいキーだけ書けばよい。手順は [docs/teleop_tuning.md](../../docs/teleop_tuning.md) §3 |

## 3. 操作

| 指令 | 入力 | 出す先 | 割込 |
|---|---|---|---|
| **デッドマン** | **R1 を押している間** | これを押している間だけ歩行・技が通る | |
| 並行移動 (前後左右・斜め) | 左スティック | `/cmd_walk` の `linear.x` / `linear.y` | |
| 旋回 左/右 | 十字キー 左 / 右 | `/cmd_motion` → `turn_l` / `turn_r`（★2026-08-29 時点で `motions.yaml` に未定義。押しても何も起きない） | |
| パンチ 右 | ○ | `/cmd_motion` → `punch_r` | |
| パンチ 左 | □ | `/cmd_motion` → `punch_l` | |
| 起き上がり 前 | △ | `/cmd_motion` → `getup_front` | ★ |
| 起き上がり 後 | ✕ | `/cmd_motion` → `getup_back` | ★ |
| しゃがみ (動作確認) | 十字キー 下 | `/cmd_motion` → `squat` | |
| **脱力** | **L1** (押した瞬間) | `/estop true` をラッチ | ★ |
| **ホームポジション** | **Options を 1 秒長押し** | `/cmd_motion` → `home`、0.1 秒後に `/estop false` | ★ |
| **その場保持で武装** | **L3 を 1 秒長押し** | `/cmd_motion` → `hold`、0.1 秒後に `/estop false`。今の姿勢のままトルクが入る（転倒 → 脱力 → 起き上がり の経路。§3「その場保持」） | ★ |
| 無線テスト | Create を押している間 | `/ui/led` + `/ui/buzzer` | ★ |
| **自律動作** | **十字キー 上 を 1 秒長押し** | `/autonomy true` | |

★ = 自律動作中でも効き、押した時点で自律動作を止める割り込み。

* **左スティックに旋回は混ざらない。** 斜め前・斜め後ろへは、機体の向きを変えずに
  平行移動で歩く。右スティック左右は `angular.z` に乗るが、**motion ノードは `angular.z` を
  使わない**（歩行は平行移動のみ）。向きを変えるのは十字キー左右のキーフレームモーション。
* 脱力を L1 に置いてあるのは、歩かせている最中 (R1 + スティック) に左手の人差し指
  だけで即座に押せて、しかも歩行操作と指が競合しないから。トルクオンは逆に、誤って
  触ってもトルクが入らないよう遠く・長押しにしてある。
* トルクが入っても、**デッドマンを一度離すまで**歩行指令は出ない (再武装)。
  自律動作から戻ったときも同じ。
* パンチはデッドマンを押している間だけ受け付ける。**起き上がりは割り込み技なので
  デッドマン不要** — 転んだ機体を起こすのに R1 を押させると、自律動作からの割り込みが
  成立しない (`motion_interrupts` で指定)。

### ホームポジション

**全軸にホームの目標角を送ってから、トルクを入れる。**

1. `/cmd_motion` → `home` … motion が全軸の目標角をホームに置く (トルクはまだOFF)
2. `home_torque_delay` (既定 0.1 秒) 後に `/estop false` … トルクを入れる

順番が逆だと、サーボに残っている**古い目標角へ飛んでから**ホームへ動く
(Feetech は目標角レジスタが生きたままトルクが入る)。1 tick で両方投げると motion 側の
受信順が保証されないので、わざと間を空けている。

ホーム姿勢（立位）へ動き出すので、**転んで寝ている機体には使えない**（脚が床を押して跳ねる）。
誤って触ってもトルクが入らないよう長押しにしてある。

### その場保持（転倒からの復帰）

転倒すると電波の瞬断か L1 で脱力に入る。そこから **L3 を 1 秒長押し**すると
`/cmd_motion` → `hold`、0.1 秒後に `/estop false` の 2 段（ホームと同じ順序）で、motion ノードが
**今の実測姿勢のまま**トルクを入れる（保持姿勢を実測姿勢に差し替えるので補間距離ゼロ、その場で
固まるだけ）。そのまま △ / ✕ で起き上がりを撃つ。転倒 → 脱力 → その場保持 → 起き上がり の経路は
これで通る（`docs/無線操縦_不足項目レビュー.md` §4.3）。既にトルクが入っているときに押すと、
今の目標姿勢で止まる（歩行も技も打ち切り）。

### 脱力・トルクを `/cmd_motion` ではなく `/estop` に載せている理由

技名で送ると「非常停止がラッチされているのに `torque_on` が届く」という矛盾した状態を
motion 側で解く羽目になる。トルクの ON/OFF は経路を 1 本に絞って、手動の脱力・非常停止・
無通信ウォッチドッグを全部そこへ集める。**motion 側の約束は「`/estop true` を受けたら
即トルクOFF、`false` を受けたらトルクON」の 1 行で済む。**

### 自律動作

十字キー 上 を 1 秒長押しすると `/autonomy true` が出て、behavior ノードに指令権が渡る。
以後ロボットは自分で相手を認識して動く。

* **teleop は `/cmd_walk` を出すのをやめる。** behavior と 2 重に publish すると指令が
  奪い合いになる。ros-architecture §2 の「behavior と teleop を同時に起動しない」運用を、
  起動したまま実現するのがこのトピック (`twist_mux` を入れずに済む)
* スティックとパンチは効かない。指令権は behavior にある
* 止めるのは **起き上がり / 脱力 / 無線確認ブザー / ホームポジション / その場保持** の 5 つ。
  押した時点で `/autonomy false` が出て手動に戻り、そのうえで押した指令が実行される
* 脱力中は自律動作に入れない。先にホームポジションでトルクを入れる
* 電波が切れたら自律動作も止めて脱力する (`autonomy.stop_on_joy_loss`)。
  **本番でこれを `false` にすると非常停止の手が無くなる。** 変えるなら物理スイッチなど
  別の停止手段を用意してから

`/autonomy` は ros-architecture §4 のトピック表に無い**新設のトピック**。behavior を
書くときに合わせること:

| トピック | 型 | 出す→受ける | 意味 |
|---|---|---|---|
| `/autonomy` | `std_msgs/Bool` (latched) | teleop → behavior | true の間だけ behavior が `/cmd_walk`・`/cmd_motion` を出してよい |

### 状態表示（OLED / RGB LED）

**操作者からは「今どのモードか」が機体を見ても分からない。** 特に自律動作中は teleop が
`/cmd_walk` を黙るので、behavior が起動していないと「入れたのに動かない」になり、故障と
区別が付かない。状態が変わるたびに OLED と LED へ出す。

| 状態 | OLED | LED | 音 |
|---|---|---|---|
| `/joy` 未受信 | `TELEOP` / `no joy` | 消灯 (`dark`) | — |
| 脱力（トルクOFF） | `RELAX` / `OPTIONS=home` | **赤の速い点滅** (`estop`) | `error` |
| 再武装待ち | `MANUAL` / `release R1` | 黄の点滅 (`warn`) | — |
| 手動・武装済み | `MANUAL` / `R1 = walk` | 緑の点灯 (`ready`) | `beep` |
| 自律動作中 | `AUTO` / `any 5 = stop` | **青の点滅** (`auto`) | `ack` |
| 無線テスト中 | `LINK TEST` / `radio ok` | シアンの点灯 (`link`) | （テスト自身のブザー） |

* **色は teleop 側に持たない。** ui のプリセット名だけを送り、実際の色は ui が決める
  (`roboone_ui` の `LED_PATTERNS`)。色を 1 箇所で管理できるのと、無線テストが終わった
  ときに「元の状態のプリセットをもう一度送る」だけで復帰できる利点がある
* 送るのは**変化したときだけ**。20Hz で latched トピックを叩き続けない
* OLED は 8x8 フォントで 1 行ちょうど 12 文字（`roboone_ui` 実測）。日本語は出せないので
  ASCII。文字数はテストで見張っている
* 音は「操作者が画面を見ていなくても気付くべき変化」だけ。起動直後の初期表示では鳴らさない
* `ui.enable: false` で全部止められる

### 無線テスト

Create ボタンを押している間、頭の RGB LED がシアンに変わり、ブザーが鳴り続ける。
機体を脱力させたまま持ち歩いて、どこまで電波が届くかを確かめるための機能。

* **デッドマン不要・脱力中でも動く。** 動作条件を付けると用を成さない。
* `/joy` が `link_test.stale` (既定 0.15s) 途切れたら即座に鳴り止む。脱力までの
  `joy_timeout` (0.5s) を待たない — 電波の切れ目を耳で探すのが目的だから。
* ブザーは `beep` プリセットを 15Hz で撃ち続けて鳴らしている。ui ノードのブザーは
  「1 回鳴って止まる」設計なので、押しっぱなし用の長いパターンを ui に足すより、
  短いのを撃ち続けるほうが安全側に倒れる。teleop が落ちても電波が切れても、
  次の 1 発が来ないので 100ms 以内に鳴り止む。
* 表示には ui ノードが要る: `ros2 run roboone_ui ui_node`。上がっていなくても teleop は
  そのまま動く（publish するだけで購読者の有無は見ていない）

## 4. 安全設計

無線なのでここが本体。

* **デッドマン** — 「押している間だけ動く」。押すと動き出す方式にしない。
* **無通信ウォッチドッグ** — `/joy` が `joy_timeout` (既定 0.5s) 途切れたら脱力を
  ラッチする。Bluetooth の切断・電池切れ・コントローラを踏む、はどれも実際に起きる。
* **起動時/解除時の再武装** — ボタンを押したままの状態から復帰していきなり歩き出さない。
* **加速度制限** — スティックの段差をそのまま速度指令にしない。二足で速度指令が階段状に
  飛ぶと、それだけで転ぶ。
* **常時 20Hz 送信** — 止まっているときもゼロを送り続ける。無送信で「最後の指令が残る」
  より、ゼロが来続けるほうが motion 側が単純で安全になる。
* **`/estop` は latched QoS** (`TRANSIENT_LOCAL`) — teleop より後に motion を起動しても
  直前の脱力状態が届く。ここを Volatile にすると「脱力させた状態で motion を再起動したら
  動き出した」が起こりうる。`/ui/*` も ui ノードの購読側が latched なので同じ設定で出す
  (VOLATILE な publisher は TRANSIENT_LOCAL な subscriber とマッチしない)。
* **終了時** — ノードが落ちるときにゼロ Twist と `/estop true`、点けていれば LED 消灯を
  置いていく。rclpy の既定シグナルハンドラは context を先に畳んでしまい publish が
  誰にも届かないので、既定を外して自前のフラグで抜けている
  (`SignalHandlerOptions.NO`)。

### motion 側に要る約束

teleop の最期の 1 発は保険であって、当てにするものではない (`kill -9`・電源断では
届かない)。**motion ノードは `/cmd_walk` が途切れたら自分で止まること。** teleop は
静止中もゼロを 20Hz 送り続けているので、「一定時間 `/cmd_walk` が来ない = 送り手が
消えた」と判定してよい。

まとめると motion 側の約束は 2 行:

1. `/estop true` を受けたら即トルクOFF、`false` を受けたらトルクON
2. `/cmd_walk` が途切れたら停止する
3. `/cmd_motion` の `home` は「全軸の目標角をホームに置く」まで。トルクを入れるのは
   `/estop false` 側の仕事 (この 2 段が §3 のホームポジション)

behavior 側は `/autonomy` が true の間だけ `/cmd_walk`・`/cmd_motion` を出すこと。
false の間は黙る (teleop が `/cmd_walk` を出しているので、両方出すと奪い合いになる)。

## 5. 割り当てを実機で確認する

`config/ps5_dualsense.yaml` の index は SDL GameController 標準配列を前提にした値。
**推測で使わず、必ず実機で確認する**。「非常停止だと思っていたボタンが実は旋回軸だった」
は起きる。

```bash
ros2 run joy game_controller_node          # 別端末
ros2 run roboone_teleop joy_probe
```

ボタンを押すと `button 10 ↓ 押した   config には "b10"   (R1?)` のように出る。
出た文字列をそのまま YAML に書けばよい。

割り当ての表記:

* `b5` … `buttons[5]` が 1 のとき ON
* `a7+` / `a7-` … `axes[7]` が ±0.5 を超えたとき ON (十字キーが軸として出る場合用)

### 軸の符号 — `invert` は 3 つとも false

**joy ノードが SDL の値を符号反転して publish している。** SDL 自体は「下 = +」
「右 = +」だが、/joy の時点で既に ROS 規約 (前 = +x / 左 = +y / 反時計 = +yaw) に
なっているので、teleop 側で重ねて反転すると全部逆を向く。

2026-08-28 実機実測 (DualSense + `game_controller_node`、軸 6 本 / ボタン 21 個):

| 操作 | /joy | /cmd_walk |
|---|---|---|
| 左スティックを前へ | `axes[1] = +0.77` | `linear.x = +0.060` |
| 左スティックを左へ | `axes[0] = +0.87` | `linear.y = +0.030` |
| 右スティックを左へ | `axes[2] = +0.88` | `angular.z = +0.400`（motion 側で捨てられる） |

反転している証拠は、無操作時に L2/R2 (`axes[4]`, `axes[5]`) が `0.0` ではなく **`-0.0`**
で出ること。負のゼロは値を反転しないと生まれない。

### 実機で確認済みのボタン (2026-08-28)

`b0` ✕ / `b1` ○ / `b2` □ / `b3` △ / `b4` Create / `b6` Options / `b9` L1 / `b10` R1 /
`b11` 十字キー 上。config の 9 個すべてが一致した。
十字キー下・左・右（`b12` / `b13` / `b14`）と L3（`b7`）はその後に足した割り当てで**未照合**。
`joy_probe` で確かめること。

ペアリング済みのコントローラ: `E8:47:3A:C7:58:6C` (`DualSense Wireless Controller`)。
`trust` 済みなので PS ボタンだけで再接続する。カーネルは `hid-playstation` が掴み、
`/dev/input/js0` が本体・`js1` が Motion Sensors (IMU)。SDL は `Mapped: true` で
内蔵の対応表が効いている (`ros2 run joy joy_enumerate_devices` で確認できる)。

## 6. 動作確認

motion ノードを上げずに、トピックを直接見る（機体を動かさずに motion 込みで確かめるなら
`ros2 launch roboone_bringup roboone.launch.py allow_torque:=false`）。

```bash
ros2 topic echo /estop
ros2 topic hz   /cmd_walk        # R1 を押していなくても 20Hz 出る (中身はゼロ)
ros2 topic echo /cmd_walk
ros2 topic echo /cmd_motion
```

ウォッチドッグの確認は、コントローラの PS ボタン長押しで電源を切る。0.5 秒後に
`*** 脱力 (トルクOFF) ***` がログに出て `/estop` が true になれば正しい。

自律動作の確認:

```bash
ros2 topic echo /autonomy
ros2 topic hz /cmd_walk          # 自律動作中は teleop から出なくなる
```

`colcon test` にデッドマン・ウォッチドッグ・脱力ラッチ・「並行移動に旋回が混ざらない」
・ホームの 2 段送信・自律動作の 4 割り込み・無線テストの結線を見る機能テストが
入っている (`test/test_teleop.py`)。コントローラ無しで走る。
走らせたままの調整 (`ros2 param set` 相当) の反映と拒否も同じファイルで見ている。
`params.py` の表と `config/ps5_dualsense.yaml` の整合 (打ち間違い・型・範囲) は
`test/test_params.py` が見張る。こちらは ROS 無しでも走る。

## 7. まだ決めていないこと

* `scale.*` (最高速) は仮値。実機で歩かせながら詰める (`docs/teleop_tuning.md` §5.1)。
  今は「歩けるより遅い」側に振ってある。
* `motion_bindings` の技名のうち `punch_r` / `punch_l` / `getup_front` / `getup_back` / `squat` は
  `motions.yaml` に定義済み。**`turn_l` / `turn_r` は未定義**で、押しても何も起きない
  (`docs/無線操縦_不足項目レビュー.md` §2.2)。teleop は起動時に `motions.yaml` と照合して
  無い名前を警告する。起動前に見るなら
  `ros2 run roboone_teleop teleop_params --check config/ps5_dualsense.yaml --motions <motions.yaml>`。
* 起き上がりの「前 / 後」は**倒れた向き**で名付けてある (`getup_front` = うつ伏せから、
  `getup_back` = 仰向けから)。motion 側と解釈を合わせること。
* `b7` (L3) / `b12`〜`b14` (十字下・左・右) は実機未照合。`joy_probe` で確かめる。
* 自律動作中に teleop が黙るので、behavior が上がっていないまま自律に入ると `/cmd_walk`
  が完全に止まる。motion の「途切れたら止まる」が効いて機体は停止する (安全側) が、
  操作者から見ると「入ったのに動かない」になる。`/behavior/state` を ui に出すのはこれから。
