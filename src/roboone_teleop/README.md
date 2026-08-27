# roboone_teleop

PS5 (DualSense) コントローラによる無線操縦。`/joy` を `/estop`・`/cmd_walk`・`/cmd_motion`
に変換する。ros-architecture §2 の teleop ノード、作る順番 §5 の 2 にあたる。

本番 (ROBO-ONE Auto) での役割は**非常停止だけ**。手動操作は開発中に behavior を止めて
歩行だけを試すためのもの。

```
DualSense --BT--> joy (既製: game_controller_node) --/joy--> teleop --> /estop
                                                                    --> /cmd_walk
                                                                    --> /cmd_motion
                                                                    --> /ui/led, /ui/buzzer
                                                                        (無線テストのときだけ)
```

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

## 3. 操作

| 指令 | 入力 | 出す先 |
|---|---|---|
| **デッドマン** | **R1 を押している間** | これを押している間だけ歩行・技が通る |
| 並行移動 (前後左右・斜め) | 左スティック | `/cmd_walk` の `linear.x` / `linear.y` |
| 旋回 右/左 | 右スティック 左右 | `/cmd_walk` の `angular.z` |
| パンチ 右 | ○ | `/cmd_motion` → `punch_r` |
| パンチ 左 | □ | `/cmd_motion` → `punch_l` |
| 起き上がり 前 | △ | `/cmd_motion` → `getup_front` |
| 起き上がり 後 | ✕ | `/cmd_motion` → `getup_back` |
| **脱力** | **L1** (押した瞬間) | `/estop true` をラッチ |
| **トルクオン** | **Options を 1 秒長押し** | `/estop false` |
| 無線テスト | Create を押している間 | `/ui/led` + `/ui/buzzer` |

* **左スティックに旋回は混ざらない。** 斜め前・斜め後ろへは、機体の向きを変えずに
  平行移動で歩く。旋回は右スティックだけが出す。
* 脱力を L1 に置いてあるのは、歩かせている最中 (R1 + スティック) に左手の人差し指
  だけで即座に押せて、しかも歩行操作と指が競合しないから。トルクオンは逆に、誤って
  触ってもトルクが入らないよう遠く・長押しにしてある。
* トルクオンしても、**デッドマンを一度離すまで**歩行指令は出ない (再武装)。
* パンチと起き上がりもデッドマンを押している間だけ受け付ける
  (`motion_requires_deadman: false` で外せる)。

### 脱力・トルクオンを `/cmd_motion` ではなく `/estop` に載せている理由

技名で送ると「非常停止がラッチされているのに `torque_on` が届く」という矛盾した状態を
motion 側で解く羽目になる。トルクの ON/OFF は経路を 1 本に絞って、手動の脱力・非常停止・
無通信ウォッチドッグを全部そこへ集める。**motion 側の約束は「`/estop true` を受けたら
即トルクOFF、`false` を受けたらトルクON」の 1 行で済む。**

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
* 相手側に ui ノードが要る: `ros2 run roboone_ui ui_node`

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
  置いていく。

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

## 6. 動作確認

motion ノードはまだ無いので、トピックを直接見る。

```bash
ros2 topic echo /estop
ros2 topic hz   /cmd_walk        # R1 を押していなくても 20Hz 出る (中身はゼロ)
ros2 topic echo /cmd_walk
ros2 topic echo /cmd_motion
```

ウォッチドッグの確認は、コントローラの PS ボタン長押しで電源を切る。0.5 秒後に
`*** 脱力 (トルクOFF) ***` がログに出て `/estop` が true になれば正しい。

`colcon test` にデッドマン・ウォッチドッグ・脱力ラッチ・「並行移動に旋回が混ざらない」
・無線テストの結線を見る機能テストが入っている (`test/test_teleop.py`)。コントローラ
無しで走る。

## 7. まだ決めていないこと

* `scale.*` (最高速) は仮値。歩行パターン生成 (作る順番 §5 の 3) ができてから実測で詰める。
  今は「歩けるより遅い」側に振ってある。
* `motion_bindings` の技名 (`punch_r` / `punch_l` / `getup_front` / `getup_back`) は
  teleop 側で決め打ちしたもの。motion ノードを書くときにこの名前に合わせるか、
  YAML を書き換えるか、どちらかで揃える。
* 起き上がりの「前 / 後」は**倒れた向き**で名付けてある (`getup_front` = うつ伏せから、
  `getup_back` = 仰向けから)。motion 側と解釈を合わせること。
* behavior と teleop が両方 `/cmd_walk` を出す衝突は、当面「同時に起動しない」運用で回避
  (ros-architecture §2)。両立が必要になったら `twist_mux`。
