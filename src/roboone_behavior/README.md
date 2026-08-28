# roboone_behavior

自律動作の行動層。相手の位置とリングの縁から、20 Hz で歩行指令 `/cmd_walk` と
技 `/cmd_motion` を決める `behavior` ノード。

仕様は [docs/behavior_planning.pdf](../../docs/behavior_planning.pdf)（技術ノート(3)
『自律機の動作計画指針』）。ノード構成は `docs/ros-architecture.md`、歩行との
境界は技術ノート(2)『歩行制御の ROS 2 実装設計』に従う。

## 何をするか

優先順位付きの状態機械を 20 Hz で回して、指令を 1 つに落とす。

| | 状態 | すること | 入る条件 |
|---|---|---|---|
| 1 | `WAIT` | 何も出さない | `/autonomy` が false、または `/estop` |
| 2 | `SELF_DOWN` | 待つ（起き上がりは motion の仕事） | `/motion/state` が FALL / ESTOP |
| 3 | `EDGE` | 中央へ戻る | 縁の余裕 `d_margin` を割った |
| 4 | `RETREAT` | 離れて復帰を待つ | 相手が転倒中（規則 10.2(b)(i)） |
| 5 | `ENGAGE` | 技を 1 回渡す | 立っている相手が間合い `ρ_s` かつ正面 |
| 6 | `APPROACH` | 相手の `ρ_s` 手前まで直線で歩く | 見えていて正面 |
| 7 | `SEARCH` | 旋回して視野に入れる | それ以外 |

上の 3 つはどの状態にも割り込む。状態を変えたら最低 `T_dwell` は留まり、閾値は
全部ヒステリシス付きにしてある。

**上げただけでは機体は動かない。** `/autonomy` が true の間しか `/cmd_walk` を
出さない。true にするのは teleop の長押しで、これが規則 5.1.2 の無線始動機構を
兼ねる。止めるのも teleop の割り込み 4 つ（起き上がり / 脱力 / 無線確認 /
ホームポジション）のいずれか。

## トピック

| トピック | 型 | 向き |
|---|---|---|
| `/opponent` | `roboone_interfaces/Opponent` | opponent_detector → |
| `/ring_edge` | `std_msgs/Float32MultiArray` | opponent_detector → |
| `/motion/state` | `std_msgs/String` | motion → |
| `/odom` | `nav_msgs/Odometry` | motion →（**未実装。無くても動く**） |
| `/autonomy` | `std_msgs/Bool` (latched) | teleop → |
| `/estop` | `std_msgs/Bool` (latched) | teleop → |
| `/cmd_walk` | `geometry_msgs/Twist` | → motion（20 Hz） |
| `/cmd_motion` | `std_msgs/String` | → motion（イベント） |
| `/behavior/state` | `std_msgs/String` | → 記録・ui（状態変化時） |
| `/behavior/debug` | `std_msgs/Float64MultiArray` | → 記録・PlotJuggler（20 Hz） |

`/autonomy` と `/estop` は latched（TRANSIENT_LOCAL）で購読する。behavior を
teleop より後に上げても直前の状態が届くようにするためなので、**この 2 つを出す側は
必ず latched で出すこと。** VOLATILE で出すと QoS が合わず 1 通も届かない。

`/behavior/debug` の列の意味は `behavior/types.py` の `DEBUG_ORDER`。列を足すときは
末尾に足し、既にある列の順番は変えないこと（過去の bag が読めなくなる）。

## 中身

判断は ROS に依存しない `roboone_behavior/behavior/` に閉じている。ノードは
トピックとパラメータの面倒を見るだけで、判断を 1 つも持たない。

| ファイル | 中身 |
|---|---|
| `params.py` | 定数。`match`（競技）/ `robot`（機体）/ `tune`（実装）の 3 群 |
| `types.py` | `Observation` と `Command`、状態の名前、デバッグ列の並び |
| `tracking.py` | 相手の追跡（§2.2）と転倒判定（§2.3） |
| `ring.py` | リング座標系の自己位置、八角形の縁、先読みの縮小（§2.4, §4.8） |
| `keepalive.py` | 止まらないための小移動（§4.9、規則 10.3(d)） |
| `core.py` | 状態機械そのもの（§3, §4） |

`BehaviorCore` は時計も乱数も持たない。`Observation` を順に流せば決定的に同じ
指令列が出るので、試合の bag から観測列を作り直せば机上で再現できる。

## 文書から変えたところ

実装して合わなかった箇所。理由は各ファイルの冒頭にも書いてある。

1. **「はじめ」の合図が `/match/cmd` (String) ではなく `/autonomy` (Bool)。**
   teleop が先に `/autonomy` で実装されている。規則 5.1.2 の機構としては同じもので、
   行動層から見ると hold と stop の区別が要らない。
2. **間合いの中での転倒判定を凍結しない**（`tracking.py` の `_step_close`）。
   文書は `ρ < ρ_s + 0.2 m = 0.45 m` で判定を凍結するが、`ρ_s = 0.25 m` なので
   **間合いに入ってから相手が倒れた場合を一度も検出できない**。技を当てて倒した
   直後がまさにそれで、規則 10.2(b)(i) が効く場面を取り逃がす。復帰の判定だけ
   凍結を残し、転倒の判定は「上端が低い」に加えて「横に広い」を要求して通す。
3. **先読みの縮小に下限を付けた**（`ring.py` 冒頭）。字義どおりだと、余裕を
   割った位置で EDGE の「中央へ戻る」指令までゼロになり縁から動けない。
   「今より悪化させない指令なら通す」に変えてある。
4. **`/odom` が無くても動く。** motion ノードがまだ無いので、来ないうちは自分が
   出した指令を積分する。来れば自動で乗り換える。滑りも指令とのずれも、式 (6) の
   余裕 `d_margin` が歩行距離に比例して吸収する前提は同じ。
5. **相手の陰の切り分けを検出器に任せた。** 文書は行動層で「相手までの距離と
   同じ切れ目は縁ではない」と切るが、検出器が既に遮蔽を NaN にしている。二重に
   切ると、相手が本物の縁の近くに立っているときに本物の縁を捨ててしまう。
6. **しゃがみ後の「3 歩」を時間で数える。** 行動層に歩の境界は見えない。並進
   指令を出していた時間を `T_step` で割る。規則に対しては安全側に外れる。
7. **検出器が外挿しているフレームは観測として取り込まない。** 取り込むと見失いの
   タイマが戻る。外挿は自分の動きを知っているこちらでやるほうが正確でもある。
8. **`/motion/state` の状態名が文書と違う。** 実機は `RELAX`/`ARMING`/`HOLD`/
   `WALK`/`MOTION:<技名>` を出す（下の「motion との接点」）。歩けない状態を
   `SELF_DOWN` で受けるようにしてある。

## motion との接点

`roboone_motion_node` の実装に合わせてある（`motion_node.cpp`）。文書が想定して
いた歩行ノート(2) の状態名とは違うので、読み方をここに残す。

`/motion/state` が出すのは `RELAX` / `ARMING` / `HOLD` / `WALK` / `MOTION:<技名>`
の 5 つで、latched。行動層はこう解釈する。

| 文字列 | 行動層の扱い |
|---|---|
| `RELAX` | 脱力。歩けないので `SELF_DOWN`（指令を出さず、位置の推定も進めない） |
| `ARMING` | トルク投入中。同上 |
| `HOLD` | 立って待機。歩ける |
| `WALK` | 歩行中。歩ける |
| `MOTION:<技名>` | 技の再生中。歩行指令を出さず、終わるまで `ENGAGE` に留まる |

技名が**コロン区切り**で付くのがここだけの癖で、読み違えると技の再生中に歩行
指令を重ねることになる。`test_parses_the_motion_state_strings_the_real_node_emits`
で固定してある。歩ける状態の一覧は `motion_ready_states` パラメータ、技の再生中を
表す語は `motion_busy_tokens` パラメータで、歩行ノート(2) の名前
（`IDLE`/`START`/`STEP`/`STOP`）も将来のために入れてある。

`RELAX` を素通りさせないのが効くのは、指令を出しても機体が動かないときに
**リング座標系の推定だけが「歩いたつもり」で進んでしまう**からである
（`/odom` が無い間の推定は自分の指令の積分なので）。

### まだ足りないもの

- **`/odom` がまだ無い**（motion は publish していない）。出してくれれば自動で
  乗り換える。歩の境界ごとの支持足の乗り換えを累積したもの＋IMU ヨー。
  それまでは指令の積分なので、`tune.edge_margin_slip` は大きめにしてある。
- **転倒を報せる状態が無い。** motion の状態に `FALL` にあたるものが無いので、
  自分が転んだことを行動層は知れない（`SELF_DOWN` には入れない）。今は
  「転んだら人が脱力させる」→ `RELAX` → `SELF_DOWN` で受けている。
  転倒判定を motion が持つなら、状態名を 1 つ足してもらえれば拾う。
- **足踏みの維持ができない。** 文書 §4.9 は「歩行指令が零でも足踏み（STEP, v=0）
  のままにする」としているが、`/cmd_walk` に「零だが足踏みは続ける」を表す口が
  無い（歩行ノート §4.3 では v=0 は STOP そのもの）。今は小移動だけで規則
  10.3(d) を満たしている。足踏みが要るなら motion 側に march in place の口を。

## 使う

```bash
colcon build --packages-select roboone_behavior
source install/setup.bash

# 行動層だけ（検出器と motion は別で上げてある）
ros2 launch roboone_behavior behavior.launch.py

# 検出器とカメラごと
ros2 launch roboone_behavior behavior.launch.py detector:=true camera:=true

# 技を出さずに歩行だけ見る
ros2 launch roboone_behavior behavior.launch.py techniques:="[]"

# 機体一式に混ぜる
ros2 launch roboone_bringup roboone.launch.py camera:=true detector:=true behavior:=true
```

見るとき:

```bash
ros2 topic echo /behavior/state          # 状態と、そこへ入った理由
ros2 topic echo /cmd_walk
ros2 param set /behavior tune.k_bearing 2.0   # tune.* は実行時に変えられる
```

`match.*` と `robot.*`（技名を除く）は実行時に変えられない。前者は規則から
決まる数で、後者はリング座標系の推定の前提そのものだから。

## テスト

```bash
colcon test --packages-select roboone_behavior
colcon test-result --verbose --test-result-base build/roboone_behavior

# ROS 抜きで直接
python3 -m pytest src/roboone_behavior/test/test_behavior.py -q
```

合成の観測列（`test/sim.py`）で状態遷移と規則の制約を固定してある。滑りも遅れも
入っていない運動学だけの模擬なので、**実機の挙動を主張するものではない。**
「文書が挙げた条件から外れないこと」の据え付けである。

## 実機で詰める順（文書 §6）

平らな床で 1〜4、段差が要るのは 5 だけ。各段で `/behavior/debug` を bag に残す。

1. 静止した箱に対する `ρ, β` の精度。ここで深度の死角 `ρ_blind` を実測して
   `robot.blind_range` に入れる
2. 箱を視野外に置いて `SEARCH`。回る向きが最後の方位側であること
3. `APPROACH` の停止距離の分布。`tune.k_range` と `tune.range_eps` を詰める
4. 箱を横倒しにして転倒判定 →`RETREAT`。立て直したら復帰すること
5. 低い台の上で縁。`d_cliff` が台の縁と合うこと。**転落防止に紐を付ける**
6. 小移動。無移動が `T_ka` を超えないことをログで確かめる
7. 通しの模擬試合

## 未確定

`docs/behavior_planning.pdf` §7 の表がそのまま残っている。特に効くのは

- 開始位置と向き（`robot.start_*`）— リング座標系の初期化。赤青コーナーの運用を確認する
- 深度の最短距離（`robot.blind_range`）— 近距離死角の扱いと `ρ_s` の関係
- 技の間合いと再生時間（`robot.strike_range`）— 技ができてから
- 脚オドメトリの滑り（`tune.edge_margin_slip`）— 直進 1 m と旋回 1 周の誤差を測る
- リングの段差の高さ — 切れ目の検出が成立するか
