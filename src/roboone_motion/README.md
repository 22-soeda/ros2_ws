# roboone_motion — motion ノード（200Hz ループの本体）

`/cmd_walk`・`/cmd_motion`・`/estop` を受けて、歩行計画と技を走らせ、IK を通して
Feetech サーボへ送る。**実機に位置指令を書くのは、通常運用ではこのノードだけ。**
`ros-architecture` §3 の「200Hz ループ」の実体。

同じバスを 2 プロセスが掴むと壊れるので、`motion_node` を上げたまま
`feetech_shell` や `motion_teach` を叩くことはできない。

## 層の分け方

**ノードは 1 つ。ファイルは 4 層に分けてある。** どの層も ROS を知らないので、
実機なしで `motion_selftest` から叩ける。

```
include/roboone_motion/
  side.hpp           左右の添字だけの最小ヘッダ
  event.hpp          層から上へ渡す「出来事」。ログの代わり
  servo_map.hpp      角度 3 種の橋（生カウント / T ポーズ基準角 / 絶対サーボ角）
  body_pose.hpp      機体の 1 姿勢の型。IK・FK を呼ぶ場所をここ 1 か所に絞る
  servo_bank.hpp     ★サーボ層  2 バスの開閉・スレッド・トルク・生カウントの授受
  pose_codec.hpp     ★変換層    BodyPose <-> 生カウント（Σ_U / Σ_B の境界）
  motion_config.hpp  ★設定層    YAML の読み込みと起動時の門
  motion_library.hpp キーフレーム技の読み込みと再生
  motion_control.hpp ★生成層    状態機械・歩行・技の再生
src/
  motion_node.cpp    ROS の殻。パラメータを読んで層を組み、200Hz で回してログを流す
```

上の 4 層 + `servo_map` + `body_pose` + `motion_library` が
`libroboone_motion_core.so`（ROS 非依存）。**`motion_node` と `motion_teach` が
同じものを使う。姿勢の解釈を 2 実装にしない。**

### 1 周期はこれだけ

```
bank_.states()  ->  codec_.decode()  ->  ctrl_.step()  ->  codec_.encode()
                                                       ->  bank_.setTargets()
```

### スレッド

| | 周期 | 仕事 |
|---|---|---|
| `main` | — | `rclcpp::spin`。購読コールバックが値を controller へ置くだけ |
| `control` | 200Hz | 上の 1 周期。**シリアルを触らない** |
| `bus` ×2 | 書き 200Hz / 読み 50Hz | `ServoBank` が持つ。1 パケットで書く + 一定周期で読む |

制御ループをシリアルから切り離してあるのは、**読み出しが 1 往復で数 ms かかり、
応答が欠けると最大 `timeout_ms`（20ms）待つ**ため。同じスレッドに置くと 200Hz の
周期が読み出しの都合で崩れる。書き込みは送りっぱなし（TX のみ）なので速い。

スレッド間の受け渡しは `ServoBank` の中の 2 本の配列だけ。

## データの流れ

```
/cmd_walk ─┐
/cmd_motion├→ MotionController ─→ BodyPose [mm・Σ_U] ─→ PoseCodec.encode()
/estop ────┘   状態機械 / WalkEngine / MotionPlayer         │ bodyPitchApply (Σ_U→Σ_B)
                                                            │ IK + ankleClampJoints
                                                            │ legServoFromJoints
                                                            │ servo_limits.yaml で丸め
                                                            ↓
                                              ServoBank.setTargets() ─→ [Feetech 2 バス]
                                                            ↑
       BodyPose [Σ_U] ←─ PoseCodec.decode() ←─ ServoBank.states()
         bodyPitchRemove ← FK ← legJointsFromServo + ankleFk ← 生カウント
```

## 押さえる 3 つの概念

### 角度が 3 種類ある（`servo_map.hpp`）

| | 何 | どこで使う |
|---|---|---|
| **[1] 生カウント** | 0-4095 | サーボが実際にやり取りする唯一の量 |
| **[2] T ポーズ基準角** | `servo_home.yaml` の home を 0 deg とした角 | **人が読み書きする**（`motions.yaml` の `R_leg`、ティーチの出力、腕） |
| **[3] 絶対サーボ角** | `leg_servo.hpp` がやり取りする角 | **機体の中を流れる**（IK/FK の境界） |

[1]↔[2] は素直。**[2]↔[3] のズレ**（膝は伸び切りが T ポーズ、足首は鎖ごとの原点）が
`ServoMap` の主眼で、`leg_service.cpp` の `servo` コマンドが同じ足し算をしている。
**あちらと数値が食い違ったらどちらかが壊れている。**

脚 6 軸の ID は `{1, 2, 3, 4, 6, 5}`。**足首は 6, 5 の順**（`ankle_parallel.hpp` の
鎖 0 = 短ロッド = ID6 に合わせる）。5, 6 にするとロールとピッチが入れ替わる。

### 座標系が 2 つある（`body_pose.hpp`）

| | 何 |
|---|---|
| **Σ_U** | 骨盤が直立している前提の系。**walk_core・motions.yaml・home_pose.yaml が書いている系** |
| **Σ_B** | 実際の骨盤系。胴体が `body_pitch` だけ前傾している |

胴体を ψ 前傾させるのは**股ピッチ θ1 に −ψ を足すのと厳密に等価**なので、
**膝・足首の関節角は 1 度も変わらない**（足首 θ6 の余裕を食わずに前傾できる）。

**変換を掛けるのは `PoseCodec` だけ。** 上の層（歩行・技・補間・ホーム）は前傾を
知らない。`encode` の `bodyPitchApply` と `decode` の `bodyPitchRemove` は対で、
片方だけ直すと**トルクを入れた瞬間に前傾ぶん跳ねる**。

### 脚の書き方が 2 つある（`LegMode`）

| | 書き方 | 指令の経路 | 取り柄 / 裏返し |
|---|---|---|---|
| `Foot` | 足裏の (p, R) | **IK を通る** | 寸法が変わっても IK が吸収 / **IK で解ける姿勢しか書けない** |
| `Servo` | サーボ角 | **IK も FK も通らない** | 可動域の縁・特異点・寝た姿勢も書ける / 組み替えると付いてこない |

「順変換では出せるのに IK では戻せない」領域は実際にある。そこを狙う技のための
逃げ道が `Servo`。歯止めは `servo_limits.yaml` の窓だけなので、**起動時の門
（`checkMotionLegServo`）が唯一の事前チェック**になる。

## 状態機械（`motion_control.hpp`）

```
        /estop true（どこからでも即座に）
   ┌────────────────────────────────────┐
   ↓                                    │
 RELAX ──/estop false && canArm()──→ ARMING ──補間終了──→ HOLD ⇄ WALK
   │        （実測姿勢が取れるまで待つ）    │                  │      ↑
   │                                      └─hold なら─→ STAY  │  /cmd_walk
   └── /cmd_motion "home"/"hold" は        （目標を作らない）    │
       「予約」だけして RELAX のまま                    技 ──→ MOTION
```

### 順序の約束 — 崩すと実機で事故る

**全部、実機でしか踏めなかったバグの修正。** `motion_selftest [5]` が守っている。

1. **技の要求を武装判定より先に捌く。** teleop は `home` →(0.1s)→ `/estop false`
   の順に来る。逆順だと最初の 1 本が「立ち上げ中なので出さない」で捨てられる
2. **実測姿勢が 1 度も取れていないうちは武装しない。** 起点が無いと補間にならず、
   「トルクが入るだけで動かない」（2026-08-28 実機）
3. **`WALK → HOLD` は `walk_idle_hold` 待ってから。** 歩き始めは歩行エンジンが
   IDLE と START を 55ms 周期で往復する（2026-08-28 実機）
4. **その場保持の武装後は `HOLD` ではなく `STAY`。** `HOLD` だと `tickWalk` が
   足先を立位のスタンスへ上書きするので、寝た姿勢からだと跳ねる
5. **`/estop true` はどの状態からでもトルクの要求を下ろす。** RELAX のまま武装待ち
   （`want_torque = true` だが `torque_ready` がまだ）の最中に来ても落とす

`/motion/state` の書式は **behavior がテストで固定している**ので変えない。足すなら
空白区切りで後ろに（詳細は `motion_control.hpp` の `stateText()`）。

## トルクの入れ方 — ここが事故の起きる場所（`servo_bank.hpp`）

**Feetech は目標角レジスタが生きたままトルクが入る。** 前回の目標角が残っている
ところへトルクを入れると**そこへ全速で飛ぶ**。だから必ず 3 段:

```
1) 実測位置を読む
2) その実測位置を目標として書く   ← これで「今いる場所」が目標になる
3) トルクを入れる                 ← 動かない
4) 実測姿勢から保持姿勢へ torque_on_time 秒かけて補間（MotionController）
```

さらに **実測が揃わないうちは入れない**（読めなかった軸だけが古い目標角へ飛ぶ）。
ここで止まるときは**まず電源を疑う**（低電圧で応答が間欠的に欠ける実機の癖）。

## 起動時の門（`motion_config.hpp`）

解けない config を実機で踏むと「動かない」のか「解けていない」のかが現場で
切り分けられない。**走り出す前に言葉で出す。**

| 門 | 見るもの |
|---|---|
| `checkPoseReachable` | ホーム姿勢が IK で解けるか |
| `checkMotionLegServo` | 角度書きのキーフレームが `servo_limits.yaml` の窓に入るか |
| `checkGait` | 遊脚が本当に床へ届くか（降下は `td_speed_max` で飽和する） |
| `checkStance` | 歩行の足間隔が実機の股間隔と揃っているか |
| `checkWalkEnvelope` | 歩行が指令しうる足先の箱が IK の到達域に収まるか |

`checkWalkEnvelope` は `roboone_walk_core/src/gait_from_kinematics.cpp` の逆向き。
あちらは到達域から `gait.yaml` を**決める**、こちらは入っている値で本当に届くかを
**確かめる**。片方だけ手で書き換えたときに気付くための門。

**`motion_selftest --strict` で実機なしに全部通せる。**

## 実行形

| | 何 | サーボへの書き込み |
|---|---|---|
| `motion_node` | 本体 | **する**（`allow_torque` 既定 true）|
| `motion_teach` | 脱力させた機体を手で構えて姿勢を捕まえる CLI | 起動時のトルク OFF 1 回だけ |
| `motion_selftest` | 自己検算。実機もサーボも要らない | しない |

```bash
# 本体（★機体が動く。起動前に支えること）
ros2 launch roboone_motion motion.launch.py
ros2 launch roboone_motion motion.launch.py allow_torque:=false  # 読むだけ
ros2 launch roboone_motion motion.launch.py dry_run:=true        # バスも開かない

# ティーチ（スペースで捕まえる。--keep-torque なら書き込みゼロ）
ros2 run roboone_motion motion_teach > punch_r.yaml
ros2 run roboone_motion motion_teach --format angle   # 脚をサーボ角で出す

# 自己検算
ros2 run roboone_motion motion_selftest
ros2 run roboone_motion motion_selftest --strict      # 設定の門のエラーも失敗にする
ros2 run roboone_motion motion_selftest --gait /tmp/g.yaml --home-pose /tmp/h.yaml --strict
```

**実装・デバッグの過程で実機を走らせるときはトルクを入れない**
（`allow_torque:=false` か `dry_run:=true`）。CLAUDE.md の約束。

## config の置き場

| 何 | どこ |
|---|---|
| ノードのパラメータ | `roboone_motion/config/motion_node.yaml` |
| 技のキーフレーム | `roboone_motion/config/motions.yaml` |
| **歩行の設定** | `roboone_walk_ref/config/gait.yaml` |
| **ホーム姿勢** | `roboone_walk_ref/config/home_pose.yaml` |
| 原点 / リミット | `feetech_servo/config/servo_home.yaml`, `servo_limits.yaml` |

歩行とホーム姿勢の原本が `roboone_walk_ref` にあるのは、**歩行計画の仕様原本
（Python 版 walk_core）と同じ値を実機が読む**ようにするため。share から読むだけで、
コードの依存は無い（`exec_depend`）。

ホームポジションを `motions.yaml` に書くと二重定義になるので、読み込み時に警告して捨てる。

## 出すトピック

| トピック | 中身 |
|---|---|
| `/motion/state` | 状態（latched・変化時） |
| `/joint_states` | 実測の**関節角** [rad] |
| `/motion/joint_commands` | 指令の**関節角** [rad]。同じ並び・同じ名前 |
| `/motion/servo_states` | サーボ空間。position=実測カウント / velocity=**目標カウント** / effort=負荷 |
| `/motion/servo_current` | 電流。position=サンプル数 / velocity=区間平均 / effort=**区間ピーク** |
| `/motion/diagnostics` | バスごとの電圧・最高温度・応答軸数・エラービット |

- **指令と実測を両方出す**のは、「荷重で沈む」を後から見るには**差**が要るから
- **関節空間とサーボ空間を両方出す**のは、間に膝 4 節リンクが挟まっていて
  「リンクがたわんでいる」（関節空間だけに差）と「サーボが追従していない」（両方に差）
  を分けたいから
- **電流をピークホールドする**のは、読み 50Hz / 出し 10Hz でそのまま出すと
  踏ん張った瞬間の突入電流を取りこぼすから。サンプル数を一緒に出すのは、
  低電圧で応答が欠けたときにピークの信用度が落ちるのが見えるように

`velocity` を「目標カウント」に借りているのは JointState に目標の置き場が無いため
（`record_topics.yaml` にも明記）。

## 触るときに読む順

1. この README
2. 変えたい層のヘッダ冒頭（実機で踏んだ話は全部そこに書いてある）
3. `motion_selftest` を通す。config を触ったなら `--strict` も
