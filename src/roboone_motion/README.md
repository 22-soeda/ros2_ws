# roboone_motion — motion ノード（200Hz ループの本体）

`/cmd_walk`・`/cmd_motion`・`/estop` を受けて、歩行計画と技を走らせ、IK を通して
Feetech サーボへ送る。`/camera/imu`（RealSense 内蔵 IMU）から胴体の傾きを推定し、
足首と股で立位・歩行を安定化する（下の「IMU の安定化」）。
**実機に位置指令を書くのは、通常運用ではこのノードだけ。**
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
  imu_attitude.hpp   IMU の生値 -> 胴体のロール・ピッチと角速度
  stabilizer.hpp     IMU の安定化。出力は PoseCodec へ渡す補正（PoseCorrection）
src/
  motion_node.cpp    ROS の殻。パラメータを読んで層を組み、200Hz で回してログを流す
```

上の 4 層 + `servo_map` + `body_pose` + `motion_library` が
`libroboone_motion_core.so`（ROS 非依存）。**`motion_node` と `motion_teach` が
同じものを使う。姿勢の解釈を 2 実装にしない。**

### 1 周期はこれだけ

```
bank_.states()  ->  codec_.decode()  ->  ctrl_.step()  ->  stab_.update()
                ->  codec_.encode(目標, 補正)  ->  bank_.setTargets()
```

### スレッド

| | 周期 | 仕事 |
|---|---|---|
| `main` | — | `rclcpp::spin`。購読コールバックが値を controller へ置くだけ。`/camera/imu` は姿勢推定まで回す（数 µs） |
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
/estop ────┘   状態機械 / WalkPlanner / MotionPlayer        │ bodyPitchApply (Σ_U→Σ_B)
                          │ 支持脚・位相                     │   └ + 胴体の補正
/camera/imu → ImuAttitude → Stabilizer ─→ PoseCorrection ──→│ IK
                                                            │   └ + 足首の補正
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
**膝・足首の関節角は 1 度も変わらない**（足首ピッチ θ5 の余裕を食わずに前傾できる）。

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
   │  （実測と経路の検査が通るまで待つ）    │                  │      ↑
   │                                      └─hold なら─→ STAY  │  /cmd_walk
   └── /cmd_motion "home"/"hold" は        （目標を作らない）    │
       「予約」だけして RELAX のまま                    技 ──→ MOTION
```

### 順序の約束 — 崩すと実機で事故る

**全部、実機でしか踏めなかったバグの修正。** `motion_selftest [5]` が守っている。

1. **技の要求を武装判定より先に捌く。** teleop は `home` →(0.1s)→ `/estop false`
   の順に来る。逆順だと最初の 1 本が「立ち上げ中なので出さない」で捨てられる
2. **補間の起点は「その周期の」実測サーボ角。取れない・経路が通らないなら武装しない。**
   起点が無いと補間にならず、「トルクが入るだけで動かない」（2026-08-28 実機）。
   起点はサーボ角で持ち、補間もサーボ角で回す（下の「武装の起点」）
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

### 武装の起点はサーボ角（2026-09-18）

4) の補間は**実測のサーボ角 → 保持姿勢のサーボ角（IK の影）**で回す。`decode()` の
実測姿勢は `LegMode::Servo` で、足裏は FK の影（表示用）でしかない。

以前は足裏で補間していたので起点に FK が要り、FK が解けないと武装しなかった。
脱力した足首は FK の箱（`CRANK_LIMIT_DEG` = -42..+60 deg）の外に垂れているのが普通で
（2026-09-17 の bag: 両脚ともクランク ≈ +94 deg、`status=8` = `AnkleClamped`）、
毎回手で足首を戻していた。しかもその姿勢はロッドが死点を越えていて、FK の足裏を
IK で戻すと別のクランク角（+83 deg）になる = 足裏で補間すると初周期に跳ぶ。

代わりに、トルクを入れる前と補間を始める前に次を確かめる（`rk::legServoPath`）。

| 見るもの | 止まる例 |
|---|---|
| 全軸のカウントが 0-4095 の中 | 多回転の軸（足首 ID5/ID6・股ヨー ID3）の巻き数ずれ |
| 膝が組める | 膝の読みが機構としてあり得ない |
| 足首クランクが `ARM_CRANK_LIMIT_DEG` = -42..+170 deg の中 | 両クランクを - へ振り切った（型 2 特異点の側） |
| 起点から保持姿勢まで、サーボ角の直線上で膝・足首が組めたまま通る | 0.5 deg 刻みで順変換。特異点に触れる・跳ぶでも止まる |

箱の中の始点はどれも組めて、設計域・ホーム姿勢・エンベロープの端のどれへも直線で
通れる（`ankle_selftest` の「武装の経路」）。止まったら `/motion/events` の
「武装しない: ...」に理由が出る。

補間の形は変わる。足裏の直線ではなく関節の直線なので、**接地したまま**起点と保持姿勢が
大きく違うと足裏が擦れる。機体を支えて（浮かせて）トルクを入れる運用なら差は無い。

## 動歩行と静歩行（`walk_planner.hpp`）

歩行の計画器は**起動時に 1 回だけ**選ぶ（パラメータ `walk_mode`。launch の
`walk_mode:=dynamic|static`。read_only なので実行中は変えられない）。

| walk_mode | 計画器 | 設定 | 歩き方 |
|---|---|---|---|
| `dynamic`（既定） | `rwc::WalkEngine`（walk_core） | `gait.yaml` | DCM・純フィードフォワード。1 歩 0.6 s |
| `static` | `rwc::StaticWalkEngine`（static_walk） | `static_gait.yaml` | 両足支持で重心を支持足の上へ移し（SHIFT）、止めたまま足を振り出す（SWING）。1 歩の時間は `static_gait.yaml` 次第（2026-09-18 は観察用に約 6.4 s） |

どちらも出力は `rwc::WalkOutputs` なので、足先の組み立て（`walkFeet()`。計画の立位を
ホーム姿勢の足へ平行移動する）・IK・安定化・`/motion/stab` は共通。モードで違うのは

- 計画上の足間隔と重心高さ（`WalkSetup::planFootSpacing()` / `planZc()`）
- 遊脚の時間と接地の位相（`WalkSetup::swingTiming()`。安定化の着地ゲート）
- 状態の番号（静歩行は `walk_state` 5 = SHIFT / 6 = SWING。片足支持は `singleSupport()`）
- 起動時の門（下の表）
- `/motion/state` の末尾（静歩行だけ ` walk=static` が付く。behavior は先頭の語だけ読む）

★静歩行は `static_gait.yaml` の `foot_spacing` を **home_pose.yaml の foot.y の 2 倍**と
揃える前提。ずれた分だけ振り出し中の重心が支持足の中心から横へずれる
（`checkStaticStance` が言う）。

## 起動時の門（`motion_config.hpp`）

解けない config を実機で踏むと「動かない」のか「解けていない」のかが現場で
切り分けられない。**走り出す前に言葉で出す。**

| 門 | 見るもの |
|---|---|
| `checkPoseReachable` | ホーム姿勢が IK で解けるか |
| `checkMotionLegServo` | 角度書きのキーフレームが `servo_limits.yaml` の窓に入るか |
| `checkGait` | 遊脚が本当に床へ届くか（降下は `td_speed_max` で飽和する） |
| `checkStance` | 歩行の立位（= ホーム姿勢の足）と計画上の足間隔の関係。計画が実機の足より狭ければ警告 |
| `checkWalkEnvelope` | 歩行が指令しうる足先の箱が IK の到達域に収まるか |
| `checkStaticGait` | （静歩行）遊脚が床へ届くか・重心の横ずらしが足裏に収まるか。1 歩の時間と速さも言う |
| `checkStaticStance` | （静歩行）計画の足間隔とホーム姿勢の足の食い違い = 振り出し中の重心のずれと静的余裕 |
| `checkStaticWalkEnvelope` | （静歩行）11 通りの指令で計画を回し、**全時刻の足先**が機構の到達域に入るか（約 60 ms） |

`checkGait` / `checkStance` / `checkWalkEnvelope` は動歩行のとき、`checkStatic*` は
静歩行のときだけ通す。

`checkWalkEnvelope` は `roboone_walk_core/src/gait_from_kinematics.cpp` の逆向き。
あちらは到達域から `gait.yaml` を**決める**、こちらは入っている値で本当に届くかを
**確かめる**。片方だけ手で書き換えたときに気付くための門。

**`motion_selftest --strict` で実機なしに全部通せる。**

## IMU の安定化（`imu_attitude.hpp` / `stabilizer.hpp`）

設計の原本は `docs/imu_biped_walking.pdf` §4 と `docs/ros2_walk_implementation.pdf` §5・§9。
そのうち**段 1〜3（足首のジャイロ減衰・傾きの比例・胴体の補正）**を入れてある。
踏み出し補正（推定 ξ を walk_core の着地点へ入れる段 7・8）と接地判定・転倒検知はまだ無い。

| | 中身 |
|---|---|
| 姿勢推定 | `/camera/imu` をジャイロで積分し、加速度の重力方向へ `tau_c` で引き戻す相補フィルタ。**歩行計画の重心加速度 ω²(x_C − p) を比力から引いてから**使う（横揺れで傾きがずれないように）。零点は `/motion/imu_zero`（カメラは水平付けなので、組み付けのずれを取るときだけ） |
| 足首戦略 | 支持脚の足首 θ5 / θ6 に `kp·傾き + kd·角速度` ぶんの回転を足す。「胴体から見て足裏を回す量」を足首 2 軸へ直す行列は、ホーム姿勢の FK から数値で取る |
| 胴体の補正 | `body_pitch` に `−k_torso·ピッチ` を足す（前へ倒れたら胴体を起こす） |
| 効かせ方 | HOLD / WALK のときだけ。遊脚は離地で抜いて着地前に戻す。着地の前後はゲイン半分。技・その場保持・武装中は `fade_time` で抜き、脱力で即 0。IMU が古ければ抜く |
| 掛ける場所 | **目標姿勢には混ぜず**、`PoseCodec::encode()` の境界でだけ掛ける。補正を入れると解けない周期は補正を外して出す |

**ゲインの既定は全部 0。** 実行中に `ros2 param set /motion stab.kd_pitch 0.05` のように
上げる（範囲は descriptor で縛ってある）。手順と見るもの（`/motion/stab`）は
`docs/commands.md`「IMU の安定化」。

★向き（符号）は `motion_selftest` の [8] が順運動学で検算している。前へ倒れているときに
胴体から見てつま先を下げる（= 押し戻す）向きでなければ落ちる。文書の式 (47) の
R_BL をそのまま `body_pitch` に読み替えると胴体の補正の向きが逆になる点は
`stabilizer.hpp` の冒頭に書いた。

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
ros2 launch roboone_motion motion.launch.py walk_mode:=static    # 静歩行で起動

# ティーチ（スペースで捕まえる。--keep-torque なら書き込みゼロ）
ros2 run roboone_motion motion_teach > punch_r.yaml
ros2 run roboone_motion motion_teach --format angle   # 脚をサーボ角で出す

# 自己検算
ros2 run roboone_motion motion_selftest
ros2 run roboone_motion motion_selftest --strict      # 設定の門のエラーも失敗にする
#   [7] IMU の姿勢推定、[8] 安定化の向き、[9] 静歩行も見る
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

## 足裏の座標を打って片脚を動かす `motion_leg_goto`

脚 IK 全体（IK → 膝 4 節 → 足首パラレル → サーボ角 → 生カウント）を実機で確かめる。
motion ノードと同じ変換（`body_pose.hpp` の `servoFromFootPose` / `footPoseFromServo`、
`ServoMap` の生カウント変換）を通すので、ここで出るカウントはノードが出すものと一致する。

```bash
ros2 run roboone_motion motion_leg_goto --leg R --p 0 -89.3 -261 --rpy 0 0 0         # 計算と現在値だけ
ros2 run roboone_motion motion_leg_goto --leg L --rel --p 0 0 -261 --rpy 0 -20 0     # 股中心からの相対
ros2 run roboone_motion motion_leg_goto --leg R --p 0 -89.3 -261 --move              # ★実機が動く
ros2 run roboone_motion motion_leg_goto --leg L --move --repl    # ★対話。1 行 "x y z [roll pitch yaw]"
ros2 run roboone_motion motion_leg_goto --leg R --off            # その脚 6 軸のトルクを切る
```

- 座標は機体座標 Σ_B（x 前 / y 左 / z 上、原点は股 3 軸の高さ）[mm]、rpy は足裏の姿勢 [deg]
  （`home_pose.yaml` の `foot` と同じ取り方）。股中心は右 (0, −89.3, 0) / 左 (0, +89.3, 0)。
  `--rel` で x, y を股中心からの相対にできる。
- **★★脚 6 軸がまとめて動く。** 必ず機体を吊るか、脚が空中にある姿勢で使う。既定は読むだけで、
  `--move` を付けたときだけトルク ON と位置指令を出す。6 軸は 1 パケットで同時に動く。
- 動かさない条件: IK が解けない / 膝・足首の機構が届かない（足首はエンベロープで丸めたときも）/
  関節リミットの外 / カウントが `servo_limits.yaml` の窓の外。理由を印字して止まる。
- 動かしたあとは実測カウントを順変換で足裏の姿勢に戻し、指令との差を出す。
- 終了時トルクは入ったまま。`--off` は単独でも使える。

