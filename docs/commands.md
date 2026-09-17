# コマンドメモ

このワークスペースで実際に使ったコマンドを置いておく場所。
新しく使ったもの・変えたものは、その都度ここに追記する。

前提: ROS 2 Jazzy / Raspberry Pi 5 / ワークスペースは `~/ros2_ws`。

## ビルド

```bash
# 環境
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 全体
colcon build

# パッケージ指定（普段はこちら）
colcon build --packages-select roboone_behavior
colcon build --packages-select roboone_kinematics
colcon build --packages-select roboone_walk_core
colcon build --packages-select feetech_servo roboone_kinematics
colcon build --packages-select feetech_servo --cmake-args -DCMAKE_BUILD_TYPE=Release
```

ビルド後は `source install/setup.bash` を忘れない。

### パッケージを改名したあとの後始末

**colcon は旧名の `build/` `install/` を消さない。** 残っていると `ros2 run` /
`ros2 launch` が古いほうを拾い、「直したのに変わらない」が起きる。改名を含む
コミットを pull したら、旧名の残骸を消してから建て直す。

```bash
# 2026-09-10 の改名 (roboone_motion_node -> roboone_motion,
#            旧 roboone_motion -> roboone_walk_ref + roboone_viz) の場合
rm -rf build/roboone_motion_node install/roboone_motion_node
rm -rf build/roboone_motion install/roboone_motion
colcon build
source install/setup.bash

# 効いたかの確認 (旧名が消えて新名が出る)
ros2 pkg list | grep roboone
ros2 pkg executables roboone_motion       # motion_node motion_teach motion_selftest
```

### 設定 YAML / ヘッダを変えたあとの反映

**config の YAML は再コンパイル不要、`*_config.hpp` は依存パッケージ全部の再ビルドが要る。**
定数はヘッダの `constexpr` なので、`roboone_kinematics` だけ建て直しても
`roboone_motion` の中に古い値が焼き込まれたまま残る。

```bash
# leg_config.hpp / ankle_config.hpp / knee_config.hpp を変えた
# --packages-above は「そのパッケージ + それに依存する全部」
colcon build --packages-above roboone_kinematics
source install/setup.bash
# 対象: feetech_servo roboone_kinematics roboone_motion roboone_walk_core roboone_viz roboone_bringup
# Pi 5 で約 60 秒（2026-09-15 実測）

# 効いたかの確認（実機不要・バスを開かない）
ros2 run roboone_kinematics leg_selftest     # 最後に「すべて一致」
ros2 run roboone_motion motion_selftest      # 最後に「全部通った」
```

servo_home.yaml / servo_limits.yaml / home_pose.yaml / motions.yaml / gait.yaml は
share に入るデータなので、ビルドの `install` 段だけ通れば足りる。

```bash
colcon build --packages-select feetech_servo        # servo_home / servo_limits
colcon build --packages-select roboone_walk_ref     # home_pose / gait
colcon build --packages-select roboone_motion       # motions
```

**ただし symlink で入っているファイルは編集がそのまま効く**（`--symlink-install` で
建てたパッケージ）。どちらなのかは見れば分かる。

```bash
ls -la install/feetech_servo/share/feetech_servo/config/servo_limits.yaml
# -> src/... へのシンボリックリンクなら編集は即反映。通常ファイルならビルドが要る
```

いずれの場合も**走っているノードには効かない**。`motion_node` は起動時に YAML を
読むだけなので、上げ直す。ツール（`motion_leg_goto` ほか）は起動ごとに読む。

```bash
# 反映されたかを数字で見る（★読むだけ。トルクも位置指令も出さない）
ros2 run roboone_motion motion_leg_goto --leg R --p 0 -89.3 -261 --rpy 0 0 0
#   count 窓の外なら各行に「★servo_limits の窓の外」が付く（これは動かさない条件）
#   関節リミットの外なら「★関節リミット … の外（止めてはいない）」（表示だけ）
```

servo_limits.yaml はここまで全部**ソフト側のクランプ**（`ServoMap` が count を丸める）。
サーボの EEPROM に窓を書くのは別で、こちらは実機操作。

```bash
ros2 run feetech_servo feetech_set_limits --dry-run   # バスは開くが書かない。差分の表示だけ
ros2 run feetech_servo feetech_set_limits             # ★EEPROM に書く。確認プロンプトあり
```

## テスト

```bash
# C++（脚 / 膝 / 足首）
colcon test --packages-select roboone_kinematics
colcon test-result --verbose

# 行動層（ROS 抜きで直接叩ける）
colcon test --packages-select roboone_behavior
colcon test-result --verbose --test-result-base build/roboone_behavior
python3 -m pytest src/roboone_behavior/test/test_behavior.py -q

# Python リファレンス実装
python3 -m pytest scripts/test_knee_fourbar.py -q
python3 -m pytest src/roboone_walk_ref/test/test_walk_core.py
python3 -m pytest src/roboone_walk_ref/test/test_static_walk.py -q   # 静歩行（全時刻 ZMP が支持多角形の中 / 停止 / 歩幅の固定）
python3 -m pytest src/roboone_teleop/test/test_params.py -q   # teleop の調整表と config の整合（ROS 不要）
colcon test --packages-select roboone_teleop                     # 結線テスト（デッドマン / ウォッチドッグ / 2 段トルクオン / 自律の割り込み）

# C++ と Python の突き合わせ
python3 scripts/crosscheck_knee.py
python3 scripts/crosscheck_cpp.py

# Python 単体での自己検算
python3 scripts/knee_fourbar.py
python3 scripts/leg_servo.py
```

## teleop の調整（人の手で値を変える）

手順書は [docs/teleop_tuning.md](teleop_tuning.md)（PDF 同名）。項目の一覧は
`src/roboone_teleop/roboone_teleop/params.py` の表が出どころで、
`config/ps5_dualsense.yaml` はその全項目を並べたもの。

```bash
# 項目の一覧（意味・単位・既定・範囲）と、編集した config の検査
ros2 run roboone_teleop teleop_params
ros2 run roboone_teleop teleop_params --check src/roboone_teleop/config/ps5_dualsense.yaml
ros2 run roboone_teleop teleop_params --check ~/teleop_overrides.yaml
# 割り当てた技名が motions.yaml に本当にあるか（無いものは NG。teleop は起動時に照合しない）
ros2 run roboone_teleop teleop_params --check src/roboone_teleop/config/ps5_dualsense.yaml \
    --motions src/roboone_motion/config/motions.yaml

# 走っているノードの値を読む（★書き換えは受け付けない。YAML を直して上げ直す）
ros2 param list /teleop
ros2 param describe /teleop scale.x
ros2 param get /teleop scale.x

# 自分用の差分 YAML を config の上に重ねて起動する（変えたいキーだけ書く）
ros2 launch roboone_teleop teleop.launch.py overrides:=~/teleop_overrides.yaml
ros2 launch roboone_bringup roboone.launch.py teleop_overrides:=~/teleop_overrides.yaml

# YAML / Python を直しても再ビルド不要にする（初回 1 回。build と install を消してから symlink で入れ直す）
rm -rf build/roboone_teleop install/roboone_teleop
colcon build --packages-select roboone_teleop --symlink-install
source install/setup.bash
```

## 立ち上げ（launch）

```bash
# 手動操縦モード 一式（ui + joy + teleop + motion）
# ★サーボにトルクが入る（allow_torque 既定 true）。起動前に機体を支えておくこと。
#   起動しただけでは動かず、Options 長押し（home → /estop false）で立ち上がる
ros2 launch roboone_bringup roboone.launch.py

# 機体を動かさずに操縦系だけ確かめる（バスは開いて読むが、トルクも位置指令も送らない）
ros2 launch roboone_bringup roboone.launch.py allow_torque:=false

# ★記録は既定で ON（~/roboone_logs/rosbag2_<日時>/ に溜まる）。切りたいときだけ
ros2 launch roboone_bringup roboone.launch.py record:=false

# カメラも上げる（検知の開発時）
ros2 launch roboone_bringup roboone.launch.py camera:=true

# ★既定で RealSense が IMU だけのモード（深度・点群なし）で上がる（imu:=true）。
#   motion ノードの安定化の入力 /camera/imu を出すため。止めたいときだけ
ros2 launch roboone_bringup roboone.launch.py imu:=false

# 自律動作の一式（カメラ + 検出器 + 行動層）
ros2 launch roboone_bringup roboone.launch.py camera:=true detector:=true behavior:=true

# 個別
ros2 launch roboone_behavior behavior.launch.py                    # 行動層だけ
ros2 launch roboone_behavior behavior.launch.py detector:=true camera:=true
ros2 launch roboone_behavior behavior.launch.py techniques:="[]"   # 技を出さない
ros2 launch roboone_teleop teleop.launch.py
ros2 launch roboone_motion motion.launch.py                      # motion だけ
ros2 launch roboone_motion motion.launch.py allow_torque:=false  # 読むだけ
ros2 launch roboone_motion motion.launch.py dry_run:=true        # バスも開かない
ros2 launch realsense_bringup realsense.launch.py
ros2 launch realsense_bringup realsense.launch.py enable_depth:=false   # IMU だけ（安定化用）
ros2 launch feetech_servo feetech_demo.launch.py                 # 動かさない確認用
ros2 launch feetech_servo feetech_demo.launch.py enable_motion:=true   # 実機が動く
```

## motion ノード（歩行 / 技 / IK / サーボ送信）

```bash
# 状態と関節角を見る
ros2 topic echo /motion/state
ros2 topic echo /joint_states --once

# 関節角を deg の表で見る（/joint_states は rad なので読みにくい）
ros2 topic echo --once /joint_states | python3 -c "
import sys,yaml,math
d=next(yaml.safe_load_all(sys.stdin))
for n,p in zip(d['name'],d['position']): print(f'{n:<18}{math.degrees(p):8.2f} deg')
"
#   指令側と並べたいなら /motion/joint_commands に差し替える（並び・名前は同じ）
#   サーボの生カウントが見たいなら /motion/servo_states
#     position=実測カウント / velocity=目標カウント / effort=負荷
watch -n 0.5 'ros2 topic echo --once /joint_states --field position'   # 雑に流し見

# 手で叩く。★/estop は latched (TRANSIENT_LOCAL) なので QoS を合わせないと届かない
ros2 topic pub -t 3 /cmd_motion std_msgs/msg/String "{data: home}"
ros2 topic pub -t 3 --qos-durability transient_local --qos-reliability reliable \
  /estop std_msgs/msg/Bool "{data: false}"          # トルクオン（★実測姿勢から
                                                    #   torque_on_time かけてホーム姿勢へ移る。
                                                    #   サーボ角で補間するので足首は垂れたままでよい）
ros2 topic pub -t 3 --qos-durability transient_local --qos-reliability reliable \
  /estop std_msgs/msg/Bool "{data: true}"           # 脱力
ros2 topic pub -t 3 /cmd_motion std_msgs/msg/String "{data: squat}"   # 動作確認用の技
# その場保持で武装（転倒 → 脱力 のあと、寝た姿勢のままトルクを入れて起き上がりに繋ぐ）。
# **今の姿勢のまま全軸一斉にトルクを入れたい**ならこれ。補間距離ゼロなので機体は動かない
# （servo_bank が「実測位置を読む → それを目標に書く → トルクON」を左右バスで行う）。
# コントローラからは L3 長押し。手で叩くなら hold → estop false の順
ros2 topic pub -t 3 /cmd_motion std_msgs/msg/String "{data: hold}"
# ↑ コントローラからは R1（デッドマン）を押しながら十字キー 下 でも出せる

# パンチ（★機体が動く。腰を回すので足裏が地面をこする。足元を空けておくこと）
ros2 topic pub -t 3 /cmd_motion std_msgs/msg/String "{data: punch_r}"      # 右（腰 +20deg）
ros2 topic pub -t 3 /cmd_motion std_msgs/msg/String "{data: punch_l}"      # 左（腰は回らない）

# 起き上がり（★機体が動く。トルクを入れた状態でしか意味が無いので、実行前に確認を取る。
#   実装中の検証は allow_torque:=false で立ち上げ、/motion/state と /joint_states で見る）
ros2 topic pub -t 3 /cmd_motion std_msgs/msg/String "{data: getup_front}"  # うつ伏せから
ros2 topic pub -t 3 /cmd_motion std_msgs/msg/String "{data: getup_back}"   # 仰向けから
ros2 topic pub -r 20 /cmd_walk geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

起動ログで確認すること:

- `ホーム姿勢 ... 足裏 高さ/前後/半間隔/rpy` が `home_pose.yaml` のとおりか
- `歩行 z_c=...` が `gait.yaml` と一致（ずれていると警告が出る）
- `歩行の足先の箱は ...` — `mech 域には収まる` なら想定内。`届かない` が出たら
  `gait_from_kinematics` で `gait.yaml` を出し直す
- `応答 N/10 軸` が全軸そろっているか（欠けるのは大抵サーボ電源か低電圧）

状態遷移は `RELAX → ARMING → HOLD → WALK / MOTION`。
起動直後は `require_home_before_arm` により、`/cmd_motion` を 1 回受けるまで脱力のまま
（teleop の Options 長押しが `home` → `/estop false` の順に送るので操作は変わらない）。

サーボの加速度（`move_acc`）は 254（明示できる最大。単位 100 step/s²）。2026-09-16 に 50 から
上げた。0 はベンダ資料では「最大」だが、この HLS 系で確かめていないので使わない。

## IMU の安定化（足首戦略・胴体の補正）

motion ノードが `/camera/imu` から胴体のロール・ピッチを推定し、支持脚の足首と股に補正を
足す。中身は `src/roboone_motion/include/roboone_motion/stabilizer.hpp` と `imu_attitude.hpp`。
**コードのゲインの既定は全部 0**（入れても今までと同じ動き）で、実機の値は
`src/roboone_motion/config/motion_node.yaml` の `stab:`。実行中に `ros2 param set` で変えられる。

足裏へ配る方式が 2 つある（`stab.board`。実行中に切り替えられる。既定は板）。

| | 足裏の動き | 遊脚 | 上限 |
|---|---|---|---|
| `stab.board: false`（足首だけ） | 足首 2 軸だけ回す。位置は動かない | 支持足首が傾いた分だけ床へ押し込まれる | `ankle_clamp` 0.12 rad |
| `stab.board: true`（板） | 両足裏を 1 枚の板と見て 2 足の中点まわりに回す。脚が (W/2)·sin u 伸び縮みする | 支持足の足裏と同じ平面に乗ったまま（押し込まない） | `board_clamp` 0.07 rad（約 4 deg） |

板の上限は足首パラレルリンクで決まる。ロール単独なら 6 deg まで届くが、ロールとピッチが
同時に入る隅は 4 deg が上限（2026-09-18 の走査。**歩幅には依らない**）。上限を超える周期は
`PoseCodec` が板を**両脚まとめて**縮めて出すので、指令そのものは止まらない
（縮めた割合は `/motion/stab` の `board_scale`）。

前提: RealSense が上がっていること（`roboone.launch.py` の既定 `imu:=true`、または
`camera:=true`）。来ていなければ起動 10 秒後に警告が出て、補正 0 のまま動く。

```bash
# 1) 立たせて傾きの読みを見る（下の 3) の roll / pitch）。カメラは胴体に水平付けなので、
#    取り付けは既定の [0, 0, 0] のままでよい。立位の後傾（膝のしなり）もそのまま読める
#
#    零点は組み付けのずれを取るときだけ。★胴体が水平だと分かっている状態（水準器を当てる等）で、
#    静止させてから呼ぶ。ホーム姿勢で立たせて呼ぶと、立位の後傾まで 0 と覚えて kp が直さなくなる。
#    25° 以上傾いた読み（寝ている・吊られて傾いている）や、動いているときは断る
ros2 service call /motion/imu_zero std_srvs/srv/Trigger
#    ログに出る「mount_rpy_deg: [...]」を motion_node.yaml の imu: に書き写すと
#    次回の起動から効く（書かないと再起動で [0, 0, 0] に戻る）

# 2) ゲインを入れる。★トルクが入っている機体の足首が動く。支えた状態で、段ごとに押して確かめる
#    （docs/ros2_walk_implementation.pdf §9 の順）
ros2 param set /motion stab.kd_pitch 0.05      # ① ジャイロ減衰（ピッチ）。振動の手前まで上げて 2〜3 割戻す
ros2 param set /motion stab.kd_roll 0.05       #    同（ロール）
ros2 param set /motion stab.kp_pitch 0.3       # ② 傾きの比例。傾いたまま戻らない分が減るか
ros2 param set /motion stab.kp_roll 0.3
ros2 param set /motion stab.k_torso 0.3        # ③ 胴体を起こす（前後のみ）
ros2 param set /motion stab.enable false       # まとめて切る（ゲインは残る）
ros2 param set /motion stab.board false        # 足首だけ回す方式へ戻す（既定は板 = true）
ros2 param set /motion stab.board_clamp 0.05   # 板の上限 [rad]（届かない時刻を減らしたいとき）
#    ★方式を切り替えても跳ばない（使わないほうの補正は rate_limit で 0 へ戻る）
#    ★double は小数点付きで打つ（0 ではなく 0.0）。整数だと型違いで弾かれる
#    範囲外（kd > 0.5 など）も弾かれる。範囲と説明は describe で見る
ros2 param describe /motion stab.kd_pitch
ros2 param dump /motion | grep -A30 "stab:"    # いまの値をまとめて見る

# 3) 中身を見る（100Hz。並びは layout に名前で載っている）
ros2 topic echo --once /motion/stab | python3 -c "
import sys,yaml,math
d=next(yaml.safe_load_all(sys.stdin))
for n,v in zip(d['layout']['dim'][0]['label'].split(','), d['data']):
    print(f'{n:<14}{v:+10.4f}' + (f'  ({math.degrees(v):+.2f} deg)' if n in ('roll','pitch') else ''))
"
```

見るもの（`/motion/stab`。bag にも入る）:

| 項目 | 意味 |
|---|---|
| `roll` / `pitch` | 胴体の傾き [rad]。**roll + = 右へ倒れている / pitch + = 前へ倒れている** |
| `gyro_x` / `gyro_y` | その速さ [rad/s]（Σ_B） |
| `imu_ok` / `imu_age` | IMU が新しいか / 最後のサンプルからの経過（通常 5ms 前後、最大 20ms 程度） |
| `active` / `fade` | 補正を入れる条件が揃っているか（HOLD / WALK・IMU あり・ゲイン非 0）/ 出し入れ |
| `w_R` / `w_L` / `gate` | 脚ごとの効かせ方（遊脚は 0）/ 着地前後でゲインを半分にしているか |
| `u_roll` / `u_pitch` | 足裏を胴体に対して回したい量 [rad] |
| `ank_R_th5` ほか | 足首 θ5（ピッチ）/ θ6（ロール）に実際に足した量 [rad] |
| `torso` | 胴体の前傾に足した量 [rad]（前へ倒れたら負 = 起こす） |
| `ff_ax` / `ff_ay` | IMU に教えた歩行計画の重心加速度（横揺れで傾きの推定がずれないように引く） |
| `corr_dropped` | 補正を入れると IK が解けないので外した脚の数（足首方式） |
| `board_roll` / `board_pitch` | 両足の平面ごと回した量 [rad]（`stab.board: true` のときだけ） |
| `board_scale` | 板を出せた割合。1 = そのまま / 0.75, 0.5, 0.25 = 届かないので縮めた / 0 = 外した |

符号の確かめ方（トルクなしでよい。`allow_torque:=false` で立ち上げ、HOLD まで進めてから
ゲインを入れ、機体を手で傾ける）: 前へ倒す → `pitch` と `u_pitch` が + で、`ank_*_th5` も +
（胴体から見てつま先を下げる向き = 前へ倒れるのを押し戻す）。右へ倒す → `roll` と `u_roll` が +、
`ank_*_th6` が +（足裏の左縁を上げる向き）。向きの根拠は `motion_selftest` の [8]。

板が計画の全時刻で届くかは**起動時**に出る（方式が板のときだけ。バスを開かずに見られる）。

```bash
# 静歩行 / 動歩行それぞれで、板を上限の 8 通り（ロール・ピッチ 1 軸ずつと隅 4 つ）に
# 回して IK と機構層に通す。届かない時刻の数と、どこまでなら通るかが出る
ros2 run roboone_motion motion_node --ros-args -p dry_run:=true -p allow_torque:=false \
  -p walk_mode:=static -p stab.board:=true 2>&1 | grep "板の補正"
#   2026-09-18 の値: 静歩行 ±4 deg で 2 / 62656 点（0.003%）
#                    動歩行 ±4 deg で 1435 / 57200 点（2.5%。計画そのものが到達域の縁）
```

`imu_rx_lag`（受信時刻 − header.stamp）は RealSense が機器の時計を換算した stamp なので
**約 −20ms の定数が乗る**（2026-09-16 実測）。絶対の遅れではない。受信間隔は p50 5.0ms /
p99 5.7ms / 最大 27ms（0.5% が 8ms 超）。

## 胴体の前傾（body_pitch）

前傾角は `src/roboone_walk_ref/config/home_pose.yaml` の **`body_pitch`**（deg・+ が前傾）。
股ピッチ ID1 に `-body_pitch` を足すのと厳密に等価で、**膝・足首の関節角は変わらない**
（足首ピッチ θ5 の余裕を食わない）。`foot.rpy` の pitch とは別の操作で、あちらは足裏の位置を
固定したまま姿勢だけ回すので足首が全部吸収する。前傾させたいだけなら `body_pitch` を使う。

```bash
# 1) 値を変える（yaml だけ。C++ は触らない）
nano src/roboone_walk_ref/config/home_pose.yaml     # body_pitch: 8.0

# 2) config を install へ入れ直す（--symlink-install 済みなら 1) だけで効く）
colcon build --packages-select roboone_walk_ref
source install/setup.bash

# 3) 実機に触らずに到達域を確かめる（バスを開かない・トルクも入らない）
ros2 run roboone_motion motion_node --ros-args -p dry_run:=true     -p allow_torque:=false     -p home_pose_yaml:=$PWD/src/roboone_walk_ref/config/home_pose.yaml
#   起動ログの「胴体を +N deg 前傾させる」と「歩行の足先の箱」の 2 行を見る
#   design / mech なら可。「**届かない**」が出たら入れすぎ（股ピッチの窓に当たる）

# 4) 剛体回転であること（足首を食わないこと）の検算
printf "ikpose R -20 -89.3 -261 0 0 0
ikpose R 16.518818 -89.3 -261.243436 0 -8 0
"   | ./build/roboone_kinematics/leg_service
#   股ピッチだけが -8.000000 deg 動き、膝・足首・クランク余裕は一致する
```

走査結果（height 261 / x -20 / foot.rpy 0、2026-08-29）: **13 deg まで可、14 で届かなくなる**。
`height` / `x` / `foot.rpy` を変えたら取り直すこと。

★ `motion_teach` で捕まえた姿勢は前傾込み（実機そのまま）で出る。`body_pitch` を入れた
まま捕まえた行を `motions.yaml` へ貼ると再生時に二重に傾く。ティーチ中は 0 に戻すこと。

## サーボを 1 軸ずつ見る（対話シェル）

```bash
# 1軸だけを選んで手で叩く対話 CLI（既定 2 ポートを開く）
ros2 run feetech_servo feetech_shell
ros2 run feetech_servo feetech_shell --bus 1 --id 5    # 最初から bus1 の ID5 を選ぶ
echo -e "id 4\nstate\npos" | ros2 run feetech_servo feetech_shell   # パイプで 1 発

# シェル内の「読むだけ」のコマンド（★これらはサーボに書き込まない）
#   buses            バス一覧と接続状態
#   scan [min max]   このバスの ID を総当たり ping（既定 1..20）
#   id N / ping N    対象 ID の選択 / 応答確認
#   pos              現在位置（0-4095 と deg）
#   state            位置/速度/負荷/電圧/温度/電流/moving/エラー
#   info             EEPROM（型番・モード・トルク・角度リミット・目標位置・目標トルク）
#   watch [秒]       位置を連続表示（既定 5 秒、Ctrl-C で抜ける）
#   stats            このバスの tx / rx_fail
#   pos @7           行末に @ID を付けるとその行だけ別 ID を見る
# ★書き込み系（on/off/go/jog/setb/setw/limits）は実機が動く。使う前に確認を取る。

# バスに何が生きているかだけ見たい（シェルを開かずに列挙）
ros2 run feetech_servo feetech_scan_test --id-max 12
ros2 run feetech_servo feetech_scan_test --port /dev/feetech_right --id-max 12
```

前提: `colcon build --packages-select feetech_servo`。バスは udev 固定名
（`/dev/feetech_left` / `/dev/feetech_right`）。ID7 があるほうが右半身。
motion ノードが走っていると同じポートを掴めないので、先に止めること。

## サーボのゲイン / トルク上限

```bash
# 現在値を読む（読むだけ。P/D/I・最大トルク・トルク上限）
ros2 run feetech_servo feetech_gains
ros2 run feetech_servo feetech_gains --ids 4            # 膝だけ

# 目標と実位置の差・負荷・電流を流し見る（★トルクを入れた状態で）
ros2 run feetech_servo feetech_gains --follow --ids 4

# ★EEPROM を書き換える。--write が無ければ「何を書くか」の表示だけ
ros2 run feetech_servo feetech_gains --ids 4 --scale-p 2 --write   # P を 2 倍
ros2 run feetech_servo feetech_gains --ids 4 --set-p 32 --write    # 既定値へ戻す
ros2 run feetech_servo feetech_gains --ids 4 --set-i 4 --write     # I を入れる
```

出荷時は全軸 `P=32 / D=32 / I=0`、最大トルクとトルク上限は 980〜1000（ほぼ上限）。
位置指令に載せる `GOAL_TORQUE` も 1000（最大）なので、**トルクを上げる余地はほぼ無い**。

「浮かせたときと接地したときで姿勢が変わる」のは**荷重に対する定常偏差**で、
サーボは「偏差 × P」ぶんのトルクしか出そうとしないため、上限を上げても偏差は変わらない。
消すには P を上げるか、I（既定 0）を入れて時間積分で押し切る。上げすぎると軸が唸る。

2026-08-28: 膝（R/L ID4）を `P=128`（既定 32 の 4 倍）にしてある。

### 全軸のトルク上限 / 電流上限を一覧する

```bash
# 両バスを総当たりして、トルク上限と電流・保護のしきい値を表で出す
# 読むだけ（write を一切呼ばない）ので、★トルクが切れたまま実行してよい
ros2 run feetech_servo feetech_limits
ros2 run feetech_servo feetech_limits --ids 4,5,6      # ID を絞る
ros2 run feetech_servo feetech_limits --only left      # 片側だけ
ros2 run feetech_servo feetech_limits --csv            # 差分を取りたいとき
```

2026-08-29 の実測（全 19 軸、トルク OFF のまま計測）。左右で値は完全に一致し、
**型番ごとに 2 種類しかない**：

| 型番 | 該当 ID | 最大トルク(16) / トルク上限(48) | 目標トルク(44) | 保護電流(28) | 入力電圧範囲 |
|---|---|---|---|---|---|
| 4618 | 1,2,3,4,7,8,9 | 980 (98%) | 1000 (100%) | 1000 = 6500mA | 8.0-16.0V |
| 5130 | 5,6,10 | 1000 (100%) | 500 (50%) | 500 = 3250mA | 4.0-16.0V |

全軸共通: 過電流時間(38) 200 = 2000ms、保護時間(35) 10 = 100ms、保護トルク(34)
4618 は 30% / 5130 は 50%、温度上限 80℃、過負荷(36) は 255（HLS でこれが % か
不明なので生値のまま出している）。起動最小力(24) は 4618 が 0、5130 が 16。

トルク上限側は既にほぼ上限まで開いており、**上げる余地は無い**（上の節と同じ理由で、
上げても定常偏差は変わらない）。電流側は 5130（足首 ID5/6・腕先 ID10）だけ
しきい値が半分なので、詰まるとしたらここから。

ID7 は右バスにしか無い（左は 9 軸）。

### 生レジスタを全軸まとめて読む / 書く

`feetech_shell` は行末に `@ID` を付けるとその行だけ対象 ID を変えられるので、パイプで流せば
全軸への一括操作になる。`bus 0` = `/dev/feetech_right`、`bus 1` = `/dev/feetech_left`。
軸は右 ID1-10（10 軸）＋ 左 ID1-6, 8-10（9 軸）の計 19 軸（左の ID7 は欠番）。

```bash
# 読むだけ。addr 62 = 電圧(0.1V) / 40 = トルクON / 19 = 脱力する保護条件
{ echo "bus 0"; for i in 1 2 3 4 5 6 7 8 9 10; do echo "getb 19 @$i"; done
  echo "bus 1"; for i in 1 2 3 4 5 6 8 9 10;    do echo "getb 19 @$i"; done
} | ros2 run feetech_servo feetech_shell

# ★EEPROM 書き込み（addr < 40 は setb が unlock → 書き → lock を自動でやる）
{ echo "bus 0"; for i in 1 2 3 4 5 6 7 8 9 10; do echo "setb 19 4 @$i"; done
  echo "bus 1"; for i in 1 2 3 4 5 6 8 9 10;    do echo "setb 19 4 @$i"; done
} | ros2 run feetech_servo feetech_shell
```

前提が 3 つある。

- **バスを他のプロセスが掴んでいないこと。** motion ノードや別の `feetech_shell` が開いて
  いると同じ tty に 2 者が書いて通信が壊れる。`fuser /dev/feetech_right /dev/feetech_left`
  で確かめる（motion ノードは終了時に必ず全軸を脱力してから閉じる）。
- **トルク OFF。** EEPROM 書き込みはトルクが入っていると通らない。`getb 40` が 0 か見る。
- **電源が入っていること。** 電圧が 4618 の下限 8.0V を割ると 4618 系（股・膝・ID7,8,9）は
  1 軸も応答せず、下限 4.0V の 5130（ID5,6,10）だけが返る。**この「半分だけ応答する」状態で
  書くと一部の軸にしか入らない**ので、先に `getb 62` で 120 前後（12.0V）を確認する。

2026-09-15 にこの手順で `UNLOADING_COND(19)` を全 19 軸 4（過熱のみ）にした。中身は
[servo-registers.md](../src/feetech_servo/docs/servo-registers.md#脱力する保護を過熱のみにした2026-09-15)。

## ログを後から追う（沈み込み・追従誤差）

```bash
# 記録は既定で ON
ros2 launch roboone_bringup roboone.launch.py

# 指令と実測の差を軸ごとに集計する
python3 scripts/bag_droop.py ~/roboone_logs/rosbag2_*
python3 scripts/bag_droop.py ~/roboone_logs/rosbag2_* --state HOLD   # 静止区間だけ
python3 scripts/bag_droop.py ~/roboone_logs/rosbag2_* --csv /tmp/d.csv

# 歩行区間ごとのロールの揺れ（/motion/stab から。歩ごとの「遊脚側への傾き」を出す）
python3 scripts/bag_walk_roll.py ~/roboone_logs/rosbag2_*
python3 scripts/bag_walk_roll.py ~/roboone_logs/rosbag2_* --series 19 27   # その区間の時系列
```

★静歩行では「遊脚側への傾き」を鵜呑みにしない。この値は SWING の間しか見ないので、
SHIFT 中に荷重側（外側）へ倒れて、その傾きが次の SWING まで残ると「遊脚側」に数えられる
（2026-09-18 01_47_57 の +22° がこれ。倒れ始めは SHIFT の最初、反対の足の着地）。
`--series` で倒れ始めの walk / φ を確かめること。

`/joint_states`（実測）だけでは沈み込みは分からないので、`/motion/joint_commands`
（指令）と `/motion/servo_states`（サーボ空間の生カウントと負荷）も記録している。
両方を見ることで切り分けられる:

| サーボ空間の差 | 関節空間の差 | 原因 |
|---|---|---|
| あり | あり | サーボが指令位置を保持できていない → P / I ゲイン |
| ほぼ無し | あり | リンク・フレームのたわみ → ゲインでは直らない |

`/motion/diagnostics` にバスごとの電圧・最高温度・応答軸数が入るので、
低電圧で応答が欠けていた区間も後から分かる。

## モーションを作る（ティーチ）

技（攻撃・旋回・起き上がり）は `roboone_motion/config/motions.yaml` に
「時間間隔 + 脚 x2 + ID7-10 の角度」で書く。脚の書き方は 2 通りある。

| 書き方 | キー | 中身 | 再生時 |
|---|---|---|---|
| 足裏 | `R_foot` / `L_foot` | 足裏の位置姿勢 (p, R) | IK を通る。寸法が変わっても追従する |
| 角度 | `R_leg` / `L_leg` | サーボ角（T ポーズ基準 deg・ID 別） | IK も FK も通らずそのまま出る |

角度書きは**IK で戻せない姿勢**（可動域の縁、足首の特異点の近く、寝た姿勢）を
書くための逃げ道。歯止めは `servo_limits.yaml` の窓だけなので、届く姿勢は足裏で
書いておく。値は手で構えて捕まえる。

```bash
# 脱力させて、手で構えた姿勢を YAML のキーフレームとして捕まえる
# ★サーボへの書き込みは起動時のトルク OFF 1 回だけ
ros2 run roboone_motion motion_teach
ros2 run roboone_motion motion_teach --out ~/draft.yaml   # ファイルにも追記
ros2 run roboone_motion motion_teach --t 0.15             # 最初に出す t: の値
ros2 run roboone_motion motion_teach --format angle       # 脚をサーボ角で出す
ros2 run roboone_motion motion_teach --format both        # 両方（角度側は # 付き）
```

| キー | はたらき |
|---|---|
| スペース / `c` | 今の姿勢を `motions.yaml` の書式で出す |
| `+` / `-` | 次に出す `t:`（区間の長さ [s]）を 0.05 ずつ増減 |
| `f` | 出す書式を切り替える（`ik` → `angle` → `both`） |
| `n` | 新しい技の見出し（`<技名>: / keyframes:`）を出す |
| `q` | 終了 |

画面は標準エラー・捕まえた YAML は標準出力に出るので、`> draft.yaml` で溜められる。
捕まえるたびに、`ik` なら「その姿勢を IK で戻せるか」を往復誤差で、`angle` なら
「`servo_limits.yaml` の窓に収まっているか」を確かめ、駄目なら警告する
（順変換で出せても IK で戻せるとは限らない。可動域の縁と足首の特異点の近くが危ない。
そこは `--format angle` で捕まえればそのまま再生できる）。

出た行を `motions.yaml` の `keyframes:` の下に貼り、`t:` を狙いの時間に直せば技になる。
`--format both` で捕まえた場合は、角度で書きたい側の `#` を外して `R_foot` の行を消す。
1 本の技の中で足裏書きと角度書きは混ぜてよい（混ざった区間は角度空間で補間する）。

```bash
# 姿勢まわりの自己検算（実機もサーボも要らない。config を書き換えたら 1 回通す）
#   [1] 角の 3 つの表し方の一巡  [2] body_pitch の等価性  [3] motions.yaml
#   [4] 脚の角度書き            [5] 状態機械の順序        [6] 設定の門
ros2 run roboone_motion motion_selftest
ros2 run roboone_motion motion_selftest --motions ~/draft.yaml   # 下書きを読ませる

# ★gait.yaml / home_pose.yaml を書き換えたら --strict。**実機を起こす前にここで見る。**
#   ホーム姿勢が IK で解けるか / 遊脚が床に届くか / 歩行の足先の箱が到達域に収まるか、
#   を motion ノードの起動時と同じ門で確かめる（以前は実機で立ち上げるしかなかった）。
ros2 run roboone_motion motion_selftest --strict
ros2 run roboone_motion motion_selftest --strict \
    --gait /tmp/g.yaml --home-pose /tmp/h.yaml
```

既定では [6] の門はエラーを**表示するだけで落とさない**（config の調整途中なら当然
エラーが出る。[1]-[5] のコードの赤に気付けなくなるので混ぜない）。`--strict` で落ちる。

技名は `roboone_teleop/config/ps5_dualsense.yaml` の `motion_bindings` と一致させる
（`punch_r` `punch_l` `turn_l` `turn_r` `getup_front` `getup_back`）。
定義していない技名を押しても、motion ノードが「知らない技」と出すだけで何も起きない。

## 可視化

```bash
# 歩行
python3 src/roboone_viz/roboone_viz/gen_walk_viz.py --serve 8100
python3 src/roboone_walk_core/tools/compare_walk_engines.py   # 動歩行と静歩行の両方。--engine static で静歩行だけ、--engine dynamic で動歩行だけ

# 静歩行の設定を変えて見る（static_gait.yaml の上に重ねるだけ。yaml は書き換えない）
# --static-com-offset: 振り出し中の重心の横ずらし [m]（+ で外側） / --static-swing-height: 足上げ [m]
# --static-zmp-tol / --static-t-swing / --static-gait <yaml> もある。
# 届かない組み合わせは画面の足の枠が注意色になる（表は static_gait.yaml の swing_height の注記）
python3 src/roboone_viz/roboone_viz/gen_walk_viz.py --serve 8100 --static-com-offset 0.01 --static-swing-height 0.02
# leg_service による「足先が届くか」の判定を省く（roboone_kinematics 未ビルドなら自動で省く）
python3 src/roboone_viz/roboone_viz/gen_walk_viz.py --serve 8100 --no-reach

# ↑ を手元の PC で見る（どれか 1 つ）
#   VSCode Remote-SSH の端末から: 手元のブラウザで開き、ポート転送も VSCode が張る
"$BROWSER" http://localhost:8100/walk_viz.html
#   同じ LAN なら直接: http://<Pi の IP>:8100/walk_viz.html（Pi の IP は ip -4 addr で見る）
#   ssh のトンネル（手元の PC 側で打つ）: ssh -L 8100:localhost:8100 <pi> のあと http://localhost:8100/walk_viz.html
# ★2026-09-17 までの版は 1 接続ずつしか捌かず、ブラウザの空の先行接続で固まってページが
#   出なかった（ThreadingHTTPServer に直した）。止めた直後に Address already in use が
#   出たら、古い接続が消えるまで数十秒待つ

# ホーム姿勢 (脚ピッチ曲げ角) から z_c と到達域を出し、gait.yaml の目安を印字する
# --map で到達域の ASCII マップ、--bend で曲げ角 [deg]、--t-step で歩周期を変える
./build/roboone_walk_core/gait_from_kinematics --bend 30 --map

# 歩周期 T を変えるときの下敷き。T ごとの e^{ωT} / a_max 上限 / v_max を並べて見る
for T in 0.40 0.30 0.28 0.25; do echo "== T=$T"; \
  install/roboone_walk_core/lib/roboone_walk_core/gait_from_kinematics --t-step $T \
  | sed -n '/歩行パラメータの目安/,$p'; done
```

### 歩周期 (t_step) / 遊脚高さ (swing_height) を変えるときの手順

`t_step` は **4 か所に同じ値がある**（Python が原本、他はその写し）。1 つでも
食い違うと `compare_walk_engines.py` が落ちる。

1. `roboone_walk_ref/roboone_walk_ref/walk_core/params.py`（原本）
2. `roboone_walk_ref/config/gait.yaml`（実機が実際に読む値）
3. `roboone_walk_core/include/roboone_walk_core/gait_params.hpp`（C++ の既定）
4. `roboone_viz/roboone_viz/walkcore.js`（可視化 JS の既定）

T を変えたら**連動して直す値**（放置すると別の壊れ方をする）:

- `td_speed_max` > `(swing_height + td_overdrive) / (0.55 T)`
  下回ると遊脚が床に届かないまま歩の境界を迎える。
- `step_clamp_in` > `(foot_spacing/2) / e^{ωT}`（＝停止準備歩の `b_stop`）
  下回ると準備歩の着地が必ずクランプに当たり、横歩きから止まると公称より
  広い立位で止まる。
- `a_max` は `a·T² (1 + 1/(e^{ωT}∓1)) < クランプ` の 6 割。
  `gait_from_kinematics --t-step <T>` が上限を印字する。
- **`v_max` は T に比例して歩幅になる (`歩幅 = v * T`)。T を伸ばしたら下げる。**
  `gait_from_kinematics` が出す `v_max` は**接地面 (z = -z_c) の到達域だけ**から
  逆算した値で、遊脚頂点での前後到達域を見ていない。頂点は接地面よりずっと狭い
  （50 mm で 前 56 / 後 37 mm、接地面は 前 148 / 後 142 mm）ので、
  T が長いときはツールの値をそのまま入れると足先が到達域を出る。
  例: T=0.60 で `v_max=[0.15,0.08]` にすると頂点の前後が ±49 mm になり後ろが届かない。
- **`t_step` の下限は `swing_height` を出せるサーボ速度で決まる。**
  上昇区間は `0.45 T` しかない。膝サーボは足先 1 mm の持ち上げに約 0.31 deg 動く
  （`gait_from_kinematics --bend N` を振ると出る。曲げ角を変えると足先はほぼ真上に
  動き、前後には 50 mm あたり 1.4 mm しかずれない）。5 次多項式のピークは平均の
  1.875 倍なので

      ピーク角速度 = 1.875 * (0.0054 rad/mm * h_sw[mm]) / (0.45 T)

  無負荷 4.7 rad/s に対し、支持脚の設計則は「半分以下」。遊脚は体重が乗らないので
  8 割程度までは実用範囲。h_sw=50mm なら T=0.30 で 3.73 rad/s (79%)、
  T=0.25 では 4.48 rad/s (95%) になり追従せず、**足が上がりきらずに床を擦る**。

`t_step` を**伸ばす**側の上限は `e^{ωT}` (純 FF の増幅率)。T=0.30 で 6.3、
T=0.60 で 39.6、T=1.00 で 460。静歩行に寄せるほど「重心が支持足の真上に来る」
（単脚支持の中央での重心と支持足の距離 = `(W/2) sech(ωT/2)`、T=0.30 で 61.4 mm、
T=0.60 で 27.7 mm）が、同時に計画と実機のずれが 1 歩で `e^{ωT}` 倍に増える。
**推定 ξ (IMU / 状態推定) を入れるまで T=0.60 より伸ばさないこと。**
なお walk_core は両脚支持期を持たない（支持脚の交代は瞬時）ので、T を伸ばしても
厳密な静歩行にはならない。

`swing_height` を上げると、起動ログの `歩行の足先の箱の隅に **届かない**` が出る
ことがある。これはチェックが `x = ±step_clamp_x` と `z = +swing_height` を
**同時に**満たす隅を見ているためで、実際にはその高さで前後に振れるのは ±20 mm
程度（歩の中央で頂点を通るので）。実行時の警告
（`足先が機構の到達域の外`）が出ていなければ、指令自体は到達域の内側にある。

```bash
# 直したら必ずこの 3 つ
colcon build --packages-select roboone_walk_core roboone_walk_ref roboone_motion
python3 -m pytest src/roboone_walk_ref/test/test_walk_core.py
python3 src/roboone_walk_core/tools/compare_walk_engines.py   # 「照合: 全て一致」
#   両足支持 (ds_time > 0) のケースは C++ とだけ比べる (JS 版は ds_time を持たない)
./build/roboone_walk_core/walk_dump 0.10 0 4.5 10 0.005 ds_time=0.4 | head   # key=value で上書き

# 膝 4 節リンク 3D（デモ / 実機追従）
python3 src/roboone_viz/roboone_viz/serve_knee3d.py --demo
python3 src/roboone_viz/roboone_viz/serve_knee3d.py --side right
python3 src/roboone_viz/roboone_viz/serve_knee3d.py --port /dev/feetech_right --id 4

# 脚 IK 3D（既定 :8101。実機不要）。関節 / 足先 IK / 歩行の 3 モード。
# 膝 4 節リンクと足首パラレルリンクの組み方、サーボ指令角（T ポーズ基準の差分）も出る。
# 先に colcon build --packages-select roboone_kinematics（子プロセスの leg_service を使う）
python3 src/roboone_viz/roboone_viz/serve_leg3d.py
python3 src/roboone_viz/roboone_viz/serve_leg3d.py --port 8101

# leg_service を単体で叩く（1 行 1 リクエスト・1 行 1 JSON。サーボには繋がらない）
printf 'ik R 0 -89.3 -260\nfk L 0 0 0 30 0 0\n' | ./build/roboone_kinematics/leg_service
printf 'mech 0\nik R 20 -89 -250\n' | ./build/roboone_kinematics/leg_service   # 機構層を切る

# 起き上がりのキーフレームを検算する（motions.yaml の p / rpy をそのまま渡す）。
# status ok・clamped 0 で、股ピッチ |hip| <= 90（leg_config.hpp の可動域）・
# 足首ピッチ th5 > -50（エンベロープ -55 に余裕を残す）なら、その姿勢は実機で出せる。
# ★キーフレームの間の補間点も通るので、隣り合う 2 枚を数点に割って同じように見ること。
printf 'ikpose R 0 -89.3 -170 0 10 0\nikpose L 0 89.3 -170 0 10 0\n' \
  | ./build/roboone_kinematics/leg_service \
  | python3 -c 'import sys,json
for l in sys.stdin:
    r=json.loads(l); m=r["mech"]
    print(r["status"], "th5=%.1f"%m["ankle"]["th5"], "bend=%.1f"%m["knee"]["bend"],
          "hip=%.1f"%r["theta"][0], "ankle=%s clamped=%d"%(m["status"],m["ankle"]["clamped"]))'

# 両脚 3D（既定 :8103。実機なしで見るなら --demo、片脚だけなら --only right）
python3 src/roboone_viz/roboone_viz/serve_legs3d.py
python3 src/roboone_viz/roboone_viz/serve_legs3d.py --demo

# 足首パラレルリンク 3D（既定 :8102・左脚）。サーボには書き込まない
python3 src/feetech_servo/viz/serve_ankle_live.py
python3 src/feetech_servo/viz/serve_ankle_live.py --leg R          # 右脚
python3 src/feetech_servo/viz/serve_ankle_live.py --relax          # 手で動かす（トルクを切る）
```

足首ビューアの前提: 先に `colcon build --packages-select feetech_servo` と
`source install/setup.bash`。12V を入れておくこと（電圧が低いと通信が間欠で欠ける）。
バスは `--leg` から udev 固定名（`/dev/feetech_left` / `/dev/feetech_right`）を選ぶ。
`--ids` の既定は `1,2,3,4,6,5` で、**足首は 6, 5 の順**（ID5 が長ロッド側 = 鎖2）。
ここを `5,6` にすると足首の 2 軸が入れ替わる。

PC のブラウザから見るときは有線 LAN で `http://<Pi の IP>:8102/`、
または `ssh -L 8102:localhost:8102 <pi>` して `http://localhost:8102/`。

## 足首パラレルリンク（特異点とリミット）

足首は **θ5 = ピッチ（上側ピボット）/ θ6 = ロール（下側ピボット）**（2026-09-14 に上下の軸を
入れ替えた。それまでの「θ5 = ロール」は実機と逆だった）。つま先を上げすぎると（ピッチ θ5 が
+88 deg の型 2 特異点に近づくと）順変換が発散するので、指令側は θ5 を ±55 deg の
エンベロープに丸め、順変換はロール θ6 を窓 ±40 deg の中だけで解く（外に出たら窓の縁に
張り付く `AnkleFkStatus::Clamped`）。クランクは -42..+60 deg。詳しくは
`roboone_kinematics/include/roboone_kinematics/ankle_parallel.hpp` と `ankle_config.hpp` の冒頭。

```bash
# 特異点・窓・クランクリミットと、servo_limits.yaml に貼る生カウントを印字する。
# --home は servo_home.yaml の home（鎖1 = ID6、鎖2 = ID5 の順）。
ros2 run roboone_kinematics ankle_dump --limits --home 2133 1971            # 左脚
ros2 run roboone_kinematics ankle_dump --limits --home 2033 2193 --right    # 右脚

# ある姿勢の中間量（IK の Δ、順変換の反復数、ヤコビアン）を並べる
ros2 run roboone_kinematics ankle_dump --th5 10 --th6 -30

# 検算（「型 2 特異点と順変換の頑健性」の節が再発防止用。「武装の経路」の節は
# motion の武装（垂れた足首からサーボ角の直線でホーム姿勢へ）が通ることの確認）
ros2 run roboone_kinematics ankle_selftest -n 3000
```

足首の逆変換を**実機で**確かめる（関節角 → サーボ角 → 動かす → 実測を順変換で戻す）。
motion ノードが走っているとポートを掴めないので先に止める。

```bash
# 読むだけ（計算結果と ID6/ID5 の現在値・順変換の θ5/θ6 を出す。何も書かない）
ros2 run feetech_servo feetech_ankle_goto --th5 10 --th6 -5
ros2 run feetech_servo feetech_ankle_goto --leg R --pitch -5 --roll 10   # --pitch→θ5, --roll→θ6

# ★実機が動く（ID6/ID5 にトルクを入れて 2 軸同時に動かす。足を浮かせてから）
ros2 run feetech_servo feetech_ankle_goto --th5 10 --th6 -5 --move
ros2 run feetech_servo feetech_ankle_goto --move --repl      # 対話。1 行 "θ5 θ6" [deg]、q で終了
ros2 run feetech_servo feetech_ankle_goto --th5 0 --th6 0 --move --off   # 原点へ戻してトルクを切る
```

θ5 = ピッチ（上側）/ θ6 = ロール（下側）は `ankle_config.hpp` の約束。届かない・クランク
リミット外の目標は動かさない。終了時トルクは入ったまま（`--off` で切る）。

膝も同じ作法で、曲げ量 [deg]（伸展 0・屈曲 +）を指定して 4 節リンクの逆変換を実機で確かめる。

```bash
# 読むだけ（曲げ量 → ロッカー θ4 → クランク θ2 → サーボ角 → count と、ID4 の現在値）
ros2 run feetech_servo feetech_knee_goto --bend 30
ros2 run feetech_servo feetech_knee_goto --leg R --bend 60

# ★実機が動く。★★膝は機体を支える軸なので、必ず吊るか寝かせてから
ros2 run feetech_servo feetech_knee_goto --bend 30 --move
ros2 run feetech_servo feetech_knee_goto --move --repl        # 対話。1 行に曲げ量、q で終了
ros2 run feetech_servo feetech_knee_goto --bend 0 --move      # 伸び切り（T ポーズ）へ戻す
```

曲げ量 0 が `servo_home.yaml` の ID4 の `home`。目標カウントが `servo_limits.yaml` の窓の外なら
動かさない。`--rocker` / `--crank` でロッカー角・クランク角を直接指定することもできる。

脚 IK 全体は足裏の座標で。motion ノードと同じ変換を通すのでカウントはノードと一致する。

```bash
# 読むだけ（IK → 膝・足首の機構 → サーボ角 → count と、6 軸の現在値 → 順変換の足裏）
ros2 run roboone_motion motion_leg_goto --leg R --p 0 -89.3 -261 --rpy 0 0 0      # Σ_B [mm]・[deg]
ros2 run roboone_motion motion_leg_goto --leg L --rel --p 0 0 -261 --rpy 0 -20 0  # 股中心からの相対

# ★実機が動く。★★脚 6 軸がまとめて動くので、必ず機体を吊るか脚が空中の姿勢で
ros2 run roboone_motion motion_leg_goto --leg R --p 0 -89.3 -261 --move
ros2 run roboone_motion motion_leg_goto --leg L --move --repl    # 対話。1 行 "x y z [roll pitch yaw]"、q で終了
ros2 run roboone_motion motion_leg_goto --leg R --off            # その脚 6 軸のトルクを切る
```

**動かさない条件は 2 つだけ**（2026-09-15 に足首の丸めをやめた）:

- **IK が解けない** … 膝の三角形が閉じない / 足首のロッドが届かない (Δ < 0) / 股中心に近すぎる。
  この場合サーボ角と count の列は `-`（未計算）で出る。IK 自体が解けない目標では
  「最寄り姿勢」を参考値として表示するが、指令には使わない
- **count が `servo_limits.yaml` の窓の外** … 該当行に「★servo_limits の窓の外」が付く

止めなくなったもの（外に出ていると ★ が付くだけで、解けるなら動く）:

- 足首のエンベロープ `TH5_ENVELOPE_DEG` (±55 deg) … 「★足首がエンベロープの外（丸めてはいない）」。
  ここから外は**逆変換は解けるが順変換（実測姿勢）が飛びうる**帯。深く屈むと出る
- 関節リミット `JOINT_LIMIT`（`leg_config.hpp`）… 「★関節リミット … の外（止めてはいない）」
- `ReachLevel`（`mech` / `design`）… 判定として表示するだけ

```bash
# 例: 深屈み。以前はエンベロープで丸められて拒否されていた目標（θ5 = -56.02 deg）
ros2 run roboone_motion motion_leg_goto --leg R --p 10 -89.3 -160 --rpy 0 0 0
#   → IK Ok / 機構 Ok / 判定 mech  ★足首がエンベロープの外（丸めてはいない）
#      足首クランクは +59.4 / -59.0 deg で、CRANK_LIMIT の ±60 まで 0.6 deg しか無い

# 例: ロッドが届かない目標（θ5 ≈ -69 deg）は動かさない
ros2 run roboone_motion motion_leg_goto --leg R --p 10 -89.3 -160 --rpy 0 -12 0
#   → 機構 AnkleUnreachable（足首のロッドが届かない） / → この目標には動かさない
```

motion ノード（200Hz）も同じで、**不可行に入った瞬間から出るまでその周期の指令を送らない**
（前周期の指令が生きるので機体はそこで止まる）。歩行中に起きていないかは
`/motion/diagnostics` の「IK が解けない」「足首がエンベロープの外」で見る。

サーボの角度リミットを EEPROM に書くのは実機操作。**先に --dry-run で確認する。**

```bash
ros2 run feetech_servo feetech_set_limits --dry-run
ros2 run feetech_servo feetech_set_limits          # 書き込み（確認プロンプトあり）
ros2 run feetech_servo feetech_set_limits -c /tmp/limits_一部.yaml --dry-run   # 軸を絞る
```

★`--ids` のような絞り込みは**無い**。`servo_limits.yaml` に載っている軸を全部見て、
EEPROM と食い違う軸を**まとめて**書く。1 軸だけ直したつもりでも、YAML が EEPROM より
新しい軸が他にあれば一緒に書かれる。**必ず --dry-run の「変更する」行を数えてから流す。**
一部だけ書きたいときは、その軸だけ書いた YAML を作って `-c` で渡す（書式は同じ）。

`servo_limits.yaml` の行き先は 2 つあって、反映のしかたが違う:

| 行き先 | 反映 |
|---|---|
| ソフトの窓（`ServoMap` / `pose_codec` / `motion_teach` / `*_goto` が指令を丸める） | ファイルを読み直すだけ = **ツール / ノードの再起動のみ**（install は symlink なのでビルド不要） |
| サーボの EEPROM（addr 9 / 11） | **`feetech_set_limits` を流さないと変わらない** |

`servo_home.yaml` はソフトしか読まないので、こちらは**再起動だけ**で足りる。

この 2 つがずれていると、**ソフトは通すのにサーボが途中で止まる**。指令 count は窓の
内側なので `*_goto` は `★servo_limits の窓の外` を出さず、ファームが目標位置を古い窓へ
丸めるので、その軸だけ目標に届かない（2026-09-15 の実例: 右 ID1 の YAML を
`[818, 3216]` に広げたが EEPROM は `[1100, 3200]` のままで、count 878 の指令が 1100 で
止まった）。EEPROM の現物は 1 軸ずつ読める（★読むだけ・サーボは動かない）:

```bash
printf "id 1\ninfo\n" | ros2 run feetech_servo feetech_shell --port /dev/feetech_right
#   角度リミット: 1100 .. 3200   ← これが EEPROM の現物。YAML と突き合わせる
#   目標位置    : 878            ← 直前に送った指令。窓の外なら動かずここに残る
```

### 腕（ID8/9/10）の可動域を手で探る

```bash
# そのバスの全 ID（腕も含む）の生カウントと T ポーズ基準 deg を 30Hz で出し続ける
ros2 run feetech_servo leg_live_test --scan --leg R          # 書き込み無し
ros2 run feetech_servo leg_live_test --scan --relax --leg R  # ★起動時にトルクを切る
```

名前は「leg」だが `--scan` は `servo_home.yaml` に載っている**そのバスの全 ID** を読むので
腕にも使える。`--relax` は★**そのバスの全軸が脱力する**（機体が落ちる）ので吊るか寝かせてから。
手で可動端まで動かし、突き当たりの生カウントを両端ぶん控えて `servo_limits.yaml` に書く。

### ホーム姿勢の足裏ピッチ（後傾の補正）を決める

機体が後傾するとき `home_pose.yaml` の `foot.rpy` の pitch を負にして胴体を前へ戻す。
入れすぎると遊脚の隅で足首パラレルリンクのロッドが届かなくなるので、**実機に触る前に
到達域を走査して決める**。バスを開かないので★トルクは入らない。

```bash
# 今の config のまま到達域チェックだけ見る（起動ログの「歩行の足先の箱」の行）
ros2 run roboone_motion motion_node --ros-args \
  -p dry_run:=true -p allow_torque:=false

# 候補の yaml を当てて走査する（home_pose と gait は別々に差し替えられる）
ros2 run roboone_motion motion_node --ros-args \
  -p dry_run:=true -p allow_torque:=false \
  -p home_pose_yaml:=/tmp/h261_p-8.yaml -p gait_yaml:=/tmp/gait_261.yaml
```

判定は 3=design 域 / 2=mech 域（歩ける）/「届かない」=不可。**到達域チェックが見る
高さは `gait.yaml` の `z_c` であって `home_pose.yaml` の `height` ではない**ので、
高さを振るときは両方を揃えて差し替えること（片方だけだと表がまったく動かない）。

2026-08-29: 後傾 10 deg に対し `z_c=0.261` のままで入るのは **pitch -8 まで**
（-10 は届かない）。走査結果の表は `home_pose.yaml` の `rpy` のコメントにある。

## 自律動作（behavior）

**behavior は上げただけでは機体を動かさない。** `/autonomy` が true の間しか
`/cmd_walk` を出さず、それを立てるのは PS5 コントローラの長押し（teleop）。
止めるのも teleop の割り込み（起き上がり / 脱力 / ホームポジション / その場保持）。

```bash
# 今どの状態か・なぜそこに入ったか
ros2 topic echo /behavior/state

# 出ている指令
ros2 topic echo /cmd_walk
ros2 topic echo /cmd_motion

# ゲインを走らせたまま変える（tune.* と robot.techniques だけ変えられる）
ros2 param set /behavior tune.k_bearing 2.0
ros2 param set /behavior tune.search_omega 0.4
ros2 param list /behavior
```

`/behavior/debug` は 20Hz の float 配列で、列の意味は
`src/roboone_behavior/roboone_behavior/behavior/types.py` の `DEBUG_ORDER`。

コントローラ抜きで机上で試すとき（**サーボは動かない前提**。motion を上げて
いなければ指令は誰も受け取らない）:

```bash
ros2 topic pub --once /autonomy std_msgs/Bool "data: true"
ros2 topic pub --once /autonomy std_msgs/Bool "data: false"
```

## ログ（rosbag）

```bash
ros2 bag info ~/roboone_logs/rosbag2_YYYY_MM_DD-HH_MM_SS
ros2 bag play ~/roboone_logs/rosbag2_YYYY_MM_DD-HH_MM_SS

# 生カウントが 0-4095 の外へ出ていないかを軸ごとに見る（多回転の巻き数ずれの検出）。
# 足首 ID5/ID6 は servo_limits.yaml が [0, 0] = 多回転可。脱力で手早く動かすと巻き数が
# ±4096 ぶん乗ることがあり、「実測姿勢が取れないので武装しない: ... カウントが 0-4095 の外
# (多回転の巻き数ずれ)」で武装しなくなる（2026-09-18 から理由に軸とカウントが出る）。
# （2026-09-10 実機: R_ID6 -1672 / L_ID6 6103 = ちょうど ∓4096 ずれ）
BAG=~/roboone_logs/rosbag2_YYYY_MM_DD-HH_MM_SS
python3 -c '
import sys, rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="mcap"), rosbag2_py.ConverterOptions("",""))
r.set_filter(rosbag2_py.StorageFilter(topics=["/motion/servo_states"]))
lo, hi = {}, {}
while r.has_next():
    _, d, _ = r.read_next()
    m = deserialize_message(d, JointState)
    for n, p in zip(m.name, m.position):
        lo[n] = min(lo.get(n, p), p); hi[n] = max(hi.get(n, p), p)
for n in lo:
    print("%-6s count %8.0f..%-8.0f%s" % (n, lo[n], hi[n],
          "  <== 0-4095 の外" if lo[n] < 0 or hi[n] > 4095 else ""))
' $BAG
```

## 実機まわり

サーボのバスは udev 固定名（`/dev/feetech_left` / `/dev/feetech_right`）で参照する。
ttyACM の番号は入れ替わる。ID7 がある側が右半身。

```bash
# PS5 コントローラのペアリング
./scripts/ps5_pair.sh

# コントローラの電池残量（接続中のみ読める。切れていると power_supply ごと消える）
cat /sys/class/power_supply/ps-controller-battery-*/capacity   # %
cat /sys/class/power_supply/ps-controller-battery-*/status     # Charging / Discharging

# 繋がっているか
bluetoothctl devices          # ペアリング済みの一覧
bluetoothctl info <MAC>       # Connected: yes/no
```

**接続が切れて数十秒で切れ直す場合はまず電池を疑う。** 接続していられる時間が
だんだん短くなるのが症状（例: 38 秒 → 11 秒）。USB-C で繋いだまま使えば切り分けできる。
コントローラが切れると teleop のウォッチドッグ（`joy_timeout` 0.5s）が `/estop true` を
ラッチするので、機体は脱力して止まる。**復帰は自動ではない** — 繋ぎ直したあと
Options 長押し（`home` → `/estop false`）でもう一度トルクを入れる。

## RealSense

apt ではなくソースビルドしたもの（RSUSB バックエンド）を使う。
上流クローンの再取得:

```bash
git clone -b 4.58.2 https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros
```

```bash
# 実行中のカメラをいじる（ノード名は launch の camera_name 既定で /camera）
ros2 param set /camera enable_color false
ros2 param set /camera pointcloud__neon_.enable true
ros2 param set /camera depth_module.emitter_enabled true
```

### IMU のストリーム確認

`realsense.launch.py` は既定 `enable_imu:=true`。gyro+accel を 200Hz で開き、
`unite_imu_method: 2`（線形補間）で 1 本の `/camera/imu` にまとめて publish する。
IMU を止めたいときだけ `enable_imu:=false`。

```bash
ros2 launch realsense_bringup realsense.launch.py                     # 既定で IMU 込み
ros2 launch realsense_bringup realsense.launch.py enable_imu:=false   # IMU を止める

# 出ているか（/camera/imu が 200Hz なら OK。gyro/accel の生トピックも別に出ている）
ros2 topic hz /camera/imu
ros2 topic echo /camera/imu --once

# 中身の検査（静止させて実行。重力・ジャイロバイアス・ノイズ・共分散の参考値）
ros2 run realsense_bringup rs_imu_test --duration 30

# motion ノードの安定化用に IMU だけ上げる（深度・点群なし）
ros2 launch realsense_bringup realsense.launch.py enable_depth:=false

# 全ストリームの周期・ジッタ・遅延をまとめて見る
ros2 run realsense_bringup rs_stream_test --duration 20
ros2 run realsense_bringup rs_depth_test --frames 60
```

`/camera/imu` が 1 つも流れないときは IMU (HID) のエンドポイントが握られたまま。
`initial_reset:=true`（既定）で直る。README の「ハマりどころ」参照。

### IMU の姿勢を 3D で見る（ロール・ピッチ・ヨーの対応付け確認）

`/camera/imu` を EKF に通して姿勢を推定し、ブラウザに 3D で出す。矢印が機体の
x/y/z 正方向。推定は publish しない（読むだけ）。

```bash
# 1) カメラを上げる（別ターミナル。開けっぱなしにする）
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch realsense_bringup realsense.launch.py

# 2) 姿勢推定 + ビジュアライザ。既定で ssh 元から見える IP に bind する
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 src/realsense_bringup/viz/serve_imu3d.py

# 実機なしで表示だけ確かめる / 別ポート / 別の bind 先
python3 src/realsense_bringup/viz/serve_imu3d.py --demo
python3 src/realsense_bringup/viz/serve_imu3d.py --port 8104 --bind 0.0.0.0
```

起動時に URL を印字するので、手元の PC のブラウザでそれを開く（既定 8104 番）。
確認のしかた: 右を下げる→roll +、お辞儀→pitch +、左へ回す→yaw +。
ヨーは 6 軸 IMU では観測できないのでドリフトする（詳細は
`src/realsense_bringup/README.md`）。

## ドキュメント（md → PDF）

Windows の開発 PC で回す（pandoc と Edge/Chrome が要る）。Pi 上では使わない。
出力は入力と同じ場所に同名 `.pdf`（`docs/` の PDF はそこに置く約束）。

```powershell
powershell -ExecutionPolicy Bypass -File docs/build_md_pdf.ps1 docs/teleop_tuning.md
```

## ROS の無い開発 PC で確かめる（tools/）

Pi に繋げないときの手元検証。**最終判定は Pi の `colcon test`。** 詳細は [tools/README.md](../tools/README.md)。

```bash
# 準備（1 回）
python -m venv tools/.venv
tools/.venv/Scripts/python.exe -m pip install -r tools/requirements-dev.txt   # Windows
tools/.venv/bin/python -m pip install -r tools/requirements-dev.txt           # Mac / Linux

# ament_flake8 / ament_pep257 と同じ規約で lint
python tools/lint_like_ament.py                       # roboone_teleop
python tools/lint_like_ament.py src/roboone_behavior  # 別のパッケージ

# roboone_teleop の結線テストを最小の rclpy 代替 (tools/fakeros) で回す
python tools/run_teleop_tests_without_ros.py
python tools/run_teleop_tests_without_ros.py -k hold  # pytest の引数はそのまま通る

# Pi の上で試したいとき（ROS が source された端末では本物の rclpy を隠さないよう
# 拒否される）。環境変数を落として呼ぶ。最終判定はあくまで colcon test
env -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION -u COLCON_PREFIX_PATH \
    -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_PYTHON_VERSION -u AMENT_PYTHON_EXECUTABLE \
    python3 tools/run_teleop_tests_without_ros.py
```

## 静歩行（static_walk）

重心を支持足の上へ移してから足を振り出す歩き方。設定は
`src/roboone_walk_ref/config/static_gait.yaml`（動歩行の gait.yaml とは別ファイル）。
仕様の原本は `roboone_walk_ref/static_walk/engine.py`、C++ 版は
`roboone_walk_core/static_walk_engine.hpp`、JS 版は `roboone_viz/staticwalk.js`。
motion ノードでは launch の `walk_mode:=static` で使う（既定は `dynamic` = 動歩行。
起動時にしか選べず、コントローラでは切り替えない）。

```bash
# 静歩行で起動する（★トルクが入る。機体を支えてから）
ros2 launch roboone_bringup roboone.launch.py walk_mode:=static
ros2 launch roboone_motion motion.launch.py walk_mode:=static

# 実装の確認はトルクを入れずに（CLAUDE.md）。起動ログの「歩行モード: static」と
# 静歩行の門 3 行（静歩行の立位 / 足先は全時刻で到達域の内側）を見る
ros2 launch roboone_motion motion.launch.py walk_mode:=static allow_torque:=false
ros2 topic echo /motion/state          # "HOLD walk=static"。歩くと "WALK walk=static"

# motion 側の静歩行の検算（[9]。static_gait.yaml の門のエラーは --strict で落とす）
# ★2026-09-18 時点では [6] の動歩行の門が 1 件エラーを出す（立位 ±70mm で内側クランプの
#   隅に届かない）ので、--strict なしで回して [9] の行と最後の「全部通った」を見る
ros2 run roboone_motion motion_selftest
ros2 run roboone_motion motion_selftest --static-gait /tmp/s.yaml --strict

# 実機での確かめ方（操作は teleop の割り当てのまま。コントローラで歩行モードは変えない）
#   Options 長押し  … home -> トルクオン（allow_torque:=false なら入ったことにして進む）
#   R1 + 左スティック … 歩く（倒した量が歩幅 = v x 0.6 s。1 歩の時間は static_gait.yaml 次第で、
#                       2026-09-18 は観察用に時間 3 倍 = 1 歩約 6.4 s。元に戻す値は yaml の冒頭）
#   L1             … 脱力（即時）
# 走らせた bag は ~/roboone_logs/rosbag2_<日時>。静歩行の区間（walk_state 5 / 6）も読める
python3 scripts/bag_walk_roll.py ~/roboone_logs/rosbag2_<日時>
# 設定を変えたら walk_ref を入れ直してから launch し直す（walk_mode と static_gait.yaml は起動時だけ読む）
colcon build --packages-select roboone_walk_ref

# 単体テストと 3 実装の照合（軌道と既定値）
python3 -m pytest src/roboone_walk_ref/test/test_static_walk.py -q
colcon build --packages-select roboone_walk_core
colcon test --packages-select roboone_walk_core && colcon test-result --test-result-base build/roboone_walk_core
python3 src/roboone_walk_core/tools/compare_walk_engines.py --engine static   # 「照合: 全て一致」

# 動歩行の照合。walk_engine.hpp にユーザの足踏み実験（lx = 0）が未コミットで入っている間は、
# その 1 行を戻した一時コピーで walk_dump を作って照合する（ツリーのファイルは触らない）
T=$(mktemp -d) && mkdir -p $T/roboone_walk_core && I=src/roboone_walk_core/include/roboone_walk_core
cp $I/gait_params.hpp $T/roboone_walk_core/
sed -e 's|^    // const double lx = v_\[0\] \* p_.t_step; // 歩行用実装$|    const double lx = v_[0] * p_.t_step;|' \
    -e '/^    const double lx = 0.0; \/\/ 足踏み用$/d' $I/walk_engine.hpp > $T/roboone_walk_core/walk_engine.hpp
g++ -std=c++17 -O2 -I$T src/roboone_walk_core/src/walk_dump.cpp -o $T/walk_dump
python3 -c "import sys; from pathlib import Path; sys.path.insert(0, 'src/roboone_walk_core/tools'); \
import compare_walk_engines as c; real = c._find_exe; \
c._find_exe = lambda n: Path('$T/walk_dump') if n == 'walk_dump' else real(n); \
sys.exit(c.main(['--engine', 'dynamic']))"

# 足先が実機の脚で届くかを設定ごとに走査（static_gait.yaml の swing_height の表を作ったもの）
# 先に colcon build --packages-select roboone_kinematics（leg_service を使う）。-v で最初に届かない点を出す
python3 src/roboone_viz/roboone_viz/static_reach.py \
    --swing-height 0.035,0.03,0.025,0.02 --com-offset 0.01,0.005,0,-0.005,-0.01
python3 src/roboone_viz/roboone_viz/static_reach.py --swing-height 0.03 -v

# 設定を変えたら install へ入れ直す（yaml は symlink ではなく複製）
colcon build --packages-select roboone_walk_ref roboone_viz
```

## 両足支持区間（swing_ratio）

`src/roboone_walk_ref/config/gait.yaml` の **`swing_ratio`**。遊脚が歩周期 T のうち何割を
使うか。1.0 = 従来（φ=1 ちょうどで着く）。下げるとその手前で着地点に達し、残りは
両足が着いたまま止まる。

**ZMP は歩の間ずっと支持足に固定されたまま**なので、ここを変えても DCM も `b` の閉形式も
`a_max` の発散条件も変わらない。増えるのは支持多角形が広い時間だけで、**計画上の重心経路は
変わらない**。yaml だけで済む。install の gait.yaml は symlink ではなく複製なので
（2026-09-17 確認）、`colcon build --packages-select roboone_walk_ref` のあと motion ノードを
上げ直すと効く（下の `-p gait_yaml:=$PWD/src/...` なら src を直接読む）。

```bash
# 起動ログが実測値を出す（狙いの値はそのまま出ない。降下が td_speed_max で飽和するため）
ros2 run roboone_motion motion_node --ros-args -p dry_run:=true \
    -p allow_torque:=false -p gait_yaml:=$PWD/src/roboone_walk_ref/config/gait.yaml
#   遊脚 swing_ratio=0.75 (狙いの両足支持 25.0%) -> 実際は位相 0.817 で接地し 両足支持 18.3% (110 ms)

# 値を振って一覧にする
for sr in 1.00 0.85 0.75 0.66 0.50 0.33; do
  sed "s/^swing_ratio: 1.00/swing_ratio: $sr/" src/roboone_walk_ref/config/gait.yaml > /tmp/g.yaml
  echo -n "swing_ratio=$sr -> "
  timeout 6 ros2 run roboone_motion motion_node --ros-args -p dry_run:=true \
      -p allow_torque:=false -p gait_yaml:=/tmp/g.yaml 2>&1 \
    | grep -oE "両足支持 [0-9.]+% \([0-9]+ ms\)|遊脚が床に届かない" | head -1
done
```

走査（swing_height 50mm / td_overdrive 4mm / td_speed_max 0.20 / T=0.60。2026-08-29 実測）:

| swing_ratio | 1.00 | 0.85 | 0.75 | 0.66 | 0.50 | 0.33 |
|---|---|---|---|---|---|---|
| 狙いの両足支持 | 0% | 15% | 25% | 34% | 50% | 66.7% |
| **実際の両足支持** | 3.3% | 12.5% | 18.3% | 23.3% | 32.5% | 41.7% |
| 接地の位相 | 0.967 | 0.875 | 0.817 | 0.767 | 0.675 | 0.583 |

全域で降下が `td_speed_max` に張り付くので、実際は必ず狙いより小さく出る。
0.50 以下は接地が `swing_lock_phase`(0.70) より早くなり起動時に WARN が出る
（着地点がまだ動いている最中に足が床へ着く）。今の設定で無理なく入るのは **0.75〜0.66**。

起動時のチェックは 3 通り出る:

- INFO … 接地の位相・実際の両足支持・飽和しているか
- WARN … 接地が `swing_lock_phase` より早い
- ERROR … 遊脚が床に届かないまま歩が終わる（空中で支持脚が入れ替わる）。
  必要な `td_speed_max` を数値で出すので、その値以上へ上げるか `swing_height` を下げる

## 両足支持（gait.yaml の ds_time）

`ds_time` を正にすると、動歩行の各歩の頭に両足支持を置き、その間に ZMP を前の支持足から
新しい支持足へ移す（既定 0 = 従来）。骨盤の横の速さが落ち、骨盤が支持足へ寄る。
**足上げと足間隔を一緒に動かさないと遊脚が届かない。** 値の候補と理由は `gait.yaml` の
`ds_time` の注記（例: `ds_time 0.3 / foot_spacing 0.140 / swing_height 0.04`）。

- 起動ログに「両足支持 0.30s + 単脚支持 0.60s = 1 歩 0.90s。歩く速さは指令の 67%」が出る
- 0.4 s を超えると警告が出る（今の `a_max` では計画が発散しうる）
- `/motion/stab` では、両足支持の間は `walk_state` = 2（STEP）で `support` = 0

```bash
# 設定ごとに横振り・発散・足先の到達を走査する（実機不要。leg_service を使う。数分かかる）
# 実機の足の位置は home_pose.yaml の foot.y を読む
colcon build --packages-select roboone_kinematics
python3 src/roboone_viz/roboone_viz/walk_reach.py \
    --ds-time 0,0.2,0.3,0.4 --foot-spacing 0.14,0.15 --swing-height 0.05,0.04,0.03
```

## 歩行の横振り（foot_spacing と home_pose.yaml の foot.y）

横の重心経路（骨盤をどこまで支持足へ寄せるか）は `gait.yaml` の **`foot_spacing`**（計画上の
足間隔）で決まる。**実機の足の位置は `home_pose.yaml` の `foot.y` だけで決まり**、歩行と HOLD の
立位もそこに揃う（motion ノードが計画の立位をホーム姿勢の足へ平行移動する）。計画だけ広げると、
**足の位置はそのままで骨盤の横振りだけが増える**。今は計画 170mm・実機 ±70mm（2026-09-17）。
理由と候補の表は `gait.yaml` の `foot_spacing` のコメント。
（2026-09-18 に `motion_node.yaml` の `stance_y_offset` を廃止した。params に残っていると
起動時に警告が出る。）

- 単脚支持で**遊脚側へ**倒れる（`bag_walk_roll.py` の「遊脚側への傾き」が + で歩ごとに育つ）
  → 横振りが足りない。`foot_spacing` を上げる（`foot.y` は触らない）
- 支持足の**外側へ**倒れる（同じ値が −）→ 振りすぎ。`foot_spacing` を下げる
- `foot.y` を 89.3（股の真下）へ広げると、足上げ 50mm の遊脚が届かなくなる（`home_pose.yaml` の注記）
- どちらのファイルも起動時にしか読まない。両方ビルドして motion を上げ直す

```bash
colcon build --packages-select roboone_walk_ref roboone_motion

# 起動時の検査だけ見る（★バスを開かない。走っている /motion と混ざらないよう
#   ドメインとノード名を分ける）
ROS_DOMAIN_ID=87 timeout -s INT 8 ros2 run roboone_motion motion_node --ros-args \
    -r __node:=motion_check \
    --params-file install/roboone_motion/share/roboone_motion/config/motion_node.yaml \
    -p dry_run:=true -p allow_torque:=false 2>&1 | grep -E "歩行|足先|足間隔|ホーム姿勢"
#   歩行 z_c=0.261m T=0.60s W=0.170m ... が出る
#   「足先の箱の隅に **届かない** (ik止まり: R脚 p=[-40.0, -50.0, -211.0])」の ERROR は
#   足間隔 140mm にした時点から出ている（オフセット 0・W=140 でも同じ点）。箱の隅
#   （後ろ 40・内 20・足上げ 50mm）を同時に取る検査で、実際の歩行軌道では当たらない
```

実際の軌道で届くかは、walk_core（Python 版）を回して足先を `leg_service` に通して見た
（`roboone_viz/reach.py` の `LegReach`）。横 140mm でも x=0 なら足上げ 52mm まで届くが、
足が後ろにあると急に減る（x=−40 で 30mm）。`W=180 / -20` は全速前進で遊脚の頂点が届かない。
