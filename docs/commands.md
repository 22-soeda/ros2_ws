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
python3 -m pytest src/roboone_motion/test/test_walk_core.py
python3 -m pytest src/roboone_teleop/test/test_params.py -q   # teleop の調整表と config の整合（ROS 不要）
colcon test --packages-select roboone_teleop                     # 結線テスト + 走らせたままの調整

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
# 割り当てた技名が motions.yaml に本当にあるか (無いものは NG。起動時にも警告が出る)
ros2 run roboone_teleop teleop_params --check src/roboone_teleop/config/ps5_dualsense.yaml \
    --motions src/roboone_motion_node/config/motions.yaml

# 走らせたまま試す（ノードを再起動すると消える。決まったら YAML に写す）
ros2 param list /teleop
ros2 param describe /teleop scale.x
ros2 param get /teleop scale.x
ros2 param set /teleop scale.x 0.08
ros2 param dump /teleop                      # いま効いている全値を YAML の形で出す

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

# 自律動作の一式（カメラ + 検出器 + 行動層）
ros2 launch roboone_bringup roboone.launch.py camera:=true detector:=true behavior:=true

# 個別
ros2 launch roboone_behavior behavior.launch.py                    # 行動層だけ
ros2 launch roboone_behavior behavior.launch.py detector:=true camera:=true
ros2 launch roboone_behavior behavior.launch.py techniques:="[]"   # 技を出さない
ros2 launch roboone_teleop teleop.launch.py
ros2 launch roboone_motion_node motion.launch.py                      # motion だけ
ros2 launch roboone_motion_node motion.launch.py allow_torque:=false  # 読むだけ
ros2 launch roboone_motion_node motion.launch.py dry_run:=true        # バスも開かない
ros2 launch realsense_bringup realsense.launch.py
ros2 launch feetech_servo feetech_demo.launch.py                 # 動かさない確認用
ros2 launch feetech_servo feetech_demo.launch.py enable_motion:=true   # 実機が動く
```

## motion ノード（歩行 / 技 / IK / サーボ送信）

```bash
# 状態と関節角を見る
ros2 topic echo /motion/state
ros2 topic echo /joint_states --once

# 手で叩く。★/estop は latched (TRANSIENT_LOCAL) なので QoS を合わせないと届かない
ros2 topic pub -t 3 /cmd_motion std_msgs/msg/String "{data: home}"
ros2 topic pub -t 3 --qos-durability transient_local --qos-reliability reliable \
  /estop std_msgs/msg/Bool "{data: false}"          # トルクオン
ros2 topic pub -t 3 --qos-durability transient_local --qos-reliability reliable \
  /estop std_msgs/msg/Bool "{data: true}"           # 脱力
ros2 topic pub -t 3 /cmd_motion std_msgs/msg/String "{data: squat}"   # 動作確認用の技
# その場保持で武装（転倒 → 脱力 のあと、寝た姿勢のままトルクを入れて起き上がりに繋ぐ）。
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

## ログを後から追う（沈み込み・追従誤差）

```bash
# 記録は既定で ON
ros2 launch roboone_bringup roboone.launch.py

# 指令と実測の差を軸ごとに集計する
python3 scripts/bag_droop.py ~/roboone_logs/rosbag2_*
python3 scripts/bag_droop.py ~/roboone_logs/rosbag2_* --state HOLD   # 静止区間だけ
python3 scripts/bag_droop.py ~/roboone_logs/rosbag2_* --csv /tmp/d.csv
```

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

技（攻撃・旋回・起き上がり）は `roboone_motion_node/config/motions.yaml` に
「時間間隔 + 足裏の位置姿勢 (p, R) x2 + ID7-10 の角度」で書く。値は手で構えて捕まえる。

```bash
# 脱力させて、手で構えた姿勢を YAML のキーフレームとして捕まえる
# ★サーボへの書き込みは起動時のトルク OFF 1 回だけ
ros2 run roboone_motion_node motion_teach
ros2 run roboone_motion_node motion_teach --out ~/draft.yaml   # ファイルにも追記
ros2 run roboone_motion_node motion_teach --t 0.15             # 最初に出す t: の値
```

| キー | はたらき |
|---|---|
| スペース / `c` | 今の姿勢を `motions.yaml` の書式で出す |
| `+` / `-` | 次に出す `t:`（区間の長さ [s]）を 0.05 ずつ増減 |
| `n` | 新しい技の見出し（`<技名>: / keyframes:`）を出す |
| `q` | 終了 |

画面は標準エラー・捕まえた YAML は標準出力に出るので、`> draft.yaml` で溜められる。
捕まえるたびに「その姿勢を IK で戻せるか」を往復誤差で確かめ、駄目なら警告する
（順変換で出せても IK で戻せるとは限らない。可動域の縁と足首の特異点の近くが危ない）。

出た行を `motions.yaml` の `keyframes:` の下に貼り、`t:` を狙いの時間に直せば技になる。
技名は `roboone_teleop/config/ps5_dualsense.yaml` の `motion_bindings` と一致させる
（`punch_r` `punch_l` `turn_l` `turn_r` `getup_front` `getup_back`）。
定義していない技名を押しても、motion ノードが「知らない技」と出すだけで何も起きない。

## 可視化

```bash
# 歩行
python3 src/roboone_motion/roboone_motion/viz/gen_walk_viz.py --serve 8100
python3 src/roboone_walk_core/tools/compare_walk_engines.py

# ホーム姿勢 (脚ピッチ曲げ角) から z_c と到達域を出し、gait.yaml の目安を印字する
# --map で到達域の ASCII マップ、--bend で曲げ角 [deg]、--t-step で歩周期を変える
./build/roboone_walk_core/gait_from_kinematics --bend 30 --map

# 膝 4 節リンク 3D（デモ / 実機追従）
python3 src/roboone_motion/roboone_motion/viz/serve_knee3d.py --demo
python3 src/roboone_motion/roboone_motion/viz/serve_knee3d.py --side right
python3 src/roboone_motion/roboone_motion/viz/serve_knee3d.py --port /dev/feetech_right --id 4

# 脚 IK 3D（既定 :8101。実機不要）。関節 / 足先 IK / 歩行の 3 モード。
# 膝 4 節リンクと足首パラレルリンクの組み方、サーボ指令角（T ポーズ基準の差分）も出る。
# 先に colcon build --packages-select roboone_kinematics（子プロセスの leg_service を使う）
python3 src/roboone_motion/roboone_motion/viz/serve_leg3d.py
python3 src/roboone_motion/roboone_motion/viz/serve_leg3d.py --port 8101

# leg_service を単体で叩く（1 行 1 リクエスト・1 行 1 JSON。サーボには繋がらない）
printf 'ik R 0 -89.3 -260\nfk L 0 0 0 30 0 0\n' | ./build/roboone_kinematics/leg_service
printf 'mech 0\nik R 20 -89 -250\n' | ./build/roboone_kinematics/leg_service   # 機構層を切る

# 起き上がりのキーフレームを検算する（motions.yaml の p / rpy をそのまま渡す）。
# status ok・clamped 0 で、股ピッチ |hip| <= 60（leg_config.hpp の可動域）・
# 足首 th6 > -50（窓 -55 に余裕を残す）なら、その姿勢は実機で出せる。
# ★キーフレームの間の補間点も通るので、隣り合う 2 枚を数点に割って同じように見ること。
printf 'ikpose R 0 -89.3 -170 0 10 0\nikpose L 0 89.3 -170 0 10 0\n' \
  | ./build/roboone_kinematics/leg_service \
  | python3 -c 'import sys,json
for l in sys.stdin:
    r=json.loads(l); m=r["mech"]
    print(r["status"], "th6=%.1f"%m["ankle"]["th6"], "bend=%.1f"%m["knee"]["bend"],
          "hip=%.1f"%r["theta"][0], "ankle=%s clamped=%d"%(m["status"],m["ankle"]["clamped"]))'

# 両脚 3D（既定 :8103。実機なしで見るなら --demo、片脚だけなら --only right）
python3 src/roboone_motion/roboone_motion/viz/serve_legs3d.py
python3 src/roboone_motion/roboone_motion/viz/serve_legs3d.py --demo

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

足裏を前後に傾けすぎると（ピッチ θ6 が -65.6 deg の型 2 特異点に近づくと）順変換が
発散していた。順変換は窓 ±55 deg の中だけを解くようにしてあり、その外に出たら窓の
縁に張り付く（`AnkleFkStatus::Clamped`）。詳しくは
`roboone_kinematics/include/roboone_kinematics/ankle_parallel.hpp` の冒頭。

```bash
# 特異点・窓・クランクリミットと、servo_limits.yaml に貼る生カウントを印字する。
# --home は servo_home.yaml の home（鎖1 = ID6、鎖2 = ID5 の順）。
ros2 run roboone_kinematics ankle_dump --limits --home 2133 1971            # 左脚
ros2 run roboone_kinematics ankle_dump --limits --home 2033 2193 --right    # 右脚

# ある姿勢の中間量（IK の Δ、順変換の反復数、ヤコビアン）を並べる
ros2 run roboone_kinematics ankle_dump --th5 10 --th6 -30

# 検算（「型 2 特異点と順変換の頑健性」の節が再発防止用）
ros2 run roboone_kinematics ankle_selftest -n 3000
```

サーボの角度リミットを EEPROM に書くのは実機操作。**先に --dry-run で確認する。**

```bash
ros2 run feetech_servo feetech_set_limits --dry-run
ros2 run feetech_servo feetech_set_limits          # 書き込み（確認プロンプトあり）
```

## 自律動作（behavior）

**behavior は上げただけでは機体を動かさない。** `/autonomy` が true の間しか
`/cmd_walk` を出さず、それを立てるのは PS5 コントローラの長押し（teleop）。
止めるのも teleop の割り込み（起き上がり / 脱力 / 無線確認 / ホームポジション）。

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
```
