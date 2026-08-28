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

# Python リファレンス実装
python3 -m pytest scripts/test_knee_fourbar.py -q
python3 -m pytest src/roboone_motion/test/test_walk_core.py

# C++ と Python の突き合わせ
python3 scripts/crosscheck_knee.py
python3 scripts/crosscheck_cpp.py

# Python 単体での自己検算
python3 scripts/knee_fourbar.py
python3 scripts/leg_servo.py
```

## 立ち上げ（launch）

```bash
# 機体一式（ui + teleop）
ros2 launch roboone_bringup roboone.launch.py

# 記録つき（練習・試合）
ros2 launch roboone_bringup roboone.launch.py record:=true

# カメラも上げる（検知の開発時）
ros2 launch roboone_bringup roboone.launch.py camera:=true

# 個別
ros2 launch roboone_teleop teleop.launch.py
ros2 launch realsense_bringup realsense.launch.py
ros2 launch feetech_servo feetech_demo.launch.py                 # 動かさない確認用
ros2 launch feetech_servo feetech_demo.launch.py enable_motion:=true   # 実機が動く
```

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
```

## RealSense

apt ではなくソースビルドしたもの（RSUSB バックエンド）を使う。
上流クローンの再取得:

```bash
git clone -b 4.58.2 https://github.com/IntelRealSense/realsense-ros.git src/realsense-ros
```

```bash
# 実行中のカメラをいじる
ros2 param set /camera/camera enable_color false
ros2 param set /camera/camera pointcloud.enable true
ros2 param set /camera/camera depth_module.emitter_enabled true
```
