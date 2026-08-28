# realsense_bringup

RealSense D435if を **depth + IMU** 構成で起動するための launch とパラメータ、
および実機ストリームを検証する CLI テストツール。

ストリーミング本体は既製の `realsense2_camera_node`（`src/realsense-ros/`）を使う。
このパッケージが持つのは「この機体でどう起動するか」の一点だけ。
`docs/ros-architecture.md` の方針（realsense2_camera は既製パッケージ）に従っている。

## 起動

```bash
# 既定: depth 848x480x30 + IMU 200Hz + XYZ点群。RGB は OFF
ros2 launch realsense_bringup realsense.launch.py

# RGB も出す（点群に色が付く）
ros2 launch realsense_bringup realsense.launch.py enable_color:=true

# 点群が要らない（depth 画像だけ / CPU を空けたい）
ros2 launch realsense_bringup realsense.launch.py enable_pointcloud:=false
```

出るトピック。名前は `docs/ros-architecture.md` のトピック表に合わせてある。

| トピック | 型 | 周期 | 受け側 |
|---|---|---|---|
| `/camera/imu` | sensor_msgs/Imu | 200Hz | imu_filter |
| `/camera/depth/color/points` | sensor_msgs/PointCloud2 | 30Hz | —（rviz 用）|
| `/camera/depth/image_rect_raw` | sensor_msgs/Image (16UC1) | 30Hz | opponent_detector |
| `/camera/color/image_raw` | sensor_msgs/Image (rgb8) | 30Hz | `enable_color:=true` のときだけ |

主な launch 引数（全部は `ros2 launch realsense_bringup realsense.launch.py --show-args`）:

| 引数 | 既定 | 意味 |
|---|---|---|
| `enable_color` | `false` | RGB を出すか |
| `enable_imu` | `true` | gyro+accel を出し `/camera/imu` を作るか |
| `enable_pointcloud` | `true` | 点群を出すか |
| `initial_reset` | `true` | 起動時にカメラをハードリセット（下の「ハマりどころ」参照） |
| `pointcloud_ns` | `pointcloud__neon_` | 点群フィルタのパラメータ接頭辞（同上） |

解像度・IMU レート・フィルタは `config/realsense.yaml`。

## 既定構成の根拠

- **depth 848x480x30** — この Pi の RSUSB バックエンドで実測 30.0Hz 安定。
  90/60fps も選べるが、60fps 以上は RGB と同時だと USB 帯域がぎりぎりになる。
- **IMU 200Hz**（gyro 200Hz + accel 200Hz、`unite_imu_method=2`）— `docs/ros-architecture.md`
  の `/camera/imu` 目安周期に合わせた。accel を gyro の時刻へ線形補間して 1 本にまとめるので、
  imu_filter 側は gyro/accel を自分で同期しなくてよい。
- **RGB OFF** — opponent_detector は depth しか使わない。ON にすると depth の遅延が
  約 18ms → 約 57ms に増える（下の実測表）。必要になったら `enable_color:=true`。
- **点群は XYZ のみ** — RGB OFF のときは自動でテクスチャなしになる（`stream_filter=0`）。
  `enable_color:=true` にすると自動で色付き（`stream_filter=2`, `point_step` 16→20）になる。
- **後処理フィルタは全部 OFF** — Pi5 の CPU は motion ノードの 200Hz ループに回す。

## 実測（2026-08-27, D435if S/N 327122072324, FW 5.15.0.2, USB3 5Gbps）

`rs_stream_test --duration 20` の結果。CPU は `realsense2_camera_node` 1プロセスの
1コア比（Pi5 は4コアなので全体では 10〜13%）。

| 構成 | depth | imu | points | color | ノードCPU |
|---|---|---|---|---|---|
| 既定 (RGB OFF) | 30.02Hz / jitter 2.4ms / 遅延 17.6ms | 200.03Hz / jitter 0.67ms / 遅延 0.5ms | 30.02Hz / 平均 43,624点 | — | 40% |
| `enable_color:=true` | 30.05Hz / jitter 3.3ms / 遅延 57.5ms | 199.69Hz / jitter 1.2ms / 遅延 0.8ms | 30.00Hz / 平均 23,318点 | 30.05Hz 1280x720 | 50% |

**RGB を足しても全ストリームが目標レートを維持する。** 帯域は足りている。
代償は depth/点群の遅延が約 40ms 増えること（色付き点群のために depth と color を
同じフレームセットへ揃える必要があり、その待ち合わせぶん）と、CPU が 10 ポイント増えること。
点数が減るのは、色が取れない画素を点群から落としているため（`allow_no_texture_points: false`）。

IMU（静止・`rs_imu_test --duration 15`）:

```
実測 200.19Hz / タイムスタンプ逆行 0 / NaN 0
|a| 平均 9.851 m/s^2（重力 9.807 との差 +0.044）
ジャイロのバイアス 最大 0.0048 rad/s
ノイズ（標準偏差）accel 最大 0.0355 m/s^2 / gyro 最大 0.0029 rad/s
```

`config/realsense.yaml` の共分散は暫定で `0.01` のまま置いてある。上の実測から出る値
（accel 1.3e-3 / gyro 8e-6）はあくまで**静止時の白色ノイズ**で、歩行中の振動や
バイアスドリフトを含まない。imu_filter のチューニングを始めるときに、この値を
下限の目安として使うこと。

## テストツール

どれも実機が要るので ctest には載せていない。手で走らせる CLI で、
**判定が通れば終了コード 0、外れれば 1**（起動確認のゲートに使える）。
`feetech_servo/test/` と同じ方針。

### `rs_stream_test` — まずこれ

全トピックを一定時間購読して、周期・ジッタ・最大間隔・遅延・中身の妥当性を1つの表にする。

```bash
ros2 run realsense_bringup rs_stream_test                    # 10秒
ros2 run realsense_bringup rs_stream_test --duration 30 --tolerance 5
```

- 出ていないストリーム（RGB OFF のときの color など）は自動でスキップする。
  depth と imu だけは必須で、無ければ NG。
- publisher の QoS に合わせて購読する。IMU は SENSOR_DATA(BEST_EFFORT)、画像は
  RELIABLE なので、決め打ちにすると IMU だけ受け取れない。
- `!` は NG の原因、`?` は知っておくべきだが合否に効かないもの。

### `rs_imu_test` — IMU の中身

静止状態で走らせて、重力の大きさ・向き、ジャイロのバイアス、ノイズを測る。
imu_filter へ渡す共分散の参考値も出す。

```bash
ros2 run realsense_bringup rs_imu_test --duration 30
```

重力がどの軸に乗っているかで取り付け姿勢が分かる。IMU 光学フレームは
**X=右 / Y=下 / Z=前** なので、正立・水平なら `Y(下)` に約 -9.8 が出る。

### `rs_depth_test` — depth の中身

`rs_stream_test` は「30Hz で届いているか」までしか見ない。全画素 0 の空っぽな画像でも
30Hz は出るので、距離が本当に取れているかは別に確かめる。

```bash
ros2 run realsense_bringup rs_depth_test --frames 60
```

画素を3つに分けて数える。ひとまとめに「有効/無効」で見ると、カメラが壊れているのか
ただ何も無い方を向いているだけなのかが区別できないため。

- **測距できた** — 実際に距離が出た画素
- **測距失敗 (0)** — テクスチャ無し・遮蔽・近すぎ（0.2m 未満）
- **レンジ外** — `--max-range`（既定 10m）より遠い。視差がほぼ 0 の画素で、
  数十mという値が出るがこれは距離ではなく「遠い」という意味しかない

さらに画面を 16x9 に割った距離マップを出すので、数字が見た目と合っているかで判断できる。
**0.5〜3m 先に物がある向き**（リングに見立てた状態）で走らせるのが本来の使い方。
既定の合格ライン（測距できた画素 20%）は、机に伏せた状態などでは当然落ちる。

## ハマりどころ

この Pi 固有の話は `docs/` 側ではなくここにまとめてある。

### 点群のパラメータ名が `pointcloud.*` ではない

realsense2_camera はフィルタのパラメータ名を librealsense のフィルタ名から機械的に作る。
librealsense は点群フィルタに SIMD バックエンド名を付けるので、この Pi (aarch64) では
`Pointcloud (NEON)` → **`pointcloud__neon_`** になり、公式ドキュメントや `rs_launch.py` が
使う `pointcloud` では効かない。

刺さり方が意地悪で、起動ログに `Failed to get parameters: pointcloud.enable` と1行出るだけで、
**点群トピックが黙って作られない**。実際の名前は `ros2 param list /camera | grep point` で確認できる。
launch 引数 `pointcloud_ns` で変えられる。

### IMU が何も言わずに止まる → `initial_reset`

ノードを Ctrl-C 以外で落とすと、次の起動で IMU (HID) のエンドポイントが握られたままになり、
ログには `Starting Sensor: Motion Module` / `Open profile: Accel, Gyro` まで正常に出るのに
`/camera/imu` が1つも流れない状態に落ちることがある。エラーは何も出ない。

`initial_reset:=true`（**既定で有効**）でカメラをハードリセットすれば確実に直る。
起動が数秒延びるだけなので、既定で入れてある。`rs_stream_test` を走らせれば
「imu … NG 受信なし」で即座に分かる。

### 点群の `row_step` が data サイズと合わない

realsense-ros の点群フィルタは、無効画素を捨てて `width` を縮めた後に `row_step` を
更新し忘れる（上流の実装の癖）。例: `width=35816, point_step=16, data=573,056 bytes` なのに
`row_step=6,512,640`（縮める前の 848x480 ぶん）。

`PointCloud2Iterator` も `pcl_conversions` も `width`/`height`/`point_step` しか見ないので
実害はないが、**`row_step` を信じてポインタを進める自作コードを書くとバッファ外を読む**。
opponent_detector を書くときは点数を `width * height` から取ること。
`rs_stream_test` はこれを `?`（警告）として毎回表示する。

### 無視してよい警告

- `control_transfer returned error ... Resource temporarily unavailable` — RSUSB バックエンドの
  センサ初期化時に毎回1〜2回出る。ストリーミングには影響しない。
- `For the 'unite_imu_method' param update to take effect, re-enable either gyro or accel stream.` —
  起動時のパラメータ適用順で出るだけ。実際には効いている（`/camera/imu` が 200Hz で出ていれば正常）。
- `IMU Calibration is not available, default intrinsic and extrinsic will be used.` — この個体に
  IMU の工場校正が書かれていないという意味。`rs-imu-calibration.py` で焼けるが、
  imu_filter を入れる前提なら必須ではない。

## 前提

librealsense2 は **RSUSB バックエンドでソースビルドしたもの**（`/usr/local`）でなければならない。
この Pi の kernel には Intel の UVC メタデータパッチが当てられないため。
`ros-jazzy-librealsense2` / `ros-jazzy-realsense2-camera` を apt で入れてはいけない。
