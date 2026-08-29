# tools — ROS の無い開発 PC で確かめるための道具

Pi に繋げないときに、手元 (Windows / Mac / Linux、ROS 無し) で Python パッケージの
整合性を見るためのもの。**ここで通っても最終判定は Pi の `colcon test`。**

| ファイル | 役割 |
|---|---|
| `lint_like_ament.py` | `ament_flake8` / `ament_pep257` と同じ規約で flake8 と pydocstyle を回す |
| `run_teleop_tests_without_ros.py` | `fakeros` を使って `roboone_teleop` のテスト (結線テスト一式 + 表と config の整合) を回す |
| `fakeros/` | 最小の rclpy 代替 (下) |
| `requirements-dev.txt` | venv に入れるもの |
| `.venv/` | 上のための venv (git には入れない) |

## 準備 (1 回)

```bash
python -m venv tools/.venv
tools/.venv/Scripts/python.exe -m pip install -r tools/requirements-dev.txt     # Windows
tools/.venv/bin/python -m pip install -r tools/requirements-dev.txt             # Mac / Linux
```

## 使う

```bash
python tools/lint_like_ament.py                       # roboone_teleop を lint
python tools/lint_like_ament.py src/roboone_behavior  # 別のパッケージ
python tools/run_teleop_tests_without_ros.py          # 結線テスト (fakeros)
python tools/run_teleop_tests_without_ros.py -k param # pytest の引数はそのまま通る
```

`ROS_DISTRO` が立っている端末 (Pi) では `run_teleop_tests_without_ros.py` は動かない
(本物の rclpy を隠してしまうため)。Pi では `colcon test` を使う。

## fakeros が「まねているもの」と「まねていないもの」

まねている:

* プロセス内の pub/sub (トピック名で結ぶ。型は見ない)
* QoS の depth と、`TRANSIENT_LOCAL` の latched 配信 (後から来た購読者に最後の 1 件を渡す)。
  `VOLATILE` の publisher と `TRANSIENT_LOCAL` の subscription がマッチしない性質も同じ
* タイマー (`create_timer` / `cancel`) と `SingleThreadedExecutor.spin_once`
* パラメータ: `declare_parameter` (overrides・`dynamic_typing`・型の不一致で例外)、
  `get_parameter`、`set_parameters` (1 個ずつ。コールバックは適用前、失敗したら入らない)、
  `add_on_set_parameters_callback` (新しいものが先)、`_parameter_overrides`
* `get_logger()` (stderr に出す)、`get_clock().now().nanoseconds` (壁時計)
* メッセージ型: `std_msgs` Bool/String、`geometry_msgs` Twist、`sensor_msgs` Joy、
  `roboone_interfaces` OledText/LedColor、`rcl_interfaces` ParameterDescriptor/SetParametersResult
* `ament_index_python.packages.get_package_share_directory` (share の代わりに `src/<pkg>` を返す)

まねていない:

* DDS (プロセス間通信・discovery・遅延・取りこぼし)。同一プロセスの 2 ノード間だけ
* メッセージのフィールド型の検査 (本物は `axes` に int を入れると落ちる)
* サービス・アクション・コールバックグループ・マルチスレッド executor
* パラメータサービス (`ros2 param set` の経路)。`node.set_parameters()` を直接呼ぶ
* 時刻の `use_sim_time`

新しいパッケージのテストをここで回したくなったら、使うメッセージ型を `fakeros/<pkg>/msg/`
に足す (`src/<pkg>/msg/*.msg` の定義と揃えること)。
