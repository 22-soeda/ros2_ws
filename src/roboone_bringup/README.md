# roboone_bringup

機体をひとまとめに立ち上げる launch。各パッケージの launch はそのまま残してあり、
このパッケージは「どれを、どの順で、どの引数で上げるか」だけを持つ。

```bash
ros2 launch roboone_bringup roboone.launch.py
```

これで ui・teleop（joy 込み）・motion が上がり、`ros2 bag` の記録も回る。止めるのは Ctrl-C。

★ **既定でサーボにトルクが入る**（`allow_torque:=true`）。起動前に機体を支えておくこと。
ただし起動しただけでは動かず、`/cmd_motion` を 1 回受けるまで脱力で待つ
（teleop の Options 長押しが `home` → `/estop false` の順に送る）。

## 引数

| 引数 | 既定 | 意味 |
|---|---|---|
| `ui` | `true` | ui ノード（OLED / RGB LED / ブザー） |
| `teleop` | `true` | joy + teleop ノード |
| `motion` | `true` | motion ノード（歩行 / 技 / IK / サーボ送信） |
| `allow_torque` | `true` | ★サーボにトルクを入れる。`false` で「読むだけ」の通し確認（歩行計画・IK・`/joint_states` は全部回る） |
| `camera` | `false` | RealSense。USB 帯域と CPU を食うので、要るときだけ |
| `detector` | `false` | opponent_detector（敵機検出）。`camera:=true` と対で使う |
| `behavior` | `false` | behavior（自律の行動判断）。`detector:=true` と対で使う。上げただけでは動かず、`/autonomy` を teleop の長押しで立てたときだけ指令を出す |
| `record` | `true` | `ros2 bag` 記録（`config/record_topics.yaml` のトピック）。切るなら `record:=false` |
| `log_dir` | `~/roboone_logs` | 記録の置き場 |
| `joy_backend` | `game_controller` | teleop へそのまま渡す |
| `teleop_config` | `roboone_teleop/config/ps5_dualsense.yaml` | 割り当て YAML |
| `teleop_overrides` | （なし） | teleop の差分 YAML。`teleop_config` の上に重ねる（`docs/teleop_tuning.md` §3.3） |

よく使う組み合わせ:

```bash
# 練習・試合（既定。記録あり・トルクあり）
ros2 launch roboone_bringup roboone.launch.py

# 機体を動かさずに操縦系だけ確かめる
ros2 launch roboone_bringup roboone.launch.py allow_torque:=false

# 検知の開発（カメラも上げる）
ros2 launch roboone_bringup roboone.launch.py camera:=true

# 自律動作の一式
ros2 launch roboone_bringup roboone.launch.py camera:=true detector:=true behavior:=true

# 表示だけ確認したい
ros2 launch roboone_bringup roboone.launch.py teleop:=false motion:=false

# 自分用の teleop 調整値で
ros2 launch roboone_bringup roboone.launch.py teleop_overrides:=~/teleop_overrides.yaml
```

## 起動順と落ち方

* **ui を先に上げる。** teleop の状態表示は latched なので順序に厳密な依存は無いが、
  ui が先にいれば起動直後から「まだ `/joy` が来ていない」が画面に出る。立ち上げ中に
  何も表示されない時間を作らない。
* **joy_node は respawn する。** Bluetooth は切れるものなので、落ちたら上げ直す。
  切れている間は teleop の無通信ウォッチドッグが脱力を掛けるので、勝手に上げ直しても
  安全側は壊れない。
* **motion は teleop より後で構わない。** `/estop` は latched なので、後から上げても直前の
  脱力状態が届く。
* **どのノードが落ちても launch 全体は落とさない。** 特に teleop が死んだときに全部
  道連れにすると、記録が切れて原因が追えなくなり、OLED も消えて操作者が状況を読めなく
  なる。teleop は終了時にゼロ指令と `/estop true` を置いていくので、teleop だけが
  死んでも機体は脱力して止まる（motion 側の `/cmd_walk` タイムアウトも効く）。
  **止めるより、止まったことが分かる状態を残すほうを採る。**
* teleop が死ぬと OLED は最後の表示のまま固まり、LED は赤の点滅で残る。ターミナルには
  `process has died` が出る。この 3 つで気付ける前提。

### ★ 端末から起動すること

`ros2 launch` を**バックグラウンドで起動すると Ctrl-C（SIGINT）が効かない。**
非対話シェルのバックグラウンドジョブは SIGINT を無視する設定を継承し、それが launch の
子プロセスにも伝わるため。systemd 化するときも `KillSignal=SIGTERM` にすること
（既定なのでそのままでよい）。手で止めるときは:

```bash
kill -TERM <ros2 launch の PID>
```

## 記録

既定で `config/record_topics.yaml` のトピックを `~/roboone_logs/rosbag2_<日時>/` に録る。

**点群と画像は入れていない。** 848x480x30 の点群だけで数十 MB/s になり、SD カードが
持たない。カメラを録りたいときは別建てで `ros2 bag record /camera/...` を回す。

`/joint_states`（実測）だけでは沈み込みが分からないので、`/motion/joint_commands`（指令）と
`/motion/servo_states`（サーボ空間の生カウントと負荷）も録っている。あとから追う手順は
`docs/commands.md` の「ログを後から追う」。

実測: teleop + ui だけで **60 秒 142KB**（`/cmd_walk` の 20Hz が 1210 件）。motion 込みでも
80 秒 2.4MB（2026-08-28）。1 時間回しても数百 MB なので、練習中は付けっぱなしで構わない。

```bash
ros2 bag info ~/roboone_logs/rosbag2_2026_08_28-21_00_46
ros2 bag play ~/roboone_logs/rosbag2_2026_08_28-21_00_46
```
