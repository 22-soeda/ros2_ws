# roboone_bringup

機体をひとまとめに立ち上げる launch。各パッケージの launch はそのまま残してあり、
このパッケージは「どれを、どの順で、どの引数で上げるか」だけを持つ。

```bash
ros2 launch roboone_bringup roboone.launch.py
```

これで ui と teleop（joy 込み）が上がる。止めるのは Ctrl-C。

## 引数

| 引数 | 既定 | 意味 |
|---|---|---|
| `ui` | `true` | ui ノード（OLED / RGB LED / ブザー） |
| `teleop` | `true` | joy + teleop ノード |
| `camera` | `false` | RealSense。USB 帯域と CPU を食うので、要るときだけ |
| `record` | `false` | `ros2 bag` 記録（`config/record_topics.yaml` のトピック） |
| `log_dir` | `~/roboone_logs` | 記録の置き場 |
| `joy_backend` | `game_controller` | teleop へそのまま渡す |
| `teleop_config` | `roboone_teleop/config/ps5_dualsense.yaml` | 割り当て YAML |

よく使う組み合わせ:

```bash
# 練習・試合（記録あり）
ros2 launch roboone_bringup roboone.launch.py record:=true

# 検知の開発（カメラも上げる）
ros2 launch roboone_bringup roboone.launch.py camera:=true

# 表示だけ確認したい
ros2 launch roboone_bringup roboone.launch.py teleop:=false
```

## 起動順と落ち方

* **ui を先に上げる。** teleop の状態表示は latched なので順序に厳密な依存は無いが、
  ui が先にいれば起動直後から「まだ `/joy` が来ていない」が画面に出る。立ち上げ中に
  何も表示されない時間を作らない。
* **joy_node は respawn する。** Bluetooth は切れるものなので、落ちたら上げ直す。
  切れている間は teleop の無通信ウォッチドッグが脱力を掛けるので、勝手に上げ直しても
  安全側は壊れない。
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

`record:=true` で `config/record_topics.yaml` のトピックを録る。

**点群と画像は入れていない。** 848x480x30 の点群だけで数十 MB/s になり、SD カードが
持たない。カメラを録りたいときは別建てで `ros2 bag record /camera/...` を回す。

実測: teleop + ui だけで **60 秒 142KB**（`/cmd_walk` の 20Hz が 1210 件）。
1 時間回しても 10MB 弱なので、練習中は付けっぱなしで構わない。

まだ誰も publish していないトピック（`/motion/state`・`/joint_states`・`/opponent`）も
先に書いてある。無いトピックは無視されるだけなので、ノードができたら勝手に載る。

```bash
ros2 bag info ~/roboone_logs/rosbag2_2026_08_28-04_55_01
ros2 bag play ~/roboone_logs/rosbag2_2026_08_28-04_55_01
```

## motion を足すとき

`roboone.launch.py` の「--- 3) motion（未実装）」のコメントの位置に 1 ブロック足す。
`/cmd_walk`・`/cmd_motion`・`/estop` の受け手なので、teleop より後で構わない
（`/estop` は latched なので、後から上げても直前の脱力状態が届く）。

記録トピックは既に `/motion/state` と `/joint_states` を含んでいるので、そちらの
変更は要らない。
