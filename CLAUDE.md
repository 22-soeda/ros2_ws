# CLAUDE.md

このリポジトリで作業するときの指針。

## コマンドは docs/commands.md に残す

**コードを実行するためのコマンド（ビルド・テスト・launch・可視化・実機操作など）を
新しく使ったり、既存のものを変えたりしたら、その都度
[docs/commands.md](docs/commands.md) に追記・更新すること。**

- 「一度うまくいったコマンド」を次回そのまま貼れる状態にしておくのが目的。
- 追記は該当セクションへ。なければ節を足す。
- 実機やハード依存（ポート名、電源、権限）の前提があるなら一行添える。
- 使わなくなった・動かなくなったコマンドは消すか、動かない旨を書く。

## ワークスペース

ROS 2 Jazzy の colcon ワークスペース。ターゲットは Raspberry Pi 5 上の二足ロボット。

| パッケージ | 中身 |
|---|---|
| `feetech_servo` | Feetech サーボの C++ ドライバ（2 バス構成）。`vendor/scservo` は上流ベンダ |
| `roboone_kinematics` | 脚 IK / 膝 4 節リンク / 足首パラレルリンク（C++）。`*_selftest` `*_dump` が実行形 |
| `roboone_walk_core` | 歩行コア（C++）。`walk_selftest` `walk_dump` |
| `roboone_motion` | 歩行の Python 実装と可視化（`viz/`） |
| `roboone_perception` | 相手検知（`detect/`） |
| `roboone_teleop` | PS5 DualSense からの操縦 |
| `roboone_ui` | OLED / RGB LED / ブザー |
| `roboone_bringup` | 機体全体の立ち上げ launch |
| `realsense_bringup` | RealSense の起動ラッパ |
| `roboone_interfaces` | msg / srv 定義 |
| `src/realsense-ros` | 上流クローン。**追跡外**（`.gitignore`）。触らない |

`scripts/` は C++ 実装と突き合わせるための Python リファレンス実装（`knee_fourbar.py`,
`leg_ik.py`, `leg_servo.py` ほか）。C++ 側を変えたら `scripts/crosscheck_*.py` で照合する。

## 作法

- ドキュメント・コミットメッセージ・コード内コメントは日本語（既存に合わせる）。
- `build/` `install/` `log/` `captures/` はコミットしない。
- 実機に指令を出すコマンド（サーボを動かす、電源を入れる類）は、実行前に必ず確認を取る。

## 実装中の検証ではトルクを入れない

**Claude が実装・デバッグの過程で実機を走らせるときは、サーボのトルクを入れない。**
これは「製品としての既定」ではなく、**開発作業中の走らせ方**の約束。通常運用では
トルクは入る（機体を支えた状態でユーザが動かす）。

- 検証で motion ノードを起動するときは `allow_torque:=false`（バスは開くが
  `enable_torque(id, true)` と位置指令を送らない。歩行計画・IK・モーション再生・
  `/joint_states` は全部回る）か、`dry_run:=true`（バスも開かない）を付ける。
- `roboone_motion_node` の `allow_torque` の**既定は `true`**（通常運用）。
  検証で落とすのは Claude 側の責任で、既定値を下げて誤魔化さない。
- トルクを入れるコマンドを提案・実行するときは、毎回そう明言して確認を取る。
- 新しくサーボへ書くツールを足すときは、既定を読み取り専用にして、書き込みは
  明示のフラグでしか起きないようにする（`feetech_leg_stream` / `leg_live_test` /
  `motion_teach` が先例。書き込みは起動時のトルク OFF 1 回だけ）。
