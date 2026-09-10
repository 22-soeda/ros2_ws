# roboone_viz — 歩行・脚・膝の可視化

**ROS ノードは持たない。** 素の Python で起動して、ブラウザに自己完結 HTML を配信する。
どれも SSH ポートフォワード越しに手元の PC から見る前提。

歩行計画の側は `roboone_walk_ref`（仕様原本）を、脚と膝の側は `roboone_kinematics` の
`leg_service` を子プロセスで叩く。**可視化のために書き直した別実装は持たない。**
画面に出ているのは実装そのものの出力。

## 歩行計画の可視化

```bash
python3 src/roboone_viz/roboone_viz/gen_walk_viz.py --serve 8100
# → 手元の PC から:  ssh -L 8100:localhost:8100 <pi>  →  http://localhost:8100/walk_viz.html
# 任意の指令のシナリオを足す:  --vx 0.12 --vy -0.04
```

入っているもの:

- 上面図アニメーション: 足配置・遊脚軌道・クランプ域・名目/補正後の着地点、
  ξ (DCM)・重心・ZMP 参照の軌跡
- 指令 → 整形 (式 1,2) → 歩幅 (式 4) → 名目着地 (式 5) → b (式 8) →
  終端 ξ 予測 (式 9) → 着地点 (式 10,11) のパイプライン表示 (毎周期の値)
- x / y / 遊脚 z の時系列チャート (ホバーで値、ドラッグでシーク)
- 歩の履歴表 (歩の境界で確定したパラメータ、クランプ発動の表示)
- **「🕹 操縦」タブ = ライブシミュレータ**: 右パネルのパッドをドラッグ
  (または WASD / 矢印キー) すると、JS 版 walk_core (`walkcore.js`) が 200 Hz で回って
  歩行がリアルタイムに生成される。離すと停止シーケンスに入る

## 脚・膝の機構の可視化

```bash
# 両脚 3D (leg_service を子プロセスで起動する。先に roboone_kinematics をビルドすること)
python3 src/roboone_viz/roboone_viz/serve_legs3d.py

# 片脚 3D + 歩行エンジン連動
python3 src/roboone_viz/roboone_viz/serve_leg3d.py --port 8101

# 膝 4 節リンク (--id で実機のサーボを読む。--demo なら実機不要)
python3 src/roboone_viz/roboone_viz/serve_knee3d.py --demo
```

## 中身

| ファイル | 役割 |
|---|---|
| `gen_walk_viz.py` | 歩行の可視化 HTML を生成して配信する。`walk_viz` として install される |
| `record.py` | walk_core を回してシナリオごとのデータセットを作る |
| `walkcore.js` | walk_core の JS 移植（ブラウザのライブシミュレータ用） |
| `serve_leg3d.py` / `serve_legs3d.py` | `leg_service` を叩く脚の 3D ビューア |
| `serve_knee3d.py` | 膝 4 節リンクのビューア |
| `template.html` `*3d.html` | 配信するページの雛形 |
