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
- **足踏み (試作)**: タブ「足踏み (試作)」「前進と足踏み (試作)」と、操縦タブの
  「足踏み」ボタン (M キー)。**可視化だけの試作で、実機の歩行エンジン
  (roboone_walk_core) にも仕様原本にも walkcore.js にも入っていない。**
  `record.py` の `MarchWalkEngine` と `template.html` の `MarchWalkEngineJS` が
  walk_core を包み、足踏み中だけ歩き出し・歩き続けのしきい値 (v_start_eps /
  v_stop_eps) を無効にして回す。速度指令は a_max で整形されたままなので、前進から
  足踏みへはなめらかに移る（踏み出し量を一気に 0 にすると純 FF では発散する）
- **静歩行**: タブ「静歩行 前進 / 左移動 / 斜め後ろ / 足踏み (試作)」と、操縦タブの
  「計画」ボタン (P キー)。計画は `roboone_walk_ref` の static_walk (仕様原本) で、
  ライブ操縦はその JS 版 `staticwalk.js`。設定は `static_gait.yaml` を読む。
  重心を両足支持で次の支持足の上へ移してから足を振り出す。速さは static_gait.yaml の値で、
  2026-09-18 は観察用に時間 3 倍 (1 歩約 6.4 s) にしてあるので、記録タブでは数歩しか進まない。
  パイプ欄に ZMP の静的余裕 (支持多角形の縁まで。足裏 118 × 74 mm) を出す。
  実機では motion ノードを `walk_mode:=static` で起動すると同じ計画 (C++ 版) で歩く。足踏みは可視化だけの包み
  (`record.py` の `MarchStaticWalkEngine` / `template.html` の `MarchStaticWalkEngineJS`)
- 足裏は実寸 (static_gait.yaml の sole_length / sole_width) で描く
- **実機の脚で届くか**: `leg_service` がビルドしてあれば、記録シナリオの各時刻の足先を
  IK と機構層 (膝・足首リンク) に通し、届かない足を注意色で囲む (`--no-reach` で省く)。
  足裏は水平で見ている (home_pose の rpy・body_pitch が 0 の前提)。
  ★動歩行のシナリオは、motion ノードが計画の立位をホーム姿勢の足 (home_pose.yaml の
  foot.y) へ平行移動する分を掛けずに、計画の足間隔 (gait.yaml の 170 mm) のまま見ている。
  実機の足 (140 mm) より届かない点が多く出る

```bash
# 静歩行の設定を変えて見る (static_gait.yaml の上に重ねる。yaml は書き換えない)
python3 src/roboone_viz/roboone_viz/gen_walk_viz.py --serve 8100 \
    --static-zmp-tol 0.005 --static-t-swing 0.8 --static-com-offset 0.01 --static-swing-height 0.02

# 静歩行の設定ごとに、足先が届くかを走査する (static_gait.yaml の表を作ったもの)
python3 src/roboone_viz/roboone_viz/static_reach.py --swing-height 0.03,0.025 --com-offset 0,0.005
```

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
| `staticwalk.js` | 静歩行 static_walk の JS 移植（ライブシミュレータ用。3 実装照合の対象） |
| `reach.py` | 記録した足先が届くかを `leg_service` で調べる |
| `static_reach.py` | 静歩行の設定ごとに、足先が届くかを走査する |
| `serve_leg3d.py` / `serve_legs3d.py` | `leg_service` を叩く脚の 3D ビューア |
| `serve_knee3d.py` | 膝 4 節リンクのビューア |
| `template.html` `*3d.html` | 配信するページの雛形 |
