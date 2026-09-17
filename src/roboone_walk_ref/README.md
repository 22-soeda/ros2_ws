# roboone_walk_ref — 歩行計画 walk_core の仕様原本

`docs/ros2_walk_implementation.pdf` の §11「実装の順番」の **手順 1〜2** に相当する。
**ROS ノードは持たない。** ここにあるのは計画アルゴリズムそのものと、その正準の設定。

> walk_core は 3 つの実装がある。**ここの Python 版が仕様の原本。**
>
> | 実装 | 用途 |
> |---|---|
> | `roboone_walk_ref/walk_core/` (Python) | **仕様の原本** |
> | `roboone_walk_core` (C++) | 実機用。motion ノード (`roboone_motion`) がリンクする |
> | `roboone_viz/walkcore.js` (JS) | ブラウザシミュレータ |
>
> ロジックを変えるときは 3 つ揃えて
> `python3 src/roboone_walk_core/tools/compare_walk_engines.py` で数値一致を確認する。

- `roboone_walk_ref/walk_core/` — ROS 非依存の歩行計画ライブラリ。
  時計も乱数も持たず `update(vx, vy, dt)` の入力列だけで決定的に動く (文書 §1.3)
- `config/gait.yaml` — 静的設定 (文書 表 2 のうち計画に効く項目)。
  **motion ノードもこれを share から読む**（設定の原本もここに一本化してある）
- `config/home_pose.yaml` — ホーム姿勢 (足裏の位置姿勢)。同じく motion ノードが読む
- `test/test_walk_core.py` — 文書 §9 の単体テスト (LIPM 整合・決定性・収束など)
- `roboone_walk_ref/static_walk/` — **静歩行**の計画ライブラリ (仕様原本)。下の節
- `config/static_gait.yaml` — 静歩行の設定 (gait.yaml とは別ファイル・別の型)
- `test/test_static_walk.py` — 静歩行の単体テスト

可視化は `roboone_viz` に分けてある。

motion ノード本体 (RT スレッド・サーボ送信・/cmd_walk 購読) は `roboone_motion` に
実装済みで、**そこが走らせるのは C++ 移植の `roboone_walk_core`**。ここの Python 版は
実機の経路には入らない。

## 今回の設計判断 (文書からの微調整)

機体方針: **平行移動のみ (ωz ≡ 0、足ヨー ≡ 0)・IMU 外挿なしの純フィードフォワード**。

1. **世界座標で積分する。** 回転がないと支持足座標系の乗り換え (式 3) は純平行移動に
   なり、世界座標での積分と厳密に等価。オドメトリと可視化がそのまま取れる。
   回転を入れる段になったら式 (3) の乗り換えを外側に足す。
2. **DCM オフセット b は 2 歩周期の厳密解。** `b = (ℓ_A e^{ωT} + ℓ_B)/(e^{2ωT} − 1)`。
   前後は式 (6)、左右 (Ly=0) は式 (7) に一致し、横移動 (Ly≠0) では式 (8) の近似より
   正確 (導出は engine.py の docstring)。
3. **楕円制限 (式 2) は (vx, vy) に読み替え。** 旋回がないので、斜め歩きの
   遊脚到達域 (式 18) 超過を防ぐ役に転用した。
4. **横移動は進行方向側の足から踏み出す。** 文書 §4.1 の既定 (左支持・右足から) の
   ままだと、右へ歩くとき 1 歩目が閉じる方向になり内側クランプに当たるため。
5. **a_max を (0.3, 0.2) → (0.15, 0.05) に下げた。** 純 FF では 1 歩あたりの指令変化
   ΔL = a·T² が着地点クランプ (式 11) で吸収できる範囲を超えると、残差が
   e^{ωT} 倍で増幅されて発散する (params.py の注記)。踏み出し補正
   (推定 ξ) を入れる段階で戻すか再検討。
6. **START の遷移と歩の境界は閉形式で正確に取る。** 5 ms 離散化の行き過ぎ (最大 3 mm)
   も e^{ωT} 倍に増幅されるため、交差時刻を解いて ξ を交差点に置く。
7. **停止は 2 段 (prep → stop)。** 停止を判断した歩では ξ の始点が既に歩行の
   オフセットを持っていて終端 ξ を変えられないので、式 (21) の b になるよう
   準備歩の着地点を置き、次の歩で足を真横に揃える (engine.py `_update_prep_landing`)。
   停止シーケンス中に指令が復活しても完了させてから START でやり直す
   (途中復帰は ξ の整合が崩れる)。


## 使い方

```bash
# 単体テスト
python3 -m pytest src/roboone_walk_ref/test/test_walk_core.py

# ライブラリとして
from roboone_walk_ref.walk_core import WalkEngine, GaitParams
eng = WalkEngine(GaitParams.from_yaml('config/gait.yaml'))
out = eng.update(vx, vy, 0.005)   # 200 Hz で回す
```

可視化は `src/roboone_viz/README.md` を見ること。

## 静歩行 (static_walk)

重心を直接計画して、各瞬間で静的に立っている歩き方を作る。walk_core と同じく
`update(vx, vy, dt)` の入力列だけで決まり、出力は walk_core の `WalkOutputs` と同じ型
(state は IDLE / SHIFT / SWING / STOP / ESTOP)。

    IDLE -> SHIFT (両足支持で重心を支持足の上へ) -> SWING (止めたまま反対の足を振り出す)
         -> SHIFT -> ... -> 指令ゼロ: 足を揃える -> STOP (重心を中点へ) -> IDLE

- 重心移動の時間は「ZMP が重心の真下から `zmp_tol` 以上ずれない」ように距離から決める
  (10mm で足間隔 140mm に 1.47 s)
- 歩幅は `v·stride_time` で、振り出しの開始時に固定する。全速前進で 1 歩 2.13 s・0.028 m/s
- `foot_spacing` は実機の足間隔そのもの (動歩行の stance_y_offset の仕掛けは使わない)
- 足裏 118 × 74 mm (2026-09-17 実測) を IK の目標点中心の長方形とみなし、
  `support_margin()` で ZMP の静的余裕を出す。テストは全時刻でこれが正であることを見る
- 足上げは 25mm。重心を支持足の上に置くと遊脚が骨盤から横へ 140mm 以上開き、
  足首リンクが高く上げられない (表は static_gait.yaml)

walk_core と同じく 3 実装を揃える (JS は `roboone_viz/staticwalk.js`、
C++ は `roboone_walk_core` に入れる予定で未実装)。

```bash
python3 -m pytest src/roboone_walk_ref/test/test_static_walk.py -q
python3 src/roboone_walk_core/tools/compare_walk_engines.py --engine static

from roboone_walk_ref.static_walk import StaticWalkEngine, StaticGaitParams
eng = StaticWalkEngine(StaticGaitParams.from_yaml('config/static_gait.yaml'))
out = eng.update(vx, vy, 0.005)
```
