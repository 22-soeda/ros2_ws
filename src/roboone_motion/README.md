# roboone_motion — 歩行計画 (walk_core) と可視化

`docs/ros2_walk_implementation.pdf` の §11「実装の順番」の **手順 1〜2** に相当する。

> walk_core は C++ 版 (`src/roboone_walk_core`、実機用) と JS 版
> (`roboone_motion/viz/walkcore.js`、ブラウザシミュレータ用) にも移植してある。
> **ここの Python 版が仕様の原本。** ロジックを変えるときは 3 つ揃えて
> `python3 src/roboone_walk_core/tools/compare_walk_engines.py` で数値一致を確認する。

- `roboone_motion/walk_core/` — ROS 非依存の歩行計画ライブラリ。
  時計も乱数も持たず `update(vx, vy, dt)` の入力列だけで決定的に動く (文書 §1.3)
- `config/gait.yaml` — 静的設定 (文書 表 2 のうち計画に効く項目)
- `roboone_motion/viz/` — 歩行パラメータ生成〜重心軌道生成の過程を可視化する
  自己完結 HTML の生成器
- `test/test_walk_core.py` — 文書 §9 の単体テスト (LIPM 整合・決定性・収束など)

motion ノード本体 (RT スレッド・サーボ送信・/cmd_walk 購読) は次の段階。
walk_core の入出力は文書 §1.3 の表の形に固定してあるので、ノードは配線するだけ。

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
python3 -m pytest src/roboone_motion/test/test_walk_core.py

# ライブラリとして
from roboone_motion.walk_core import WalkEngine, GaitParams
eng = WalkEngine(GaitParams.from_yaml('config/gait.yaml'))
out = eng.update(vx, vy, 0.005)   # 200 Hz で回す

# 可視化 HTML の生成 + 配信
python3 src/roboone_motion/roboone_motion/viz/gen_walk_viz.py --serve 8100
# → SSH の PC から:  ssh -L 8100:localhost:8100 <pi>  →  http://localhost:8100/walk_viz.html
# 任意の指令のシナリオを足す:  --vx 0.12 --vy -0.04
```

可視化には次が入っている:

- 上面図アニメーション: 足配置・遊脚軌道・クランプ域・名目/補正後の着地点、
  ξ (DCM)・重心・ZMP 参照の軌跡
- 指令 → 整形 (式 1,2) → 歩幅 (式 4) → 名目着地 (式 5) → b (式 8) →
  終端 ξ 予測 (式 9) → 着地点 (式 10,11) のパイプライン表示 (毎周期の値)
- x / y / 遊脚 z の時系列チャート (ホバーで値、ドラッグでシーク)
- 歩の履歴表 (歩の境界で確定したパラメータ、クランプ発動の表示)
- **「🕹 操縦」タブ = ライブシミュレータ**: 右パネルのパッドをドラッグ
  (または WASD / 矢印キー) すると、JS 版 walk_core が 200 Hz で回って
  歩行がリアルタイムに生成される。離すと停止シーケンスに入る
