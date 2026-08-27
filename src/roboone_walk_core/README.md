# roboone_walk_core — 歩行計画エンジンの C++ 版

`roboone_motion` (Python) の walk_core を機械移植したもの。ヘッダオンリ・ROS 非依存。
**Python 版が仕様の原本**で、設計判断と式の導出は
`src/roboone_motion/roboone_motion/walk_core/engine.py` の docstring にある。

- `include/roboone_walk_core/walk_engine.hpp` — エンジン本体
- `include/roboone_walk_core/gait_params.hpp` — 静的設定 (既定値は gait.yaml と同じ。
  YAML 読み込みは持たず、motion ノードが ROS パラメータから詰める)
- `src/walk_selftest.cpp` — 自己検算 (colcon test で回る)
- `src/walk_dump.cpp` — 軌道 CSV ダンプ (照合と後段への受け渡し用)
- `tools/compare_walk_engines.py` — **Python / C++ / JS の 3 実装の数値照合**

## 3 実装の同期ルール

walk_core は 3 か所に同じロジックがある。**変更は必ず 3 つ揃えて**照合を回すこと。

| 実装 | 場所 | 用途 |
|---|---|---|
| Python (原本) | roboone_motion/walk_core/engine.py | 仕様・単体テスト・可視化データ生成 |
| C++ | この walk_engine.hpp | motion ノード (実機 200 Hz) |
| JS | roboone_motion/viz/walkcore.js | ブラウザのライブ操縦シミュレータ |

```bash
colcon build --packages-select roboone_walk_core
python3 src/roboone_walk_core/tools/compare_walk_engines.py
# → 全指令ケースで最大誤差 ~1e-15 (機械精度) を確認済み。許容は 1e-6 m
```

## 性能 (Pi 5 実測)

`update()` 1 回 = **約 0.1 µs** (Python 版は約 11 µs)。200 Hz の周期 5 ms に対して
どちらも十分軽く、移植の動機は速度ではなく「IK・サーボ経路が C++ だから」である。

## motion ノードからの使い方 (次段階)

```cpp
#include "roboone_walk_core/walk_engine.hpp"
roboone_walk_core::WalkEngine eng;          // GaitParams を ROS パラメータから詰める
auto out = eng.update(vx, vy, 0.005, estop);
// out.left_foot_in_pelvis() / right_foot_in_pelvis() を
// roboone_kinematics の IK に渡す (骨盤水平座標系 {L}、ヨー 0)
```
