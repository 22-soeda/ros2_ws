# -*- coding: utf-8 -*-
"""static_walk — ROS 非依存の静歩行の計画ライブラリ。

walk_core (動歩行) と同じく、時計も乱数も持たず update(vx, vy, dt) の入力列だけで
決定的に動く。出力は walk_core の WalkOutputs と同じ型。
"""

from .engine import (check_static_gait, SHIFT, StaticWalkEngine, support_margin,
                     SWING)
from .params import StaticGaitParams

__all__ = [
    'StaticGaitParams', 'StaticWalkEngine', 'check_static_gait', 'support_margin',
    'SHIFT', 'SWING',
]
