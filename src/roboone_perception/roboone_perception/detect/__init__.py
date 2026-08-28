# -*- coding: utf-8 -*-
"""敵機検出の計算部分。ROS に依存しない。

docs/opponent_detection.pdf の tracker.py / detector.py にあたる層で、
段の並びとしきい値はその文書を仕様として実装してある。ノード
(roboone_perception.opponent_detector_node) はここへ深度画像と IMU を
渡して結果を publish するだけで、判断はここに閉じている。
"""

from .attitude import AttitudeEstimator
from .clusters import Cluster
from .geometry import (Deprojector, forward_ref, Intrinsics, ring_basis,
                       to_plane)
from .grid import GridSpec
from .params import BodyParams, DetectorParams, MatchParams, TuneParams
from .pipeline import (ATTITUDE_STALE, DetectionResult, NO_OPPONENT, OK,
                       RING_LOST, RingDetector)
from .tracker import AlphaBetaTracker

__all__ = [
    'ATTITUDE_STALE', 'AlphaBetaTracker', 'AttitudeEstimator', 'BodyParams',
    'Cluster', 'Deprojector', 'DetectionResult', 'DetectorParams', 'GridSpec',
    'Intrinsics', 'MatchParams', 'NO_OPPONENT', 'OK', 'RING_LOST',
    'RingDetector', 'TuneParams', 'forward_ref', 'ring_basis', 'to_plane',
]
