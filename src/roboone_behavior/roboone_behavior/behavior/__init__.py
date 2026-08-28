# -*- coding: utf-8 -*-
"""行動層の計算部分。ROS に依存しない。

docs/behavior_planning.pdf §6.2 の「行動の本体は behavior_core に置き、観測列を
与えれば決定的に動く」にあたる層。ノード (roboone_behavior.behavior_node) は
ここへ観測を渡して結果を publish するだけで、判断は 1 つも持たない。
"""

from .core import BehaviorCore
from .keepalive import KeepAlive
from .params import BehaviorParams, MatchParams, RobotParams, TuneParams
from .ring import distance_to_edge, forward_cliff, ray_to_edge, RingPose
from .tracking import FallenDetector, OpponentTracker
from .types import (APPROACH, Command, debug_array, DEBUG_ORDER, EDGE,
                    ENGAGE, Observation, RETREAT, SEARCH, SELF_DOWN, STATES,
                    STATUS_ATTITUDE_STALE, STATUS_NO_OPPONENT, STATUS_OK,
                    STATUS_RING_LOST, WAIT)

__all__ = [
    'APPROACH', 'BehaviorCore', 'BehaviorParams', 'Command', 'DEBUG_ORDER',
    'EDGE', 'ENGAGE', 'FallenDetector', 'KeepAlive', 'MatchParams',
    'Observation', 'OpponentTracker', 'RETREAT', 'RingPose', 'RobotParams',
    'SEARCH', 'SELF_DOWN', 'STATES', 'STATUS_ATTITUDE_STALE',
    'STATUS_NO_OPPONENT', 'STATUS_OK', 'STATUS_RING_LOST', 'TuneParams',
    'WAIT', 'debug_array', 'distance_to_edge', 'forward_cliff', 'ray_to_edge',
]
