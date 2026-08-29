# -*- coding: utf-8 -*-
"""rclpy.qos の最小版。depth / reliability / durability だけ意味を持つ。"""

from enum import IntEnum


class ReliabilityPolicy(IntEnum):
    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2


class DurabilityPolicy(IntEnum):
    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2


class HistoryPolicy(IntEnum):
    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2


class QoSProfile:

    def __init__(self, *, depth=10, history=HistoryPolicy.KEEP_LAST,
                 reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE, **ignored):
        self.depth = depth
        self.history = history
        self.reliability = reliability
        self.durability = durability


def as_profile(qos):
    """create_publisher / create_subscription の第 3 引数 (int か QoSProfile) を揃える。"""
    if isinstance(qos, QoSProfile):
        return qos
    return QoSProfile(depth=int(qos))
