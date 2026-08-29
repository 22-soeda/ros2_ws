# -*- coding: utf-8 -*-
"""rclpy.signals の最小版。値を持っているだけで、シグナルには何もしない。"""

from enum import IntEnum


class SignalHandlerOptions(IntEnum):
    NO = 0
    SIGINT = 1
    SIGTERM = 2
    ALL = 3
