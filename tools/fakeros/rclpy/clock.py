# -*- coding: utf-8 -*-
"""rclpy.clock の最小版。壁時計 (time.time) をそのまま ROS 時刻にする。"""

import time


class Time:

    def __init__(self, nanoseconds=0):
        self.nanoseconds = int(nanoseconds)

    def seconds_nanoseconds(self):
        return divmod(self.nanoseconds, 1_000_000_000)


class Clock:

    def now(self):
        return Time(int(time.time() * 1e9))
