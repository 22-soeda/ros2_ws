# -*- coding: utf-8 -*-
"""geometry_msgs.msg の最小版。"""

from _msgbase import Msg


class Vector3(Msg):
    _fields = (('x', 0.0), ('y', 0.0), ('z', 0.0))


class Twist(Msg):
    _fields = (('linear', Vector3), ('angular', Vector3))
