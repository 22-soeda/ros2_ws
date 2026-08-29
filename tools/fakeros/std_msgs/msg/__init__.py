# -*- coding: utf-8 -*-
"""std_msgs.msg の最小版。"""

from _msgbase import Msg


class Bool(Msg):
    _fields = (('data', False),)


class String(Msg):
    _fields = (('data', ''),)


class Float32(Msg):
    _fields = (('data', 0.0),)


class Int32(Msg):
    _fields = (('data', 0),)


class Header(Msg):
    _fields = (('stamp', None), ('frame_id', ''))
