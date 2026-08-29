# -*- coding: utf-8 -*-
"""roboone_interfaces.msg の最小版。定義は src/roboone_interfaces/msg/*.msg と揃えること。"""

from _msgbase import Msg


class OledText(Msg):
    _fields = (('line1', ''), ('line2', ''), ('r', 0), ('g', 0), ('b', 0))


class LedColor(Msg):
    _fields = (('r', 0), ('g', 0), ('b', 0))
