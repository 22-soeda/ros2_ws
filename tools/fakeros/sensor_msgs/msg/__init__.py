# -*- coding: utf-8 -*-
"""sensor_msgs.msg の最小版。"""

from _msgbase import Msg
from std_msgs.msg import Header


class Joy(Msg):
    _fields = (('header', Header), ('axes', list), ('buttons', list))
