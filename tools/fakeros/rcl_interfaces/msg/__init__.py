# -*- coding: utf-8 -*-
"""rcl_interfaces.msg の最小版 (パラメータまわりだけ)。"""

from _msgbase import Msg


class ParameterType:
    PARAMETER_NOT_SET = 0
    PARAMETER_BOOL = 1
    PARAMETER_INTEGER = 2
    PARAMETER_DOUBLE = 3
    PARAMETER_STRING = 4
    PARAMETER_BYTE_ARRAY = 5
    PARAMETER_BOOL_ARRAY = 6
    PARAMETER_INTEGER_ARRAY = 7
    PARAMETER_DOUBLE_ARRAY = 8
    PARAMETER_STRING_ARRAY = 9


class ParameterDescriptor(Msg):
    _fields = (
        ('name', ''), ('type', 0), ('description', ''), ('additional_constraints', ''),
        ('read_only', False), ('dynamic_typing', False),
        ('floating_point_range', list), ('integer_range', list),
    )


class SetParametersResult(Msg):
    _fields = (('successful', False), ('reason', ''))
