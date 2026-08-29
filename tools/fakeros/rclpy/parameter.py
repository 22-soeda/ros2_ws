# -*- coding: utf-8 -*-
"""rclpy.parameter の最小版。型の推定規則は本物と同じにしてある。"""

from enum import Enum


class Parameter:

    class Type(Enum):
        NOT_SET = 0
        BOOL = 1
        INTEGER = 2
        DOUBLE = 3
        STRING = 4
        BYTE_ARRAY = 5
        BOOL_ARRAY = 6
        INTEGER_ARRAY = 7
        DOUBLE_ARRAY = 8
        STRING_ARRAY = 9

        @classmethod
        def from_parameter_value(cls, value):
            if value is None:
                return cls.NOT_SET
            if isinstance(value, bool):
                return cls.BOOL
            if isinstance(value, int):
                return cls.INTEGER
            if isinstance(value, float):
                return cls.DOUBLE
            if isinstance(value, str):
                return cls.STRING
            if isinstance(value, (bytes, bytearray)):
                return cls.BYTE_ARRAY
            if isinstance(value, (list, tuple)):
                if all(isinstance(v, bool) for v in value):
                    return cls.BOOL_ARRAY
                if all(isinstance(v, int) and not isinstance(v, bool) for v in value):
                    return cls.INTEGER_ARRAY
                if all(isinstance(v, float) for v in value):
                    return cls.DOUBLE_ARRAY
                if all(isinstance(v, str) for v in value):
                    return cls.STRING_ARRAY
            raise TypeError(f'パラメータにできない値: {value!r}')

    def __init__(self, name, type_=None, value=None):
        if type_ is None:
            type_ = Parameter.Type.from_parameter_value(value)
        self._name = name
        self._type_ = type_
        self._value = value

    @property
    def name(self):
        return self._name

    @property
    def type_(self):
        return self._type_

    @property
    def value(self):
        return self._value

    def __repr__(self):
        return f'Parameter({self._name!r}, {self._type_.name}, {self._value!r})'
