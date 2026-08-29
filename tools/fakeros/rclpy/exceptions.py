# -*- coding: utf-8 -*-
"""rclpy.exceptions の最小版。"""


class ParameterException(Exception):
    pass


class ParameterAlreadyDeclaredException(ParameterException):
    pass


class ParameterNotDeclaredException(ParameterException):
    pass


class InvalidParameterTypeException(ParameterException):
    pass


class InvalidParameterValueException(ParameterException):
    pass


class ParameterUninitializedException(ParameterException):
    pass
