# -*- coding: utf-8 -*-
"""rclpy.node.Node の最小版。

まねているもの: publisher / subscription / timer / logger / clock / パラメータ
(宣言・上書き・型の静的/動的・on_set コールバック・set_parameters)。
パラメータの挙動は本物の rclpy (Jazzy) の順序に合わせてある:

  * declare_parameter は parameter_overrides の値を採り、dynamic_typing でなければ
    既定値と型が違うと InvalidParameterTypeException
  * set_parameters は 1 個ずつ。コールバックは**適用前**に呼ばれ、どれかが
    successful=False を返すとその値は入らない
  * add_on_set_parameters_callback は新しいものが先に呼ばれる
"""

import collections
import sys
import time

from . import _bus
from .clock import Clock
from .exceptions import (InvalidParameterTypeException, InvalidParameterValueException,
                         ParameterAlreadyDeclaredException, ParameterNotDeclaredException)
from .parameter import Parameter
from .qos import as_profile, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


class Logger:

    def __init__(self, name):
        self._name = name

    def _out(self, level, msg):
        print(f'[{level}] [{time.time():.3f}] [{self._name}]: {msg}', file=sys.stderr)

    def debug(self, msg, *a, **k):
        self._out('DEBUG', msg)

    def info(self, msg, *a, **k):
        self._out('INFO', msg)

    def warn(self, msg, *a, **k):
        self._out('WARN', msg)

    warning = warn

    def error(self, msg, *a, **k):
        self._out('ERROR', msg)

    def fatal(self, msg, *a, **k):
        self._out('FATAL', msg)

    def get_child(self, name):
        return Logger(f'{self._name}.{name}')


class Publisher:

    def __init__(self, node, msg_type, topic, qos):
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self.qos = qos
        self.latched = qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        self.last = None

    @property
    def topic_name(self):
        return self.topic

    def publish(self, msg):
        _bus.publish(self, msg)

    def get_subscription_count(self):
        return _bus.subscription_count(self)


class Subscription:

    def __init__(self, node, msg_type, topic, callback, qos):
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos = qos
        self.transient_local = qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
        self.queue = collections.deque()

    @property
    def topic_name(self):
        return self.topic

    def enqueue(self, msg):
        self.queue.append(msg)
        # KEEP_LAST depth を超えた古いものは捨てる (本物と同じ)
        while len(self.queue) > max(1, self.qos.depth):
            self.queue.popleft()


class Timer:

    def __init__(self, node, period_sec, callback):
        self.node = node
        self.period = float(period_sec)
        self.callback = callback
        self.next_due = time.monotonic() + self.period
        self._canceled = False

    @property
    def timer_period_ns(self):
        return int(self.period * 1e9)

    def cancel(self):
        self._canceled = True

    def reset(self):
        self._canceled = False
        self.next_due = time.monotonic() + self.period

    def is_canceled(self):
        return self._canceled

    def is_ready(self):
        return not self._canceled and time.monotonic() >= self.next_due

    def fire(self):
        # 遅れを引きずらないよう「今」から次を数える (本物は等間隔だが、テスト用途では十分)
        self.next_due = time.monotonic() + self.period
        self.callback()

    def destroy(self):
        self.cancel()
        self.node.destroy_timer(self)


class Node:

    def __init__(self, node_name, *, context=None, cli_args=None, namespace=None,
                 use_global_arguments=True, enable_rosout=True, start_parameter_services=True,
                 parameter_overrides=None, allow_undeclared_parameters=False,
                 automatically_declare_parameters_from_overrides=False):
        self._name = node_name
        self._logger = Logger(node_name)
        self._clock = Clock()
        self._parameters = {}
        self._descriptors = {}
        self._declared_types = {}
        self._parameter_overrides = {p.name: p for p in (parameter_overrides or [])}
        self._parameters_callbacks = []
        self.publishers = []
        self.subscriptions = []
        self.timers = []

    # ---------------------------------------------------------------- 基本
    def get_name(self):
        return self._name

    def get_namespace(self):
        return '/'

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    # ---------------------------------------------------------- パラメータ
    def declare_parameter(self, name, value=None, descriptor=None, ignore_override=False):
        if name in self._parameters:
            raise ParameterAlreadyDeclaredException(name)
        if descriptor is None:
            descriptor = ParameterDescriptor()
        declared_type = Parameter.Type.from_parameter_value(value)
        if not ignore_override and name in self._parameter_overrides:
            value = self._parameter_overrides[name].value
        param = Parameter(name, value=value)
        if not descriptor.dynamic_typing and declared_type != Parameter.Type.NOT_SET \
                and param.type_ != declared_type:
            raise InvalidParameterTypeException(
                f'{name}: 宣言は {declared_type.name} なのに {param.type_.name} が来た')
        descriptor.name = name
        descriptor.type = declared_type.value
        self._descriptors[name] = descriptor
        self._declared_types[name] = declared_type
        result = self._run_callbacks([param])
        if not result.successful:
            raise InvalidParameterValueException(f'{name}: {result.reason}')
        self._parameters[name] = param
        return param

    def declare_parameters(self, namespace, parameters):
        out = []
        for name, value, *rest in parameters:
            full = f'{namespace}.{name}' if namespace else name
            out.append(self.declare_parameter(full, value, rest[0] if rest else None))
        return out

    def has_parameter(self, name):
        return name in self._parameters

    def get_parameter(self, name):
        if name not in self._parameters:
            raise ParameterNotDeclaredException(name)
        return self._parameters[name]

    def get_parameters(self, names):
        return [self.get_parameter(n) for n in names]

    def set_parameters(self, parameter_list):
        return [self._set_one(p) for p in parameter_list]

    def _set_one(self, param):
        if param.name not in self._parameters:
            raise ParameterNotDeclaredException(param.name)
        desc = self._descriptors[param.name]
        declared = self._declared_types[param.name]
        if not desc.dynamic_typing and param.type_ != declared:
            raise InvalidParameterTypeException(
                f'{param.name}: 宣言は {declared.name} なのに {param.type_.name} を入れようとした')
        result = self._run_callbacks([param])
        if result.successful:
            self._parameters[param.name] = param
        return result

    def add_on_set_parameters_callback(self, callback):
        self._parameters_callbacks.insert(0, callback)

    def remove_on_set_parameters_callback(self, callback):
        self._parameters_callbacks.remove(callback)

    def _run_callbacks(self, params):
        for cb in self._parameters_callbacks:
            r = cb(params)
            if r is None or not isinstance(r, SetParametersResult):
                raise TypeError('on_set_parameters コールバックは SetParametersResult を返すこと')
            if not r.successful:
                return r
        return SetParametersResult(successful=True)

    # -------------------------------------------------------------- 通信
    def create_publisher(self, msg_type, topic, qos_profile, **kwargs):
        pub = Publisher(self, msg_type, topic, as_profile(qos_profile))
        self.publishers.append(pub)
        _bus.add_pub(pub)
        return pub

    def create_subscription(self, msg_type, topic, callback, qos_profile, **kwargs):
        sub = Subscription(self, msg_type, topic, callback, as_profile(qos_profile))
        self.subscriptions.append(sub)
        _bus.add_sub(sub)
        return sub

    def create_timer(self, timer_period_sec, callback, callback_group=None, clock=None,
                     autostart=True):
        t = Timer(self, timer_period_sec, callback)
        if not autostart:
            t.cancel()
        self.timers.append(t)
        return t

    def destroy_timer(self, timer):
        timer.cancel()
        if timer in self.timers:
            self.timers.remove(timer)
        return True

    def destroy_publisher(self, pub):
        _bus.remove_pub(pub)
        if pub in self.publishers:
            self.publishers.remove(pub)
        return True

    def destroy_subscription(self, sub):
        _bus.remove_sub(sub)
        if sub in self.subscriptions:
            self.subscriptions.remove(sub)
        return True

    def destroy_node(self):
        for t in list(self.timers):
            self.destroy_timer(t)
        for p in list(self.publishers):
            self.destroy_publisher(p)
        for s in list(self.subscriptions):
            self.destroy_subscription(s)
        return True
