# -*- coding: utf-8 -*-
"""最小の rclpy 代替。ROS の無い開発 PC で roboone_teleop の結線テストを回すためのもの。

本物の rclpy と同じ名前・同じ呼び方で、**プロセス内の** pub/sub・タイマー・パラメータだけを
まねる。DDS も rcl も無い。ここで通っても「本物の rclpy でも通る」とは限らないので、
最終判定は Pi の ``colcon test``。何をまねて何をまねていないかは tools/README.md。

使い方は tools/run_teleop_tests_without_ros.py を見ること (sys.path にこのディレクトリを
先頭で足すだけ)。ROS 環境 (ROS_DISTRO が立っている) では使わない。
"""

from . import _bus

_ok = False


def init(args=None, *, context=None, signal_handler_options=None, domain_id=None):
    global _ok
    _bus.reset()
    _ok = True


def ok(*, context=None):
    return _ok


def shutdown(*, context=None):
    global _ok
    _ok = False
    _bus.reset()


def spin_once(node, *, executor=None, timeout_sec=None):
    from .executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.spin_once(timeout_sec=timeout_sec)


def spin(node, executor=None):
    from .executors import SingleThreadedExecutor
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    while _ok:
        ex.spin_once(timeout_sec=0.1)
