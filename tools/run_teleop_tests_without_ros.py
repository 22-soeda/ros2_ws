#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS の無い開発 PC で roboone_teleop のテストを回す。

    python tools/run_teleop_tests_without_ros.py            # 全部
    python tools/run_teleop_tests_without_ros.py -k deadman # pytest の引数はそのまま通る

tools/fakeros (最小の rclpy 代替) を sys.path の先頭に入れて pytest を起動するだけ。
本物の rclpy ではないので、ここで通るのは「自分のコードの結線が壊れていない」まで。
**最終判定は Pi の ``colcon test --packages-select roboone_teleop``。**
"""

import os
import pathlib
import sys

WS = pathlib.Path(__file__).resolve().parents[1]

if os.environ.get('ROS_DISTRO'):
    sys.exit('ROS 環境ではこれを使わない (本物の rclpy を隠してしまう)。'
             'colcon test --packages-select roboone_teleop を回すこと')

sys.path.insert(0, str(WS / 'tools' / 'fakeros'))
sys.path.insert(0, str(WS / 'src' / 'roboone_teleop'))

import pytest  # noqa: E402

TESTS = [
    str(WS / 'src' / 'roboone_teleop' / 'test' / 'test_params.py'),
    str(WS / 'src' / 'roboone_teleop' / 'test' / 'test_teleop.py'),
]

if __name__ == '__main__':
    extra = sys.argv[1:]
    print('[fakeros] 本物の rclpy ではない。最終判定は Pi の colcon test', file=sys.stderr)
    sys.exit(pytest.main(['-q', '-p', 'no:cacheprovider', *TESTS, *extra]))
