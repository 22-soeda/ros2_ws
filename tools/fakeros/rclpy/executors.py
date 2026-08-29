# -*- coding: utf-8 -*-
"""rclpy.executors の最小版。spin_once で「準備できた 1 件」を実行する。"""

import time

from . import _bus


class SingleThreadedExecutor:

    def __init__(self, *, context=None):
        self._nodes = []

    def add_node(self, node):
        if node not in self._nodes:
            self._nodes.append(node)
        return True

    def remove_node(self, node):
        if node in self._nodes:
            self._nodes.remove(node)

    def get_nodes(self):
        return list(self._nodes)

    def spin_once(self, timeout_sec=None):
        end = None if timeout_sec is None else time.monotonic() + max(0.0, timeout_sec)
        while True:
            if self._run_one():
                return
            now = time.monotonic()
            if end is not None and now >= end:
                return
            wait = 0.05
            nxt = self._next_timer_due()
            if nxt is not None:
                wait = min(wait, max(0.0, nxt - now))
            if end is not None:
                wait = min(wait, max(0.0, end - now))
            if wait > 0.0:
                _bus.wait(wait)

    def spin(self):
        while True:
            self.spin_once(timeout_sec=0.1)

    def shutdown(self, timeout_sec=None):
        self._nodes.clear()
        return True

    # ---------------------------------------------------------------------
    def _run_one(self):
        for node in list(self._nodes):
            for t in list(node.timers):
                if t.is_ready():
                    t.fire()
                    return True
        for node in list(self._nodes):
            for s in list(node.subscriptions):
                if s.queue:
                    msg = s.queue.popleft()
                    s.callback(msg)
                    return True
        return False

    def _next_timer_due(self):
        due = [t.next_due for n in self._nodes for t in n.timers if not t.is_canceled()]
        return min(due) if due else None
