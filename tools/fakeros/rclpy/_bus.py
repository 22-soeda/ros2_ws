# -*- coding: utf-8 -*-
"""プロセス内の配線。トピック名で publisher と subscription を結ぶ。

まねている DDS の性質:
  * 同じトピック名なら型を問わず届く (型の照合はしない)
  * TRANSIENT_LOCAL の publisher は最後の 1 件を持ち、後から来た TRANSIENT_LOCAL の
    subscription にそれを渡す (latched)
  * VOLATILE の publisher と TRANSIENT_LOCAL の subscription はマッチしない
  * 届く順序はトピック内で publish 順。別トピック間の順序は保証しない
"""

import copy
import threading

_lock = threading.RLock()
_wake = threading.Event()
_pubs = []
_subs = []


def reset():
    with _lock:
        _pubs.clear()
        _subs.clear()
        _wake.clear()


def add_pub(pub):
    with _lock:
        _pubs.append(pub)


def remove_pub(pub):
    with _lock:
        if pub in _pubs:
            _pubs.remove(pub)


def add_sub(sub):
    with _lock:
        _subs.append(sub)
        # latched: 既に出ている最後の 1 件を後追いで渡す
        for pub in _pubs:
            if pub.topic == sub.topic and _match(pub, sub) and pub.latched and pub.last is not None:
                sub.enqueue(copy.deepcopy(pub.last))
        _wake.set()


def remove_sub(sub):
    with _lock:
        if sub in _subs:
            _subs.remove(sub)


def publish(pub, msg):
    with _lock:
        if pub.latched:
            pub.last = copy.deepcopy(msg)
        for sub in _subs:
            if sub.topic == pub.topic and _match(pub, sub):
                sub.enqueue(copy.deepcopy(msg))
        _wake.set()


def subscription_count(pub):
    with _lock:
        return sum(1 for s in _subs if s.topic == pub.topic and _match(pub, s))


def wait(timeout):
    """何か届くかタイムアウトまで待つ。"""
    _wake.wait(timeout)
    with _lock:
        _wake.clear()


def _match(pub, sub):
    # subscription が TRANSIENT_LOCAL を求めるなら publisher もそうでないと繋がらない
    return pub.latched or not sub.transient_local
