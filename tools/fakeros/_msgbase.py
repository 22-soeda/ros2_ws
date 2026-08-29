# -*- coding: utf-8 -*-
"""rosidl が生成するメッセージクラスの最小版。キーワード初期化と == と repr だけ。"""


class Msg:
    """サブクラスは ``_fields = (('name', default_or_factory), ...)`` を持つ。"""

    _fields = ()

    def __init__(self, **kw):
        for name, default in self._fields:
            v = kw.pop(name, None)
            if v is None:
                v = default() if callable(default) else default
            setattr(self, name, v)
        if kw:
            raise TypeError(f'{type(self).__name__} に無いフィールド: {sorted(kw)}')

    def __eq__(self, other):
        return type(self) is type(other) and all(
            getattr(self, n) == getattr(other, n) for n, _ in self._fields)

    def __repr__(self):
        body = ', '.join(f'{n}={getattr(self, n)!r}' for n, _ in self._fields)
        return f'{type(self).__name__}({body})'
