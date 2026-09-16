# -*- coding: utf-8 -*-
"""記録した足先が実機の脚で届くかを、roboone_kinematics の leg_service で調べる。

leg_service はサーボに繋がらない計算だけのプロセス (serve_leg3d.py と同じものを使う)。
IK (ik) と、その下の機構層 (膝 4 節リンク・足首パラレルリンク) の両方が ok なら「届く」。

★足裏は水平 (ik コマンド) で見ている。motion ノードは home_pose.yaml の foot.rpy と
  body_pitch を掛けて IK を解くので、どちらかを 0 以外にしたら結果がずれる
  (2026-09-17 時点ではどちらも 0)。
"""

import json
import os
from pathlib import Path
import subprocess
from typing import Optional


def _candidates():
    ws = Path(__file__).resolve().parents[3]      # <ws>/src/roboone_viz/roboone_viz/reach.py
    yield ws / 'build' / 'roboone_kinematics' / 'leg_service'
    yield ws / 'install' / 'roboone_kinematics' / 'lib' / 'roboone_kinematics' / 'leg_service'
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = Path(get_package_prefix('roboone_kinematics'))
        yield prefix / 'lib' / 'roboone_kinematics' / 'leg_service'
    except Exception:
        pass


class LegReach:
    """leg_service 子プロセスの薄いラッパ。1 行 1 往復。"""

    def __init__(self, exe: Path):
        self.exe = exe
        self.proc = subprocess.Popen(
            [str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)

    @classmethod
    def find(cls) -> Optional['LegReach']:
        """leg_service が見つからなければ None (到達の判定を省く)。"""
        for c in _candidates():
            if c.is_file() and os.access(c, os.X_OK):
                return cls(c)
        return None

    def ok(self, side: str, x_mm: float, y_mm: float, z_mm: float) -> bool:
        """骨盤 (Σ_B 原点) から見た足裏中心 [mm] に、その脚が届くか。"""
        self.proc.stdin.write('ik %s %.3f %.3f %.3f\n' % (side, x_mm, y_mm, z_mm))
        self.proc.stdin.flush()
        r = json.loads(self.proc.stdout.readline())
        mech = r.get('mech') or {}
        return r.get('status') == 'ok' and mech.get('status', 'ok') == 'ok'

    def close(self):
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=2)
        except Exception:
            self.proc.kill()
