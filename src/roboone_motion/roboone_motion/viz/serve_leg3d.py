# -*- coding: utf-8 -*-
"""脚の 3D ビジュアライザを HTTP で配信する。

    python3 src/roboone_motion/roboone_motion/viz/serve_leg3d.py --port 8101

有線 LAN (eth0) に出すので、PC のブラウザから

    http://<Pi の eth0 アドレス>:8101/

で開く。SSH のポートフォワード (ssh -L 8101:localhost:8101 <pi>) でも同じ。

**画面に出る関節位置はすべて C++ の leg_kinematics.hpp が計算したもの**で、
可視化のために書き直した別実装ではない。roboone_kinematics の leg_service を
子プロセスとして起動し、1 行 1 リクエストの JSON でやりとりしている。
歩行モードは walk_core (Python 版) の出力をそのまま足先目標に流し込む。
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
from urllib.parse import parse_qs, urlparse

_HERE = Path(__file__).resolve().parent
_WS = _HERE.parents[3]                     # <ws>/src/roboone_motion/roboone_motion/viz

if __package__ in (None, ''):
    sys.path.insert(0, str(_HERE.parents[1]))
from roboone_motion.walk_core import GaitParams, WalkEngine   # noqa: E402

PAGE = _HERE / 'leg3d.html'
ENGINE_DT = 0.005          # walk_core を回す刻み [s]


def find_service() -> Path:
    """leg_service の実行ファイルを探す。"""
    cands = [
        _WS / 'build' / 'roboone_kinematics' / 'leg_service',
        _WS / 'install' / 'roboone_kinematics' / 'lib' / 'roboone_kinematics' / 'leg_service',
    ]
    for c in cands:
        if c.is_file() and os.access(c, os.X_OK):
            return c
    raise SystemExit(
        'leg_service が見つからない。先に\n'
        '  colcon build --packages-select roboone_kinematics\n'
        'を実行する。探した場所:\n  ' + '\n  '.join(str(c) for c in cands))


class LegService:
    """leg_service 子プロセスの薄いラッパ。要求は 1 本ずつ直列に流す。"""

    def __init__(self, exe: Path):
        self.proc = subprocess.Popen(
            [str(exe)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        self.lock = threading.Lock()

    def call(self, line: str) -> dict:
        with self.lock:
            self.proc.stdin.write(line.rstrip('\n') + '\n')
            self.proc.stdin.flush()
            out = self.proc.stdout.readline()
        if not out:
            raise RuntimeError('leg_service が落ちた')
        return json.loads(out)

    def calls(self, lines):
        """複数行をまとめて往復する（1 行 1 応答なので順序で対応が取れる）。"""
        with self.lock:
            for ln in lines:
                self.proc.stdin.write(ln.rstrip('\n') + '\n')
            self.proc.stdin.flush()
            return [json.loads(self.proc.stdout.readline()) for _ in lines]


SVC: LegService | None = None


def _f(q, key, default):
    try:
        return float(q.get(key, [default])[0])
    except (TypeError, ValueError):
        return float(default)


def api_walk(q) -> dict:
    """walk_core を回し、各フレームの足先目標を IK に通して関節位置まで出す。"""
    vx = _f(q, 'vx', 0.10)
    vy = _f(q, 'vy', 0.0)
    dur = min(max(_f(q, 'dur', 6.0), 0.5), 30.0)
    zc = _f(q, 'zc', 0.32)
    ramp = _f(q, 'ramp', 0.6)          # 指令を立ち上げる時間 [s]
    hold = _f(q, 'hold', 1.5)          # 最後に指令を 0 に落として止める前の保持
    every = max(int(_f(q, 'every', 4)), 1)

    p = GaitParams()
    p.z_c = zc
    # 足間隔 W。既定は gait.yaml の値だが、股間隔との噛み合わせを見たいので開ける
    w = _f(q, 'w', p.foot_spacing)
    p.foot_spacing = w
    eng = WalkEngine(p)

    n = int(round(dur / ENGINE_DT))
    stop_at = max(dur - hold, 0.2)
    frames = []
    reqs = []
    meta = []
    for i in range(n):
        t = i * ENGINE_DT
        if t < stop_at:
            k = min(t / ramp, 1.0) if ramp > 1e-6 else 1.0
            cx, cy = vx * k, vy * k
        else:
            cx = cy = 0.0
        o = eng.update(cx, cy, ENGINE_DT)
        if i % every:
            continue
        px, py, pz = o.pelvis
        # 骨盤水平座標系 = Σ_B。m -> mm
        lf = [(o.left_foot[j] - (px, py, pz)[j]) * 1000.0 for j in range(3)]
        rf = [(o.right_foot[j] - (px, py, pz)[j]) * 1000.0 for j in range(3)]
        reqs.append('ik L %.4f %.4f %.4f' % tuple(lf))
        reqs.append('ik R %.4f %.4f %.4f' % tuple(rf))
        meta.append({
            't': round(o.t, 3), 'state': o.state, 'support': o.support,
            'pelvis': [round(px * 1000, 2), round(py * 1000, 2), round(pz * 1000, 2)],
            'xi': [round(o.xi[0] * 1000, 1), round(o.xi[1] * 1000, 1)],
            'zmp': [round(o.zmp[0] * 1000, 1), round(o.zmp[1] * 1000, 1)],
            'target': [[round(v, 2) for v in lf], [round(v, 2) for v in rf]]})

    res = SVC.calls(reqs)
    for k, m in enumerate(meta):
        l, r = res[2 * k], res[2 * k + 1]
        m['left'] = l.get('origins')
        m['right'] = r.get('origins')
        m['thL'] = [round(v, 3) for v in (l.get('theta') or [])]
        m['thR'] = [round(v, 3) for v in (r.get('theta') or [])]
        m['stL'] = l.get('status')
        m['stR'] = r.get('status')
        m['RL'] = [round(v, 5) for v in (l.get('R') or [])]
        m['RR'] = [round(v, 5) for v in (r.get('R') or [])]
        frames.append(m)

    bad = [f['t'] for f in frames if f['stL'] != 'ok' or f['stR'] != 'ok']
    return {'ok': 1, 'dt': ENGINE_DT * every, 'frames': frames,
            'unreachable': len(bad), 'first_bad': (bad[0] if bad else None),
            'params': {'vx': vx, 'vy': vy, 'zc': zc, 'dur': dur, 'w': w}}


class Handler(BaseHTTPRequestHandler):
    server_version = 'leg3d'

    def log_message(self, fmt, *args):        # 既定のアクセスログは黙らせる
        pass

    def _send(self, body: bytes, ctype: str, code: int = 200):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(json.dumps(obj).encode('utf-8'), 'application/json', code)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        try:
            if u.path in ('/', '/index.html'):
                self._send(PAGE.read_bytes(), 'text/html; charset=utf-8')
                return
            if u.path == '/api/params':
                self._json(SVC.call('params'))
                return
            if u.path == '/api/set':
                key = q.get('key', [''])[0]
                self._json(SVC.call('set %s %s' % (key, _f(q, 'value', 0.0))))
                return
            if u.path == '/api/fk':
                side = q.get('side', ['R'])[0]
                th = q.get('th', ['0,0,0,0,0,0'])[0]
                vals = ' '.join('%.5f' % float(v) for v in th.split(','))
                self._json(SVC.call('fk %s %s' % (side, vals)))
                return
            if u.path == '/api/ik':
                side = q.get('side', ['R'])[0]
                self._json(SVC.call('ikpose %s %.4f %.4f %.4f %.4f %.4f %.4f' % (
                    side, _f(q, 'x', 0), _f(q, 'y', 0), _f(q, 'z', -260),
                    _f(q, 'roll', 0), _f(q, 'pitch', 0), _f(q, 'yaw', 0))))
                return
            if u.path == '/api/walk':
                self._json(api_walk(q))
                return
            self._send(b'not found', 'text/plain', 404)
        except Exception as e:                  # noqa: BLE001  画面に出したいので握る
            self._json({'ok': 0, 'error': '%s: %s' % (type(e).__name__, e)}, 500)


def local_addrs():
    out = []
    try:
        raw = subprocess.run(['ip', '-4', '-o', 'addr', 'show'],
                             capture_output=True, text=True).stdout
        for line in raw.splitlines():
            f = line.split()
            if len(f) > 3 and f[1] != 'lo':
                out.append((f[1], f[3].split('/')[0]))
    except OSError:
        out.append(('?', socket.gethostbyname(socket.gethostname())))
    return out


def main() -> int:
    global SVC
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8101)
    ap.add_argument('--bind', default='0.0.0.0')
    args = ap.parse_args()

    SVC = LegService(find_service())
    print(SVC.call('params'))

    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    print('=' * 62)
    print('脚 3D ビジュアライザ  http://<addr>:%d/' % args.port)
    for name, addr in local_addrs():
        print('   %-6s http://%s:%d/' % (name, addr, args.port))
    print('   停止は Ctrl-C')
    print('=' * 62, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n停止')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
