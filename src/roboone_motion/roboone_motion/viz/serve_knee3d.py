# -*- coding: utf-8 -*-
"""膝 4 節リンクの動作確認。サーボの角度を読んで 3D 表示をリアルタイムに動かす。

    # 実機（サーボ電源を入れてから）
    python3 src/roboone_motion/roboone_motion/viz/serve_knee3d.py --id 4

    # 実機なしで表示だけ確かめる
    python3 src/roboone_motion/roboone_motion/viz/serve_knee3d.py --demo

ブラウザで http://<Pi の eth0 アドレス>:8102/ を開く。
SSH のポートフォワード (ssh -L 8102:localhost:8102 <pi>) でも同じ。

===========================================================================
やること・やらないこと
===========================================================================
* サーボは **角度を読むだけ**。トルクは入れない。子プロセスの
  ``feetech_knee_stream`` が起動時に 1 回だけトルク OFF（脱力）を書き、
  それ以外の書き込み経路を持たない。膝は手で動かす。
* **起動した瞬間の姿勢を「膝が伸び切った状態」とみなす**（T ポーズ）。
  そのときのサーボ生角を θ2 = 185.71°（曲げ量 0 のクランク角）に対応づける。
  これはそのまま knee_config の φ0（サーボ原点）の原点出しになっていて、
  画面に「knee_config.py に書く値」として出る。
  ずれていたら脚を伸ばし直して「ゼロ取り直し」を押す。

===========================================================================
表示している量の出どころ
===========================================================================
サーボ生角 → クランク角 θ2 → **順変換 (KN-5)** → ロッカー角 θ4 → 曲げ量
→ 脚の関節位置。使っているのは ``scripts/knee_fourbar.py`` と ``scripts/leg_ik.py``
で、どちらも C++ 実装と突き合わせ済み（``crosscheck_knee.py`` / ``crosscheck_cpp.py``
で 1e-12 rad 以下の一致）。可視化のために書き直した別実装ではない。

4 節リンクを脚のどこに描くかについて: リンクの取り付け向き（O4→O2 が大腿の
どちらを向くか）は寸法 r1–r4 からは決まらないので、**表示では大腿の中心線上**
（膝から股方向に r1 = 20 mm）に置いている。**動き方は取り付け向きに依らず正しい**
——— 4 節リンクの地節は大腿に、ロッカーは下腿に固定されているので、機構全体は
大腿・下腿と一体で回るため。
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlparse

import numpy as np

_HERE = Path(__file__).resolve().parent
_WS = _HERE.parents[3]
sys.path.insert(0, str(_WS / 'scripts'))

import knee_config as kcfg                                 # noqa: E402
import knee_fourbar                                        # noqa: E402
import leg_ik                                              # noqa: E402
from leg_servo import leg_angle_from_knee_bend             # noqa: E402

PAGE = _HERE / 'knee3d.html'
D, R = math.degrees, math.radians
COUNTS = 4096.0          # サーボ 1 回転のカウント数


def find_stream() -> Path:
    """feetech_knee_stream の実行ファイルを探す。"""
    cands = [
        _WS / 'build' / 'feetech_servo' / 'feetech_knee_stream',
        _WS / 'install' / 'feetech_servo' / 'lib' / 'feetech_servo' / 'feetech_knee_stream',
    ]
    for c in cands:
        if c.is_file() and os.access(c, os.X_OK):
            return c
    raise SystemExit(
        'feetech_knee_stream が見つからない。先に\n'
        '  colcon build --packages-select feetech_servo\n'
        'を実行する。探した場所:\n  ' + '\n  '.join(str(c) for c in cands))


def to_body(p) -> list[float]:
    """文書の Σ_0（x 右 / y 前 / z 上）-> 機体座標 Σ_B（x 前 / y 左 / z 上）。

    leg3d.html と同じ向きで描けるように、送る前にここで 1 回だけ読み替える
    （leg_config.py の x_doc = -y_B, y_doc = x_B の逆向き）。
    """
    return [round(float(p[1]), 3), round(float(-p[0]), 3), round(float(p[2]), 3)]


def wrap180(a: float) -> float:
    """角を (−180, 180] に畳む [deg]。"""
    return D(math.atan2(math.sin(R(a)), math.cos(R(a))))


# --------------------------------------------------------------------------
# サーボ角の供給元
# --------------------------------------------------------------------------
class ServoSource:
    """feetech_knee_stream の JSON 行を読み続けて、最新サンプルを持つ。"""

    def __init__(self, exe: Path, port: str, sid: int, rate: float):
        self.proc = subprocess.Popen(
            [str(exe), '--port', port, '--id', str(sid), '--rate', str(rate)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1)
        self.latest: dict = {'ok': False, 'n': 0, 'miss': 0}
        self.err: list[str] = []
        self.lock = threading.Lock()
        self.label = f'{port} ID {sid}'
        threading.Thread(target=self._pump, daemon=True).start()
        threading.Thread(target=self._pump_err, daemon=True).start()

    def _pump(self) -> None:
        for line in self.proc.stdout:
            line = line.strip()
            if not line.startswith('{'):
                continue
            try:
                s = json.loads(line)
            except ValueError:
                continue
            with self.lock:
                # 読めなかったサンプルは前の生角を保持して、表示を固まらせない
                if not s.get('ok') and 'raw' in self.latest:
                    s['raw'] = self.latest['raw']
                    s['deg'] = self.latest.get('deg')
                self.latest = s

    def _pump_err(self) -> None:
        for line in self.proc.stderr:
            line = line.rstrip()
            if line:
                with self.lock:
                    self.err.append(line)
                    del self.err[:-8]
                print('[stream]', line, file=sys.stderr)

    def read(self) -> dict:
        with self.lock:
            s = dict(self.latest)
            s['stderr'] = list(self.err)
        if self.proc.poll() is not None:
            s['dead'] = True
        return s

    def stop(self) -> None:
        if self.proc.poll() is None:
            self.proc.terminate()


class DemoSource:
    """実機なしで表示を確かめるための、膝を手で曲げ伸ばしする動きの模擬。

    曲げ量 0 → 120° → 0 を 8 秒周期で往復させ、それを逆変換でサーボ生角に
    直して流す。**実機の値ではない**ので、画面にも DEMO と出す。
    """

    def __init__(self, link: knee_fourbar.KneeFourBar, raw0: int, gear: float, sigma_m: int):
        self.link, self.raw0, self.gear, self.sigma_m = link, raw0, gear, sigma_m
        self.t0 = time.monotonic()
        self.label = 'demo（実機なし）'
        self.theta2_ext = link.ik(link.rocker_from_joint(0.0)).theta2

    def read(self) -> dict:
        t = time.monotonic() - self.t0
        bend = 60.0 * (1.0 - math.cos(2.0 * math.pi * t / 8.0))       # 0..120 deg
        theta2 = self.link.ik(self.link.rocker_from_joint(R(bend))).theta2
        dphi = self.sigma_m * self.gear * D(theta2 - self.theta2_ext)
        raw = self.raw0 + dphi * COUNTS / 360.0
        return {'ok': True, 't': round(t, 3), 'raw': raw,
                'deg': raw * 360.0 / COUNTS, 'volt': 11.8, 'temp': 30,
                'load': 0.0, 'n': int(t * 50), 'miss': 0, 'stderr': [], 'demo': True}

    def stop(self) -> None:
        pass


# --------------------------------------------------------------------------
# 変換と姿勢
# --------------------------------------------------------------------------
class KneeModel:
    """サーボ生角 -> 4 節リンク -> 脚の関節位置。"""

    def __init__(self, side: str = 'right', sigma_m: int | None = None,
                 gear: float | None = None):
        self.side = side
        self.link = knee_fourbar.knee_fourbar(side)
        self.leg = leg_ik.leg_params(side)
        # 既定は knee_config の左右別の値。引数が来たらそちらを優先する
        self.sigma_m = int(kcfg.SIGMA_MOTOR[side]) if sigma_m is None else sigma_m
        self.gear = float(kcfg.GEAR[side]) if gear is None else gear
        self.raw0: float | None = None
        self.lock = threading.Lock()
        self.last_good: dict | None = None
        # 表示用: 4 節リンクの平面基底の向き。ロッカーと下腿が同じ向きに回るように取る
        self._e2_sign = self._solve_e2_sign()

    # -- 枝やギア比を変えると伸び切りのクランク角も変わる ------------------
    @property
    def theta2_ext(self) -> float:
        return self.link.ik(self.link.rocker_from_joint(0.0)).theta2

    @staticmethod
    def _plane_basis(o) -> tuple[np.ndarray, np.ndarray]:
        """膝の回転面（矢状面）の正規直交基底 (e1, e2) を作る。

        膝軸は Σ_0 の x（J4 は Rx）なので、回転面は y-z。大腿は膝軸方向にも
        オフセットを持つ（a3 + a4）ので、**膝軸成分を落としてから**正規化しないと
        基底が正規直交にならず、埋め込んだリンクの長さが縮む。

        e1 = 膝→股 を回転面に落として正規化、e2 = 膝軸 × e1。
        股 3 軸を 0 に保つこのツールでは膝軸は x̂ で固定。
        """
        axis = np.array([1.0, 0.0, 0.0])                   # 膝軸（Σ_0）
        e1 = o[0] - o[1]                                   # 膝 o4 -> 股 o3
        e1 = e1 - float(e1 @ axis) * axis                  # 回転面に落とす
        e1 = e1 / np.linalg.norm(e1)
        return e1, np.cross(axis, e1)                      # |e2| = 1（直交かつ単位）

    def _shank_angle(self, bend: float) -> float:
        """矢状面での下腿の向き（膝→足首）の角度 [rad]。"""
        th = np.zeros(6)
        th[3] = leg_angle_from_knee_bend(bend, self.leg)
        o = leg_ik.joint_origins(th, self.leg)
        e1, e2 = self._plane_basis(o)
        v = o[2] - o[1]                                    # 膝 o4 -> 足首 o5
        return math.atan2(float(v @ e2), float(v @ e1))

    def _solve_e2_sign(self) -> float:
        """下腿とロッカーが同じ向きに回るように e2 の符号を決める。

        ロッカーは下腿に固定されているので、両者は必ず同じ向き・同じ量だけ回る。
        表示でそこが食い違うと、膝を曲げたときにリンクだけ逆に動いて見える。
        取り付け向きは寸法からは決まらないが、**回る向き**は決まるのでここで揃える。
        """
        a0 = self._shank_angle(0.0)
        a1 = self._shank_angle(R(30.0))
        d_shank = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        d_rocker = self.link.sigma_joint * R(30.0)         # θ4 の変化
        return 1.0 if d_shank * d_rocker > 0.0 else -1.0

    def zero_to(self, raw: float) -> None:
        with self.lock:
            self.raw0 = float(raw)

    def phi0_deg(self) -> float | None:
        """求まったサーボ原点 φ0 = φ(raw0) − σ_m·n·θ2_ext [deg]。"""
        if self.raw0 is None:
            return None
        return wrap180(self.raw0 * 360.0 / COUNTS - self.sigma_m * self.gear * D(self.theta2_ext))

    def solve(self, sample: dict) -> dict:
        """1 サンプルぶんの変換。失敗しても例外にせず status で返す。"""
        out: dict = {}
        raw = sample.get('raw')
        if raw is None:
            return {'status': 'NoData'}
        with self.lock:
            if self.raw0 is None:
                self.raw0 = float(raw)
            raw0 = self.raw0

        dphi = wrap180((float(raw) - raw0) * 360.0 / COUNTS)
        theta2 = self.theta2_ext + R(dphi) / (self.sigma_m * self.gear)
        out['dphi'] = round(dphi, 3)
        out['theta2'] = round(D(theta2), 3)

        try:
            pose = self.link.fk(theta2)
        except knee_fourbar.Unreachable as exc:
            out['status'] = 'Unreachable'
            out['detail'] = str(exc)
            return out

        bend = self.link.joint_from_rocker(pose.theta4)
        try:
            ratio = self.link.ratio(pose)
        except knee_fourbar.DeadPoint:
            ratio = None
            out['status'] = 'DeadPoint'
        out.setdefault('status', 'Ok')
        out.update({
            'theta3': round(D(pose.theta3), 3),
            'theta4': round(D(pose.theta4), 3),
            'bend': round(D(bend), 3),
            'ratio': None if ratio is None else round(ratio, 4),
            'gamma': round(D(self.link.transmission_angle(pose)), 3),
            'loop_res': round(pose.loop_residual(self.link.r3), 12),
        })

        # 脚の関節位置（Σ_0、mm）。膝だけを曲げて他は 0
        th = np.zeros(6)
        th[3] = leg_angle_from_knee_bend(bend, self.leg)
        o = leg_ik.joint_origins(th, self.leg)
        out['leg'] = [to_body(p) for p in o]
        out['knee_angle_pub'] = round(D(th[3]), 3)

        # 4 節リンクを矢状面に置く（原点 = 膝 o4）
        e1, e2 = self._plane_basis(o)
        e2 = self._e2_sign * e2
        knee = o[1]

        def to3d(p) -> list[float]:
            return to_body(knee + p[0] * e1 + p[1] * e2)

        L = self.link
        out['link'] = {
            'O4': to3d((0.0, 0.0)), 'O2': to3d((L.r1, 0.0)),
            'A': to3d(pose.A), 'B': to3d(pose.B),
            'r': [L.r1, L.r2, L.r3, L.r4],
        }
        self.last_good = out
        return out


# --------------------------------------------------------------------------
# HTTP
# --------------------------------------------------------------------------
SRC = None
MODEL: KneeModel | None = None
ARGS = None


def api_state() -> dict:
    s = SRC.read()
    st = MODEL.solve(s)
    return {
        'servo': {k: s.get(k) for k in
                  ('ok', 't', 'raw', 'deg', 'volt', 'temp', 'load', 'n', 'miss', 'dead')},
        'stderr': s.get('stderr', []),
        'demo': bool(s.get('demo')),
        'source': SRC.label,
        'kin': st,
        'params': {
            'sigma_m': MODEL.sigma_m, 'gear': MODEL.gear,
            'beta': MODEL.link.beta, 'eps': MODEL.link.eps,
            'sigma_knee': MODEL.link.sigma_joint,
            'side': MODEL.side,
            'theta4_zero': round(D(MODEL.link.theta4_zero), 4),
            'theta2_ext': round(D(MODEL.theta2_ext), 4),
            'raw0': MODEL.raw0,
            'phi0_deg': None if MODEL.phi0_deg() is None else round(MODEL.phi0_deg(), 4),
        },
    }


def api_set(q) -> dict:
    """枝・サーボ向き・ギア比を実機に合わせて切り替える（原点出しの一部）。"""
    def geti(key, allowed):
        v = q.get(key, [None])[0]
        if v is None:
            return None
        try:
            iv = int(v)
        except ValueError:
            return None
        return iv if iv in allowed else None

    changed = []
    sm = geti('sigma_m', (+1, -1))
    if sm is not None and sm != MODEL.sigma_m:
        MODEL.sigma_m = sm
        changed.append('sigma_m')
    for key in ('beta', 'eps'):
        v = geti(key, (+1, -1))
        if v is not None and v != getattr(MODEL.link, key):
            MODEL.link = knee_fourbar.KneeFourBar(**{
                **{f: getattr(MODEL.link, f) for f in
                   ('r1', 'r2', 'r3', 'r4', 'beta', 'eps', 'theta4_zero',
                    'sigma_joint', 'phi0', 'sigma_motor', 'gear')},
                key: v})
            changed.append(key)
    g = q.get('gear', [None])[0]
    if g is not None:
        try:
            gv = float(g)
            if gv != 0.0 and gv != MODEL.gear:
                MODEL.gear = gv
                changed.append('gear')
        except ValueError:
            pass
    return {'ok': True, 'changed': changed}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):        # アクセスログは出さない
        pass

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        try:
            if u.path in ('/', '/index.html'):
                self._send(200, PAGE.read_bytes(), 'text/html; charset=utf-8')
            elif u.path == '/api/state':
                self._send(200, json.dumps(api_state()).encode(), 'application/json')
            elif u.path == '/api/zero':
                s = SRC.read()
                if s.get('raw') is None:
                    self._send(200, json.dumps(
                        {'ok': False, 'msg': 'サーボがまだ読めていない'}).encode(),
                        'application/json')
                    return
                MODEL.zero_to(s['raw'])
                self._send(200, json.dumps(
                    {'ok': True, 'raw0': MODEL.raw0,
                     'phi0_deg': round(MODEL.phi0_deg(), 4)}).encode(), 'application/json')
            elif u.path == '/api/set':
                self._send(200, json.dumps(api_set(q)).encode(), 'application/json')
            else:
                self._send(404, b'not found', 'text/plain')
        except BrokenPipeError:
            pass
        except Exception as exc:                                   # noqa: BLE001
            self._send(500, json.dumps({'error': str(exc)}).encode(), 'application/json')


def lan_addr() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))
        a = s.getsockname()[0]
        s.close()
        return a
    except OSError:
        return '127.0.0.1'


def main() -> int:
    global SRC, MODEL, ARGS
    ap = argparse.ArgumentParser(description='膝 4 節リンクの動作確認ビジュアライザ')
    ap.add_argument('--side', default='right', choices=('right', 'left'),
                    help='どちらの膝か。**膝サーボは左右とも ID 4** なので、'
                         'これがバスの選択（= 左右の選択）になる')
    ap.add_argument('--port', default=None,
                    help='サーボのシリアルポート（既定は --side から決まる）')
    ap.add_argument('--id', type=int, default=None,
                    help=f'膝サーボの ID（既定 {kcfg.SERVO_ID}。左右とも同じ）')
    ap.add_argument('--rate', type=float, default=50.0, help='サーボ読み取り [Hz]')
    ap.add_argument('--http-port', type=int, default=8102)
    ap.add_argument('--sigma-m', type=int, default=None, choices=(1, -1),
                    help='サーボの回転方向 σ_m（既定は knee_config の左右別の値。'
                         '画面でも切り替えられる）')
    ap.add_argument('--gear', type=float, default=None, help='ギア比 n')
    ap.add_argument('--demo', action='store_true', help='実機なしで表示だけ確かめる')
    ARGS = ap.parse_args()

    port = ARGS.port or kcfg.SERVO_PORT[ARGS.side]
    sid = kcfg.SERVO_ID if ARGS.id is None else ARGS.id
    MODEL = KneeModel(side=ARGS.side, sigma_m=ARGS.sigma_m, gear=ARGS.gear)
    if ARGS.demo:
        SRC = DemoSource(MODEL.link, raw0=2048, gear=MODEL.gear, sigma_m=MODEL.sigma_m)
        MODEL.zero_to(2048)      # 模擬の伸び切り姿勢。実機では最初のサンプルで取る
        print('*** DEMO モード: サーボは読んでいない。表示の確認用。')
    else:
        SRC = ServoSource(find_stream(), port, sid, ARGS.rate)
        print(f'{ARGS.side} 膝  サーボ: {SRC.label}  トルクは入れない（読むだけ）')

    srv = ThreadingHTTPServer(('0.0.0.0', ARGS.http_port), Handler)
    print(f'\n  http://{lan_addr()}:{ARGS.http_port}/   をブラウザで開く')
    print(f'  http://localhost:{ARGS.http_port}/        （SSH -L で転送した場合）\n')
    print('起動時の姿勢を「膝が伸び切った状態」として扱う。')
    print('脚を伸ばしてから開き、ずれていたら画面の「ゼロ取り直し」を押す。')
    print('Ctrl-C で終了。')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n終了')
    finally:
        SRC.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
