# -*- coding: utf-8 -*-
"""両脚 3D ビジュアライザ。実機のサーボ角を読んで、重心起点の姿勢をそのまま描く。

    # 実機（サーボ電源を入れてから）
    python3 src/roboone_motion/roboone_motion/viz/serve_legs3d.py

    # 実機なしで表示だけ確かめる
    python3 src/roboone_motion/roboone_motion/viz/serve_legs3d.py --demo

ブラウザで http://<Pi の eth0 アドレス>:8103/ を開く。

===========================================================================
やること・やらないこと
===========================================================================
* サーボは **角度を読むだけ**。トルクは入れない。子プロセスの
  ``feetech_leg_stream`` が起動時に 1 回だけトルク OFF（脱力）を書き、
  それ以外の書き込み経路を持たない。脚は手で動かす。
* パラレルリンク（膝 4 節・足首 2 鎖）は **抽象化して見せない**。画面に描くのは
  重心（骨盤原点）から θ1..θ6 の回転で繋がるリンクと足裏だけ。機構の中身は
  サーボ角 -> θ の変換の中にしまってある。
* 表示している姿勢はすべて C++ の ``leg_servo.hpp`` / ``leg_kinematics.hpp`` が
  計算した値。可視化のために書き直した別実装は無い（子プロセス ``leg_service``）。

===========================================================================
原点（初期姿勢）の定義
===========================================================================
``feetech_servo/config/servo_home.yaml`` に入っている軸ごとの生カウントを
**「脚が真下に伸びた姿勢」** と定義する。つまり全 12 軸がその値のとき

    θ1..θ6 = 0、足裏中心 = (0, ∓89.3, -299.945) [mm]（股中心の真下）

になる。起動時の姿勢は基準にしないので、脚が曲がったまま起動しても原点はずれない。
起動時にこの対応を 1 回計算して表示するので、ずれていればそこで分かる。

実機の姿勢を真下に伸ばしても表示が 0 にならない場合は、servo_home.yaml の値が
その軸について古い（``feetech_calibrate_home`` を回し直す）。画面の
「今を T ポーズに」はこのプロセスの中だけの一時上書きで、ファイルは書き換えない。

サーボの回転方向が Σ_B の正方向と逆な軸は ``leg_config.hpp`` の AXIS_FLIP /
AXIS_FLIP_LEFT で反転する（左脚 ID1 は反転済み。2026-08-28 実機で確認）。

===========================================================================
サーボ角 -> 関節角の経路
===========================================================================
生カウント -> T ポーズ（servo_home.yaml の原点）からの差分 [deg]
  -> leg_service の ``servo`` コマンド
       J1-J3  そのまま関節角（サーボ直結）
       J4     4 節リンクの順変換 (KN-5) -> 曲げ量 -> θ4
       J5・J6 パラレルリンクの順変換 (AP-15、1 変数ニュートン法) -> θ5, θ6
  -> jointOrigins() で関節位置

ID と関節の対応は docs/servo-registers.md（2026-08-28 実機で確定）。
足首は ``ankle_parallel.hpp`` の鎖の添字に合わせて **ID の並びが 6, 5** になる。
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.parse import parse_qs, urlparse

_HERE = Path(__file__).resolve().parent
_WS = _HERE.parents[3]

PAGE = _HERE / 'legs3d.html'
COUNTS = 4096.0

#: 読む ID の並び。ankle_parallel の鎖 0 = ID6（短ロッド）、鎖 1 = ID5（長ロッド）
IDS = [1, 2, 3, 4, 6, 5]
PORTS = {'R': '/dev/feetech_right', 'L': '/dev/feetech_left'}


def wrap180(a: float) -> float:
    """角を (-180, 180] に畳む [deg]。"""
    return (a + 180.0) % 360.0 - 180.0


def find_exe(pkg: str, name: str) -> Path:
    for c in (_WS / 'build' / pkg / name,
              _WS / 'install' / pkg / 'lib' / pkg / name):
        if c.is_file() and os.access(c, os.X_OK):
            return c
    raise SystemExit(
        f'{name} が見つからない。先に\n'
        f'  colcon build --packages-select {pkg}\n'
        'を実行する。')


def read_home() -> tuple[dict, str]:
    """servo_home.yaml から {side: {id: home count}} を読む。"""
    path = _WS / 'src' / 'feetech_servo' / 'config' / 'servo_home.yaml'
    out = {'R': {}, 'L': {}}
    try:
        import yaml
        doc = yaml.safe_load(path.read_text())
        by_port = {b.get('port'): (b.get('servos') or {}) for b in doc.get('buses', [])}
        missing = []
        for side, port in PORTS.items():
            servos = by_port.get(port, {})
            for sid in IDS:
                ent = servos.get(sid)
                if ent is None:
                    missing.append(f'{side}:ID{sid}')
                else:
                    out[side][sid] = float(ent['home'])
        note = ('servo_home.yaml に無い軸: ' + ', '.join(missing)) if missing else ''
        return out, note
    except Exception as exc:                                   # noqa: BLE001
        return out, f'{path.name} が読めない（{exc}）'


class LegService:
    """leg_service 子プロセス。要求は 1 本ずつ直列に流す。"""

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


class ServoReader(threading.Thread):
    """feetech_leg_stream の出力を読み続けて、最新の生カウントを持つ。

    欠損（低電圧などで応答が来なかった軸）は前の値を保持する。
    """

    daemon = True

    def __init__(self, cmd: list[str] | None, demo: bool):
        super().__init__()
        self.demo = demo
        self.cmd = cmd
        self.lock = threading.Lock()
        self.raw = {'R': {}, 'L': {}}
        self.health = {'R': {}, 'L': {}}
        self.t = 0.0
        self.n = 0
        self.alive = False
        self.error = ''
        self.proc: subprocess.Popen | None = None

    def run(self):
        if self.demo:
            self._run_demo()
            return
        try:
            self.proc = subprocess.Popen(
                self.cmd, stdout=subprocess.PIPE, stderr=None, text=True, bufsize=1)
        except OSError as exc:
            self.error = f'{self.cmd[0]} を起動できない（{exc}）'
            return
        self.alive = True
        for line in self.proc.stdout:
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            with self.lock:
                self.t = d.get('t', 0.0)
                self.n = d.get('n', 0)
                for side in ('R', 'L'):
                    leg = d.get(side)
                    if not leg:
                        continue
                    raws, valid = leg.get('raw', []), leg.get('valid', [])
                    for j, sid in enumerate(IDS):
                        if j < len(valid) and valid[j]:
                            self.raw[side][sid] = float(raws[j])
                    self.health[side] = {
                        'volt': leg.get('volt'), 'temp': leg.get('temp'),
                        'miss': leg.get('miss'), 'nvalid': leg.get('nvalid'),
                    }
        self.alive = False
        self.error = self.error or 'feetech_leg_stream が終了した'

    def _run_demo(self):
        """実機なしの確認用。ゆっくり全軸を振る。"""
        self.alive = True
        home, _ = read_home()
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            with self.lock:
                self.t = t
                self.n += 1
                for side in ('R', 'L'):
                    s = 1.0 if side == 'R' else -1.0
                    for j, sid in enumerate(IDS):
                        amp = (10.0, 8.0, 6.0, 25.0, 6.0, 6.0)[j]
                        ang = amp * math.sin(2.0 * math.pi * (t / 9.0) + j * 0.7) * s
                        self.raw[side][sid] = home[side].get(sid, 2048.0) + ang * COUNTS / 360.0
                    self.health[side] = {'volt': 11.8, 'temp': 32, 'miss': 0,
                                         'nvalid': len(IDS)}
            time.sleep(0.02)

    def snapshot(self) -> tuple[dict, dict, float, int, bool, str]:
        with self.lock:
            return ({k: dict(v) for k, v in self.raw.items()},
                    {k: dict(v) for k, v in self.health.items()},
                    self.t, self.n, self.alive, self.error)


# ---------------------------------------------------------------------------
# 状態
# ---------------------------------------------------------------------------
SVC: LegService | None = None
READER: ServoReader | None = None
HOME: dict = {'R': {}, 'L': {}}
HOME_NOTE = ''
HOME_SRC = 'servo_home.yaml'
FLIP = {'R': [0] * 6, 'L': [0] * 6}  # leg_config.hpp の AXIS_FLIP（表示用）
SEED = {'R': 0.0, 'L': 0.0}         # 足首順変換に渡す前周期の θ6 [deg]
LAST_OK = {'R': None, 'L': None}    # 最後に解けた姿勢（解けない間はこれを描く）
LOCK = threading.Lock()


def build_frame() -> dict:
    raw, health, t, n, alive, error = READER.snapshot()
    out = {'ok': 1, 't': round(t, 3), 'n': n, 'alive': alive, 'error': error,
           'home_src': HOME_SRC, 'home_note': HOME_NOTE, 'flip': FLIP, 'legs': {}}
    for side in ('R', 'L'):
        leg = {'health': health.get(side, {})}
        counts = raw.get(side, {})
        homes = HOME.get(side, {})
        if not counts or not homes:
            leg['status'] = 'no_data'
            leg['raw'] = {str(k): counts.get(k) for k in IDS}
            out['legs'][side] = leg
            continue

        # 原点（servo_home.yaml）からの差。全軸 0 = 脚が真下に伸びた姿勢。
        # サーボの回転向きの反転は leg_config.hpp の AXIS_FLIP が受け持つので、
        # ここでは触らない（二重に反転しないため）。
        #
        # 一度も読めていない軸を 0 として流すと「原点にいる」と区別が付かないので、
        # have で持ち上げて呼び側に見せる（電源断や低電圧はここに出る）。
        d, have = [], []
        for sid in IDS:
            c, h = counts.get(sid), homes.get(sid)
            ok = (c is not None and h is not None)
            have.append(ok)
            d.append(wrap180((c - h) * 360.0 / COUNTS) if ok else 0.0)

        with LOCK:
            seed = SEED[side]
        res = SVC.call('servo %s %s %.5f' % (
            side, ' '.join('%.5f' % v for v in d), seed))
        # 足首の順変換は 1 変数ニュートン法なので、発散した値を次の種にすると
        # 姿勢が戻っても復帰しなくなる。解けたときだけ種を更新する。
        if res.get('status') == 'ok' and res.get('theta'):
            with LOCK:
                SEED[side] = res['theta'][5]
                LAST_OK[side] = res

        missing = [IDS[j] for j, ok in enumerate(have) if not ok]
        # 解けなかったときの θ・関節位置は当てにならないので描かない。
        # 直前に解けた姿勢を stale として出し、画面ではその旨を出す。
        draw = res if res.get('status') == 'ok' else (LAST_OK[side] or res)
        leg.update({
            'status': res.get('status'),
            'stale': res.get('status') != 'ok',
            'have': have,
            'missing': missing,
            'raw': {str(k): counts.get(k) for k in IDS},
            'delta': [round(v, 3) for v in d],
            'theta': [round(v, 3) for v in (res.get('theta') or [])],
            'origins': draw.get('origins'),
            'foot': draw.get('foot'),
            'R': draw.get('R'),
            'bend': res.get('bend'),
            'theta2': res.get('theta2'),
            'q': res.get('q'),
        })
        out['legs'][side] = leg
    return out


def api_zero() -> dict:
    """今の姿勢を T ポーズ基準にする（このプロセスの中だけ・一時）。"""
    global HOME, HOME_SRC
    raw, _, _, _, _, _ = READER.snapshot()
    if not raw['R'] and not raw['L']:
        return {'ok': 0, 'error': 'まだサーボを読めていない'}
    HOME = {side: dict(raw.get(side, {})) for side in ('R', 'L')}
    HOME_SRC = '今の姿勢（一時）'
    return {'ok': 1, 'home_src': HOME_SRC}


def api_home() -> dict:
    """servo_home.yaml の原点に戻す。"""
    global HOME, HOME_SRC, HOME_NOTE
    HOME, HOME_NOTE = read_home()
    HOME_SRC = 'servo_home.yaml'
    return {'ok': 1, 'home_src': HOME_SRC, 'home_note': HOME_NOTE}


class Handler(BaseHTTPRequestHandler):
    server_version = 'legs3d'

    def log_message(self, fmt, *args):
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
            if u.path == '/favicon.ico':
                self._send(b'', 'image/x-icon', 204)
            elif not u.path.startswith('/api/'):
                # /api/ 以外はすべてページを返す。パスを付けて開いても
                # not found にならないようにしておく。
                self._send(PAGE.read_bytes(), 'text/html; charset=utf-8')
            elif u.path == '/api/state':
                self._json(build_frame())
            elif u.path == '/api/params':
                self._json(SVC.call('params'))
            elif u.path == '/api/zero':
                self._json(api_zero())
            elif u.path == '/api/home':
                self._json(api_home())
            else:
                self._json({'ok': 0, 'error': '知らない API: %s' % u.path}, 404)
        except Exception as exc:                                # noqa: BLE001
            self._json({'ok': 0, 'error': '%s: %s' % (type(exc).__name__, exc)}, 500)


def kill_stale_streams(exe: Path) -> list[int]:
    """argv[0] が exe と一致するプロセスだけを止める。戻り値は止めた PID。"""
    killed = []
    me = os.getpid()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == me:
            continue
        try:
            argv0 = (entry / 'cmdline').read_bytes().split(b'\0')[0].decode()
        except OSError:
            continue
        if argv0 and Path(argv0).resolve() == exe.resolve():
            try:
                os.kill(pid, signal.SIGTERM)
                killed.append(pid)
            except (ProcessLookupError, PermissionError):
                pass
    return killed


def local_addrs():
    out = []
    ssh = os.environ.get('SSH_CONNECTION', '').split()
    if len(ssh) >= 3:
        out.append(('ssh', ssh[2]))
    try:
        raw = subprocess.run(['ip', '-4', '-o', 'addr', 'show'],
                             capture_output=True, text=True).stdout
        for line in raw.splitlines():
            f = line.split()
            if len(f) > 3 and f[1] != 'lo':
                addr = f[3].split('/')[0]
                if addr not in [a for _, a in out]:
                    out.append((f[1], addr))
    except OSError:
        pass
    return out


def main() -> int:
    global SVC, READER, HOME, HOME_NOTE
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', type=int, default=8103)
    ap.add_argument('--bind', default='0.0.0.0')
    ap.add_argument('--rate', type=float, default=50.0, help='サーボ読み取り [Hz]')
    ap.add_argument('--only', default='both', choices=('both', 'right', 'left'))
    ap.add_argument('--demo', action='store_true', help='実機なしで表示だけ確かめる')
    ap.add_argument('--keep-torque', action='store_true',
                    help='起動時のトルク OFF も行わない（読むだけなのは変わらない）')
    args = ap.parse_args()

    SVC = LegService(find_exe('roboone_kinematics', 'leg_service'))
    HOME, HOME_NOTE = read_home()
    if HOME_NOTE:
        print('注意:', HOME_NOTE)

    # 原点の定義をここで 1 回確かめて出す（全軸 servo_home.yaml の値 = 真下）
    prm = SVC.call('params')
    for side in ('R', 'L'):
        FLIP[side] = prm.get('flipR' if side == 'R' else 'flipL', [0] * 6)
        z = SVC.call('servo %s 0 0 0 0 0 0' % side)
        sole = z.get('origins', [[0, 0, 0]] * 5)[4]
        print('原点 %s: θ = %s / 足裏 = (%.1f, %.1f, %.1f) mm  AXIS_FLIP %s'
              % (side, ' '.join('%.1f' % v for v in z['theta']),
                 sole[0], sole[1], sole[2], FLIP[side]))
    print('  ↑ servo_home.yaml の生カウント = 脚が真下に伸びた姿勢、という定義')

    cmd = None
    if not args.demo:
        exe = find_exe('feetech_servo', 'feetech_leg_stream')
        # 前の起動の取り残しがシリアルを掴んでいると読み値が壊れる。先に止める。
        # ※ argv[0] が実行ファイルそのものの場合だけ。文字列一致で探すと、
        #    たまたま同じ語を含むシェルまで巻き添えにする。
        stale = kill_stale_streams(exe)
        if stale:
            print('取り残しの feetech_leg_stream を停止した: %s'
                  % ', '.join(str(x) for x in stale))
            time.sleep(0.5)
        cmd = [str(exe), '--rate', str(args.rate), '--only', args.only,
               '--ids', ','.join(str(i) for i in IDS)]
        if args.keep_torque:
            cmd.append('--keep-torque')
    READER = ServoReader(cmd, args.demo)
    READER.start()
    time.sleep(0.6)

    def shutdown(_sig, _frm):
        """kill されても子プロセスを道連れにする（シリアルを掴んだまま残さない）。"""
        if READER.proc:
            READER.proc.terminate()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    print('=' * 66)
    print('両脚 3D ビジュアライザ（サーボは読むだけ・トルクは入れない）')
    for name, addr in local_addrs():
        print('   %-6s http://%s:%d/' % (name, addr, args.port))
    if args.demo:
        print('   --demo: 実機は読んでいない')
    print('   停止は Ctrl-C')
    print('=' * 66, flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n停止')
    finally:
        if READER.proc:
            READER.proc.terminate()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
