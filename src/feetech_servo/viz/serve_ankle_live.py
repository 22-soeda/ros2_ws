#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""足首パラレルリンクの実機姿勢を 3D で見るテストラン。

    python3 src/feetech_servo/viz/serve_ankle_live.py

有線 LAN 越しに PC のブラウザから http://<Pi の IP>:8102/ で開く。
SSH のポートフォワード (ssh -L 8102:localhost:8102 <pi>) でも同じ。

**このサーバはサーボに書き込まない。** leg_live_test を読み取り専用で起動し、
出てくる JSON をそのままブラウザへ中継するだけ。関節が固くて手で動かせないときは
--relax を付けると起動時に 1 回だけトルクを切る（入れることはしない）。

★画面に出ている点はすべて C++ の ankle_parallel.hpp が計算したもの。
  ブラウザ側は送られてきた点を線で結んでいるだけで、可視化用に式を書き直していない。
"""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import pathlib
import queue
import shutil
import signal
import socket
import subprocess
import sys
import threading

HERE = pathlib.Path(__file__).resolve().parent
PAGE = HERE / 'ankle_live.html'

_latest: dict = {'frame': -1}
_subscribers: list[queue.Queue] = []
_lock = threading.Lock()
_proc: subprocess.Popen | None = None


def find_tool() -> list[str]:
    """leg_live_test の起動コマンドを決める。install 済みなら ros2 run を使う。"""
    if shutil.which('ros2'):
        return ['ros2', 'run', 'feetech_servo', 'leg_live_test']
    ws = HERE.parents[2]
    direct = ws / 'install' / 'feetech_servo' / 'lib' / 'feetech_servo' / 'leg_live_test'
    if direct.exists():
        return [str(direct)]
    sys.exit('leg_live_test が見つからない。先に colcon build すること')


def reader(cmd: list[str]) -> None:
    """leg_live_test の標準出力を読み、購読者へ配る。"""
    global _proc
    _proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=None,
                             text=True, bufsize=1)
    for line in _proc.stdout:                       # type: ignore[union-attr]
        line = line.strip()
        # 先頭に SDK が出す "serial speed 1000000" のような非 JSON 行が混じる
        if not line.startswith('{'):
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue
        with _lock:
            _latest.clear()
            _latest.update(msg)
            dead = []
            for q in _subscribers:
                try:
                    q.put_nowait(line)
                except queue.Full:
                    dead.append(q)               # 追いつけない購読者は切る
            for q in dead:
                _subscribers.remove(q)
    print('leg_live_test が終了した', file=sys.stderr)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):                      # アクセスログは出さない
        pass

    def _send(self, code, ctype, body: bytes):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self._send(200, 'text/html; charset=utf-8', PAGE.read_bytes())
            return
        if self.path == '/state':
            with _lock:
                body = json.dumps(_latest).encode()
            self._send(200, 'application/json', body)
            return
        if self.path == '/stream':
            q: queue.Queue = queue.Queue(maxsize=4)
            with _lock:
                _subscribers.append(q)
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while True:
                    line = q.get()
                    self.wfile.write(f'data: {line}\n\n'.encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with _lock:
                    if q in _subscribers:
                        _subscribers.remove(q)
            return
        self._send(404, 'text/plain; charset=utf-8', b'not found')


def access_urls(port: int) -> list:
    """ブラウザから開ける URL を、使えそうな順に並べて返す。

    既定経路 (8.8.8.8 への UDP) だけを見ると、Wi-Fi と有線が両方生きている機体で
    Wi-Fi 側のアドレスが出る。SSH で入っているならその接続が着いている側の
    アドレスが確実に届くので、**SSH_CONNECTION を最優先**にする。
    """
    urls, seen = [], set()

    def add(ip: str, note: str) -> None:
        if ip and ip not in seen:
            seen.add(ip)
            urls.append(f'http://{ip}:{port}/{note}')

    # 1) SSH で入ってきた先のアドレス（この経路は確実に通っている）
    conn = os.environ.get('SSH_CONNECTION', '').split()
    if len(conn) >= 3:
        add(conn[2], '   ← SSH と同じ経路')

    # 2) 既定経路のアドレス
    try:
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.connect(('8.8.8.8', 1))
        add(sk.getsockname()[0], '')
        sk.close()
    except OSError:
        pass

    add('127.0.0.1', f'   ← ssh -L {port}:localhost:{port} 越しならこちら')
    return urls


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    # 既定は実機で確定した対応（docs/servo-registers.md）。ID 5 が長ロッド側なので
    # 足首の 2 つは 6, 5 の順に並ぶ。ここを 5,6 にすると足首の 2 軸が入れ替わる。
    ap.add_argument('--ids', default='1,2,3,4,6,5',
                    help='股ピッチ,股ロール,股ヨー,膝,足首鎖1(短),足首鎖2(長) の軸 ID')
    ap.add_argument('--leg', default='L', choices=['L', 'R', 'l', 'r'])
    ap.add_argument('--hz', type=float, default=30.0)
    ap.add_argument('--relax', action='store_true',
                    help='起動時に 1 回だけトルクを切る（入れることはしない）')
    ap.add_argument('--port', type=int, default=8102)
    args = ap.parse_args()

    cmd = find_tool() + ['--json', '--ids', args.ids, '--leg', args.leg,
                         '--hz', str(args.hz)]
    if args.relax:
        cmd.append('--relax')

    threading.Thread(target=reader, args=(cmd,), daemon=True).start()

    srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    for url in access_urls(args.port):
        print(f'  {url}', file=sys.stderr)
    print('  （Ctrl-C で終了）', file=sys.stderr)

    def bye(*_a):
        if _proc is not None:
            _proc.terminate()
        os._exit(0)
    signal.signal(signal.SIGINT, bye)
    signal.signal(signal.SIGTERM, bye)
    srv.serve_forever()


if __name__ == '__main__':
    main()
