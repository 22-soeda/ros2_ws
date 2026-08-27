# -*- coding: utf-8 -*-
"""歩行計画の可視化 HTML を生成する CLI。

使い方 (ワークスペース直下から):

    python3 src/roboone_motion/roboone_motion/viz/gen_walk_viz.py \
        --out ~/walk_viz/walk_viz.html

    # 任意の指令でシナリオを 1 本追加
    python3 .../gen_walk_viz.py --vx 0.12 --vy -0.04 --out ...

    # 生成してそのまま配信 (ssh 先の PC のブラウザで見る)
    python3 .../gen_walk_viz.py --serve 8100

生成物は自己完結の 1 ファイルで、ブラウザで開くだけで動く (外部依存なし)。
SSH 接続の PC から見るには次のどちらか:
  * ポートフォワード:  ssh -L 8100:localhost:8100 <pi>  →  http://localhost:8100/
  * 同一 LAN なら直接:  http://<pi の IP>:8100/
"""

import argparse
import http.server
import json
import os
from pathlib import Path
import socketserver
import sys

if __package__ in (None, ''):
    # colcon を通さず直接実行されたとき用
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from roboone_motion.viz.record import (Scenario, build_dataset,
                                           default_scenarios)
    from roboone_motion.walk_core import GaitParams
else:
    from .record import Scenario, build_dataset, default_scenarios
    from ..walk_core import GaitParams

TEMPLATE = Path(__file__).with_name('template.html')
WALKCORE_JS = Path(__file__).with_name('walkcore.js')
MARKER = '/*__WALK_DATA__*/null'
MARKER_JS = '/*__WALK_CORE_JS__*/'


def generate(out_path: Path, params: GaitParams, extra=None) -> Path:
    scenarios = default_scenarios()
    if extra is not None:
        scenarios.append(extra)
    data = build_dataset(scenarios, params)
    html = TEMPLATE.read_text(encoding='utf-8')
    assert MARKER in html, 'template.html のデータ差し込み位置が見つからない'
    assert MARKER_JS in html, 'template.html の walkcore.js 差し込み位置が見つからない'
    payload = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
    html = html.replace(MARKER_JS, WALKCORE_JS.read_text(encoding='utf-8'))
    html = html.replace(MARKER, payload)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html, encoding='utf-8')
    return out_path


def serve(directory: Path, port: int):
    class Handler(http.server.SimpleHTTPRequestHandler):
        def __init__(self, *a, **kw):
            super().__init__(*a, directory=str(directory), **kw)

        def log_message(self, fmt, *args):
            pass

    with socketserver.TCPServer(('0.0.0.0', port), Handler) as httpd:
        print(f'配信中: http://0.0.0.0:{port}/walk_viz.html  (Ctrl-C で終了)')
        print(f'  SSH の PC からは  ssh -L {port}:localhost:{port} でトンネルして '
              f'http://localhost:{port}/walk_viz.html')
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass


def main(argv=None):
    ap = argparse.ArgumentParser(description='歩行計画の可視化 HTML を生成する')
    ap.add_argument('--out', default='~/walk_viz/walk_viz.html',
                    help='出力 HTML パス (既定: ~/walk_viz/walk_viz.html)')
    ap.add_argument('--gait', default=None,
                    help='gait.yaml のパス (省略時はパッケージの config/gait.yaml)')
    ap.add_argument('--vx', type=float, default=None, help='追加シナリオの vx [m/s]')
    ap.add_argument('--vy', type=float, default=None, help='追加シナリオの vy [m/s]')
    ap.add_argument('--duration', type=float, default=8.0, help='追加シナリオの長さ [s]')
    ap.add_argument('--serve', type=int, default=None, metavar='PORT',
                    help='生成後にそのディレクトリを HTTP 配信する')
    args = ap.parse_args(argv)

    gait = args.gait
    if gait is None:
        cand = Path(__file__).resolve().parents[2] / 'config' / 'gait.yaml'
        gait = cand if cand.exists() else None
    params = GaitParams.from_yaml(str(gait)) if gait else GaitParams()

    extra = None
    if args.vx is not None or args.vy is not None:
        vx = args.vx or 0.0
        vy = args.vy or 0.0
        walk_end = max(1.0, args.duration - 3.5)
        extra = Scenario(
            'custom', f'カスタム ({vx:+.2f}, {vy:+.2f})',
            f'vx={vx:+.2f}, vy={vy:+.2f} m/s を {walk_end - 0.5:.1f} s → 停止',
            args.duration,
            lambda t: (vx, vy) if 0.5 <= t < walk_end else (0.0, 0.0))

    out = generate(Path(os.path.expanduser(args.out)), params, extra)
    size = out.stat().st_size / 1024
    print(f'生成した: {out}  ({size:.0f} kB)')
    if args.serve:
        serve(out.parent, args.serve)


if __name__ == '__main__':
    main()
