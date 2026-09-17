# -*- coding: utf-8 -*-
"""歩行計画の可視化 HTML を生成する CLI。

使い方 (ワークスペース直下から):

    python3 src/roboone_viz/roboone_viz/gen_walk_viz.py \
        --out ~/walk_viz/walk_viz.html

    # 任意の指令でシナリオを 1 本追加
    python3 .../gen_walk_viz.py --vx 0.12 --vy -0.04 --out ...

    # 生成してそのまま配信 (ssh 先の PC のブラウザで見る)
    python3 .../gen_walk_viz.py --serve 8100

    # 静歩行の設定を変えて見る (static_gait.yaml の上に重ねる。yaml は書き換えない)
    python3 .../gen_walk_viz.py --serve 8100 --static-com-offset 0.01 --static-swing-height 0.02

roboone_kinematics の leg_service がビルドしてあれば、記録シナリオの各時刻の足先が
実機の脚で届くかも調べて画面に出す (--no-reach で省く)。

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
import sys

# walk_core は別パッケージ roboone_walk_ref (仕様原本) にある。ソース木から
# 直接叩くとき用に、兄弟パッケージのパスを record より先に通しておく。
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'roboone_walk_ref'))

if __package__ in (None, ''):
    # colcon を通さず直接実行されたとき用
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from roboone_viz.reach import LegReach
    from roboone_viz.record import (Scenario, build_dataset,
                                    default_scenarios)
else:
    from .reach import LegReach
    from .record import Scenario, build_dataset, default_scenarios

from roboone_walk_ref.static_walk import StaticGaitParams   # noqa: E402
from roboone_walk_ref.walk_core import GaitParams   # noqa: E402


def _config_candidates(name):
    """roboone_walk_ref の config/<name> の置き場の候補。ソース木 → install の share の順。"""
    here = Path(__file__).resolve()
    yield here.parents[2] / 'roboone_walk_ref' / 'config' / name
    try:
        from ament_index_python.packages import get_package_share_directory
        yield Path(get_package_share_directory('roboone_walk_ref')) / 'config' / name
    except Exception:
        pass


TEMPLATE = Path(__file__).with_name('template.html')
WALKCORE_JS = Path(__file__).with_name('walkcore.js')
STATIC_JS = Path(__file__).with_name('staticwalk.js')
MARKER = '/*__WALK_DATA__*/null'
MARKER_JS = '/*__WALK_CORE_JS__*/'
MARKER_STATIC_JS = '/*__STATIC_WALK_JS__*/'


def generate(out_path: Path, params: GaitParams, extra=None,
             sparams: StaticGaitParams = None, reach=None) -> Path:
    scenarios = default_scenarios()
    if extra is not None:
        scenarios.append(extra)
    data = build_dataset(scenarios, params, sparams, reach)
    for e in data['static']['errors']:
        print(f'  ★静歩行の設定: {e}')
    html = TEMPLATE.read_text(encoding='utf-8')
    assert MARKER in html, 'template.html のデータ差し込み位置が見つからない'
    assert MARKER_JS in html, 'template.html の walkcore.js 差し込み位置が見つからない'
    assert MARKER_STATIC_JS in html, 'template.html の staticwalk.js 差し込み位置が見つからない'
    payload = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
    html = html.replace(MARKER_JS, WALKCORE_JS.read_text(encoding='utf-8'))
    html = html.replace(MARKER_STATIC_JS, STATIC_JS.read_text(encoding='utf-8'))
    html = html.replace(MARKER, payload)
    if reach is not None:
        for sc in data['scenarios']:
            n = len(sc['ticks']['rk'])
            mark = '  ★届かない時刻あり' if sc['reach_bad'] else ''
            print(f"  到達 {sc['id']:10s} {sc['reach_bad']:4d} / {n} 点で脚が届かない{mark}")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(html, encoding='utf-8')
    return out_path


def serve(directory: Path, port: int):
    class Handler(http.server.SimpleHTTPRequestHandler):

        def __init__(self, *a, **kw):
            super().__init__(*a, directory=str(directory), **kw)

        def log_message(self, fmt, *args):
            pass

    # 1 接続ずつしか捌かないサーバーだと、ブラウザが先に開く空の接続 (preconnect) を
    # 読み待ちしたまま固まり、本物の要求が受け付けられずページが出ない
    # (2026-09-17 VSCode のポート転送越しに再現)。接続ごとにスレッドを立てる。
    with http.server.ThreadingHTTPServer(('0.0.0.0', port), Handler) as httpd:
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
                    help='gait.yaml のパス (省略時は roboone_walk_ref の config/gait.yaml)')
    ap.add_argument('--vx', type=float, default=None, help='追加シナリオの vx [m/s]')
    ap.add_argument('--vy', type=float, default=None, help='追加シナリオの vy [m/s]')
    ap.add_argument('--duration', type=float, default=8.0, help='追加シナリオの長さ [s]')
    ap.add_argument('--serve', type=int, default=None, metavar='PORT',
                    help='生成後にそのディレクトリを HTTP 配信する')
    g = ap.add_argument_group('静歩行 (static_gait.yaml の上に重ねる。yaml は書き換えない)')
    g.add_argument('--static-gait', default=None,
                   help='static_gait.yaml のパス (省略時は roboone_walk_ref の config/static_gait.yaml)')
    g.add_argument('--static-zmp-tol', type=float, default=None,
                   help='ZMP が重心の真下からずれてよい量 [m]')
    g.add_argument('--static-t-swing', type=float, default=None, help='振り出しの時間 [s]')
    g.add_argument('--static-com-offset', type=float, default=None,
                   help='振り出し中の重心を支持足の中心から横へずらす量 [m] (+ で外側)')
    g.add_argument('--static-swing-height', type=float, default=None,
                   help='足上げの高さ [m]')
    ap.add_argument('--no-reach', action='store_true',
                    help='leg_service で足先が届くかを調べない')
    args = ap.parse_args(argv)

    # gait.yaml / static_gait.yaml の原本は roboone_walk_ref が持つ。ソース木と install の両方を見る。
    gait = args.gait or next((c for c in _config_candidates('gait.yaml') if c.exists()), None)
    params = GaitParams.from_yaml(str(gait)) if gait else GaitParams()
    sgait = args.static_gait or next(
        (c for c in _config_candidates('static_gait.yaml') if c.exists()), None)
    sparams = StaticGaitParams.from_yaml(str(sgait)) if sgait else StaticGaitParams()
    for key, val in (('zmp_tol', args.static_zmp_tol), ('t_swing', args.static_t_swing),
                     ('com_offset_y', args.static_com_offset),
                     ('swing_height', args.static_swing_height)):
        if val is not None:
            setattr(sparams, key, val)

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

    reach = None if args.no_reach else LegReach.find()
    if reach is None and not args.no_reach:
        print('leg_service が見つからないので、足先が届くかは調べない '
              '(colcon build --packages-select roboone_kinematics)')
    try:
        out = generate(Path(os.path.expanduser(args.out)), params, extra, sparams, reach)
    finally:
        if reach is not None:
            reach.close()
    size = out.stat().st_size / 1024
    print(f'生成した: {out}  ({size:.0f} kB)')
    if args.serve:
        serve(out.parent, args.serve)


if __name__ == '__main__':
    main()
