#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ament_flake8 / ament_pep257 と同じ設定で、パッケージを手元の venv で lint する。

    python tools/lint_like_ament.py                          # roboone_teleop
    python tools/lint_like_ament.py src/roboone_behavior src/roboone_ui

前提: tools/.venv に flake8 一式と pydocstyle が入っていること (tools/README.md)。
colcon test の test_flake8 / test_pep257 と同じ規約で見るので、ここで通れば
Pi の linter も通る (プラグインの版差で 1〜2 件ずれることはある)。
"""

import os
import pathlib
import subprocess
import sys

WS = pathlib.Path(__file__).resolve().parents[1]
VENV_PY = WS / 'tools' / '.venv' / ('Scripts/python.exe' if os.name == 'nt' else 'bin/python')

# ament_flake8 の既定 (パッケージに .flake8.ini が無いときだけ使う)
AMENT_FLAKE8_IGNORE = 'B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202'
# ament_pep257 の既定 ignore。パッケージの test_pep257.py の --add-ignore を足す
AMENT_PEP257_IGNORE = ['D100', 'D101', 'D102', 'D103', 'D104', 'D105', 'D106', 'D107',
                       'D203', 'D212', 'D404']


def _add_ignore_from_test(pkg: pathlib.Path):
    """test/test_pep257.py の ``--add-ignore`` を拾う。"""
    f = pkg / 'test' / 'test_pep257.py'
    if not f.exists():
        return []
    import re
    m = re.search(r"--add-ignore',\s*'([^']+)'", f.read_text(encoding='utf-8'))
    return m.group(1).split(',') if m else []


def lint(pkg: pathlib.Path) -> int:
    rc = 0
    cfg = pkg / '.flake8.ini'
    if cfg.exists():
        cmd = [str(VENV_PY), '-m', 'flake8', '--config', str(cfg), str(pkg)]
    else:
        cmd = [str(VENV_PY), '-m', 'flake8', '--max-line-length=99',
               '--import-order-style=google', f'--extend-ignore={AMENT_FLAKE8_IGNORE}', str(pkg)]
    print(f'== flake8  {pkg.relative_to(WS)}')
    rc |= subprocess.call(cmd, cwd=str(pkg))

    ignore = AMENT_PEP257_IGNORE + _add_ignore_from_test(pkg)
    cmd = [str(VENV_PY), '-m', 'pydocstyle', '--ignore', ','.join(ignore),
           '--match', r'.*\.py', str(pkg)]
    print(f'== pep257  {pkg.relative_to(WS)}  (ignore {",".join(ignore)})')
    rc |= subprocess.call(cmd, cwd=str(pkg))
    return rc


def main(argv):
    if not VENV_PY.exists():
        sys.exit(f'venv が無い: {VENV_PY}  (tools/README.md の手順で作る)')
    pkgs = [WS / p for p in (argv or ['src/roboone_teleop'])]
    rc = 0
    for pkg in pkgs:
        rc |= lint(pkg)
    print('OK' if rc == 0 else 'NG')
    return rc


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
