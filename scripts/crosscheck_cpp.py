#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C++ 実装 (roboone_kinematics) と Python 参照実装 (leg_ik.py) の突き合わせ。

同じパラメータ・同じ姿勢に対して両者の FK と IK が一致することを確認する。
独立に書いた 2 つの実装を比べるので、片方だけの取り違えを拾える。

  python3 scripts/crosscheck_cpp.py [-n 姿勢数]

C++ 側は build/roboone_kinematics/leg_dump を使う (colcon build 済みであること)。
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import leg_ik  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.dirname(_HERE)

_STATUS = {0: "Ok", 1: "AnkleOutOfRange", 2: "KneeOutOfRange", 3: "NoBranch"}

#: (a3, a4, b, sigma, flipmask) の組。x 成分・膝の分岐・回転方向を混ぜる
CASES = [
    (0.0, 0.0, 0.0, +1, 0),
    (11.0, -4.0, 7.0, +1, 0),
    (-20.0, 27.0, 7.0, +1, 0),      # 和が同じで分け方だけ違う
    (-6.0, 2.5, -3.0, -1, 0),
    (9.0, -3.0, 4.0, +1, 0b101010),
    (9.0, -3.0, 4.0, -1, 0b010101),
    (3.5, 3.5, 0.0, +1, 0b111111),
]


def _find_dump() -> str:
    for path in (os.path.join(_WS, "build", "roboone_kinematics", "leg_dump"),
                 os.path.join(_WS, "install", "roboone_kinematics",
                              "lib", "roboone_kinematics", "leg_dump")):
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    raise SystemExit(
        "leg_dump が見つからない。先に "
        "`colcon build --packages-select roboone_kinematics` を実行する。")


def main() -> int:
    ap = argparse.ArgumentParser(description="C++ と Python 実装の突き合わせ")
    ap.add_argument("-n", type=int, default=5000, help="1 ケースあたりの姿勢数")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()

    dump = _find_dump()
    print("=" * 70)
    print("C++ (roboone_kinematics) vs Python (leg_ik.py)")
    print("=" * 70)
    print(f"  leg_dump: {os.path.relpath(dump, _WS)}")

    failures = 0
    for ci, (a3, a4, b, sigma, mask) in enumerate(CASES):
        flip = tuple((mask >> k) & 1 for k in range(6))
        prm = leg_ik.leg_params("right", a3=a3, a4=a4, b=b, sigma=sigma, flip=flip)

        raw = subprocess.run(
            [dump, repr(a3), repr(a4), repr(b), str(sigma), str(mask),
             str(args.n), str(args.seed + ci)],
            capture_output=True, text=True, check=True).stdout

        w_p = w_R = w_ik = 0.0
        n_status = 0
        rows = 0
        for line in raw.strip().splitlines():
            v = [float(x) for x in line.split(",")]
            th = np.array(v[0:6])
            p_cpp = np.array(v[6:9])
            R_cpp = np.array(v[9:18]).reshape(3, 3)
            ik_cpp = np.array(v[18:24])
            st = int(v[24])
            rows += 1

            # FK の照合
            p_py, R_py = leg_ik.fk(th, prm)
            w_p = max(w_p, float(np.max(np.abs(p_py - p_cpp))))
            w_R = max(w_R, float(np.max(np.abs(R_py - R_cpp))))

            # IK の照合（同じ枝を選んでいるか含めて）
            try:
                ik_py = leg_ik.ik(p_cpp, R_cpp, prm, clamp=False)
            except leg_ik.Unreachable:
                if st == 0:
                    n_status += 1
                continue
            if st != 0:
                n_status += 1
                continue
            d = np.abs(np.arctan2(np.sin(ik_py - ik_cpp), np.cos(ik_py - ik_cpp)))
            w_ik = max(w_ik, float(np.max(d)))

        ok = w_p < 1e-11 and w_R < 1e-13 and w_ik < 1e-9 and n_status == 0
        failures += 0 if ok else 1
        print(f"\n  [{ci}] a3={a3:g} a4={a4:g} (a={a3 + a4:g}) b={b:g} "
              f"σ={sigma:+d} flip={flip}  ({rows} 姿勢)")
        print(f"        FK 位置 {w_p:.2e} mm / 姿勢 {w_R:.2e}")
        print(f"        IK 関節角 {math.degrees(w_ik):.2e} deg")
        print(f"        到達可否の判定が食い違った数 {n_status}")
        if not ok:
            print("        *** 不一致 ***")

    print("\n" + "=" * 70)
    print("両実装は一致" if failures == 0 else f"不一致 {failures} ケース")
    print("=" * 70)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
