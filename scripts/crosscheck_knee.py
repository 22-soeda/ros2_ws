#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""C++ 実装 (roboone_kinematics/knee_fourbar.hpp) と Python 参照実装
(knee_fourbar.py) の突き合わせ。

同じ寸法・同じ枝・同じ θ2 に対して、両者の順変換・逆変換・伝達比・伝達角・
速度・加速度・サーボ換算がすべて一致することを確認する。独立に書いた 2 つの
実装を比べるので、片方だけの取り違えを拾える（crosscheck_cpp.py と同じ狙い）。

膝の平面幾何は左右共通なので、脚の左右は関係しない（文書 §11）。

  python3 scripts/crosscheck_knee.py [-n 点数]

C++ 側は build/roboone_kinematics/knee_dump を使う (colcon build 済みであること)。
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import knee_fourbar  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.dirname(_HERE)

_STATUS = {0: "Ok", 1: "Unreachable", 2: "Degenerate", 3: "DeadPoint"}

#: (r1, r2, r3, r4, β, ε, θ4_zero[deg], σ, θ2 の範囲[deg])
#: 実機の寸法だけでなく、r1 ≠ r4 と退化ケースも通して簡約の混入を検査する。
CASES = [
    (20.0, 45.0, 35.0, 20.0, -1, +1, 89.3, +1, (160.0, 270.0)),   # 実機
    (20.0, 45.0, 35.0, 26.0, -1, +1, 89.3, +1, (160.0, 270.0)),   # r1 ≠ r4
    (20.0, 45.0, 35.0, 14.0, -1, +1, 89.3, +1, (170.0, 260.0)),
    (20.0, 45.0, 35.0, 20.0, +1, -1, 89.3, -1, (160.0, 270.0)),   # 枝も σ も逆
    (20.0, 45.0, 20.0, 45.0, +1, -1, 0.0, +1, (2.0, 178.0)),      # 平行四辺形
    (0.0, 45.0, 35.0, 20.0, -1, +1, 0.0, +1, (0.0, 357.0)),       # r1 = 0（同軸）
    (33.0, 40.0, 30.0, 25.0, -1, +1, 45.0, -1, (150.0, 240.0)),   # 無関係な寸法
]


def _find_dump() -> str:
    for path in (os.path.join(_WS, "build", "roboone_kinematics", "knee_dump"),
                 os.path.join(_WS, "install", "roboone_kinematics",
                              "lib", "roboone_kinematics", "knee_dump")):
        if os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    raise SystemExit(
        "knee_dump が見つからない。先に "
        "`colcon build --packages-select roboone_kinematics` を実行する。")


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def main() -> int:
    ap = argparse.ArgumentParser(description="膝 4 節リンク C++ と Python の突き合わせ")
    ap.add_argument("-n", type=int, default=441, help="1 ケースあたりの θ2 の点数")
    args = ap.parse_args()

    dump = _find_dump()
    print("=" * 70)
    print("C++ (knee_fourbar.hpp) vs Python (knee_fourbar.py)")
    print("=" * 70)
    print(f"  knee_dump: {os.path.relpath(dump, _WS)}")

    failures = 0
    for ci, (r1, r2, r3, r4, beta, eps, z, sig, (lo, hi)) in enumerate(CASES):
        link = knee_fourbar.KneeFourBar(
            r1=r1, r2=r2, r3=r3, r4=r4, beta=beta, eps=eps,
            theta4_zero=math.radians(z), sigma_joint=sig)

        raw = subprocess.run(
            [dump, repr(r1), repr(r2), repr(r3), repr(r4), str(beta), str(eps),
             repr(z), str(sig), str(args.n), repr(lo), repr(hi)],
            capture_output=True, text=True, check=True).stdout

        w = {"θ3": 0.0, "θ4": 0.0, "IK θ2": 0.0, "伝達比": 0.0, "γ": 0.0,
             "ω3,ω4": 0.0, "α3,α4": 0.0, "曲げ量": 0.0, "サーボ": 0.0}
        n_status = 0
        rows = n_ok = 0
        for line in raw.strip().splitlines():
            v = [float(x) for x in line.split(",")]
            t2, st = v[0], int(v[12])
            rows += 1

            try:
                pose = link.fk(t2)
            except knee_fourbar.Unreachable:
                if st == 0:
                    n_status += 1          # C++ だけが解けたことになっている
                continue
            if st != 0:
                n_status += 1              # Python だけが解けたことになっている
                continue
            n_ok += 1

            w["θ3"] = max(w["θ3"], abs(_wrap_pi(pose.theta3 - v[1])))
            w["θ4"] = max(w["θ4"], abs(_wrap_pi(pose.theta4 - v[2])))
            w["伝達比"] = max(w["伝達比"], abs(link.ratio(pose) - v[3]))
            w["γ"] = max(w["γ"], abs(link.transmission_angle(pose) - v[4]))
            w["IK θ2"] = max(w["IK θ2"], abs(_wrap_pi(link.ik(pose.theta4).theta2 - v[5])))

            o3, o4 = link.velocity(pose, 1.0)
            w["ω3,ω4"] = max(w["ω3,ω4"], abs(o3 - v[6]), abs(o4 - v[7]))
            a3, a4 = link.acceleration(pose, 1.0, 1.0)
            w["α3,α4"] = max(w["α3,α4"], abs(a3 - v[8]), abs(a4 - v[9]))

            bend = link.joint_from_rocker(pose.theta4)
            w["曲げ量"] = max(w["曲げ量"], abs(_wrap_pi(bend - v[10])))
            w["サーボ"] = max(w["サーボ"],
                           abs(_wrap_pi(link.joint_to_motor(bend) - v[11])))

        ok = (n_status == 0 and n_ok > 0 and
              max(w["θ3"], w["θ4"], w["IK θ2"], w["曲げ量"], w["サーボ"]) < 1e-12 and
              max(w["伝達比"], w["γ"], w["ω3,ω4"], w["α3,α4"]) < 1e-9)
        failures += 0 if ok else 1

        print(f"\n  [{ci}] r=({r1:g}, {r2:g}, {r3:g}, {r4:g}) β={beta:+d} ε={eps:+d} "
              f"θ4_zero={z:g}° σ={sig:+d}")
        print(f"        θ2 {lo:g}〜{hi:g}° を {rows} 点（うち両方が解けた {n_ok} 点）")
        print(f"        角 θ3 {math.degrees(w['θ3']):.2e} / θ4 {math.degrees(w['θ4']):.2e} / "
              f"IK θ2 {math.degrees(w['IK θ2']):.2e} deg")
        print(f"        伝達比 {w['伝達比']:.2e} / γ {math.degrees(w['γ']):.2e} deg")
        print(f"        速度 {w['ω3,ω4']:.2e} / 加速度 {w['α3,α4']:.2e}")
        print(f"        曲げ量 {math.degrees(w['曲げ量']):.2e} / "
              f"サーボ {math.degrees(w['サーボ']):.2e} deg")
        print(f"        到達可否の判定が食い違った数 {n_status}")
        if not ok:
            print("        *** 不一致 ***")

    print("\n" + "=" * 70)
    print("両実装は一致" if failures == 0 else f"不一致 {failures} ケース")
    print("=" * 70)
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
