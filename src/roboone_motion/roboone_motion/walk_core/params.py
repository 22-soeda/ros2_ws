# -*- coding: utf-8 -*-
"""歩行の静的設定 (gait.yaml の三層のうち最上段)。

docs/ros2_walk_implementation.pdf 表 2 の項目のうち、歩行計画 (フィードフォワード)
に効くものだけを持つ。ゲイン類 (k_d_ankle など) は姿勢補償の実装時に足す。

今回の機体方針により回転は扱わない (ωz ≡ 0、足ヨー ≡ 0)。そのぶん
v_max / a_max は (x, y) の 2 成分になり、文書式 (2) の楕円制限は
前進と横移動の同時要求 (斜め歩き) に読み替えて適用する。
"""

import math
from dataclasses import dataclass, field, fields


@dataclass
class GaitParams:
    # --- 力学 -------------------------------------------------------------
    z_c: float = 0.16               # [m]   骨盤 (≒重心) 高さ。ω = sqrt(g/z_c)
    gravity: float = 9.81           # [m/s^2]
    t_step: float = 0.40            # [s]   1 歩の周期 T
    foot_spacing: float = 0.08      # [m]   左右の足間隔 W

    # --- 遊脚 -------------------------------------------------------------
    swing_height: float = 0.02      # [m]   遊脚の頂点高さ h_sw
    swing_lock_phase: float = 0.70  # [-]   着地点の凍結位相 φ_lock
    td_overdrive: float = 0.004     # [m]   名目床面より下へ突き抜ける量 z_od
    td_speed_max: float = 0.10      # [m/s] 着地直前の降下速度上限

    # --- 指令の整形 -------------------------------------------------------
    v_max: tuple = (0.15, 0.08)     # [m/s]     (x, y) の飽和
    a_max: tuple = (0.3, 0.2)       # [m/s^2]   (x, y) のレート制限

    # --- 着地点クランプ (式 11) -------------------------------------------
    step_clamp_x: float = 0.04      # [m] 前後の許容ずれ
    step_clamp_out: float = 0.045   # [m] 外側の許容ずれ
    step_clamp_in: float = 0.015    # [m] 内側の許容ずれ (脚同士の干渉のため狭い)

    # --- 状態機械 ---------------------------------------------------------
    start_pushoff_max: float = 0.15  # [s]   押し出しの最長時間
    k_dcm: float = 1.0               # [-]   踏み出し補正のゲイン (計画では 1)
    cmd_timeout: float = 0.5         # [s]   指令途絶で停止に入る
    loop_hz: float = 200.0           # [Hz]  周期

    # --- 閾値 (実装で追加。文書に明示値がないもの) --------------------------
    v_start_eps: float = 0.005       # [m/s] これ以上で歩き始める
    v_stop_eps: float = 0.003        # [m/s] 歩の境界でこれ未満なら停止へ
    settle_eps: float = 0.002        # [m]   |ξ - x_C| がこれ未満で静止とみなす
    stop_outside_eps: float = 0.005  # [m]   ξ が支持多角形からこれ以上外れたらもう 1 歩

    @property
    def omega(self) -> float:
        """LIPM の時定数 ω = sqrt(g/z_c)。z_c=0.16 で 7.83 rad/s。"""
        return math.sqrt(self.gravity / self.z_c)

    @property
    def e_wt(self) -> float:
        """e^{ωT}。既定値で 22.9 (文書 §3.4 と一致)。"""
        return math.exp(self.omega * self.t_step)

    @classmethod
    def from_dict(cls, d: dict) -> 'GaitParams':
        known = {f.name for f in fields(cls)}
        kw = {}
        for k, v in d.items():
            if k not in known:
                raise KeyError(f'gait.yaml に未知のキー: {k}')
            kw[k] = tuple(v) if isinstance(v, list) else v
        return cls(**kw)

    @classmethod
    def from_yaml(cls, path: str) -> 'GaitParams':
        import yaml
        with open(path, encoding='utf-8') as f:
            d = yaml.safe_load(f) or {}
        return cls.from_dict(d)

    def to_dict(self) -> dict:
        return {f.name: (list(v) if isinstance(v := getattr(self, f.name), tuple) else v)
                for f in fields(self)}
