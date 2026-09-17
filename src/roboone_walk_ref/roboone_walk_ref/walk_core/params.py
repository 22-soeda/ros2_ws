# -*- coding: utf-8 -*-
"""歩行の静的設定 (gait.yaml の三層のうち最上段)。

docs/ros2_walk_implementation.pdf 表 2 の項目のうち、歩行計画 (フィードフォワード)
に効くものだけを持つ。ゲイン類 (k_d_ankle など) は姿勢補償の実装時に足す。

今回の機体方針により回転は扱わない (ωz ≡ 0、足ヨー ≡ 0)。そのぶん
v_max / a_max は (x, y) の 2 成分になり、文書式 (2) の楕円制限は
前進と横移動の同時要求 (斜め歩き) に読み替えて適用する。
"""

from dataclasses import dataclass, fields
import math


@dataclass
class GaitParams:
    # --- 力学 -------------------------------------------------------------
    # z_c はホーム姿勢 (config/home_pose.yaml: 脚ピッチ 30 deg 曲げ) での重心質点
    # (= 運動学 Σ_B 原点) から足裏までの高さ。roboone_walk_core/gait_from_kinematics
    # が FK で出した値 (260.9 mm)。姿勢を変えたらツールで出し直す。
    z_c: float = 0.261              # [m]   重心高さ。ω = sqrt(g/z_c) = 6.13 rad/s
    gravity: float = 9.81           # [m/s^2]
    # T は倒れる時定数 1/ω = 163 ms との比 ωT で効く。**静歩行寄りに倒してある。**
    # T を伸ばすほど重心が支持足の真上に寄る (= 静歩行に漸近)。単脚支持の中央で
    # 重心が支持足から内側にどれだけ残るかは (W/2) sech(ωT/2):
    #   T=0.30 -> 61.4mm   T=0.60 -> 27.7mm   T=1.00 -> 8.3mm
    # 引き換えに e^{ωT} が 6.3 -> 39.6 に増え、純 FF では計画と実機のずれが 1 歩で
    # 39.6 倍に増幅される。**推定 ξ を入れるまでこれ以上伸ばさないこと。**
    # 短くする側の下限は swing_height を出せるサーボ速度。上昇区間は 0.45T しか
    # なく、膝サーボは 50mm の持ち上げに 15.4 deg (0.269 rad) 動く。ピークは 5 次
    # 多項式の 1.875 倍で 1.875 * 0.269 / (0.45 T)。T=0.60 で 1.87 rad/s = 無負荷
    # 4.7 rad/s の 40%、T=0.25 だと 95% になって追従せず、足が上がりきらない。
    t_step: float = 0.60            # [s]   1 歩の周期 T
    foot_spacing: float = 0.1786    # [m]   左右の足間隔 W = 股間隔 (leg_config HIP_Y x 2)

    # --- 遊脚 -------------------------------------------------------------
    # 20mm では実機で両足とも床を擦ってその場でもじもじするだけだった (2026-08-29)。
    # 上げると td_speed_max の下限と t_step の下限が連動して動く (上の t_step 参照)。
    swing_height: float = 0.05      # [m]   遊脚の頂点高さ h_sw
    swing_lock_phase: float = 0.70  # [-]   着地点の凍結位相 φ_lock
    td_overdrive: float = 0.004     # [m]   名目床面より下へ突き抜ける量 z_od
    # 降下区間は 0.55T しかないので、(swing_height + td_overdrive) / (0.55 T) を
    # 下回ると足が床に届かないまま歩が終わる。54mm / (0.55 * 0.60) = 最低 0.164 m/s。
    td_speed_max: float = 0.20      # [m/s] 着地直前の降下速度上限
    # 遊脚が歩周期 T のうち何割を使うか。1.0 = 従来（φ=1 ちょうどで着く）。
    # 下げるとその手前で着地点に達し、残りは両足が着いたまま止まる。
    #
    # ★ZMP は _enter_step() で支持足に固定され歩の間ずっと動かないので、ここを
    #   変えても DCM の伝播も b の閉形式も a_max の発散条件も**一切変わらない**。
    #   増えるのは「支持多角形が広い時間」だけで、**計画上の重心経路は同じ**。
    #   重心の動きまで静的にしたいなら ZMP を両足間で動かす必要があり、それは
    #   b の導出からやり直しになる（この項目では届かない）。
    #
    # ★狙った値がそのまま出るわけではない。降下が td_speed_max で飽和するぶん
    #   着地は計画より遅れる。実際の値は check_swing_landing() が出す。
    swing_ratio: float = 1.0        # [-]   遊脚が使う歩周期の割合
    # 両足支持の時間 Td。**0 = 従来** (両足支持なし。支持足は歩の境界で一瞬で入れ替わる)。
    # 正にすると、各歩の頭に Td の両足支持を置き、その間に ZMP を前の支持足から新しい
    # 支持足へ直線で移す。t_step はその後の単脚支持の長さ Ts のままなので、1 歩は
    # Td + Ts に伸びる (歩幅は v·t_step のままなので、実際の速さは Ts/(Td+Ts) 倍に落ちる)。
    # 横の重心の速さが落ち、骨盤も支持足へ寄る (docs/サーボ追従と両足支持_実装計画.md §1.8)。
    # 引き換えに 1 歩の増幅 e^{ω(Td+Ts)} と、ずれの吸収に要る着地点のずらし
    # e^{ωTd}/κ 倍 (κ = (e^{ωTd}-1)/(ωTd)) が増える。値の根拠は gait.yaml の注記。
    ds_time: float = 0.0            # [s]   両足支持の時間 Td

    # --- 指令の整形 -------------------------------------------------------
    # 歩幅 = v * T なので、**T を伸ばしたらここを下げないと足先が到達域を出る。**
    # T=0.60 で (0.15, 0.08) のままだと遊脚頂点 (50mm) の前後が ±49mm になり、
    # その高さの機構到達 (前 56 / 後 37 mm) を超える。(0.10, 0.04) なら頂点 ±33mm、
    # 遊脚の横は骨盤系 158.6mm (機構限界 162.3mm)。
    v_max: tuple = (0.10, 0.04)     # [m/s]     (x, y) の飽和
    # a_max は文書表 2 では (0.3, 0.2) だが、純フィードフォワードでは 1 歩あたりの
    # 指令変化 ΔL = (a·T)·T = a·T² が着地点クランプで吸収できる範囲
    #   ΔLx (1 + 1/(e^{ωT}-1)) < step_clamp_x,  ΔLy (1 + 1/(e^{ωT}+1)) < step_clamp_in
    # を超えると、吸収残りが e^{ωT} (z_c=0.261, T=0.60 で 39.6) 倍に増幅されて発散する。
    # 上限は T² で効くので T を倍にすると 1/4 に狭まる。その条件 (a_x < 0.11,
    # a_y < 0.05) の 6 割程度に抑えてある (gait_from_kinematics --t-step 0.60 が印字)。
    # a_y の上限は step_clamp_in に比例するので、下の 15->20mm と連動している。
    # 踏み出し補正 (推定 ξ) を入れた段階で再検討。
    a_max: tuple = (0.06, 0.03)     # [m/s^2]   (x, y) のレート制限

    # --- 着地点クランプ (式 11) -------------------------------------------
    step_clamp_x: float = 0.04      # [m] 前後の許容ずれ
    step_clamp_out: float = 0.045   # [m] 外側の許容ずれ
    # **停止準備歩のオフセット b_stop = (W/2)/e^{ωT} より大きく取ること。** 下回ると
    # _update_prep_landing の着地が必ずここでクランプされ、横の残留を吸収しきれずに
    # 公称より広い立位で止まる。T=0.60 では b_stop=2.3mm なので余裕がある
    # (T=0.30 では 14.2mm、T=0.25 では 19.3mm だった)。20mm のままにしてあるのは
    # a_y の上限が step_clamp_in に比例するため。T を変えたらこの不等式を必ず確認する。
    step_clamp_in: float = 0.020    # [m] 内側の許容ずれ (脚同士の干渉のため狭い)

    # --- 状態機械 ---------------------------------------------------------
    start_pushoff_max: float = 0.15  # [s]   押し出しの最長時間
    k_dcm: float = 1.0               # [-]   踏み出し補正のゲイン (計画では 1)
    cmd_timeout: float = 0.5         # [s]   指令途絶で停止に入る
    loop_hz: float = 200.0           # [Hz]  周期

    # --- 閾値 (実装で追加。文書に明示値がないもの) --------------------------
    v_start_eps: float = 0.005       # [m/s] これ以上で歩き始める
    v_stop_eps: float = 0.010        # [m/s] 歩の境界でこれ未満なら停止シーケンスへ
    settle_eps: float = 0.002        # [m]   |ξ - x_C| がこれ未満で静止とみなす
    stop_outside_eps: float = 0.005  # [m]   ξ が支持多角形からこれ以上外れたらもう 1 歩

    @property
    def omega(self) -> float:
        """LIPM の時定数 ω = sqrt(g/z_c)。z_c=0.261 で 6.13 rad/s。"""
        return math.sqrt(self.gravity / self.z_c)

    @property
    def e_wt(self) -> float:
        """e^{ωT}。既定値 (z_c=0.261, T=0.60) で 39.6。文書 §3.4 の 22.9 は z_c=0.16 のとき。"""
        return math.exp(self.omega * self.t_step)

    def ds_consts(self):
        """両足支持の定数 (E_d, κ, E, K)。

        E_d = e^{ωTd}、κ = (E_d - 1)/(ωTd) (Td → 0 で 1)、E = E_d·e^{ωTs}、K = κ·e^{ωTs}。
        1 歩 (両足支持 Td で ZMP が変位 l だけ動き、単脚支持 Ts で止まる) の間に、
        歩の頭の ZMP から見た ξ のずれ c は c' = E·c - K·l に進む (engine.py の docstring)。
        """
        es = self.e_wt
        wt = self.omega * self.ds_time
        ed = math.exp(wt)
        kappa = (ed - 1.0) / wt if wt > 0.0 else 1.0
        return ed, kappa, ed * es, kappa * es

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
