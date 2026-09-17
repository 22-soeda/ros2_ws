# -*- coding: utf-8 -*-
"""静歩行の静的設定 (config/static_gait.yaml)。

動歩行の GaitParams (gait.yaml) とは**別のファイル・別の型**にしてある。
キーの厳密チェック (未知キーで KeyError) を両方で保ったまま、片方にしかない
項目を足せるようにするため。共通の意味を持つ項目 (z_c, v_max など) は
名前と単位を GaitParams と揃えてある。
"""

from dataclasses import dataclass, fields
import math


@dataclass
class StaticGaitParams:
    # --- 力学 -------------------------------------------------------------
    z_c: float = 0.261              # [m]   重心高さ (gait.yaml と同じ。ZMP と重心のずれの計算に使う)
    gravity: float = 9.81           # [m/s^2]
    # **実機の足間隔そのもの。** 動歩行は計画の W を広く取り motion ノードの
    # stance_y_offset で実機の足へ戻しているが、静歩行で同じことをすると重心が
    # 支持足の中心から外へずれる (W/2 + offset と W/2 の差)。静歩行ではオフセットを
    # 使わず、ここに実機の値を入れる。重心を横へずらしたいときは com_offset_y。
    foot_spacing: float = 0.140     # [m]   左右の足間隔 W (2026-09-14 実機校正)

    # --- 足裏 -------------------------------------------------------------
    # 足裏は IK の目標点 (足首軸の真下の「足裏中心」) を中心にした長方形とみなす。
    # 静的余裕 (ZMP が支持多角形の縁からどれだけ内側か) の判定に使う。
    sole_length: float = 0.118      # [m]   前後 (2026-09-17 実測)
    sole_width: float = 0.074       # [m]   左右 (同上)

    # --- 重心移動 (SHIFT) ---------------------------------------------------
    # 重心の加速度で ZMP が重心の真下からずれてよい量。LIPM では p = c - c''/ω² なので、
    # 5 次多項式の加速度の最大 (10/√3)·D/τ² を ω²·zmp_tol 以下にする τ を距離 D から決める。
    # 5mm で足間隔 140mm の移動に約 2.1 s。
    zmp_tol: float = 0.005          # [m]
    t_shift_min: float = 0.3        # [s]   重心移動の最短時間 (短い距離でも急がない)
    # 振り出し中の重心を、支持足の中心から横へずらす量。**+ で外側** (支持足のさらに外)、
    # - で内側 (もう片足の側)。計画は骨盤 = 重心とみなすが、実機は遊脚を上げると重心が
    # 遊脚側へ寄るので、+ で打ち消せる。ただし + にするほど遊脚が骨盤から横へ離れ、
    # 足首リンクで足が上がらなくなる (- はその逆)。静的余裕は sole_width/2 - |これ|。
    com_offset_y: float = 0.0       # [m]

    # --- 遊脚 (SWING) -----------------------------------------------------
    t_swing: float = 0.8            # [s]   振り出しの時間 (重心は支持足の上で止めておく)
    # 重心を支持足の上に置くと、遊脚は骨盤から横へ W 離れたまま上がる。足首の
    # パラレルリンクは外へ開いた足を高く上げられないので、動歩行 (50mm) より低い。
    # 値の根拠 (leg_service での走査表) は static_gait.yaml の注記。
    swing_height: float = 0.025     # [m]   足上げの頂点高さ
    td_overdrive: float = 0.004     # [m]   名目床面より下へ突き抜ける量 (gait.yaml と同じ意味)
    # 降下区間は 0.55·t_swing。(swing_height + td_overdrive) / (0.55·t_swing) を
    # 下回ると足が床に届かないまま振り出しが終わる (check_static_gait が見る)。
    td_speed_max: float = 0.20      # [m/s] 着地直前の降下速度上限

    # --- 指令の解釈 -------------------------------------------------------
    # /cmd_walk の速度 v を「歩幅」に読み替える係数。**歩幅 = v·stride_time。**
    # 動歩行 (gait.yaml の t_step) と揃えると、同じ指令で同じ足跡になる。
    # 実際の速さは 歩幅 / (重心移動 + 振り出し) で、動歩行の約 1/5
    # (全速前進で 1 歩 2.96 s・0.020 m/s)。
    stride_time: float = 0.60       # [s]
    v_max: tuple = (0.10, 0.04)     # [m/s]   (x, y) の飽和。歩幅の上限 = v_max·stride_time
    # 純 FF の発散条件は無い (1 歩ごとに静止する) ので、ここは操縦の手触りを
    # 動歩行と揃えるためだけの値。歩幅は振り出しの開始時に固定する。
    a_max: tuple = (0.06, 0.03)     # [m/s^2] (x, y) のレート制限

    # --- 閾値 -------------------------------------------------------------
    v_start_eps: float = 0.005      # [m/s] これ以上で歩き始める
    v_stop_eps: float = 0.010       # [m/s] 歩の境界でこれ未満なら足を揃えて止まる
    loop_hz: float = 200.0          # [Hz]  check_static_gait の遊脚の走査に使う

    @property
    def omega(self) -> float:
        """LIPM の時定数 ω = sqrt(g/z_c)。"""
        return math.sqrt(self.gravity / self.z_c)

    @classmethod
    def from_dict(cls, d: dict) -> 'StaticGaitParams':
        known = {f.name for f in fields(cls)}
        kw = {}
        for k, v in d.items():
            if k not in known:
                raise KeyError(f'static_gait.yaml に未知のキー: {k}')
            kw[k] = tuple(v) if isinstance(v, list) else v
        return cls(**kw)

    @classmethod
    def from_yaml(cls, path: str) -> 'StaticGaitParams':
        import yaml
        with open(path, encoding='utf-8') as f:
            d = yaml.safe_load(f) or {}
        return cls.from_dict(d)

    def to_dict(self) -> dict:
        return {f.name: (list(v) if isinstance(v := getattr(self, f.name), tuple) else v)
                for f in fields(self)}
