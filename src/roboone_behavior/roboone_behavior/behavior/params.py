# -*- coding: utf-8 -*-
"""行動層の定数。

docs/behavior_planning.pdf §5「数値のまとめ」の表がそのまま入っている。
分け方は roboone_perception.detect.params と同じ 3 群にしてある。

    MatchParams  競技から決まる … リングの寸法と規則の時間。実測で動かさない
    RobotParams  機体から決まる … 歩行の上限・技の間合い・深度の死角
    TuneParams   実装から決まる … ゲインとヒステリシス。実機で追い込む

境界をこう引く理由は検出器側と同じで、「規則から来る数」と「実機を見て動かす数」を
混ぜると、調整のたびに規則側の値まで動かしてしまうためである。MatchParams を
bag や試走の数字で書き換えてはいけない。

config/behavior.yaml に書くのは「この機体での選択」だけで、既定値はここにある。
"""

from dataclasses import dataclass, field, fields
import math


def _from_dict(cls, src, prefix=''):
    """{名前: 値} から dataclass を作る。未知のキーは黙って捨てる。"""
    known = {f.name for f in fields(cls)}
    kwargs = {k[len(prefix):]: v for k, v in src.items()
              if k.startswith(prefix) and k[len(prefix):] in known}
    return cls(**kwargs)


@dataclass
class MatchParams:
    """競技規則とリングの寸法から決まる定数。

    出典は第44回ROBO-ONE競技規則（2025年6月11日作成）。条番号は
    docs/behavior_planning.pdf §1.1 の表に対応する。
    """

    #: [m] リングの半幅 a。360 cm 角なので 1.8 m（4.1 図C-1）
    ring_half_width: float = 1.8
    #: [m] 四隅の落とし c。角の辺は |x|+|y| = 2a-c = 2.7 m（4.1 図C-2）
    ring_corner_cut: float = 0.9

    #: [m] 相手がダウンしたとき離れる距離 ρ_r。規則 10.2(b)(i) は
    #: 「起き上がりを妨げない距離」としか書いておらず、数値は規定に無い。
    #: 腕を伸ばした相手に触れない距離として決めた値
    retreat_range: float = 0.6
    #: [m] 1 回の RETREAT で許す後退距離。後退は視野外の移動なので上限を切る
    retreat_max_travel: float = 0.4

    #: [s] 小移動の周期 T_ka。規則 10.3(d) の「10 秒以上前後左右に移動しない」に
    #: 対して、歩行の遅れと横移動の所要時間を見込んで十分短く取る
    keepalive_period: float = 6.0
    #: [m] 1 回の小移動で横へ動く距離
    keepalive_shift: float = 0.05

    #: しゃがみを含む技のあとに要求される歩数。規則 10.2(l)
    crouch_steps: int = 3


@dataclass
class RobotParams:
    """機体から決まる定数。起動時に固定する。"""

    #: [m/s] 前進・横移動の上限。歩行ノート(2) 表 2 の v_max
    v_max: float = 0.15
    v_y_max: float = 0.08
    #: [rad/s] 旋回の上限 ω_max
    w_max: float = 0.8

    #: [m] 技の間合い ρ_s。腕の届く距離から数 cm 引く。技ができてから詰める
    strike_range: float = 0.25
    #: [m] 深度の最短距離 ρ_blind。解像度設定ごとに実測すること（§7 の未確定表）
    blind_range: float = 0.25

    #: [rad] 深度の水平視野の半角 β_fov。これを超えたら見えない
    fov_half: float = math.radians(43.5)

    #: [s] 1 歩の周期。しゃがみ後の「3 歩」を時間で数えるのに使う（gait.yaml の T_step）
    step_period: float = 0.40

    #: ENGAGE で出す技の名前。順に出す。/cmd_motion にそのまま載るので、
    #: motion 側の config/motions.yaml にある名前でなければならない。
    #: 空にすると ENGAGE は間合いを保って待つだけになる（歩行だけの試走用）
    techniques: list = field(default_factory=lambda: ['punch_r', 'punch_l'])
    #: 上のうち「しゃがみを含む」もの。規則 10.2(l) の 3 歩の縛りが掛かる
    crouch_techniques: list = field(default_factory=list)

    #: [m] リング座標系の初期位置と向き。「はじめ」の時点の自陣コーナー。
    #: 規則の赤青コーナーの運用を確認して置き直すこと（§7 の未確定表）
    start_x: float = -1.2
    start_y: float = -0.9
    start_yaw: float = math.radians(33.7)


@dataclass
class TuneParams:
    """実装から決まる定数。実機で追い込む対象はここに集める。"""

    #: 判断周期 [Hz]。/cmd_walk と同じ
    rate_hz: float = 20.0

    # --- 相手の追跡 (§2.2) ------------------------------------------------
    #: 観測の取り込み率 α
    track_alpha: float = 0.5
    #: [m] 予測から離れた観測を外れ値とみなす幅 g_max
    track_gate: float = 0.3
    #: 外れ値がこの回数続いたら乗り換える
    track_gate_relax: int = 2
    #: [s] 最後に受理した観測からこれだけ経ったら「見失い」T_lost
    lost_time: float = 0.5
    #: [s] 近距離の死角で最後の位置を保持する時間 T_close
    close_hold_time: float = 2.0
    #: [m] 「近すぎて見えない」と判定する余裕。ρ < ρ_blind + これ
    close_margin: float = 0.1
    #: [m] 死角で見失ったまま T_close を過ぎたときに下がる距離
    close_back_off: float = 0.15

    # --- 転倒判定 (§2.3) --------------------------------------------------
    #: 立位高さ H_o に対する転倒・復帰のしきい比 κ_d / κ_u
    fallen_ratio: float = 0.5
    stand_ratio: float = 0.75
    #: [s] 転倒・復帰の継続時間 T_down / T_up
    fallen_time: float = 0.7
    stand_time: float = 0.5
    #: [s] 「はじめ」直後に相手の立位高さ H_o を測る窓
    height_cal_time: float = 2.0
    #: [m] H_o の初期値。測れないうちはこれを使う
    height_default: float = 0.35
    #: [m] これより近いと z_top が視野で切れる（ρ_s + これ）。この中では
    #: 転倒の判定に横幅の証拠を併せて要求し、復帰の判定は凍結する
    #: （FallenDetector._step_close の注記を見ること）
    fallen_freeze_margin: float = 0.2
    #: 間合いの中で転倒と認めるのに要る横幅の、H_o に対する比。上端が切れても
    #: クラスタが横に広がることはない、という切り分けに使う
    close_width_ratio: float = 0.75

    # --- リング座標系と縁 (§2.4) ------------------------------------------
    #: [m] 縁の余裕の切片 d_0
    edge_margin0: float = 0.25
    #: 最後に補正してからの歩行距離に対する余裕の増え方 κ_s。滑りの実測で決める
    edge_margin_slip: float = 0.15
    #: [m] 中央へ寄せ始める距離・EDGE の解除距離 d_1
    edge_release: float = 0.5
    #: [s] 縁の事前確認の先読み T_h
    lookahead: float = 1.0
    #: [m/s] EDGE で中央へ戻る速さ v_e
    edge_speed: float = 0.1
    #: [m/s] RETREAT で離れる速さ v_r
    retreat_speed: float = 0.1
    #: [m] 前方の切れ目でリング座標系を補正する条件。予測との差がこれ以内なら
    #: 同じ縁を見ているとみなす。これを超える差は別の縁か誤検出として捨てる
    cliff_fix_gate: float = 0.6

    # --- 状態遷移 (§3) ----------------------------------------------------
    #: [rad] 前進を許す方位のヒステリシス β_in / β_out
    bearing_in: float = math.radians(8.0)
    bearing_out: float = math.radians(15.0)
    #: [1/s] 方位サーボのゲイン k_β
    k_bearing: float = 1.5
    #: [1/s] 接近の減速ゲイン k_v
    k_range: float = 0.8
    #: [m] 到達判定の幅 ε_ρ
    range_eps: float = 0.03
    #: [rad/s] SEARCH の旋回速度 ω_search
    search_omega: float = 0.6
    #: 知覚が縮退しているとき（ATTITUDE_STALE / RING_LOST）の旋回速度の掛け率。
    #: 姿勢がジャイロ任せに落ちている間に速く回すとヨーの誤差がそのまま乗る
    search_degraded_scale: float = 0.5
    #: [s] 状態の最短滞在 T_dwell
    dwell: float = 0.5
    #: [m/s] 「移動している」とみなす速度。小移動のタイマの判定に使う
    move_eps: float = 0.02
    #: [s] 技を渡してから再生が始まらないまま諦める時間。motion が名前を
    #: 知らないときに ENGAGE から出られなくなるのを防ぐ
    technique_timeout: float = 1.0


@dataclass
class BehaviorParams:
    """3 群をまとめたもの。ROS ノードはこれをパラメータから組み立てて渡す。"""

    match: MatchParams = field(default_factory=MatchParams)
    robot: RobotParams = field(default_factory=RobotParams)
    tune: TuneParams = field(default_factory=TuneParams)

    @staticmethod
    def from_flat(flat):
        """{'tune.k_bearing': 1.5, ...} の平坦な辞書から組み立てる。

        ROS のパラメータはドット区切りで来るので、その形をそのまま受ける。
        """
        return BehaviorParams(
            match=_from_dict(MatchParams, flat, 'match.'),
            robot=_from_dict(RobotParams, flat, 'robot.'),
            tune=_from_dict(TuneParams, flat, 'tune.'),
        )
