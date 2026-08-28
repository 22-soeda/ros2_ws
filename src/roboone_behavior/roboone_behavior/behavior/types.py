# -*- coding: utf-8 -*-
"""行動層の入出力の型と状態の名前。

docs/behavior_planning.pdf §6.2 の「観測列を与えれば決定的に動く」を満たすため、
BehaviorCore が外から受け取るものを全部 Observation 1 個に閉じてある。時計も
乱数も持たないので、テストは Observation を並べるだけで書ける。
"""

from dataclasses import dataclass, field

#: 状態。優先順位の高い順に並べる（§3.1 の列挙）。この並びがそのまま
#: 「割り込みが通常状態に勝つ」判定に使われるので、順番を変えないこと。
WAIT = 'WAIT'
SELF_DOWN = 'SELF_DOWN'
EDGE = 'EDGE'
RETREAT = 'RETREAT'
ENGAGE = 'ENGAGE'
APPROACH = 'APPROACH'
SEARCH = 'SEARCH'

STATES = (WAIT, SELF_DOWN, EDGE, RETREAT, ENGAGE, APPROACH, SEARCH)

#: 優先順位。小さいほど強い
PRIORITY = {s: i for i, s in enumerate(STATES)}

#: 割り込み状態。最短滞在 T_dwell を飛び越えて入れる（§3.1 末尾）
INTERRUPTS = (WAIT, SELF_DOWN, EDGE)

#: Opponent.msg の status と同じ値。msg を import せずに済ませるための写し
STATUS_OK = 0
STATUS_NO_OPPONENT = 1
STATUS_ATTITUDE_STALE = 2
STATUS_RING_LOST = 3


@dataclass
class Observation:
    """1 周期ぶんの入力。ノードが最新値を詰めて渡す。

    「見えていない」を None で表す。0 で埋めると、行動層が「原点に相手がいる」
    と読む事故が起きるため（検出器側 §9.2 と同じ考え方）。
    """

    #: [s] 前周期からの経過。ノードは実測値を入れる（タイマの公称値ではなく）
    dt: float = 0.05

    # --- 指令権 -----------------------------------------------------------
    #: /autonomy。false なら WAIT。teleop が指令権を持っている
    autonomy: bool = False
    #: /estop。true なら WAIT（脱力中）
    estop: bool = False

    # --- 歩行ノードの状態 (/motion/state) ---------------------------------
    #: /motion/state の先頭の語。実機の motion は RELAX / ARMING / HOLD /
    #: WALK / MOTION を出す（歩行ノートの IDLE/START/STEP/STOP/FALL/ESTOP は
    #: walk_core の内部状態で、motion ノードはそれを包んだ名前を出す）
    motion_state: str = 'HOLD'
    #: 技を再生中か。再生中は歩行指令を出さない（§3.5）
    motion_busy: bool = False
    #: motion が歩ける状態にあるか。RELAX（脱力）や ARMING（トルク投入中）は
    #: false。false のまま歩行指令を出すと、機体は動かないのに行動層だけが
    #: 「歩いたつもり」でリング座標系を進めてしまう
    motion_ready: bool = True

    # --- 相手 (/opponent) --------------------------------------------------
    #: このフレームに新しい /opponent が届いたか
    opponent_fresh: bool = False
    #: Opponent.status
    opponent_status: int = STATUS_NO_OPPONENT
    #: Opponent.extrapolated。検出器の追尾が外挿に落ちている（新しい観測ではない）
    opponent_extrapolated: bool = False
    #: [m] 機体座標での相手位置 (x 前, y 左)。見えていなければ None
    opponent_xy: tuple = None
    #: [m] クラスタ上端のリング面からの高さ z_top。無ければ None
    opponent_top: float = None
    #: [m] 水平方向の広がり。無ければ None
    opponent_width: float = None

    # --- リングの縁 (/ring_edge) -------------------------------------------
    #: 方位ビンごとの d_cliff(θ)。見ていない方位は NaN。無ければ None
    cliff: list = None
    #: [rad] cliff の半角。ビン k の中心方位は -half + (k+0.5)·2·half/len
    cliff_half_fov: float = 0.0

    # --- 自分の位置 (/odom) -------------------------------------------------
    #: リング座標系での (x, y, yaw)。/odom が無ければ None（自前の推測に落ちる）
    odom: tuple = None


@dataclass
class Command:
    """1 周期ぶんの出力。"""

    #: [m/s], [rad/s] 機体座標での歩行指令
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    #: 技名。出さない周期は None（毎周期出すと連射になる）
    motion: str = None
    #: 選ばれた状態と、選ばれた理由。/behavior/state にそのまま載せる
    state: str = WAIT
    reason: str = ''
    #: /behavior/debug 用。順番は debug_order() で固定する
    debug: dict = field(default_factory=dict)


#: /behavior/debug の並び。bag を後から読むときの列の意味なので、
#: 足すときは末尾に足し、既にある列の順番は変えないこと。
DEBUG_ORDER = (
    'rho', 'beta', 'z_top', 'H_o', 'fallen',
    'd_edge', 'd_margin', 'd_cliff0', 'lambda',
    'x_r', 'y_r', 'psi', 's',
    'state_id', 't_in_state', 't_since_obs', 't_since_move',
    'vx', 'vy', 'wz', 'edge_scale',
)


def debug_array(debug):
    """デバッグ辞書を DEBUG_ORDER の並びの配列にする。欠けている項目は NaN。"""
    return [float(debug.get(k, float('nan'))) for k in DEBUG_ORDER]
