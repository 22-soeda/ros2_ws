# -*- coding: utf-8 -*-
"""検出器の定数。

docs/opponent_detection.pdf §11 の分け方をそのままデータ構造にしてある。

    BodyParams   機体から決まる     … 起動時に固定。実測して合わせる対象ではない
    MatchParams  競技から決まる     … 規定の寸法から取る。bag の数字から取ってはいけない (§8.3)
    TuneParams   実装から決まる     … depth のノイズと計算量から決まる。実測で追い込む

分けてある理由は §8.3 の失敗を避けるためで、桜木町の bag に写っていたミニロボットの
寸法 (上端 26.8 cm) をそのまま MatchParams に持ち込むと、本番の等身大の相手で落ちる。
「物体の寸法に依存する定数は規定から、依存しない定数は bag から」という切り分けを、
クラスの境界として残してある。
"""

from dataclasses import dataclass, field, fields
import math


def _from_dict(cls, src, prefix=''):
    """{名前: 値} から dataclass を作る。未知のキーは呼び出し側で弾く。"""
    known = {f.name for f in fields(cls)}
    kwargs = {k[len(prefix):]: v for k, v in src.items()
              if k.startswith(prefix) and k[len(prefix):] in known}
    return cls(**kwargs)


@dataclass
class BodyParams:
    """機体から決まる定数。起動時に固定する。"""

    #: [m] リング面からカメラ原点までの高さ h_cam。高さヒストグラムの窓の中心 (§6.2)
    cam_height: float = 0.35
    #: [deg] 光軸の俯角。姿勢の初期値にだけ使う (以降は面法線が決める)
    cam_pitch_deg: float = 30.0
    #: [m] 機体原点からカメラ原点までの前方 / 左オフセット。/opponent を機体座標に直す
    cam_offset_x: float = 0.0
    cam_offset_y: float = 0.0

    @property
    def up_from_mount(self):
        """取り付けから決まる鉛直 u の初期値 (カメラ座標 x右 y下 z前)。

        俯角 θ だけ下を向いたカメラでは世界の上向きは (0, -cosθ, -sinθ) になる。
        加速度が取れないまま起動したときの出発点で、以降はジャイロと面法線が運ぶ。
        """
        t = math.radians(self.cam_pitch_deg)
        return (0.0, -math.cos(t), -math.sin(t))


@dataclass
class MatchParams:
    """競技から決まる定数。相手の寸法に依存するものだけをここに置く。

    現状の値は暫定である。docs/opponent_detection.pdf §8.3 / §12 のとおり、
    obj_top_max と obj_width_max は ROBO-ONE Auto の規定を当たって置き直すこと。
    bag に写ったミニロボット (上端 26.8 cm・幅 23.2 cm) は「この値でミニロボも
    取れるか」の下限側の確認にだけ使う。
    """

    #: [m] クラスタ上端高さの下限 / 上限
    obj_top_min: float = 0.10
    obj_top_max: float = 0.70
    #: [m] 水平方向の広がり max(w, d) の下限 / 上限
    obj_width_min: float = 0.05
    obj_width_max: float = 0.60
    #: [m] これより遠いクラスタは相手候補にしない
    range_max: float = 3.0


@dataclass
class TuneParams:
    """実装から決まる定数。相手の大きさに依存しないので bag から取ってよい。"""

    # --- 間引きと逆投影 (§4) ---------------------------------------------
    #: 深度画像を何画素ごとに読むか。§4 表 2 より d=2 で検出率は変わらず 3.3 倍速い
    stride: int = 2
    #: [m] 有効とみなす深度の範囲。D435 の最短測距と、視差がほぼ 0 の「遠い」を捨てる
    depth_min: float = 0.15
    depth_max: float = 6.0
    #: 視野の縁と判定する画素マージン [画素]。ring_edge の NaN 判定に使う (§7)
    border_px: int = 4

    # --- 姿勢 (§5) --------------------------------------------------------
    #: 面法線を u へ混ぜる比 β。式 (8)
    plane_blend: float = 0.3
    #: [m] あてはめ残差がこれを超えたら補正しない。式 (8) の門
    fit_resid_max: float = 0.010
    #: [deg] 予測した u と面法線の食い違いがこれを超えたら補正しない。式 (8) の門
    fit_angle_max_deg: float = 12.0
    #: [m] 面あてはめに使う点の高さ帯 (±) と水平距離の上限。§5.4 の 2 つの罠に対応する
    fit_band: float = 0.05
    fit_radius: float = 1.5
    #: あてはめに必要な最小点数
    fit_min_points: int = 200
    #: あてはめに使う点数の上限。超える分は等間隔に間引く。1.5 m の広がりに対して
    #: 残差 2 mm なら法線は数千点で 0.1° 級に決まるので、全点を使う意味がない。
    #: Pi 5 実測で fit_plane が 9.8 ms → 1.5 ms になる (§10 の予算に効く)
    fit_max_points: int = 4000
    #: 補正が連続で入らないフレーム数がこれを超えたら姿勢を劣化とみなす (§9.2)
    stale_frames: int = 10

    # --- リング面の高さ (§6) ---------------------------------------------
    #: [m] 高さヒストグラムの窓 (±)。34 cm の段差に対し場外の床が窓に入らない幅
    hist_window: float = 0.25
    #: [m] ビン幅と、最頻ビン周りの精密化の幅
    hist_bin: float = 0.010
    hist_refine: float = 0.020
    #: 窓の中にこれだけ点がなければリング面なしとする (§9.2 の 1 番目の縮退)
    hist_min_points: int = 300

    # --- グリッドと連結成分 (§7) ------------------------------------------
    #: [m] 占有グリッドのセル幅
    cell: float = 0.05
    #: [m] 床除去の許容 (±)。この帯に入る点をリング面とみなす
    floor_band: float = 0.030
    #: [m] グリッドの範囲。前方 u ∈ [-grid_back, grid_forward]、左右 v ∈ [-grid_side, grid_side]
    grid_forward: float = 4.0
    grid_back: float = 0.5
    grid_side: float = 3.0
    #: リング成分の種にする窓 [m]。§7 は自機直下 (0,0) のセルを使うと書いているが、
    #: 俯角 30 度・高さ 0.35 m では水平 0.188 m より近い面は写らないので直下のセルは
    #: 常に空になる。代わりに「正面の、写り始めるあたり」を種にする。自機はリングの
    #: 上に立っていて、そこからこの窓までリングは切れずに続くので、根拠は同じ。
    seed_near: float = 0.15
    seed_far: float = 0.70
    seed_half_width: float = 0.25
    #: 種の窓が空だったとき最大成分に落ちる。手持ち検証用の逃げ道 (§7)
    seed_fallback_to_largest: bool = True

    # --- 物体の抽出 (§8) --------------------------------------------------
    #: [m] 面のすぐ上の無視量 h_lo。面推定の残差と鉛直誤差から決まる量で、相手の大きさに依らない
    obj_h_lo: float = 0.040
    #: [m] 候補にする高さの上限 h_hi。MatchParams.obj_top_max より少し高く取る
    obj_h_hi: float = 0.90
    #: クラスタに要求する最小セル数
    min_cells: int = 2
    #: 距離依存の最小点数 N_min(r) = N0 (r0/r)^2。式 (14) の写像から (§8.2)
    min_points_n0: int = 60
    min_points_r0: float = 1.0
    #: 距離によらない最低ライン。遠方で N_min が落ちすぎないように
    min_points_floor: int = 12
    #: リング成分を何セル膨張させた領域まで候補を許すか (§8.1)
    ring_dilate_cells: int = 1

    # --- リングのエッジ (§7) ----------------------------------------------
    #: d_cliff(θ) の方位ビン数と、覆う方位の半幅 [deg]
    edge_bins: int = 64
    edge_half_fov_deg: float = 45.0

    # --- 追尾 (§9.1) ------------------------------------------------------
    track_alpha: float = 0.5
    track_beta: float = 0.2
    #: [m] 予測と観測の食い違いがこれを超えたら観測を棄却して外挿に落ちる
    track_gate: float = 0.35
    #: 外挿がこのフレーム数続いたら軌跡を捨てる
    track_max_coast: int = 8


@dataclass
class DetectorParams:
    """3 群をまとめたもの。ROS ノードはこれをパラメータから組み立てて渡す。"""

    body: BodyParams = field(default_factory=BodyParams)
    match: MatchParams = field(default_factory=MatchParams)
    tune: TuneParams = field(default_factory=TuneParams)

    @staticmethod
    def from_flat(flat):
        """{'body.cam_height': 0.35, ...} の平坦な辞書から組み立てる。

        ROS のパラメータはドット区切りで来るので、その形をそのまま受ける。
        知らないキーは黙って捨てるだけなので、ノード側が宣言済みのものしか
        渡さない前提にしてある。
        """
        return DetectorParams(
            body=_from_dict(BodyParams, flat, 'body.'),
            match=_from_dict(MatchParams, flat, 'match.'),
            tune=_from_dict(TuneParams, flat, 'tune.'),
        )
