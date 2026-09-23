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
    高さの境界と obj_width_max は ROBO-ONE Auto の規定を当たって置き直すこと。
    bag に写ったミニロボット (上端 26.8 cm・幅 23.2 cm) は「この値でミニロボも
    取れるか」の下限側の確認にだけ使う。

    **高さの境界は絶対値で置く** (2026-09-23)。以前は転倒の判定だけ行動層にあって、
    「はじめ」直後に測った立位高さ H_o に対する比 (0.5 / 0.75) で決めていた。
    比は較正が当たっているかどうかで意味が変わり、会場で yaml を見ても何が起きるか
    読めない。クラスタ上端 z_top の絶対値で 3 つに割り、当日はこの 3 つの数字だけを
    動かす (docs/相手機の認識.md §5)。

        z_top < obj_top_min                    … 床のゴミ・ケーブル。相手にしない
        obj_top_min ≦ z_top < fallen_top_max   … 転倒した二足機。RETREAT の対象
        fallen_top_max ≦ z_top < robot_top_max … 立っている二足機。攻撃の対象
        robot_top_max ≦ z_top                  … 人・人の腕・什器。相手にしない

    上の帯 (tune.obj_h_hi) で点を切っているので、背の高い人の z_top はその高さで
    飽和する。飽和しても robot_top_max は超えるので「人」に落ちる。
    """

    #: [m] クラスタ上端高さの下限。これ未満は床のゴミ
    obj_top_min: float = 0.10
    #: [m] ここまでは「転倒した二足機」。bag のミニロボットは立位で 26.8 cm なので、
    #: これを 0.27 より上げると立っているミニロボが転倒に化ける
    fallen_top_max: float = 0.25
    #: [m] ここまでは「立っている二足機」= 攻撃の対象。これ以上は人・什器とみなす。
    #: 第 44 回規則に身長の上限規定は無いので、背の高い出場機を人と誤判定しない
    #: 下限でもある。上げれば人を拾いやすくなり、下げれば相手を見落とす
    robot_top_max: float = 0.60
    #: [m] 水平方向の広がり max(w, d) の下限 / 上限
    obj_width_min: float = 0.05
    #: 第 44 回規則 表 2: 3 kg 以下級の腕は軸から 30 cm まで。両腕を広げた幅 (2026-09-20)
    obj_width_max: float = 0.80
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
    #: [m] 占有グリッドのセル幅。**判定の解像度そのもの** (表示だけの話ではない)。
    #: リング面の連結・崖・塊の足もとは全部この格子の上で決まる。
    #: 小さくすると細かくなるが、セル数が 1/cell^2 で増えて計算量が上がる
    #: (Pi 5・合成シーンで 0.05 -> 13.6 ms / 0.025 -> 19.4 ms / 0.02 -> 23.3 ms)。
    #: **セル数で効く量はここに書かない。** 長さ [m] と広さ [m^2] で持って、
    #: 下の *_cells が cell から換算する。cell だけ変えても意味がずれない
    cell: float = 0.025
    #: [m] 1 セルぶんの穴を埋めるモルフォロジの半径。占有の close と、
    #: 崖の遮蔽に使う「面より上」の膨張の両方。cell = 0.05 のころの 1 セル
    morph_radius: float = 0.05
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
    #: 種の窓を「床が写り始める距離」から自動で置く (2026-09-20)。水平付けのカメラは
    #: 垂直画角の下端 (俯角 29 度) より手前の床が写らないので、高さ 0.365 m なら 0.66 m
    #: より近い床は無い。固定の 0.15〜0.70 m では窓が常に空になる。true のとき窓は
    #:   near = max(seed_near, 写り始め + 1 セル),  far = near + (seed_far - seed_near)
    #: 写り始めは cam_height / tan(俯角 + 画角の下半分)。geometry.floor_visible_from
    seed_auto: bool = True
    #: 種の窓が空だったとき最大成分に落ちる。手持ち検証用の逃げ道 (§7)
    seed_fallback_to_largest: bool = True
    #: [m] 面より上の物の背後を、この長さまで「影」としてリング成分の連結に使う
    #: (2026-09-20)。近い相手は見えている床を左右 2 つに分断し、種の窓も影に入る。
    #: 影をまたいでつながないと片側しかリングとして拾えない。影のセルは連結に使う
    #: だけで、リング面の面積にも縁にも数えない。場外の床 (面より下の点) が写って
    #: いるセルは影にしない。0 で無効
    shadow_bridge: float = 1.5
    #: [m] これより下に点が写っているセルは「場外の床が見えている」として影にしない
    below_band: float = 0.10
    #: [m^2] 影をまたいで足す床の成分の最小の広さ。物の垂直な面が床の帯に落とす
    #: 数セルを床と取り違えないため (pipeline._bridge_shadows)
    bridge_min_area: float = 0.015

    # --- 物体の抽出 (§8) --------------------------------------------------
    #: [m] 面のすぐ上の無視量 h_lo。面推定の残差と鉛直誤差から決まる量で、相手の大きさに依らない
    obj_h_lo: float = 0.040
    #: [m] 候補にする高さの上限 h_hi。**MatchParams.robot_top_max より高く取る**。
    #: ここで点を切るとクラスタの z_top がこの高さで飽和するので、これが
    #: robot_top_max と同じだと「人」と「立っている機体」が区別できなくなる。
    #: 1.20 は水平付け・カメラ高さ 0.365 m で距離 1.51 m の人の見える上端 (§4 の視野)
    obj_h_hi: float = 1.20
    #: [m] 塊を切るボクセルの高さ。**cell と揃えて立方体にしてある** (2026-09-23)。
    #: 塊は真上から見た 2 次元ではなく、**高さも入れた 26 近傍**でつなぐ
    #: (grid.label_voxels)。相手の真上に浮いたノイズを同じ塊に入れないため。
    #: 深度の点の間隔は距離 3 m・stride=2 でも 15 mm なので、25 mm はまだ面が
    #: ばらばらに割れない側。小さくすると相手が割れ、大きくすると浮いたノイズを
    #: 拾い直す (この高さより近くに浮いた点は同じ塊に入る)
    cell_z: float = 0.025
    #: [m^2] 塊の真上に別の塊がこれだけの広さぶん乗っていたら相手にしない。0 で無効。
    #: レフリーの腕が相手の上を通っている間は、そこへ踏み込むべきではない
    #: (docs/相手機の認識.md §6)。浮いたノイズ数ボクセルでは立たない広さにする
    overhead_min_area: float = 0.010
    #: [m^2] クラスタに要求する最小の広さ
    min_area: float = 0.005
    #: 距離依存の最小点数 N_min(r) = N0 (r0/r)^2。式 (14) の写像から (§8.2)
    min_points_n0: int = 60
    min_points_r0: float = 1.0
    #: 距離によらない最低ライン。遠方で N_min が落ちすぎないように
    min_points_floor: int = 12
    #: [m] リング成分をこれだけ膨張させた領域まで候補を許す (§8.1)
    ring_dilate: float = 0.05
    #: 「リングの内側」を、見えたリング面と自機の位置 (0,0) の凸包で決める (2026-09-20)。
    #: false なら従来どおり「見えたリング面の隣まで」。リングは凸 (八角形) で自機は
    #: その上に立っているので、凸包は必ずリングの中に収まる。これが要る理由は 2 つ:
    #:   * 水平付けのカメラは足元の床が写らない。写り始めより近い相手は足元に床が
    #:     見えず、従来の規則だと丸ごと候補から落ちる (間合いで見失う)
    #:   * 相手の影で背後の床が欠け、天面や奥の面の点が候補から外れて重心が手前に偏る
    ring_hull: bool = True

    # --- 外から差し込む塊の棄却 (docs/相手機の認識.md §6 の門 3) ------------
    #: [m] リングの縁から外へこの幅の帯の中で「外の物」を探す
    #: (規則: 操縦者はリングから 50 cm 以上離れる。腕から胴体までたどれる幅)
    intrude_band: float = 0.50
    #: [m^2] 内側の塊からつながる「外の物」がこの広さ以上なら、外から差し込んでいる
    #: (レフリーの腕など) として相手にしない。0 で無効
    intrude_min_area: float = 0.010
    #: 「外の物」とみなすセルの最小点数。深度の段差に出る飛び画素を数えないため。
    #: **これだけはセルの大きさで割らない** (飛び画素は面積ではなく点の数の話)。
    #: cell を小さくすると 1 セルに入る点が減るので、下げないと帯が空になる
    intrude_cell_points: int = 2

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

    # --- セル数への換算 ----------------------------------------------------
    # 長さ [m] と広さ [m^2] で持っている量を、いまの cell でセル数に直す。
    # **呼ぶ側は今までどおり *_cells を読む。** こうしてあるので、cell を変えても
    # 帯の幅や塊の最小の広さといった物理的な意味は動かない (2026-09-23)。
    # 0 は「無効」の意味なので、0 のときは 0 のまま返す。

    def _len_cells(self, metres):
        return 0 if metres <= 0 else max(1, int(round(metres / self.cell)))

    def _area_cells(self, area):
        return 0 if area <= 0 else max(1, int(round(area / (self.cell * self.cell))))

    @property
    def morph_cells(self):
        return max(1, self._len_cells(self.morph_radius))

    @property
    def ring_dilate_cells(self):
        return self._len_cells(self.ring_dilate)

    @property
    def intrude_band_cells(self):
        return self._len_cells(self.intrude_band)

    @property
    def min_cells(self):
        return max(1, self._area_cells(self.min_area))

    @property
    def intrude_min_cells(self):
        return self._area_cells(self.intrude_min_area)

    @property
    def bridge_min_cells(self):
        return max(1, self._area_cells(self.bridge_min_area))

    @property
    def overhead_min_cells(self):
        return self._area_cells(self.overhead_min_area)


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
