# -*- coding: utf-8 -*-
"""検出パイプラインの単体テスト。

docs/opponent_detection.pdf が実測から導いた判断を、合成シーン (scene.py) で
再現できる形にしたもの。狙いは「文書が挙げた失敗の形に落ちないこと」の固定で、
実機での正しさを主張するものではない (正解ラベルが無い件は §12)。

    §6.1  最大平面は場外の床とリング面を行き来する  → 高さ窓なら取り違えない
    §8.1  制限を外すと場外クラッタが候補に入り、相手と融合して重心が場外へ引かれる
    §7    視野の縁をリングの端として報せない
    §5    傾いても相手の位置が動かない
    §9.2  「見えない」を 1 つに潰さない

ROS は要らない。roboone_walk_ref の walk_core と同じで、計算部分だけを直接叩く。
"""

import math

import numpy as np
import pytest
from roboone_perception.detect import (AlphaBetaTracker, ATTITUDE_STALE,
                                       AttitudeEstimator, DetectorParams,
                                       forward_ref, Intrinsics,
                                       mean_accel_if_still, NO_OPPONENT,
                                       OK, ring_basis, RING_LOST, RingDetector)
from roboone_perception.detect import clusters as cl
from roboone_perception.detect import edge as ed
from roboone_perception.detect import grid as g
from roboone_perception.detect import ring as rg
from roboone_perception.sim import scene as S

# 桜木町の記録の depth 内部パラメータ (§7)
INTR = Intrinsics(640, 480, 386.9, 386.9, 325.7, 246.8)
CAM_H = 0.35
DT = 1.0 / 30.0
#: D435 系の深度雑音の目安 (σ = noise·z²)
NOISE = 0.0015


def run(scene, frames=3, params=None, **kw):
    """シーンを frames 枚流して最後の結果を返す。姿勢の収束に数枚要る。"""
    depth = S.render(scene, INTR, cam_height=kw.pop('cam_height', CAM_H),
                     noise=kw.pop('noise', NOISE), **kw)
    det = RingDetector(params)
    res = None
    for _ in range(frames):
        res = det.step(depth, INTR, DT)
    return det, res


def opponent(x=1.06, y=0.0, w=0.23, d=0.20, h=0.27):
    return S.Box(x, y, w, d, 0.0, h)


# ------------------------------------------------------ シーンの前提の確認
def test_scene_has_two_layers():
    """合成シーンがリング面と場外の床の 2 層 (段差 34 cm) を作れていること。"""
    depth = S.render(S.Scene(), INTR, cam_height=CAM_H, noise=0.0)
    det = RingDetector()
    res = det.step(depth, INTR, DT)
    assert res.ring_height == pytest.approx(-CAM_H, abs=0.01)


# ------------------------------------------------------ §6 リング面の高さ
def test_height_window_beats_largest_layer():
    """場外の床の方が大きく写っていても、リング面を取り違えないこと。

    §6.1 の「最大平面は 2 つの面を行き来する」の再現。同じフレームで
    「最も点数の多い層」を採ると場外の床 (-0.69 m) に貼り付くが、
    設計上のカメラ高さの近くを見る方式 (式 9) はリング面 (-0.35 m) を採る。
    """
    # リングの前端 0.3 m。視野のほとんどが場外の床になる立ち位置
    sc = S.Scene(center=(-1.5, 0.0))
    det, res = run(sc)

    from roboone_perception.detect.geometry import Deprojector, to_plane
    pts, _ = Deprojector(INTR, 2, 4)(S.render(sc, INTR, cam_height=CAM_H,
                                              noise=NOISE))
    u = det.attitude.u
    e1, e2 = ring_basis(u)
    h, _, _ = to_plane(pts, u, e1, e2)

    # 素朴な「最大の層」— 全域のヒストグラムの最頻ビン
    counts, edges = np.histogram(h, bins=int(3.0 / 0.01), range=(-2.0, 1.0))
    naive = 0.5 * (edges[np.argmax(counts)] + edges[np.argmax(counts) + 1])

    assert naive < -0.55, '前提: このシーンでは最大の層が場外の床であること'
    assert res.ring_height == pytest.approx(-CAM_H, abs=0.02)
    assert abs(res.ring_height - naive) > 0.25, '2 つの面は 34 cm 離れている'


def test_ring_height_survives_holes_and_noise():
    """抜けと雑音があっても h_r が動かないこと。"""
    heights = []
    for seed in range(5):
        sc = S.Scene(boxes=[opponent()])
        _, res = run(sc, noise=0.003, hole_rate=0.15, seed=seed)
        heights.append(res.ring_height)
    assert np.std(heights) < 0.005, '5 フレームの h_r のばらつきが 5 mm 未満'


# ------------------------------------------------------ §8 物体の抽出と選択
def test_detects_opponent_on_ring():
    """リング上の箱を、位置・上端高さ・幅つきで取れること。"""
    _, res = run(S.Scene(boxes=[opponent(x=1.06, y=0.60)]))
    assert res.status == OK
    assert res.valid
    # 重心は見えている面 (手前と上) に寄るので、奥行きの半分ぶん手前に出る
    assert res.position[0] == pytest.approx(1.06, abs=0.15)
    assert res.position[1] == pytest.approx(0.60, abs=0.10)
    assert res.top_height == pytest.approx(0.27, abs=0.03)
    assert res.width == pytest.approx(0.23, abs=0.10)


def test_rejects_off_ring_clutter():
    """場外の什器だけのシーンで、相手ありにしないこと (§8.1)。

    机や椅子の列は最大幅 p90 で 3.34 m あり、高さ・大きさのフィルタだけでは
    落としきれない。リング成分の内側に閉じてから連結成分を取る順序で落とす。
    """
    sc = S.Scene(boxes=[S.Box(2.6, -2.2, 1.2, 0.6, -0.34, 0.6),
                        S.Box(2.9, 1.9, 1.6, 0.5, -0.34, 0.9)])
    _, res = run(sc)
    assert res.status == NO_OPPONENT
    assert res.selected is None
    assert res.ring_area > 1.0, 'リング面自体は取れていること'


def test_ring_restriction_prevents_fusion():
    """縁際の相手が場外の什器と融合して重心が場外へ引かれないこと (§8.1)。

    リングの前端すぐ内側に相手を置き、その真後ろの場外に大きな什器を置く。
    制限を外すと 1 つの連結成分に融合し、重心が場外へ動く。
    """
    sc = S.Scene(center=(-0.8, 0.0),               # 前端は 1.0 m
                 boxes=[opponent(x=0.85, y=0.0),   # 縁の内側 15 cm
                        S.Box(1.35, 0.0, 1.4, 0.5, -0.34, 0.5)])  # 縁の外側
    _, res = run(sc)
    assert res.status == OK
    assert res.position[0] < 1.0, '重心がリングの外へ出ていないこと'
    assert res.top_height == pytest.approx(0.27, abs=0.05)


def test_nearest_is_selected():
    """条件を通ったもののうち最も近いものを採ること (§8.2)。"""
    sc = S.Scene(boxes=[opponent(x=1.9, y=-0.5, w=0.4, d=0.4, h=0.5),
                        opponent(x=1.0, y=0.3)])
    _, res = run(sc)
    assert res.status == OK
    assert res.position[0] == pytest.approx(1.0, abs=0.2)
    assert res.position[1] == pytest.approx(0.3, abs=0.15)


def test_min_points_falls_with_range():
    """点数のしきい値が距離の 2 乗で緩むこと (式 14)。"""
    n1 = cl.min_points_at(1.0, 60, 1.0, 12)
    n2 = cl.min_points_at(2.0, 60, 1.0, 12)
    assert n1 == pytest.approx(60)
    assert n2 == pytest.approx(15)
    assert cl.min_points_at(5.0, 60, 1.0, 12) == 12, '下限で止まること'


def test_oversized_object_rejected():
    """規定の上限を超える塊は相手にしないこと (§8.3 の hmax)。"""
    sc = S.Scene(boxes=[S.Box(1.2, 0.0, 1.0, 0.8, 0.0, 1.2)])
    _, res = run(sc)
    assert res.selected is None


# ------------------------------------------------------ 水平付けのカメラ
#: 848x480 の D435 (垂直画角 58 度前後)。水平付けの実機と同じ条件
INTR_H = Intrinsics(848, 480, 424.0, 424.0, 424.0, 240.0)


def run_horizontal(scene, params=None, frames=3, cam_height=0.40):
    flat = {'body.cam_pitch_deg': 0.0, 'body.cam_height': cam_height}
    flat.update(params or {})
    depth = S.render(scene, INTR_H, cam_height=cam_height, pitch_deg=0.0,
                     noise=NOISE)
    det = RingDetector(DetectorParams.from_flat(flat))
    res = None
    for _ in range(frames):
        res = det.step(depth, INTR_H, DT)
    return det, res


def test_seed_window_follows_the_visible_floor():
    """水平付けでは床が 0.72 m より先にしか写らない。種の窓がそこへ動くこと。"""
    _, res = run_horizontal(S.Scene())
    assert res.status == NO_OPPONENT, 'リング面が取れていること'
    assert res.seed_window[0] > 0.70
    assert res.ring_area > 1.0
    # 固定の窓のままだと種が空で、最大成分への逃げ道に頼ることになる
    _, res = run_horizontal(S.Scene(), {'tune.seed_auto': False,
                                        'tune.seed_fallback_to_largest': False})
    assert res.status == RING_LOST


def test_close_opponent_survives_the_floor_blind_zone():
    """床が写らない近距離の相手を落とさないこと (docs/相手機の認識.md §3)。

    0.40 m 先の相手は足元に床が 1 画素も写らない。従来の「見えたリング面の隣まで」
    では候補から丸ごと落ち、間合いに入った相手を見失う。
    """
    sc = S.Scene(boxes=[S.Box(0.45, 0.0, 0.25, 0.15, 0.0, 0.45)])
    _, res = run_horizontal(sc)
    assert res.status == OK
    assert res.position[0] == pytest.approx(0.40, abs=0.08)
    _, res = run_horizontal(sc, {'tune.ring_hull': False})
    assert res.selected is None, '凸包を切ると落ちる (これが直した不具合)'


def test_hull_stays_inside_the_ring():
    """凸包が場外へはみ出さず、縁の外の什器を候補に入れないこと。"""
    sc = S.Scene(center=(-0.8, 0.0),
                 boxes=[S.Box(1.35, 0.0, 1.4, 0.5, -0.34, 0.5)])
    _, res = run(sc)
    assert res.selected is None
    assert not res.clusters or all(c.fwd < 1.1 for c in res.clusters)


# ------------------------------------- 格子の細かさ (2026-09-23)
def _bfs_labels(mask):
    """4 近傍の連結成分を素朴な BFS で数える参照実装。"""
    lab = np.zeros(mask.shape, np.int32)
    n = 0
    for r in range(mask.shape[0]):
        for c in range(mask.shape[1]):
            if mask[r, c] and lab[r, c] == 0:
                n += 1
                stack = [(r, c)]
                lab[r, c] = n
                while stack:
                    y, x = stack.pop()
                    for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        q, w = y + dy, x + dx
                        if (0 <= q < mask.shape[0] and 0 <= w < mask.shape[1]
                                and mask[q, w] and lab[q, w] == 0):
                            lab[q, w] = n
                            stack.append((q, w))
    return lab, n


def _same_partition(a, b, mask):
    """ラベルの番号は違ってよい。分け方が同じであることだけを見る。"""
    seen = {}
    for r in range(mask.shape[0]):
        for c in range(mask.shape[1]):
            if mask[r, c] and seen.setdefault(int(a[r, c]), int(b[r, c])) != b[r, c]:
                return False
    return True


def test_label_components_matches_a_reference():
    """ラベリングの中身を入れ替えたので、素朴な実装と突き合わせる。

    セルを細かくすると占有セル数が 1/cell^2 で増え、ここが段の中で一番重くなる。
    いまは cv2 があればそれを使い、無ければ numpy 版に落ちる。**どちらも**
    素朴な BFS と同じ分け方をすること。
    """
    rng = np.random.default_rng(7)
    for _ in range(20):
        m = rng.random((int(rng.integers(4, 40)),
                        int(rng.integers(4, 40)))) < rng.uniform(0.2, 0.7)
        want, n_want = _bfs_labels(m)
        for got, n_got in (g.label_components(m), g._label_components_numpy(m)):
            assert n_got == n_want
            assert _same_partition(got, want, m)


def test_cell_size_does_not_move_the_physical_thresholds():
    """セルの大きさを変えても、セル数で効く門の**物理的な意味**が動かないこと。

    帯の幅や塊の最小の広さは長さ [m] / 広さ [m^2] で持っていて、params が
    いまの cell で割る。ここが崩れると「格子を細かくしただけ」のつもりで
    棄却の条件まで変わる。
    """
    a = DetectorParams.from_flat({'tune.cell': 0.05}).tune
    b = DetectorParams.from_flat({'tune.cell': 0.025}).tune
    assert a.intrude_band_cells * a.cell == pytest.approx(
        b.intrude_band_cells * b.cell, abs=0.01)
    assert a.ring_dilate_cells * a.cell == pytest.approx(
        b.ring_dilate_cells * b.cell, abs=0.01)
    for name in ('min_cells', 'intrude_min_cells', 'bridge_min_cells',
                 'overhead_min_cells'):
        assert getattr(a, name) * a.cell ** 2 == pytest.approx(
            getattr(b, name) * b.cell ** 2, rel=0.05), name
    # 0 は「無効」のまま。広さで割って 1 に化けない
    off = DetectorParams.from_flat({'tune.cell': 0.025,
                                    'tune.overhead_min_area': 0.0,
                                    'tune.intrude_min_area': 0.0}).tune
    assert off.overhead_min_cells == 0 and off.intrude_min_cells == 0


def test_a_finer_grid_finds_the_same_opponent():
    """格子を細かくしても、相手の位置・上端・種別が変わらないこと。"""
    sc = S.Scene(boxes=[opponent(h=0.40)])
    out = []
    for cell in (0.05, 0.025):
        _, res = run(sc, params=DetectorParams.from_flat({'tune.cell': cell}))
        assert res.status == OK, cell
        out.append(res)
    assert out[1].position[0] == pytest.approx(out[0].position[0], abs=0.04)
    assert out[1].position[1] == pytest.approx(out[0].position[1], abs=0.04)
    assert out[1].top_height == pytest.approx(out[0].top_height, abs=0.03)
    assert out[1].kind == out[0].kind
    # 細かいほうがセル数は増える (同じ物を細かく見ている)
    assert out[1].selected.n_cells > out[0].selected.n_cells


# ------------------------------------- 3 次元のクラスタリング (2026-09-23)
def _block(u0, u1, v0, v1, z0, z1, step=0.01):
    """直方体の面を点で埋める。リング平面座標 (前方 u, 左 v, 高さ z) で返す。"""
    uu, vv, zz = np.meshgrid(np.arange(u0, u1 + 1e-9, step),
                             np.arange(v0, v1 + 1e-9, step),
                             np.arange(z0, z1 + 1e-9, step), indexing='ij')
    return zz.ravel(), uu.ravel(), vv.ravel()


def _extract(pieces, tune=None):
    """点の塊をいくつか渡して extract() を直接叩く。リング面は全面とする。"""
    spec = g.GridSpec(cell=0.05, u_min=-0.5, u_max=4.0, v_min=-3.0, v_max=3.0)
    t = tune or DetectorParams().tune
    h = np.concatenate([p[0] for p in pieces])
    fwd = np.concatenate([p[1] for p in pieces])
    left = np.concatenate([p[2] for p in pieces])
    ring = np.ones(spec.shape, dtype=bool)
    found, _, _ = cl.extract(spec, h, fwd, left, 0.0, ring, t)
    return found


def test_noise_floating_above_the_opponent_does_not_raise_the_top():
    """相手の真上に浮いたノイズを同じ塊に入れないこと。

    真上から見た 2 次元の連結だと、0.90 m に 1 点あるだけで上端が 0.90 m になり、
    立っている相手が「人・什器」に化ける。高さもボクセルで切れば別の塊になる。
    """
    robot = _block(0.95, 1.15, -0.10, 0.10, 0.05, 0.40)
    noise = (np.array([0.90, 0.88]), np.array([1.05, 1.06]),
             np.array([0.0, 0.01]))
    found = _extract([robot, noise])
    p = DetectorParams()
    best, ok = cl.select(found, p.match, p.tune)
    assert best is not None
    assert best.top_height == pytest.approx(0.40, abs=0.03), '上端はノイズに触らない'
    assert best.kind == cl.ROBOT_STANDING
    # ノイズは別の塊として残るが、セル数と点数の門で落ちる
    others = [c for c in found if c is not best]
    assert others, 'ノイズは別の塊になっていること'
    assert all(cl.reject_reason(c, p.match, p.tune) for c in others)


def test_the_opponent_itself_is_not_split_by_the_voxels():
    """ボクセルで切っても相手が割れないこと。

    深度の点の間隔は距離 3 m・stride=2 でも 15 mm で、ボクセル 50 mm より細かい。
    ここでは 10 mm 刻みで面を作って、縦にも横にも 1 つの塊になることを見る。
    """
    found = _extract([_block(0.95, 1.15, -0.10, 0.10, 0.05, 0.40)])
    big = [c for c in found if c.n_cells >= 4]
    assert len(big) == 1, '塊は 1 つ'
    assert big[0].height_min < 0.10 and big[0].top_height > 0.38


def test_something_above_the_opponent_is_not_a_target():
    """相手の真上に物が乗っている間は相手にしないこと（意図した挙動）。

    レフリーの腕が上を通っているときにそこへ踏み込まない。ノイズと違って
    広さが立つので、overhead_min_area で切り分けられる。
    """
    robot = _block(0.95, 1.15, -0.10, 0.10, 0.05, 0.40)
    arm = _block(0.95, 1.15, -0.10, 0.10, 0.70, 0.75)   # 30 cm 上に板
    found = _extract([robot, arm])
    p = DetectorParams()
    low = [c for c in found if c.top_height < 0.60]
    assert low, '相手そのものは塊として取れている'
    assert low[0].covered and low[0].overhead_cells >= p.tune.overhead_min_cells
    assert '上に物が乗っている' in cl.reject_reason(low[0], p.match, p.tune)
    assert cl.select(found, p.match, p.tune)[0] is None
    # 門は切れる。切れば同じ相手が通る
    t = DetectorParams.from_flat({'tune.overhead_min_area': 0.0}).tune
    found = _extract([robot, arm], tune=t)
    low = [c for c in found if c.top_height < 0.60]
    assert not low[0].covered


def test_a_single_noise_voxel_does_not_count_as_something_above():
    """上に乗っているかの判定は、ノイズ 1 点では立たないこと。"""
    robot = _block(0.95, 1.15, -0.10, 0.10, 0.05, 0.40)
    noise = (np.array([0.90]), np.array([1.05]), np.array([0.0]))
    found = _extract([robot, noise])
    p = DetectorParams()
    best, _ = cl.select(found, p.match, p.tune)
    assert best is not None and not best.covered
    assert best.overhead_cells < p.tune.overhead_min_cells


# --------------------------------------- 高さの絶対値で種別を割る (2026-09-23)
def test_standing_opponent_is_classified_as_a_robot():
    """立っている相手が ROBOT_STANDING で出ること。"""
    _, res = run(S.Scene(boxes=[opponent(h=0.40)]))
    assert res.status == OK
    assert res.kind == cl.ROBOT_STANDING
    assert res.selected.kind == cl.ROBOT_STANDING


def test_fallen_opponent_is_classified_but_still_tracked():
    """倒れた相手は ROBOT_FALLEN になり、**相手として追い続ける**こと。

    転倒した機体を候補から落とすと位置が出ず、規則 10.2(b)(i) で離れる判断も
    できなくなる。落とすのは人と床のゴミだけ。
    """
    _, res = run(S.Scene(boxes=[S.Box(1.06, 0.0, 0.40, 0.30, 0.0, 0.18)]))
    assert res.status == OK, '転倒した相手も相手として出る'
    assert res.kind == cl.ROBOT_FALLEN
    assert res.top_height < DetectorParams().match.fallen_top_max


def test_debris_below_the_floor_of_the_band_is_not_an_opponent():
    """obj_top_min を割る低い塊は相手にしない (種別は NOISE)。"""
    _, res = run(S.Scene(boxes=[S.Box(1.06, 0.0, 0.30, 0.25, 0.0, 0.07)]))
    assert res.selected is None
    p = DetectorParams()
    low = [c for c in res.clusters if cl.classify(c, p.match) == cl.NOISE]
    assert low and '低すぎる' in cl.reject_reason(low[0], p.match, p.tune)


def test_person_standing_on_the_ring_is_rejected_as_human():
    """リングの上に立つ人 (外の物につながっていない) を高さだけで落とすこと。

    門 3 (外から差し込む塊) は「外の胴体につながっているか」で見るので、
    縁の内側で完結して見える塊には効かない。そこを高さが受ける。
    """
    sc = S.Scene(boxes=[S.Box(1.2, 0.0, 0.35, 0.30, 0.0, 0.90)])
    _, res = run_horizontal(sc)
    assert res.selected is None
    p = DetectorParams()
    tall = [c for c in res.clusters if c.top_height > p.match.robot_top_max]
    assert tall, '塊としては取れていること'
    assert cl.classify(tall[0], p.match) == cl.HUMAN
    assert '人・什器' in cl.reject_reason(tall[0], p.match, p.tune)


def test_the_point_band_reaches_above_the_human_boundary():
    """tune.obj_h_hi が match.robot_top_max より高いこと。

    帯で点を切ると z_top がその高さで飽和する。帯 = 境界にすると「人」と
    「背の高い機体」が同じ値になって区別が付かなくなるので、帯は上に取る。
    """
    p = DetectorParams()
    assert p.tune.obj_h_hi > p.match.robot_top_max
    sc = S.Scene(boxes=[S.Box(1.2, 0.0, 0.35, 0.30, 0.0, 0.90)])
    _, res = run_horizontal(sc)
    tall = max(res.clusters, key=lambda c: c.top_height)
    assert tall.top_height == pytest.approx(0.90, abs=0.06), '0.90 で測れている'


def test_the_bands_come_from_the_parameters():
    """境界は yaml で動かせること。会場で追い込むのはこの 3 つだけ。"""
    sc = S.Scene(boxes=[S.Box(1.2, 0.0, 0.35, 0.30, 0.0, 0.90)])
    _, res = run_horizontal(sc, {'match.robot_top_max': 1.00})
    assert res.status == OK, '境界を上げれば同じ塊が相手として通る'
    assert res.kind == cl.ROBOT_STANDING
    # 転倒の境界も同じ。立っているミニロボットを転倒に化けさせられる
    sc = S.Scene(boxes=[opponent(h=0.268)])
    _, res = run(sc)
    assert res.kind == cl.ROBOT_STANDING, 'bag のミニロボットは既定では立位'
    _, res = run(sc, params=DetectorParams.from_flat({'match.fallen_top_max': 0.30}))
    assert res.kind == cl.ROBOT_FALLEN


# ------------------------------------------------------ 外から差し込む塊
def _referee_reaching_in():
    """左の縁 (y = +0.6) の外に立つ人が、リングの上へ腕を伸ばしている。"""
    person = S.Box(1.2, 0.95, 0.4, 0.3, -0.34, 1.3)
    arm = S.Box(1.2, 0.50, 0.6, 0.08, 0.25, 0.33)      # y = 0.2〜0.8
    return S.Scene(center=(0.0, -1.2), boxes=[person, arm])


def test_intruding_arm_is_not_an_opponent():
    """リングの外から差し込む腕を相手にしないこと (門 3)。"""
    _, res = run(_referee_reaching_in())
    assert res.status == NO_OPPONENT
    arms = [c for c in res.clusters if c.intruding]
    assert arms, '腕の塊は取れていて、外から差し込んでいると判定されること'
    p = DetectorParams()
    assert '外から' in cl.reject_reason(arms[0], p.match, p.tune)


def test_intrusion_gate_can_be_switched_off():
    """門 3 を切ると、同じ腕が相手として通ってしまうこと (門が効いている証拠)。"""
    _, res = run(_referee_reaching_in(),
                 params=DetectorParams.from_flat({'tune.intrude_min_area': 0.0}))
    assert res.status == OK
    assert res.position[1] > 0.1


def test_opponent_at_the_edge_is_not_an_intruder():
    """縁に立つ相手と、縁の外に立つ人が離れていれば、相手は落ちないこと。"""
    person = S.Box(1.2, 1.05, 0.4, 0.3, -0.34, 1.3)      # 縁から 25 cm 外
    robot = S.Box(1.2, 0.45, 0.23, 0.20, 0.0, 0.40)      # 縁の内側
    _, res = run(S.Scene(center=(0.0, -1.2), boxes=[person, robot]))
    assert res.status == OK
    assert res.position[1] == pytest.approx(0.45, abs=0.12)
    assert not res.selected.intruding


# ------------------------------------------------------ §5 姿勢
def test_tilt_does_not_move_the_opponent():
    """機体が傾いても相手の位置と上端高さが動かないこと。

    リング面の法線で鉛直を取り直しているので、傾いた分は姿勢に吸収される。
    ここが加速度の生値なら、歩行時 p95 28° の傾き誤差がそのまま高さに化ける。
    """
    box = opponent(x=1.10, y=0.25)
    _, flat = run(S.Scene(boxes=[box]), frames=6)
    _, tilted = run(S.Scene(boxes=[box]), frames=6, roll_deg=8.0)
    assert tilted.status == OK
    assert tilted.position[0] == pytest.approx(flat.position[0], abs=0.04)
    assert tilted.position[1] == pytest.approx(flat.position[1], abs=0.04)
    assert tilted.top_height == pytest.approx(flat.top_height, abs=0.03)


def test_roll_does_not_swing_the_bearing():
    """ロールで方位が振れないこと (geometry.ring_basis の注記)。

    e1 を光軸の水平成分に取ると、俯角 30 度・ロール 8 度で方位が 4.6 度ずれる。
    機体前方から作れば振れない。行動層はこの方位で相手へ向かうので、ここが
    振れると歩きながら常に舵を切り続けることになる。
    """
    box = opponent(x=1.10, y=0.25)
    _, flat = run(S.Scene(boxes=[box]), frames=6)
    _, tilted = run(S.Scene(boxes=[box]), frames=6, roll_deg=8.0)
    b_flat = math.degrees(math.atan2(flat.position[1], flat.position[0]))
    b_tilt = math.degrees(math.atan2(tilted.position[1], tilted.position[0]))
    assert abs(b_flat - b_tilt) < 1.5

    # 基準を光軸に取ると、同じ姿勢で 4 度以上ずれる (直したのはここ)
    u = np.array([0.0, -math.cos(math.radians(8.0)), 0.0])
    u = S.rot_x(math.radians(8.0)) @ S.mount_matrix(30.0)
    u = u.T @ np.array([0.0, 0.0, 1.0])
    e1_axis, _ = ring_basis(u)
    e1_body, _ = ring_basis(u, forward_ref(30.0))
    swing = math.degrees(math.acos(min(1.0, float(np.dot(e1_axis, e1_body)))))
    assert swing > 4.0


def test_attitude_converges_to_plane_normal():
    """初期姿勢がずれていても、面法線が数フレームで引き戻すこと (式 8)。"""
    depth = S.render(S.Scene(boxes=[opponent()]), INTR, cam_height=CAM_H,
                     noise=NOISE, roll_deg=6.0)
    det = RingDetector()
    angles = []
    for _ in range(10):
        res = det.step(depth, INTR, DT)
        # 真の鉛直 (カメラ座標) は roll 6 度ぶん傾いた向き
        truth = S.rot_x(math.radians(6.0)) @ S.mount_matrix(30.0)
        truth = truth.T @ np.array([0.0, 0.0, 1.0])
        angles.append(math.degrees(math.acos(
            min(1.0, abs(float(np.dot(res.up, truth)))))))
    assert angles[-1] < 0.5, '最終的に 0.5 度以内へ収束すること'
    assert angles[-1] < angles[0], '単調に良くなっていること'


def test_gyro_prediction_rotates_up_vector():
    """ジャイロ積分が慣性固定ベクトルを正しく運ぶこと (式 7)。"""
    est = AttitudeEstimator((0.0, -1.0, 0.0))
    # センサが x 軸まわりに +90 度回ると、慣性に固定された上向きは
    # センサ座標では -90 度回って見える
    est.predict((math.radians(90.0), 0.0, 0.0), 1.0)
    assert np.allclose(est.u, [0.0, 0.0, 1.0], atol=1e-6)


def test_correction_gate_rejects_bad_planes():
    """残差が大きい面と、大きく食い違う法線を採らないこと (§5.4 の門)。"""
    est = AttitudeEstimator((0.0, -1.0, 0.0), resid_max=0.010,
                            angle_max_deg=12.0)
    assert not est.correct((0.0, -1.0, 0.0), resid=0.05), '残差で落ちること'
    assert not est.correct((0.0, -math.cos(math.radians(30)),
                            -math.sin(math.radians(30))), resid=0.001), \
        '30 度ずれた法線で落ちること'
    assert est.correct((0.0, -1.0, 0.0), resid=0.001), '正常な面は通ること'
    assert est.since_correction == 0


def test_attitude_goes_stale_without_correction():
    """補正が入らないフレームが続けば劣化と分かること (§9.2 の 2 番目)。"""
    est = AttitudeEstimator((0.0, -1.0, 0.0), stale_frames=3)
    assert not est.stale
    for _ in range(4):
        est.missed()
    assert est.stale


def _tilted(u, deg):
    """鉛直 u をカメラの x 軸まわりに deg 度回したもの (= 俯角の読み違い)。"""
    a = math.radians(deg)
    r = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)],
                  [0, math.sin(a), math.cos(a)]])
    return r @ np.asarray(u)


def test_attitude_cannot_recover_by_itself_but_reset_does():
    """鉛直が門 (12 度) より大きく外れると自力では戻れず、取り直せば戻ること。

    2026-09-20 の実機のテストランで起きた形。脱力した姿勢で起動すると、立ち上がる
    間は床が見えず引き戻しが効かない。立ったあとは正しい床の法線が門で弾かれ続ける。
    """
    sc = S.Scene(boxes=[opponent(x=1.06, y=0.2)])
    depth = S.render(sc, INTR, cam_height=CAM_H, noise=NOISE)
    det = RingDetector()
    det.step(depth, INTR, DT)
    det.attitude.u = _tilted(det.attitude.u, 25.0)
    for _ in range(20):
        res = det.step(depth, INTR, DT)
    assert res.status in (ATTITUDE_STALE, RING_LOST), '自力では戻れない'

    # 静止中の加速度 (= 上向き) から置き直す。取り付けの鉛直と同じ向き
    assert det.reset_reference(accel=9.81 * np.asarray(det.p.body.up_from_mount))
    for _ in range(3):
        res = det.step(depth, INTR, DT)
    assert res.status == OK
    assert res.position[0] == pytest.approx(1.06, abs=0.15)


def test_reset_without_imu_falls_back_to_the_mount():
    """IMU が無くても、取り付けから決まる鉛直へ戻せること。"""
    det = RingDetector()
    det.attitude.u = _tilted(det.attitude.u, 25.0)
    assert det.reset_reference(accel=None)
    assert np.allclose(det.attitude.u, det.p.body.up_from_mount)


def test_reference_is_only_taken_while_standing_still():
    """動いている間の加速度では基準姿勢を取らないこと (§5.2: 歩行中は 28 度ずれる)。"""
    def imu(gyro, acc=(0.0, -9.81, 0.0), n=100):
        return [(0.005 * k, acc, gyro) for k in range(n)]

    acc, why = mean_accel_if_still(imu(0.01))
    assert why == '' and acc[1] == pytest.approx(-9.81)
    acc, why = mean_accel_if_still(imu(0.8))
    assert acc is None and '動いている' in why
    acc, why = mean_accel_if_still(imu(0.01, acc=(3.0, -11.5, 0.0)))
    assert acc is None and '重力' in why
    acc, why = mean_accel_if_still(imu(0.01, n=10))
    assert acc is None and '足りない' in why
    assert mean_accel_if_still([])[0] is None


def test_plane_fit_is_restricted_to_near_points():
    """面あてはめが水平 1.5 m 以内に限られていること (§5.4 の 2 つ目の罠)。"""
    det, res = run(S.Scene(boxes=[opponent()]))
    assert res.plane_resid < 0.005, '平らな面なら残差はミリ級'
    assert res.plane_corrected


# ------------------------------------------------------ §7 リングの縁
def test_cliff_matches_truth_ahead():
    """正面の縁までの距離が真値と合うこと。"""
    for offset, truth in ((0.0, 1.8), (-1.3, 0.5), (-0.8, 1.0)):
        _, res = run(S.Scene(center=(offset, 0.0)))
        bear = np.degrees(ed.bin_bearings(res.cliff.size, 45.0))
        ahead = res.cliff[np.abs(bear) < 8.0]
        assert np.isfinite(ahead).sum() >= 8, '正面のビンが埋まっていること'
        assert np.nanmedian(ahead) == pytest.approx(truth, abs=0.08)


def test_cliff_is_nan_outside_the_field_of_view():
    """視野の外はリングの端ではなく NaN であること (§7)。

    リングは視野より広いので、左右の「境界」は多くの場合リングの端ではなく
    見えている範囲の端になる。そのままの値を載せると、リングの中央にいるのに
    縁が近いと誤って伝わる。
    """
    _, res = run(S.Scene())
    bear = np.degrees(ed.bin_bearings(res.cliff.size, 45.0))
    # 水平画角は 79.2 度 (±39.6 度)。その外側は必ず NaN
    assert np.all(np.isnan(res.cliff[np.abs(bear) > 42.0]))
    assert np.isfinite(res.cliff[np.abs(bear) < 30.0]).any()


def test_cliff_is_nan_behind_an_occluder():
    """相手の影を崖と読まないこと。"""
    sc = S.Scene(center=(-0.8, 0.0), boxes=[opponent(x=0.6, y=0.0)])
    _, res = run(sc)
    bear = np.degrees(ed.bin_bearings(res.cliff.size, 45.0))
    behind = res.cliff[np.abs(bear) < 6.0]
    assert np.all(np.isnan(behind)), '影の方位は「見えていない」であること'


# ------------------------------------------------------ §9.2 縮退
def test_status_ring_lost_when_nothing_visible():
    """床が無い (全画素が無効) ときは RING_LOST。"""
    det = RingDetector()
    blank = np.zeros((INTR.height, INTR.width), dtype=np.uint16)
    res = det.step(blank, INTR, DT)
    assert res.status == RING_LOST
    assert not res.valid


def test_status_ring_lost_when_looking_off_the_ring():
    """リング面が高さの窓に入らない向き (真下が場外の床) でも RING_LOST。"""
    det = RingDetector()
    # カメラを 1 m の高さに置くと、リング面は窓 (-0.35±0.25) から外れる
    depth = S.render(S.Scene(), INTR, cam_height=1.0, noise=NOISE)
    for _ in range(3):
        res = det.step(depth, INTR, DT)
    assert res.status == RING_LOST


def test_status_distinguishes_no_opponent_from_broken():
    """相手がいないだけの正常な状態は NO_OPPONENT であること。"""
    _, res = run(S.Scene())
    assert res.status == NO_OPPONENT
    assert res.ring_area > 1.0
    assert not res.valid, 'valid は「相手が見えている」のときだけ'


def test_status_attitude_stale_is_reported():
    """面あてはめが通らない状態が続けば ATTITUDE_STALE を出すこと。"""
    p = DetectorParams()
    p.tune.fit_resid_max = 1e-9      # どんな面も門を通らないようにする
    p.tune.stale_frames = 2
    _, res = run(S.Scene(boxes=[opponent()]), frames=6, params=p)
    assert res.status == ATTITUDE_STALE


# ------------------------------------------------------ §9.1 追尾
def test_tracker_smooths_and_estimates_velocity():
    """等速で動く相手の速度を追えること。"""
    t = AlphaBetaTracker(alpha=0.5, beta=0.2)
    pos, vel = 1.5, -0.4          # [m], [m/s]
    for _ in range(60):
        pos += vel * DT
        t.update((pos, 0.0), DT)
    assert t.pos[0] == pytest.approx(pos, abs=0.02)
    assert t.vel[0] == pytest.approx(vel, abs=0.05)


def test_tracker_rejects_jumps_and_gives_up():
    """ゲートを外れた観測を棄却し、外挿が続いたら軌跡を捨てること。"""
    t = AlphaBetaTracker(gate=0.35, max_coast=8)
    t.update((1.0, 0.0), DT)
    t.update((3.0, 0.0), DT)              # 2 m 飛んだ観測は棄却
    assert t.extrapolated and t.pos[0] == pytest.approx(1.0, abs=0.01)
    for _ in range(8):
        t.update(None, DT)
    assert not t.active, '外挿が続けば未検出に戻ること'


def test_extrapolated_flag_reaches_the_result():
    """相手が消えた次のフレームで extrapolated が立つこと。"""
    det = RingDetector()
    with_box = S.render(S.Scene(boxes=[opponent()]), INTR, cam_height=CAM_H,
                        noise=NOISE)
    without = S.render(S.Scene(), INTR, cam_height=CAM_H, noise=NOISE)
    for _ in range(3):
        res = det.step(with_box, INTR, DT)
    assert res.status == OK and not res.extrapolated
    res = det.step(without, INTR, DT)
    assert res.extrapolated
    assert res.position is not None, '外挿の間は位置を出し続けること'


# ------------------------------------------------------ グリッド
def test_closing_fills_single_cell_holes():
    """1 セルの穴 (depth の抜け) を塞ぎ、外周は痩せないこと (§7)。"""
    m = np.zeros((9, 9), dtype=bool)
    m[2:7, 2:7] = True
    m[4, 4] = False
    closed = g.close(m, 1)
    assert closed[4, 4]
    assert closed[2:7, 2:7].all(), '閉じるたびに外周が削れないこと'


def test_seed_component_beats_largest():
    """最大成分ではなく、自分が乗っている成分を選ぶこと (§7)。"""
    m = np.zeros((20, 20), dtype=bool)
    m[2:6, 2:6] = True          # 種のある小さい成分
    m[10:19, 5:19] = True       # もっと広い、別の成分
    labels, n = g.label_components(m)
    seed = np.zeros_like(m)
    seed[3, 3] = True
    assert g.component_of_seed(labels, n, seed) == labels[3, 3]
    assert labels[3, 3] != labels[15, 15]


def test_seed_falls_back_to_largest_when_empty():
    """種が空なら最大成分に落ちること (検証時の逃げ道)。"""
    m = np.zeros((20, 20), dtype=bool)
    m[10:19, 5:19] = True
    labels, n = g.label_components(m)
    seed = np.zeros_like(m)
    assert g.component_of_seed(labels, n, seed) == labels[15, 15]
    assert g.component_of_seed(labels, n, seed, False) == 0


def test_fit_plane_needs_enough_points():
    """点が足りなければあてはめないこと。"""
    pts = np.zeros((10, 3), dtype=np.float32)
    fit = rg.fit_plane(pts, np.zeros(10), np.zeros(10), np.zeros(10), 0.0)
    assert not fit.ok


# ------------------------------------------------------ 決定性
def test_same_input_gives_same_output():
    """同じ入力列から同じ出力が出ること (乱数も時計も持たない)。"""
    depth = S.render(S.Scene(boxes=[opponent()]), INTR, cam_height=CAM_H,
                     noise=NOISE)
    outs = []
    for _ in range(2):
        det = RingDetector()
        for _ in range(4):
            res = det.step(depth, INTR, DT)
        outs.append((res.ring_height, res.position, res.top_height,
                     np.nan_to_num(res.cliff, nan=-1.0).tolist()))
    assert outs[0] == outs[1]


# ------------------------------------------------------ デバッグ出力
def test_point_labels_match_clusters():
    """want_debug の点ごとの塊の番号が、塊の点数とそろうこと (bag のビューアが使う)。"""
    depth = S.render(S.Scene(boxes=[opponent()]), INTR, cam_height=CAM_H,
                     noise=NOISE)
    det = RingDetector()
    for _ in range(3):
        res = det.step(depth, INTR, DT, want_debug=True)
    assert res.clusters
    assert res.points.shape[0] == res.n_points == res.point_labels.shape[0]
    for c in res.clusters:
        assert int(np.count_nonzero(res.point_labels == c.label)) == c.n_points
    # 渡さなければ持たない (本番の経路は今までどおり)
    res = det.step(depth, INTR, DT)
    assert res.points is None and res.point_labels is None
