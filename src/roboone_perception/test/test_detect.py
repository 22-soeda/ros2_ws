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

ROS は要らない。roboone_motion の walk_core と同じで、計算部分だけを直接叩く。
"""

import math

import numpy as np
import pytest
from roboone_perception.detect import (AlphaBetaTracker, ATTITUDE_STALE,
                                       AttitudeEstimator, DetectorParams,
                                       forward_ref, Intrinsics, NO_OPPONENT,
                                       OK, ring_basis, RING_LOST, RingDetector)
from roboone_perception.detect import clusters as cl
from roboone_perception.detect import edge as ed
from roboone_perception.detect import grid as g
from roboone_perception.detect import ring as rg
import scene as S

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
