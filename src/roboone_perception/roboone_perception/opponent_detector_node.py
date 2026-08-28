# -*- coding: utf-8 -*-
"""opponent_detector ノード — depth と IMU から相手位置とリングの縁を出す。

ros-architecture §2/§4 と、docs/opponent_detection.pdf §11 の約束事:

    受け取る: /camera/depth/image_rect_raw  (sensor_msgs/Image 16UC1, 30Hz)
              /camera/depth/camera_info     (sensor_msgs/CameraInfo)
              /camera/imu                   (sensor_msgs/Imu, 200Hz)
    出す:     /opponent    (roboone_interfaces/Opponent)     depth と同じ周期
              /ring_edge   (std_msgs/Float32MultiArray)      同上
              /detector/debug (sensor_msgs/Image rgb8)       購読者がいるときだけ

検出の中身は roboone_perception.detect（ROS 非依存）にある。このノードが持つのは
「トピックとパラメータの面倒を見る」ことだけで、判断は 1 つも持たない。


ros-architecture から変えたところ (理由つき)
--------------------------------------------

**1. 入力を点群ではなく深度画像にした。** アーキ文書の表では opponent_detector は
/camera/…/points を購読することになっているが、点群では検出パイプラインが成立しない。

  * §4 の間引きは「d 画素ごと」であって「d 点ごと」ではない。realsense_bringup は
    `ordered_pc=false`（無効画素を落とした 1 行 N 点）で点群を出しているので、
    点群からは画素の並びが復元できない。
  * §7 の「視野の縁とリングの端を区別する」判定には、その点がどの画素から来たかが要る。
    点群にはその情報が無い。
  * 点群を作る仕事 (realsense2_camera 側) と読む仕事が丸ごと無駄になる。深度画像から
    直接逆投影すれば同じ点が得られる。

  → 検出器を上げるときは realsense を `enable_pointcloud:=false` で起動してよい
    (launch/opponent_detector.launch.py の既定)。rviz で点群を見たいときだけ ON にする。

**2. 姿勢を imu_filter_madgwick からもらわない。** §11 のとおり、歩行の安定化が使う
姿勢 (200Hz・機体の傾き) と検出が要る姿勢 (30Hz・リング面に対する鉛直) は別物で、
1 本にまとめるとどちらの時定数も中途半端になる。このノードは /camera/imu の生値を
自分で積分し、リング面の法線で毎フレーム引き戻す。/imu/data は購読しない。

**3. /opponent の型を Opponent に昇格させた。** 技術ノート(3) §5.1 の予定どおり。
転倒判定に上端高さが要るので PointStamped では足りない。あわせて velocity
(§2.1 の追加提案) と status (§9.2 の縮退の区別) を載せている。

**4. 「見えない」を 1 つに潰さない。** リング面が取れない / 面あてはめの門を通らない /
相手がいない、の 3 つは行動層での扱いが違う。status で区別して出す。
"""

from collections import deque
from dataclasses import fields
import math

from geometry_msgs.msg import Point, Vector3
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from roboone_interfaces.msg import Opponent
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from .detect import (ATTITUDE_STALE, BodyParams, DetectorParams, Intrinsics,
                     MatchParams, NO_OPPONENT, OK, RING_LOST, RingDetector,
                     TuneParams)
from .detect import grid as g

#: 深度画像の QoS。realsense2_camera は画像を RELIABLE で出す。ここを取り違えると
#: 購読が成立せず「1 枚も来ない」で止まるので、名前で選べるようにしてある
#: (realsense_bringup の README 参照)。
_QOS = {
    'reliable': QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE),
    'sensor_data': QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT),
}

#: status 文字列 → Opponent.msg の定数
_STATUS = {
    OK: Opponent.STATUS_OK,
    NO_OPPONENT: Opponent.STATUS_NO_OPPONENT,
    ATTITUDE_STALE: Opponent.STATUS_ATTITUDE_STALE,
    RING_LOST: Opponent.STATUS_RING_LOST,
}

#: デバッグ俯瞰図の色 (図 7 に合わせる)
_C_RING = (70, 70, 70)
_C_EDGE = (230, 140, 40)
_C_IN = (60, 200, 80)
_C_OUT = (200, 60, 60)
_C_SEL = (255, 255, 80)
_C_SELF = (80, 160, 255)


def _stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class OpponentDetectorNode(Node):

    def __init__(self, **kwargs):
        super().__init__('opponent_detector', **kwargs)

        # --- トピックと入出力の設定 ------------------------------------
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/depth/camera_info')
        self.declare_parameter('imu_topic', '/camera/imu')
        self.declare_parameter('depth_qos', 'reliable')
        self.declare_parameter('frame_id', 'base_link')
        # 撮像から publish までが遅れているとき、古いフレームを捨てて追いつく。
        # 遅れの予算は §10 で露光から発行まで 60 ms。ここを超えたフレームは
        # 処理しても行動層には古すぎるので、捨てて次に賭ける
        self.declare_parameter('max_frame_lag', 0.12)
        # Pi 5 が間に合わないときの最後の手 (§10 の 1 番目)。2 なら 1 枚おき
        self.declare_parameter('process_every_n', 1)
        # 深度が途切れたときに「見えていない」を出し続ける間隔
        self.declare_parameter('watchdog_period', 0.2)
        self.declare_parameter('depth_timeout', 0.5)
        self.declare_parameter('debug_scale', 4)

        # --- 検出器の定数。§11 の 3 群をそのまま宣言する ----------------
        for prefix, cls in (('body.', BodyParams), ('match.', MatchParams),
                            ('tune.', TuneParams)):
            for f in fields(cls):
                self.declare_parameter(prefix + f.name, f.default)

        self.frame_id = self.get_parameter('frame_id').value
        self.debug_scale = int(self.get_parameter('debug_scale').value)
        self.max_lag = float(self.get_parameter('max_frame_lag').value)
        self.every_n = max(1, int(self.get_parameter('process_every_n').value))
        self.depth_timeout = float(self.get_parameter('depth_timeout').value)

        self.detector = RingDetector(self._collect_params())
        self.add_on_set_parameters_callback(self._on_params)

        # --- 状態 -------------------------------------------------------
        self.intr = None
        self.imu = deque(maxlen=2000)      # (時刻[s], ω[3])
        self.accel = None
        self.prev_stamp = None
        self.last_imu_t = None
        self.frame_count = 0
        self.last_depth_wall = None
        self.last_status = None
        self.dropped = 0
        self._pending_rebuild = False

        # --- 出入り口 ---------------------------------------------------
        qos_name = str(self.get_parameter('depth_qos').value)
        depth_qos = _QOS.get(qos_name, _QOS['reliable'])
        self.pub_opp = self.create_publisher(Opponent, '/opponent', 5)
        self.pub_edge = self.create_publisher(Float32MultiArray, '/ring_edge', 5)
        self.pub_dbg = self.create_publisher(Image, '/detector/debug', 1)

        self.create_subscription(CameraInfo,
                                 self.get_parameter('camera_info_topic').value,
                                 self._on_info, depth_qos)
        self.create_subscription(Image, self.get_parameter('depth_topic').value,
                                 self._on_depth, depth_qos)
        self.create_subscription(Imu, self.get_parameter('imu_topic').value,
                                 self._on_imu, _QOS['sensor_data'])
        self.create_timer(float(self.get_parameter('watchdog_period').value),
                          self._on_watchdog)

        self.get_logger().info(
            'opponent_detector 起動: depth=%s (%s) imu=%s / h_cam=%.3f m 俯角=%.1f deg' % (
                self.get_parameter('depth_topic').value, qos_name,
                self.get_parameter('imu_topic').value,
                self.detector.p.body.cam_height, self.detector.p.body.cam_pitch_deg))

    # ------------------------------------------------------------ パラメータ
    def _collect_params(self):
        flat = {}
        for prefix, cls in (('body.', BodyParams), ('match.', MatchParams),
                            ('tune.', TuneParams)):
            for f in fields(cls):
                flat[prefix + f.name] = self.get_parameter(prefix + f.name).value
        return DetectorParams.from_flat(flat)

    def _on_params(self, params):
        """実行時に変えてよいのは tune.* と match.* だけ (§11)。

        body.* は姿勢と高さヒストグラムの前提そのものなので、走らせたまま変えると
        検出器の状態と食い違う。起動時に固定する。
        """
        for p in params:
            if p.name.startswith('body.'):
                return SetParametersResult(
                    successful=False,
                    reason='body.* は起動時に固定する。launch のパラメータで指定すること')
        touched = any(p.name.startswith(('tune.', 'match.')) for p in params)
        if touched:
            # 反映は次フレームから。tune.* はグリッドの形まで変えうるので
            # 検出器ごと作り直す (追尾と姿勢は 1 フレームぶん失う)
            self._pending_rebuild = True
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ 購読
    def _on_info(self, msg):
        intr = Intrinsics.from_camera_info(msg)
        if self.intr != intr:
            self.intr = intr
            hfov = 2 * math.degrees(math.atan(0.5 * intr.width / intr.fx))
            vfov = 2 * math.degrees(math.atan(0.5 * intr.height / intr.fy))
            self.get_logger().info(
                'depth %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f (画角 %.1f x %.1f deg)'
                % (intr.width, intr.height, intr.fx, intr.fy, intr.cx, intr.cy,
                   hfov, vfov))

    def _on_imu(self, msg):
        t = _stamp_sec(msg.header.stamp)
        w = msg.angular_velocity
        self.imu.append((t, (w.x, w.y, w.z)))
        a = msg.linear_acceleration
        self.accel = (a.x, a.y, a.z)

    def _gyro_since(self, t_prev, t_now):
        """(t_prev, t_now] のジャイロを [(ω, dt), ...] にして取り出す。

        200 Hz を全部使う。§1.1 のとおり文書の評価は playback の都合で depth 1 枚
        あたり 1 サンプルしか使えていないので、実機ではここより良くなる方向にしかずれない。
        """
        out = []
        last = t_prev
        while self.imu and self.imu[0][0] <= t_prev:
            self.imu.popleft()
        for t, w in self.imu:
            if t > t_now:
                break
            dt = t - last
            if 0.0 < dt < 0.1:          # 異常な間隔は積分しない
                out.append((w, dt))
            last = t
        tail = t_now - last
        if out and 0.0 < tail < 0.1:
            out.append((out[-1][0], tail))
        return out

    def _on_depth(self, msg):
        self.last_depth_wall = self.get_clock().now().nanoseconds * 1e-9
        if self.intr is None:
            return
        self.frame_count += 1
        if self.frame_count % self.every_n:
            return
        if self._pending_rebuild:
            self.detector = RingDetector(self._collect_params())
            self._pending_rebuild = False
            self.get_logger().info('パラメータ変更を反映して検出器を作り直した')

        stamp = _stamp_sec(msg.header.stamp)
        lag = self.last_depth_wall - stamp
        if lag > self.max_lag and self.prev_stamp is not None:
            # 処理が追いついていない。古い絵を出すより、捨てて次を待つ
            self.dropped += 1
            self.prev_stamp = stamp
            if self.dropped % 30 == 1:
                self.get_logger().warn(
                    '遅れ %.0f ms のフレームを捨てた (累計 %d 枚)。'
                    'tune.stride か process_every_n を上げる' % (lag * 1e3, self.dropped))
            return

        depth, scale = self._as_depth(msg)
        if depth is None:
            self.get_logger().warn('未対応の深度形式: %s' % msg.encoding, once=True)
            return

        dt = (stamp - self.prev_stamp) if self.prev_stamp else 1.0 / 30.0
        gyro = self._gyro_since(self.prev_stamp, stamp) if self.prev_stamp else ()
        self.prev_stamp = stamp

        want_debug = self.pub_dbg.get_subscription_count() > 0
        res = self.detector.step(depth, self.intr, dt, gyro=gyro,
                                 accel=self.accel, depth_scale=scale,
                                 want_debug=want_debug)
        self._publish(res, msg.header.stamp)
        if want_debug:
            self.pub_dbg.publish(self._debug_image(res, msg.header.stamp))

        if res.status != self.last_status:
            self.get_logger().info(
                '%s → %s (面積 %.2f m^2 / 残差 %.1f mm / %.1f ms)'
                % (self.last_status or '-', res.status, res.ring_area,
                   res.plane_resid * 1e3, res.timings.get('total', 0) * 1e3))
            self.last_status = res.status

    @staticmethod
    def _as_depth(msg):
        """Image → (uint16 か float32 の 2 次元配列, m へのスケール)。"""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ('16UC1', 'mono16'):
            a = buf.view(np.uint16).reshape(msg.height, msg.width)
            return a, 0.001
        if msg.encoding == '32FC1':
            a = buf.view(np.float32).reshape(msg.height, msg.width)
            return a, 1.0
        return None, None

    # ------------------------------------------------------------ 発行
    def _publish(self, res, stamp):
        b = self.detector.p.body
        m = Opponent()
        m.header.stamp = stamp
        m.header.frame_id = self.frame_id
        m.status = _STATUS[res.status]
        m.valid = bool(res.status == OK and res.position is not None)
        m.extrapolated = bool(res.extrapolated)
        if res.position is not None:
            # リング平面座標 (カメラ原点) から機体座標へ。カメラは機体固定なので
            # 平行移動だけで済む
            x = res.position[0] + b.cam_offset_x
            y = res.position[1] + b.cam_offset_y
            m.position = Point(x=float(x), y=float(y), z=float(res.height))
            m.velocity = Vector3(x=float(res.velocity[0]), y=float(res.velocity[1]),
                                 z=0.0)
            m.range = float(math.hypot(x, y))
            m.bearing = float(math.atan2(y, x))
            m.top_height = float(res.top_height)
            m.width = float(res.width)
        else:
            m.position = Point()
            m.velocity = Vector3()
            m.range = float('nan')
            m.bearing = float('nan')
            m.top_height = float('nan')
            m.width = float('nan')
        self.pub_opp.publish(m)

        if res.cliff is not None:
            self.pub_edge.publish(self._edge_msg(res.cliff))

    def _edge_msg(self, cliff):
        """d_cliff(θ) を Float32MultiArray に載せる。

        型は技術ノート(3) §5.1 で決まっているので変えない。ビン k の中心方位は

            θ_k = -half_fov + (k + 0.5) · 2·half_fov / bins   [rad]

        で、half_fov と bins は tune.edge_half_fov_deg / tune.edge_bins。
        受け側が推測しなくて済むように、その 2 つを dim のラベルにも書いておく。
        見えていない方位は NaN で、「縁が無い」ではなく「見ていない」を意味する。
        """
        t = self.detector.p.tune
        msg = Float32MultiArray()
        dim = MultiArrayDimension()
        dim.label = 'bearing_deg[%+.1f,%+.1f]' % (-t.edge_half_fov_deg,
                                                  t.edge_half_fov_deg)
        dim.size = int(t.edge_bins)
        dim.stride = int(t.edge_bins)
        msg.layout.dim = [dim]
        msg.data = [float(v) for v in cliff]
        return msg

    def publish_lost(self):
        """「今この瞬間、何も見えていない」を 1 通出す。

        黙るのではなく出し続けるのは、行動層が最後の /opponent を握ったまま
        古い位置へ歩き続けるのを避けるため。header.stamp は現在時刻にする。
        """
        m = Opponent()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.status = Opponent.STATUS_RING_LOST
        m.valid = False
        m.position = Point()
        m.velocity = Vector3()
        m.range = m.bearing = m.top_height = m.width = float('nan')
        self.pub_opp.publish(m)

    def _on_watchdog(self):
        """深度が途切れていないか見張る。途切れていれば見失いを出し続ける。"""
        if self.last_depth_wall is None:
            return
        gap = self.get_clock().now().nanoseconds * 1e-9 - self.last_depth_wall
        if gap < self.depth_timeout:
            return
        self.publish_lost()
        if self.last_status != 'NO_DEPTH':
            self.get_logger().warn('深度が %.1f s 途切れている' % gap)
            self.last_status = 'NO_DEPTH'

    # ------------------------------------------------------------ デバッグ表示
    def _debug_image(self, res, stamp):
        """図 7 と同じ俯瞰図。購読者がいるときだけ作る (試合中は誰も見ない)。"""
        spec = self.detector.spec
        s = max(1, self.debug_scale)
        img = np.zeros((spec.nu, spec.nv, 3), dtype=np.uint8)

        def paint(mask, color):
            if mask is not None and mask.any():
                img[mask] = color

        if res.above_mask is not None:
            paint(res.above_mask, _C_OUT)              # 面より上・リング外は赤
        if res.ring_mask is not None:
            paint(res.ring_mask, _C_RING)
            paint(g.boundary(res.ring_mask), _C_EDGE)
        paint(res.obj_mask, _C_IN)                     # リング内の塊は緑
        if res.selected is not None:
            iu, iv, ok = spec.index(np.array([res.selected.fwd]),
                                    np.array([res.selected.left]))
            if ok[0]:
                img[max(0, iu[0] - 1):iu[0] + 2, max(0, iv[0] - 1):iv[0] + 2] = _C_SEL
        iu, iv, ok = spec.index(np.array([0.0]), np.array([0.0]))
        if ok[0]:
            img[iu[0], iv[0]] = _C_SELF

        # 前方 u を上、左 v を左にする (図 7 と同じ向き)
        img = np.flip(np.flip(img, axis=0), axis=1)
        img = np.repeat(np.repeat(img, s, axis=0), s, axis=1)

        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = img.shape[1] * 3
        msg.data = img.tobytes()
        return msg


def main(args=None):
    # 既定のシグナルハンドラは context を先に畳むので、終了時に publish したいものが
    # あるノードでは切っておく (roboone_teleop と同じ理由)。ここでは最後に
    # 「見えていない」を 1 回置いていく
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = OpponentDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_lost()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
