# -*- coding: utf-8 -*-
"""RealSense の .bag (ROS1 bag v2.0) を読む。ROS にも pyrealsense2 にも依存しない。

realsense-viewer の「録画」が作るファイルは ROS1 の bag 形式で、ROS 2 の
`ros2 bag` では読めない。このマシンには rosbag (ROS1) も pyrealsense2 も
入っていないので、bag の形式をここで直接読む。必要なのは

    depth  /device_0/sensor_0/Depth_0/image/data     sensor_msgs/Image (mono16)
           /device_0/sensor_0/Depth_0/info/camera_info
           /device_0/sensor_0/option/Depth_Units/value   std_msgs/Float32
    color  /device_0/sensor_1/Color_0/image/data     sensor_msgs/Image (rgb8/bgr8)
    IMU    /device_0/sensor_2/Accel_0/imu/data       sensor_msgs/Imu (加速度だけ)
           /device_0/sensor_2/Gyro_0/imu/data        sensor_msgs/Imu (角速度だけ)

だけなので、メッセージの逆直列化もその型の分だけ書いてある。topic 名ではなく型と
encoding で選ぶので、ROS1 の realsense2_camera で録った bag (/camera/depth/...) も読める。

**索引がある (録画が正常に閉じた) ファイルは索引から、無いファイルは先頭から
走査して**メッセージの位置を集める。コピーの途中や録画の強制終了で末尾が切れた
ファイルでも、読めたところまでは使える。chunk の圧縮は none と bz2 に対応
(realsense-viewer は none で書く。lz4 は Python 標準に無いので断る)。
"""

import bisect
import bz2
from collections import OrderedDict
from dataclasses import dataclass
import os
import struct

import numpy as np

_MAGIC = b'#ROSBAG V2.0\n'
_OP_MSG, _OP_BAG, _OP_INDEX, _OP_CHUNK, _OP_CONN, _OP_CHUNK_INFO = 2, 3, 4, 5, 7, 6


def _fields(b):
    """Record の header (len + name=value の並び) を辞書にする。"""
    d = {}
    i = 0
    n_all = len(b)
    while i + 4 <= n_all:
        n = struct.unpack_from('<I', b, i)[0]
        i += 4
        k, _, v = bytes(b[i:i + n]).partition(b'=')
        d[k.decode('ascii', 'replace')] = v
        i += n
    return d


def _u32(v):
    return struct.unpack('<I', v)[0]


def _time(v):
    s, ns = struct.unpack('<II', v)
    return s + ns * 1e-9


def _records(buf, start=0):
    """Buf の中の record を (header 辞書, data の開始, data の長さ, record の開始) で返す。

    末尾が切れていたらそこで止まる。
    """
    i = start
    n_all = len(buf)
    while i + 4 <= n_all:
        rec = i
        hl = struct.unpack_from('<I', buf, i)[0]
        i += 4
        if i + hl + 4 > n_all:
            return
        hd = _fields(buf[i:i + hl])
        i += hl
        dl = struct.unpack_from('<I', buf, i)[0]
        i += 4
        if i + dl > n_all:
            return
        yield hd, i, dl, rec
        i += dl


# ==========================================================================
# 逆直列化 (ROS1)
# ==========================================================================
class _Reader:
    __slots__ = ('b', 'i')

    def __init__(self, b, i=0):
        self.b = b
        self.i = i

    def take(self, fmt):
        v = struct.unpack_from(fmt, self.b, self.i)
        self.i += struct.calcsize(fmt)
        return v

    def string(self):
        n = self.take('<I')[0]
        s = bytes(self.b[self.i:self.i + n]).decode('utf-8', 'replace')
        self.i += n
        return s

    def header(self):
        _seq, s, ns = self.take('<III')
        frame = self.string()
        return s + ns * 1e-9, frame


@dataclass
class ImageMsg:
    stamp: float
    height: int
    width: int
    encoding: str
    step: int
    data: memoryview

    def array(self):
        """Numpy の配列にする。mono16/16UC1 は (h, w) の uint16、rgb8/bgr8 は (h, w, 3)。"""
        enc = self.encoding.lower()
        if enc in ('mono16', '16uc1', 'z16'):
            a = np.frombuffer(self.data, dtype='<u2', count=self.height * self.step // 2)
            return a.reshape(self.height, self.step // 2)[:, :self.width]
        if enc in ('rgb8', 'bgr8', '8uc3'):
            a = np.frombuffer(self.data, dtype=np.uint8, count=self.height * self.step)
            return a.reshape(self.height, self.step)[:, :self.width * 3].reshape(
                self.height, self.width, 3)
        if enc in ('mono8', '8uc1', 'y8'):
            a = np.frombuffer(self.data, dtype=np.uint8, count=self.height * self.step)
            return a.reshape(self.height, self.step)[:, :self.width]
        raise ValueError('対応していない encoding: %s' % self.encoding)


def parse_image(b):
    r = _Reader(b)
    stamp, _ = r.header()
    h, w = r.take('<II')
    enc = r.string()
    _be, step = r.take('<BI')
    n = r.take('<I')[0]
    return ImageMsg(stamp, h, w, enc, step, memoryview(b)[r.i:r.i + n])


def parse_camera_info(b):
    """幅・高さ・K[9] を返す。"""
    r = _Reader(b)
    r.header()
    h, w = r.take('<II')
    r.string()                              # distortion_model
    nd = r.take('<I')[0]
    r.i += 8 * nd
    k = r.take('<9d')
    return int(w), int(h), k


def parse_imu(b):
    """時刻・角速度[3]・加速度[3] を返す。"""
    r = _Reader(b)
    stamp, _ = r.header()
    r.i += 8 * 4 + 8 * 9                    # orientation + covariance
    w = r.take('<3d')
    r.i += 8 * 9
    a = r.take('<3d')
    return stamp, w, a


def parse_float32(b):
    return struct.unpack_from('<f', b, 0)[0]


def parse_transform(b):
    """Transform (geometry_msgs) → 並進[3] と四元数 x,y,z,w。"""
    v = struct.unpack_from('<7d', b, 0)
    return v[:3], v[3:]


def quat_to_matrix(q):
    x, y, z, w = q
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)]])


# ==========================================================================
# bag
# ==========================================================================
@dataclass
class Connection:
    conn: int
    topic: str
    msg_type: str


class Ros1Bag:
    """ROS1 bag を開いて、topic ごとのメッセージの位置を持つ。

    本体は読まずに位置だけ持つので、数百 MB の bag でも開くのは速い
    (索引が無いファイルは chunk を 1 回ずつ読む)。本体は read() で 1 通ずつ取り出す。
    """

    def __init__(self, path, chunk_cache=3):
        self.path = path
        self.size = os.path.getsize(path)
        self.f = open(path, 'rb')
        if self.f.read(len(_MAGIC)) != _MAGIC:
            raise ValueError('ROS1 bag (#ROSBAG V2.0) ではない: %s' % path)
        self.connections = {}               # conn → Connection
        #: conn → [(record 時刻, chunk 番号, chunk 内の位置), ...]  時刻順
        self.messages = {}
        self._chunks = []                   # [(data の位置, data の長さ, 圧縮, 展開後の長さ)]
        self._cache = OrderedDict()
        self._cache_n = int(chunk_cache)
        self.indexed = False
        self.truncated = False

        hd, pos, dl, _ = self._read_record(len(_MAGIC))
        if hd is None or hd.get('op', b'\0')[0] != _OP_BAG:
            raise ValueError('bag の先頭 record が壊れている: %s' % path)
        index_pos = struct.unpack('<Q', hd['index_pos'])[0]
        first = pos + dl
        if 0 < index_pos < self.size and self._load_index(index_pos):
            self.indexed = True
        else:
            self.connections.clear()
            self.messages.clear()
            self._chunks.clear()
            self._scan(first)
        for v in self.messages.values():
            v.sort(key=lambda m: m[0])

    # ------------------------------------------------------------ 低レベル
    def _read_record(self, pos, with_data=False):
        """Pos の record の (header, data の位置, data の長さ, data) を返す。切れていれば None。"""
        f = self.f
        f.seek(pos)
        raw = f.read(4)
        if len(raw) < 4:
            return None, 0, 0, None
        hl = struct.unpack('<I', raw)[0]
        hb = f.read(hl)
        raw = f.read(4)
        if len(hb) < hl or len(raw) < 4:
            return None, 0, 0, None
        dl = struct.unpack('<I', raw)[0]
        dpos = pos + 4 + hl + 4
        if dpos + dl > self.size:
            return None, 0, 0, None
        data = f.read(dl) if with_data else None
        return _fields(hb), dpos, dl, data

    def _add_conn(self, hd, data):
        c = _u32(hd['conn'])
        if c not in self.connections:
            d = _fields(data)
            self.connections[c] = Connection(
                c, hd['topic'].decode('utf-8', 'replace'),
                d.get('type', b'').decode('utf-8', 'replace'))
            self.messages.setdefault(c, [])

    def _load_index(self, index_pos):
        """末尾の索引 (connection と chunk info) と、chunk ごとの index record を読む。"""
        chunk_pos = []
        pos = index_pos
        while pos < self.size:
            hd, dpos, dl, data = self._read_record(pos, with_data=True)
            if hd is None:
                return False
            op = hd['op'][0]
            if op == _OP_CONN:
                self._add_conn(hd, data)
            elif op == _OP_CHUNK_INFO:
                chunk_pos.append(struct.unpack('<Q', hd['chunk_pos'])[0])
            pos = dpos + dl
        if not chunk_pos:
            return False
        for cp in sorted(chunk_pos):
            hd, dpos, dl, _ = self._read_record(cp)
            if hd is None or hd['op'][0] != _OP_CHUNK:
                return False
            k = len(self._chunks)
            self._chunks.append((dpos, dl, hd['compression'].decode(), _u32(hd['size'])))
            # chunk の直後に connection ごとの index record が続く
            pos = dpos + dl
            while pos < self.size:
                ihd, idpos, idl, idata = self._read_record(pos, with_data=True)
                if ihd is None or ihd['op'][0] != _OP_INDEX:
                    break
                c = _u32(ihd['conn'])
                arr = np.frombuffer(idata, dtype='<u4').reshape(-1, 3)
                lst = self.messages.setdefault(c, [])
                for s, ns, off in arr.tolist():
                    lst.append((s + ns * 1e-9, k, off))
                pos = idpos + idl
        return True

    def _scan(self, pos):
        """索引が無いファイル。先頭から chunk を 1 つずつ開いて位置を集める。"""
        while pos < self.size:
            hd, dpos, dl, _ = self._read_record(pos)
            if hd is None:
                self.truncated = True
                break
            op = hd['op'][0]
            if op == _OP_CHUNK:
                k = len(self._chunks)
                self._chunks.append((dpos, dl, hd['compression'].decode(), _u32(hd['size'])))
                buf = self._chunk(k)
                for h2, d2, l2, rec in _records(buf):
                    op2 = h2['op'][0]
                    if op2 == _OP_CONN:
                        self._add_conn(h2, buf[d2:d2 + l2])
                    elif op2 == _OP_MSG:
                        c = _u32(h2['conn'])
                        self.messages.setdefault(c, []).append((_time(h2['time']), k, rec))
            elif op == _OP_CONN:
                self.f.seek(dpos)
                self._add_conn(hd, self.f.read(dl))
            pos = dpos + dl

    def _chunk(self, k):
        buf = self._cache.get(k)
        if buf is not None:
            self._cache.move_to_end(k)
            return buf
        dpos, dl, comp, size = self._chunks[k]
        self.f.seek(dpos)
        raw = self.f.read(dl)
        if comp == 'none':
            buf = raw
        elif comp == 'bz2':
            buf = bz2.decompress(raw)
        else:
            raise ValueError('chunk の圧縮 %s には対応していない (none / bz2 のみ)' % comp)
        self._cache[k] = buf
        while len(self._cache) > self._cache_n:
            self._cache.popitem(last=False)
        return buf

    # ------------------------------------------------------------ 読み出し
    def read(self, entry):
        """Messages の 1 要素から、メッセージ本体 (bytes) を返す。"""
        _t, k, off = entry
        buf = self._chunk(k)
        hl = struct.unpack_from('<I', buf, off)[0]
        dl = struct.unpack_from('<I', buf, off + 4 + hl)[0]
        d0 = off + 8 + hl
        return buf[d0:d0 + dl]

    def topics(self):
        """Topic・型・件数の組を並べて返す。"""
        return [(c.topic, c.msg_type, len(self.messages.get(c.conn, [])))
                for c in sorted(self.connections.values(), key=lambda c: c.conn)]

    def find(self, pred):
        """条件に合う connection を件数の多い順に返す。"""
        cs = [c for c in self.connections.values() if pred(c)]
        return sorted(cs, key=lambda c: -len(self.messages.get(c.conn, [])))

    def close(self):
        self.f.close()


# ==========================================================================
# RealSense の録画としての見方
# ==========================================================================
@dataclass
class ImuSamples:
    """時刻順に並べた IMU。角速度と加速度は別のストリームで、時刻も別。"""

    gyro_t: np.ndarray       # [N]
    gyro: np.ndarray         # [N, 3] rad/s (depth の光学座標 x右 y下 z前 に直したもの)
    accel_t: np.ndarray      # [M]
    accel: np.ndarray        # [M, 3] m/s^2 (同上)

    @property
    def empty(self):
        return self.gyro_t.size == 0 and self.accel_t.size == 0

    def gyro_between(self, t0, t1):
        """区間 (t0, t1] の角速度を [(ω, dt), ...] で返す。検出器の step(gyro=...) の形。"""
        if self.gyro_t.size == 0 or t1 <= t0:
            return []
        i0 = bisect.bisect_right(self.gyro_t, t0)
        i1 = bisect.bisect_right(self.gyro_t, t1)
        out = []
        prev = t0
        for i in range(i0, i1):
            t = float(self.gyro_t[i])
            out.append((self.gyro[i], t - prev))
            prev = t
        return out

    def accel_around(self, t, half=0.25):
        """時刻 t の前後 half 秒の加速度の (平均, 標準偏差のノルム, 件数)。"""
        if self.accel_t.size == 0:
            return None, float('nan'), 0
        i0 = bisect.bisect_left(self.accel_t, t - half)
        i1 = bisect.bisect_right(self.accel_t, t + half)
        if i1 - i0 < 3:
            return None, float('nan'), i1 - i0
        a = self.accel[i0:i1]
        return a.mean(axis=0), float(np.linalg.norm(a.std(axis=0))), i1 - i0

    def gyro_rms_around(self, t, half=0.25):
        if self.gyro_t.size == 0:
            return float('nan')
        i0 = bisect.bisect_left(self.gyro_t, t - half)
        i1 = bisect.bisect_right(self.gyro_t, t + half)
        if i1 <= i0:
            return float('nan')
        return float(np.sqrt(np.mean(np.sum(self.gyro[i0:i1] ** 2, axis=1))))


class RealSenseBag:
    """depth・color・IMU を取り出す面。depth のフレームを番号で引ける。"""

    def __init__(self, path):
        self.bag = Ros1Bag(path)
        b = self.bag

        def is_img(c):
            return c.msg_type == 'sensor_msgs/Image'

        depth = b.find(lambda c: is_img(c) and 'depth' in c.topic.lower()
                       and 'aligned' not in c.topic.lower())
        if not depth:
            raise ValueError('depth の画像 topic が無い: %s' % path)
        self.depth_conn = depth[0]
        color = b.find(lambda c: is_img(c) and ('color' in c.topic.lower()
                                                or 'rgb' in c.topic.lower()))
        self.color_conn = color[0] if color else None

        # 内部パラメータ: depth の topic と同じ階層の camera_info
        root = self.depth_conn.topic.rsplit('/image', 1)[0]
        infos = b.find(lambda c: c.msg_type == 'sensor_msgs/CameraInfo'
                       and c.topic.startswith(root))
        if not infos:
            infos = b.find(lambda c: c.msg_type == 'sensor_msgs/CameraInfo'
                           and 'depth' in c.topic.lower())
        if not infos:
            raise ValueError('depth の camera_info が無い: %s' % path)
        w, h, k = parse_camera_info(b.read(b.messages[infos[0].conn][0]))
        self.intr_wh = (w, h)
        self.K = k

        # 深度の単位 (realsense-viewer は option として 1 回だけ書く。無ければ 1 mm)
        self.depth_scale = 0.001
        du = b.find(lambda c: c.topic.endswith('Depth_Units/value'))
        if du and b.messages[du[0].conn]:
            v = parse_float32(b.read(b.messages[du[0].conn][0]))
            if 1e-5 < v < 1.0:
                self.depth_scale = float(v)

        self.depth_index = b.messages[self.depth_conn.conn]
        self.color_index = b.messages[self.color_conn.conn] if self.color_conn else []
        self._color_t = [m[0] for m in self.color_index]
        self.device = self._device_info()
        self.imu = self._load_imu()
        self._stamps = None

    # ------------------------------------------------------------ 付帯情報
    def _device_info(self):
        b = self.bag
        out = {}
        for c in b.find(lambda c: c.topic == '/device_0/info'
                        and c.msg_type == 'diagnostic_msgs/KeyValue'):
            for e in b.messages[c.conn]:
                r = _Reader(b.read(e))
                k = r.string()
                out[k] = r.string()
        return out

    def _imu_rotation(self, conn):
        """IMU ストリームの外部パラメータ (tf/0) の回転。無ければ単位行列。

        realsense の bag は各ストリームの tf/0 に「基準ストリームから見た姿勢」を持つ。
        D435i 系は IMU の軸を depth に合わせて出すので普通は単位行列だが、
        入っていれば depth の光学座標へ回してから使う。
        """
        root = conn.topic.rsplit('/imu', 1)[0]
        tfs = self.bag.find(lambda c: c.topic == root + '/tf/0')
        if not tfs or not self.bag.messages[tfs[0].conn]:
            return np.eye(3)
        _, q = parse_transform(self.bag.read(self.bag.messages[tfs[0].conn][0]))
        return quat_to_matrix(q)

    def _load_imu(self):
        b = self.bag
        imus = b.find(lambda c: c.msg_type == 'sensor_msgs/Imu')
        gt, gv, at, av = [], [], [], []
        for c in imus:
            name = c.topic.lower()
            want_g = 'accel' not in name
            want_a = 'gyro' not in name
            rot = self._imu_rotation(c)
            for e in b.messages[c.conn]:
                t, w, a = parse_imu(b.read(e))
                if want_g:
                    gt.append(t)
                    gv.append(rot @ np.asarray(w))
                if want_a:
                    at.append(t)
                    av.append(rot @ np.asarray(a))

        def pack(ts, vs):
            if not ts:
                return np.zeros(0), np.zeros((0, 3))
            o = np.argsort(ts, kind='stable')
            return np.asarray(ts)[o], np.asarray(vs)[o]

        g_t, g_v = pack(gt, gv)
        a_t, a_v = pack(at, av)
        return ImuSamples(g_t, g_v, a_t, a_v)

    # ------------------------------------------------------------ フレーム
    def __len__(self):
        return len(self.depth_index)

    def depth(self, i):
        """I 番目の depth を (header の時刻, uint16 の (h, w) 配列) で返す。"""
        m = parse_image(self.bag.read(self.depth_index[i]))
        return m.stamp, m.array()

    def color_near(self, i):
        """I 番目の depth に記録時刻が最も近い color を (h, w, 3) RGB で返す。無ければ None。"""
        if not self.color_index:
            return None
        t = self.depth_index[i][0]
        j = bisect.bisect_left(self._color_t, t)
        if j > 0 and (j == len(self._color_t)
                      or abs(self._color_t[j - 1] - t) <= abs(self._color_t[j] - t)):
            j -= 1
        m = parse_image(self.bag.read(self.color_index[j]))
        img = m.array()
        if m.encoding.lower() == 'bgr8':
            img = img[:, :, ::-1]
        return img

    def record_time(self, i):
        """Bag に記録された時刻 (録画開始からの秒)。シークの目盛りに使う。"""
        return self.depth_index[i][0]

    @property
    def duration(self):
        if not self.depth_index:
            return 0.0
        return self.depth_index[-1][0] - self.depth_index[0][0]

    def close(self):
        self.bag.close()
