"""실시간 동적 물체(사람 등) 필터: /fastlio2/body_cloud -> /fastlio2/body_cloud_static

원리 (스캔 단위, 시간 순방향 '빈 공간에 새로 나타남')
  최근 몇 초 동안의 스캔(과거 스캔)마다 16채널 x 방위각 거리 영상을 들고 있다가, 새 스캔의 점 p 를
  과거 스캔 거리 영상에 투영한다. 과거 스캔의 광선이 p 보다 확실히 멀리까지 갔으면(= 그때 p 자리는 비어 있었음)
  p 는 그 사이에 들어온 물체 -> 동적 표 1. 표가 min_votes 이상이고 '뚫고 지나감' 비율이 min_ratio 이상이면 동적.
  동적 점이 있는 voxel 의 점은 같이 빼고(몸 전체), 그 voxel 은 track_time 동안 기억해서 사람이 잠깐 멈춰도 계속 뺀다
  (기억은 실제 판정이 있을 때만 갱신 -> 옆 벽으로 번지지 않음).

한계
  - 처음부터 서 있던 사람/물체는 '새로 나타난' 게 아니라서 못 거름 -> 저장할 때 remove_dynamic 이 처리
  - 판정은 바닥 위 사람 높이(로봇 바닥 + floor_margin ~ max_height)만. 바닥/천장은 그대로 통과
  - 라이다 축 = 로봇 축 가정 (luco2). sensor 파라미터 = base_footprint 기준 라이다 위치

입력 자세: /fastlio2/lio_odom (lio_world 기준, body_cloud 와 같은 시각). PGO 보정은 필요 없음(최근 몇 초 안의 상대 자세만 씀).
"""
import time
from collections import deque

import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import PointCloud2

VLP16_EL = np.arange(-15.0, 15.1, 2.0)


def _xyz(msg):
    """PointCloud2 -> (N,3) float64 (x,y,z float32 가정, 비조직)"""
    offs = {f.name: f.offset for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)
    out = np.empty((n, 3), np.float64)
    for k, name in enumerate('xyz'):
        out[:, k] = raw[:, offs[name]:offs[name] + 4].copy().view(np.float32).ravel()
    return out, raw


def _vkey(pw, size):
    """world 좌표 -> voxel 정수 키 (축마다 21비트)"""
    k = np.floor(pw / size).astype(np.int64) + (1 << 20)
    return (k[:, 0] << 42) | (k[:, 1] << 21) | k[:, 2]


def _project(ps, az_res, el_tol):
    r = np.linalg.norm(ps, axis=1)
    el = np.degrees(np.arcsin(np.clip(ps[:, 2] / np.maximum(r, 1e-6), -1, 1)))
    row = np.round((el - VLP16_EL[0]) / 2.0).astype(np.int64)
    ok = (row >= 0) & (row < 16)
    ok &= np.abs(el - VLP16_EL[np.clip(row, 0, 15)]) < el_tol
    col = (np.floor((np.degrees(np.arctan2(ps[:, 1], ps[:, 0])) + 180.0) / az_res).astype(np.int64)) % int(round(360 / az_res))
    return r, np.clip(row, 0, 15), col, ok


DEFAULTS = dict(
    sensor=[0.12545, 0.0, 0.3677],  # base_footprint 기준 라이다 위치 (luco2)
    window=2.5,        # 과거 스캔 보관 시간 (s)
    min_age=0.3,       # 이보다 최근 스캔은 비교 안 함 (같은 물체가 거의 같은 자리)
    n_past=8,          # 비교할 과거 스캔 수 (window 안에서 고르게)
    min_votes=2,       # '뚫고 지나감' 최소 횟수
    min_ratio=0.7,     # 뚫고 지나감 / (뚫고 지나감 + 맞음)
    floor_margin=0.15, # 판정 높이: 로봇 바닥 + floor_margin ~ max_height
    max_height=2.2,
    min_range=0.5,
    max_range=15.0,
    az_res=1.0,        # 거리 영상 방위각 해상도 (도)
    el_tol=0.3,        # 채널 고도각 허용 오차 (도)
    erode_cols=1,      # '뚫고 지나감'은 좌우 이 칸수까지 본 최소 거리로 판정 (물체 모서리 오탐 방지)
    margin=0.15,       # 뚫고 지나감/맞음 판정 여유 (m) + 2%/m
    voxel=0.3,
    min_cluster=3,     # voxel 안 판정 점이 이만큼 있어야 동적으로 인정 (외톨이 오탐 제거)
    track_time=1.0,    # 동적 voxel 기억 시간 (s), 실제 판정으로만 갱신
    persist_scans=30,  # 이 스캔 수 이상 점이 찍힌 voxel 은 정적으로 보호 (가구 등). 0 이면 끔
    persist_time=3.0,  #   + 처음 찍힌 뒤 이 시간(s) 이상 지났을 때
)


class DynCore:
    """ROS 와 무관한 판정부 (노드와 오프라인 평가가 같이 씀)"""

    def __init__(self, **kw):
        c = dict(DEFAULTS, **kw)
        for k, v in c.items():
            setattr(self, k, v)
        self.t_bs = np.array(self.sensor, dtype=np.float64)
        self.ncol = int(round(360 / self.az_res))
        self.past = deque()          # (stamp, R, o_sensor_world, img_raw, img_eroded)
        self.track = {}              # voxel key -> 마지막 판정 시각
        self.occ = {}                # voxel key -> [점이 찍힌 스캔 수, 처음 찍힌 시각] (정적 보호용)

    def _imgs(self, ps):
        r, row, col, ok = _project(ps, self.az_res, 0.8)
        img = np.full((16, self.ncol), np.inf)
        np.minimum.at(img, (row[ok], col[ok]), r[ok])
        er = img.copy()
        for d in range(1, self.erode_cols + 1):
            er = np.minimum(er, np.minimum(np.roll(img, d, 1), np.roll(img, -d, 1)))
        return img, er

    def process(self, st, R, t, pb):
        """st: 시각(s), R/t: world 기준 base_footprint 자세, pb: base_footprint 좌표 점 (N,3) -> 동적 여부 (N,)"""
        ps_self = pb - self.t_bs                      # 센서 좌표 (축 = 로봇 축)
        rng = np.linalg.norm(ps_self, axis=1)
        cand = (pb[:, 2] > self.floor_margin) & (pb[:, 2] < self.max_height) & \
               (rng > self.min_range) & (rng < self.max_range)
        pw = R.apply(pb) + t
        dyn = np.zeros(len(pb), bool)

        while self.past and st - self.past[0][0] > self.window:
            self.past.popleft()
        usable = [x for x in self.past if st - x[0] >= self.min_age]
        if len(usable) > self.n_past:
            usable = [usable[i] for i in np.linspace(0, len(usable) - 1, self.n_past).round().astype(int)]
        ci = np.nonzero(cand)[0]
        ckeys = _vkey(pw[ci], self.voxel)
        # 정적 보호: 오래 꾸준히 찍힌 voxel (성긴 가구는 시점마다 광선이 틈으로 빠져서 '뚫고 지나감'이 나옴)
        protected = np.zeros(len(ci), bool)
        if self.persist_scans > 0 and len(ci):
            uk = np.unique(ckeys)
            prot = []
            for k in uk.tolist():
                e = self.occ.get(k)
                if e is None:
                    self.occ[k] = [1, st]
                else:
                    e[0] += 1
                    if e[0] >= self.persist_scans and st - e[1] >= self.persist_time:
                        prot.append(k)
            if prot:
                protected = np.isin(ckeys, np.array(prot, np.int64))
        if usable and len(ci):
            see = np.zeros(len(ci), np.int32)
            hit = np.zeros(len(ci), np.int32)
            for _, Rj, oj, img, er in usable:
                ps = Rj.inv().apply(pw[ci] - oj)
                r, row, col, ok = _project(ps, self.az_res, self.el_tol)
                m = self.margin + 0.02 * r
                rs = np.where(ok, img[row, col], np.inf)
                re = np.where(ok, er[row, col], np.inf)
                see += ok & np.isfinite(re) & (re > r + m)
                hit += ok & np.isfinite(rs) & (np.abs(rs - r) <= m)
            det = (see >= self.min_votes) & (see >= self.min_ratio * (see + hit)) & ~protected
            if det.any():
                keys = ckeys[det]
                uk, cnt = np.unique(keys, return_counts=True)
                good = uk[cnt >= self.min_cluster]
                for k in good.tolist():
                    self.track[k] = st
        # 동적 voxel (이번 판정 + 최근 기억) 안의 사람 높이 점 전부
        if self.track and len(ci):
            self.track = {k: v for k, v in self.track.items() if st - v <= self.track_time}
            if self.track:
                hit_t = np.isin(ckeys, np.fromiter(self.track.keys(), np.int64)) & ~protected
                dyn[ci[hit_t]] = True

        self.past.append((st, R, t + R.apply(self.t_bs), *self._imgs(ps_self)))
        return dyn


class DynamicFilter(Node):
    def __init__(self):
        super().__init__('dynamic_filter')
        kw = {k: self.declare_parameter(k, v).value for k, v in DEFAULTS.items()}
        self.core = DynCore(**kw)
        self.stats = deque(maxlen=200)
        self.pub_static = self.create_publisher(PointCloud2, '~/static', 10)
        self.pub_dyn = self.create_publisher(PointCloud2, '~/dynamic', 10)
        sub_c = message_filters.Subscriber(self, PointCloud2, '/fastlio2/body_cloud')
        sub_o = message_filters.Subscriber(self, Odometry, '/fastlio2/lio_odom')
        self.sync = message_filters.ApproximateTimeSynchronizer([sub_c, sub_o], 20, 0.02)
        self.sync.registerCallback(self._cb)
        self.create_timer(10.0, self._log)
        self.get_logger().info(f'/fastlio2/body_cloud -> ~/static (과거 {kw["window"]}s 중 {kw["n_past"]}개 스캔과 비교)')

    def _cb(self, cloud, odom):
        t0 = time.perf_counter()
        st = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        q = odom.pose.pose.orientation
        R = Rot.from_quat([q.x, q.y, q.z, q.w])
        t = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z])
        pb, raw = _xyz(cloud)
        dyn = self.core.process(st, R, t, pb)
        self._publish(self.pub_static, cloud, raw[~dyn])
        if self.pub_dyn.get_subscription_count() > 0:
            self._publish(self.pub_dyn, cloud, raw[dyn])
        self.stats.append(((time.perf_counter() - t0) * 1000, int(dyn.sum()), len(pb)))

    @staticmethod
    def _publish(pub, src, rows):
        m = PointCloud2()
        m.header = src.header
        m.height = 1
        m.width = len(rows)
        m.fields = src.fields
        m.is_bigendian = src.is_bigendian
        m.point_step = src.point_step
        m.row_step = src.point_step * len(rows)
        m.is_dense = src.is_dense
        m.data = rows.tobytes()
        pub.publish(m)

    def _log(self):
        if not self.stats:
            return
        a = np.array(self.stats)
        self.get_logger().info(f'처리 평균 {a[:, 0].mean():.1f} ms (최대 {a[:, 0].max():.1f}), '
                               f'동적 점 {100 * a[:, 1].sum() / max(a[:, 2].sum(), 1):.2f}%, 기억 voxel {len(self.core.track)}')


def main():
    rclpy.init()
    node = DynamicFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
