"""초기 위치 찾기: 시작 직후 몇 초 스캔을 모아 기준 지도(maps/nav/map.pcd)에서 로봇 위치를 찾고 localizer 에 넘김

  localization.launch.py 가 띄움. 따로:  ros2 run luco3dnav_mapping global_localize --ros-args -p map_dir:=<폴더>

1. FAST-LIO body_cloud + lio_odom 을 collect_sec 초 모아 lio_world 좌표 점군을 만든다 (로봇이 서 있어도 됨).
2. 벽 높이(바닥 + band_low ~ band_high) 점을 2D 이미지로 만들고, yaw_step 도마다 돌려 FFT 상호상관으로
   지도에서 가장 잘 겹치는 (yaw, x, y) 후보를 찾는다 (초기 추정 필요 없음).
3. 상위 후보마다 point-to-plane ICP (6 DoF) -> 스캔 점이 지도 면에 붙는 비율(score)이 가장 높은 것을 고른다.
4. /localizer/relocalize 로 현재 자세를 넘기고 /localizer/relocalize_check 로 성공 확인. 그 뒤 추적은 localizer 가 함
   (TF map -> lio_world).
실패하거나 애매하면(대칭 복도 등) 더 모아서 다시 시도. RViz "2D Pose Estimate"(/initialpose)로 직접 줘도 된다.
"""
import os
import time

import numpy as np
import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from luco3dnav_mapping.remove_dynamic import read_pcd, write_pcd


def voxel(p, s):
    k = np.floor(p / s).astype(np.int64)
    k -= k.min(0)
    key = (k[:, 0] * (k[:, 1].max() + 1) + k[:, 1]) * (k[:, 2].max() + 1) + k[:, 2]
    _, i = np.unique(key, return_index=True)
    return p[i]


def raster(p2, lo, shape, res):
    img = np.zeros(shape, np.float32)
    ij = np.floor((p2 - lo) / res).astype(int)
    ok = (ij >= 0).all(1) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
    img[ij[ok, 0], ij[ok, 1]] = 1.0
    return img


def normals(tree, pts, k=12):
    _, nb = tree.query(pts, k=k, workers=-1)
    X = pts[nb] - pts[nb].mean(1, keepdims=True)
    return np.linalg.eigh(np.einsum('nki,nkj->nij', X, X))[1][:, :, 0]


class MapModel:
    def __init__(self, map_dir, band, res, pad, log):
        t0 = time.time()
        P, _ = read_pcd(os.path.join(map_dir, 'map.pcd'))
        pp = os.path.join(map_dir, 'poses.txt')
        if os.path.isfile(pp):
            z = [float(ln.split()[3]) for ln in open(pp) if len(ln.split()) == 8]
            self.floor = float(np.median(z))
        else:
            self.floor = float(np.percentile(P[:, 2], 5))
        self.P = voxel(P, 0.1)
        self.tree = cKDTree(self.P)
        self.N = normals(self.tree, self.P)
        w = self.P[(self.P[:, 2] > self.floor + band[0]) & (self.P[:, 2] < self.floor + band[1]), :2]
        self.res = res
        self.lo = w.min(0) - pad
        self.shape = tuple(np.ceil((w.max(0) + pad - self.lo) / res).astype(int))
        self.F = np.fft.rfft2(raster(w, self.lo, self.shape, res))
        self.center = w.mean(0)
        log(f'지도 {len(P)}점 -> {len(self.P)} (0.1 m), 벽 {len(w)}점, 바닥 z {self.floor:.2f}, '
            f'{time.time() - t0:.1f}s')


def candidates(M, wall2, yaw_step, k):
    """-> [(corr, yaw_deg, t2)] 서로 다른 후보 k개"""
    cb = wall2.mean(0)
    per = []
    for yaw in np.arange(0.0, 360.0, yaw_step):
        R2 = Rot.from_euler('z', yaw, degrees=True).as_matrix()[:2, :2]
        q = (wall2 - cb) @ R2.T + M.center
        corr = np.fft.irfft2(M.F * np.conj(np.fft.rfft2(raster(q, M.lo, M.shape, M.res))), s=M.shape)
        # yaw 마다 상위 3개 peak (같은 yaw 에서 다른 위치도 후보로)
        flat = corr.ravel()
        for idx in np.argpartition(flat, -3)[-3:]:
            di, dj = np.unravel_index(idx, M.shape)
            di = di - M.shape[0] if di > M.shape[0] // 2 else di
            dj = dj - M.shape[1] if dj > M.shape[1] // 2 else dj
            t2 = M.center - R2 @ cb + np.array([di, dj]) * M.res
            per.append((float(flat[idx]), yaw, t2))
    per.sort(key=lambda c: -c[0])
    out = []
    for c in per:
        if all(np.linalg.norm(c[2] - o[2]) > 1.0 or abs((c[1] - o[1] + 180) % 360 - 180) > 15 for o in out):
            out.append(c)
        if len(out) >= k:
            break
    return out


def icp(M, src, R, t, max_dist=(1.0, 0.5, 0.25), iters=15):
    for md in max_dist:
        for _ in range(iters):
            p = R.apply(src) + t
            d, j = M.tree.query(p, distance_upper_bound=md, workers=-1)
            v = np.isfinite(d)
            if v.sum() < 50:
                return R, t, 0.0
            n = M.N[j[v]]
            r = np.einsum('ij,ij->i', n, p[v] - M.P[j[v]])
            J = np.hstack([np.cross(p[v], n), n])
            dx = -np.linalg.solve(J.T @ J + 1e-6 * np.eye(6), J.T @ r)
            Rd = Rot.from_rotvec(dx[:3])
            R, t = Rd * R, Rd.apply(t) + dx[3:]
            if np.linalg.norm(dx) < 1e-5:
                break
    p = R.apply(src) + t
    d, j = M.tree.query(p, distance_upper_bound=0.3, workers=-1)
    v = np.isfinite(d)
    pl = np.abs(np.einsum('ij,ij->i', M.N[j[v]], p[v] - M.P[j[v]]))
    return R, t, float((pl < 0.1).sum() / len(src))


class GlobalLocalize(Node):
    def __init__(self):
        super().__init__('global_localize')
        dp = self.declare_parameter
        self.map_dir = dp('map_dir', '').value
        if not self.map_dir:
            from luco3dnav_mapping.set_nav_map import default_nav_dir
            self.map_dir = default_nav_dir()
        self.collect = dp('collect_sec', 3.0).value
        self.band = (dp('band_low', 0.3).value, dp('band_high', 1.8).value)
        self.yaw_step = dp('yaw_step', 2.0).value
        self.n_cand = dp('candidates', 8).value
        self.accept = dp('accept_score', 0.5).value
        self.ambiguous = dp('ambiguous_ratio', 0.9).value
        self.retries = dp('retries', 5).value
        self.auto = dp('auto', True).value
        self.warmup = dp('warmup_sec', 1.0).value
        self.t_lio0 = None
        self.dump = dp('dump_dir', os.environ.get('GLOBAL_LOCALIZE_DUMP', '')).value
        res = dp('raster_res', 0.2).value
        src = os.path.join(self.map_dir, 'map.pcd')
        if not os.path.isfile(src):
            raise SystemExit(f'기준 지도 없음: {src}  (set_nav_map 으로 정하기)')
        self.M = MapModel(self.map_dir, self.band, res, pad=20.0, log=self.get_logger().info)
        # localizer 는 relocalize 때마다 지도를 읽고 0.1 m 로 줄임 -> 미리 줄인 지도를 주면 수 초 -> 0.1 초
        # (그동안 로봇이 움직이면 넘긴 자세가 낡아서 ICP 가 실패함)
        self.map_pcd = os.path.join(self.map_dir, 'map_localizer.pcd')
        if not os.path.isfile(self.map_pcd) or os.path.getmtime(self.map_pcd) < os.path.getmtime(src):
            write_pcd(self.map_pcd, self.M.P, np.zeros(len(self.M.P), np.float32))
        self.odom = {}
        self.last_odom = None
        self.clouds = []
        self.manual = None
        self.done = False
        self.create_subscription(Odometry, dp('odom_topic', '/fastlio2/lio_odom').value, self.on_odom, 50)
        self.create_subscription(PointCloud2, dp('cloud_topic', '/fastlio2/body_cloud').value, self.on_cloud, 20)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initialpose, 1)
        from interface.srv import IsValid, Relocalize
        self.Relocalize, self.IsValid = Relocalize, IsValid
        self.cli = self.create_client(Relocalize, '/localizer/relocalize')
        self.check = self.create_client(IsValid, '/localizer/relocalize_check')

    @staticmethod
    def stamp(h):
        return h.stamp.sec + h.stamp.nanosec * 1e-9

    @staticmethod
    def pose(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        return Rot.from_quat([q.x, q.y, q.z, q.w]), np.array([p.x, p.y, p.z])

    def on_odom(self, m):
        self.last_odom = self.pose(m)
        self.odom[round(self.stamp(m.header), 3)] = self.last_odom
        if len(self.odom) > 200:
            self.odom.pop(next(iter(self.odom)))

    def on_cloud(self, m):
        st = self.stamp(m.header)
        o = self.odom.get(round(st, 3))
        if o is None:
            return
        if self.t_lio0 is None:
            self.t_lio0 = st
        # FAST-LIO 초기화 직후 몇 프레임은 자세가 엉망(10^12 m 등) -> 처음 warmup 초와 비정상 자세는 버림
        if st - self.t_lio0 < self.warmup or not np.all(np.isfinite(o[1])) or np.abs(o[1]).max() > 1e3:
            return
        xyz = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'z'), skip_nans=True).astype(np.float64)
        xyz = xyz[np.abs(xyz).max(1) < 100.0]
        self.clouds.append((st, o[0].apply(xyz) + o[1]))

    def on_initialpose(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.manual = (Rot.from_quat([q.x, q.y, q.z, q.w]).as_euler('zyx')[0], p.x, p.y)

    # ------------------------------------------------------------------
    def wait_scans(self, sec):
        self.clouds = []
        t_first = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.manual:
                return None
            if self.clouds and t_first is None:
                t_first = self.clouds[0][0]
            if t_first is not None and self.clouds[-1][0] - t_first >= sec:
                return np.vstack([c for _, c in self.clouds])
        return None

    def solve(self, S):
        """S: lio_world 점 -> (R, t, score, 2등 score) : map <- lio_world"""
        lo, hi = self.last_odom[1][2] + self.band[0], self.last_odom[1][2] + self.band[1]
        S = voxel(S, 0.1)
        wall = S[(S[:, 2] > lo) & (S[:, 2] < hi), :2]
        if len(wall) < 300:
            return None
        src = voxel(S, 0.2)
        dz = self.M.floor - self.last_odom[1][2]
        res = []
        for corr, yaw, t2 in candidates(self.M, wall, self.yaw_step, self.n_cand):
            R, t, sc = icp(self.M, src, Rot.from_euler('z', yaw, degrees=True), np.r_[t2, dz])
            res.append((sc, R, t, yaw, corr))
        res.sort(key=lambda r: -r[0])
        best = res[0]
        # 2등: 1등과 다른 자세(1 m / 15도 이상)인 후보 중 최고
        second = 0.0
        for r in res[1:]:
            dyaw = abs((r[1] * best[1].inv()).magnitude())
            if np.linalg.norm(r[2][:2] - best[2][:2]) > 1.0 or dyaw > np.radians(15):
                second = r[0]
                break
        return best[1], best[2], best[0], second

    def relocalize_offset(self, R, t):
        """map <- lio_world (R, t) -> 가장 최근 odom 으로 현재 자세를 만들어 넘김 (계산하는 동안 움직인 것 반영)"""
        end = time.time() + 0.2
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        R_lb, t_lb = self.last_odom
        return self.relocalize(R * R_lb, R.apply(t_lb) + t)

    def relocalize(self, R_mb, t_mb):
        yaw, roll, pitch = R_mb.as_euler('ZXY')        # localizer: R = Rz(yaw) Rx(roll) Ry(pitch)
        req = self.Relocalize.Request(pcd_path=self.map_pcd, x=float(t_mb[0]), y=float(t_mb[1]), z=float(t_mb[2]),
                                      yaw=float(yaw), roll=float(roll), pitch=float(pitch))
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60.0)
        if not fut.done() or not fut.result().success:
            self.get_logger().error(f'relocalize 호출 실패: {fut.result().message if fut.done() else "시간 초과"}')
            return False
        end = time.time() + 15.0
        while time.time() < end:
            f = self.check.call_async(self.IsValid.Request(code=0))
            rclpy.spin_until_future_complete(self, f, timeout_sec=2.0)
            if f.done() and f.result().valid:
                return True
            t = time.time() + 0.5
            while time.time() < t:
                rclpy.spin_once(self, timeout_sec=0.05)
        return False

    def run(self):
        log = self.get_logger()
        if not self.cli.wait_for_service(timeout_sec=30.0):
            log.error('/localizer/relocalize 서비스 없음 (localizer_node 안 뜸?)')
            return 1
        attempt, sec = 0, self.collect
        while rclpy.ok():
            if self.manual or not self.auto or attempt >= self.retries:
                if not self.manual and not self.done:
                    log.warn('자동 위치 찾기 안 함/실패 -> RViz "2D Pose Estimate" 로 위치를 찍어 주세요')
                while rclpy.ok() and not self.manual:
                    rclpy.spin_once(self, timeout_sec=0.1)
                if not rclpy.ok():
                    break
                yaw, x, y = self.manual
                self.manual = None
                z = self.M.floor
                ok = self.relocalize(Rot.from_euler('z', yaw), np.array([x, y, z]))
                log.info(f'수동 위치 ({x:.2f}, {y:.2f}, yaw {np.degrees(yaw):.0f}도): {"성공" if ok else "실패"}')
                continue
            attempt += 1
            log.info(f'[{attempt}/{self.retries}] 스캔 {sec:.0f}초 모으는 중...')
            S = self.wait_scans(sec)
            if S is None:
                continue
            if self.dump:
                np.savez(os.path.join(self.dump, f'scan_{attempt}.npz'), S=S, R=self.last_odom[0].as_quat(),
                         t=self.last_odom[1])
            t0 = time.time()
            r = self.solve(S)
            if r is None:
                log.warn('벽 점이 너무 적음 -> 더 모음')
                sec *= 2
                continue
            R, t, sc, second = r
            R_lb, t_lb = self.last_odom
            R_mb, t_mb = R * R_lb, R.apply(t_lb) + t
            e = R_mb.as_euler('zyx', degrees=True)
            log.info(f'후보 선택 ({time.time() - t0:.1f}s): 위치 ({t_mb[0]:.2f}, {t_mb[1]:.2f}, {t_mb[2]:.2f}) '
                     f'yaw {e[0]:.1f}도, score {sc:.2f} (2등 {second:.2f})')
            if sc < self.accept:
                log.warn(f'score {sc:.2f} < {self.accept} -> 더 모아서 다시 (로봇을 조금 움직이면 도움)')
                sec = min(sec * 2, 20.0)
                continue
            if second > self.ambiguous * sc:
                log.warn('비슷하게 맞는 다른 위치가 있음 (대칭 구조?) -> 더 모아서 다시')
                sec = min(sec * 2, 20.0)
                continue
            if self.relocalize_offset(R, t):
                log.info('초기 위치 찾기 성공 -> localizer 가 추적 중 (TF map -> lio_world)')
                self.done = True            # 이후엔 /initialpose 로만 다시
                attempt = self.retries
            else:
                log.warn('localizer ICP 가 수렴 안 함 -> 다시')
        return 0


def main():
    rclpy.init()
    node = GlobalLocalize()
    try:
        return node.run()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
