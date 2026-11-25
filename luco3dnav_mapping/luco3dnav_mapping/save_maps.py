"""지도 한 번에 저장: PGO 3D 점군 + octomap voxel + 2D occupancy + 2.5D elevation -> maps/pcd/<이름>/

  ros2 run luco3dnav_mapping save_maps                 # 이름 = luco_<월일_시분>
  ros2 run luco3dnav_mapping save_maps my_map          # 이름 지정
  ros2 run luco3dnav_mapping save_maps my_map --tar    # maps/pcd/my_map.tar.gz 도 만듦
  ros2 run luco3dnav_mapping save_maps --no-patches    # PGO 키프레임 패치 생략 (동적 물체 제거도 생략됨)
  ros2 run luco3dnav_mapping save_maps --keep-dynamic  # 사람 등 움직인 물체 제거 안 함
  ros2 run luco3dnav_mapping save_maps --rebuild-layers  # octomap/2D/elevation을 실시간 결과 대신
                                                          # PGO 최종 자세 + 정적 점으로 다시 만듦 (rebuild_layers, 약 1분)

SLAM(PGO)이 돌고 있는 PC에서 실행해야 한다 (PGO가 자기 디스크에 map.pcd를 씀).
  로봇 PC -> ~/luco_ws/src/luco3dnav/maps/pcd/<이름>,  개발 PC -> <워크스페이스>/maps/pcd/<이름>

저장 파일
  map.pcd, poses.txt, patches/   PGO 3D 점군 (localizer용). 움직인 물체(사람 등) 제거된 정적 지도
  map_raw.pcd, map_dynamic.pcd   제거 전 원본 / 제거된 점 (remove_dynamic.py 참고)
  octomap.bt                     3D voxel (octovis / octomap_server <파일> 로 다시 띄우기)
  grid.pgm, grid.yaml            2D occupancy (Nav2 map_server 형식)
  elevation.npz                  2.5D elevation 원본 (elevation/min/count float32, NaN = 없음)
  elevation.pgm, elevation.yaml  elevation 미리보기 이미지 (밝을수록 높음, 0 = 없음)
  info.yaml                      저장 시각, 각 항목 성공 여부
"""
import argparse
import os
import shutil
import subprocess
import tarfile
import time
from datetime import datetime

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def default_map_root():
    """install/<pkg>/share/<pkg> -> 워크스페이스 루트. 개발 PC는 <ws>/maps, 로봇 PC는 clone 안 maps."""
    ws = os.path.abspath(os.path.join(get_package_share_directory('luco3dnav_mapping'), '..', '..', '..', '..'))
    if os.path.isdir(os.path.join(ws, 'maps')):
        return os.path.join(ws, 'maps', 'pcd')
    return os.path.join(ws, 'src', 'luco3dnav', 'maps', 'pcd')


def write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())


class Saver(Node):
    def __init__(self):
        super().__init__('save_maps')
        self.msgs = {}
        # octomap_server(latch: true) / elevation_mapper 둘 다 transient_local 로 발행 -> 마지막 맵을 바로 받음
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/projected_map', lambda m: self.msgs.setdefault('grid', m), qos)
        self.create_subscription(GridMap, '/elevation_mapper/elevation_map',
                                 lambda m: self.msgs.setdefault('elevation', m), qos)

    def wait_msgs(self, keys, timeout):
        end = time.time() + timeout
        while time.time() < end and not all(k in self.msgs for k in keys):
            rclpy.spin_once(self, timeout_sec=0.1)

    def save_pgo(self, out, patches):
        from interface.srv import SaveMaps
        cli = self.create_client(SaveMaps, '/pgo/save_maps')
        if not cli.wait_for_service(timeout_sec=5.0):
            return False, '/pgo/save_maps 서비스 없음 (SLAM 안 떠 있음?)'
        fut = cli.call_async(SaveMaps.Request(file_path=out, save_patches=patches))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=120.0)
        if not fut.done():
            return False, '응답 시간 초과'
        r = fut.result()
        return r.success, r.message or 'map.pcd'

    def save_grid(self, out):
        m = self.msgs.get('grid')
        if m is None:
            return False, '/projected_map 수신 안 됨'
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        img = np.full(g.shape, 205, np.uint8)       # unknown
        img[g == 0] = 254                           # free
        img[g >= 65] = 0                            # occupied
        write_pgm(os.path.join(out, 'grid.pgm'), img[::-1])   # 이미지 위쪽 = +y
        o = m.info.origin.position
        q = m.info.origin.orientation
        yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        with open(os.path.join(out, 'grid.yaml'), 'w') as f:
            f.write(f'image: grid.pgm\nmode: trinary\nresolution: {m.info.resolution}\n'
                    f'origin: [{o.x}, {o.y}, {yaw}]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        return True, f'{m.info.width}x{m.info.height} @ {m.info.resolution:.2f} m, 장애물 {(g >= 65).sum()} 셀'

    def save_elevation(self, out):
        m = self.msgs.get('elevation')
        if m is None:
            return False, '/elevation_mapper/elevation_map 수신 안 됨'
        layers = {}
        for name, arr in zip(m.layers, m.data):
            cols = arr.layout.dim[0].size
            rows = arr.layout.dim[1].size
            layers[name] = np.array(arr.data, dtype=np.float32).reshape((rows, cols), order='F')
        res = m.info.resolution
        px, py = m.info.pose.position.x, m.info.pose.position.y
        # grid_map index(i: x 감소, j: y 감소) -> 이미지(가로 = +x, 위쪽 = +y)
        to_img = {k: v[::-1, :].T for k, v in layers.items()}
        x0 = px - m.info.length_x / 2
        y0 = py - m.info.length_y / 2
        np.savez_compressed(os.path.join(out, 'elevation.npz'), resolution=res, origin=np.array([x0, y0]),
                            frame=m.header.frame_id,
                            note='배열[row, col]: row 0 = 위쪽(+y 끝), col 0 = x0. 셀 중심 = '
                                 '(x0 + (col+0.5)*res, y0 + (rows-1-row+0.5)*res)',
                            **to_img)
        e = to_img['elevation']
        valid = ~np.isnan(e)
        if not valid.any():
            return False, 'elevation 비어 있음'
        zmin, zmax = float(np.nanmin(e)), float(np.nanmax(e))
        img = np.zeros(e.shape, np.uint8)
        img[valid] = (1 + (e[valid] - zmin) / max(zmax - zmin, 1e-6) * 254).astype(np.uint8)
        write_pgm(os.path.join(out, 'elevation.pgm'), img)
        with open(os.path.join(out, 'elevation.yaml'), 'w') as f:
            f.write(f'image: elevation.pgm   # 1..255 = z_min..z_max, 0 = 데이터 없음 (원본은 elevation.npz)\n'
                    f'resolution: {res}\norigin: [{x0}, {y0}, 0.0]\nz_min: {zmin:.3f}\nz_max: {zmax:.3f}\n'
                    f'frame: {m.header.frame_id}\n')
        return True, f'유효 셀 {int(valid.sum())}, z {zmin:.2f} ~ {zmax:.2f} m'


def remove_dynamic(out):
    try:
        p = subprocess.run(['ros2', 'run', 'luco3dnav_mapping', 'remove_dynamic', out, '--replace'],
                           capture_output=True, text=True, timeout=600)
    except subprocess.TimeoutExpired:
        return False, '시간 초과'
    lines = [ln for ln in (p.stdout + p.stderr).splitlines() if ln.startswith('동적 점')]
    if p.returncode != 0 or not lines:
        tail = (p.stderr or p.stdout).strip().splitlines()
        return False, tail[-1] if tail else '실패'
    return True, lines[-1] + ' (원본 map_raw.pcd)'


def rebuild_layers(out, keep_dynamic):
    cmd = ['ros2', 'run', 'luco3dnav_mapping', 'rebuild_layers', out] + (['--keep-dynamic'] if keep_dynamic else [])
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        return False, '시간 초과'
    lines = [ln.strip() for ln in p.stdout.splitlines() if ln.strip().startswith('[')]
    if p.returncode != 0:
        tail = (p.stderr or p.stdout).strip().splitlines()
        return False, '; '.join(lines) or (tail[-1] if tail else '실패')
    return True, '; '.join(lines)


def save_octomap(out):
    path = os.path.join(out, 'octomap.bt')
    try:
        p = subprocess.run(['ros2', 'run', 'octomap_server', 'octomap_saver_node',
                            '--ros-args', '-p', f'octomap_path:={path}'],
                           capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        return False, 'octomap_saver_node 시간 초과 (octomap_server 안 떠 있음?)'
    if os.path.isfile(path) and os.path.getsize(path) > 0:
        return True, f'{os.path.getsize(path) / 1e3:.0f} KB'
    lines = (p.stderr or p.stdout).strip().splitlines()
    return False, lines[-1] if lines else 'octomap 저장 실패'


def main():
    ap = argparse.ArgumentParser(description='PGO 점군 + octomap + 2D + elevation 한 번에 저장')
    ap.add_argument('name', nargs='?', default=datetime.now().strftime('luco_%m%d_%H%M'))
    ap.add_argument('--dir', default=None, help='저장할 상위 폴더 (기본: maps/pcd)')
    ap.add_argument('--no-patches', action='store_true', help='PGO 키프레임 패치 생략')
    ap.add_argument('--tar', action='store_true', help='<이름>.tar.gz 한 파일로도 묶기')
    ap.add_argument('--keep-dynamic', action='store_true', help='움직인 물체(사람 등) 제거 안 함')
    ap.add_argument('--rebuild-layers', action='store_true',
                    help='레이어를 실시간 결과 대신 키프레임으로 다시 만듦 (rebuild_layers, 패치 필요)')
    args, ros_args = ap.parse_known_args()

    root = os.path.abspath(os.path.expanduser(args.dir or default_map_root()))
    out = os.path.join(root, args.name)
    os.makedirs(out, exist_ok=True)
    print(f'[save_maps] 저장 위치: {out}')

    rclpy.init(args=ros_args)
    node = Saver()
    results = {}
    try:
        node.wait_msgs(['grid', 'elevation'], timeout=5.0)
        # 같은 이름에 다시 저장: 이전 저장의 부산물(제거 전 원본, 이전 레이어) 정리
        for f in ('map_raw.pcd', 'map_dynamic.pcd', 'map_static.pcd'):
            if os.path.exists(os.path.join(out, f)):
                os.remove(os.path.join(out, f))
        shutil.rmtree(os.path.join(out, 'layers_old'), ignore_errors=True)
        results['pgo (map.pcd)'] = node.save_pgo(out, not args.no_patches)
        if results['pgo (map.pcd)'][0] and not args.no_patches and not args.keep_dynamic:
            results['동적 물체 제거 (map.pcd)'] = remove_dynamic(out)
        if args.rebuild_layers and results['pgo (map.pcd)'][0] and not args.no_patches:
            results['레이어 재생성 (octomap/2D/elevation)'] = rebuild_layers(out, args.keep_dynamic)
        else:
            results['octomap (octomap.bt)'] = save_octomap(out)
            node.wait_msgs(['grid', 'elevation'], timeout=3.0)
            results['2D (grid.pgm/yaml)'] = node.save_grid(out)
            results['elevation (elevation.npz/pgm)'] = node.save_elevation(out)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    with open(os.path.join(out, 'info.yaml'), 'w') as f:
        f.write(f'saved: {datetime.now().isoformat(timespec="seconds")}\n')
        for k, (ok, msg) in results.items():
            f.write(f'"{k}": {{ok: {str(ok).lower()}, msg: "{msg}"}}\n')
    for k, (ok, msg) in results.items():
        print(f'  [{"OK" if ok else "실패"}] {k}: {msg}')

    if args.tar:
        tar_path = out + '.tar.gz'
        with tarfile.open(tar_path, 'w:gz') as t:
            t.add(out, arcname=args.name)
        print(f'  [OK] 한 파일: {tar_path} ({os.path.getsize(tar_path) / 1e6:.1f} MB)')
    return 0 if all(ok for ok, _ in results.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
