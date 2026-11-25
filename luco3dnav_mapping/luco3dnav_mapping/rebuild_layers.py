"""저장된 지도 폴더에서 octomap(3D voxel) / 2D occupancy / 2.5D elevation 다시 만들기 (오프라인)

  ros2 run luco3dnav_mapping rebuild_layers maps/pcd/<이름>
  ros2 run luco3dnav_mapping rebuild_layers maps/pcd/<이름> --keep-dynamic   # 사람 등 제거 안 하고

실시간 레이어와 다른 점
  - 키프레임을 PGO 최종(루프 보정 후) 자세로 넣음. 실시간은 넣는 순간의 보정만 반영돼서 루프가 닫혀도
    이미 쌓인 voxel은 안 고쳐짐.
  - remove_dynamic 판정으로 움직인 물체(사람 등) 점을 빼고 넣음.
  - raycast 원점을 라이다 위치로 둠 (실시간은 base_footprint, 바닥).
  - 입력은 키프레임(0.5 m / 10도 간격)만이라 실시간(모든 스캔)보다 관측 수는 적음.

필요: patches/ + poses.txt (save_maps 기본값으로 저장한 폴더)
결과: octomap.bt, grid.pgm/yaml, elevation.npz/pgm/yaml 덮어씀 (기존 파일은 layers_old/ 로 옮김)
      octomap_server / elevation_mapper 를 자체 ROS 도메인(기본 87, localhost)에서 띄우므로
      로봇에서 SLAM이 돌고 있어도 섞이지 않음.
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from luco3dnav_mapping.remove_dynamic import add_args, classify

OLD_FILES = ['octomap.bt', 'grid.pgm', 'grid.yaml', 'elevation.npz', 'elevation.pgm', 'elevation.yaml']


def main():
    ap = argparse.ArgumentParser(description='저장된 키프레임으로 octomap/2D/elevation 다시 만들기')
    ap.add_argument('map_dir')
    ap.add_argument('--keep-dynamic', action='store_true', help='움직인 물체 제거 안 함')
    ap.add_argument('--domain', type=int, default=87, help='재생성용 ROS 도메인 (실행 중인 SLAM과 분리)')
    ap.add_argument('--rate', type=float, default=10.0, help='키프레임 넣는 속도 (개/s)')
    add_args(ap)
    args = ap.parse_args()
    d = os.path.abspath(args.map_dir)

    # 자체 도메인 + localhost: rclpy init / 하위 노드 실행 전에 설정
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ.pop('CYCLONEDDS_URI', None)   # luco_net.sh 의 유니캐스트 피어 설정 무시

    kfs, P, I, K, dyn = classify(d, args)
    if args.keep_dynamic:
        dyn[:] = False
    t_bs = np.array([float(v) for v in args.sensor.split(',')])

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    from tf2_ros import TransformBroadcaster
    from ament_index_python.packages import get_package_share_directory
    from luco3dnav_mapping.save_maps import Saver, save_octomap

    share = get_package_share_directory('luco3dnav_mapping')
    procs = [
        subprocess.Popen(['ros2', 'run', 'octomap_server', 'octomap_server_node', '--ros-args',
                          '-r', '__node:=octomap_server', '--params-file', os.path.join(share, 'config', 'octomap.yaml'),
                          '-r', 'cloud_in:=/rebuild/cloud', '-p', 'base_frame_id:=base_footprint'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True),
        subprocess.Popen(['ros2', 'run', 'luco3dnav_mapping', 'elevation_mapper', '--ros-args',
                          '--params-file', os.path.join(share, 'config', 'elevation.yaml'),
                          '-p', 'cloud_topic:=/rebuild/cloud', '-p', 'tf_wait:=5.0'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True),
    ]

    rclpy.init()
    node = Node('rebuild_layers')
    pub = node.create_publisher(PointCloud2, '/rebuild/cloud', 10)
    tfb = TransformBroadcaster(node)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
              for i, n in enumerate(('x', 'y', 'z', 'intensity'))]

    def tf_msg(stamp, child, t, q_xyzw):
        m = TransformStamped()
        m.header.stamp = stamp
        m.header.frame_id = 'map'
        m.child_frame_id = child
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, t)
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = map(float, q_xyzw)
        return m

    try:
        # 하위 노드가 구독할 때까지 대기
        end = time.time() + 20
        while time.time() < end and pub.get_subscription_count() < 2:
            rclpy.spin_once(node, timeout_sec=0.2)
        if pub.get_subscription_count() < 2:
            print('octomap_server / elevation_mapper 가 안 뜸', file=sys.stderr)
            return 1
        time.sleep(1.0)

        print(f'키프레임 {len(kfs)}개 넣는 중 (정적 점 {int((~dyn).sum())}개)...')
        for k, (_, t, q) in enumerate(kfs):
            R = Rot.from_quat([q[1], q[2], q[3], q[0]])
            sel = (K == k) & ~dyn
            # map 좌표 -> 라이다(축 = base_footprint 축, 원점 = 라이다) 좌표
            pts = R.inv().apply(P[sel] - t) - t_bs
            arr = np.zeros((len(pts), 4), np.float32)
            arr[:, :3] = pts
            arr[:, 3] = I[sel]
            stamp = node.get_clock().now().to_msg()
            qx = R.as_quat()
            tfb.sendTransform([tf_msg(stamp, 'base_footprint', t, qx),
                               tf_msg(stamp, 'rebuild_lidar', t + R.apply(t_bs), qx)])
            msg = PointCloud2(header=Header(stamp=stamp, frame_id='rebuild_lidar'), height=1, width=len(pts),
                              fields=fields, is_bigendian=False, point_step=16, row_step=16 * len(pts),
                              is_dense=True, data=arr.tobytes())
            time.sleep(0.02)   # TF 가 먼저 도착하도록
            pub.publish(msg)
            t_next = time.time() + 1.0 / args.rate
            while time.time() < t_next:
                rclpy.spin_once(node, timeout_sec=0.01)
            if (k + 1) % 50 == 0:
                print(f'  {k + 1}/{len(kfs)}')
        # 마지막 처리 + elevation 발행 주기(2 Hz) 대기
        end = time.time() + 4
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)

        old = os.path.join(d, 'layers_old')
        os.makedirs(old, exist_ok=True)
        for f in OLD_FILES:
            src = os.path.join(d, f)
            if os.path.exists(src) and not os.path.exists(os.path.join(old, f)):
                shutil.move(src, os.path.join(old, f))
            elif os.path.exists(src):
                os.remove(src)

        saver = Saver()
        saver.wait_msgs(['grid', 'elevation'], timeout=10.0)
        results = {
            'octomap (octomap.bt)': save_octomap(d),
            '2D (grid.pgm/yaml)': saver.save_grid(d),
            'elevation (elevation.npz/pgm)': saver.save_elevation(d),
        }
        saver.destroy_node()
        for k, (ok, m) in results.items():
            print(f'  [{"OK" if ok else "실패"}] {k}: {m}')
        with open(os.path.join(d, 'info.yaml'), 'a') as f:
            f.write(f'rebuild_layers: "{time.strftime("%Y-%m-%dT%H:%M:%S")}, 키프레임 {len(kfs)}개, '
                    f'동적 제거 {"안 함" if args.keep_dynamic else "함"} (이전 레이어: layers_old/)"\n')
            for k, (ok, m) in results.items():
                f.write(f'"rebuild {k}": {{ok: {str(ok).lower()}, msg: "{m}"}}\n')
        return 0 if all(ok for ok, _ in results.values()) else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        # 각 하위 노드는 자기 프로세스 그룹 (ros2 run + 실제 노드) -> 그룹째 종료
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)

if __name__ == '__main__':
    raise SystemExit(main())
