"""기준 지도 정하기: 지도 폴더의 결과물을 maps/nav/ 로 복사 (localization.launch.py 와 전역 경로 계획이 쓰는 지도)

  ros2 run luco3dnav_mapping set_nav_map maps/pcd/luco_1126_merged
  ros2 run luco3dnav_mapping set_nav_map maps/pcd/luco_1126_merged --dest <폴더>

복사하는 것 (있는 것만)
  map.pcd                  3D 점군 -> localizer (초기 위치 찾기 + 실시간 위치 추적)
  grid.pgm, grid.yaml      2D occupancy -> Nav2 map_server / 전역 경로 계획
  elevation.npz/pgm/png/yaml  2.5D elevation
  voxels.pcd, voxels.ply   높이 색 voxel
  octomap.bt               3D voxel (octomap)
  poses.txt                키프레임 위치 (바닥 높이 추정용, 작음)
이전 maps/nav 는 maps/nav_old 로 옮김. 어느 지도에서 왔는지는 maps/nav/source.yaml.
"""
import argparse
import os
import shutil
from datetime import datetime

FILES = ['map.pcd', 'grid.pgm', 'grid.yaml', 'elevation.npz', 'elevation.pgm', 'elevation.png', 'elevation.yaml',
         'voxels.pcd', 'voxels.ply', 'octomap.bt', 'poses.txt', 'info.yaml', 'merge_info.yaml']
REQUIRED = ['map.pcd', 'grid.pgm', 'grid.yaml']


def default_nav_dir():
    from luco3dnav_mapping.save_maps import default_map_root
    return os.path.join(os.path.dirname(default_map_root()), 'nav')


def main(argv=None):
    ap = argparse.ArgumentParser(description='지도 폴더 -> 기준 지도(maps/nav)')
    ap.add_argument('map_dir')
    ap.add_argument('--dest', default=None, help='기본: maps/nav')
    a = ap.parse_args(argv)
    src = os.path.abspath(os.path.expanduser(a.map_dir))
    dest = os.path.abspath(os.path.expanduser(a.dest or default_nav_dir()))
    miss = [f for f in REQUIRED if not os.path.isfile(os.path.join(src, f))]
    if miss:
        raise SystemExit(f'{src} 에 {", ".join(miss)} 없음')
    if os.path.isdir(dest):
        old = dest.rstrip('/') + '_old'
        shutil.rmtree(old, ignore_errors=True)
        shutil.move(dest, old)
        print(f'이전 기준 지도 -> {old}')
    os.makedirs(dest)
    got = []
    for f in FILES:
        p = os.path.join(src, f)
        if os.path.isfile(p):
            shutil.copy2(p, dest)
            got.append(f)
    with open(os.path.join(dest, 'source.yaml'), 'w') as f:
        f.write(f'source: {src}\nset: {datetime.now().isoformat(timespec="seconds")}\nfiles: [{", ".join(got)}]\n')
    print(f'기준 지도: {dest}')
    for f in got:
        print(f'  {f:16s} {os.path.getsize(os.path.join(dest, f)) / 1e6:7.1f} MB')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
