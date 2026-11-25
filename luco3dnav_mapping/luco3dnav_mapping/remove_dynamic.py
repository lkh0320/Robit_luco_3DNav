"""저장된 PGO 지도에서 움직이는 물체(사람 등) 점 지우기 (오프라인, 가시성 기반)

  ros2 run luco3dnav_mapping remove_dynamic maps/pcd/<이름>          # map_static.pcd, map_dynamic.pcd 생성
  ros2 run luco3dnav_mapping remove_dynamic maps/pcd/<이름> --replace  # map.pcd 를 정적 지도로 교체 (원본은 map_raw.pcd)

원리 (Removert/ERASOR 계열 '보이는데 없다' 판정)
  지도 점 p 를 각 키프레임 스캔의 거리 영상(16채널 x 방위각)에 투영했을 때, 그 스캔의 광선이
  p 보다 확실히 멀리까지 갔다면(= p 자리를 뚫고 지나감) 그 순간 p 자리는 비어 있었다는 뜻.
  여러 스캔에서 '뚫고 지나감'이 '거기서 맞음'보다 많으면 p 는 잠깐 있다 사라진 물체 -> 동적.
  사람이 지나간 자리는 나중에 로봇이 그 자리 너머 벽을 보면서 지워지고, 벽/기둥처럼 계속 있는 것은
  매번 '맞음'이 나와서 남는다.

필요: save_maps 를 패치 포함(기본값)으로 저장한 폴더 (patches/*.pcd + poses.txt)
주의: 바닥 근처(키프레임 높이 + floor_margin 아래)와 머리 위(+max_height 위)는 판정하지 않는다
      (낮은 각도로 비스듬히 맞는 바닥/천장은 '뚫고 지나감' 오판이 많음). 사람 높이 범위만 지운다.
      한 번도 다른 각도에서 다시 보지 못한 곳(로봇이 지나간 뒤 다시 안 본 곳)의 사람은 지울 근거가 없어서 남는다.
"""
import argparse
import os
import shutil

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as Rot

# VLP-16 채널 고도각(도): -15 ~ +15, 2도 간격
VLP16_EL = np.arange(-15.0, 15.1, 2.0)


def read_pcd(path):
    with open(path, 'rb') as f:
        h = {}
        while True:
            line = f.readline().decode('ascii', 'ignore').strip()
            if not line or line.startswith('#'):
                continue
            k = line.split()[0]
            h[k] = line.split()[1:]
            if k == 'DATA':
                break
        fields = h['FIELDS']
        sizes = [int(s) for s in h['SIZE']]
        types = h['TYPE']
        counts = [int(c) for c in h.get('COUNT', ['1'] * len(fields))]
        n = int(h['POINTS'][0])
        if h['DATA'][0] != 'binary':
            raise ValueError(f'{path}: binary PCD만 지원')
        dt = []
        for i, (name, s, t, c) in enumerate(zip(fields, sizes, types, counts)):
            base = {'F': 'f', 'U': 'u', 'I': 'i'}[t] + str(s)
            nm = name if name != '_' else f'_pad{i}'
            dt.append((nm, base) if c == 1 else (nm, base, (c,)))
        a = np.frombuffer(f.read(n * sum(s * c for s, c in zip(sizes, counts))), dtype=np.dtype(dt), count=n)
    xyz = np.stack([a['x'], a['y'], a['z']], 1).astype(np.float64)
    inten = a['intensity'].astype(np.float32) if 'intensity' in a.dtype.names else np.zeros(n, np.float32)
    return xyz, inten


def write_pcd(path, xyz, inten):
    n = len(xyz)
    a = np.zeros(n, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('intensity', 'f4')])
    a['x'], a['y'], a['z'], a['intensity'] = xyz[:, 0], xyz[:, 1], xyz[:, 2], inten
    with open(path, 'wb') as f:
        f.write((f'# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n'
                 f'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {n}\nHEIGHT 1\n'
                 f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n').encode())
        f.write(a.tobytes())


def load_keyframes(d):
    poses = []
    with open(os.path.join(d, 'poses.txt')) as f:
        for line in f:
            s = line.split()
            if len(s) == 8:
                poses.append((s[0], np.array(s[1:4], float), np.array(s[4:8], float)))  # t, q(wxyz)
    if not poses:
        raise SystemExit(f'{d}/poses.txt 가 비어 있음: save_maps 를 패치 포함으로 저장해야 함')
    return poses


def range_image(pts_s, az_res):
    """센서 좌표 점 -> (16, ncol) 최소 거리 영상 (없으면 inf)"""
    ncol = int(round(360.0 / az_res))
    r = np.linalg.norm(pts_s, axis=1)
    el = np.degrees(np.arcsin(np.clip(pts_s[:, 2] / np.maximum(r, 1e-6), -1, 1)))
    row = np.round((el - VLP16_EL[0]) / 2.0).astype(int)
    ok = (row >= 0) & (row < 16) & (np.abs(el - VLP16_EL[np.clip(row, 0, 15)]) < 0.8)
    col = (np.floor((np.degrees(np.arctan2(pts_s[:, 1], pts_s[:, 0])) + 180.0) / az_res).astype(int)) % ncol
    img = np.full((16, ncol), np.inf)
    np.minimum.at(img, (row[ok], col[ok]), r[ok])
    return img


def main():
    ap = argparse.ArgumentParser(description='PGO 지도에서 움직이는 물체(사람 등) 지우기')
    ap.add_argument('map_dir')
    ap.add_argument('--sensor', default='0.12545,0.0,0.3677',
                    help='base_footprint 기준 라이다 위치 x,y,z (luco2: base_link (0.12545, 0, 0.3064) + 0.0613)')
    ap.add_argument('--max-range', type=float, default=15.0, help='판정에 쓰는 최대 거리 (m)')
    ap.add_argument('--floor-margin', type=float, default=0.15, help='키프레임 바닥 높이 + 이 값 아래는 판정 안 함 (m)')
    ap.add_argument('--max-height', type=float, default=2.2, help='키프레임 바닥 높이 + 이 값 위는 판정 안 함 (m)')
    ap.add_argument('--min-see', type=int, default=3, help='동적 판정에 필요한 최소 "뚫고 지나감" 횟수')
    ap.add_argument('--ratio', type=float, default=0.6, help='뚫고 지나감 / (뚫고 지나감 + 맞음) 이 이 값 이상이면 동적')
    ap.add_argument('--az-res', type=float, default=1.0, help='거리 영상 방위각 해상도 (도)')
    ap.add_argument('--voxel', type=float, default=0.1, help='판정 단위 voxel (m). 같은 voxel 점은 같이 지움')
    ap.add_argument('--replace', action='store_true', help='map.pcd 를 정적 지도로 교체 (원본 map_raw.pcd 보관)')
    args = ap.parse_args()

    d = args.map_dir
    t_bs = np.array([float(v) for v in args.sensor.split(',')])
    kfs = load_keyframes(d)
    print(f'키프레임 {len(kfs)}개 읽는 중...')
    P, I, K, scans_s, origins, rots, floor_z = [], [], [], [], [], [], []
    for k, (name, t, q) in enumerate(kfs):
        xyz, inten = read_pcd(os.path.join(d, 'patches', name))
        R = Rot.from_quat([q[1], q[2], q[3], q[0]])
        P.append(R.apply(xyz) + t)
        I.append(inten)
        K.append(np.full(len(xyz), k, np.int32))
        scans_s.append(xyz - t_bs)          # 키프레임 센서 좌표 (라이다 축 = 로봇 축)
        origins.append(t + R.apply(t_bs))
        rots.append(R)
        floor_z.append(t[2])
    P = np.concatenate(P)
    I = np.concatenate(I)
    K = np.concatenate(K)
    origins = np.array(origins)
    floor_z = np.array(floor_z)
    print(f'지도 점 {len(P)}개')

    # voxel 대표점으로 판정 (속도 + 같은 voxel은 같은 운명)
    vkey = np.floor(P / args.voxel).astype(np.int64)
    _, vid, vinv = np.unique(vkey, axis=0, return_index=True, return_inverse=True)
    vinv = vinv.ravel()
    V = P[vid]
    # 판정 높이: 가장 가까운 키프레임 바닥 기준
    _, nk = cKDTree(origins[:, :2]).query(V[:, :2], workers=-1)
    rel = V[:, 2] - floor_z[nk]
    cand = (rel > args.floor_margin) & (rel < args.max_height)
    Vc = V[cand]
    print(f'voxel {len(V)}개 중 판정 대상(사람 높이) {len(Vc)}개')

    see = np.zeros(len(Vc), np.int32)
    hit = np.zeros(len(Vc), np.int32)
    tree = cKDTree(Vc)
    for k in range(len(kfs)):
        img = range_image(scans_s[k], args.az_res)
        idx = np.array(tree.query_ball_point(origins[k], args.max_range), dtype=np.int64)
        if len(idx) == 0:
            continue
        ps = rots[k].inv().apply(Vc[idx] - origins[k])
        r = np.linalg.norm(ps, axis=1)
        el = np.degrees(np.arcsin(np.clip(ps[:, 2] / np.maximum(r, 1e-6), -1, 1)))
        row = np.round((el - VLP16_EL[0]) / 2.0).astype(int)
        ok = (r > 1.0) & (row >= 0) & (row < 16) & (np.abs(el - VLP16_EL[np.clip(row, 0, 15)]) < 0.5)
        col = (np.floor((np.degrees(np.arctan2(ps[:, 1], ps[:, 0])) + 180.0) / args.az_res).astype(int)) % img.shape[1]
        rs = np.where(ok, img[np.clip(row, 0, 15), col], np.inf)
        valid = ok & np.isfinite(rs)
        margin = 0.15 + 0.02 * r
        see[idx[valid & (rs > r + margin)]] += 1
        hit[idx[valid & (np.abs(rs - r) <= margin)]] += 1
        if (k + 1) % 50 == 0:
            print(f'  {k + 1}/{len(kfs)}')

    dyn_c = (see >= args.min_see) & (see >= args.ratio * (see + hit))
    dyn_v = np.zeros(len(V), bool)
    dyn_v[np.nonzero(cand)[0][dyn_c]] = True
    dyn = dyn_v[vinv]
    print(f'동적 점 {dyn.sum()}개 ({100 * dyn.mean():.2f}%), 정적 {(~dyn).sum()}개')

    write_pcd(os.path.join(d, 'map_dynamic.pcd'), P[dyn], I[dyn])
    if args.replace:
        # 원본은 한 번만 보관 (다시 돌려도 map_raw.pcd 는 처음 PGO 지도 그대로)
        if not os.path.exists(os.path.join(d, 'map_raw.pcd')):
            shutil.copy(os.path.join(d, 'map.pcd'), os.path.join(d, 'map_raw.pcd'))
        write_pcd(os.path.join(d, 'map.pcd'), P[~dyn], I[~dyn])
        print('저장: map.pcd (정적 지도로 교체, 원본 map_raw.pcd), map_dynamic.pcd')
    else:
        write_pcd(os.path.join(d, 'map_static.pcd'), P[~dyn], I[~dyn])
        print('저장:', os.path.join(d, 'map_static.pcd'), os.path.join(d, 'map_dynamic.pcd'))


if __name__ == '__main__':
    main()
