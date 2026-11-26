"""2.5D elevation 저장 공통 코드: 점에서 바로 계산(바닥 얼룩 없음) + npz/pgm/컬러 png/yaml 쓰기

셀 높이 정하는 법 (robust_elevation)
  - 셀 점 중 max(3개, 10 %) 이상이 국소 바닥 + 6 cm 보다 높으면 장애물 셀 -> 가장 높은 점
  - 아니면 바닥 셀 -> 바닥 점(국소 바닥 + 6 cm 이하)들의 중앙값
  셀 최고점만 쓰면 평평한 바닥에서도 노이즈/링 무늬 점 몇 개(4~8 cm)가 셀을 띄워서 얼룩덜룩해짐.
"""
import os
import struct
import zlib

import numpy as np
from scipy.ndimage import minimum_filter
from scipy.spatial import cKDTree

# Google Turbo colormap 33점 (Apache-2.0), 선형 보간
_TURBO = np.array([
    [48, 18, 59], [57, 42, 115], [64, 64, 162], [68, 86, 199], [70, 107, 227], [70, 128, 246], [66, 148, 255],
    [55, 168, 250], [40, 188, 235], [28, 205, 216], [24, 221, 194], [31, 233, 175], [50, 242, 152], [78, 249, 125],
    [109, 254, 98], [139, 255, 75], [164, 252, 60], [185, 246, 53], [205, 236, 52], [223, 223, 55], [238, 207, 58],
    [248, 190, 57], [253, 172, 52], [254, 150, 43], [251, 126, 33], [244, 102, 23], [235, 80, 14], [223, 63, 8],
    [208, 47, 5], [190, 33, 2], [169, 22, 1], [146, 11, 1], [122, 4, 3]], dtype=np.float64)


def turbo(x):
    """x 0..1 -> (N, 3) uint8 (파랑 -> 빨강)"""
    x = np.clip(np.asarray(x, dtype=np.float64), 0.0, 1.0) * (len(_TURBO) - 1)
    i = np.minimum(x.astype(int), len(_TURBO) - 2)
    t = (x - i)[..., None]
    return np.round(_TURBO[i] * (1 - t) + _TURBO[i + 1] * t).astype(np.uint8)


def write_png(path, rgba):
    """(H, W, 4) uint8 -> PNG (row 0 = 위)"""
    rgba = np.ascontiguousarray(rgba, dtype=np.uint8)
    h, w, _ = rgba.shape
    raw = b''.join(b'\x00' + rgba[i].tobytes() for i in range(h))

    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 6, 0, 0, 0)))
        f.write(chunk(b'IDAT', zlib.compress(raw, 6)))
        f.write(chunk(b'IEND', b''))


def write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(np.ascontiguousarray(img, dtype=np.uint8).tobytes())


def local_floor(P, kf_xyz, res=0.2):
    """점마다 바닥 높이: 키프레임 높이 근처 점의 셀(0.2 m)별 10 % 분위수, 3x3 최소값"""
    _, nk = cKDTree(kf_xyz[:, :2]).query(P[:, :2], workers=-1)
    near = np.abs(P[:, 2] - kf_xyz[nk, 2]) < 0.3
    ij = np.floor(P[:, :2] / res).astype(np.int64)
    ij -= ij.min(0)
    shp = tuple(ij.max(0) + 1)
    grid = np.full(shp, np.inf)
    idx = np.nonzero(near)[0]
    idx = idx[np.lexsort((P[idx, 2], ij[idx, 1], ij[idx, 0]))]
    key = ij[idx, 0] * shp[1] + ij[idx, 1]
    uk, st, cnt = np.unique(key, return_index=True, return_counts=True)
    grid.flat[uk] = P[idx[st + (cnt * 0.1).astype(int)], 2]
    grid = minimum_filter(grid, size=3)
    f = grid[ij[:, 0], ij[:, 1]]
    return np.where(np.isfinite(f), f, kf_xyz[nk, 2])


def robust_elevation(P, floor, res=0.1, length_x=40.0, length_y=40.0, max_h=1.2, min_h=-1.5, obj=0.06,
                     min_points=2):
    """-> (layers{elevation,min,count} 이미지 배열 (row 0 = +y 끝), x0, y0). 맵이 더 크면 격자를 키움 (원점 중심)"""
    rel = P[:, 2] - floor
    keep = (rel > min_h) & (rel < max_h)
    P, rel = P[keep], rel[keep]
    if len(P):
        length_x = max(length_x, 2 * np.abs(P[:, 0]).max() + 2 * res)
        length_y = max(length_y, 2 * np.abs(P[:, 1]).max() + 2 * res)
    nx, ny = int(np.ceil(length_x / res)), int(np.ceil(length_y / res))
    x0, y0 = -nx * res / 2, -ny * res / 2
    col = np.floor((P[:, 0] - x0) / res).astype(np.int64)
    row = np.floor((y0 + ny * res - P[:, 1]) / res).astype(np.int64)
    ok = (col >= 0) & (col < nx) & (row >= 0) & (row < ny)
    cell, z, r = row[ok] * nx + col[ok], P[ok, 2].astype(np.float64), rel[ok]
    N = nx * ny
    cnt = np.bincount(cell, minlength=N)
    zmax = np.full(N, np.nan)
    np.fmax.at(zmax, cell, z)
    zmin = np.full(N, np.nan)
    np.fmin.at(zmin, cell, z)
    up = np.bincount(cell, weights=(r > obj).astype(float), minlength=N)
    is_obj = up >= np.maximum(3, 0.1 * cnt)
    lo = r <= obj
    o = np.lexsort((z[lo], cell[lo]))
    cs, zs = cell[lo][o], z[lo][o]
    uc, st, ct = np.unique(cs, return_index=True, return_counts=True)
    med = np.full(N, np.nan)
    med[uc] = zs[st + ct // 2]
    elev = np.where(is_obj | np.isnan(med), zmax, med)
    sparse = cnt < min_points
    elev[sparse], zmin[sparse] = np.nan, np.nan
    shape = (ny, nx)
    return dict(elevation=elev.reshape(shape).astype(np.float32), min=zmin.reshape(shape).astype(np.float32),
                count=np.where(cnt > 0, cnt, np.nan).reshape(shape).astype(np.float32)), x0, y0


def elevation_params(share):
    import yaml
    with open(os.path.join(share, 'config', 'elevation.yaml')) as f:
        ep = yaml.safe_load(f) or {}
    return ep.get('elevation_mapper', ep).get('ros__parameters', ep)


def elevation_from_points(P, kf_xyz, ep, out, frame='map'):
    """정적 점 + 키프레임 위치 -> elevation 파일들. -> (ok, msg)"""
    res = float(ep.get('resolution', 0.1))
    layers, x0, y0 = robust_elevation(
        P, local_floor(P, kf_xyz), res, float(ep.get('length_x', 40.0)), float(ep.get('length_y', 40.0)),
        float(ep.get('max_height_above_robot', 1.2)), float(ep.get('min_height_above_robot', -1.5)),
        min_points=int(ep.get('min_points', 2)))
    return save_elevation_layers(layers, res, x0, y0, frame, out)


def save_elevation_layers(img_layers, res, x0, y0, frame, out):
    """이미지 배열(row 0 = +y 끝, col 0 = x0) -> elevation.npz/pgm/png/yaml. -> (ok, msg)"""
    np.savez_compressed(os.path.join(out, 'elevation.npz'), resolution=res, origin=np.array([x0, y0]), frame=frame,
                        note='배열[row, col]: row 0 = 위쪽(+y 끝), col 0 = x0. 셀 중심 = '
                             '(x0 + (col+0.5)*res, y0 + (rows-1-row+0.5)*res)', **img_layers)
    e = img_layers['elevation']
    valid = ~np.isnan(e)
    if not valid.any():
        return False, 'elevation 비어 있음'
    zmin, zmax = float(np.nanmin(e)), float(np.nanmax(e))
    span = max(zmax - zmin, 1e-6)
    img = np.zeros(e.shape, np.uint8)
    img[valid] = (1 + (e[valid] - zmin) / span * 254).astype(np.uint8)
    write_pgm(os.path.join(out, 'elevation.pgm'), img)
    rgba = np.zeros(e.shape + (4,), np.uint8)
    rgba[valid, :3] = turbo((e[valid] - zmin) / span)
    rgba[valid, 3] = 255
    write_png(os.path.join(out, 'elevation.png'), rgba)
    with open(os.path.join(out, 'elevation.yaml'), 'w') as f:
        f.write(f'image: elevation.pgm   # 1..255 = z_min..z_max, 0 = 데이터 없음 (원본은 elevation.npz)\n'
                f'color_image: elevation.png   # 파랑 = z_min .. 빨강 = z_max, 투명 = 데이터 없음\n'
                f'resolution: {res}\norigin: [{x0}, {y0}, 0.0]\nz_min: {zmin:.3f}\nz_max: {zmax:.3f}\n'
                f'frame: {frame}\n')
    return True, f'유효 셀 {int(valid.sum())}, z {zmin:.2f} ~ {zmax:.2f} m (+ elevation.png)'


def despeckle_grid(img, res, origin, P, floor, max_cells=12, min_points=20, band=(0.1, 2.0)):
    """작은 장애물 덩어리 지우기 (영문 설명은 lidar_map_cleaner). remove small occupied blobs of a grid image (row 0 = top, 0 = occupied) that the 3D map does not support.

    A blob (8-connected occupied cells) of at most `max_cells` cells is set free when fewer than `min_points` map
    points between floor + band[0] and floor + band[1] fall into its cells (octomap specks from noise / leftovers).
    Real thin objects (table legs, poles) are seen from many keyframes and keep their points. -> number of cells freed
    """
    from scipy import ndimage
    h, w = img.shape
    rel = P[:, 2] - floor
    s = (rel > band[0]) & (rel < band[1])
    col = np.floor((P[s, 0] - origin[0]) / res).astype(np.int64)
    row = h - 1 - np.floor((P[s, 1] - origin[1]) / res).astype(np.int64)
    ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    cnt = np.zeros(img.shape, np.int64)
    np.add.at(cnt, (row[ok], col[ok]), 1)
    lab, n = ndimage.label(img == 0, np.ones((3, 3)))
    if n == 0:
        return 0
    idx = np.arange(1, n + 1)
    size = ndimage.sum(np.ones_like(cnt), lab, idx)
    pts = ndimage.sum(cnt, lab, idx)
    drop = np.zeros(n + 1, bool)
    drop[1:] = (size <= max_cells) & (pts < min_points)
    m = drop[lab]
    img[m] = 254
    return int(m.sum())
