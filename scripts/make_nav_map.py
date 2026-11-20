#!/usr/bin/env python3
"""把 SLAM 建出的原始地图加工成 Nav2 可用的导航地图。

输入  maps/robocon.yaml + .pgm       （SLAM Toolbox 保存的原始地图，不会被修改）
输出  maps/robocon_nav.yaml + .pgm   （导航用地图）+ robocon_nav_preview.png

为什么要加工（都是实测发现的问题）：
  1. 碎点：个别 1~3 格的孤立黑点会被 Nav2 当成障碍。
  2. 边界墙只有 0.08 m 高，雷达看不到，地图上没有外边界。
  3. 中央高台内部在地图里是"未知"，Nav2 默认允许穿过未知区域，规划器会直接穿过平台。
  4. 斜坡低处的表面低于雷达扫描面，雷达看不见，Nav2 会以为可以走上去。
     本阶段小车不上坡，所以把平台、台阶和斜坡一并标成不可通行。
  5. 地图只覆盖小车走过的区域，场地内没扫到的地方是"未知"，在场地内一律当作可通行。

用法:
    python3 make_nav_map.py                 # 默认读写 maps/ 目录
    python3 make_nav_map.py --maps-dir DIR
"""
import argparse
import math
import os
import re
from collections import deque

import numpy as np
from PIL import Image

OCC, UNK, FREE = 0, 205, 254

# ---- 场地几何（世界坐标，米），来自 robocon_ground/model.sdf ----
WALL_INNER = 5.5            # 边界墙内表面：|x|、|y| 超过它就出了场地（墙中心 5.55，厚 0.1）
KEEP_OUT = [                # 不可通行的矩形 (x_min, x_max, y_min, y_max)
    (-3.0, 3.0, -3.0, 3.0),         # 中央高台，6 x 6 m，高 0.6 m
    (-2.7, 2.75, 3.0, 4.0),         # 上侧台阶 + 斜坡（y = 3.5，宽 1.0 m）
    (-2.7, 2.75, -4.0, -3.0),       # 下侧台阶 + 斜坡
]
MIN_BLOB = 4                # 小于这个格数的孤立黑点视为噪声


def read_map(yaml_path):
    text = open(yaml_path, encoding='utf-8').read()
    img = os.path.join(os.path.dirname(yaml_path), re.search(r'image:\s*(\S+)', text).group(1))
    res = float(re.search(r'resolution:\s*([\d.eE+-]+)', text).group(1))
    ox, oy = [float(v) for v in re.search(r'origin:\s*\[([^\]]+)\]', text).group(1).split(',')[:2]]
    return np.array(Image.open(img)), res, ox, oy


def remove_small_blobs(occ, min_size):
    """删掉连通（8 邻域）格数 < min_size 的占用块，返回被删的格数。"""
    h, w = occ.shape
    seen = np.zeros_like(occ, bool)
    removed = 0
    for y0 in range(h):
        for x0 in range(w):
            if not occ[y0, x0] or seen[y0, x0]:
                continue
            blob, q = [], deque([(y0, x0)])
            seen[y0, x0] = True
            while q:
                y, x = q.popleft()
                blob.append((y, x))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < h and 0 <= nx < w and occ[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            q.append((ny, nx))
            if len(blob) < min_size:
                for y, x in blob:
                    occ[y, x] = False
                removed += len(blob)
    return removed


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument('--maps-dir', default=os.path.join(here, '..', 'maps'))
    args = ap.parse_args()
    maps = os.path.abspath(args.maps_dir)

    im, res, ox, oy = read_map(os.path.join(maps, 'robocon.yaml'))
    h, w = im.shape
    print(f'原始地图: {w} x {h} 格 ({w*res:.2f} x {h*res:.2f} m)，原点 ({ox}, {oy})')

    # 1. 去碎点（被删的格子改成"可通行"）
    occ = im == OCC
    removed = remove_small_blobs(occ, MIN_BLOB)
    im = im.copy()
    im[(im == OCC) & ~occ] = FREE
    print(f'删除碎点: {removed} 格')

    # 2. 扩大画布，使它完整覆盖场地外圈（保持格子和原图对齐，不重采样）
    need = WALL_INNER + 0.3
    left = max(0, math.ceil((ox + need) / res))
    bottom = max(0, math.ceil((oy + need) / res))
    right = max(0, math.ceil((need - (ox + w * res)) / res))
    top = max(0, math.ceil((need - (oy + h * res)) / res))
    big = np.full((h + top + bottom, w + left + right), UNK, np.uint8)
    big[top:top + h, left:left + w] = im          # 图像第 0 行 = 最大的 y，所以"上侧"扩展放在前面
    nox, noy = ox - left * res, oy - bottom * res
    H, W = big.shape

    # 每个格子中心的世界坐标
    xs = nox + (np.arange(W) + 0.5) * res
    ys = noy + (H - 1 - np.arange(H) + 0.5) * res
    X, Y = np.meshgrid(xs, ys)

    # 3. 场地内的未知 -> 可通行；场地外 -> 不可通行
    inside = (np.abs(X) < WALL_INNER) & (np.abs(Y) < WALL_INNER)
    filled = int(((big == UNK) & inside).sum())
    big[(big == UNK) & inside] = FREE
    big[~inside] = OCC                              # 边界墙 + 墙外
    print(f'场地内未知 -> 可通行: {filled} 格；墙外全部标为占用')

    # 4. 平台、台阶、斜坡 -> 不可通行
    for x0, x1, y0, y1 in KEEP_OUT:
        big[(X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)] = OCC

    # 5. 保存
    pgm = os.path.join(maps, 'robocon_nav.pgm')
    Image.fromarray(big).save(pgm)
    with open(os.path.join(maps, 'robocon_nav.yaml'), 'w', encoding='utf-8') as f:
        f.write('image: robocon_nav.pgm\nmode: trinary\n'
                f'resolution: {res}\norigin: [{nox:.2f}, {noy:.2f}, 0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    Image.fromarray(big).resize((W * 3, H * 3), Image.NEAREST).save(
        os.path.join(maps, 'robocon_nav_preview.png'))
    free = int((big == FREE).sum())
    print(f'导航地图: {W} x {H} 格，原点 ({nox:.2f}, {noy:.2f})，可通行 {free} 格 ({free*res*res:.1f} m²)')
    print('已写入 robocon_nav.yaml / .pgm / _preview.png')


if __name__ == '__main__':
    main()
