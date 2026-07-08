#!/usr/bin/env python3
"""plot_run — 一次 run 的可视化:占据地图底 + 参考折线 + waypoints + 真值轨迹。

run_bench.sh 跑完自动调;也可手动补出图。产出 plot_<run_id>.png(headless,Agg)。

地图底优先抓活着的 /map(localization latched,当前图);抓不到退回磁盘 .pgm/.yaml
(可能是旧栅格,标题会注明)。轨迹是 track_<run_id>.csv 里 tf_ok=1 的真值点。

用法(localization 在跑时,域0):
  python3 plot_run.py --run-id S1S4_run01 [--output-dir /tmp/david_nav_logs/bench]
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_common import load_reference, wrap_to_pi  # noqa: E402


# --------------------------------------------------------------------------
# 地图底:活 /map 优先,退回磁盘 pgm
# --------------------------------------------------------------------------

def grab_live_map(timeout_sec=5.0):
    """抓一帧 latched /map -> (img[HxW] 0..100/-1, res, ox, oy) 或 None。"""
    try:
        import rclpy
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        from nav_msgs.msg import OccupancyGrid
    except Exception:
        return None

    box = {}
    rclpy.init()
    node = rclpy.create_node('bench_plot_mapgrab')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)

    def cb(msg):
        box['msg'] = msg
    node.create_subscription(OccupancyGrid, '/map', cb, qos)

    import time as _t
    t0 = _t.monotonic()
    while 'msg' not in box and (_t.monotonic() - t0) < timeout_sec:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()

    if 'msg' not in box:
        return None
    m = box['msg']
    img = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    return (img, m.info.resolution,
            m.info.origin.position.x, m.info.origin.position.y, 'live /map')


def load_disk_map(map_yaml):
    """退回:读磁盘 pgm/yaml -> (img 0..100/-1, res, ox, oy)。"""
    try:
        import yaml
        from PIL import Image
    except Exception:
        return None
    if not os.path.exists(map_yaml):
        return None
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    pgm = os.path.join(os.path.dirname(map_yaml), meta['image'])
    if not os.path.exists(pgm):
        return None
    arr = np.array(Image.open(pgm))          # 0(占用)..255(空闲),205 未知
    occ = np.full(arr.shape, -1, dtype=np.int16)
    p = arr.astype(np.float32) / 255.0       # map_server: occ=(255-px)/255
    occv = (1.0 - p)
    free_t, occ_t = meta.get('free_thresh', 0.25), meta.get('occupied_thresh', 0.65)
    occ[occv < free_t] = 0
    occ[occv > occ_t] = 100
    occ = np.flipud(occ)                      # pgm 顶行是最大 y;翻成 origin='lower'
    res = meta['resolution']
    ox, oy = meta['origin'][0], meta['origin'][1]
    tag = f'disk {os.path.basename(pgm)}' + (
        '  (stale raster?)' if True else '')
    return (occ, res, ox, oy, tag)


def occ_to_gray(img):
    """OccupancyGrid 值 -> 灰度 RGB(空闲白 / 占用黑 / 未知浅灰)。"""
    h, w = img.shape
    g = np.full((h, w), 0.85)     # 未知 -> 浅灰
    g[img == 0] = 1.0             # 空闲 -> 白
    g[img >= 50] = 0.0            # 占用 -> 黑
    g[(img > 0) & (img < 50)] = 0.6
    return np.dstack([g, g, g])


# --------------------------------------------------------------------------
# 主出图
# --------------------------------------------------------------------------

def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float('nan')


def plot(run_id, out_dir, ref, map_bg):
    track = os.path.join(out_dir, f'track_{run_id}.csv')
    ax_xy = []
    ax_yaw = []
    with open(track) as f:
        for r in csv.DictReader(f):
            if r.get('tf_ok') in ('1', '1.0'):
                x, y = _f(r['actual_x']), _f(r['actual_y'])
                if not (math.isnan(x) or math.isnan(y)):
                    ax_xy.append((x, y))
                    ax_yaw.append(_f(r['actual_yaw']))

    poly = ref['polyline']
    wps = ref['waypoints']
    start = ref['start']
    goal = ref['goal']

    fig, ax = plt.subplots(figsize=(9, 8))

    map_tag = 'no map'
    if map_bg is not None:
        img, res, ox, oy, map_tag = map_bg
        extent = [ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res]
        ax.imshow(occ_to_gray(img), extent=extent, origin='lower',
                  interpolation='nearest', zorder=0)

    # 参考折线 + waypoints
    px = [p[0] for p in poly]
    py = [p[1] for p in poly]
    ax.plot(px, py, '--', color='#1f6feb', lw=1.6, zorder=3,
            label='reference')
    for i, w in enumerate(wps):
        wx, wy, wyaw = float(w['x']), float(w['y']), float(w['yaw'])
        ax.plot(wx, wy, 'o', color='#1f6feb', ms=7, zorder=4)
        ax.annotate(f"{i+1}", (wx, wy), textcoords='offset points',
                    xytext=(6, 6), color='#1f6feb', fontsize=9, zorder=5)
        ax.arrow(wx, wy, 0.25 * math.cos(wyaw), 0.25 * math.sin(wyaw),
                 head_width=0.06, color='#1f6feb', alpha=0.6, zorder=4)

    # 真值轨迹
    if ax_xy:
        tx = [p[0] for p in ax_xy]
        ty = [p[1] for p in ax_xy]
        ax.plot(tx, ty, '-', color='#d1242f', lw=2.0, zorder=6,
                label='actual (slam_gt truth)')
        # 终点朝向箭头
        eyaw = ax_yaw[-1]
        ax.arrow(tx[-1], ty[-1], 0.3 * math.cos(eyaw), 0.3 * math.sin(eyaw),
                 head_width=0.08, color='#d1242f', zorder=7)

    # start / goal
    ax.plot(float(start['x']), float(start['y']), '^', color='#1a7f37',
            ms=13, zorder=8, label='start')
    ax.plot(float(goal['x']), float(goal['y']), '*', color='#8250df',
            ms=18, zorder=8, label='goal')

    # 终点误差标题
    subtitle = ''
    if ax_xy:
        ex, ey, ea = tx[-1], ty[-1], ax_yaw[-1]
        gx, gy, ga = float(goal['x']), float(goal['y']), float(goal['yaw'])
        ep = math.hypot(ex - gx, ey - gy)
        eyaw_err = math.degrees(wrap_to_pi(ea - ga))
        subtitle = f'end err {ep*100:.1f}cm / yaw {eyaw_err:+.1f}deg'

    ax.set_title(f'{run_id}   {subtitle}\nmap: {map_tag}', fontsize=11)
    ax.set_xlabel('x (m, map)')
    ax.set_ylabel('y (m, map)')
    ax.set_aspect('equal', 'box')
    ax.grid(True, ls=':', alpha=0.4)
    ax.legend(loc='best', fontsize=9)

    # 视野聚焦到路径附近(留 0.8m 边)
    allx = px + ([p[0] for p in ax_xy] if ax_xy else [])
    ally = py + ([p[1] for p in ax_xy] if ax_xy else [])
    m = 0.8
    ax.set_xlim(min(allx) - m, max(allx) + m)
    ax.set_ylim(min(ally) - m, max(ally) + m)

    out_png = os.path.join(out_dir, f'plot_{run_id}.png')
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    print(f'[plot_run] 出图: {out_png}  (map底={map_tag})')
    return out_png


def main():
    p = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    p.add_argument('--run-id', required=True)
    p.add_argument('--output-dir', default='/tmp/david_nav_logs/bench')
    p.add_argument('--reference', default=os.path.join(here, 'reference_path.yaml'))
    p.add_argument('--map-yaml',
                   default=os.path.expanduser(
                       '~/ros2_ws/src/david_slam_gt/maps/david_map.yaml'),
                   help='活 /map 抓不到时退回的磁盘地图')
    p.add_argument('--no-live-map', action='store_true',
                   help='跳过抓活 /map,直接用磁盘 pgm')
    args = p.parse_args()

    ref = load_reference(args.reference)
    map_bg = None
    if not args.no_live_map:
        map_bg = grab_live_map()
    if map_bg is None:
        map_bg = load_disk_map(args.map_yaml)
    if map_bg is None:
        print('[plot_run] 无地图底(活 /map + 磁盘都没),只画轨迹', file=sys.stderr)

    plot(args.run_id, args.output_dir, ref, map_bg)


if __name__ == '__main__':
    main()
