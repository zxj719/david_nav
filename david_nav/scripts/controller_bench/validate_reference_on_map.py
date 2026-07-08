#!/usr/bin/env python3
"""validate_reference_on_map — 用活着的 /map 校核固定参考折线是否穿墙。

reference_path.yaml 的中间 waypoint 是几何布置的,没对地图校过(存图时 .pgm 常没更新)。
position_controller 逐段【盲走直线】,所以每段必须在空闲区里。本工具:
  - 订阅 /map (nav_msgs/OccupancyGrid,localization 起来后 slam_toolbox 会latched发)
  - 载入 reference_path.yaml,把 start+waypoints 连成折线
  - 沿每段中心线按分辨率采样,检查机器人半径内有没有 占用 / 未知 栅格
  - 给每段 OK / UNKNOWN / BLOCKED 判定 + 最坏点坐标

用法(localization 在跑时,域0):
  ros2 run ...  # 不需要,直接:
  python3 validate_reference_on_map.py [--radius 0.35] [--reference reference_path.yaml]

判定:BLOCKED=有占用栅格(会撞,必须改点);UNKNOWN=路径过未建图区(盲走有风险,建议补图或改点)。
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_common import load_reference  # noqa: E402


class MapValidator(Node):
    def __init__(self, args):
        super().__init__('bench_map_validator')
        self.args = args
        self.ref = load_reference(args.reference)
        self.got = False
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, qos)
        self.get_logger().info(
            f'等 {args.map_topic} (localization 起了吗?) …校 '
            f'{len(self.ref["waypoints"])} 段,机器人半径 {args.radius}m')

    # world -> cell value; None if out of grid bounds
    def _cell(self, grid, wx, wy):
        info = grid.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return grid.data[my * info.width + mx]

    # 点周围 radius 内最坏栅格: 'blocked' / 'unknown' / 'free'
    def _probe(self, grid, wx, wy, radius):
        res = grid.info.resolution
        rc = int(math.ceil(radius / res))
        worst = 'free'
        for dy in range(-rc, rc + 1):
            for dx in range(-rc, rc + 1):
                if math.hypot(dx, dy) * res > radius:
                    continue
                v = self._cell(grid, wx + dx * res, wy + dy * res)
                if v is None or v < 0:               # 出界 or 未知
                    if worst == 'free':
                        worst = 'unknown'
                elif v >= self.args.occ_thresh:       # 占用
                    return 'blocked'
        return worst

    def _on_map(self, grid: OccupancyGrid):
        if self.got:
            return
        self.got = True
        info = grid.info
        self.get_logger().info(
            f'/map {info.width}x{info.height} res={info.resolution} '
            f'origin=({info.origin.position.x:.2f},{info.origin.position.y:.2f})')

        names = ['start'] + [w.get('scenario', f'wp{i}')
                             for i, w in enumerate(self.ref['waypoints'])]
        poly = self.ref['polyline']
        step = info.resolution
        overall_ok = True
        print('\n=== 参考折线 vs /map 校核 ===')
        for i in range(len(poly) - 1):
            (ax, ay), (bx, by) = poly[i], poly[i + 1]
            seg_len = math.hypot(bx - ax, by - ay)
            n = max(1, int(seg_len / step))
            verdict = 'free'
            worst_pt = None
            for k in range(n + 1):
                t = k / n
                px, py = ax + t * (bx - ax), ay + t * (by - ay)
                r = self._probe(grid, px, py, self.args.radius)
                if r == 'blocked':
                    verdict, worst_pt = 'blocked', (px, py)
                    break
                if r == 'unknown' and verdict == 'free':
                    verdict, worst_pt = 'unknown', (px, py)
            tag = {'free': 'OK      ', 'unknown': 'UNKNOWN ',
                   'blocked': 'BLOCKED '}[verdict]
            if verdict != 'free':
                overall_ok = False
            wp = f'({bx:.2f},{by:.2f})'
            extra = f'  最坏点=({worst_pt[0]:.2f},{worst_pt[1]:.2f})' if worst_pt else ''
            print(f'  [{tag}] seg{i} -> {names[i+1]:<20} {wp} len={seg_len:.2f}m{extra}')
        print('\n结论:', '✅ 全段可盲走' if overall_ok else
              '❌ 有 BLOCKED/UNKNOWN 段 —— 改 waypoint 或补图后再跑', '\n')
        rclpy.shutdown()


def main():
    p = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    p.add_argument('--reference', default=os.path.join(here, 'reference_path.yaml'))
    p.add_argument('--map-topic', default='/map')
    p.add_argument('--radius', type=float, default=0.35,
                   help='机器人内切半径(m),沿路径检查这个半径内有无障碍。默认 0.35')
    p.add_argument('--occ-thresh', type=int, default=50,
                   help='OccupancyGrid 值 >= 此判占用(0..100)。默认 50')
    args = p.parse_args()

    rclpy.init()
    node = MapValidator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
