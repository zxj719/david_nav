#!/usr/bin/env python3
"""validate_dock_case —— 首跑前校核 dock_case.yaml：占位坐标拒跑 + 目标/入位直线不在障碍里。

查两张图：
  /map                      —— 结构性占据（墙在哪）
  /global_costmap/costmap   —— ★决定性的一张：带膨胀的代价。nav2 的 Costmap2DPublisher
                               把 254(LETHAL)→100、253(INSCRIBED)→99、255(UNKNOWN)→-1，
                               而 **NavFn 里 cost>=253 就是硬障碍**，所以这里 >=99 即不可达。
                               inflation_radius=0.60 对内切半径 0.255m 的车，
                               窄走廊会被整条封死 —— 只看 /map 是看不出来的。

校核的三样东西：
  ① goal 本身
  ② 预备点 P = G - approach_dist × [cos,sin](yaw_G)
  ③ P→G 这条**入位直线**上的逐点采样（这段是 dock_navigator 沿航向直线走的，
     被膨胀带蹭到就会入位失败）
另外报出 start 点，回程到不了起点整轮都会作废。

用法（alias 3 + alias 0 在跑时，域 0）：
  python3 validate_dock_case.py [--case dock_case.yaml] [--radius 0.30]
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dock_common import prep_point  # noqa: E402

PLACEHOLDER_HINT = (
    '★ dock_case.yaml 里 start/goals 还是占位坐标 (全 0)。\n'
    '  填之前先在 rviz 的 Global Costmap 图层上选点，别点在膨胀带里。\n'
    '  goal 的 yaw 必须准 —— 它定义理想入位直线，是全部贴合度指标的基准。')


def is_placeholder(case):
    """全 0 视作没填。真要在原点入位，把 yaw 写成 0.0001 之类绕过即可。"""
    def zero(d, keys):
        return all(abs(float(d.get(k, 0.0))) < 1e-9 for k in keys)
    if zero(case['start'], ('x', 'y', 'yaw_deg')):
        return True
    return any(zero(g, ('x', 'y', 'yaw_deg')) for g in case['goals'])


class CaseValidator(Node):
    def __init__(self, args):
        super().__init__('dock_case_validator')
        self.args = args
        self.maps = {}
        self.lock = threading.Lock()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
        for topic in (args.map_topic, args.costmap_topic):
            self.create_subscription(
                OccupancyGrid, topic,
                lambda m, t=topic: self._on_map(t, m), qos)

    def _on_map(self, topic, msg):
        with self.lock:
            self.maps[topic] = msg

    def wait_maps(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self.lock:
                if self.args.map_topic in self.maps and \
                        self.args.costmap_topic in self.maps:
                    return
            time.sleep(0.2)

    @staticmethod
    def _cell(grid, wx, wy):
        info = grid.info
        mx = int((wx - info.origin.position.x) / info.resolution)
        my = int((wy - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return grid.data[my * info.width + mx]

    def probe(self, grid, wx, wy, radius):
        """点周围 radius 内的最坏栅格值。返回 (worst, n_unknown, n_out)。"""
        res = grid.info.resolution
        steps = max(1, int(radius / res))
        worst, unknown, out = -128, 0, 0
        for iy in range(-steps, steps + 1):
            for ix in range(-steps, steps + 1):
                if math.hypot(ix * res, iy * res) > radius:
                    continue
                v = self._cell(grid, wx + ix * res, wy + iy * res)
                if v is None:
                    out += 1
                elif v < 0:
                    unknown += 1
                else:
                    worst = max(worst, v)
        return worst, unknown, out


def verdict(worst, unknown, out, lethal_at):
    if out:
        return 'OUT_OF_MAP'
    if worst >= lethal_at:
        return 'BLOCKED'
    if unknown:
        return 'UNKNOWN'
    return 'OK'


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser()
    p.add_argument('--case', default=os.path.join(here, 'dock_case.yaml'))
    p.add_argument('--radius', type=float, default=0.30,
                   help='机身外接半径，默认 0.30（footprint 半宽 0.30）')
    p.add_argument('--map-topic', default='/map')
    p.add_argument('--costmap-topic', default='/global_costmap/costmap')
    p.add_argument('--timeout', type=float, default=15.0)
    p.add_argument('--samples', type=int, default=11, help='入位直线上的采样点数')
    args = p.parse_args()

    with open(args.case, encoding='utf-8') as f:
        case = yaml.safe_load(f)

    if is_placeholder(case):
        print(PLACEHOLDER_HINT)
        return 2

    rclpy.init()
    node = CaseValidator(args)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    node.get_logger().info(f'等 {args.map_topic} 和 {args.costmap_topic} …')
    node.wait_maps(args.timeout)

    with node.lock:
        gmap = node.maps.get(args.map_topic)
        cmap = node.maps.get(args.costmap_topic)

    rc = 0
    if gmap is None:
        print(f'!! 没收到 {args.map_topic} —— alias 3/0 起了吗？')
        rc = 3
    if cmap is None:
        print(f'!! 没收到 {args.costmap_topic} —— nav2 起了吗？'
              '（nav2 的 costmap 只在有订阅者时才发布，本脚本订阅了就该有）')
        print('   仅用 /map 校核，看不出膨胀带 —— 结果偏乐观。')

    ad = float(case.get('approach_dist', 0.5))
    pts = [('start', case['start']['x'], case['start']['y'])]
    for g in case['goals']:
        gx, gy = float(g['x']), float(g['y'])
        gyaw = math.radians(float(g['yaw_deg']))
        px, py = prep_point(gx, gy, gyaw, ad)
        pts.append((f'{g["id"]}:goal', gx, gy))
        pts.append((f'{g["id"]}:prep', px, py))
        for i in range(args.samples):
            f = i / (args.samples - 1)
            pts.append((f'{g["id"]}:line[{f:.2f}]',
                        px + (gx - px) * f, py + (gy - py) * f))

    print(f'\n{"点":<26} {"/map":<12} {"costmap":<12} 判定')
    print('-' * 68)
    for name, x, y in pts:
        v_map = v_cost = 'n/a'
        vd = 'OK'
        if gmap is not None:
            w, u, o = node.probe(gmap, x, y, args.radius)
            v_map = f'{w}'
            vd = verdict(w, u, o, 100)      # /map: 100 = 占用
        if cmap is not None:
            w, u, o = node.probe(cmap, x, y, args.radius)
            v_cost = f'{w}'
            vd_c = verdict(w, u, o, 99)     # costmap: >=99 = INSCRIBED/LETHAL
            # costmap 是决定性的那张，它说不行就是不行
            if vd_c != 'OK':
                vd = vd_c
        flag = '' if vd == 'OK' else '  <== '
        if vd in ('BLOCKED', 'OUT_OF_MAP'):
            rc = max(rc, 1)
        print(f'{name:<26} {v_map:<12} {v_cost:<12} {vd}{flag}')

    print()
    if rc == 0:
        print('全部 OK —— 可以跑 run_dock_bench.sh')
    elif rc == 1:
        print('!! 有 BLOCKED/OUT_OF_MAP 点。NavFn 里 cost>=253(即 costmap 图上 >=99) '
              '就是硬障碍，规划会直接失败或车贴墙。换个 goal，或缩小 --radius 重评。')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
