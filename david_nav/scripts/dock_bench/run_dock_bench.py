#!/usr/bin/env python3
"""run_dock_bench —— 精准入位(alias 0)的 N 轮自动测评编排器。

一次调用 = N 轮。每轮：
    ① 自动导航回起点（进到 return_tol 就 cancel，不等 nav2 末端精确收敛）
    ② 静置 settle_sec 让定位稳
    ③ 发 /dock_goal
    ④ 被动订阅 /rosout 等 dock_navigator 的终局判词（或超时）
    ⑤ 落一行 runs_dock.csv

本脚本**不启动任何被测组件**：alias 3(lightning，且已在 rviz 给过初始位姿)和
alias 0(nav2 + dock_navigator + 底盘)必须已经在跑。轨迹由 controller_bench 的
record_track.py 单独录（run_dock_bench.sh 负责起停），本脚本只管编排与判定。

★ 绝对不要在这里用 rclpy.spin_until_future_complete(self, fut)：
  本节点归 MultiThreadedExecutor 所有，那个函数会把它塞进全局 SingleThreadedExecutor，
  两个执行器对同一个 action client 各建一套 wait set → rcl_action wait set 下标越界
  → 进程 exit 1。dock_navigator 2026-09-08 就是这么崩的。统一用 wait_future()。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import threading
import time

import rclpy
import tf2_ros
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dock_common import (RUN_FIELDS, parse_dock_timeline,  # noqa: E402
                         prep_point, signed_cross_track, wrap, yaw_of_quat)


class DockBenchRunner(Node):
    def __init__(self, args, case):
        super().__init__('dock_bench_runner')
        self.args = args
        self.case = case
        self.cbg = ReentrantCallbackGroup()

        self.map_frame = args.map_frame
        self.base_frame = args.base_frame
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.goal_pub = self.create_publisher(PoseStamped, args.dock_goal_topic, 1)
        self.nav_cli = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                    callback_group=self.cbg)

        # /rosout 只留 dock_navigator。QoS 用 depth 1000 的 RELIABLE，
        # 与 rosout 发布端兼容（订阅端 VOLATILE 比发布端要求低，可匹配）。
        self._log_lock = threading.Lock()
        self._logs = []          # [(t_unix, msg)]
        self.create_subscription(
            Log, '/rosout', self._on_log,
            QoSProfile(depth=1000, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.cbg)

        self.get_logger().info(
            f'[bench] 就绪: goal_topic={args.dock_goal_topic} '
            f'watch_node={args.dock_node} frames={self.map_frame}->{self.base_frame}')

    # ------------------------------------------------------------ 基础设施
    def _on_log(self, msg: Log):
        if msg.name != self.args.dock_node:
            return
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        with self._log_lock:
            self._logs.append((t, msg.msg))

    def logs_since(self, t0):
        with self._log_lock:
            return [(t, m) for (t, m) in self._logs if t >= t0]

    @staticmethod
    def wait_future(fut, timeout_sec):
        """事件等待，自己绝不 spin（见模块 docstring 的血泪注释）。"""
        ev = threading.Event()
        fut.add_done_callback(lambda _f: ev.set())
        if fut.done():
            return True
        return ev.wait(timeout_sec)

    def pose_now(self, timeout_sec=1.0):
        try:
            tr = self.tf_buf.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'[bench] 取位姿失败: {e}')
            return None
        t = tr.transform.translation
        return (t.x, t.y, yaw_of_quat(tr.transform.rotation.x, tr.transform.rotation.y,
                                      tr.transform.rotation.z, tr.transform.rotation.w))

    def make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = self.map_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    # ------------------------------------------------------------ 回起点
    def return_to_start(self, sx, sy, syaw, tol, hard_tol, timeout):
        """导航回起点。一旦进入 tol 半径立刻 cancel —— alias 0 用的是精确参数
        (xy_goal_tolerance=0.10 / yaw 5°)，让它跑完末端收敛既慢又可能失败，
        而用户明确"起点无强约束"。返回 (ok, pose, err)。"""
        p = self.pose_now()
        if p is not None and math.hypot(p[0] - sx, p[1] - sy) <= tol:
            self.get_logger().info('[bench] 已在起点容差内，跳过回程')
            return True, p, math.hypot(p[0] - sx, p[1] - sy)

        if not self.nav_cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('[bench] navigate_to_pose 不可用 —— alias 0 起了吗？')
            return False, p, float('nan')

        g = NavigateToPose.Goal()
        g.pose = self.make_pose(sx, sy, syaw)
        send = self.nav_cli.send_goal_async(g)
        if not self.wait_future(send, 10.0):
            self.get_logger().error('[bench] 回程目标 10s 无应答')
            return False, self.pose_now(), float('nan')
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('[bench] 回程目标被拒绝')
            return False, self.pose_now(), float('nan')

        res_fut = handle.get_result_async()
        deadline = time.monotonic() + timeout
        cancelled = False
        while time.monotonic() < deadline:
            if res_fut.done():
                break
            p = self.pose_now(timeout_sec=0.2)
            if p is not None and math.hypot(p[0] - sx, p[1] - sy) <= tol:
                self.get_logger().info('[bench] 进入起点容差，cancel 回程')
                self.wait_future(handle.cancel_goal_async(), 5.0)
                cancelled = True
                break
            time.sleep(0.2)
        if not cancelled and not res_fut.done():
            self.get_logger().warn('[bench] 回程超时，cancel')
            self.wait_future(handle.cancel_goal_async(), 5.0)
        # cancel 之后底盘还要减速，给它一点时间落定
        time.sleep(1.0)

        p = self.pose_now()
        if p is None:
            return False, None, float('nan')
        err = math.hypot(p[0] - sx, p[1] - sy)
        if err > hard_tol:
            self.get_logger().warn(
                f'[bench] 回程后距起点 {err:.3f}m > {hard_tol:.2f}m，本轮标 INVALID')
            return False, p, err
        return True, p, err

    # ------------------------------------------------------------ 一轮入位
    def run_once(self, it, goal, tol, timing, session):
        gx, gy, gyaw = goal['x'], goal['y'], math.radians(goal['yaw_deg'])
        px, py = prep_point(gx, gy, gyaw, self.case.get('approach_dist', 0.5))

        row = {k: '' for k in RUN_FIELDS}
        row.update(session=session, iteration=it, goal_id=goal['id'],
                   timestamp=time.strftime('%Y-%m-%d %H:%M:%S'),
                   goal_x=f'{gx:.4f}', goal_y=f'{gy:.4f}', goal_yaw=f'{gyaw:.5f}',
                   prep_x=f'{px:.4f}', prep_y=f'{py:.4f}',
                   battery_v=self.args.battery_v, note=self.args.note)

        # ① 回起点
        s = self.case['start']
        ok, sp, serr = self.return_to_start(
            s['x'], s['y'], math.radians(s['yaw_deg']),
            tol['return_tol'], tol['start_tol_hard'], timing['return_timeout_sec'])
        if sp is not None:
            row.update(start_x=f'{sp[0]:.4f}', start_y=f'{sp[1]:.4f}',
                       start_yaw=f'{sp[2]:.5f}')
        row['start_err'] = f'{serr:.4f}' if serr == serr else ''
        if not ok:
            row['verdict_class'] = 'INVALID'
            row['verdict_detail'] = '回程未到起点容差内'
            return row

        # ② 静置
        time.sleep(timing['settle_sec'])

        # ③ 发目标
        t_sent = time.time()
        self.goal_pub.publish(self.make_pose(gx, gy, gyaw))
        row['t_dock_sent'] = f'{t_sent:.3f}'
        self.get_logger().info(
            f'[bench] === 第 {it} 轮：发 {self.args.dock_goal_topic} '
            f'({gx:.3f}, {gy:.3f}) yaw={goal["yaw_deg"]:.1f}°')

        # ④ 等判词
        deadline = time.monotonic() + timing['dock_timeout_sec']
        tl = None
        while time.monotonic() < deadline:
            tl = parse_dock_timeline(self.logs_since(t_sent - 0.5))
            if tl['verdict_class'] is not None:
                break
            time.sleep(0.2)

        end = self.pose_now()          # 判词时刻的真值，越快取越准
        if tl is None or tl['verdict_class'] is None:
            row['verdict_class'] = 'TIMEOUT'
            row['verdict_detail'] = f'{timing["dock_timeout_sec"]:.0f}s 内无判词'
            self.get_logger().error(f'[bench] 第 {it} 轮 TIMEOUT')
        else:
            row.update(verdict_class=tl['verdict_class'],
                       verdict_detail=(tl['verdict_detail'] or '').replace('\n', ' ')[:200],
                       stage1_status=tl['stage1_status'] or '',
                       stage2_status=tl['stage2_status'] or '',
                       n_spin=tl['n_spin'])
            for k_row, k_tl in (('t_stage1_start', 'stage1_start'),
                                ('t_stage2_start', 'stage2_start'),
                                ('t_verdict', 'verdict_t')):
                row[k_row] = f'{tl[k_tl]:.3f}' if tl[k_tl] is not None else ''
            if tl['stage2_start'] and tl['verdict_t']:
                row['stage2_dur'] = f'{tl["verdict_t"] - tl["stage2_start"]:.2f}'
            if tl['verdict_t']:
                row['total_dur'] = f'{tl["verdict_t"] - t_sent:.2f}'
            if tl['node_exy'] is not None:
                row['node_exy'] = f'{tl["node_exy"]:.4f}'
                row['node_eyaw_deg'] = f'{tl["node_eyaw_deg"]:.2f}'

        # ⑤ bench 自己复算终点误差（不采信节点自评）
        if end is not None:
            row.update(end_x=f'{end[0]:.4f}', end_y=f'{end[1]:.4f}',
                       end_yaw=f'{end[2]:.5f}')
            row['exy'] = f'{math.hypot(end[0] - gx, end[1] - gy):.4f}'
            row['eyaw'] = f'{wrap(end[2] - gyaw):.5f}'
            row['cross_track_end'] = f'{signed_cross_track(end[0], end[1], gx, gy, gyaw):.4f}'
        return row


def append_row(path, row):
    exists = os.path.exists(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'a', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=RUN_FIELDS)
        if not exists:
            w.writeheader()
        w.writerow(row)


def dump_logs(path, logs):
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['t_unix', 'msg'])
        w.writerows(logs)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    p = argparse.ArgumentParser()
    p.add_argument('--case', default=os.path.join(here, 'dock_case.yaml'))
    p.add_argument('--session', default='')
    p.add_argument('--runs', type=int, default=0, help='覆盖 case 里的 runs')
    p.add_argument('--output-dir',
                   default=os.path.expanduser('~/david_nav_logs/dock_bench'))
    p.add_argument('--dock-goal-topic', default='/dock_goal')
    p.add_argument('--dock-node', default='dock_navigator')
    p.add_argument('--map-frame', default='map')
    p.add_argument('--base-frame', default='base_footprint')
    p.add_argument('--battery-v', default='')
    p.add_argument('--note', default='')
    args = p.parse_args()

    with open(args.case, encoding='utf-8') as f:
        case = yaml.safe_load(f)
    session = args.session or time.strftime('dock_%Y%m%d_%H%M%S')
    n_runs = args.runs or int(case.get('runs', 10))
    tol, timing = case['tolerances'], case['timing']

    rclpy.init()
    node = DockBenchRunner(args, case)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    spin_thread = threading.Thread(target=ex.spin, daemon=True)
    spin_thread.start()

    runs_csv = os.path.join(args.output_dir, 'runs_dock.csv')
    counts = {}
    try:
        time.sleep(2.0)   # 等 TF / 发现建立
        if node.pose_now() is None:
            node.get_logger().error(
                '[bench] 拿不到 map->base TF —— alias 3 起了吗？初始位姿给了吗？')
            return 2
        for goal in case['goals']:
            for it in range(1, n_runs + 1):
                row = node.run_once(it, goal, tol, timing, session)
                append_row(runs_csv, row)
                cls = row['verdict_class'] or 'UNKNOWN'
                counts[cls] = counts.get(cls, 0) + 1
                node.get_logger().info(
                    f'[bench] 第 {it}/{n_runs} 轮 -> {cls} '
                    f'exy={row["exy"] or "n/a"} eyaw={row["eyaw"] or "n/a"} '
                    f'spin={row["n_spin"]}')
    except KeyboardInterrupt:
        node.get_logger().warn('[bench] 被中断')
    finally:
        with node._log_lock:
            logs = list(node._logs)
        dump_logs(os.path.join(args.output_dir, f'rosout_{session}.csv'), logs)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print(f'\n=== session {session} 结束 ===')
    for k, v in sorted(counts.items()):
        print(f'  {k:<20} {v}')
    print(f'runs: {runs_csv}')
    print(f'汇总: python3 dock_metrics.py --session {session}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
