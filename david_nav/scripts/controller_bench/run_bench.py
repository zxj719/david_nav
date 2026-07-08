#!/usr/bin/env python3
"""run_bench — 用固定参考路径驱动"被测 controller",并落一行 run 汇总。

统一框架:同一条固定 map 折线(reference_path.yaml)喂给任意 controller,只换
adapter。现在实现 PositionControllerAdapter(逐段 MoveTo POSE);以后加 DwbAdapter
(FollowPath 吃整条 Path)就能横向对比同一参考下的跟踪/终点表现。planner 对所有
controller 都被绕过 —— 测的是 controller,不是 planner(用户明确)。

真值:每段起点与最终终点位姿都读 slam_toolbox 的 map->base TF。忽略定位漂移。

流程:
  1. 读 reference_path.yaml,发成 /bench/reference_path (latched) 供 recorder/RViz。
  2. adapter 逐段驱动(读实时 map 位姿 -> 机体相对目标 -> MoveTo)。
  3. 读最终 map 位姿 -> 终点误差(vs goal,含 yaw)。
  4. 追加一行到 runs.csv(含失败分类,供稳定性统计)。

record_track.py 需并行在跑(由 run_bench.sh 编排)。本脚本只管驱动 + 汇总。

用法:
  ./run_bench.py --reference reference_path.yaml --run-id S1S4_run03 \
                 --battery-v 25.9 --output-dir /tmp/david_nav_logs/bench
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from david_nav_msgs.action import MoveTo

# bench 自包含的纯函数(与 waypoint_sparsifier 数学一致)
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bench_common import (load_reference, relative_target, relative_yaw,  # noqa: E402
                          wrap_to_pi)


def _yaw_from_quat(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


# --------------------------------------------------------------------------
# Controller 适配器
# --------------------------------------------------------------------------

class ControllerAdapter:
    """被测 controller 的执行抽象。drive() 返回每段状态字符串列表。

    状态: SUCCEEDED / ABORTED / TIMEOUT / REJECTED / NO_POSE。
    """
    name = 'base'

    def drive(self, runner: 'BenchRunner', ref: dict) -> list[str]:
        raise NotImplementedError


class PositionControllerAdapter(ControllerAdapter):
    """逐段 MoveTo POSE(relative)。镜像 waypoint_follower._drive_segment。"""
    name = 'position_controller'

    def __init__(self, move_action: str, server_timeout: float):
        self.move_action = move_action
        self.server_timeout = server_timeout

    def drive(self, runner: 'BenchRunner', ref: dict) -> list[str]:
        mover = ActionClient(runner, MoveTo, self.move_action,
                             callback_group=runner.cb)
        param_cli = runner.create_client(
            SetParameters, self.move_action.rsplit('/', 1)[0] + '/set_parameters',
            callback_group=runner.cb)

        if not mover.wait_for_server(timeout_sec=self.server_timeout):
            runner.get_logger().error(
                f'move server {self.move_action} unavailable')
            return ['REJECTED'] * len(ref['waypoints'])

        tol = ref['tolerances']
        wps = ref['waypoints']
        n = len(wps)
        results = []
        for i, wp in enumerate(wps):
            is_goal = (i == n - 1)
            dist_tol = tol['goal_dist_tol'] if is_goal else tol['wp_dist_tol']
            yaw_tol = tol['goal_yaw_tol'] if is_goal else tol['wp_yaw_tol']
            self._set_tols(runner, param_cli, dist_tol, yaw_tol)

            pose = runner.read_map_pose()
            if pose is None:
                runner.get_logger().error(f'seg {i}: no map pose (TF)')
                results.append('NO_POSE')
                break
            cx, cy, cyaw = pose
            rx, ry = relative_target(cx, cy, cyaw, float(wp['x']), float(wp['y']))
            ryaw = relative_yaw(cyaw, float(wp['yaw']))
            runner.get_logger().info(
                f'seg {i}/{n-1} [{wp.get("scenario","?")}] '
                f'-> rel({rx:.2f},{ry:.2f}) dyaw={math.degrees(ryaw):.0f}deg '
                f'tol(d={dist_tol},yaw={yaw_tol})')

            status = self._send_move(runner, mover, rx, ry, ryaw,
                                     ref['per_segment_timeout_sec'])
            results.append(status)
            if status != 'SUCCEEDED':
                runner.get_logger().error(f'seg {i} -> {status}, aborting run')
                break
        return results

    def _set_tols(self, runner, cli, dist_tol, yaw_tol):
        if not cli.wait_for_service(timeout_sec=2.0):
            runner.get_logger().warn('set_parameters unavailable, tols unchanged')
            return

        def _dp(name, val):
            return Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(val)))

        req = SetParameters.Request()
        req.parameters = [_dp('pose_dist_tol', dist_tol),
                          _dp('pose_yaw_tol', yaw_tol)]
        done = threading.Event()
        box = {}
        cli.call_async(req).add_done_callback(
            lambda f: (box.__setitem__('r', f.result()), done.set()))
        if not done.wait(timeout=2.0):
            runner.get_logger().warn('set_parameters timeout, tols unchanged')

    def _send_move(self, runner, mover, rx, ry, ryaw, timeout) -> str:
        goal = MoveTo.Goal()
        goal.mode = MoveTo.Goal.MODE_POSE
        goal.target.x = float(rx)
        goal.target.y = float(ry)
        goal.yaw = float(ryaw)
        goal.relative = True

        done = threading.Event()
        box = {}

        def _result_cb(fut):
            box['result'] = fut.result()
            done.set()

        def _goal_cb(fut):
            gh = fut.result()
            if not gh.accepted:
                box['rejected'] = True
                done.set()
                return
            box['gh'] = gh
            gh.get_result_async().add_done_callback(_result_cb)

        mover.send_goal_async(goal).add_done_callback(_goal_cb)
        if not done.wait(timeout=timeout):
            gh = box.get('gh')
            if gh is not None:
                gh.cancel_goal_async()
            return 'TIMEOUT'
        if box.get('rejected'):
            return 'REJECTED'
        wrapped = box.get('result')
        if wrapped is None or wrapped.result is None:
            return 'ABORTED'
        return 'SUCCEEDED' if bool(wrapped.result.success) else 'ABORTED'


# --------------------------------------------------------------------------
# Runner 节点
# --------------------------------------------------------------------------

class BenchRunner(Node):
    def __init__(self, args, ref):
        super().__init__('bench_run')
        self.args = args
        self.ref = ref
        self.cb = ReentrantCallbackGroup()

        # 真值主源:david_slam_gt 的 /groundtruth/pose(map 系 base_footprint)。
        # 与 record_track 一致;拿不到退回 map->base TF。
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._gt = None
        self._gt_mono = 0.0
        self._gt_stale_sec = 0.5
        self.create_subscription(PoseWithCovarianceStamped, args.truth_topic,
                                 self._on_truth, 10)

        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.path_pub = self.create_publisher(Path, '/bench/reference_path', latched)

    def _on_truth(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        self._gt = (p.position.x, p.position.y,
                    _yaw_from_quat(p.orientation.x, p.orientation.y,
                                   p.orientation.z, p.orientation.w))
        self._gt_mono = time.monotonic()

    def publish_reference(self):
        path = Path()
        path.header.frame_id = self.ref['frame']
        path.header.stamp = self.get_clock().now().to_msg()
        for pt in [self.ref['start']] + list(self.ref['waypoints']):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(pt['x'])
            ps.pose.position.y = float(pt['y'])
            qx, qy, qz, qw = _quat_from_yaw(float(pt['yaw']))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            path.poses.append(ps)
        self.path_pub.publish(path)
        self.get_logger().info(
            f'published reference ({len(path.poses)} poses) on /bench/reference_path')

    def read_map_pose(self, retries: int = 25, wait: float = 0.1):
        """真值位姿 (x,y,yaw)。优先新鲜的 /groundtruth/pose,否则查 map->base TF。
        等真值出现(定位/gt_publisher 刚起可能慢)。"""
        for _ in range(retries):
            gt = self._gt
            if gt is not None and (time.monotonic() - self._gt_mono) < self._gt_stale_sec:
                return gt
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.args.map_frame, self.args.base_frame, rclpy.time.Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                return (t.x, t.y, _yaw_from_quat(q.x, q.y, q.z, q.w))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                time.sleep(wait)
        return None


def _overall_status(seg_status: list[str]) -> str:
    if seg_status and all(s == 'SUCCEEDED' for s in seg_status):
        return 'SUCCEEDED'
    for s in seg_status:
        if s != 'SUCCEEDED':
            return s  # 第一个失败类型
    return 'EMPTY'


def _append_summary(path, row, fieldnames):
    new = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        if new:
            w.writeheader()
        w.writerow(row)


def main():
    p = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    p.add_argument('--reference', default=os.path.join(here, 'reference_path.yaml'))
    p.add_argument('--run-id', default='')
    p.add_argument('--controller', default='position_controller',
                   choices=['position_controller'])
    p.add_argument('--move-action', default='/position_controller/move')
    p.add_argument('--server-timeout-sec', type=float, default=10.0)
    p.add_argument('--battery-v', type=float, default=float('nan'))
    p.add_argument('--note', default='')
    p.add_argument('--output-dir', default='/tmp/david_nav_logs/bench')
    p.add_argument('--map-frame', default='map')
    p.add_argument('--base-frame', default='base_footprint')
    p.add_argument('--truth-topic', default='/groundtruth/pose',
                   help='david_slam_gt 真值话题(主源;拿不到退回 map->base TF)')
    args = p.parse_args()

    ref = load_reference(args.reference)
    os.makedirs(args.output_dir, exist_ok=True)
    run_id = args.run_id or time.strftime('%Y%m%d_%H%M%S')

    rclpy.init()
    node = BenchRunner(args, ref)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    if args.controller == 'position_controller':
        adapter = PositionControllerAdapter(args.move_action, args.server_timeout_sec)
    else:  # pragma: no cover - 未来 controller
        raise SystemExit(f'unknown controller {args.controller}')

    try:
        node.publish_reference()
        time.sleep(0.5)  # 让 recorder / RViz 收到 latched path

        start_pose = node.read_map_pose()
        node.get_logger().info(f'start map pose: {start_pose}')

        t0 = time.time()
        seg_status = adapter.drive(node, ref)
        duration = time.time() - t0

        end_pose = node.read_map_pose()
        goal = ref['goal']
        gx, gy, gyaw = float(goal['x']), float(goal['y']), float(goal['yaw'])
        if end_pose is not None:
            ex, ey, eyaw = end_pose
            err_x, err_y = ex - gx, ey - gy
            err_pos = math.hypot(err_x, err_y)
            err_yaw = wrap_to_pi(eyaw - gyaw)
        else:
            ex = ey = eyaw = err_x = err_y = err_pos = err_yaw = float('nan')

        overall = _overall_status(seg_status)
        row = {
            'run_id': run_id, 'controller': adapter.name,
            'battery_v': args.battery_v, 'note': args.note,
            'overall_status': overall, 'duration_s': round(duration, 2),
            'n_seg': len(seg_status),
            'n_seg_ok': sum(1 for s in seg_status if s == 'SUCCEEDED'),
            'goal_x': gx, 'goal_y': gy, 'goal_yaw': gyaw,
            'end_x': ex, 'end_y': ey, 'end_yaw': eyaw,
            'err_x': err_x, 'err_y': err_y, 'err_pos': err_pos, 'err_yaw': err_yaw,
            'seg_status': '|'.join(seg_status),
        }
        summary = os.path.join(args.output_dir, 'runs.csv')
        _append_summary(summary, row, list(row.keys()))

        node.get_logger().info(
            f'RUN {run_id} {overall} | end_err pos={err_pos:.3f}m '
            f'yaw={math.degrees(err_yaw):.1f}deg | {duration:.1f}s')
        node.get_logger().info(f'summary appended: {summary}')
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
