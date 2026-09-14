#!/usr/bin/env python3
"""record_track — controller_bench 被动轨迹记录器(controller 无关)。

不管谁在驱动(position_controller / DWB / 其他),都记录同样三样东西,供跟踪
与终点分析:

  ① 实际轨迹   —— slam_toolbox 的 map->base TF(★真值,忽略定位漂移),20Hz 落 CSV
  ② 参考路径   —— runner 发的 /bench/reference_path (Path),开跑时落一次 sidecar
  ③ planner    —— /plan (Path),最后一版落一次 sidecar(controller-only 模式下常空;
                   给以后测 DWB 用)

另外顺带记 cmd_vel 全链(controller 输出的原始命令)+ action 状态,方便看
controller 内部行为。

用法:
  ./record_track.py --run-id S1S4_run03 --output-dir /tmp/david_nav_logs/bench
  停止:Ctrl-C(runner 跑完会发 SIGTERM)。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time
from threading import Lock

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

import tf2_ros


def _yaw_from_quat(x, y, z, w) -> float:
    """atan2(2(wz+xy), 1-2(yy+zz)) —— 与 workspace 其它节点一致。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


STATUS_NAMES = {
    0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
}


class TrackRecorder(Node):
    def __init__(self, args):
        super().__init__('bench_record_track')
        self.args = args
        self.lock = Lock()

        self.map_frame = args.map_frame
        self.base_frame = args.base_frame

        # 真值主源:david_slam_gt 的 /groundtruth/pose(map 系 base_footprint,20Hz)。
        # 拿不到时退化到直接查 map->base TF(同一位姿,只是少了 gt_publisher 这层)。
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._gt = None          # (x, y, yaw)
        self._gt_mono = 0.0      # time.monotonic() at last gt msg
        self._gt_stale_sec = 0.5

        self.row = {
            't_unix': 0.0,
            'actual_x': float('nan'), 'actual_y': float('nan'),
            'actual_yaw': float('nan'), 'tf_ok': 0,
            # odom 位姿(轮速计积分,会漂)—— 与 actual(真值)对比 = 定位漂移 vs 控制误差
            'odom_x': float('nan'), 'odom_y': float('nan'), 'odom_yaw': float('nan'),
            'odom_v': float('nan'), 'odom_w': float('nan'),
            'cmd_vel_nav_v': float('nan'), 'cmd_vel_nav_w': float('nan'),
            'cmd_vel_smoothed_v': float('nan'), 'cmd_vel_smoothed_w': float('nan'),
            'cmd_vel_safe_v': float('nan'), 'cmd_vel_safe_w': float('nan'),
            'cmd_vel_raw_v': float('nan'), 'cmd_vel_raw_w': float('nan'),
            'cmd_vel_v': float('nan'), 'cmd_vel_w': float('nan'),
            'action_status': '', 'event': '',
        }
        self._last_status = None

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)

        for name in ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel_safe',
                     'cmd_vel_raw', 'cmd_vel'):
            self.create_subscription(Twist, '/' + name,
                                     self._mk_twist_cb(name), qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.create_subscription(PoseWithCovarianceStamped, args.truth_topic,
                                 self._on_truth, qos)
        self.create_subscription(GoalStatusArray,
                                 '/navigate_to_pose/_action/status',
                                 self._on_action_status, qos)
        # ---- dock_bench 用的两个可选订阅(不传参数时完全不生效)----
        # 追加一个 action status(dock 用 /spin/_action/status 数纯旋转补偿)
        self._extra_status = None
        if getattr(args, 'extra_action_status', ''):
            self.row['extra_action_status'] = ''
            self.create_subscription(GoalStatusArray, args.extra_action_status,
                                     self._on_extra_status, qos)
        # /rosout 只留指定节点(dock 用 dock_navigator，判词即状态机)。
        # rosout 发布端是 depth 1000，这里也开大，别在忙的时候丢判词。
        self._rosout = []
        if getattr(args, 'rosout_node', ''):
            self.create_subscription(
                Log, '/rosout', self._on_rosout,
                QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                           history=HistoryPolicy.KEEP_LAST))
        # 参考路径 + planner:latched 各存一份 sidecar
        self.create_subscription(Path, '/bench/reference_path',
                                 self._on_reference, latched)
        self.create_subscription(Path, '/plan', self._on_plan, qos)

        os.makedirs(args.output_dir, exist_ok=True)
        rid = args.run_id or time.strftime('%Y%m%d_%H%M%S')
        self.rid = rid
        self.csv_path = os.path.join(args.output_dir, f'track_{rid}.csv')
        self.ref_path = os.path.join(args.output_dir, f'track_{rid}_reference.csv')
        self.plan_path = os.path.join(args.output_dir, f'track_{rid}_plan.csv')
        self.rosout_path = os.path.join(args.output_dir, f'rosout_{rid}.csv')
        self._ref_written = False
        self._plan = None

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=list(self.row.keys()))
        self.writer.writeheader()
        self.csv_file.flush()

        self.timer = self.create_timer(1.0 / args.rate_hz, self._tick)
        self.get_logger().info(f'track CSV: {self.csv_path}')
        self.get_logger().info(
            f'truth = {args.truth_topic} (fallback TF {self.map_frame}->'
            f'{self.base_frame}) @ {args.rate_hz}Hz')

    def _mk_twist_cb(self, name):
        def cb(msg: Twist):
            with self.lock:
                self.row[f'{name}_v'] = float(msg.linear.x)
                self.row[f'{name}_w'] = float(msg.angular.z)
        return cb

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        with self.lock:
            self.row['odom_x'] = float(p.position.x)
            self.row['odom_y'] = float(p.position.y)
            self.row['odom_yaw'] = _yaw_from_quat(
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            self.row['odom_v'] = float(msg.twist.twist.linear.x)
            self.row['odom_w'] = float(msg.twist.twist.angular.z)

    def _on_truth(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        yaw = _yaw_from_quat(p.orientation.x, p.orientation.y,
                             p.orientation.z, p.orientation.w)
        self._gt = (p.position.x, p.position.y, yaw)
        self._gt_mono = time.monotonic()

    def _on_action_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        code = max(msg.status_list, key=lambda s: s.status).status
        if code == self._last_status:
            return
        self._last_status = code
        name = STATUS_NAMES.get(code, f'CODE_{code}')
        with self.lock:
            self.row['action_status'] = name
            self.row['event'] = f'ACTION_STATUS_CHANGE->{name}'
        self.get_logger().info(f'>>> ACTION STATUS {name} ({code})')

    def _on_extra_status(self, msg: GoalStatusArray):
        if not msg.status_list:
            return
        code = max(msg.status_list, key=lambda s: s.status).status
        if code == self._extra_status:
            return
        self._extra_status = code
        with self.lock:
            self.row['extra_action_status'] = STATUS_NAMES.get(code, str(code))
            self.row['event'] = (self.row['event'] + '|' if self.row['event'] else '') \
                + f'extra:{STATUS_NAMES.get(code, code)}'

    def _on_rosout(self, msg: Log):
        if msg.name != self.args.rosout_node:
            return
        with self.lock:
            self._rosout.append(
                (msg.stamp.sec + msg.stamp.nanosec * 1e-9, msg.name, msg.msg))

    def _on_reference(self, msg: Path):
        if self._ref_written or not msg.poses:
            return
        with open(self.ref_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['x', 'y', 'yaw'])
            for ps in msg.poses:
                p = ps.pose
                yaw = _yaw_from_quat(p.orientation.x, p.orientation.y,
                                     p.orientation.z, p.orientation.w)
                w.writerow([p.position.x, p.position.y, yaw])
        self._ref_written = True
        self.get_logger().info(f'reference sidecar: {self.ref_path}')

    def _on_plan(self, msg: Path):
        if msg.poses:
            self._plan = msg  # 存最后一版,退出时落盘

    def _read_truth_pose(self):
        """真值位姿 (x,y,yaw)。优先 /groundtruth/pose(新鲜),否则查 map->base TF。"""
        gt = self._gt
        if gt is not None and (time.monotonic() - self._gt_mono) < self._gt_stale_sec:
            return gt
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        return (t.x, t.y, _yaw_from_quat(q.x, q.y, q.z, q.w))

    def _tick(self):
        pose = self._read_truth_pose()
        with self.lock:
            self.row['t_unix'] = self.get_clock().now().nanoseconds * 1e-9
            if pose is not None:
                self.row['actual_x'], self.row['actual_y'], self.row['actual_yaw'] = pose
                self.row['tf_ok'] = 1
            else:
                self.row['tf_ok'] = 0
            self.writer.writerow(self.row)
            self.row['event'] = ''
        self.csv_file.flush()

    def cleanup(self):
        if self._rosout:
            with open(self.rosout_path, 'w', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                w.writerow(['t_unix', 'node', 'msg'])
                w.writerows(self._rosout)
            self.get_logger().info(f'rosout sidecar: {self.rosout_path}')
        if self._plan is not None:
            with open(self.plan_path, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x', 'y'])
                for ps in self._plan.poses:
                    w.writerow([ps.pose.position.x, ps.pose.position.y])
            self.get_logger().info(f'plan sidecar: {self.plan_path}')
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info(f'track CSV written: {self.csv_path}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run-id', default='')
    p.add_argument('--output-dir', default=os.path.expanduser('~/david_nav_logs/bench'))
    p.add_argument('--map-frame', default='map')
    p.add_argument('--base-frame', default='base_footprint')
    p.add_argument('--truth-topic', default='/groundtruth/pose',
                   help='david_slam_gt 真值话题(主源;拿不到退回 map->base TF)')
    p.add_argument('--rate-hz', type=float, default=20.0)
    # ---- dock_bench 用；不传时行为与改动前完全一致 ----
    p.add_argument('--extra-action-status', default='',
                   help='追加订阅的 action status 话题(dock: /spin/_action/status)')
    p.add_argument('--rosout-node', default='',
                   help='只记录该节点的 /rosout 消息(dock: dock_navigator)')
    args = p.parse_args()

    rclpy.init()
    node = TrackRecorder(args)

    def sigh(signum, frame):
        node.get_logger().warn(f'signal {signum} -> stopping')
        node.cleanup()
        rclpy.shutdown()
        sys.exit(0)
    signal.signal(signal.SIGINT, sigh)
    signal.signal(signal.SIGTERM, sigh)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cleanup()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
