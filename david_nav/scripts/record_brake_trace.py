#!/usr/bin/env python3
"""Record the cmd_vel chain + odom + action status into a CSV around a
NavigateToPose goal, so we can analyse why brake isn't firing at goal
arrival.

Subscribes to:
  /cmd_vel_nav         DWB output (Twist)
  /cmd_vel_smoothed    after velocity_smoother (Twist)
  /cmd_vel_safe        after collision_monitor (Twist)
  /cmd_vel_raw         after david_move (Twist)
  /cmd_vel             separator output / chassis input (Twist)
  /odom                wheeltec wheel odometry (Odometry — twist used)
  /plan                planner output (Path — last pose = goal)
  /navigate_to_pose/_action/status   action progress (status code)

Writes a CSV row at 20 Hz with all the latest values. Special events
(action status changes, e.g. SUCCEEDED=4 / ABORTED=6) are tagged in
the `event` column AND printed to stdout.

CSV path printed at startup. Stop with Ctrl-C.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from threading import Lock

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


# nav2 action GoalStatus codes (subset)
STATUS_NAMES = {
    0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
    4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED',
}


class BrakeTracer(Node):
    def __init__(self, args):
        super().__init__('record_brake_trace')
        self.args = args
        self.lock = Lock()

        self.row = {
            't_unix': 0.0,
            'cmd_vel_nav_v': float('nan'), 'cmd_vel_nav_w': float('nan'),
            'cmd_vel_smoothed_v': float('nan'), 'cmd_vel_smoothed_w': float('nan'),
            'cmd_vel_safe_v': float('nan'), 'cmd_vel_safe_w': float('nan'),
            'cmd_vel_raw_v': float('nan'), 'cmd_vel_raw_w': float('nan'),
            'cmd_vel_v': float('nan'), 'cmd_vel_w': float('nan'),
            'odom_v': float('nan'), 'odom_w': float('nan'),
            'odom_x': float('nan'), 'odom_y': float('nan'),
            'plan_goal_x': float('nan'), 'plan_goal_y': float('nan'),
            'plan_age_ms': float('nan'),
            'action_status': '',
            'event': '',
        }
        self._plan_t = None
        self._last_status = None

        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # cmd_vel chain
        self.create_subscription(Twist, '/cmd_vel_nav',
                                 self._mk_twist_cb('cmd_vel_nav'), qos)
        self.create_subscription(Twist, '/cmd_vel_smoothed',
                                 self._mk_twist_cb('cmd_vel_smoothed'), qos)
        self.create_subscription(Twist, '/cmd_vel_safe',
                                 self._mk_twist_cb('cmd_vel_safe'), qos)
        self.create_subscription(Twist, '/cmd_vel_raw',
                                 self._mk_twist_cb('cmd_vel_raw'), qos)
        self.create_subscription(Twist, '/cmd_vel',
                                 self._mk_twist_cb('cmd_vel'), qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos)
        self.create_subscription(Path, '/plan', self._on_plan, qos)
        # GoalStatusArray published on /navigate_to_pose/_action/status
        self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self._on_action_status, qos)

        # CSV writer
        ts = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = args.output or f'/tmp/brake_trace_{ts}.csv'
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=list(self.row.keys()))
        self.writer.writeheader()
        self.csv_file.flush()

        self.timer = self.create_timer(1.0 / args.rate_hz, self._tick)

        self.get_logger().info(f'CSV: {self.csv_path}')
        self.get_logger().info(f'Recording at {args.rate_hz} Hz — Ctrl-C to stop')

    def _mk_twist_cb(self, name):
        def cb(msg: Twist):
            with self.lock:
                self.row[f'{name}_v'] = float(msg.linear.x)
                self.row[f'{name}_w'] = float(msg.angular.z)
        return cb

    def _on_odom(self, msg: Odometry):
        with self.lock:
            self.row['odom_v'] = float(msg.twist.twist.linear.x)
            self.row['odom_w'] = float(msg.twist.twist.angular.z)
            self.row['odom_x'] = float(msg.pose.pose.position.x)
            self.row['odom_y'] = float(msg.pose.pose.position.y)

    def _on_plan(self, msg: Path):
        if not msg.poses:
            return
        with self.lock:
            last = msg.poses[-1].pose.position
            self.row['plan_goal_x'] = float(last.x)
            self.row['plan_goal_y'] = float(last.y)
            self._plan_t = self.get_clock().now()

    def _on_action_status(self, msg: GoalStatusArray):
        # Track the LATEST status for any goal (usually one active).
        if not msg.status_list:
            return
        latest = max(msg.status_list, key=lambda s: s.status)
        code = latest.status
        if code == self._last_status:
            return
        self._last_status = code
        name = STATUS_NAMES.get(code, f'CODE_{code}')
        with self.lock:
            self.row['action_status'] = name
            self.row['event'] = f'ACTION_STATUS_CHANGE→{name}'
        self.get_logger().info(f'>>> ACTION STATUS {name} ({code})')

    def _tick(self):
        now = self.get_clock().now()
        with self.lock:
            self.row['t_unix'] = now.nanoseconds * 1e-9
            if self._plan_t is not None:
                age = (now - self._plan_t).nanoseconds * 1e-3 / 1000  # → ms
                self.row['plan_age_ms'] = age
            self.writer.writerow(self.row)
            # Clear event so it doesn't repeat across rows.
            self.row['event'] = ''
        self.csv_file.flush()

    def cleanup(self):
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info(f'CSV written: {self.csv_path}')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--rate-hz', type=float, default=20.0,
                   help='CSV write rate (default 20)')
    p.add_argument('--output', default='',
                   help='CSV path (default /tmp/brake_trace_<ts>.csv)')
    args = p.parse_args()

    rclpy.init()
    node = BrakeTracer(args)

    def sigh(signum, frame):
        node.get_logger().warn(f'signal {signum} → stopping')
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
