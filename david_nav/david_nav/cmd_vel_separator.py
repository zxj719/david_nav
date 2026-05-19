#!/usr/bin/env python3
"""cmd_vel mutex separator: 100% strict linear / angular decoupling.

Sits at the very end of the cmd_vel chain (after david_move, before the
wheeltec serial driver). Per incoming Twist, it picks whichever axis
(|v_x| or |ω_z|) dominates by normalised magnitude and zeros the other.
A small hysteresis margin avoids mode chatter when the two normalised
magnitudes are close.

Design contract:
- Output is ALWAYS pure (v_x, 0, 0) or (0, 0, ω_z); never both non-zero.
- y velocity is always zero (David is differential-drive).
- On stale input (no msg within `timeout_sec`), publish a zero Twist
  and reset the mode.

Plan D (2026-05-19): paired with disabled obstacle/inflation layers and
DWB discretisation (vx_samples=2, vtheta_samples=3). DWB's mixed samples
get cut at the wheel-driver hop so the chassis only ever does pure spin
or pure straight motion — natural correction comes from re-aligning when
heading drifts.
"""

from __future__ import annotations

from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class Mode(Enum):
    IDLE = 0
    LINEAR = 1
    ANGULAR = 2


class CmdVelSeparator(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_separator')

        self.declare_parameter('input_topic', '/cmd_vel_raw')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('v_max_ref', 0.20)
        self.declare_parameter('w_max_ref', 0.50)
        self.declare_parameter('hysteresis_margin', 0.10)
        self.declare_parameter('min_norm_threshold', 0.05)
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('log_period_sec', 2.0)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.v_max_ref = max(1e-6, float(self.get_parameter('v_max_ref').value))
        self.w_max_ref = max(1e-6, float(self.get_parameter('w_max_ref').value))
        self.hysteresis_margin = float(
            self.get_parameter('hysteresis_margin').value
        )
        self.min_norm_threshold = float(
            self.get_parameter('min_norm_threshold').value
        )
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.log_period_sec = max(
            0.1, float(self.get_parameter('log_period_sec').value)
        )

        self.mode = Mode.IDLE
        self._last_input_stamp = self.get_clock().now()
        self._last_log_stamp = self.get_clock().now()
        self._counters = {Mode.IDLE: 0, Mode.LINEAR: 0, Mode.ANGULAR: 0}

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(Twist, self.output_topic, qos)
        self.sub = self.create_subscription(
            Twist, self.input_topic, self._on_cmd, qos
        )

        self.timer = self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f'cmd_vel_separator up: {self.input_topic} -> {self.output_topic} '
            f'(v_max_ref={self.v_max_ref:.2f}, w_max_ref={self.w_max_ref:.2f}, '
            f'hysteresis={self.hysteresis_margin:.2f}, '
            f'min_norm={self.min_norm_threshold:.2f})'
        )

    def _on_cmd(self, msg: Twist) -> None:
        self._last_input_stamp = self.get_clock().now()

        v = float(msg.linear.x)
        w = float(msg.angular.z)
        lin_norm = abs(v) / self.v_max_ref
        ang_norm = abs(w) / self.w_max_ref

        # Both axes near zero → publish stop, return to IDLE (no preference).
        if (lin_norm < self.min_norm_threshold
                and ang_norm < self.min_norm_threshold):
            self.mode = Mode.IDLE
            self._publish_stop()
            self._tick_log()
            return

        # State machine with hysteresis:
        # - From IDLE: pick whichever axis is bigger.
        # - From LINEAR: stay LINEAR unless ang_norm > lin_norm + margin.
        # - From ANGULAR: stay ANGULAR unless lin_norm > ang_norm + margin.
        if self.mode is Mode.IDLE:
            self.mode = Mode.ANGULAR if ang_norm >= lin_norm else Mode.LINEAR
        elif self.mode is Mode.LINEAR:
            if ang_norm > lin_norm + self.hysteresis_margin:
                self.mode = Mode.ANGULAR
        elif self.mode is Mode.ANGULAR:
            if lin_norm > ang_norm + self.hysteresis_margin:
                self.mode = Mode.LINEAR

        out = Twist()
        if self.mode is Mode.LINEAR:
            out.linear.x = v
            # angular.z stays 0
        else:  # ANGULAR
            out.angular.z = w
            # linear.x stays 0
        self.pub.publish(out)
        self._counters[self.mode] += 1
        self._tick_log()

    def _publish_stop(self) -> None:
        self.pub.publish(Twist())
        self._counters[Mode.IDLE] += 1

    def _watchdog(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_input_stamp).nanoseconds * 1e-9
        if dt > self.timeout_sec and self.mode is not Mode.IDLE:
            self.mode = Mode.IDLE
            self._publish_stop()
            self.get_logger().warn(
                f'cmd_vel_separator: input stale ({dt:.2f}s) -> stop'
            )

    def _tick_log(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_log_stamp).nanoseconds * 1e-9
        if dt < self.log_period_sec:
            return
        self._last_log_stamp = now
        c = self._counters
        total = max(1, c[Mode.LINEAR] + c[Mode.ANGULAR] + c[Mode.IDLE])
        self.get_logger().info(
            f'cmd_vel_separator stats (last {dt:.1f}s): '
            f'LINEAR={c[Mode.LINEAR]} ANGULAR={c[Mode.ANGULAR]} '
            f'IDLE={c[Mode.IDLE]} mode={self.mode.name} '
            f'lin_frac={c[Mode.LINEAR]/total:.2f}'
        )
        for k in c:
            c[k] = 0


def main(argv=None):
    rclpy.init(args=argv)
    node = CmdVelSeparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
