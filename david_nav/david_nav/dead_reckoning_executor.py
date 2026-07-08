#!/usr/bin/env python3
"""dead_reckoning_executor — odom-only fixed-sequence action server.

收到任意 nav2_msgs/NavigateToPose goal,忽略 target_pose,执行固定开环序列:
    1. 后退 reverse_distance(m,默认 1.0)
    2. 原地转 turn_angle(rad,默认 π/2)
    3. 前进 forward_distance(m,默认 0.0 = 关掉这段)
全程只看 /odom(wheeltec 纯轮速)闭里程,直接发 /cmd_vel。到点 succeed。

不依赖 nav2 / planner / map / TF map→odom。与 navigation_executor.py 的
WebSocket 协议**完全无关**:协议层把 target_pose 透到 NavigateToPose action,
本节点作为 action 端点把它丢掉,改跑死序列(start_waypoint_nav_headless.sh
当前默认这个 action)。

参数(rosparam,皆有默认):
  action_name              str   动作名(绝对)默认 /dead_reckoning/navigate
  odom_topic               str   /odom
  cmd_vel_topic            str   /cmd_vel
  reverse_distance         double 1.0 m
  turn_angle               double π/2 rad
  forward_distance         double 0.0 m(<= dist_tol 即整段跳过)
  lin_speed                double 0.20 m/s(绝对值,符号由相位决定)
  ang_speed                double 0.35 rad/s
  dist_tol                 double 0.05 m
  yaw_tol                  double 0.052 rad(≈3°)
  inter_phase_pause_sec    double 0.5 s(段与段之间停一下,排空动量)
  odom_wait_timeout_sec    double 5.0 s
  control_rate_hz          double 20.0 Hz
"""

from __future__ import annotations

import math
import threading
import time
from enum import Enum

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Phase(Enum):
    REVERSE = "REVERSE"
    TURN = "TURN"
    FORWARD = "FORWARD"


class DeadReckoningExecutor(Node):
    def __init__(self) -> None:
        super().__init__("dead_reckoning_executor")

        self.declare_parameter("action_name", "/dead_reckoning/navigate")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("reverse_distance", 1.0)
        self.declare_parameter("turn_angle", math.pi / 2.0)
        self.declare_parameter("forward_distance", 0.0)

        self.declare_parameter("lin_speed", 0.20)
        self.declare_parameter("ang_speed", 0.35)

        self.declare_parameter("dist_tol", 0.05)
        self.declare_parameter("yaw_tol", math.radians(3.0))

        self.declare_parameter("inter_phase_pause_sec", 0.5)
        self.declare_parameter("odom_wait_timeout_sec", 5.0)
        self.declare_parameter("control_rate_hz", 20.0)

        g = self.get_parameter
        self.action_name = str(g("action_name").value)
        self.odom_topic = str(g("odom_topic").value)
        self.cmd_vel_topic = str(g("cmd_vel_topic").value)
        self.reverse_distance = float(g("reverse_distance").value)
        self.turn_angle = float(g("turn_angle").value)
        self.forward_distance = float(g("forward_distance").value)
        self.lin_speed = abs(float(g("lin_speed").value))
        self.ang_speed = abs(float(g("ang_speed").value))
        self.dist_tol = float(g("dist_tol").value)
        self.yaw_tol = float(g("yaw_tol").value)
        self.inter_phase_pause_sec = float(g("inter_phase_pause_sec").value)
        self.odom_wait_timeout_sec = float(g("odom_wait_timeout_sec").value)
        self.control_rate_hz = float(g("control_rate_hz").value)

        self._cb = ReentrantCallbackGroup()
        self._odom_lock = threading.Lock()
        # (x, y, yaw) — wheeltec 纯轮速,frame 一般是 odom
        self._latest_odom: tuple[float, float, float] | None = None
        # 单 goal 互斥:第二个 goal 进来时让前一个先 cancel
        self._goal_lock = threading.Lock()
        self._cancel_evt = threading.Event()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, 50,
            callback_group=self._cb,
        )
        self._server = ActionServer(
            self, NavigateToPose, self.action_name,
            execute_callback=self._execute,
            goal_callback=lambda _r: GoalResponse.ACCEPT,
            cancel_callback=self._on_cancel,
            callback_group=self._cb,
        )

        self.get_logger().info(
            "dead_reckoning_executor up: action='%s', odom='%s', cmd_vel='%s', "
            "seq=[-%.2fm, +%.0f°, +%.2fm], v=%.2fm/s w=%.2frad/s, tol=(%.0fcm, %.0f°)"
            % (
                self.action_name, self.odom_topic, self.cmd_vel_topic,
                self.reverse_distance, math.degrees(self.turn_angle),
                self.forward_distance, self.lin_speed, self.ang_speed,
                self.dist_tol * 100.0, math.degrees(self.yaw_tol),
            )
        )

    # ---- odom ----
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._odom_lock:
            self._latest_odom = (float(p.x), float(p.y), yaw)

    def _read_odom(self) -> tuple[float, float, float] | None:
        with self._odom_lock:
            return self._latest_odom

    def _wait_for_odom(self) -> bool:
        deadline = time.monotonic() + self.odom_wait_timeout_sec
        while time.monotonic() < deadline:
            if self._cancel_evt.is_set():
                return False
            if self._read_odom() is not None:
                return True
            time.sleep(0.05)
        return False

    def publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    # ---- cancel ----
    def _on_cancel(self, _goal_handle) -> CancelResponse:
        self._cancel_evt.set()
        return CancelResponse.ACCEPT

    # ---- main execute ----
    def _execute(self, goal_handle):
        # 串行化:同时只跑一个序列。第二个 goal 进来 → 先让旧的看到 cancel。
        with self._goal_lock:
            self._cancel_evt.clear()

            # 幅度 <= 容差视为关闭该段:目前默认前进 2m 已取消(2026-06-23)。
            # 用户/launch 想再开启时,把 forward_distance 调到 > dist_tol 即可。
            phases = []
            if self.reverse_distance > self.dist_tol:
                phases.append((Phase.REVERSE, self.reverse_distance, -1))
            if self.turn_angle > self.yaw_tol:
                phases.append((Phase.TURN, self.turn_angle, +1))
            if self.forward_distance > self.dist_tol:
                phases.append((Phase.FORWARD, self.forward_distance, +1))

            seq_str = " → ".join(
                "%s%.2f%s" % (
                    "-" if sign < 0 else "+",
                    math.degrees(mag) if ph is Phase.TURN else mag,
                    "°" if ph is Phase.TURN else "m",
                ) for ph, mag, sign in phases
            ) or "(空序列,直接 succeed)"
            self.get_logger().info(
                "[dead_reckoning] goal accepted → " + seq_str
            )

            if not phases:
                goal_handle.succeed()
                return NavigateToPose.Result()

            if not self._wait_for_odom():
                self.publish_zero()
                goal_handle.abort()
                self.get_logger().error(
                    "[dead_reckoning] odom %.1fs 未到,abort"
                    % self.odom_wait_timeout_sec
                )
                return NavigateToPose.Result()

            for phase, magnitude, sign in phases:
                outcome = self._run_phase(goal_handle, phase, magnitude, sign)
                if outcome != "ok":
                    self.publish_zero()
                    # _run_phase 已经走过 canceled()/abort();这里只补零速
                    return NavigateToPose.Result()
                self.publish_zero()
                # 段间停一下:让动量散掉,避免上段惯性吃进下段 odom 积分。
                self._sleep_interruptible(self.inter_phase_pause_sec)
                if self._cancel_evt.is_set():
                    goal_handle.canceled()
                    self.get_logger().warn("[dead_reckoning] 段间被取消")
                    return NavigateToPose.Result()

            goal_handle.succeed()
            self.get_logger().info("[dead_reckoning] 序列完成 → succeed")
            return NavigateToPose.Result()

    def _run_phase(self, goal_handle, phase: Phase, magnitude: float,
                   sign: int) -> str:
        """开环跑一段,只看 odom 闭里程。

        返回 "ok" / "cancel" / "abort"。出错的状态机收尾(canceled/abort)在这里
        调,_execute 只补零速 + 返回。"""
        start = self._read_odom()
        if start is None:
            goal_handle.abort()
            self.get_logger().error("[%s] 起点无 odom,abort" % phase.value)
            return "abort"
        x0, y0, yaw0 = start

        cmd = Twist()
        if phase is Phase.TURN:
            cmd.angular.z = sign * self.ang_speed
            tol = self.yaw_tol
        else:
            cmd.linear.x = sign * self.lin_speed
            tol = self.dist_tol

        rate_period = 1.0 / max(1.0, self.control_rate_hz)
        self.get_logger().info(
            "[%s] start magnitude=%.3f tol=%.3f cmd=(v=%.2f,w=%.2f)"
            % (phase.value, magnitude, tol, cmd.linear.x, cmd.angular.z)
        )

        last_log = 0.0
        while rclpy.ok():
            if goal_handle.is_cancel_requested or self._cancel_evt.is_set():
                self.publish_zero()
                goal_handle.canceled()
                self.get_logger().warn("[%s] 取消" % phase.value)
                return "cancel"

            cur = self._read_odom()
            if cur is None:
                self.publish_zero()
                goal_handle.abort()
                self.get_logger().error("[%s] 失去 odom,abort" % phase.value)
                return "abort"
            x, y, yaw = cur

            if phase is Phase.TURN:
                progress = abs(_wrap_pi(yaw - yaw0))
            else:
                progress = math.hypot(x - x0, y - y0)
            remaining = magnitude - progress

            if remaining <= tol:
                self.publish_zero()
                self.get_logger().info(
                    "[%s] reached: progress=%.3f remaining=%.3f"
                    % (phase.value, progress, remaining)
                )
                return "ok"

            self.cmd_pub.publish(cmd)
            now = time.monotonic()
            if now - last_log >= 1.0:
                self.get_logger().info(
                    "[%s] progress=%.3f/%.3f remaining=%.3f"
                    % (phase.value, progress, magnitude, remaining)
                )
                last_log = now
            time.sleep(rate_period)

        self.publish_zero()
        return "abort"

    def _sleep_interruptible(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if self._cancel_evt.is_set():
                return
            time.sleep(0.05)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DeadReckoningExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_zero()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
