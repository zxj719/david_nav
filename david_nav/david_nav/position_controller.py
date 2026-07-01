#!/usr/bin/env python3
"""position_controller — David 上位机位置环 (body-pose closed loop on /odom).

Nav2 之外的并列速度来源: 对车体位姿做闭环, 输出速度命令到 /cmd_vel_position,
经 mux (cmd_vel_mux / twist_mux) 二选一进底盘。不做避障/规划 (那是 Nav2),
只做直线 + 原地转 + turn-drive-turn 定点。两轮差速, 只发 linear.x / angular.z。

控制数学全部在 position_control_core (纯函数, 可单测); 本节点只负责 ROS I/O:
/odom → (x,y,yaw,v,w) → PositionController.update → /cmd_vel_position。

接口 (SPEC §2):
  Action ~/move    : david_nav_msgs/action/MoveTo  (主, 支持 cancel)
  Topic  ~/goal_pose: geometry_msgs/PoseStamped     (便捷, 触发 POSE)
See SPEC 位置环实现方案 §2-§5.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from david_nav_msgs.action import MoveTo

from david_nav.position_control_core import (
    Goal,
    Limits,
    Mode,
    PidGains,
    PositionController,
    Tolerances,
    wrap_to_pi,
)


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """yaw from quaternion (SPEC §5). atan2(2(wz+xy), 1-2(yy+zz))."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


class _OdomSnap:
    __slots__ = ("x", "y", "yaw", "v", "w", "stamp")

    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0
        self.stamp = None  # rclpy.time.Time or None until first odom


class PositionControllerNode(Node):
    def __init__(self) -> None:
        super().__init__("position_controller")

        # ---- params (SPEC §4) ----
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_topic", "/cmd_vel_position")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("v_max", 0.25)
        self.declare_parameter("w_max", 0.8)
        self.declare_parameter("a_max", 0.5)
        self.declare_parameter("alpha_max", 1.5)
        # Separate (faster) angular decel limit; 0 = symmetric (= alpha_max).
        # Lets the rotate_lead cut stop fast without making turn-start jerky.
        self.declare_parameter("alpha_decel_max", 0.0)
        # Angular approach-decel: cap |w| by sqrt(2*alpha_approach*err) near the
        # target so the chassis arrives slow (less momentum overshoot). 0 = off.
        self.declare_parameter("alpha_approach", 0.0)
        self.declare_parameter("approach_decel", 0.5)
        self.declare_parameter("dist.Kp", 1.2)
        self.declare_parameter("dist.Ki", 0.0)
        self.declare_parameter("dist.Kd", 0.1)
        self.declare_parameter("dist.I_max", 0.3)
        self.declare_parameter("yaw.Kp", 2.0)
        self.declare_parameter("yaw.Ki", 0.0)
        self.declare_parameter("yaw.Kd", 0.15)
        self.declare_parameter("yaw.I_max", 0.5)
        self.declare_parameter("dist_tol", 0.02)
        self.declare_parameter("pose_dist_tol", 0.05)
        self.declare_parameter("yaw_tol", 0.02)
        self.declare_parameter("pose_yaw_tol", 0.35)
        self.declare_parameter("pose_align_tol", 0.10)
        self.declare_parameter("rotate_lead", 0.15)
        self.declare_parameter("drive_drift_lead", 0.0)
        self.declare_parameter("drive_overshoot_margin", 0.05)
        self.declare_parameter("pose_max_cycles", 4)
        self.declare_parameter("arrival_cycles", 10)
        self.declare_parameter("v_stop", 0.01)
        self.declare_parameter("w_stop", 0.05)
        self.declare_parameter("goal_timeout", 30.0)
        self.declare_parameter("odom_timeout", 0.3)
        # Grace to wait for the FIRST odom at goal start (DDS discovery race for
        # a freshly-spawned node / chassis still coming up). Distinct from
        # odom_timeout, which guards mid-run odom loss.
        self.declare_parameter("odom_start_grace", 5.0)
        self.declare_parameter("invert_linear", False)
        self.declare_parameter("min_cmd_v", 0.0)
        self.declare_parameter("min_cmd_w", 0.0)
        self.declare_parameter("robot_frame", "base_footprint")

        g = self.get_parameter
        self.odom_topic = str(g("odom_topic").value)
        self.cmd_topic = str(g("cmd_topic").value)
        self.control_rate = max(1.0, float(g("control_rate").value))
        self.goal_timeout = float(g("goal_timeout").value)
        self.odom_timeout = float(g("odom_timeout").value)
        self.odom_start_grace = float(g("odom_start_grace").value)
        self.invert_linear = bool(g("invert_linear").value)
        self.robot_frame = str(g("robot_frame").value)

        self.limits = Limits(
            v_max=float(g("v_max").value),
            w_max=float(g("w_max").value),
            a_max=float(g("a_max").value),
            alpha_max=float(g("alpha_max").value),
            alpha_decel_max=float(g("alpha_decel_max").value),
            alpha_approach=float(g("alpha_approach").value),
            approach_decel=float(g("approach_decel").value),
            min_cmd_v=float(g("min_cmd_v").value),
            min_cmd_w=float(g("min_cmd_w").value),
        )
        self.dist_gains = PidGains(
            kp=float(g("dist.Kp").value), ki=float(g("dist.Ki").value),
            kd=float(g("dist.Kd").value), i_max=float(g("dist.I_max").value),
        )
        self.yaw_gains = PidGains(
            kp=float(g("yaw.Kp").value), ki=float(g("yaw.Ki").value),
            kd=float(g("yaw.Kd").value), i_max=float(g("yaw.I_max").value),
        )
        self.tol = self._read_tolerances()

        # ---- state ----
        self._odom = _OdomSnap()
        self._odom_lock = threading.Lock()
        self._shutdown_requested = False
        # Only one driver (action OR topic goal) at a time.
        self._drive_lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(Twist, self.cmd_topic, qos)
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self._on_odom, qos, callback_group=cb)
        self.sub_goal = self.create_subscription(
            PoseStamped, "~/goal_pose", self._on_goal_pose, qos,
            callback_group=cb)

        self.action_server = ActionServer(
            self, MoveTo, "~/move",
            execute_callback=self._execute,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=cb,
        )

        self.get_logger().info(
            f"position_controller up: {self.odom_topic} -> {self.cmd_topic} "
            f"@ {self.control_rate:.0f} Hz "
            f"(v_max={self.limits.v_max:.2f}, w_max={self.limits.w_max:.2f}, "
            f"invert_linear={self.invert_linear})"
        )

    # ------------------------------------------------------------------ #
    # odom
    # ------------------------------------------------------------------ #
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        with self._odom_lock:
            self._odom.x = p.x
            self._odom.y = p.y
            self._odom.yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            self._odom.v = msg.twist.twist.linear.x
            self._odom.w = msg.twist.twist.angular.z
            self._odom.stamp = self.get_clock().now()

    def _snap(self) -> _OdomSnap:
        with self._odom_lock:
            s = _OdomSnap()
            s.x, s.y, s.yaw = self._odom.x, self._odom.y, self._odom.yaw
            s.v, s.w, s.stamp = self._odom.v, self._odom.w, self._odom.stamp
            return s

    # ------------------------------------------------------------------ #
    # publishing (non-holonomic: only linear.x / angular.z ever set)
    # ------------------------------------------------------------------ #
    def _publish(self, v: float, w: float) -> None:
        out = Twist()
        out.linear.x = -v if self.invert_linear else v
        out.angular.z = w
        self.pub.publish(out)

    def _stop(self, times: int = 3) -> None:
        for _ in range(times):
            self._publish(0.0, 0.0)

    # ------------------------------------------------------------------ #
    def _read_tolerances(self) -> Tolerances:
        """从当前参数值构造 Tolerances。每次接 goal 都重读(2026-06-10):
        waypoint_follower 在段间用 set_parameters 切分层容差——中间 waypoint
        松(0.8m/不管角度)、最终 goal 紧(0.4m/0.4rad)。"""
        g = self.get_parameter
        return Tolerances(
            dist_tol=float(g("dist_tol").value),
            pose_dist_tol=float(g("pose_dist_tol").value),
            yaw_tol=float(g("yaw_tol").value),
            pose_yaw_tol=float(g("pose_yaw_tol").value),
            pose_align_tol=float(g("pose_align_tol").value),
            rotate_lead=float(g("rotate_lead").value),
            drive_drift_lead=float(g("drive_drift_lead").value),
            drive_overshoot_margin=float(g("drive_overshoot_margin").value),
            pose_max_cycles=int(g("pose_max_cycles").value),
            arrival_cycles=int(g("arrival_cycles").value),
            v_stop=float(g("v_stop").value),
            w_stop=float(g("w_stop").value),
        )

    # ------------------------------------------------------------------ #
    # goal resolution: convert a request into an odom-frame core Goal
    # ------------------------------------------------------------------ #
    def _resolve_goal(self, mode, distance, yaw, tx_in, ty_in, relative,
                      start: _OdomSnap) -> Goal:
        if mode == Mode.DISTANCE:
            return Goal(mode=Mode.DISTANCE, distance=distance)
        if mode == Mode.ROTATE:
            return Goal(mode=Mode.ROTATE, yaw=yaw)
        # POSE. yaw=NaN ⇒ position-only (skip the final in-place turn). Useful
        # when the heading doesn't matter (e.g. re-home): a large final turn
        # scrubs the point via caster trailing and can fail to converge.
        final = not math.isnan(yaw)
        if relative:
            c, s = math.cos(start.yaw), math.sin(start.yaw)
            tx = start.x + tx_in * c - ty_in * s
            ty = start.y + tx_in * s + ty_in * c
            target_yaw = wrap_to_pi(start.yaw + yaw) if final else 0.0
        else:
            tx, ty = tx_in, ty_in
            target_yaw = wrap_to_pi(yaw) if final else 0.0
        return Goal(mode=Mode.POSE, target_x=tx, target_y=ty,
                    yaw=target_yaw, has_final_heading=final)

    # ------------------------------------------------------------------ #
    # the universal control loop — used by BOTH action and topic goal.
    # goal_handle is None for the topic path.
    # returns (success, final_error)
    # ------------------------------------------------------------------ #
    def _run_goal(self, goal: Goal, goal_handle=None) -> tuple:
        if not self._drive_lock.acquire(blocking=False):
            self.get_logger().warn("position_controller busy — goal rejected")
            if goal_handle is not None:
                goal_handle.abort()
            return False, float("nan")
        self.tol = self._read_tolerances()  # 段间可能被 set_parameters 改过
        controller = PositionController(
            self.limits, self.dist_gains, self.yaw_gains, self.tol)
        try:
            # Wait for the first odom (DDS discovery race for a freshly-spawned
            # node, or chassis still booting). Only the *startup* case waits;
            # mid-run loss is handled by odom_timeout in the loop below.
            t_wait = time.monotonic()
            start = self._snap()
            while start.stamp is None and rclpy.ok():
                if time.monotonic() - t_wait > self.odom_start_grace:
                    self.get_logger().error(
                        f"no /odom after {self.odom_start_grace:.1f}s — abort goal")
                    self._stop()
                    if goal_handle is not None:
                        goal_handle.abort()
                    return False, float("nan")
                time.sleep(0.05)
                start = self._snap()
            controller.begin(goal, start.x, start.y, start.yaw)

            period = 1.0 / self.control_rate
            t_start = self.get_clock().now()
            last_tick = time.monotonic()
            final_error = float("nan")
            success = False
            last_stage = ""
            self.get_logger().info(
                f"goal start: mode={goal.mode.name} from "
                f"({start.x:.2f},{start.y:.2f},{math.degrees(start.yaw):.0f}deg)")

            while rclpy.ok() and not self._shutdown_requested:
                tick = time.monotonic()
                dt = tick - last_tick
                last_tick = tick
                if dt <= 0.0:
                    dt = period

                snap = self._snap()
                now = self.get_clock().now()

                # odom liveness
                odom_age = (now - snap.stamp).nanoseconds * 1e-9
                if odom_age > self.odom_timeout:
                    self.get_logger().warn(
                        f"/odom stale ({odom_age:.2f}s > {self.odom_timeout:.2f}s)"
                        " — abort, zero")
                    self._stop()
                    if goal_handle is not None:
                        goal_handle.abort()
                    return False, final_error

                # goal timeout
                if (now - t_start).nanoseconds * 1e-9 > self.goal_timeout:
                    self.get_logger().warn("goal_timeout — abort, zero")
                    self._stop()
                    if goal_handle is not None:
                        goal_handle.abort()
                    return False, final_error

                # cancel (action only)
                if goal_handle is not None and goal_handle.is_cancel_requested:
                    self.get_logger().info("goal canceled — zero")
                    self._stop()
                    goal_handle.canceled()
                    return False, final_error

                out = controller.update(snap.x, snap.y, snap.yaw, dt,
                                        snap.v, snap.w)
                final_error = out.error_remaining

                # log POSE/phase transitions for diagnosis
                if out.stage != last_stage:
                    self.get_logger().info(
                        f"  stage {last_stage or '-'} -> {out.stage} "
                        f"(err={out.error_remaining:+.3f})")
                    last_stage = out.stage

                if out.done:
                    self._stop()
                    success = out.success
                    break

                self._publish(out.v, out.w)

                if goal_handle is not None:
                    fb = MoveTo.Feedback()
                    fb.error_remaining = float(out.error_remaining)
                    fb.current_speed = float(
                        abs(out.w) if goal.mode == Mode.ROTATE else abs(out.v))
                    goal_handle.publish_feedback(fb)

                # pace the loop
                sleep_left = period - (time.monotonic() - tick)
                if sleep_left > 0.0:
                    time.sleep(sleep_left)

            # loop may have exited on shutdown request (Ctrl-C) → zero the chassis
            if self._shutdown_requested and not success:
                self._stop()
                if goal_handle is not None:
                    goal_handle.abort()
                return False, final_error
            if goal_handle is not None:
                if success:
                    goal_handle.succeed()
                # abort/cancel already handled in-loop
            self.get_logger().info(
                f"goal done: success={success} final_error={final_error:.4f}")
            return success, final_error
        finally:
            self._drive_lock.release()

    # ------------------------------------------------------------------ #
    # action callbacks
    # ------------------------------------------------------------------ #
    def _on_goal_request(self, goal_request) -> GoalResponse:
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        req = goal_handle.request
        mode = Mode(req.mode)
        start = self._snap()
        goal = self._resolve_goal(
            mode, req.distance, req.yaw,
            req.target.x, req.target.y, req.relative, start)
        success, final_error = self._run_goal(goal, goal_handle)
        result = MoveTo.Result()
        result.success = bool(success)
        result.final_error = float(final_error)
        return result

    # ------------------------------------------------------------------ #
    # topic goal (convenience) — runs in its own thread so the executor
    # is never blocked; _drive_lock serializes vs the action path.
    # ------------------------------------------------------------------ #
    def _on_goal_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        frame = (msg.header.frame_id or "").lstrip("/")
        relative = frame in (self.robot_frame, "base_footprint", "base_link", "")
        if frame and frame not in (self.robot_frame, "base_footprint",
                                   "base_link", "odom", ""):
            self.get_logger().warn(
                f"goal_pose frame '{frame}' not odom/base_footprint — "
                "treating as odom-absolute (no TF transform here)")
            relative = False
        start = self._snap()
        goal = self._resolve_goal(
            Mode.POSE, 0.0, yaw, msg.pose.position.x, msg.pose.position.y,
            relative, start)
        self.get_logger().info(
            f"goal_pose -> POSE target=({goal.target_x:.3f},{goal.target_y:.3f}) "
            f"yaw={goal.yaw:.3f} relative={relative}")
        threading.Thread(target=self._run_goal, args=(goal, None),
                         daemon=True).start()


def main(argv=None):
    # SAFETY: manage signals ourselves so rclpy's default handler does NOT shut
    # the context down on SIGINT before we can publish zero. Otherwise Ctrl-C
    # leaves the chassis running its last command (observed 2026-06-09).
    import signal as _signal
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
        managed = True
    except Exception:
        rclpy.init(args=argv)
        managed = False

    node = PositionControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    if managed:
        def _on_stop(signum, frame):
            node._shutdown_requested = True   # break the goal loop + spin loop
        _signal.signal(_signal.SIGINT, _on_stop)
        _signal.signal(_signal.SIGTERM, _on_stop)

    try:
        while rclpy.ok() and not node._shutdown_requested:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node._shutdown_requested = True
        # Wait briefly for the goal thread to release the drive lock (it zeros on
        # exit), then hammer zero and flush so the chassis actually stops.
        got = node._drive_lock.acquire(timeout=1.0)
        for _ in range(10):
            try:
                node._stop()
                executor.spin_once(timeout_sec=0.01)
            except Exception:
                break
        if got:
            node._drive_lock.release()
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
