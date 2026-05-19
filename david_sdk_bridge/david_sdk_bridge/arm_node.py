#!/usr/bin/env python3
"""Single-arm ROS 2 node bridging xCoreSDK to MoveIt (safe lazy power-on)."""
from __future__ import annotations
import time
import platform
import threading
from typing import List

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory

from david_sdk_bridge import setup_sdk_path  # noqa: F401

if platform.machine() == "aarch64":
    from Release.linux.arm import xCoreSDK_python
else:
    from Release.linux import xCoreSDK_python


class ArmNode(Node):
    def __init__(self):
        super().__init__("arm_node")
        self.declare_parameter("side", "left")
        self.declare_parameter("robot_ip", "192.168.160.162")
        self.declare_parameter("local_ip", "")
        self.declare_parameter("controller_name", "left_arm_controller")
        self.declare_parameter("joint_name_prefix", "AR5-5_08L-W4C4A6-ZY2_joint_")
        self.declare_parameter("sdk_speed", 800)
        self.declare_parameter("sdk_zone", 80)
        self.declare_parameter("state_pub_hz", 50.0)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("power_on_at_start", False)
        self.declare_parameter("joint_offsets_deg", [0.0]*7)  # 安全默认 false
        self.declare_parameter("home_pose_deg", [0.0]*7)  # home pose in SDK coords (deg)

        self.side = self.get_parameter("side").value
        self.robot_ip = self.get_parameter("robot_ip").value
        self.local_ip = self.get_parameter("local_ip").value
        self.controller_name = self.get_parameter("controller_name").value
        self.prefix = self.get_parameter("joint_name_prefix").value
        self.sdk_speed = int(self.get_parameter("sdk_speed").value)
        self.sdk_zone = int(self.get_parameter("sdk_zone").value)
        self.state_pub_hz = float(self.get_parameter("state_pub_hz").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.power_on_at_start = bool(self.get_parameter("power_on_at_start").value)
        import math as _math
        offs_deg = list(self.get_parameter("joint_offsets_deg").value)
        if len(offs_deg) != 7:
            offs_deg = [0.0]*7
        self.joint_offsets_rad = [_math.radians(d) for d in offs_deg]

        home_deg = list(self.get_parameter("home_pose_deg").value)
        if len(home_deg) != 7:
            home_deg = [0.0]*7
        self._home_sdk_rad = [_math.radians(d) for d in home_deg]

        self.joint_names = [f"{self.prefix}{i}" for i in range(1, 8)]
        self.get_logger().info(
            f"[{self.side}] start | ip={self.robot_ip} local_ip={self.local_ip} "
            f"ctrl={self.controller_name} dry_run={self.dry_run} "
            f"power_on_at_start={self.power_on_at_start}"
        )

        self.robot = None
        self.ec = {}
        self.powered_on = False
        self._power_lock = threading.Lock()

        if not self.dry_run:
            self._connect()  # 只连接，不上电
            if self.power_on_at_start:
                self._power_on()

        self.js_pub = self.create_publisher(JointState, "/joint_states_arms", 10)
        self.state_timer = self.create_timer(1.0 / self.state_pub_hz, self._publish_joint_state)

        self._goal_lock = threading.Lock()
        self._action = ActionServer(
            self, FollowJointTrajectory,
            f"{self.controller_name}/follow_joint_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info(
            f"[{self.side}] action server: {self.controller_name}/follow_joint_trajectory"
        )

    def _connect(self):
        self.get_logger().info(f"[{self.side}] connecting...")
        self.robot = xCoreSDK_python.xMateErProRobot(self.robot_ip)
        if self.local_ip:
            self.robot.connectToRobot(self.robot_ip, self.local_ip)
        else:
            self.robot.connectToRobot(self.ec)
        time.sleep(0.2)
        info = self.robot.robotInfo(self.ec)
        self.get_logger().info(
            f"[{self.side}] connected. joint_num={info.joint_num} (NOT powered yet)"
        )

    def _power_on(self):
        with self._power_lock:
            if self.powered_on:
                return
            self.get_logger().warn(f"[{self.side}] POWERING ON ROBOT (servo enable)")
            self.robot.setOperateMode(xCoreSDK_python.OperateMode.automatic, self.ec)
            self.robot.setMotionControlMode(
                xCoreSDK_python.MotionControlMode.NrtCommandMode, self.ec)
            self.robot.setPowerState(True, self.ec)
            self.robot.clearServoAlarm(self.ec)
            self.robot.setDefaultZone(self.sdk_zone, self.ec)
            time.sleep(0.2)
            self.powered_on = True
            self.get_logger().info(f"[{self.side}] powered on, ready to move")

    def _disconnect(self):
        try:
            if not self.dry_run and self.robot is not None:
                self.robot.disconnectFromRobot(self.ec)
                self.get_logger().info(f"[{self.side}] disconnected")
        except Exception as e:
            self.get_logger().warn(f"[{self.side}] disconnect error: {e}")

    def _publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        try:
            if self.dry_run or self.robot is None:
                # 发 home (SDK->URDF after +offset),让 RViz 启动即显示真机起始姿态
                msg.position = [self._home_sdk_rad[i] + self.joint_offsets_rad[i] for i in range(7)]
            else:
                q = self.robot.jointPos(self.ec)
                msg.position = [float(q[i]) + self.joint_offsets_rad[i] for i in range(7)]
        except Exception as e:
            self.get_logger().warn(f"[{self.side}] jointPos failed: {e}", throttle_duration_sec=2.0)
            return
        self.js_pub.publish(msg)

    def _goal_cb(self, _):
        if self._goal_lock.locked():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _):
        self.get_logger().warn(f"[{self.side}] cancel requested")
        if not self.dry_run and self.powered_on:
            try: self.robot.stop(self.ec)
            except Exception: pass
        return CancelResponse.ACCEPT

    def _reorder(self, traj_names, positions):
        idx = {n: i for i, n in enumerate(traj_names)}
        return [positions[idx[jn]] for jn in self.joint_names]

    def _execute_cb(self, gh):
        result = FollowJointTrajectory.Result()
        with self._goal_lock:
            traj = gh.request.trajectory
            tn = list(traj.joint_names)
            missing = [j for j in self.joint_names if j not in tn]
            if missing:
                self.get_logger().error(f"[{self.side}] missing joints: {missing}")
                result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                gh.abort()
                return result

            cmds = []
            for pt in traj.points:
                q = self._reorder(tn, list(pt.positions))
                # 反向映射：发到 SDK 时减去 offset
                q_sdk = [q[i] - self.joint_offsets_rad[i] for i in range(7)]
                cmds.append(xCoreSDK_python.MoveAbsJCommand(q_sdk, self.sdk_speed))

            if not cmds:
                self.get_logger().warn(f"[{self.side}] empty traj")
                gh.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            self.get_logger().info(f"[{self.side}] exec {len(cmds)} pts")

            if self.dry_run:
                total = traj.points[-1].time_from_start
                t_total = float(total.sec) + float(total.nanosec) * 1e-9
                t0 = time.time()
                while time.time() - t0 < t_total:
                    if gh.is_cancel_requested:
                        gh.canceled(); result.error_code = FollowJointTrajectory.Result.SUCCESSFUL; return result
                    time.sleep(0.05)
                gh.succeed(); result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result

            # Lazy power-on: 收到第一个 goal 时上电
            if not self.powered_on:
                self._power_on()

            try:
                self.robot.moveReset(self.ec)
                cid = xCoreSDK_python.PyString()
                self.robot.moveAppend(cmds, cid, self.ec)
                self.robot.moveStart(self.ec)
            except Exception as e:
                self.get_logger().error(f"[{self.side}] moveStart failed: {e}")
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                gh.abort(); return result

            t0 = time.time()
            fb = FollowJointTrajectory.Feedback()
            fb.joint_names = list(self.joint_names)
            while time.time() - t0 < 300.0:
                if gh.is_cancel_requested:
                    try: self.robot.stop(self.ec)
                    except Exception: pass
                    gh.canceled(); result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                try:
                    q = self.robot.jointPos(self.ec)
                    fb.actual.positions = [float(q[i]) + self.joint_offsets_rad[i] for i in range(7)]
                    gh.publish_feedback(fb)
                    st = self.robot.operationState(self.ec)
                    if st == xCoreSDK_python.OperationState.idle:
                        break
                except Exception:
                    pass
                time.sleep(0.05)
            else:
                self.get_logger().error(f"[{self.side}] timeout")
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                gh.abort(); return result

            gh.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result


def main(args=None):
    rclpy.init(args=args)
    node = ArmNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
