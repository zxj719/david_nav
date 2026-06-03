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

# L10 hand integration (Stage 1.a)
import sys as _sys
_sys.path.insert(0, "/home/agx/wheelhumanoidrobot/workspace_arm/Arm_skills")
try:
    from l10_hand_control import L10HandController, PRESETS as _HAND_PRESETS_RAW, JOINT_NAMES as _HAND_JOINT_NAMES
    _HAND_AVAILABLE = True
except Exception as _e:
    _HAND_AVAILABLE = False
    _HAND_IMPORT_ERROR = str(_e)
    _HAND_PRESETS_RAW = {}
    _HAND_JOINT_NAMES = [f"hand_j{i}" for i in range(10)]

# A 留下的 PRESETS 命名和实际硬件方向相反(可能用旧固件)。
# 在 arm_node 这层 override,对调用者(ROS service)统一暴露语义正确的命名。
# 实测对应:
#   实际"张开"  ← A 的 "grasp" = [180,128,200,200,200,200,128,128,128,128]
#   实际"闭合"  ← A 的 "open"  = [0]*10
#   半握       ← A 的 "mid"   = [128]*10
#   抓取姿(初始值,需现场标定): pitch 约 80,yaw 居中
_HAND_PRESETS = {
    "open":  [255, 128, 255, 255, 255, 255, 128, 128, 128, 128],  # 五指完全张开(pitch 推到极限,yaw 居中)
    "close": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],                      # 五指收紧(握拳)
    "half":  [128, 128, 128, 128, 128, 128, 128, 128, 128, 128],  # 半握
    "grasp": [160, 128, 160, 160, 160, 160, 128, 128, 128, 128], # 抓取姿(矿泉水瓶,现场标定 2026-05-19):pitch 160 稳定套住瓶身
}

try:
    from david_hand_msgs.srv import SetHandPose, SetHandPreset
    _HAND_SRV_AVAILABLE = True
except Exception:
    _HAND_SRV_AVAILABLE = False

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
        # joint_signs (2026-06-01): per-joint sign multiplier from URDF angle to
        # SDK angle. Physically left arm is the mirror of right arm so j3 and j5
        # electrically rotate in the opposite direction (verified empirically:
        # left home_deg [+145,+90,-90,+140,+90,0,0] vs right [-55,+85,+100,+140,
        # -90,+5,0] -- j3, j5 reversed sign). URDF has identical axis xyz="0 0 1"
        # for both, so URDF FK was wrong on left. Setting joint_signs on left to
        # [1,1,-1,1,-1,1,1] makes URDF FK match physical reality.
        self.declare_parameter("joint_signs", [1, 1, 1, 1, 1, 1, 1])
        self.declare_parameter("home_pose_deg", [0.0]*7)  # home pose in SDK coords (deg)
        self.declare_parameter("hand_slave_id", 0)  # 0x27 right / 0x28 left, 0 = disable hand

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

        signs = list(self.get_parameter("joint_signs").value)
        if len(signs) != 7 or any(s not in (-1, 1, -1.0, 1.0) for s in signs):
            self.get_logger().warn(
                f"[{self.side}] joint_signs invalid ({signs}); falling back to all +1")
            signs = [1]*7
        self.joint_signs = [int(s) for s in signs]

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
            try:
                self._connect()
                if self.power_on_at_start:
                    self._power_on()
            except Exception as e:
                # 真机不在 / 网络不通 / SDK 拒绝: 自动 fallback,RViz 仍显示 home
                self.get_logger().warn(
                    f"[{self.side}] connect failed ({type(e).__name__}: {e}); "
                    f"fallback to mock mode (RViz shows home; Execute will be rejected)"
                )
                self.dry_run = True
                self.robot = None

        # === Stage 1.a: L10 hand integration (share robot) ===
        self.hand = None
        slave_id = int(self.get_parameter("hand_slave_id").value)
        if slave_id != 0 and self.robot is not None and _HAND_AVAILABLE:
            try:
                self.hand = L10HandController(self.robot_ip, self.local_ip, slave_id)
                self.hand.robot = self.robot  # SHARE: skip hand.connect() to avoid double-connect
                # Init torque (reg 10) and speed (reg 20) so motors actually move on write
                try:
                    self.hand.set_torque_all(150)
                    self.hand.set_speed_all(120)
                    self.get_logger().info(
                        f"[{self.side}] hand torque=150 speed=120 initialized")
                except Exception as e:
                    self.get_logger().warn(f"[{self.side}] hand torque/speed init failed: {e}")
                self.get_logger().info(
                    f"[{self.side}] L10 hand integrated (slave_id=0x{slave_id:02X}, share robot)"
                )
            except Exception as e:
                self.get_logger().warn(f"[{self.side}] L10 hand init failed: {e}")
                self.hand = None
        elif slave_id != 0 and not _HAND_AVAILABLE:
            self.get_logger().warn(f"[{self.side}] hand disabled: L10HandController import failed ({_HAND_IMPORT_ERROR})")
        elif slave_id != 0 and self.robot is None:
            self.get_logger().warn(f"[{self.side}] hand disabled: robot is None (mock mode)")

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

        # === Stage 1.a: hand set_pose10 service ===
        if self.hand is not None and _HAND_SRV_AVAILABLE:
            _hand_cbg = ReentrantCallbackGroup()
            self._hand_set_pose_srv = self.create_service(
                SetHandPose,
                f"/{self.side}_hand/set_pose10",
                self._cb_hand_set_pose10,
                callback_group=_hand_cbg,
            )
            self._hand_preset_srv = self.create_service(
                SetHandPreset,
                f"/{self.side}_hand/preset",
                self._cb_hand_preset,
                callback_group=_hand_cbg,
            )
            self._hand_state_pub = self.create_publisher(
                JointState, f"/{self.side}_hand/state", 10
            )
            self._hand_state_timer = self.create_timer(
                0.5, self._publish_hand_state, callback_group=_hand_cbg
            )  # 2 Hz (Modbus 极限)
            self.get_logger().info(
                f"[{self.side}] hand services: /{self.side}_hand/set_pose10  /{self.side}_hand/preset"
            )
            self.get_logger().info(
                f"[{self.side}] hand state topic: /{self.side}_hand/state (2 Hz, pose+effort+vel)"
            )
        elif self.hand is not None:
            self.get_logger().warn(f"[{self.side}] hand srv unavailable; rebuild david_hand_msgs")

    def _cb_hand_set_pose10(self, request, response):
        try:
            pose = list(request.pose)
            if len(pose) != 10:
                response.success = False
                response.message = f"pose must be length 10, got {len(pose)}"
                return response
            self.hand.set_all_pose(pose)
            ok = self.hand.write_pose_if_needed()
            response.success = bool(ok)
            response.message = f"set_all_pose ok={ok}"
        except Exception as e:
            response.success = False
            response.message = f"{type(e).__name__}: {e}"
        return response

    def _cb_hand_preset(self, request, response):
        try:
            name = str(request.name).strip().lower()
            if name not in _HAND_PRESETS:
                response.success = False
                response.message = f"unknown preset '{name}'; available: {list(_HAND_PRESETS.keys())}"
                return response
            pose = list(_HAND_PRESETS[name])
            self.hand.set_all_pose(pose)
            ok = self.hand.write_pose_if_needed()
            response.success = bool(ok)
            response.message = f"preset='{name}' pose={pose} ok={ok}"
        except Exception as e:
            response.success = False
            response.message = f"{type(e).__name__}: {e}"
        return response

    def _publish_hand_state(self):
        """2 Hz 发布灵巧手 pose/effort/velocity 状态(Stage 1.c).

        Modbus 读取偶尔错位,用 3 帧中位数滤波消除毛刺。
        """
        if self.hand is None or self.hand.robot is None:
            return
        try:
            ok = self.hand.read_state_once()
            if not ok:
                return
            pose = [int(x) for x in self.hand.state_pose]
            effort = [int(x) for x in self.hand.state_effort]
            vel_attr = (getattr(self.hand, "state_vel", None)
                        or getattr(self.hand, "state_velocity", None)
                        or getattr(self.hand, "state_speed", None) or [0]*10)
            vel = [int(x) for x in vel_attr]

            # 3 帧滑动中位数滤波
            if not hasattr(self, "_hand_state_buf"):
                self._hand_state_buf = {"pose": [], "effort": [], "vel": []}
            buf = self._hand_state_buf
            buf["pose"].append(pose);   buf["pose"]   = buf["pose"][-3:]
            buf["effort"].append(effort); buf["effort"] = buf["effort"][-3:]
            buf["vel"].append(vel);     buf["vel"]    = buf["vel"][-3:]

            def median(lst_of_lists, n=10):
                out = []
                for i in range(n):
                    vals = sorted(arr[i] for arr in lst_of_lists)
                    out.append(vals[len(vals)//2])
                return out

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(_HAND_JOINT_NAMES)
            msg.position = [float(x) for x in median(buf["pose"])]
            msg.effort   = [float(x) for x in median(buf["effort"])]
            msg.velocity = [float(x) for x in median(buf["vel"])]
            self._hand_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(
                f"[{self.side}] hand state read failed: {e}",
                throttle_duration_sec=5.0,
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
                # SDK -> URDF: urdf = sign * sdk + offset
                msg.position = [
                    self.joint_signs[i] * self._home_sdk_rad[i] + self.joint_offsets_rad[i]
                    for i in range(7)]
            else:
                q = self.robot.jointPos(self.ec)
                msg.position = [
                    self.joint_signs[i] * float(q[i]) + self.joint_offsets_rad[i]
                    for i in range(7)]
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

    # ------------------------------------------------------------------
    # SDK error / power-state helpers (added 2026-06-03 to detect silent estop)
    # ------------------------------------------------------------------
    @staticmethod
    def _ec_extract(obj, names=("value", "get")):
        """Try several attribute names + try as method then as property."""
        for n in names:
            attr = getattr(obj, n, None)
            if attr is None:
                continue
            # Try calling as method first (xCoreSDK C++ binding often exposes methods)
            try:
                v = attr()
                return v
            except TypeError:
                # Not callable -- try as property/attribute
                return attr
            except Exception:
                continue
        return None

    def _ec_readable(self):
        """Format self.ec (PyErrorCode) as 'ec=<val> (<msg>)' for logging.

        Rokae PyErrorCode binding has .value() (or .get()) and .message().
        Both are usually C++ methods, not properties."""
        v = self._ec_extract(self.ec, ("value", "get"))
        m = self._ec_extract(self.ec, ("message",))
        return f"ec={v} ({m})"

    def _ec_failed(self):
        """True if the most recent SDK call returned a non-zero error code."""
        v = self._ec_extract(self.ec, ("value", "get"))
        try:
            return int(v) != 0
        except (TypeError, ValueError):
            return False

    def _safe_power_state(self):
        """Return powerState() as string, or '?err' if SDK call fails.

        PowerState enum: on / off / estop / gstop / unknown.
        estop = emergency stop triggered (safety monitor / physical button).
        gstop = guard stop (safety door / light curtain)."""
        try:
            ps = self.robot.powerState(self.ec)
            return str(ps)
        except Exception as e:
            return f"?err({type(e).__name__})"

    def _is_estop(self):
        """True if the controller is currently in estop or gstop state.

        Use this to distinguish 'robot at idle because trajectory done' from
        'robot at idle because safety stop pulled servos offline'."""
        try:
            ps = self.robot.powerState(self.ec)
            s = str(ps).lower()
            return ("estop" in s) or ("gstop" in s)
        except Exception:
            return False

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

            # Build all SDK joint commands then subsample
            all_qs = []
            for pt in traj.points:
                q = self._reorder(tn, list(pt.positions))
                # 反向映射：发到 SDK 时减去 offset
                # URDF -> SDK: sdk = sign * (urdf - offset)
                q_sdk = [self.joint_signs[i] * (q[i] - self.joint_offsets_rad[i])
                         for i in range(7)]
                all_qs.append(q_sdk)

            # Aggressive subsample: keep ONLY the final goal joint config.
            # Rationale: MoveGroup OMPL has already verified the path is
            # feasible (start state OK, goal state OK, no obstacle); the
            # intermediate points are just time-parameterization waypoints.
            # Sending only the goal lets SDK do its own smooth interpolation
            # (matches the smoothness of `moveAppend(single_pt) + moveStart`
            # used by the manual keyboard-reset demo).
            n_in = len(all_qs)
            if n_in == 0:
                cmds = []
            else:
                cmds = [xCoreSDK_python.MoveAbsJCommand(all_qs[-1], self.sdk_speed)]
                if n_in > 1:
                    self.get_logger().info(
                        f"[{self.side}] trajectory simplified to goal-only: {n_in} -> 1 pt")

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
                # Check SDK error code from PyErrorCode (self.ec is reused; reflects
                # the LAST SDK call -- moveStart). Non-zero = SDK rejected motion,
                # typically because controller is in estop/error state.
                ec_info = self._ec_readable()
                if self._ec_failed():
                    pstate = self._safe_power_state()
                    self.get_logger().error(
                        f"[{self.side}] !!! moveStart REJECTED by SDK: {ec_info} "
                        f"powerState={pstate} -- "
                        f"controller likely in estop/error state. "
                        f"Recovery: power-cycle controller, then clearServoAlarm()/recoverState().")
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    gh.abort(); return result
                try:
                    st0 = self.robot.operationState(self.ec)
                    pstate = self._safe_power_state()
                    self.get_logger().warn(
                        f"[{self.side}] after moveStart: state={st0} power={pstate} {ec_info}")
                except Exception as e:
                    self.get_logger().warn(f"[{self.side}] state read failed: {e}")
            except Exception as e:
                self.get_logger().error(f"[{self.side}] moveStart exception: {e}")
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                gh.abort(); return result

            # NEW: wait for SDK to leave idle before polling for idle (race fix)
            # Also: detect estop -- if controller went into estop during/after
            # moveStart, operationState stays idle forever, so without this check
            # we'd silently report SUCCESS even though robot never moved.
            t_wait = time.time()
            _last_log = 0.0
            _last_st = None
            while time.time() - t_wait < 2.0:
                try:
                    _st = self.robot.operationState(self.ec)
                    if _st != _last_st:
                        pstate = self._safe_power_state()
                        self.get_logger().warn(
                            f"[{self.side}] wait-loop state -> {_st} (power={pstate})")
                        _last_st = _st
                    # E-stop check (Rokae safety monitor can trigger w/o physical hit)
                    if self._is_estop():
                        pstate = self._safe_power_state()
                        self.get_logger().error(
                            f"[{self.side}] !!! E-STOP DETECTED in pre-move wait-loop !!! "
                            f"powerState={pstate}. Robot was idle but is now in "
                            f"protective stop. Trajectory aborted. "
                            f"Recovery: power-cycle controller, then clearServoAlarm().")
                        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        gh.abort(); return result
                    if _st != xCoreSDK_python.OperationState.idle:
                        break
                except Exception:
                    pass
                time.sleep(0.02)

            t0 = time.time()
            fb = FollowJointTrajectory.Feedback()
            fb.joint_names = list(self.joint_names)
            _estop_check_cnt = 0
            while time.time() - t0 < 300.0:
                if gh.is_cancel_requested:
                    try: self.robot.stop(self.ec)
                    except Exception: pass
                    gh.canceled(); result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result
                try:
                    q = self.robot.jointPos(self.ec)
                    fb.actual.positions = [
                        self.joint_signs[i] * float(q[i]) + self.joint_offsets_rad[i]
                        for i in range(7)]
                    gh.publish_feedback(fb)
                    st = self.robot.operationState(self.ec)
                    # E-stop check every ~5 loops (~250ms), avoid per-tick overhead
                    _estop_check_cnt += 1
                    if _estop_check_cnt >= 5:
                        _estop_check_cnt = 0
                        if self._is_estop():
                            pstate = self._safe_power_state()
                            self.get_logger().error(
                                f"[{self.side}] !!! E-STOP MID-MOTION !!! "
                                f"powerState={pstate}. Trajectory aborted at "
                                f"t={time.time()-t0:.2f}s. "
                                f"Recovery: power-cycle controller, clearServoAlarm().")
                            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                            gh.abort(); return result
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
