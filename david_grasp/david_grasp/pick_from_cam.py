#!/usr/bin/env python3
"""Grasp test using OUR URDF transform (bypass B's potentially-biased calibration).

Pipeline:
    1. Subscribe to /grasp/right/detection (B's JSON), parse P_cam
       (raw camera 3D from pixel + depth)
    2. tf2 transform camera_color_optical_frame -> arm_base
       using OUR URDF camera_link (added 2026-05-28 with user-measured xyz/pitch)
    3. Apply tool0->touchpoint offset to get flange target
       (since hand is attached, fingertip is the actual end-effector)
    4. ikpy IK + JointConstraint MoveGroup goal + collision check
       (same pipeline as goto_pose.py)

Default orientation: B's calibrated "downward palm" quat -- so the
fingertip approaches bottle with gripper palm-down, ready to grasp.

Modes:
    hover (default): go to bottle + Z safety, do NOT descend
    full:            full grasp (descend + close hand + lift) -- TODO

Usage:
    ros2 run david_grasp pick_from_cam right          # hover only (safe)
    ros2 run david_grasp pick_from_cam right hover    # explicit
"""
import json
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint,
                             RobotState)
from moveit_msgs.srv import GetStateValidity
from david_hand_msgs.srv import SetHandPreset, SetHandPose

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401

from ikpy.chain import Chain


PLAN_FRAME = 'AR5-5_08R-W4C4A6-ZY2_base'
CAMERA_FRAME = 'camera_color_optical_frame'
EE_LINK = 'AR5-5_08R-W4C4A6-ZY2_flan_link'
GROUP = 'right_arm'
JOINT_PREFIX = 'AR5-5_08R-W4C4A6-ZY2_joint_'
URDF_PATH = '/home/agx/ros2_ws/src/david_description/urdf/david.urdf'

DETECTION_TOPIC = '/grasp/right/detection'
LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# Tool offset from OUR URDF right_tcp_joint (xyz=0 0 0.22, rpy=0 0 0).
# Replaces B's (0.0946, 0.0187, 0.2396); 2026-05-29 OUR URDF verified <1.5cm.
TOOL0_TO_TOUCHPOINT_LOCAL = (0.0, 0.0, 0.22)  # m

# Base orientation: palm faces robot LEFT, tool0_Z = +arm_X (forward).
# 90 deg about Y axis. Then apply WRIST_ROLL_DEG about local tool0_Z to
# fine-tune palm tilt (positive = CCW viewed from arm_base toward flange).
WRIST_ROLL_DEG = 0.0  # 2026-05-29: +5 still tilted up, drop to 0 (base orientation only)

def _quat_mul(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

import math as _math
_BASE_QUAT = (0.0, 0.7071067811865475, 0.0, 0.7071067811865475)
_roll_rad = _math.radians(WRIST_ROLL_DEG)
_ROLL_QUAT = (0.0, 0.0, _math.sin(_roll_rad/2), _math.cos(_roll_rad/2))
DEFAULT_GRIP_QUAT = _quat_mul(_BASE_QUAT, _ROLL_QUAT)

# TCP target offset from detected bottle center, in arm_base axes:
#   (+arm_X = forward, +arm_Y = up, +arm_Z = right)
# Empirically tuned 2026-05-29 from real-bot: palm needs 5cm forward,
# 7cm down, 3cm right of detected bottle center to actually grip.
TCP_OFFSET_FROM_BOTTLE = (0.04, -0.03, 0.02)  # final grasp pose: 4cm forward + 3cm down + 2cm right

# Approach (= actual grasp position): hover + forward 2cm + down 8cm.
# Palm pushes into bottle's right side, fingers wrap around.
APPROACH_OFFSET_FROM_BOTTLE = (0.05, -0.05, 0.02)  # forward 5cm, down 5cm, right 2cm (2cm closer than hover)

# After closing fingers around bottle, lift 10cm: same forward/right, raise Y.
LIFT_OFFSET_FROM_BOTTLE = (0.02, 0.07, 0.02)

# Custom "open" hand pose (10-vec): all fingers extended + Thumb_Yaw=0 so the
# thumb sits AWAY from palm, leaving room for the bottle to enter the grip.
# Order: Thumb_Pitch, Thumb_Yaw, Index_Pitch, Middle_Pitch, Ring_Pitch,
#        Little_Pitch, Index_Yaw, Ring_Yaw, Little_Yaw, Thumb_Roll
OPEN_HAND_POSE = [255, 0, 255, 255, 255, 255, 128, 128, 128, 128]

# IK / safety
MAX_JOINT_DELTA_RAD = 1.5
JOINT_TOL_RAD = 0.005
PLAN_ATTEMPTS = 10
PLAN_TIME = 5.0
VEL_SCALE = 0.2
ACC_SCALE = 0.2


def quat_to_R(q):
    qx, qy, qz, qw = q
    n = qx*qx + qy*qy + qz*qz + qw*qw
    s = 2.0/n if n > 0 else 0.0
    return np.array([
        [1-(qy*qy+qz*qz)*s, (qx*qy-qz*qw)*s, (qx*qz+qy*qw)*s],
        [(qx*qy+qz*qw)*s, 1-(qx*qx+qz*qz)*s, (qy*qz-qx*qw)*s],
        [(qx*qz-qy*qw)*s, (qy*qz+qx*qw)*s, 1-(qx*qx+qy*qy)*s],
    ])


def build_right_chain():
    full = Chain.from_urdf_file(URDF_PATH, base_elements=[PLAN_FRAME])
    return Chain(name='right', links=full.links[:9],
                 active_links_mask=[False] + [True]*7 + [False])


class PickFromCam(Node):
    def __init__(self):
        super().__init__('pick_from_cam_right')
        self.joint_names = [JOINT_PREFIX + str(i) for i in range(1, 8)]
        self.current_joints = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.chain = build_right_chain()
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity')
        self.hand_preset_client = self.create_client(SetHandPreset, '/right_hand/preset')
        self.hand_pose_client = self.create_client(SetHandPose, '/right_hand/set_pose10')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._last_detection = None
        self.create_subscription(
            String, DETECTION_TOPIC, self._det_cb, LATCHED_QOS)

        self.get_logger().info('PickFromCam[right] ready')
        self.get_logger().info(
            '  using OUR URDF for camera->arm_base TF (camera_link estimated 2026-05-28)')
        self.get_logger().info(
            '  tool0->touchpoint offset = %.4f, %.4f, %.4f m' % TOOL0_TO_TOUCHPOINT_LOCAL)
        self.get_logger().info(
            f'  GRIP_QUAT (palm LEFT + {WRIST_ROLL_DEG:+.1f} deg roll) = ' + ', '.join(f'{q:+.3f}' for q in DEFAULT_GRIP_QUAT))

    # ---- subscription -----------------------------------------------------
    def _js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.current_joints[n] = p

    def _det_cb(self, msg):
        try:
            self._last_detection = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'failed to parse detection JSON: {e}')

    def wait_ready(self, timeout=30.0):
        assert self.move_client.wait_for_server(timeout_sec=timeout), \
            'move_action missing'
        assert self.validity_client.wait_for_service(timeout_sec=timeout), \
            'check_state_validity missing'
        if not self.hand_preset_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('right_hand preset service not up; hand will not be opened')
        t0 = time.time()
        while (not all(j in self.current_joints for j in self.joint_names)
               or self._last_detection is None) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        assert all(j in self.current_joints for j in self.joint_names), 'no /joint_states'
        assert self._last_detection is not None, f'no detection on {DETECTION_TOPIC}'
        self.get_logger().info('ready')

    # ---- helpers ----------------------------------------------------------
    def cur_joints_rad(self):
        return np.array([self.current_joints[j] for j in self.joint_names])

    def _normalize_joints(self, sol, cur):
        out = np.array(sol, dtype=float)
        for i in range(7):
            while out[i] - cur[i] > np.pi:
                out[i] -= 2 * np.pi
            while out[i] - cur[i] < -np.pi:
                out[i] += 2 * np.pi
        return out

    def _check_collision(self, joints_7):
        req = GetStateValidity.Request()
        req.group_name = GROUP
        rs = RobotState()
        rs.joint_state = JointState()
        rs.joint_state.name = list(self.joint_names)
        rs.joint_state.position = [float(q) for q in joints_7]
        req.robot_state = rs
        fut = self.validity_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done() or fut.result() is None:
            return False, 'timeout'
        ok = fut.result().valid
        if ok: return True, ''
        c = fut.result().contacts
        return False, (f'{c[0].contact_body_1}<->{c[0].contact_body_2}' if c else 'in collision')

    # ---- core: P_cam -> arm_base via OUR URDF -----------------------------
    def cam_to_arm_base(self, P_cam_xyz):
        """Use tf2 (our URDF chain) to convert a point in camera_color_optical_frame
        to arm_base. Returns (x, y, z) tuple or None on TF failure.

        Uses stamp=0 (latest available) so we don't race with the live TF tree."""
        import rclpy.time as _rtime
        # Wait until the TF chain is actually available
        t0 = time.time()
        while time.time() - t0 < 3.0:
            try:
                if self.tf_buffer.can_transform(
                        PLAN_FRAME, CAMERA_FRAME,
                        _rtime.Time(),
                        timeout=Duration(seconds=0.2)):
                    break
            except Exception:
                pass
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error(
                f'TF {CAMERA_FRAME} -> {PLAN_FRAME} not available after 3 s')
            return None

        pt = PointStamped()
        pt.header.frame_id = CAMERA_FRAME
        pt.header.stamp = _rtime.Time().to_msg()  # 0 = latest
        pt.point.x = float(P_cam_xyz[0])
        pt.point.y = float(P_cam_xyz[1])
        pt.point.z = float(P_cam_xyz[2])
        try:
            pt_arm = self.tf_buffer.transform(
                pt, PLAN_FRAME, timeout=Duration(seconds=1.5))
            return (pt_arm.point.x, pt_arm.point.y, pt_arm.point.z)
        except Exception as e:
            self.get_logger().error(f'TF {CAMERA_FRAME} -> {PLAN_FRAME} failed: {e}')
            return None

    # ---- ikpy IK + collision filter (same as goto_pose) -------------------
    def _ik_one_mode(self, flange_xyz, flange_quat, mode, label):
        cur = self.cur_joints_rad()
        target_pos = np.array(flange_xyz)
        R_target = quat_to_R(flange_quat)

        seeds = [cur]
        np.random.seed(42)
        for _ in range(30):
            seeds.append(cur + np.random.uniform(-1, 1, 7) * 0.2)
        for _ in range(15):
            seeds.append(cur + np.random.uniform(-1, 1, 7) * 0.5)
        seeds.append(np.zeros(7))
        seeds.append(np.array([0.0, 1.4, 1.0, 1.0, 0.0, 0.0, 0.0]))
        seeds.append(np.array([-1.0, 1.0, 0.5, 1.5, 0.0, 0.0, 0.0]))
        seeds.append(np.array([0.5, 1.0, -0.5, 1.5, 0.0, 0.0, 0.0]))

        cands = []
        for s in seeds:
            try:
                seed_in = [0.0]*len(self.chain.links)
                for i in range(7): seed_in[i+1] = float(s[i])
                if mode == 'full':
                    target_T = np.eye(4)
                    target_T[:3, :3] = R_target
                    target_T[:3, 3]  = target_pos
                    raw = self.chain.inverse_kinematics_frame(
                        target_T, initial_position=seed_in)
                elif mode == 'z_only':
                    target_z = R_target[:, 2]
                    raw = self.chain.inverse_kinematics(
                        target_position=target_pos,
                        target_orientation=target_z,
                        orientation_mode='Z',
                        initial_position=seed_in)
                else:
                    raw = self.chain.inverse_kinematics(
                        target_position=target_pos,
                        initial_position=seed_in)
                sol = np.array([raw[i+1] for i in range(7)])
                fk = self.chain.forward_kinematics(raw)
                res = np.linalg.norm(fk[:3, 3] - target_pos) * 1000.0
                target_z_dir = R_target[:, 2]
                target_x_dir = R_target[:, 0]
                fk_z_dir = fk[:3, 2]
                fk_x_dir = fk[:3, 0]
                cos_z = float(np.clip(np.dot(target_z_dir, fk_z_dir), -1.0, 1.0))
                ori_z_deg = float(np.degrees(np.arccos(cos_z)))
                cos_x = float(np.clip(np.dot(target_x_dir, fk_x_dir), -1.0, 1.0))
                ori_x_deg = float(np.degrees(np.arccos(cos_x)))
            except Exception:
                continue
            sol = self._normalize_joints(sol, cur)
            delta = sol - cur
            md = float(np.max(np.abs(delta)))
            if res > 30.0 or md > MAX_JOINT_DELTA_RAD:
                continue
            # full mode: BOTH tool0_Z and tool0_X axes within 5 deg of target
            if mode == 'full' and (ori_z_deg > 5.0 or ori_x_deg > 5.0):
                continue
            # z_only mode: tool0_Z within 10 deg AND tool0_X within 15 deg
            # (no more j7 hack since j7 axis is link6_X, not tool0_Z)
            if mode == 'z_only' and (ori_z_deg > 10.0 or ori_x_deg > 15.0):
                continue
            cands.append((res, md, sol, delta))
        cands.sort(key=lambda c: (c[0], c[1]))
        return cands

    def solve_ik_for_flange(self, flange_xyz, flange_quat, label=''):
        cur = self.cur_joints_rad()
        deg = lambda r: r * 180.0 / np.pi
        for mode in ('full', 'z_only', 'position'):
            cands = self._ik_one_mode(flange_xyz, flange_quat, mode, label)
            if not cands:
                self.get_logger().warn(
                    f'  [{label}] {mode}: 0 IK candidates within limits, falling back')
                continue
            self.get_logger().info(
                f'  [{label}] {mode}: {len(cands)} IK candidates')
            for i, (res, md, sol, delta) in enumerate(cands):
                ok, reason = self._check_collision(sol)
                mi = int(np.argmax(np.abs(delta)))
                tag = 'OK' if ok else 'COLLIDE'
                self.get_logger().info(
                    f'    cand {i+1}/{len(cands)} res={res:.1f}mm '
                    f'max_d=j{mi+1} {deg(delta[mi]):+.1f}deg [{tag}]'
                    + ('' if ok else f' ({reason})'))
                if ok:
                    self.get_logger().info(f'  [{label}] accepted ({mode}) cand {i+1}:')
                    for j in range(7):
                        self.get_logger().info(
                            f'    j{j+1}: {deg(sol[j]):+7.2f} deg '
                            f'(cur {deg(cur[j]):+7.2f}, delta {deg(delta[j]):+7.2f})')
                    return sol
            self.get_logger().warn(f'  [{label}] {mode}: all candidates collided, falling back')
        self.get_logger().error(f'  [{label}] IK fully failed (full + z_only + position)')
        return None

    def move_to_joints(self, target_joints, label=''):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = GROUP
        c = Constraints()
        for jn, q in zip(self.joint_names, target_joints):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(q)
            jc.tolerance_above = JOINT_TOL_RAD
            jc.tolerance_below = JOINT_TOL_RAD
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        req.num_planning_attempts = PLAN_ATTEMPTS
        req.allowed_planning_time = PLAN_TIME
        req.max_velocity_scaling_factor = VEL_SCALE
        req.max_acceleration_scaling_factor = ACC_SCALE
        goal.request = req
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False

        fut = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'  [{label}] rejected'); return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=120.0)
        if not rf.done():
            self.get_logger().error(f'  [{label}] exec timeout'); return False
        code = rf.result().result.error_code.val
        if code == 1:
            self.get_logger().info(f'  [{label}] SUCCESS'); return True
        self.get_logger().error(f'  [{label}] FAIL MoveItErrorCode={code}'); return False

    # ---- hand control -----------------------------------------------------
    def open_hand(self, timeout=10.0):
        # Use custom 10-vec OPEN_HAND_POSE (Thumb_Yaw=0 keeps thumb away from palm).
        if not self.hand_pose_client.service_is_ready():
            self.get_logger().warn('  open_hand skipped: /right_hand/set_pose10 not ready')
            return False
        req = SetHandPose.Request()
        req.pose = list(OPEN_HAND_POSE)
        fut = self.hand_pose_client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not fut.done() or not fut.result() or not fut.result().success:
            self.get_logger().warn('  open_hand: did not confirm success')
            return False
        self.get_logger().info(f'  hand: OPEN (custom pose, Thumb_Yaw=0) = {OPEN_HAND_POSE}')
        return True

    def close_hand(self, preset='grasp', timeout=5.0):
        if not self.hand_preset_client.service_is_ready():
            self.get_logger().warn('  close_hand skipped: service not ready')
            return False
        req = SetHandPreset.Request()
        req.name = preset
        fut = self.hand_preset_client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not fut.done() or not fut.result() or not fut.result().success:
            self.get_logger().warn(f'  close_hand({preset}): did not confirm success')
            return False
        self.get_logger().info(f'  hand: {preset.upper()}')
        return True

    # ---- pipeline ---------------------------------------------------------
    def hover(self, mode='hover'):
        # Open hand first so fingers don't collide with bottle while approaching.
        # Skip for 'lift' mode: fingers must stay closed to keep bottle.
        if mode != 'lift':
            self.open_hand()
        # 1. get P_cam from latest detection
        d = self._last_detection
        P_cam = d.get('P_cam')
        if not P_cam:
            self.get_logger().error('no P_cam in detection'); return False
        if mode == 'approach':
            bottle_offset = APPROACH_OFFSET_FROM_BOTTLE
        elif mode == 'lift':
            bottle_offset = LIFT_OFFSET_FROM_BOTTLE
        else:
            bottle_offset = TCP_OFFSET_FROM_BOTTLE
        self.get_logger().info(f'mode = {mode}  bottle->TCP offset = {bottle_offset}')
        self.get_logger().info(
            f'P_cam (camera frame): ({P_cam[0]:.3f}, {P_cam[1]:.3f}, {P_cam[2]:.3f}) m')
        # B's interpretation, for comparison
        if 'P_pred_arm_base' in d:
            bp = d['P_pred_arm_base']
            self.get_logger().info(
                f"B's P_pred_arm_base: ({bp[0]:.3f}, {bp[1]:.3f}, {bp[2]:.3f}) m")

        # 2. tf2 transform to arm_base via OUR URDF
        bottle_arm = self.cam_to_arm_base(P_cam)
        if bottle_arm is None:
            return False
        self.get_logger().info(
            f'bottle in arm_base (OUR URDF): ({bottle_arm[0]:.3f}, '
            f'{bottle_arm[1]:.3f}, {bottle_arm[2]:.3f}) m')

        # 3. apply tool offset: flange target = bottle - R(grip_quat) * tool_offset
        # (so when flange is at flange_target, fingertip ends up at bottle)
        grip_q = DEFAULT_GRIP_QUAT
        R = quat_to_R(grip_q)
        offset = np.array(TOOL0_TO_TOUCHPOINT_LOCAL)
        d_arm = R @ offset
        self.get_logger().info(
            f'tool offset in arm_base: ({d_arm[0]:.3f}, {d_arm[1]:.3f}, {d_arm[2]:.3f}) m')

        # Hover target = bottle position + Z safety in arm_base.z direction
        hover_bottle = (bottle_arm[0] + bottle_offset[0],
                        bottle_arm[1] + bottle_offset[1],
                        bottle_arm[2] + bottle_offset[2])
        flange_target = (hover_bottle[0] - d_arm[0],
                         hover_bottle[1] - d_arm[1],
                         hover_bottle[2] - d_arm[2])
        self.get_logger().info(
            f'hover flange target: ({flange_target[0]:.3f}, '
            f'{flange_target[1]:.3f}, {flange_target[2]:.3f}) m')

        # 4. IK + MoveGroup
        joints = self.solve_ik_for_flange(flange_target, grip_q, label='hover')
        if joints is None:
            return False
        return self.move_to_joints(joints, label='hover')


def main():
    import sys
    mode = 'hover'
    for a in sys.argv[1:]:
        if a in ('hover', 'approach', 'grasp', 'lift'):
            mode = a
    rclpy.init()
    node = PickFromCam()
    try:
        node.wait_ready(30.0)
        if mode == 'grasp':
            node.get_logger().info('=== GRASP STAGE 1/3: open + move to grasp pose ===')
            ok = node.hover(mode='hover')
            if ok:
                node.get_logger().info('=== GRASP STAGE 2/3: close fingers around bottle ===')
                ok = node.close_hand(preset='grasp')
                time.sleep(0.8)
            if ok:
                node.get_logger().info('=== GRASP STAGE 3/3: lift bottle 10cm ===')
                ok = node.hover(mode='lift')
        else:
            ok = node.hover(mode=mode)
        node.get_logger().info('RESULT: ' + ('SUCCESS' if ok else 'FAIL'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()