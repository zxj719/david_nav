#!/usr/bin/env python3
"""Grasp test using OUR URDF transform (bypass B's potentially-biased calibration).

Pipeline:
    1. Subscribe to /yolo/bottle_center (geometry_msgs/PointStamped from B's YOLO).
       2026-06-01: switched from /grasp/right/detection (std_msgs/String JSON).
    2. tf2 transform msg.header.frame_id (expected camera_color_optical_frame)
       -> arm_base using OUR URDF camera_link (2026-05-28 user-measured xyz/pitch)
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
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
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


CAMERA_FRAME = 'camera_color_optical_frame'
URDF_PATH = '/home/agx/ros2_ws/src/david_description/urdf/david.urdf'

# Per-side configuration. The head-mounted camera (mounted on body Link4) is
# shared by both arms via static TF, so the same /yolo/bottle_center topic
# feeds both pipelines.
# 2026-06-01 (afternoon): added left-arm support after right thumb got
# damaged on a table strike.
_SIDE_CONFIGS = {
    'right': {
        'plan_frame':   'AR5-5_08R-W4C4A6-ZY2_base',
        'ee_link':      'AR5-5_08R-W4C4A6-ZY2_flan_link',
        'group':        'right_arm',
        'joint_prefix': 'AR5-5_08R-W4C4A6-ZY2_joint_',
        'hand_preset':  '/right_hand/preset',
        'hand_pose10':  '/right_hand/set_pose10',
        # palm faces robot LEFT, tool0_Z = +arm_X (forward). 90 deg about Y.
        'base_quat':    (0.0,  0.7071067811865475, 0.0, 0.7071067811865475),
        # arm_base axes (right): +X=forward, +Y=up, +Z=right
        'tcp_offset':   (0.04, -0.03,  0.02),
        'lift_offset':  (0.02,  0.07,  0.02),
    },
    'left': {
        'plan_frame':   'AR5-5_08L-W4C4A6-ZY2_base',
        'ee_link':      'AR5-5_08L-W4C4A6-ZY2_flan_link',
        'group':        'left_arm',
        'joint_prefix': 'AR5-5_08L-W4C4A6-ZY2_joint_',
        'hand_preset':  '/left_hand/preset',
        'hand_pose10':  '/left_hand/set_pose10',
        # base_quat: same +Y 90 deg as right arm. User confirmed palm faces
        # right (= toward the bottle, midline side) -- correct, do NOT flip.
        'base_quat':    (0.0,  0.7071067811865475, 0.0, 0.7071067811865475),
        # 2026-06-02 FINAL calibration (verified 5-6 successful grasps in a row):
        #   tcp.x=+0.06  hand 6 cm in front of bottle (pre-grasp clearance)
        #   tcp.y=+0.04  hand 4 cm below bottle center (down 1cm vs previous)
        #   tcp.z=+0.13  fingertip ~2-3 cm to bottle's robot left
        # URDF<->physical Z offset is large (~10 cm) and NOT linear because
        # left hand mesh is not mirrored in URDF. Real fix is URDF surgery
        # (TODO 6.3). For now these empirical offsets give reliable grasps.
        # NOTE: bottle is NOT in MoveIt planning scene yet, so OMPL paths
        # may sweep through bottle volume on aggressive joint reconfigs.
        # Knocked the bottle over once during 2026-06-02 testing; final
        # offsets above let OMPL find clean direct paths. TODO P1: add
        # bottle collision cylinder to planning scene for full safety.
        'tcp_offset':   (0.06,  0.04,  0.13),
        'lift_offset':  (0.04, -0.06,  0.13),
    },
}

# 2026-06-01: B switched from JSON String to PointStamped on a new topic.
# We trust msg.header.frame_id; CAMERA_FRAME above is kept as defensive fallback
# in case publisher ships an empty frame_id.
DETECTION_TOPIC = '/yolo/bottle_center'
# 2026-06-01 (afternoon): B's publisher uses BEST_EFFORT + VOLATILE QoS.
# A RELIABLE subscriber (default) is STRICTER than a BEST_EFFORT publisher,
# so ROS refuses the match and drops every message silently
# (we hit this: "incompatible QoS ... Last incompatible policy: RELIABILITY").
# Match the publisher exactly.
DETECTION_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# Tool offset from OUR URDF right_tcp_joint (xyz=0 0 0.22, rpy=0 0 0).
# Replaces B's (0.0946, 0.0187, 0.2396); 2026-05-29 OUR URDF verified <1.5cm.
TOOL0_TO_TOUCHPOINT_LOCAL = (0.0, 0.0, 0.22)  # m

# Wrist roll about local tool0_Z to fine-tune palm tilt (degrees).
# (positive = CCW viewed from arm_base toward flange)
WRIST_ROLL_DEG = 0.0

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
_roll_rad = _math.radians(WRIST_ROLL_DEG)
_ROLL_QUAT = (0.0, 0.0, _math.sin(_roll_rad/2), _math.cos(_roll_rad/2))

def grip_quat_for(side):
    base = _SIDE_CONFIGS[side]['base_quat']
    return _quat_mul(base, _ROLL_QUAT)

# Custom "open" hand pose (10-vec): all fingers extended + Thumb_Yaw=0 so the
# thumb sits AWAY from palm, leaving room for the bottle to enter the grip.
# Order: Thumb_Pitch, Thumb_Yaw, Index_Pitch, Middle_Pitch, Ring_Pitch,
#        Little_Pitch, Index_Yaw, Ring_Yaw, Little_Yaw, Thumb_Roll
# pitch 0 = fully closed (curled), pitch 255 = fully extended (open).
OPEN_HAND_POSE = [255, 0, 255, 255, 255, 255, 128, 128, 128, 128]

# 2026-06-02: SetHandPreset('grasp') was too loose -- 4 fingers stopped at
# pitch=160 (effort 1-5, fingers barely contacting bottle). Calibrated grip
# pitch from 5.26 testing is 157; using 155 here for slightly firmer hold.
# All 5 pitch joints (Thumb + 4 fingers) close to 155 to wrap around bottle.
TIGHT_CLOSE_POSE = [155, 128, 155, 155, 155, 155, 127, 128, 160, 128]

# IK / safety
# 2026-06-01: kept at 2.0 (115 deg). Even without sign flip, left arm home j1=+145
# to a bottle in front needs ~105 deg, > 1.5 rad. 2.0 is the minimum needed.
MAX_JOINT_DELTA_RAD = 2.0
JOINT_TOL_RAD = 0.005
# 6.1: cut OMPL budget. single-pose grasp doesn't need long search.
PLAN_ATTEMPTS = 3   # was 10
PLAN_TIME = 1.5     # was 5.0 s
VEL_SCALE = 0.4     # was 0.2 (~halves execution time)
ACC_SCALE = 0.4     # was 0.2

# 6.1: time the hand needs to physically close after the service returns.
# Used by the grasp pipeline between close_hand and lift. Was 0.8 s before.
CLOSE_SETTLE_SEC = 0.4

# 2026-06-01: workspace pre-check threshold on bottle.z in the arm's base.
# For BOTH arms, +Z in that arm's base frame points to that arm's OUTSIDE
# (right arm: +Z = right; left arm: +Z = left). A bottle with Z below this
# threshold sits at or beyond the body midline -- on the other arm's side
# -- so reaching it would require crossing the body, which triggered the
# left-arm self-collision against Link4 on 2026-06-01 (MoveIt's mesh-based
# collision check missed it). Refuse such targets early instead of trusting
# the IK to produce a clean solution.
MIN_BOTTLE_Z_IN_ARM_BASE = 0.05  # m


def quat_to_R(q):
    qx, qy, qz, qw = q
    n = qx*qx + qy*qy + qz*qz + qw*qw
    s = 2.0/n if n > 0 else 0.0
    return np.array([
        [1-(qy*qy+qz*qz)*s, (qx*qy-qz*qw)*s, (qx*qz+qy*qw)*s],
        [(qx*qy+qz*qw)*s, 1-(qx*qx+qz*qz)*s, (qy*qz-qx*qw)*s],
        [(qx*qz-qy*qw)*s, (qy*qz+qx*qw)*s, 1-(qx*qx+qy*qy)*s],
    ])


def build_chain(side):
    plan_frame = _SIDE_CONFIGS[side]['plan_frame']
    full = Chain.from_urdf_file(URDF_PATH, base_elements=[plan_frame])
    return Chain(name=side, links=full.links[:9],
                 active_links_mask=[False] + [True]*7 + [False])


class PickFromCam(Node):
    def __init__(self, side='right'):
        if side not in _SIDE_CONFIGS:
            raise ValueError(f'side must be left or right, got {side!r}')
        cfg = _SIDE_CONFIGS[side]
        super().__init__(f'pick_from_cam_{side}')
        self.side = side
        self.plan_frame = cfg['plan_frame']
        self.ee_link = cfg['ee_link']
        self.group = cfg['group']
        self.joint_prefix = cfg['joint_prefix']
        self.grip_quat = grip_quat_for(side)
        self.tcp_offset = cfg['tcp_offset']
        self.lift_offset = cfg['lift_offset']

        self.joint_names = [self.joint_prefix + str(i) for i in range(1, 8)]
        self.current_joints = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.chain = build_chain(side)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity')
        self.hand_preset_client = self.create_client(SetHandPreset, cfg['hand_preset'])
        self.hand_pose_client = self.create_client(SetHandPose, cfg['hand_pose10'])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._last_point = None  # geometry_msgs/PointStamped from B's YOLO
        # 6.1: freeze bottle position once at the start of a grasp pipeline
        # and reuse across stages, so stage 1 (hover) and stage 3 (lift) go
        # to the SAME world point. Without this, YOLO/depth jitter + a new
        # frame arriving between stages causes 1-3 cm drift -> bottle slips
        # after closing. Also keep last successful hover IK solution to
        # warm-start lift IK.
        self._cached_bottle_arm = None
        self._cached_hover_joints = None
        self.create_subscription(
            PointStamped, DETECTION_TOPIC, self._det_cb, DETECTION_QOS)

        palm_dir = 'LEFT' if side == 'right' else 'RIGHT'
        self.get_logger().info(f'PickFromCam[{side}] ready')
        self.get_logger().info(
            f'  plan_frame={self.plan_frame}  group={self.group}')
        self.get_logger().info(
            f'  hand preset={cfg["hand_preset"]}  pose10={cfg["hand_pose10"]}')
        self.get_logger().info(
            '  using OUR URDF for camera->arm_base TF (camera_link on body Link4)')
        self.get_logger().info(
            '  tool0->touchpoint offset = %.4f, %.4f, %.4f m' % TOOL0_TO_TOUCHPOINT_LOCAL)
        self.get_logger().info(
            f'  GRIP_QUAT (palm {palm_dir} + {WRIST_ROLL_DEG:+.1f} deg roll) = '
            + ', '.join(f'{q:+.3f}' for q in self.grip_quat))
        self.get_logger().info(
            f'  tcp_offset={self.tcp_offset}  lift_offset={self.lift_offset}')

    # ---- subscription -----------------------------------------------------
    def _js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.current_joints[n] = p

    def _det_cb(self, msg):
        # B's YOLO publisher ships geometry_msgs/PointStamped on /yolo/bottle_center.
        # header.frame_id is expected to be camera_color_optical_frame; we read
        # it back later via msg.header.frame_id (with a fallback to CAMERA_FRAME).
        self._last_point = msg

    def wait_ready(self, timeout=30.0):
        assert self.move_client.wait_for_server(timeout_sec=timeout), \
            'move_action missing'
        assert self.validity_client.wait_for_service(timeout_sec=timeout), \
            'check_state_validity missing'
        if not self.hand_preset_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('right_hand preset service not up; hand will not be opened')
        t0 = time.time()
        while (not all(j in self.current_joints for j in self.joint_names)
               or self._last_point is None) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        assert all(j in self.current_joints for j in self.joint_names), 'no /joint_states'
        assert self._last_point is not None, f'no PointStamped on {DETECTION_TOPIC}'
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
        req.group_name = self.group
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
    def cam_to_arm_base(self, pt_in):
        """Transform a PointStamped (from B's YOLO /yolo/bottle_center) into
        self.plan_frame via tf2. Trusts msg.header.frame_id; falls back to
        CAMERA_FRAME if publisher ships empty frame_id.

        Uses stamp=0 (latest TF) instead of pt_in.header.stamp to avoid
        clock-skew ExtrapolationException between B's vision host and AGX."""
        import rclpy.time as _rtime
        src_frame = pt_in.header.frame_id or CAMERA_FRAME
        # Wait until the TF chain is actually available
        t0 = time.time()
        while time.time() - t0 < 3.0:
            try:
                if self.tf_buffer.can_transform(
                        self.plan_frame, src_frame,
                        _rtime.Time(),
                        timeout=Duration(seconds=0.2)):
                    break
            except Exception:
                pass
            rclpy.spin_once(self, timeout_sec=0.1)
        else:
            self.get_logger().error(
                f'TF {src_frame} -> {self.plan_frame} not available after 3 s')
            return None

        pt = PointStamped()
        pt.header.frame_id = src_frame
        pt.header.stamp = _rtime.Time().to_msg()  # 0 = latest TF
        pt.point.x = float(pt_in.point.x)
        pt.point.y = float(pt_in.point.y)
        pt.point.z = float(pt_in.point.z)
        try:
            pt_arm = self.tf_buffer.transform(
                pt, self.plan_frame, timeout=Duration(seconds=1.5))
            return (pt_arm.point.x, pt_arm.point.y, pt_arm.point.z)
        except Exception as e:
            self.get_logger().error(
                f'TF {src_frame} -> {self.plan_frame} failed: {e}')
            return None

    # ---- ikpy IK + collision filter (same as goto_pose) -------------------
    def _ik_one_mode(self, flange_xyz, flange_quat, mode, label, warm_seed=None):
        cur = self.cur_joints_rad()
        target_pos = np.array(flange_xyz)
        R_target = quat_to_R(flange_quat)

        # 6.1: cut seed budget 49 -> ~13. Single-pose grasp has a narrow IK
        # manifold; ~12 seeds saturate ikpy (full mode ~50 ms each).
        # `warm_seed` (set by caller for lift) is the joint solution from the
        # previous similar pose -- gives near-zero-cost IK when the new target
        # is only ~10 cm away.
        seeds = [cur]
        if warm_seed is not None:
            seeds.append(np.array(warm_seed))
        np.random.seed(42)
        for _ in range(6):
            seeds.append(cur + np.random.uniform(-1, 1, 7) * 0.2)
        # 2026-06-01: add j3/j5 sign-flipped cur and other diverse seeds so
        # the optimizer can find solutions when sign_signs flip moves cur to
        # a region 180 deg away (left arm URDF axis bug). Cheap insurance.
        cur_flip35 = cur.copy()
        cur_flip35[2] *= -1  # j3
        cur_flip35[4] *= -1  # j5
        seeds.append(cur_flip35)
        seeds.append(np.zeros(7))
        seeds.append(np.array([0.0, 1.4, 1.0, 1.0, 0.0, 0.0, 0.0]))
        seeds.append(np.array([-1.0, 1.0, 0.5, 1.5, 0.0, 0.0, 0.0]))
        seeds.append(np.array([0.5, 1.0, -0.5, 1.5, 0.0, 0.0, 0.0]))
        # mirrored (j3, j5 negated) versions of the hardcoded seeds
        seeds.append(np.array([0.0, 1.4, -1.0, 1.0, 0.0, 0.0, 0.0]))
        seeds.append(np.array([-1.0, 1.0, -0.5, 1.5, 0.0, 0.0, 0.0]))
        seeds.append(np.array([0.5, 1.0, 0.5, 1.5, 0.0, 0.0, 0.0]))
        # 2026-06-01: empirically-known left-arm IK solution from pre-sign-flip
        # success (SDK frame). Translated to URDF frame by negating j3, j5.
        # Approx pose: arm reaching forward toward midline-front bottle area.
        seeds.append(np.array([1.25, 1.22, 0.66, 1.36, -0.86, -0.20, 0.34]))

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
            # 2026-06-01: WRIST safety. j6 (idx 5) and j7 (idx 6) are wrist
            # twist joints. ikpy can pick mathematically valid solutions
            # where j7 swings 47+ deg, which sends the L10 hand end-effector
            # sweeping near link5 and the torso -- URDF simplified STL mesh
            # misses this self-collision but physical contact happens.
            # Verified on 2026-06-01 left-arm self-collision incident.
            if abs(delta[5]) > 0.7 or abs(delta[6]) > 0.7:
                continue
            # full mode: BOTH tool0_Z and tool0_X axes within 5 deg of target.
            # !! DO NOT RELAX THIS THRESHOLD FOR SPEED !! 2026-06-01: tried
            # widening to 15 deg to make `full` hit more often -- this lets IK
            # return solutions where the palm meets the bottle tilted up to
            # 15 deg, so fingers strike the bottle/table at an angle instead of
            # closing cleanly. This directly contributed to the right-hand
            # thumb damage on 2026-06-01 (Thumb_Yaw jammed at 0, Pitch
            # partially seized). Keep 5 deg or tighter. If you need speed,
            # cut IK seeds or skip `full` entirely -- never widen the angle.
            if mode == 'full' and (ori_z_deg > 5.0 or ori_x_deg > 5.0):
                continue
            # z_only mode: tool0_Z within 10 deg AND tool0_X within 15 deg
            # (no more j7 hack since j7 axis is link6_X, not tool0_Z)
            if mode == 'z_only' and (ori_z_deg > 10.0 or ori_x_deg > 15.0):
                continue
            cands.append((res, md, sol, delta))
        cands.sort(key=lambda c: (c[0], c[1]))
        return cands

    def solve_ik_for_flange(self, flange_xyz, flange_quat, label='', warm_seed=None):
        cur = self.cur_joints_rad()
        deg = lambda r: r * 180.0 / np.pi
        for mode in ('full', 'z_only', 'position'):
            cands = self._ik_one_mode(flange_xyz, flange_quat, mode, label, warm_seed=warm_seed)
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
        req.group_name = self.group
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
        # 2026-06-02: SetHandPreset('grasp') closes 4 fingers only to pitch=160
        # (effort 1-5, not actually contacting bottle -> loose grip). Use
        # set_pose10 with TIGHT_CLOSE_POSE (all 5 pitches = 155) for a firm
        # wrap around bottle.
        if not self.hand_pose_client.service_is_ready():
            self.get_logger().warn('  close_hand skipped: set_pose10 service not ready')
            return False
        req = SetHandPose.Request()
        req.pose = list(TIGHT_CLOSE_POSE)
        fut = self.hand_pose_client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not fut.done() or not fut.result() or not fut.result().success:
            self.get_logger().warn('  close_hand: did not confirm success')
            return False
        self.get_logger().info(f'  hand: TIGHT CLOSE = {TIGHT_CLOSE_POSE}')
        return True

    # ---- pipeline ---------------------------------------------------------
    def _get_bottle_arm(self, use_cache, label):
        """Return bottle (x,y,z) in arm_base. If use_cache=True and we have a
        cached value from a previous fresh detection in THIS grasp session,
        reuse it (this is what fixes multi-stage drift). Otherwise read the
        latest detection JSON, run tf2, and update the cache."""
        if use_cache and self._cached_bottle_arm is not None:
            b = self._cached_bottle_arm
            self.get_logger().info(
                f'  [{label}] using CACHED bottle arm_base '
                f'({b[0]:.3f}, {b[1]:.3f}, {b[2]:.3f}) m')
            return b
        pt = self._last_point
        if pt is None:
            self.get_logger().error('no PointStamped received yet'); return None
        src_frame = pt.header.frame_id or CAMERA_FRAME
        self.get_logger().info(
            f"  [{label}] fresh P_cam (frame={src_frame}): "
            f"({pt.point.x:.3f}, {pt.point.y:.3f}, {pt.point.z:.3f}) m")
        # Sanity check: ROS convention is meters; any coord > 5 m suggests
        # B's publisher is in mm and needs fixing on their side.
        if abs(pt.point.x) > 5.0 or abs(pt.point.y) > 5.0 or abs(pt.point.z) > 5.0:
            self.get_logger().warn(
                f"  [{label}] P_cam coord > 5 m -- verify B publishes in METERS not mm")
        b = self.cam_to_arm_base(pt)
        if b is None:
            return None
        self.get_logger().info(
            f'  [{label}] fresh bottle in arm_base (OUR URDF) '
            f'({b[0]:.3f}, {b[1]:.3f}, {b[2]:.3f}) m')
        # 2026-06-02: workspace pre-check DISABLED by user request.
        # The check was defensive (block bottle in midline region to prevent
        # left arm from crossing body and hitting Link4). But after tcp_offset
        # was calibrated to (0.03, 0.03, 0.14) the URDF/physical Z offset is
        # large -- URDF-computed bottle.z near 0 corresponds to a physical
        # position the left arm can reach from its outside without crossing
        # the body. The check now blocks valid grasps. Real fix: URDF surgery
        # (left hand mesh mirroring + tcp_joint offset re-calibration).
        # if abs(b[2]) < MIN_BOTTLE_Z_IN_ARM_BASE:
        #     self.get_logger().error('WORKSPACE VIOLATION ...'); return None
        self.get_logger().info(f'  [{label}]   [CACHED for this session]')
        self._cached_bottle_arm = b
        return b

    def hover(self, mode='hover', fresh=True):
        # Open hand first so fingers don't collide with bottle while approaching.
        # Skip for 'lift' mode: fingers must stay closed to keep bottle.
        if mode != 'lift':
            self.open_hand()

        # 1. get bottle position in arm_base. For grasp's stage 3 (lift) we
        # pass fresh=False so we reuse stage-1's cached bottle -- this is the
        # multi-stage drift fix.
        bottle_arm = self._get_bottle_arm(use_cache=(not fresh), label=mode)
        if bottle_arm is None:
            return False

        # 2. offset by mode (per-side; approach mode was historically right-only
        # but is unused in the grasp pipeline -- reuses tcp_offset on left)
        if mode == 'lift':
            bottle_offset = self.lift_offset
        else:
            bottle_offset = self.tcp_offset
        self.get_logger().info(f'mode = {mode}  bottle->TCP offset = {bottle_offset}')

        # 3. apply tool offset: flange target = bottle + offset - R(grip_quat) * tool_offset
        # (so when flange is at flange_target, fingertip ends up at bottle+offset)
        grip_q = self.grip_quat
        R = quat_to_R(grip_q)
        offset = np.array(TOOL0_TO_TOUCHPOINT_LOCAL)
        d_arm = R @ offset
        tcp_target = (bottle_arm[0] + bottle_offset[0],
                      bottle_arm[1] + bottle_offset[1],
                      bottle_arm[2] + bottle_offset[2])
        flange_target = (tcp_target[0] - d_arm[0],
                         tcp_target[1] - d_arm[1],
                         tcp_target[2] - d_arm[2])
        self.get_logger().info(
            f'{mode} flange target: ({flange_target[0]:.3f}, '
            f'{flange_target[1]:.3f}, {flange_target[2]:.3f}) m')

        # 4. IK + MoveGroup. Warm-start lift IK from the hover solution
        # (target only ~10 cm away -> 1st seed nearly converges, IK ~0.2 s
        # instead of ~2 s).
        warm = self._cached_hover_joints if mode == 'lift' else None
        joints = self.solve_ik_for_flange(
            flange_target, grip_q, label=mode, warm_seed=warm)
        if joints is None:
            return False
        ok = self.move_to_joints(joints, label=mode)
        if ok and mode != 'lift':
            # Save for warm-starting the next stage's IK.
            self._cached_hover_joints = joints.copy()
        return ok


def main():
    import sys
    mode = 'hover'
    side = 'right'  # default; pass 'left' as a positional arg to switch
    for a in sys.argv[1:]:
        if a in ('left', 'right'):
            side = a
        elif a in ('hover', 'approach', 'grasp', 'lift'):
            mode = a
    rclpy.init()
    node = PickFromCam(side=side)
    try:
        node.wait_ready(30.0)
        t_start = time.time()
        if mode == 'grasp':
            # Fresh grasp session: clear any stale cache from a previous run.
            node._cached_bottle_arm = None
            node._cached_hover_joints = None
            node.get_logger().info('=== GRASP STAGE 1/3: open + move to grasp pose (FRESH detection) ===')
            t1 = time.time()
            ok = node.hover(mode='hover', fresh=True)
            node.get_logger().info(f'    stage 1 took {time.time()-t1:.2f}s')
            if ok:
                node.get_logger().info('=== GRASP STAGE 2/3: close fingers around bottle ===')
                t2 = time.time()
                ok = node.close_hand(preset='grasp')
                time.sleep(CLOSE_SETTLE_SEC)
                node.get_logger().info(f'    stage 2 took {time.time()-t2:.2f}s')
            if ok:
                node.get_logger().info('=== GRASP STAGE 3/3: lift bottle (CACHED bottle, warm IK) ===')
                t3 = time.time()
                ok = node.hover(mode='lift', fresh=False)
                node.get_logger().info(f'    stage 3 took {time.time()-t3:.2f}s')
        else:
            ok = node.hover(mode=mode)
        node.get_logger().info(
            f'RESULT: {"SUCCESS" if ok else "FAIL"}  '
            f'(total {time.time()-t_start:.2f}s)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()