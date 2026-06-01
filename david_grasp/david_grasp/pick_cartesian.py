#!/usr/bin/env python3
"""Stage 4: Cartesian pose-goal grasp via B's latched topics — ikpy + MoveGroup hybrid.

Why hybrid (5.28 reroute):
- MoveGroup's built-in KDL IK is too weak for this 7-DOF redundant arm; pose
  constraints fail with "Unable to sample any valid states for goal tree".
- B's pure-ikpy executor solves IK but skips self-collision checking.
- This module: ikpy solves IK (robust), then sends a JOINT goal to MoveGroup
  (full OMPL planning + collision check). Same pattern as RViz "drag + Plan".

Pipeline:
    /grasp/right/{target,hover}_pose (touchpoint, latched, arm_base frame)
      -> touchpoint -> flange (subtract R*tool_offset)
      -> ikpy IK to 7-DOF joints (trim chain to flan_link)
      -> safety gate (per-joint delta limit)
      -> MoveGroup joint goal (OMPL plan + collision check)
      -> right_arm_controller FollowJointTrajectory exec

Usage:
    ros2 run david_grasp pick_cartesian               # right + full
    ros2 run david_grasp pick_cartesian right hover   # hover-only safety test
"""
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint, RobotState)
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState
from david_hand_msgs.srv import SetHandPreset
from david_grasp.pick_at_pose import GraspPipeline

from ikpy.chain import Chain


# ---- frames / groups -------------------------------------------------------
PLAN_FRAME = {
    'left':  'AR5-5_08L-W4C4A6-ZY2_base',
    'right': 'AR5-5_08R-W4C4A6-ZY2_base',
}
GROUP = {'left': 'left_arm', 'right': 'right_arm'}
JOINT_PREFIX = {
    'left':  'AR5-5_08L-W4C4A6-ZY2_joint_',
    'right': 'AR5-5_08R-W4C4A6-ZY2_joint_',
}

URDF_PATH = '/home/agx/ros2_ws/src/david_description/urdf/david.urdf'

# ---- B's published topics (TRANSIENT_LOCAL latched) ------------------------
TARGET_POSE_TOPIC = '/grasp/right/target_pose'
HOVER_POSE_TOPIC  = '/grasp/right/hover_pose'

LATCHED_QOS = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

# ---- TOOL0 -> TOUCHPOINT (B's handeye_result_right_armbase.yaml) -----------
TOOL0_TO_TOUCHPOINT_LOCAL = (0.10478, 0.02165, 0.24291)  # m

# ---- IK / planning ---------------------------------------------------------
MAX_JOINT_DELTA_RAD = 3.0    # ~172 deg; reject ikpy solutions with bigger swing
JOINT_TOL_RAD = 0.005        # ~0.3 deg per-joint tolerance for MoveGroup
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


def touchpoint_to_flange(pose_msg, offset_local=TOOL0_TO_TOUCHPOINT_LOCAL):
    """PoseStamped (touchpoint pos + tool0 quat) -> 7-tuple flange pose."""
    px = pose_msg.pose.position.x
    py = pose_msg.pose.position.y
    pz = pose_msg.pose.position.z
    qx = pose_msg.pose.orientation.x
    qy = pose_msg.pose.orientation.y
    qz = pose_msg.pose.orientation.z
    qw = pose_msg.pose.orientation.w
    R = quat_to_R((qx, qy, qz, qw))
    off = np.array(offset_local)
    d = R @ off
    return (px - d[0], py - d[1], pz - d[2], qx, qy, qz, qw)


def build_arm_chain(side):
    """ikpy chain from <side>_arm_base trimmed to flan_link (drop hand chain)."""
    base = {'left':  'AR5-5_08L-W4C4A6-ZY2_base',
            'right': 'AR5-5_08R-W4C4A6-ZY2_base'}[side]
    full = Chain.from_urdf_file(URDF_PATH, base_elements=[base])
    # Keep [0] base link, [1-7] arm joints, [8] flan_joint  -- drop hand
    chain = Chain(
        name=f'{side}_arm',
        links=full.links[:9],
        active_links_mask=[False] + [True]*7 + [False],
    )
    return chain


class GraspCartesian(GraspPipeline):
    """Hybrid: ikpy IK + MoveGroup joint planning (collision-aware)."""

    def __init__(self, side='right'):
        super().__init__(side)
        self.group = GROUP[side]
        self.plan_frame = PLAN_FRAME[side]
        self.joint_prefix = JOINT_PREFIX[side]
        self.joint_names_full = [self.joint_prefix + str(i) for i in range(1, 8)]
        self.chain = build_arm_chain(side)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity')

        self._target_pose = None       # converted to flange (7-tuple)
        self._hover_pose = None
        self.create_subscription(
            PoseStamped, TARGET_POSE_TOPIC, self._target_cb, LATCHED_QOS)
        self.create_subscription(
            PoseStamped, HOVER_POSE_TOPIC, self._hover_cb, LATCHED_QOS)

        self.get_logger().info(
            'GraspCartesian[%s] group=%s plan_frame=%s'
            % (side, self.group, self.plan_frame))
        self.get_logger().info(
            '  subscribed (TRANSIENT_LOCAL): %s, %s'
            % (TARGET_POSE_TOPIC, HOVER_POSE_TOPIC))
        self.get_logger().info(
            '  tool0->touchpoint = %.3f, %.3f, %.3f (m)' % TOOL0_TO_TOUCHPOINT_LOCAL)
        self.get_logger().info(
            '  ikpy chain: %d links (7 active joints)' % len(self.chain.links))

    # ---- interfaces --------------------------------------------------------
    def wait_ready(self, timeout=30.0):
        super().wait_ready(timeout)
        assert self.move_client.wait_for_server(timeout_sec=timeout), \
            'move_action (MoveGroup) server missing'
        assert self.validity_client.wait_for_service(timeout_sec=timeout), \
            'check_state_validity service missing'
        self.get_logger().info('MoveGroup action + state-validity service ready')

    # ---- callbacks: convert touchpoint -> flange ---------------------------
    def _target_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.plan_frame:
            self.get_logger().warn(
                'target_pose frame_id=%s, expected %s. Ignoring.'
                % (msg.header.frame_id, self.plan_frame))
            return
        self._target_pose = touchpoint_to_flange(msg)
        self.get_logger().info(
            'target  touch=(%.3f,%.3f,%.3f) -> flange=(%.3f,%.3f,%.3f)'
            % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
               self._target_pose[0], self._target_pose[1], self._target_pose[2]))

    def _hover_cb(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.plan_frame:
            self.get_logger().warn(
                'hover_pose frame_id=%s, expected %s. Ignoring.'
                % (msg.header.frame_id, self.plan_frame))
            return
        self._hover_pose = touchpoint_to_flange(msg)
        self.get_logger().info(
            'hover   touch=(%.3f,%.3f,%.3f) -> flange=(%.3f,%.3f,%.3f)'
            % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
               self._hover_pose[0], self._hover_pose[1], self._hover_pose[2]))

    def get_poses(self, timeout=15.0):
        t0 = time.time()
        while (self._target_pose is None or self._hover_pose is None) \
                and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._target_pose is None:
            self.get_logger().error('timeout waiting for ' + TARGET_POSE_TOPIC)
            return None, None
        if self._hover_pose is None:
            self.get_logger().error('timeout waiting for ' + HOVER_POSE_TOPIC)
            return None, None
        return self._hover_pose, self._target_pose

    # ---- ikpy IK + safety gate --------------------------------------------
    def _current_joints_rad(self):
        """Read 7 right/left arm joints from /joint_states (URDF rad)."""
        q = []
        for jn in self.joint_names_full:
            if jn not in self.current_joints:
                return None
            q.append(self.current_joints[jn])
        return np.array(q)

    def _normalize_joints(self, sol, cur):
        """Wrap each joint to within +-pi of current (take the short way)."""
        out = np.array(sol, dtype=float)
        for i in range(7):
            while out[i] - cur[i] > np.pi:
                out[i] -= 2 * np.pi
            while out[i] - cur[i] < -np.pi:
                out[i] += 2 * np.pi
        return out

    def _ik_one_seed(self, target_T, seed_joints):
        """Run ikpy once with given 7-vector seed; return (sol_joints, residual_mm)."""
        seed = [0.0] * len(self.chain.links)
        for i in range(7):
            seed[i + 1] = float(seed_joints[i])
        raw = self.chain.inverse_kinematics_frame(target_T, initial_position=seed)
        sol = np.array([raw[i + 1] for i in range(7)])
        fk = self.chain.forward_kinematics(raw)
        res_mm = np.linalg.norm(fk[:3, 3] - target_T[:3, 3]) * 1000.0
        return sol, res_mm

    def _check_collision(self, joints_7):
        """Call MoveIt /check_state_validity for the candidate joints. True = collision-free."""
        req = GetStateValidity.Request()
        req.group_name = self.group
        rs = RobotState()
        rs.joint_state = JointState()
        rs.joint_state.name = list(self.joint_names_full)
        rs.joint_state.position = [float(q) for q in joints_7]
        req.robot_state = rs
        fut = self.validity_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done() or fut.result() is None:
            return False, 'service timeout'
        ok = fut.result().valid
        if ok:
            return True, ''
        contacts = fut.result().contacts
        if contacts:
            c0 = contacts[0]
            return False, f'{c0.contact_body_1} <-> {c0.contact_body_2}'
        return False, 'invalid (no contact info)'

    def solve_ik(self, flange_pose7, label=''):
        """Multi-seed ikpy IK with joint normalization. Returns 7-vec or None."""
        cur = self._current_joints_rad()
        if cur is None:
            self.get_logger().error('  ikpy[%s]: missing current joint state' % label)
            return None

        target = np.eye(4)
        target[:3, :3] = quat_to_R(flange_pose7[3:7])
        target[:3, 3]  = flange_pose7[:3]

        # Multiple seeds: current, zero, "extended" pose, random
        seeds = [
            cur,                                        # current
            np.zeros(7),                                # zero
            np.array([0.0, 1.4, 1.0, 1.0, 0.0, 0.0, 0.0]),   # extended-fwd-ish
            np.array([-0.5, 1.2, 0.5, 1.5, -0.5, 0.0, 0.0]), # tucked-fwd
        ]
        # add 12 random restarts within +-pi of current (more chances to find collision-free)
        np.random.seed(42)  # deterministic across runs for repeatability
        for _ in range(12):
            seeds.append(cur + np.random.uniform(-np.pi, np.pi, 7) * 0.6)

        candidates = []
        for s in seeds:
            try:
                sol_raw, res_mm = self._ik_one_seed(target, s)
            except Exception as e:
                continue
            sol = self._normalize_joints(sol_raw, cur)
            delta = sol - cur
            max_d = float(np.max(np.abs(delta)))
            if res_mm > 20.0 or max_d > MAX_JOINT_DELTA_RAD:
                continue  # drop infeasible
            candidates.append((res_mm, max_d, sol, delta))

        if not candidates:
            self.get_logger().error(
                '  ikpy[%s]: no candidate solutions within residual/delta limits' % label)
            return None

        # Sort: lowest residual first, then lowest max delta
        candidates.sort(key=lambda c: (c[0], c[1]))

        deg = lambda r: r * 180.0 / np.pi
        # Try each candidate; pass MoveIt collision check
        for i, (res_mm, max_d, sol, delta) in enumerate(candidates):
            collision_free, reason = self._check_collision(sol)
            max_idx = int(np.argmax(np.abs(delta)))
            tag = 'OK' if collision_free else 'COLLIDE'
            self.get_logger().info(
                '  ikpy[%s] cand %d/%d  res=%.1fmm  max_delta=j%d %+.1fdeg  [%s]%s'
                % (label, i + 1, len(candidates), res_mm,
                   max_idx + 1, deg(delta[max_idx]), tag,
                   '' if collision_free else ' (' + reason + ')'))
            if collision_free:
                self.get_logger().info('  ikpy[%s]: accepted cand %d' % (label, i + 1))
                for j in range(7):
                    self.get_logger().info(
                        '    joint_%d: %+7.2f deg  (cur %+7.2f, delta %+7.2f)'
                        % (j + 1, deg(sol[j]), deg(cur[j]), deg(delta[j])))
                return sol

        self.get_logger().error(
            '  ikpy[%s]: %d candidates tried, ALL collided' % (label, len(candidates)))
        return None

    # ---- MoveGroup joint goal (collision-checked planning) -----------------
    def _joint_constraints(self, joints_rad):
        c = Constraints()
        for jn, q in zip(self.joint_names_full, joints_rad):
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(q)
            jc.tolerance_above = JOINT_TOL_RAD
            jc.tolerance_below = JOINT_TOL_RAD
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    def move_to_pose_via_ikpy(self, flange_pose7, label=''):
        target_joints = self.solve_ik(flange_pose7, label=label)
        if target_joints is None:
            return False

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.group
        req.goal_constraints = [self._joint_constraints(target_joints)]
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
            self.get_logger().error('  move[%s]: goal rejected' % label)
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=120.0)
        if not rf.done():
            self.get_logger().error('  move[%s]: timeout' % label)
            return False
        code = rf.result().result.error_code.val
        if code != 1:
            self.get_logger().error(
                '  move[%s]: failed, MoveItErrorCode=%d' % (label, code))
            return False
        self.get_logger().info('  move[%s]: ok' % label)
        return True

    # ---- pipeline ----------------------------------------------------------
    def hover_only(self):
        """Safe: just plan + go to hover (no descent, no grasp)."""
        hover, _ = self.get_poses(timeout=15.0)
        if hover is None:
            return False
        self.get_logger().info('=== HOVER-ONLY START ===')
        ok = self.move_to_pose_via_ikpy(hover, 'hover')
        if ok:
            self.get_logger().info(
                '=== HOVER-ONLY DONE. Measure offset before unlocking full grasp. ===')
        return ok

    def pick(self):
        hover, target = self.get_poses(timeout=15.0)
        if hover is None or target is None:
            return False

        self.get_logger().info('=== CARTESIAN PICK START ===')

        self.get_logger().info('[1/5] open hand (async)')
        req = SetHandPreset.Request()
        req.name = 'open'
        hand_fut = self.hand_preset_client.call_async(req)

        self.get_logger().info('[2/5] plan to hover')
        if not self.move_to_pose_via_ikpy(hover, 'hover'):
            return False

        if not (hand_fut.done() and hand_fut.result()
                and hand_fut.result().success):
            self.get_logger().warn('  hand open not confirmed (continuing)')

        self.get_logger().info('[3/5] plan to target (grasp)')
        if not self.move_to_pose_via_ikpy(target, 'grasp'):
            return False

        self.get_logger().info('[4/5] force-closed grasp')
        gripped = self.force_grasp()
        time.sleep(0.5)

        self.get_logger().info('[5/5] lift (back to hover)')
        if not self.move_to_pose_via_ikpy(hover, 'lift'):
            return False

        if gripped:
            self.get_logger().info('=== CARTESIAN PICK DONE (gripped) ===')
        else:
            self.get_logger().warn(
                '=== CARTESIAN PICK DONE but grasp may have FAILED ===')
        return gripped


def main():
    side = sys.argv[1] if len(sys.argv) > 1 else 'right'
    mode = sys.argv[2] if len(sys.argv) > 2 else 'full'
    assert side in ('left', 'right'), 'usage: pick_cartesian [right|left] [full|hover]'
    assert mode in ('full', 'hover'), 'usage: pick_cartesian [right|left] [full|hover]'
    rclpy.init()
    node = GraspCartesian(side)
    try:
        node.wait_ready(30.0)
        ok = node.hover_only() if mode == 'hover' else node.pick()
        node.get_logger().info('RESULT: ' + ('SUCCESS' if ok else 'FAIL'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()