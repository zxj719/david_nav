#!/usr/bin/env python3
"""Direct arm goto test (no vision, no hand).

Takes a target flange pose in arm_base frame, computes joints via ikpy,
sends a JOINT goal to MoveGroup (collision-aware planning), executes.

Usage:
    ros2 run david_grasp goto_pose right 0.30 -0.40 0.10
       (position only; uses current orientation)

    ros2 run david_grasp goto_pose right 0.30 -0.40 0.10  0 0 0 1
       (position + orientation quat xyzw)

This bypasses B's topic entirely. Use to verify the IK + MoveGroup +
arm_node pipeline works for manually-chosen targets.
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, JointConstraint,
                             RobotState)
from moveit_msgs.srv import GetStateValidity

from ikpy.chain import Chain


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

MAX_JOINT_DELTA_RAD = 1.5    # ~86 deg; was 3.0 but 165deg j5 swings caused link5<->Link4 collision in transit
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


def build_chain(side):
    base = PLAN_FRAME[side]
    full = Chain.from_urdf_file(URDF_PATH, base_elements=[base])
    return Chain(name=side, links=full.links[:9],
                 active_links_mask=[False] + [True]*7 + [False])


class GotoPose(Node):
    def __init__(self, side):
        super().__init__('goto_pose_' + side)
        self.side = side
        self.group = GROUP[side]
        self.plan_frame = PLAN_FRAME[side]
        self.joint_names = [JOINT_PREFIX[side] + str(i) for i in range(1, 8)]
        self.chain = build_chain(side)
        self.current_joints = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.validity_client = self.create_client(GetStateValidity, '/check_state_validity')
        self.get_logger().info('GotoPose[%s] plan_frame=%s' % (side, self.plan_frame))

    def _js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.current_joints[n] = p

    def wait_ready(self, timeout=30.0):
        assert self.move_client.wait_for_server(timeout_sec=timeout), 'move_action missing'
        assert self.validity_client.wait_for_service(timeout_sec=timeout), 'check_state_validity missing'
        t0 = time.time()
        while not all(j in self.current_joints for j in self.joint_names) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        assert all(j in self.current_joints for j in self.joint_names), 'no /joint_states'
        self.get_logger().info('ready')

    def current_joints_rad(self):
        return np.array([self.current_joints[j] for j in self.joint_names])

    def current_flange_orient_quat(self):
        cur = self.current_joints_rad()
        q_in = [0.0]*len(self.chain.links)
        for i in range(7): q_in[i+1] = cur[i]
        fk = self.chain.forward_kinematics(q_in)
        # rotation matrix -> quaternion
        R = fk[:3, :3]
        tr = np.trace(R)
        if tr > 0:
            s = 0.5 / np.sqrt(tr + 1.0)
            qw = 0.25 / s
            qx = (R[2,1] - R[1,2]) * s
            qy = (R[0,2] - R[2,0]) * s
            qz = (R[1,0] - R[0,1]) * s
        else:
            if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
                qw = (R[2,1] - R[1,2]) / s; qx = 0.25 * s
                qy = (R[0,1] + R[1,0]) / s; qz = (R[0,2] + R[2,0]) / s
            elif R[1,1] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
                qw = (R[0,2] - R[2,0]) / s; qx = (R[0,1] + R[1,0]) / s
                qy = 0.25 * s; qz = (R[1,2] + R[2,1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
                qw = (R[1,0] - R[0,1]) / s; qx = (R[0,2] + R[2,0]) / s
                qy = (R[1,2] + R[2,1]) / s; qz = 0.25 * s
        return (qx, qy, qz, qw)

    def _normalize(self, sol, cur):
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

    def solve_ik(self, target_pose7, orient_free=False):
        cur = self.current_joints_rad()
        target_T = np.eye(4)
        target_T[:3, :3] = quat_to_R(target_pose7[3:7])
        target_T[:3, 3] = target_pose7[:3]

        # Seed strategy: many tight-near-current seeds + a few far ones
        # (tight seeds find smooth-near-current solutions; far seeds for branches)
        seeds = [cur]
        np.random.seed(42)
        # 10 tight seeds: cur +- 0.2 rad (~11deg/joint) -- find nearby solutions
        for _ in range(10):
            seeds.append(cur + np.random.uniform(-1, 1, 7) * 0.2)
        # 5 medium seeds: cur +- 0.5 rad (~29deg/joint) -- moderate diversity
        for _ in range(5):
            seeds.append(cur + np.random.uniform(-1, 1, 7) * 0.5)
        # 3 wide seeds for branch exploration
        seeds.append(np.zeros(7))
        seeds.append(np.array([0.0, 1.4, 1.0, 1.0, 0.0, 0.0, 0.0]))
        seeds.append(np.array([-0.5, 1.2, 0.5, 1.5, -0.5, 0.0, 0.0]))

        candidates = []
        for s in seeds:
            try:
                seed_in = [0.0]*len(self.chain.links)
                for i in range(7): seed_in[i+1] = float(s[i])
                if orient_free:
                    raw = self.chain.inverse_kinematics(
                        target_position=target_T[:3, 3],
                        initial_position=seed_in,
                        orientation_mode=None)
                else:
                    raw = self.chain.inverse_kinematics_frame(target_T, initial_position=seed_in)
                sol = np.array([raw[i+1] for i in range(7)])
                fk = self.chain.forward_kinematics(raw)
                res = np.linalg.norm(fk[:3,3] - target_T[:3,3]) * 1000.0
                # orient_free: residual is position-only, no orientation penalty
            except Exception:
                continue
            sol = self._normalize(sol, cur)
            delta = sol - cur
            md = float(np.max(np.abs(delta)))
            if res > 20.0 or md > MAX_JOINT_DELTA_RAD:
                continue
            candidates.append((res, md, sol, delta))

        if not candidates:
            self.get_logger().error('  no candidates within limits')
            return None

        candidates.sort(key=lambda c: (c[0], c[1]))
        deg = lambda r: r * 180.0 / np.pi
        for i, (res, md, sol, delta) in enumerate(candidates):
            ok, reason = self._check_collision(sol)
            mi = int(np.argmax(np.abs(delta)))
            tag = 'OK' if ok else 'COLLIDE'
            self.get_logger().info(
                '  cand %d/%d  res=%.1fmm  max_delta=j%d %+.1fdeg  [%s]%s'
                % (i+1, len(candidates), res, mi+1, deg(delta[mi]), tag,
                   '' if ok else ' ('+reason+')'))
            if ok:
                self.get_logger().info('  accepted cand %d:' % (i+1))
                for j in range(7):
                    self.get_logger().info(
                        '    j%d: %+7.2f deg (cur %+7.2f, delta %+7.2f)'
                        % (j+1, deg(sol[j]), deg(cur[j]), deg(delta[j])))
                return sol

        self.get_logger().error('  all candidates collided')
        return None

    def goto(self, target_pose7, orient_free=False):
        mode = '(position only, wrist free)' if orient_free else ''
        self.get_logger().info(
            'target flange in %s: pos=(%.3f,%.3f,%.3f) quat=(%.3f,%.3f,%.3f,%.3f) %s'
            % (self.plan_frame, *target_pose7, mode))

        target_joints = self.solve_ik(target_pose7, orient_free=orient_free)
        if target_joints is None:
            return False

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

        self.get_logger().info('  sending to MoveGroup...')
        fut = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('  rejected')
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=120.0)
        if not rf.done():
            self.get_logger().error('  exec timeout')
            return False
        code = rf.result().result.error_code.val
        if code == 1:
            self.get_logger().info('  SUCCESS')
            return True
        self.get_logger().error('  FAILED MoveItErrorCode=%d' % code)
        return False


def main():
    if len(sys.argv) < 5:
        print('usage: goto_pose <left|right> x y z [qx qy qz qw]')
        print('  position-only example: goto_pose right 0.30 -0.30 0.10')
        sys.exit(1)
    side = sys.argv[1]
    assert side in ('left', 'right')
    x = float(sys.argv[2]); y = float(sys.argv[3]); z = float(sys.argv[4])

    rclpy.init()
    node = GotoPose(side)
    try:
        node.wait_ready(30.0)
        orient_free = (len(sys.argv) < 9)
        if not orient_free:
            quat = (float(sys.argv[5]), float(sys.argv[6]),
                    float(sys.argv[7]), float(sys.argv[8]))
        else:
            quat = node.current_flange_orient_quat()
            node.get_logger().info(
                'no quat given -> position-only IK; placeholder quat=(%.3f,%.3f,%.3f,%.3f)' % quat)
        ok = node.goto((x, y, z) + quat, orient_free=orient_free)
        node.get_logger().info('RESULT: ' + ('SUCCESS' if ok else 'FAIL'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()