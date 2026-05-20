#!/usr/bin/env python3
"""Stage 3.a+3.b: joint-space grasp pipeline with force-closed grasping.

Flow: open hand -> pose1 pre-grasp -> pose2 grasp -> FORCE-CLOSE -> pose3 lift

Force-closed grasp (Stage 3.b / Q6): progressively close fingers, monitor
effort from /{side}_hand/state, stop when Index/Little Pitch effort exceeds
threshold (gripped). Solves the 'grasp too loose' problem of fixed preset.

Usage: ros2 run david_grasp pick_at_pose left
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from david_hand_msgs.srv import SetHandPreset, SetHandPose

ARM_JOINT_PREFIX = {
    'left':  'AR5-5_08L-W4C4A6-ZY2_joint_',
    'right': 'AR5-5_08R-W4C4A6-ZY2_joint_',
}

LEFT_POSES = {
    'pre_grasp': [1.0846, 1.0129, -1.0506, 1.2166, 0.9377, -0.0053, 0.0045],
    'grasp':     [1.0197, 1.2127, -0.9482, 1.2536, 1.0193, -0.1043, -0.0833],
    'lift':      [1.0053, 1.4471, -1.4913, 2.1144, 1.5703, -0.1043, -0.0833],
}

# force-closed grasp params (calibrated 2026-05-19)
PITCH_IDX = [0, 2, 3, 4, 5]          # Thumb/Index/Middle/Ring/Little pitch
DETECT_IDX = [2, 5]                  # Index_Pitch / Little_Pitch (most sensitive)
EFFORT_THRESHOLD = 50.0              # empty<=13, gripped 50-80
CLOSE_SEQUENCE = [200, 160, 130, 100, 80, 60, 40]   # progressive close steps


class GraspPipeline(Node):
    def __init__(self, side='left'):
        super().__init__('grasp_pipeline_' + side)
        self.side = side
        self.controller = side + '_arm_controller'
        self.joint_names = [ARM_JOINT_PREFIX[side] + str(i) for i in range(1, 8)]
        self.traj_client = ActionClient(
            self, FollowJointTrajectory,
            '/' + self.controller + '/follow_joint_trajectory')
        self.hand_preset_client = self.create_client(
            SetHandPreset, '/' + side + '_hand/preset')
        self.hand_pose_client = self.create_client(
            SetHandPose, '/' + side + '_hand/set_pose10')
        self.current_joints = {}
        self.latest_effort = None
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(
            JointState, '/' + side + '_hand/state', self._hand_state_cb, 10)
        self.get_logger().info('GraspPipeline[' + side + '] init')

    def _js_cb(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.current_joints[n] = p

    def _hand_state_cb(self, msg):
        if msg.effort:
            self.latest_effort = list(msg.effort)

    def wait_ready(self, timeout=30.0):
        assert self.traj_client.wait_for_server(timeout_sec=timeout), 'trajectory action missing'
        assert self.hand_preset_client.wait_for_service(timeout_sec=timeout), 'hand preset missing'
        assert self.hand_pose_client.wait_for_service(timeout_sec=timeout), 'hand set_pose10 missing'
        t0 = time.time()
        while not all(j in self.current_joints for j in self.joint_names) and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        assert all(j in self.current_joints for j in self.joint_names), 'no arm /joint_states'
        self.get_logger().info('interfaces ready')

    def move_to_joints(self, q, duration=4.0):
        q_now = [self.current_joints[j] for j in self.joint_names]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        p0 = JointTrajectoryPoint()
        p0.positions = q_now
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = list(q)
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration % 1) * 1e9)
        goal.trajectory.points = [p0, p1]
        fut = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('traj goal rejected')
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=duration + 10.0)
        if not rf.done():
            self.get_logger().error('traj exec timeout')
            return False
        return rf.result().result.error_code == 0

    def hand_preset(self, name):
        req = SetHandPreset.Request()
        req.name = name
        fut = self.hand_preset_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.done() and fut.result().success

    def hand_set_pose(self, pose10):
        req = SetHandPose.Request()
        req.pose = [int(v) for v in pose10]
        fut = self.hand_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        return fut.done() and fut.result().success

    def read_effort(self, timeout=3.0):
        self.latest_effort = None
        t0 = time.time()
        while self.latest_effort is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.latest_effort

    def force_grasp(self):
        self.get_logger().info('  force-grasp: progressive close, monitor effort')
        for pv in CLOSE_SEQUENCE:
            pose = [pv if i in PITCH_IDX else 128 for i in range(10)]
            self.hand_set_pose(pose)
            time.sleep(1.2)
            eff = self.read_effort()
            if eff is None:
                self.get_logger().warn('  no hand state; continue')
                continue
            detect = max(abs(eff[i]) for i in DETECT_IDX)
            self.get_logger().info('  pitch=%3d  Index/Little effort=%.0f' % (pv, detect))
            if detect > EFFORT_THRESHOLD:
                self.get_logger().info('  force-grasp: GRIPPED at pitch=%d effort=%.0f' % (pv, detect))
                return True
        self.get_logger().warn('  force-grasp: closed fully, effort below threshold (empty grasp?)')
        return False

    def pick(self):
        poses = LEFT_POSES
        self.get_logger().info('=== PICK START (joint-space + force grasp) ===')

        self.get_logger().info('[1/4] open hand (parallel with arm move)')
        self.hand_preset('open')

        self.get_logger().info('[2/4] move to pre-grasp')
        if not self.move_to_joints(poses['pre_grasp'], 4.0):
            return False
        self.get_logger().info('[3/4] move to grasp pose')
        if not self.move_to_joints(poses['grasp'], 3.0):
            return False

        self.get_logger().info('[4/4] force-closed grasp + lift')
        gripped = self.force_grasp()
        time.sleep(0.5)
        if not self.move_to_joints(poses['lift'], 3.0):
            return False

        if gripped:
            self.get_logger().info('=== PICK DONE (gripped) ===')
        else:
            self.get_logger().warn('=== PICK DONE but grasp may have FAILED (empty) ===')
        return gripped


def main():
    side = sys.argv[1] if len(sys.argv) > 1 else 'left'
    assert side == 'left', 'only left arm poses calibrated; use: pick_at_pose left'
    rclpy.init()
    node = GraspPipeline(side)
    try:
        node.wait_ready(30.0)
        ok = node.pick()
        node.get_logger().info('RESULT: ' + ('SUCCESS' if ok else 'FAIL'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
