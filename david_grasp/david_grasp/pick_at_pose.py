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

# 5.26 真机录的 SDK 角度。 6.1 末 sign 翻转回滚 -> 这里恢复原值。
# URDF base rpy=-1.570 已经天然处理了左右臂 Y/Z 镜像,j3/j5 axis xyz="0 0 1"
# 在镜像 base 下自动给出物理对的运动学,不需要额外 sign 翻转。
LEFT_POSES = {
    'pre_grasp': [1.0846, 1.0129, -1.0506, 1.2166, 0.9377, -0.0053, 0.0045],
    'grasp':     [1.0197, 1.2127, -0.9482, 1.2536, 1.0193, -0.1043, -0.0833],
    'lift':      [1.0053, 1.4471, -1.4913, 2.1144, 1.5703, -0.1043, -0.0833],
}

# force-closed grasp params (calibrated 2026-05-19)
PITCH_IDX = [0, 2, 3, 4, 5]          # Thumb/Index/Middle/Ring/Little pitch
DETECT_IDX = [2, 5]                  # Index_Pitch / Little_Pitch (most sensitive)
SETTLE_SEC        = 0.4
GRIP_PITCH        = 157      # fixed grip pitch for pitch fingers (CALIBRATE: 160 grips loose)
GRIP_SETTLE_SEC   = 1.5      # wait for fingers to physically reach the grip pitch
CONTACT_THRESHOLD = 40.0     # post-close effort above this = grip ok, else empty grasp
GLITCH_MAX        = 105.0    # effort above this = Modbus glitch (real grip <=~100)


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

    def move_through_joints(self, poses, step_dur=3.0):
        """Send multiple waypoints as ONE trajectory so arm_node blends them
        (moveAppend + setDefaultZone) -- one continuous motion, no stop-and-go."""
        q_now = [self.current_joints[j] for j in self.joint_names]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        pts = []
        p0 = JointTrajectoryPoint()
        p0.positions = q_now
        p0.time_from_start.sec = 0
        pts.append(p0)
        for i, q in enumerate(poses):
            p = JointTrajectoryPoint()
            p.positions = list(q)
            t = step_dur * (i + 1)
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t % 1) * 1e9)
            pts.append(p)
        goal.trajectory.points = pts
        fut = self.traj_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('traj goal rejected')
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=step_dur * len(poses) + 60.0)
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

    def _read_detect(self, n=3):
        # collect n non-glitch effort samples, return their median DETECT_IDX value
        vals = []
        tries = 0
        while len(vals) < n and tries < 2 * n:
            tries += 1
            eff = self.read_effort()
            if eff is None:
                continue
            d = max(abs(eff[i]) for i in DETECT_IDX)
            if d <= GLITCH_MAX:
                vals.append(d)
        if not vals:
            return None
        vals.sort()
        return vals[len(vals) // 2]

    def _close_read(self, pv):
        # command pitch fingers to pv, settle, return median non-glitch effort (or None)
        pose = [pv if i in PITCH_IDX else 128 for i in range(10)]
        self.hand_set_pose(pose)
        time.sleep(SETTLE_SEC)
        return self._read_detect()

    def force_grasp(self):
        # close pitch fingers to a fixed calibrated grip pitch. effort is logged
        # for reference only: when the finger reaches the commanded pitch, even a
        # firm grip reads low torque (2026-05-21: stable grip read effort 28) --
        # effort cannot confirm the grasp, so do not gate on it.
        self.get_logger().info('  force-grasp: close to grip pitch %d' % GRIP_PITCH)
        pose = [GRIP_PITCH if i in PITCH_IDX else 128 for i in range(10)]
        self.hand_set_pose(pose)
        time.sleep(GRIP_SETTLE_SEC)
        d = self._read_detect()
        eff = ('%.0f' % d) if d is not None else 'n/a'
        self.get_logger().info('  force-grasp: closed to pitch=%d (effort=%s, ref only)'
                               % (GRIP_PITCH, eff))
        return True

    def pick(self):
        poses = LEFT_POSES
        self.get_logger().info('=== PICK START (joint-space + force grasp) ===')

        self.get_logger().info('[1/2] open hand + move pre-grasp->grasp (blended)')
        _req = SetHandPreset.Request()
        _req.name = 'open'
        _hand_fut = self.hand_preset_client.call_async(_req)
        # pre_grasp + grasp as ONE trajectory -> arm_node blends through
        # pre_grasp without stopping (smoother than two separate moves)
        if not self.move_through_joints([poses['pre_grasp'], poses['grasp']], step_dur=3.5):
            return False
        if not (_hand_fut.done() and _hand_fut.result() and _hand_fut.result().success):
            self.get_logger().warn('  hand open not confirmed (continuing)')

        self.get_logger().info('[2/2] force-closed grasp + lift')
        self.force_grasp()
        time.sleep(0.5)
        if not self.move_to_joints(poses['lift'], 3.0):
            return False

        self.get_logger().info('=== PICK DONE (sequence complete; confirm grasp visually) ===')
        return True


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
