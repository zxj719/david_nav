#!/usr/bin/env python3
"""T1.a 招呼动作 (greeting wave).

Plays a smooth left-arm waving motion: from the MoveIt home pose, raise the
arm and swing the wrist a few times, then return to the MoveIt home pose.

The whole wave is sent as ONE multi-point FollowJointTrajectory to arm_node.
arm_node turns every point into a MoveAbsJ and runs them in a single
moveAppend with setDefaultZone(80), so the arm blends through all waypoints
in one continuous motion -- no stop-and-go between poses.

Wave poses are from A's workspace_arm/Arm_skills/wave_hand.py (SDK degrees).
The left arm joint_offsets_deg is [0]*7 (verified in sdk_bridge.launch.py),
so SDK and URDF joint conventions coincide -- degrees->radians is the only
conversion. Start/end pose is the MoveIt home (SRDF home group_state), NOT
wave_hand.py's own home.

Usage: ros2 run david_grasp greet
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

LEFT_JOINT_PREFIX = 'AR5-5_08L-W4C4A6-ZY2_joint_'
LEFT_CONTROLLER = 'left_arm_controller'

# MoveIt home, left arm -- SRDF home group_state
# (URDF rad == SDK [145, 90, -90, 100, 90, 0, 0] deg)
HOME = [2.5307, 1.5708, -1.5708, 1.7453, 1.5708, 0.0, 0.0]

# wave poses from wave_hand.py, SDK degrees (left-arm offset 0 -> == URDF deg)
_P1 = [81.0, 85.6, -88.5, 119.2, 145.3, -29.7, 20.4]
_P2 = [79.7, 81.5, -87.7, 115.5, 145.3, -50.0, -10.3]
_P1_TURN = _P1[:6] + [_P1[6] + 10.0]      # joint7 +10deg, rounds the swing corner
_P2_TURN = _P2[:6] + [_P2[6] + 10.0]


def _rad(deg):
    return [math.radians(d) for d in deg]


# full sequence: MoveIt home -> raise + swing the wrist -> back to MoveIt home
WAVE_SEQUENCE = [
    HOME,
    _rad(_P1),
    _rad(_P2), _rad(_P2_TURN), _rad(_P1),
    _rad(_P1_TURN), _rad(_P2),
    _rad(_P2_TURN), _rad(_P1),
    HOME,
]
STEP_SEC = 2.0   # nominal stamp per waypoint; arm_node uses sdk_speed for real motion


class Greet(Node):
    def __init__(self):
        super().__init__('greet_left')
        self.joint_names = [LEFT_JOINT_PREFIX + str(i) for i in range(1, 8)]
        self.client = ActionClient(
            self, FollowJointTrajectory,
            '/' + LEFT_CONTROLLER + '/follow_joint_trajectory')

    def wave(self):
        self.get_logger().info('=== GREET (wave) START ===')
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('arm trajectory action server missing')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        pts = []
        for i, q in enumerate(WAVE_SEQUENCE):
            p = JointTrajectoryPoint()
            p.positions = list(q)
            t = STEP_SEC * (i + 1)
            p.time_from_start.sec = int(t)
            p.time_from_start.nanosec = int((t % 1) * 1e9)
            pts.append(p)
        goal.trajectory.points = pts

        self.get_logger().info('sending %d-point wave (one blended trajectory)' % len(pts))
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if not gh or not gh.accepted:
            self.get_logger().error('wave goal rejected')
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=180.0)
        if not rf.done():
            self.get_logger().error('wave exec timeout')
            return False
        ok = rf.result().result.error_code == 0
        self.get_logger().info('=== GREET DONE ===' if ok else '=== GREET FAILED ===')
        return ok


def main():
    rclpy.init()
    node = Greet()
    try:
        ok = node.wave()
        node.get_logger().info('RESULT: ' + ('SUCCESS' if ok else 'FAIL'))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()