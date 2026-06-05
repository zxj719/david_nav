#!/usr/bin/env python3
"""Move one or both arms to home pose via MoveIt (collision-checked).

Usage:
    ros2 run david_grasp arm_home left
    ros2 run david_grasp arm_home right
    ros2 run david_grasp arm_home both
"""
import math
import sys

import rclpy

from david_grasp.pick_from_cam import PickFromCam

# 2026-06-02 calibrated home poses (URDF rad = SDK rad, joint_signs=[1]*7)
LEFT_HOME  = [math.radians(d) for d in [145.0, 90.0, -90.0, 140.0, 90.0, 0.0, 0.0]]
RIGHT_HOME = [math.radians(d) for d in [-55.0, 85.0, 100.0, 140.0, -90.0, 5.0, 0.0]]

_SIDE_HOME = {'left': LEFT_HOME, 'right': RIGHT_HOME}


class ArmHome(PickFromCam):
    def __init__(self, side: str):
        super().__init__(side=side)

    def go_home(self) -> bool:
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('MoveGroup (/move_action) not available — is real_robot.launch.py running?')
            return False
        self.get_logger().info(f'moving {self.side} arm to home (MoveIt planned)')
        return self.move_to_joints(_SIDE_HOME[self.side], label='home')


def main():
    args = sys.argv[1:]
    if not args or args[0] not in ('left', 'right', 'both'):
        print('Usage: ros2 run david_grasp arm_home <left|right|both>')
        sys.exit(1)

    side = args[0]
    sides = ['left', 'right'] if side == 'both' else [side]

    rclpy.init()
    ok = True
    for s in sides:
        node = ArmHome(side=s)
        try:
            ok = node.go_home() and ok
        finally:
            node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
