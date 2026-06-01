#!/usr/bin/env python3
"""One-shot hand pose setter via ROS SetHandPose service.

Usage:
    python3 set_hand_pose.py 255 128 255 255 255 255 128 128 128 128
                              ^   ^   ^   ^   ^   ^   ^   ^   ^   ^
                              T_p T_y I_p M_p R_p L_p I_y R_y L_y T_r
                              (Thumb/Index/Middle/Ring/Little x Pitch/Yaw/Roll)

Values 0-255, hand convention:
  - Pitch: 255 = open (extended), 0 = close (curled)
  - Yaw:   128 = center
  - Thumb_Roll: 128 = center

Examples (arm_node convention, same as our pick_from_cam):
    255 128 255 255 255 255 128 128 128 128       (default open)
    255 80  255 255 255 255 128 128 128 128       (thumb yaw'd 48 toward palm)
    255 180 255 255 255 255 128 128 128 128       (thumb yaw'd 52 outward)
"""
import sys
import rclpy
from rclpy.node import Node
from david_hand_msgs.srv import SetHandPose


def main():
    if len(sys.argv) != 11:
        print('need 10 values (0-255)')
        print('order: Thumb_Pitch Thumb_Yaw Index_Pitch Middle_Pitch Ring_Pitch '
              'Little_Pitch Index_Yaw Ring_Yaw Little_Yaw Thumb_Roll')
        sys.exit(1)
    pose = [max(0, min(255, int(v))) for v in sys.argv[1:11]]
    rclpy.init()
    node = rclpy.create_node('set_hand_pose_oneshot')
    client = node.create_client(SetHandPose, '/right_hand/set_pose10')
    if not client.wait_for_service(timeout_sec=5.0):
        print('service /right_hand/set_pose10 not up'); sys.exit(2)
    req = SetHandPose.Request()
    req.pose = pose
    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=30.0)
    if not fut.done() or fut.result() is None:
        print('call timed out'); sys.exit(3)
    r = fut.result()
    print(f'success={r.success} message={r.message!r}')
    print(f'pose sent = {pose}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
