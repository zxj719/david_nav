#!/usr/bin/env python3
"""Mock object-pose publisher for Stage 4 integration testing.

Publishes a fixed PoseStamped on /object/water_bottle_pose so pick_cartesian.py
can be developed and tested before B's vision stack is ready. Swap this out for
the real vision node when available -- the topic / message type are identical.

Usage: ros2 run david_grasp mock_grasp_pose
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

TOPIC = '/object/water_bottle_pose'
FRAME = 'base_link'
# placeholder bottle centre in base_link -- CALIBRATE to a real reachable spot
MOCK_XYZ = (0.45, 0.25, 0.85)


class MockGraspPose(Node):
    def __init__(self):
        super().__init__('mock_grasp_pose')
        self.pub = self.create_publisher(PoseStamped, TOPIC, 10)
        self.timer = self.create_timer(0.5, self._tick)   # 2 Hz
        self.get_logger().info('mock object pose -> %s  xyz=%s frame=%s'
                               % (TOPIC, MOCK_XYZ, FRAME))

    def _tick(self):
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = FRAME
        m.pose.position.x = float(MOCK_XYZ[0])
        m.pose.position.y = float(MOCK_XYZ[1])
        m.pose.position.z = float(MOCK_XYZ[2])
        m.pose.orientation.w = 1.0
        self.pub.publish(m)


def main():
    rclpy.init()
    node = MockGraspPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()