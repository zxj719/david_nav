from __future__ import annotations

import math

from g1_nav.navigation_types import CapturedPose, PoseRecord, Transform2D


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TfPoseProvider:
    def __init__(
        self,
        node,
        tf_buffer,
        robot_base_frame: str,
        odom_topic: str,
        slam_session_id_fn,
    ):
        self.node = node
        self.tf_buffer = tf_buffer
        self.robot_base_frame = robot_base_frame
        self.odom_topic = odom_topic
        self.slam_session_id_fn = slam_session_id_fn
        self.latest_odom_pose = None
        self._odom_subscription = self.node.create_subscription(
            __import__("nav_msgs.msg", fromlist=["Odometry"]).Odometry,
            odom_topic,
            self._on_odom,
            10,
        )

    def _on_odom(self, msg):
        pose = msg.pose.pose
        self.latest_odom_pose = PoseRecord(
            frame_id=msg.header.frame_id or "odom",
            x=float(pose.position.x),
            y=float(pose.position.y),
            z=float(pose.position.z),
            yaw=_yaw_from_quaternion(
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
            stamp_sec=float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9,
        )

    def _lookup_map_pose(self) -> PoseRecord:
        import rclpy

        transform = self.tf_buffer.lookup_transform(
            "map",
            self.robot_base_frame,
            rclpy.time.Time(),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        return PoseRecord(
            frame_id=transform.header.frame_id or "map",
            x=float(translation.x),
            y=float(translation.y),
            z=float(translation.z),
            yaw=_yaw_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
            stamp_sec=float(stamp.sec) + float(stamp.nanosec) / 1e9,
        )

    def current_map_pose(self) -> PoseRecord:
        return self._lookup_map_pose()

    def _lookup_odom_pose(self) -> PoseRecord:
        if self.latest_odom_pose is None:
            raise RuntimeError(f"no odometry available on {self.odom_topic}")
        return self.latest_odom_pose

    async def capture_for_poi(self) -> CapturedPose:
        map_pose = self._lookup_map_pose()
        odom_pose = self._lookup_odom_pose()
        return CapturedPose(
            map_pose=map_pose,
            odom_pose=odom_pose,
            slam_session_id=self.current_slam_session_id(),
        )

    def current_slam_session_id(self) -> str:
        return self.slam_session_id_fn()

    def current_map_from_odom(self) -> Transform2D:
        import rclpy

        transform = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
        rotation = transform.transform.rotation
        return Transform2D(
            x=float(transform.transform.translation.x),
            y=float(transform.transform.translation.y),
            yaw=_yaw_from_quaternion(
                float(rotation.x),
                float(rotation.y),
                float(rotation.z),
                float(rotation.w),
            ),
        )
