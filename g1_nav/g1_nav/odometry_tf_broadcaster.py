from __future__ import annotations

from types import SimpleNamespace


def build_transform_from_odometry(odometry_msg):
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=odometry_msg.header.stamp,
            frame_id=odometry_msg.header.frame_id,
        ),
        child_frame_id=odometry_msg.child_frame_id,
        transform=SimpleNamespace(
            translation=SimpleNamespace(
                x=odometry_msg.pose.pose.position.x,
                y=odometry_msg.pose.pose.position.y,
                z=odometry_msg.pose.pose.position.z,
            ),
            rotation=SimpleNamespace(
                x=odometry_msg.pose.pose.orientation.x,
                y=odometry_msg.pose.pose.orientation.y,
                z=odometry_msg.pose.pose.orientation.z,
                w=odometry_msg.pose.pose.orientation.w,
            ),
        ),
    )
