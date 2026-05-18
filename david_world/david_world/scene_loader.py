#!/usr/bin/env python3
"""Push scene CollisionObjects to MoveIt planning scene (no ACM changes)."""
import os
import yaml

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory


def make_box_co(name, center, size, frame_id="world", op=CollisionObject.ADD):
    co = CollisionObject()
    co.header.frame_id = frame_id
    co.id = name
    prim = SolidPrimitive()
    prim.type = SolidPrimitive.BOX
    prim.dimensions = [float(size[0]), float(size[1]), float(size[2])]
    pose = Pose()
    pose.position.x = float(center[0])
    pose.position.y = float(center[1])
    pose.position.z = float(center[2])
    pose.orientation.w = 1.0
    co.primitives.append(prim)
    co.primitive_poses.append(pose)
    co.operation = op
    return co


class SceneLoader(Node):
    def __init__(self):
        super().__init__("scene_loader")
        self.declare_parameter("scene", "double_passage")
        self.declare_parameter("clear", False)
        scene = self.get_parameter("scene").value
        clear = bool(self.get_parameter("clear").value)

        share = get_package_share_directory("david_world")
        yaml_path = os.path.join(share, "scenes", f"{scene}.yaml")
        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Scene file not found: {yaml_path}")
            return

        with open(yaml_path) as f:
            cfg = yaml.safe_load(f)
        frame_id = cfg.get("frame_id", "world")
        objects = cfg.get("objects", [])
        op_str = "REMOVE" if clear else "ADD"
        op = CollisionObject.REMOVE if clear else CollisionObject.ADD

        self.get_logger().info(
            f"Scene '{scene}': {len(objects)} objects, op={op_str}, frame={frame_id}"
        )

        client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("/apply_planning_scene service unavailable")
            return

        ps = PlanningScene()
        ps.is_diff = True
        for obj in objects:
            co = make_box_co(obj["name"], obj["center"], obj["size"], frame_id=frame_id, op=op)
            ps.world.collision_objects.append(co)

        req = ApplyPlanningScene.Request()
        req.scene = ps
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        res = fut.result()
        if res is None:
            self.get_logger().error("apply_planning_scene timeout")
            return
        if res.success:
            self.get_logger().info(f"Scene {op_str} OK ({len(objects)} objects)")
        else:
            self.get_logger().error("Scene apply failed")


def main(args=None):
    rclpy.init(args=args)
    node = SceneLoader()
    rclpy.spin_once(node, timeout_sec=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
