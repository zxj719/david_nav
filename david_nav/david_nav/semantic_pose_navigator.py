#!/usr/bin/env python3

from dataclasses import dataclass
from builtins import open
import json
import math
import os
from pathlib import Path
import sys
import time


DEFAULT_POSE_FILE = "/home/unitree/semantic_map/named_poses.json"
DEFAULT_ACTION_NAME = "/navigate_to_pose"
DEFAULT_SERVER_TIMEOUT_SEC = 10.0
DEFAULT_RESULT_TIMEOUT_SEC = 300.0
DEFAULT_GOAL_PAUSE_SEC = 0.0
DEFAULT_ROBOT_BASE_FRAME = "base"
DEFAULT_ARRIVAL_CHECK_DISTANCE = 0.75


@dataclass(frozen=True)
class NamedPose:
    pose_id: str
    name: str
    frame_id: str
    x: float
    y: float
    z: float
    yaw: float
    orientation: dict[str, float]


def _log(message: str):
    print(f"[semantic_pose_navigator] {message}", flush=True)


def _build_help_text() -> str:
    return "\n".join(
        [
            "usage: semantic_pose_navigator.py [TARGET] [--target TARGET] [--pose-file PATH] [ROS args...]",
            "",
            "Navigate to one or all poses from /home/unitree/semantic_map/named_poses.json.",
            "",
            "positional arguments:",
            "  TARGET                    Pose id, pose name, or all",
            "",
            "options:",
            "  --target TARGET           Pose id, pose name, or all",
            "  --pose-file PATH          JSON pose file to load",
            "  --action-name NAME        Nav2 action name to use",
            "  --server-timeout-sec SEC  Wait timeout for Nav2 action server",
            "  --result-timeout-sec SEC  Wait timeout for each navigation result",
            "  --goal-pause-sec SEC      Delay between goals when TARGET is all",
            "  --robot-base-frame FRAME  TF frame used to verify arrival. Default: base",
            "  --arrival-check-distance M  Max distance from goal after success. Default: 0.75",
            "  --list                    Print all named poses and exit",
            "  -h, --help                Show this help message and exit",
        ]
    )


def _parse_cli_args(raw_args):
    config = {
        "target": None,
        "pose_file": DEFAULT_POSE_FILE,
        "action_name": DEFAULT_ACTION_NAME,
        "server_timeout_sec": DEFAULT_SERVER_TIMEOUT_SEC,
        "result_timeout_sec": DEFAULT_RESULT_TIMEOUT_SEC,
        "goal_pause_sec": DEFAULT_GOAL_PAUSE_SEC,
        "robot_base_frame": DEFAULT_ROBOT_BASE_FRAME,
        "arrival_check_distance": DEFAULT_ARRIVAL_CHECK_DISTANCE,
        "list_only": False,
    }
    ros_args = []
    i = 0
    while i < len(raw_args):
        arg = raw_args[i]
        if arg == "--target" and i + 1 < len(raw_args):
            config["target"] = raw_args[i + 1].strip()
            i += 2
            continue
        if arg == "--pose-file" and i + 1 < len(raw_args):
            config["pose_file"] = os.path.expanduser(raw_args[i + 1])
            i += 2
            continue
        if arg == "--action-name" and i + 1 < len(raw_args):
            config["action_name"] = raw_args[i + 1].strip()
            i += 2
            continue
        if arg == "--server-timeout-sec" and i + 1 < len(raw_args):
            config["server_timeout_sec"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg == "--result-timeout-sec" and i + 1 < len(raw_args):
            config["result_timeout_sec"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg == "--goal-pause-sec" and i + 1 < len(raw_args):
            config["goal_pause_sec"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg == "--robot-base-frame" and i + 1 < len(raw_args):
            config["robot_base_frame"] = raw_args[i + 1].strip()
            i += 2
            continue
        if arg == "--arrival-check-distance" and i + 1 < len(raw_args):
            config["arrival_check_distance"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg == "--list":
            config["list_only"] = True
            i += 1
            continue
        if not arg.startswith("-") and config["target"] is None:
            config["target"] = arg.strip()
            i += 1
            continue
        ros_args.append(arg)
        i += 1
    return config, ros_args


def _orientation_from_yaw(yaw: float) -> dict[str, float]:
    half = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half),
        "w": math.cos(half),
    }


def _yaw_from_orientation(orientation: dict) -> float:
    x = float(orientation.get("x", 0.0))
    y = float(orientation.get("y", 0.0))
    z = float(orientation.get("z", 0.0))
    w = float(orientation.get("w", 1.0))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalise_orientation(raw_orientation: dict | None, yaw: float) -> dict[str, float]:
    if not raw_orientation:
        return _orientation_from_yaw(yaw)
    return {
        "x": float(raw_orientation.get("x", 0.0)),
        "y": float(raw_orientation.get("y", 0.0)),
        "z": float(raw_orientation.get("z", 0.0)),
        "w": float(raw_orientation.get("w", 1.0)),
    }


def _pose_from_item(item: dict) -> NamedPose:
    position = item.get("position", {})
    orientation = item.get("orientation")
    yaw = float(item["yaw"]) if "yaw" in item else _yaw_from_orientation(orientation or {})
    return NamedPose(
        pose_id=str(item["id"]),
        name=str(item.get("name", item["id"])),
        frame_id=str(item.get("frame_id", "map")),
        x=float(position["x"]),
        y=float(position["y"]),
        z=float(position.get("z", 0.0)),
        yaw=yaw,
        orientation=_normalise_orientation(orientation, yaw),
    )


def load_named_poses(path: str | Path) -> list[NamedPose]:
    pose_path = Path(path)
    raw_text = pose_path.read_text(encoding="utf-8")
    payload = json.loads(raw_text)
    if isinstance(payload, dict):
        raw_items = payload.get("poses", payload.get("named_poses", []))
    else:
        raw_items = payload
    return [_pose_from_item(item) for item in raw_items]


def select_poses(poses: list[NamedPose], target: str | None) -> list[NamedPose]:
    if target is None or target == "":
        return []
    if target.lower() in {"all", "*"}:
        return list(poses)

    selected = [
        pose for pose in poses
        if pose.pose_id == target or pose.name == target
    ]
    if selected:
        return selected

    known = ", ".join(f"{pose.pose_id}({pose.name})" for pose in poses) or "(none)"
    raise RuntimeError(f"unknown semantic pose '{target}'. known poses: {known}")


def _print_pose_summary(poses: list[NamedPose], pose_file: str) -> None:
    if not poses:
        _log(f"no poses found in {pose_file}")
        return
    _log(f"loaded {len(poses)} poses from {pose_file}")
    for pose in poses:
        print(
            f"- {pose.pose_id}: {pose.name} "
            f"({pose.frame_id}=({pose.x:.3f}, {pose.y:.3f}, yaw={pose.yaw:.3f}))",
            flush=True,
        )


def prompt_for_pose_selection(poses: list[NamedPose], pose_file: str) -> str | None:
    if not poses:
        _log(f"no poses found in {pose_file}")
        return None

    _log(f"loaded {len(poses)} poses from {pose_file}")
    for index, pose in enumerate(poses, start=1):
        print(
            f"{index}. {pose.pose_id}: {pose.name} "
            f"({pose.frame_id}=({pose.x:.3f}, {pose.y:.3f}, yaw={pose.yaw:.3f}))",
            flush=True,
        )

    tty = None
    try:
        tty = open("/dev/tty", "r", encoding="utf-8")
    except OSError:
        tty = None

    try:
        while True:
            prompt = "Select pose number, id/name, all, or q to quit: "
            if tty is None:
                value = input(prompt).strip()
            else:
                print(prompt, end="", flush=True)
                value = tty.readline().strip()
            if value.lower() in {"q", "quit", "exit"}:
                return None
            if value.isdigit():
                index = int(value)
                if 1 <= index <= len(poses):
                    return poses[index - 1].pose_id
                print(f"Number must be between 1 and {len(poses)}.", flush=True)
                continue
            if value:
                return value
            print("Input cannot be empty. Type q to quit.", flush=True)
    finally:
        if tty is not None:
            tty.close()


def _assert_arrival_is_plausible(
    pose: NamedPose,
    transform,
    max_distance: float,
    robot_base_frame: str,
) -> float:
    translation = transform.transform.translation
    distance = math.hypot(float(translation.x) - pose.x, float(translation.y) - pose.y)
    if distance > max_distance:
        raise RuntimeError(
            "Nav2 reported success but robot is "
            f"{distance:.2f}m from {pose.pose_id} ({pose.name}); "
            f"target {pose.frame_id}=({pose.x:.3f}, {pose.y:.3f}), "
            f"{robot_base_frame}=({float(translation.x):.3f}, {float(translation.y):.3f}). "
            "Check map->odom/base TF freshness before retrying."
        )
    return distance


def _lookup_robot_transform(node, tf_buffer, pose: NamedPose, robot_base_frame: str):
    import rclpy

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            return tf_buffer.lookup_transform(
                pose.frame_id,
                robot_base_frame,
                rclpy.time.Time(),
            )
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.1)
    return tf_buffer.lookup_transform(pose.frame_id, robot_base_frame, rclpy.time.Time())


def _send_goal(
    node,
    action_client,
    tf_buffer,
    navigate_to_pose,
    pose: NamedPose,
    server_timeout_sec: float,
    result_timeout_sec: float,
    robot_base_frame: str,
    arrival_check_distance: float,
):
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    import rclpy

    if not action_client.wait_for_server(timeout_sec=server_timeout_sec):
        raise RuntimeError(
            f"Nav2 action server is not available within {server_timeout_sec:.1f}s"
        )

    goal = navigate_to_pose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = pose.frame_id
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    goal.pose.pose.position.x = pose.x
    goal.pose.pose.position.y = pose.y
    goal.pose.pose.position.z = pose.z
    goal.pose.pose.orientation.x = pose.orientation["x"]
    goal.pose.pose.orientation.y = pose.orientation["y"]
    goal.pose.pose.orientation.z = pose.orientation["z"]
    goal.pose.pose.orientation.w = pose.orientation["w"]

    _log(
        "sending goal "
        f"{pose.pose_id} ({pose.name}) at "
        f"{pose.frame_id}=({pose.x:.3f}, {pose.y:.3f}, yaw={pose.yaw:.3f})"
    )
    goal_future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, goal_future, timeout_sec=result_timeout_sec)
    if not goal_future.done():
        raise RuntimeError(
            f"goal request did not complete within {result_timeout_sec:.1f}s"
        )

    goal_handle = goal_future.result()
    if goal_handle is None or not goal_handle.accepted:
        raise RuntimeError("Nav2 rejected the goal")

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=result_timeout_sec)
    if not result_future.done():
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future, timeout_sec=5.0)
        raise RuntimeError(
            f"navigation did not finish within {result_timeout_sec:.1f}s"
        )

    result = result_future.result()
    status = getattr(result, "status", None)
    if status == GoalStatus.STATUS_SUCCEEDED:
        transform = _lookup_robot_transform(node, tf_buffer, pose, robot_base_frame)
        distance = _assert_arrival_is_plausible(
            pose,
            transform,
            max_distance=arrival_check_distance,
            robot_base_frame=robot_base_frame,
        )
        _log(f"arrived at {pose.pose_id} ({pose.name}); final distance {distance:.2f}m")
        return
    if status == GoalStatus.STATUS_CANCELED:
        raise RuntimeError("navigation was canceled")
    if status == GoalStatus.STATUS_ABORTED:
        raise RuntimeError("navigation was aborted")
    raise RuntimeError(f"navigation finished with unexpected status {status}")


def main(raw_args=None):
    raw_args = list(sys.argv[1:] if raw_args is None else raw_args)
    if any(arg in {"-h", "--help"} for arg in raw_args):
        print(_build_help_text())
        return 0

    config, ros_args = _parse_cli_args(raw_args)
    poses = load_named_poses(config["pose_file"])

    if config["list_only"]:
        _print_pose_summary(poses, config["pose_file"])
        return 0

    target = config["target"]
    selected = select_poses(poses, target)
    if not selected:
        if target:
            raise RuntimeError("no target selected")

    import rclpy
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    rclpy.init(args=ros_args)
    node = Node("semantic_pose_navigator")
    action_client = ActionClient(node, NavigateToPose, config["action_name"])
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    try:
        if not target:
            while True:
                target = prompt_for_pose_selection(poses, config["pose_file"])
                if target is None:
                    return 0

                selected = select_poses(poses, target)
                if not selected:
                    print(f"Unknown pose '{target}'. Try again.", flush=True)
                    continue

                try:
                    for index, pose in enumerate(selected):
                        if index > 0 and float(config["goal_pause_sec"]) > 0.0:
                            time.sleep(float(config["goal_pause_sec"]))
                        _send_goal(
                            node,
                            action_client,
                            tf_buffer,
                            NavigateToPose,
                            pose,
                            server_timeout_sec=float(config["server_timeout_sec"]),
                            result_timeout_sec=float(config["result_timeout_sec"]),
                            robot_base_frame=config["robot_base_frame"],
                            arrival_check_distance=float(config["arrival_check_distance"]),
                        )
                except RuntimeError as exc:
                    _log(f"navigation failed: {exc}")
                    print("Select another pose or q to quit.", flush=True)
                    continue

                print("Done. Select another pose or q to quit.", flush=True)
            return 0

        for index, pose in enumerate(selected):
            if index > 0 and float(config["goal_pause_sec"]) > 0.0:
                time.sleep(float(config["goal_pause_sec"]))
            _send_goal(
                node,
                action_client,
                tf_buffer,
                NavigateToPose,
                pose,
                server_timeout_sec=float(config["server_timeout_sec"]),
                result_timeout_sec=float(config["result_timeout_sec"]),
                robot_base_frame=config["robot_base_frame"],
                arrival_check_distance=float(config["arrival_check_distance"]),
            )
    finally:
        del tf_listener
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
