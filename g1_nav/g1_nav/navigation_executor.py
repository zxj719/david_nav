#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import contextlib
import json
import os
import sys
from functools import wraps


DEFAULT_SERVER_URI = "ws://172.16.21.205:8100/ws/navigation/executor"
DEFAULT_POI_STORE_FILE = os.path.expanduser("~/.ros/g1_nav/poi_store.yaml")
DEFAULT_HEARTBEAT_INTERVAL = 30.0
DEFAULT_LOOP_CLOSURE_POLL_SEC = 1.0
DEFAULT_ACTION_NAME = "/navigate_to_pose"
DEFAULT_SERVER_TIMEOUT = 10.0
DEFAULT_ROBOT_BASE_FRAME = "base_link"
DEFAULT_ODOM_TOPIC = "/lightning/odometry"
DEFAULT_SLAM_SESSION_ID = "live_session"
DEFAULT_GRID_MAP_TOPIC = "/lightning/grid_map"
DEFAULT_GLOBAL_COSTMAP_TOPIC = "/global_costmap/costmap"
DEFAULT_NAV_GOAL_SNAP_RADIUS = 0.5
DEFAULT_NAV_GOAL_FALLBACK_SNAP_RADIUS = 1.0
DEFAULT_NAV_GOAL_CLEARANCE_RADIUS = 0.1


def _log(message: str):
    print(f"[navigation_executor] {message}", flush=True)


def _drop_loop_kwarg(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        kwargs.pop("loop", None)
        return func(*args, **kwargs)

    return wrapper


def _patch_asyncio_for_legacy_websockets():
    if getattr(asyncio, "_g1_nav_legacy_websockets_patched", False):
        return

    asyncio.Lock = _drop_loop_kwarg(asyncio.Lock)
    asyncio.sleep = _drop_loop_kwarg(asyncio.sleep)
    asyncio.wait = _drop_loop_kwarg(asyncio.wait)
    asyncio.wait_for = _drop_loop_kwarg(asyncio.wait_for)
    asyncio._g1_nav_legacy_websockets_patched = True


def _build_help_text() -> str:
    return "\n".join(
        [
            "usage: navigation_executor.py [--server-uri URI] [--poi-store-file PATH] [ROS args...]",
            "",
            "Robot-side WebSocket navigation executor for g1_nav.",
            "",
            "options:",
            "  --server-uri URI        WebSocket endpoint for /ws/navigation/executor",
            "  --poi-store-file PATH   Local YAML file used to persist executor-owned POIs",
            "  -h, --help              Show this help message and exit",
        ]
    )


def _parse_cli_args(raw_args):
    config = {
        "server_uri": DEFAULT_SERVER_URI,
        "poi_store_file": DEFAULT_POI_STORE_FILE,
        "heartbeat_interval": DEFAULT_HEARTBEAT_INTERVAL,
        "loop_closure_poll_sec": DEFAULT_LOOP_CLOSURE_POLL_SEC,
        "action_name": DEFAULT_ACTION_NAME,
        "server_timeout": DEFAULT_SERVER_TIMEOUT,
        "robot_base_frame": DEFAULT_ROBOT_BASE_FRAME,
        "odom_topic": DEFAULT_ODOM_TOPIC,
        "slam_session_id": DEFAULT_SLAM_SESSION_ID,
        "grid_map_topic": DEFAULT_GRID_MAP_TOPIC,
        "global_costmap_topic": DEFAULT_GLOBAL_COSTMAP_TOPIC,
        "nav_goal_snap_radius": DEFAULT_NAV_GOAL_SNAP_RADIUS,
        "nav_goal_fallback_snap_radius": DEFAULT_NAV_GOAL_FALLBACK_SNAP_RADIUS,
        "nav_goal_clearance_radius": DEFAULT_NAV_GOAL_CLEARANCE_RADIUS,
    }
    ros_args = []
    i = 0
    while i < len(raw_args):
        arg = raw_args[i]
        if arg == "--server-uri" and i + 1 < len(raw_args):
            config["server_uri"] = raw_args[i + 1]
            i += 2
            continue
        if arg == "--poi-store-file" and i + 1 < len(raw_args):
            config["poi_store_file"] = os.path.expanduser(raw_args[i + 1])
            i += 2
            continue
        ros_args.append(arg)
        i += 1
    return config, ros_args


async def _send_heartbeat(websocket, interval_sec: float):
    while True:
        await asyncio.sleep(interval_sec)
        await websocket.send(json.dumps({"type": "ping"}, ensure_ascii=False))


async def _drain_outbound_queue(websocket, outbound_queue: asyncio.Queue):
    while True:
        payload = await outbound_queue.get()
        await websocket.send(json.dumps(payload, ensure_ascii=False))


async def _spin_ros_node(node, spin_once_fn, interval_sec: float = 0.01):
    while True:
        spin_once_fn(node, timeout_sec=0.0)
        await asyncio.sleep(interval_sec)


async def _watch_loop_closure(pose_provider, poi_store, poll_sec: float):
    last_transform = None
    while True:
        await asyncio.sleep(poll_sec)
        try:
            current_transform = pose_provider.current_map_from_odom()
        except Exception:
            continue
        if current_transform == last_transform:
            continue
        poi_store.reproject_session(
            slam_session_id=pose_provider.current_slam_session_id(),
            map_from_odom=current_transform,
            translation_threshold=0.05,
            yaw_threshold=0.03,
        )
        last_transform = current_transform


async def main_async(raw_args):
    import rclpy

    _patch_asyncio_for_legacy_websockets()

    import websockets
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    from g1_nav.navigation_executor_core import NavigationExecutorCore
    from g1_nav.navigation_goal_resolver import NavigationGoalResolver
    from g1_nav.nav2_action_bridge import Nav2ActionBridge
    from g1_nav.poi_store import LocalPoiStore
    from g1_nav.pose_provider import TfPoseProvider

    config, ros_args = _parse_cli_args(raw_args)
    rclpy.init(args=ros_args)
    node = Node("navigation_executor")
    spin_task = asyncio.create_task(
        _spin_ros_node(
            node,
            spin_once_fn=rclpy.spin_once,
        )
    )
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    poi_store = LocalPoiStore(config["poi_store_file"])
    if hasattr(poi_store, "load"):
        poi_store.load()

    core = None
    outbound_queue: asyncio.Queue = asyncio.Queue()

    async def on_bridge_event(event: dict):
        if core is None:
            return
        for payload in await core.handle_bridge_event(event):
            await outbound_queue.put(payload)

    bridge = Nav2ActionBridge(
        node=node,
        action_name=config["action_name"],
        server_timeout=config["server_timeout"],
        event_callback=on_bridge_event,
    )
    pose_provider = TfPoseProvider(
        node=node,
        tf_buffer=tf_buffer,
        robot_base_frame=config["robot_base_frame"],
        odom_topic=config["odom_topic"],
        slam_session_id_fn=lambda: config["slam_session_id"],
    )
    goal_resolver = NavigationGoalResolver(
        node=node,
        pose_provider=pose_provider,
        map_topic=config["grid_map_topic"],
        global_costmap_topic=config["global_costmap_topic"],
        snap_radius=config["nav_goal_snap_radius"],
        fallback_snap_radius=config["nav_goal_fallback_snap_radius"],
        goal_clearance_radius=config["nav_goal_clearance_radius"],
    )
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=pose_provider,
        poi_store=poi_store,
        goal_resolver=goal_resolver,
    )

    connection_established = False

    try:
        _log(f"connecting to {config['server_uri']}")
        async with websockets.connect(config["server_uri"], ping_interval=None) as websocket:
            connection_established = True
            _log("connected")
            heartbeat_task = asyncio.create_task(
                _send_heartbeat(websocket, config["heartbeat_interval"])
            )
            outbound_task = asyncio.create_task(
                _drain_outbound_queue(websocket, outbound_queue)
            )
            loop_closure_task = asyncio.create_task(
                _watch_loop_closure(
                    pose_provider,
                    poi_store,
                    config["loop_closure_poll_sec"],
                )
            )
            try:
                async for raw_message in websocket:
                    payload = json.loads(raw_message)
                    if payload.get("type") == "pong":
                        continue
                    if payload.get("type") == "ping":
                        await websocket.send(json.dumps({"type": "pong"}, ensure_ascii=False))
                        continue
                    for outbound in await core.handle_message(payload):
                        await websocket.send(json.dumps(outbound, ensure_ascii=False))
            finally:
                heartbeat_task.cancel()
                outbound_task.cancel()
                loop_closure_task.cancel()
                spin_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await heartbeat_task
                with contextlib.suppress(asyncio.CancelledError):
                    await outbound_task
                with contextlib.suppress(asyncio.CancelledError):
                    await loop_closure_task
                with contextlib.suppress(asyncio.CancelledError):
                    await spin_task
    except Exception as exc:
        if connection_established:
            _log(f"connection closed: {exc}")
        else:
            _log(f"connection failed: {exc}")
        raise
    else:
        if connection_established:
            _log("connection closed")
    finally:
        del tf_listener
        node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    raw_args = sys.argv[1:] if args is None else list(args)
    if any(arg in ("-h", "--help") for arg in raw_args):
        print(_build_help_text())
        return
    asyncio.run(main_async(raw_args))


if __name__ == "__main__":
    main()
