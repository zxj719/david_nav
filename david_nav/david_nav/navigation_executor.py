#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import contextlib
import json
import os
import sys
import time
from functools import wraps


DEFAULT_SERVER_URI = "ws://172.16.21.205:8100/ws/navigation/executor"
DEFAULT_POI_STORE_FILE = os.path.expanduser("~/.ros/david_nav/poi_store.yaml")
DEFAULT_POSE_FILE_SOURCE = os.path.expanduser("~/semantic_map/named_poses.json")
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
DEFAULT_RECONNECT_DELAY = 3.0


def _log(message: str):
    print(f"[navigation_executor] {message}", flush=True)


def _drop_loop_kwarg(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        kwargs.pop("loop", None)
        return func(*args, **kwargs)

    return wrapper


def _patch_asyncio_for_legacy_websockets():
    if getattr(asyncio, "_david_nav_legacy_websockets_patched", False):
        return

    asyncio.Lock = _drop_loop_kwarg(asyncio.Lock)
    asyncio.sleep = _drop_loop_kwarg(asyncio.sleep)
    asyncio.wait = _drop_loop_kwarg(asyncio.wait)
    asyncio.wait_for = _drop_loop_kwarg(asyncio.wait_for)
    asyncio._david_nav_legacy_websockets_patched = True


def _build_help_text() -> str:
    return "\n".join(
        [
            "usage: navigation_executor.py [--server-uri URI] [--poi-store-file PATH] [--pose-file PATH] [ROS args...]",
            "",
            "Robot-side WebSocket navigation executor for david_nav.",
            "",
            "options:",
            "  --server-uri URI        WebSocket endpoint for /ws/navigation/executor",
            "  --poi-store-file PATH   Local YAML file used to persist executor-owned POIs",
            "  --pose-file PATH        named_poses.json to import into the POI store at startup",
            "  --action-name NAME      NavigateToPose action server to drive",
            "                          (default: /navigate_to_pose; waypoint stack uses",
            "                          /waypoint_follower/navigate)",
            "  --theta-degrees         Treat v2 target_pose.theta as degrees (default: radians)",
            "  --reconnect-delay SEC   Delay between WebSocket reconnect attempts (default: 3.0)",
            "  -h, --help              Show this help message and exit",
        ]
    )


def _parse_cli_args(raw_args):
    config = {
        "server_uri": DEFAULT_SERVER_URI,
        "poi_store_file": DEFAULT_POI_STORE_FILE,
        "pose_file": None,
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
        "reconnect_delay": DEFAULT_RECONNECT_DELAY,
        "theta_in_degrees": False,
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
        if arg == "--pose-file" and i + 1 < len(raw_args):
            config["pose_file"] = os.path.expanduser(raw_args[i + 1])
            i += 2
            continue
        if arg == "--reconnect-delay" and i + 1 < len(raw_args):
            config["reconnect_delay"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg == "--action-name" and i + 1 < len(raw_args):
            config["action_name"] = raw_args[i + 1]
            i += 2
            continue
        if arg == "--theta-degrees":
            config["theta_in_degrees"] = True
            i += 1
            continue
        ros_args.append(arg)
        i += 1
    return config, ros_args


def _populate_poi_store_from_named_poses_json(poi_store, json_path: str) -> int:
    from david_nav.navigation_types import PoseRecord

    with open(json_path, "r", encoding="utf-8") as handle:
        payload = json.load(handle)

    if isinstance(payload, dict):
        items = payload.get("poses") or payload.get("named_poses") or []
    else:
        items = payload

    count = 0
    for entry in items:
        if not isinstance(entry, dict):
            continue
        position = entry.get("position", {}) or {}
        pose_id = str(entry["id"])
        name = str(entry.get("name", pose_id))
        frame_id = str(entry.get("frame_id", "map"))
        x = float(position.get("x", 0.0))
        y = float(position.get("y", 0.0))
        z = float(position.get("z", 0.0))
        yaw = float(entry.get("yaw", 0.0))
        map_pose = PoseRecord(frame_id, x, y, z, yaw, 0.0)
        odom_pose = PoseRecord("odom", x, y, z, yaw, 0.0)
        poi_store.upsert_marked_poi(
            poi_id=pose_id,
            name=name,
            map_pose=map_pose,
            odom_pose=odom_pose,
            slam_session_id="named_poses",
        )
        count += 1
    return count


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


def _format_status_line(core, pose_provider) -> str:
    state = getattr(core.state, "value", str(core.state))
    task = core.active_task
    if task is not None:
        poi = task.target_id
        phase = task.phase
        d2g = f"{task.remaining_distance:.2f}m"
    else:
        poi = "-"
        phase = "-"
        d2g = "-"
    try:
        p = pose_provider.current_map_pose()
        pose_str = f"x={p.x:.2f} y={p.y:.2f} yaw={p.yaw:.2f}rad"
    except Exception as exc:
        pose_str = f"pose=n/a ({type(exc).__name__})"
    return (
        f"STATUS state={state} poi={poi} phase={phase} d2g={d2g} {pose_str}"
    )


async def _print_status_loop(core, pose_provider, interval_sec: float):
    # 防刷屏(2026-06-11):周期心跳只在有 active_task(导航进行中)时打,
    # 频率 1/interval_sec(默认 ~0.3Hz);空闲/出错等静止状态不周期打印,
    # 只在 (state, poi, phase) 变化的那一刻立即打一行,终端能看到转换。
    last_key = None
    last_print_time = 0.0
    poll_sec = 0.25
    while True:
        await asyncio.sleep(poll_sec)
        task = core.active_task
        key = (
            getattr(core.state, "value", str(core.state)),
            task.target_id if task else None,
            task.phase if task else None,
        )
        now = time.monotonic()
        ongoing = task is not None
        due = ongoing and (now - last_print_time) >= interval_sec
        if key != last_key or due:
            _log(_format_status_line(core, pose_provider))
            last_key = key
            last_print_time = now


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

    from david_nav.navigation_executor_core import NavigationExecutorCore
    from david_nav.navigation_goal_resolver import NavigationGoalResolver
    from david_nav.nav2_action_bridge import Nav2ActionBridge
    from david_nav.poi_store import LocalPoiStore
    from david_nav.pose_provider import TfPoseProvider

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

    if config["pose_file"]:
        if not os.path.exists(config["pose_file"]):
            _log(f"pose file not found: {config['pose_file']}")
        else:
            loaded = _populate_poi_store_from_named_poses_json(
                poi_store, config["pose_file"]
            )
            _log(f"loaded {loaded} POIs from {config['pose_file']}")

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
    _log(
        f"nav action: {config['action_name']}, "
        f"theta unit: {'degrees' if config['theta_in_degrees'] else 'radians'}"
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
        theta_in_degrees=config["theta_in_degrees"],
    )

    status_task = asyncio.create_task(
        _print_status_loop(
            core,
            pose_provider,
            # 0.3Hz:导航进行中的心跳频率;空闲态不周期打印(见 loop 内注释)
            float(config.get("status_print_interval_sec", 1.0 / 0.3)),
        )
    )

    try:
        reconnect_attempt = 0
        while True:
            connection_established = False
            # 防刷屏:连不上时第 1 次失败打印,之后每 10 次(约 30s)提示一行。
            reconnect_attempt += 1
            verbose = reconnect_attempt == 1 or reconnect_attempt % 10 == 0
            try:
                if verbose:
                    suffix = f"(第{reconnect_attempt}次尝试)" if reconnect_attempt > 1 else ""
                    _log(f"connecting to {config['server_uri']} {suffix}".rstrip())
                async with websockets.connect(config["server_uri"], ping_interval=None) as websocket:
                    connection_established = True
                    reconnect_attempt = 0
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
                        with contextlib.suppress(asyncio.CancelledError):
                            await heartbeat_task
                        with contextlib.suppress(asyncio.CancelledError):
                            await outbound_task
                        with contextlib.suppress(asyncio.CancelledError):
                            await loop_closure_task
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                if connection_established:
                    _log(f"connection closed: {exc}")
                elif verbose:
                    _log(f"connection failed: {exc}(连不上时每10次提示一行)")
            else:
                if connection_established:
                    _log("connection closed")
            if verbose or connection_established:
                _log(f"reconnecting in {config['reconnect_delay']:.1f}s")
            await asyncio.sleep(config["reconnect_delay"])
    finally:
        spin_task.cancel()
        status_task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await spin_task
        with contextlib.suppress(asyncio.CancelledError):
            await status_task
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
