#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import contextlib
import json
import os
import sys
import time
import urllib.parse
from functools import wraps


# nav 协议 v1 checklist:URL 配置走环境变量 NAV_WS_URL,默认 ws://127.0.0.1:8101。
# 实际部署地址由 --server-uri 传入(headless 脚本从 navigation_headless.yaml 读)。
DEFAULT_SERVER_URI = os.environ.get("NAV_WS_URL", "ws://127.0.0.1:8101")
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
# 2026-07-24: POI 压在 static costmap 里时逐圈外扩找可达点的上限/步长
DEFAULT_NAV_GOAL_MAX_SNAP_RADIUS = 3.0
DEFAULT_NAV_GOAL_SNAP_STEP = 0.25
DEFAULT_RECONNECT_DELAY = 3.0
# 失败消抖(2026-07-24):nav2 单次失败先压下,等 1s 重发同一导航点;连败
# 超过重试预算才上报 on_error。见 nav2_action_bridge.py。
DEFAULT_NAV_RETRY_COUNT = 1
DEFAULT_NAV_RETRY_DELAY_SEC = 1.0
# 2026-09-15: alias 0 精准入位。--dock-poi-id 列出的 POI 不发 /navigate_to_pose,
# 改发 dock_navigator 的 dock_to_pose(同为 NavigateToPose 类型,内部两段式入位)。
DEFAULT_DOCK_ACTION_NAME = "/dock_to_pose"


def _log(message: str):
    print(f"[navigation_executor] {message}", flush=True)


def _log_outbound_frame(payload: dict):
    # 导航失败必须能在 log 里查到原因:任何最终上报 server 的 on_error 帧
    # (nav2 失败/POI 不存在/目标不可达/协议错)发出前先落一行日志。逐次
    # 尝试的失败详情由 nav2_action_bridge 打,这里是终态帧。
    if payload.get("event_type") == "on_error":
        _log(
            "NAV ERROR reported to server: "
            f"request_id={payload.get('request_id')} "
            f"sub_id={payload.get('sub_id')} "
            f"error_message={payload.get('error_message')}"
        )


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
            "  --server-uri URI        Agent WebSocket server (nav WS v1). Default:",
            "                          $NAV_WS_URL or ws://127.0.0.1:8101",
            "  --poi-store-file PATH   Local YAML file used to persist executor-owned POIs",
            "  --pose-file PATH        named_poses.json to import into the POI store at startup",
            "  --action-name NAME      NavigateToPose action server to drive",
            "                          (default: /navigate_to_pose; waypoint stack uses",
            "                          /waypoint_follower/navigate)",
            "  --reconnect-delay SEC   Delay between WebSocket reconnect attempts (default: 3.0)",
            "  --nav-retry-count N     Silent re-publish attempts after a nav failure before",
            "                          reporting on_error to the server (default: 1)",
            "  --nav-retry-delay SEC   Wait between a nav failure and the re-publish (default: 1.0)",
            "  --dock-poi-id ID        Navigate this POI with two-stage precise docking through",
            "                          the docking action (repeatable; never snapped or retried)",
            "  --dock-action-name NAME Docking action server (default: /dock_to_pose)",
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
        "nav_goal_max_snap_radius": DEFAULT_NAV_GOAL_MAX_SNAP_RADIUS,
        "nav_goal_snap_step": DEFAULT_NAV_GOAL_SNAP_STEP,
        "reconnect_delay": DEFAULT_RECONNECT_DELAY,
        "nav_retry_count": DEFAULT_NAV_RETRY_COUNT,
        "nav_retry_delay_sec": DEFAULT_NAV_RETRY_DELAY_SEC,
        "dock_action_name": DEFAULT_DOCK_ACTION_NAME,
        "dock_poi_ids": [],
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
        if arg == "--nav-retry-count" and i + 1 < len(raw_args):
            config["nav_retry_count"] = int(raw_args[i + 1])
            i += 2
            continue
        if arg == "--nav-retry-delay" and i + 1 < len(raw_args):
            config["nav_retry_delay_sec"] = float(raw_args[i + 1])
            i += 2
            continue
        if arg == "--action-name" and i + 1 < len(raw_args):
            config["action_name"] = raw_args[i + 1]
            i += 2
            continue
        if arg == "--dock-action-name" and i + 1 < len(raw_args):
            config["dock_action_name"] = raw_args[i + 1]
            i += 2
            continue
        if arg == "--dock-poi-id" and i + 1 < len(raw_args):
            poi_id = raw_args[i + 1].strip()
            if poi_id and poi_id not in config["dock_poi_ids"]:
                config["dock_poi_ids"].append(poi_id)
            i += 2
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
    imported_ids = set()
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
        imported_ids.add(pose_id)
        count += 1
    # named_poses.json 是映射唯一真源:清掉上次导入残留、这次 json 里
    # 已经不存在的点位,避免 agent 发旧 id 时导航到过期坐标。
    if hasattr(poi_store, "prune_session"):
        removed = poi_store.prune_session("named_poses", imported_ids)
        if removed:
            _log(f"pruned stale named_poses POIs: {sorted(removed)}")
    return count


def _bypass_proxy_for_server(server_uri: str):
    """把 agent server 的 host 加进 no_proxy 白名单。

    机器上常驻 Clash 代理环境变量(all_proxy=socks5h://127.0.0.1:7897 等),
    websockets>=14 默认读它们;no_proxy 只含 localhost 时,ws://192.168.55.1
    的握手会被整个引向代理 → "timed out during opening handshake",症状与
    server 没起一模一样。agent server 永远在机器人局域网内,不该走代理。
    """
    host = urllib.parse.urlsplit(server_uri).hostname
    if not host:
        return
    for key in ("no_proxy", "NO_PROXY"):
        entries = [e.strip() for e in os.environ.get(key, "").split(",") if e.strip()]
        if host not in entries:
            entries.append(host)
            os.environ[key] = ",".join(entries)


async def _send_heartbeat(websocket, interval_sec: float, last_inbound_holder=None):
    """客户端心跳。2026-07-27 加半死连接告警：ping_interval=None 关掉了
    websockets 库级 keepalive，TCP 半死时只能等内核超时（分钟级）——
    这里至少把"很久没收到任何服务端帧"暴露到日志里，便于断联归因。"""
    warned = False
    while True:
        await asyncio.sleep(interval_sec)
        await websocket.send(json.dumps({"type": "ping"}, ensure_ascii=False))
        if last_inbound_holder:
            age = time.monotonic() - last_inbound_holder[0]
            if age > 3 * interval_sec and not warned:
                warned = True
                _log(f"⚠ 已 {age:.0f}s 未收到服务端任何帧（连接可能半死，等待超时断开）")
            elif age <= interval_sec:
                warned = False


async def _drain_outbound_queue(
    websocket, outbound_queue: asyncio.Queue, pending_holder: list
):
    """发送排水，带送达保证（2026-07-27）。

    旧实现先 get() 出队再 send —— 断联瞬间 send 抛异常时事件已不在队列，
    永久丢失。实测后果：机器人到点，on_arrived 恰落在断联窗口 → 服务端等
    不到到达事件 → 超时判导航失败（"明明到点却上报失败"）。
    新实现：payload 挪进 pending_holder（跨重连存活，由 main_async 持有），
    send 成功才移除；重连后新 drain 任务优先重发 holder 里的滞留事件。
    """
    while True:
        if pending_holder:
            payload = pending_holder[0]
            resend = True
        else:
            payload = await outbound_queue.get()
            pending_holder.append(payload)
            resend = False
        _log_outbound_frame(payload)
        await websocket.send(json.dumps(payload, ensure_ascii=False))
        pending_holder.pop(0)
        event_type = payload.get("event_type") or payload.get("type")
        # 终态帧与重发帧必须可审计；on_progress 等高频帧不刷屏
        if resend or event_type in ("on_arrived", "on_error"):
            tag = "RESENT(断联重发)" if resend else "sent"
            _log(
                f"OUT {tag}: {event_type} "
                f"request_id={payload.get('request_id')} sub_id={payload.get('sub_id')}"
            )


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
    _bypass_proxy_for_server(config["server_uri"])
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
        goal_retry_count=config["nav_retry_count"],
        goal_retry_delay_sec=config["nav_retry_delay_sec"],
    )
    _log(
        f"nav action: {config['action_name']} "
        "(nav WS v1.1: target_id + abort_navigation 随时中断→idle)"
    )
    _log(
        f"nav failure debounce: retry_count={config['nav_retry_count']} "
        f"retry_delay={config['nav_retry_delay_sec']:.1f}s "
        "(单次失败静默重发,连败才上报 on_error)"
    )
    if config["dock_poi_ids"]:
        # 只在配置了精准入位 POI 时才导入(alias 4 不传该参数,完全不受影响)。
        from david_nav.dock_routing import DockRoutingBridge

        dock_bridge = Nav2ActionBridge(
            node=node,
            action_name=config["dock_action_name"],
            server_timeout=config["server_timeout"],
            event_callback=on_bridge_event,
            # dock_navigator 约定入位失败后不再下发任何位移目标;这里静默重发
            # 就等于再走一遍两段式位移,所以精准入位不重试,失败直接上报。
            goal_retry_count=0,
            goal_retry_delay_sec=config["nav_retry_delay_sec"],
        )
        bridge = DockRoutingBridge(bridge, dock_bridge, config["dock_poi_ids"])
        _log(
            f"dock POIs: {', '.join(config['dock_poi_ids'])} -> "
            f"{config['dock_action_name']} (两段式精准入位;不扩圈、失败不重发)"
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
        max_snap_radius=config["nav_goal_max_snap_radius"],
        snap_step=config["nav_goal_snap_step"],
    )
    if config["dock_poi_ids"]:
        from david_nav.dock_routing import DockAwareGoalResolver

        goal_resolver = DockAwareGoalResolver(goal_resolver, config["dock_poi_ids"])
    core = NavigationExecutorCore(
        bridge=bridge,
        pose_provider=pose_provider,
        poi_store=poi_store,
        goal_resolver=goal_resolver,
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
        # 跨重连的未送达事件暂存（见 _drain_outbound_queue 送达保证注释）
        pending_outbound: list = []
        last_inbound_holder = [time.monotonic()]
        disconnected_since = None
        while True:
            connection_established = False
            connected_at = None
            # 防刷屏:连不上时第 1 次失败打印,之后每 10 次(约 30s)提示一行。
            reconnect_attempt += 1
            verbose = reconnect_attempt == 1 or reconnect_attempt % 10 == 0
            try:
                if verbose:
                    suffix = f"(第{reconnect_attempt}次尝试)" if reconnect_attempt > 1 else ""
                    _log(f"connecting to {config['server_uri']} {suffix}".rstrip())
                async with websockets.connect(config["server_uri"], ping_interval=None) as websocket:
                    connection_established = True
                    connected_at = time.monotonic()
                    gap_txt = (
                        f", 断联 {connected_at - disconnected_since:.1f}s"
                        if disconnected_since is not None
                        else ""
                    )
                    pend_txt = (
                        f", 待重发事件 {len(pending_outbound)} 条"
                        if pending_outbound
                        else ""
                    )
                    _log(f"connected (第{reconnect_attempt}次尝试{gap_txt}{pend_txt})")
                    reconnect_attempt = 0
                    last_inbound_holder[0] = time.monotonic()
                    heartbeat_task = asyncio.create_task(
                        _send_heartbeat(
                            websocket, config["heartbeat_interval"], last_inbound_holder
                        )
                    )
                    outbound_task = asyncio.create_task(
                        _drain_outbound_queue(websocket, outbound_queue, pending_outbound)
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
                            last_inbound_holder[0] = time.monotonic()
                            # v1 checklist:未知/畸形消息记日志,不要崩(崩了会
                            # 拆连接,in-flight 导航的事件全丢)。
                            try:
                                payload = json.loads(raw_message)
                            except json.JSONDecodeError:
                                _log(f"ignoring non-JSON message: {raw_message[:200]!r}")
                                continue
                            if not isinstance(payload, dict):
                                _log(f"ignoring non-object message: {raw_message[:200]!r}")
                                continue
                            if payload.get("type") == "pong":
                                continue
                            if payload.get("type") == "ping":
                                await websocket.send(json.dumps({"type": "pong"}, ensure_ascii=False))
                                continue
                            try:
                                outbounds = await core.handle_message(payload)
                            except ValueError as exc:
                                _log(f"ignoring unsupported message: {exc}")
                                continue
                            # 2026-07-27: 应答帧也走送达保证队列（原直发在断联
                            # 瞬间同样会丢，且异常会拆连接）
                            for outbound in outbounds:
                                await outbound_queue.put(outbound)
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
                    disconnected_since = time.monotonic()
                    dur = disconnected_since - connected_at
                    unsent = outbound_queue.qsize() + len(pending_outbound)
                    inbound_age = disconnected_since - last_inbound_holder[0]
                    _log(
                        f"connection closed: {exc} "
                        f"(本次连接 {dur:.1f}s, 距最后收到服务端帧 {inbound_age:.1f}s, "
                        f"未送达事件 {unsent} 条"
                        f"{'，重连后自动重发' if unsent else ''})"
                    )
                elif verbose:
                    _log(f"connection failed: {exc}(连不上时每10次提示一行)")
            else:
                if connection_established:
                    disconnected_since = time.monotonic()
                    unsent = outbound_queue.qsize() + len(pending_outbound)
                    _log(
                        f"connection closed (本次连接 "
                        f"{disconnected_since - connected_at:.1f}s, 未送达事件 {unsent} 条)"
                    )
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
