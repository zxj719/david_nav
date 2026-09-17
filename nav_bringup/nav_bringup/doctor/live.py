"""Doctor checks that look at the running ROS graph (``navrun doctor --live``).

They cover the failure modes of refactor plan §13.3 that only show while the
stack runs:

* a stale ros2 CLI daemon that hides nodes, found by comparing its node list
  with discovery without the daemon;
* more than one publisher on a velocity command topic;
* publishers on more than one host, the signature of another machine in the
  DDS domain (Fast DDS derives two GID bytes from the host);
* no map to odom transform yet, which means localization has not been
  initialized (a hint, not a failure).

ros2 CLI calls go through an injectable runner and the transform probe is
injectable too, so unit tests need no ROS graph. A running CLI daemon is only
queried; the checks never start or stop it.
"""

from __future__ import annotations

from dataclasses import dataclass
import functools
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from nav_bringup.doctor import checks
from nav_bringup.doctor.checks import CheckResult
from nav_bringup.doctor.checks import Status
from nav_bringup.recorder import manifest
from nav_bringup.recorder import param_dump

TransformProbe = Callable[[str, str], Optional[bool]]
FAST_DDS_VENDOR = ('01', '0f')


@dataclass(frozen=True)
class Endpoint:
    """One endpoint listed by ``ros2 topic info --verbose``.

    Attributes:
        node_name: Node name without namespace.
        node_namespace: Node namespace.
        endpoint_type: ``PUBLISHER`` or ``SUBSCRIPTION``.
        gid: Endpoint GID as dot-separated hexadecimal bytes.
    """

    node_name: str
    node_namespace: str
    endpoint_type: str
    gid: str

    @property
    def qualified_node(self) -> str:
        """str: Fully qualified node name."""
        return f"{self.node_namespace.rstrip('/')}/{self.node_name}"


@dataclass(frozen=True)
class LiveContext:
    """Inputs of the live checks.

    Attributes:
        runner: Runs ``ros2`` commands; see :func:`default_runner`.
        command_topics: Velocity command topics that must have at most one publisher.
        watched_topics: Topics whose publishers must all run on one host.
        map_frame: Global frame of the localization transform.
        odom_frame: Odometry frame of the localization transform.
        transform_probe: Tells whether a transform exists, or ``None`` to skip.
        spin_time_sec: Discovery seconds of each query without the daemon.
    """

    runner: manifest.CommandRunner
    command_topics: Tuple[str, ...] = ()
    watched_topics: Tuple[str, ...] = ()
    map_frame: str = 'map'
    odom_frame: str = 'odom'
    transform_probe: Optional[TransformProbe] = None
    spin_time_sec: float = 3.0


def caching_runner(runner: manifest.CommandRunner) -> manifest.CommandRunner:
    """Wrap a runner so that a repeated command runs only once.

    Args:
        runner: Runner to wrap.

    Returns:
        A runner that replays the first result of each distinct command.
    """
    results: Dict[Tuple[str, ...], Tuple[int, str]] = {}

    def run(argv: Sequence[str]) -> Tuple[int, str]:
        key = tuple(argv)
        if key not in results:
            results[key] = runner(argv)
        return results[key]

    return run


def default_runner(timeout_sec: float = 30.0) -> manifest.CommandRunner:
    """Build the caching command runner of one doctor run.

    Args:
        timeout_sec: Seconds before one ``ros2`` call is abandoned.

    Returns:
        A caching runner around :func:`nav_bringup.recorder.manifest.run_command`.
    """
    return caching_runner(functools.partial(manifest.run_command, timeout_sec=timeout_sec))


def parse_daemon_status(output: str) -> Optional[bool]:
    """Parse the output of ``ros2 daemon status``.

    Args:
        output: Standard output of the command.

    Returns:
        Whether the daemon runs, or ``None`` for unrecognized output.
    """
    if 'is not running' in output:
        return False
    if 'is running' in output:
        return True
    return None


def parse_topic_info(output: str) -> Tuple[Optional[str], List[Endpoint]]:
    """Parse the output of ``ros2 topic info --verbose``.

    Args:
        output: Standard output of the command.

    Returns:
        ``(message type, endpoints)``; the type is ``None`` if it is absent.
    """
    topic_type = None
    endpoints: List[Endpoint] = []
    current: Dict[str, str] = {}

    def flush() -> None:
        if current.get('Node name') and current.get('Endpoint type'):
            endpoints.append(Endpoint(
                node_name=current['Node name'],
                node_namespace=current.get('Node namespace', '/'),
                endpoint_type=current['Endpoint type'],
                gid=current.get('GID', ''),
            ))
        current.clear()

    for line in output.splitlines():
        if line.startswith(' ') or ':' not in line:
            continue
        key, _, value = line.partition(':')
        key, value = key.strip(), value.strip()
        if key == 'Type' and topic_type is None:
            topic_type = value
        elif key == 'Node name':
            flush()
            current[key] = value
        elif key in ('Node namespace', 'Endpoint type', 'GID'):
            current[key] = value
    flush()
    return topic_type, endpoints


def gid_host(gid: str) -> Optional[str]:
    """Extract the host bytes of a Fast DDS GID.

    Fast DDS derives bytes 2 and 3 of the GUID prefix from the host, so all
    endpoints of one machine share them.

    Args:
        gid: Dot-separated hexadecimal bytes.

    Returns:
        The two host bytes, or ``None`` for GIDs of other DDS vendors.
    """
    parts = gid.split('.')
    if len(parts) < 4 or tuple(parts[:2]) != FAST_DDS_VENDOR:
        return None
    return '.'.join(parts[2:4])


def check_cli_daemon(context: LiveContext) -> CheckResult:
    """Compare the nodes seen through the ros2 CLI daemon with direct discovery.

    A daemon that kept an old environment silently returns an empty or
    partial graph, which once looked like "bt_navigator never came up".

    Args:
        context: Live check inputs.

    Returns:
        ``WARN`` if the daemon misses nodes, ``OK`` if it agrees or does not
        run, ``SKIP`` if the graph cannot be queried.
    """
    name = 'ros2_daemon'
    code, status = context.runner(['ros2', 'daemon', 'status'])
    running = parse_daemon_status(status) if code == 0 else None
    if running is None:
        return CheckResult(name, Status.SKIP, '无法查询 ros2 守护进程状态')
    if not running:
        return CheckResult(name, Status.OK, 'ros2 守护进程未运行（每条 ros2 命令各自发现节点）')
    daemon_code, via_daemon = context.runner(['ros2', 'node', 'list'])
    direct_code, direct = context.runner(
        ['ros2', 'node', 'list', '--no-daemon', '--spin-time', _spin(context)])
    if daemon_code != 0 or direct_code != 0:
        return CheckResult(name, Status.SKIP, '无法列出节点')
    seen = set(param_dump.parse_node_list(via_daemon)[0])
    discovered = param_dump.parse_node_list(direct)[0]
    missing = [node for node in discovered if node not in seen]
    if missing:
        return CheckResult(
            name, Status.WARN,
            f'守护进程看不到 {len(missing)}/{len(discovered)} 个节点：' + '，'.join(missing[:5]),
            '守护进程僵死或沿用了旧环境：执行 ros2 daemon stop，下一条 ros2 命令会用当前环境重建它')
    return CheckResult(name, Status.OK, f'守护进程与直接发现一致（{len(discovered)} 个节点）')


def check_command_publishers(context: LiveContext) -> CheckResult:
    """Check that each velocity command topic has at most one publisher.

    Args:
        context: Live check inputs.

    Returns:
        ``WARN`` for a topic with several publishers, ``SKIP`` without
        command topics, otherwise ``OK``.
    """
    name = 'command_publishers'
    if not context.command_topics:
        return CheckResult(name, Status.SKIP, '未指定速度指令话题（--cmd-topic）')
    problems = []
    summaries = []
    for topic in context.command_topics:
        endpoints = _endpoints(context, topic)
        if endpoints is None:
            summaries.append(f'{topic}：不存在')
            continue
        publishers = sorted(
            endpoint.qualified_node for endpoint in endpoints
            if endpoint.endpoint_type == 'PUBLISHER')
        summaries.append(f'{topic}：{len(publishers)} 个发布者')
        if len(publishers) > 1:
            problems.append(f'{topic} 有 {len(publishers)} 个发布者：' + '，'.join(publishers))
    if problems:
        return CheckResult(
            name, Status.WARN, '；'.join(problems),
            '遥控、导航或残留进程在同时给底盘发指令；关掉多余的发布者')
    return CheckResult(name, Status.OK, '；'.join(summaries))


def check_foreign_hosts(context: LiveContext) -> CheckResult:
    """Check that the publishers of watched topics run on one host.

    Args:
        context: Live check inputs.

    Returns:
        ``WARN`` if publishers come from several hosts, ``SKIP`` without
        topics or Fast DDS publishers, otherwise ``OK``.
    """
    name = 'foreign_hosts'
    topics = list(dict.fromkeys(context.watched_topics + context.command_topics))
    if not topics:
        return CheckResult(name, Status.SKIP, '未指定要核对的话题（--watch-topic / --cmd-topic）')
    hosts: Dict[str, List[str]] = {}
    for topic in topics:
        for endpoint in _endpoints(context, topic) or []:
            host = gid_host(endpoint.gid)
            if host is not None and endpoint.endpoint_type == 'PUBLISHER':
                hosts.setdefault(host, []).append(f'{topic}←{endpoint.qualified_node}')
    if not hosts:
        return CheckResult(name, Status.SKIP, '未发现 Fast DDS 发布者')
    if len(hosts) > 1:
        details = '；'.join(
            f"主机 {host}：{'，'.join(sorted(set(items)))}" for host, items in sorted(hosts.items()))
        return CheckResult(
            name, Status.WARN, f'发布者来自 {len(hosts)} 台主机（GID 主机字节）：{details}',
            '若不是本机器人的另一台计算机，说明有外部机器人或电脑混入同一 DDS 域；'
            '用 DDS 白名单或不同的 ROS_DOMAIN_ID 隔离')
    return CheckResult(name, Status.OK, f'发布者都来自同一主机（GID 主机字节 {next(iter(hosts))}）')


def check_map_to_odom(context: LiveContext) -> CheckResult:
    """Check whether localization provides the map to odom transform yet.

    Args:
        context: Live check inputs.

    Returns:
        ``WARN`` while the transform is missing, ``SKIP`` without a probe or
        when TF cannot be queried, otherwise ``OK``.
    """
    name = 'map_to_odom'
    transform = f'{context.map_frame}→{context.odom_frame}'
    if context.transform_probe is None:
        return CheckResult(name, Status.SKIP, '未启用 TF 查询')
    available = context.transform_probe(context.map_frame, context.odom_frame)
    if available is None:
        return CheckResult(name, Status.SKIP, '无法查询 TF')
    if not available:
        return CheckResult(
            name, Status.WARN, f'{transform} 变换不存在',
            '定位尚未完成：在 RViz 用 2D Pose Estimate 给出初始位姿（这只是提示）')
    return CheckResult(name, Status.OK, f'{transform} 变换存在')


ALL_LIVE_CHECKS: Tuple[Callable[[LiveContext], CheckResult], ...] = (
    check_cli_daemon,
    check_command_publishers,
    check_foreign_hosts,
    check_map_to_odom,
)


def run_live_checks(
    context: LiveContext,
    selected: Sequence[Callable[[LiveContext], CheckResult]] = ALL_LIVE_CHECKS,
) -> List[CheckResult]:
    """Run live checks, turning an unexpected error in a check into ``FAIL``.

    Args:
        context: Live check inputs.
        selected: Checks to run, in order.

    Returns:
        One result per check.
    """
    return checks.run_checks(context, selected)


def ros_transform_probe(wait_sec: float = 3.0) -> TransformProbe:
    """Build a transform probe that uses a short-lived ROS node.

    ROS is imported only when the probe runs.

    Args:
        wait_sec: Seconds to wait for the transform.

    Returns:
        A callable ``(parent, child)`` that returns whether the transform
        exists, or ``None`` if ROS is unavailable.
    """
    def probe(parent: str, child: str) -> Optional[bool]:
        try:
            import rclpy
            from rclpy.time import Time
            import tf2_ros
        except ImportError:
            return None
        rclpy.init()
        node = rclpy.create_node('navrun_doctor_tf_probe')
        listener: Any = None
        try:
            buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(buffer, node)
            deadline = time.monotonic() + wait_sec
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if buffer.can_transform(parent, child, Time()):
                    return True
            return False
        finally:
            del listener
            node.destroy_node()
            rclpy.try_shutdown()

    return probe


def _spin(context: LiveContext) -> str:
    return f'{context.spin_time_sec:g}'


def _endpoints(context: LiveContext, topic: str) -> Optional[List[Endpoint]]:
    """用不依赖 ROS daemon 的只读话题查询取得当前端点列表.

    命令执行由上下文提供的 runner 承担，环境与超时也由外层负责。

    Args:
        context: 诊断配置及命令执行端口，含发现等待秒数。
        topic: 要查询的完整话题名称。

    Returns:
        解析出的端点列表；命令非零退出返回 None，与成功但无端点区分。

    Raises:
        Exception: runner 或话题信息解析器发生未转换的错误。
    """
    code, output = context.runner(
        ['ros2', 'topic', 'info', '--verbose', '--no-daemon', '--spin-time', _spin(context),
         topic])
    if code != 0:
        return None
    return parse_topic_info(output)[1]
