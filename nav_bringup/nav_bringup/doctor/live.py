"""通过可注入的命令与 TF 探针提供只读在线诊断（navrun doctor --live）.

检查 daemon 是否漏报直接发现的节点、指令话题是否有多个发布者，以及
Fast DDS GID 主机片段是否不同。后者只是启发式线索，不证明有外来机器人。
TF 探针只检查变换可用性，不测新鲜度；缺失时给提示，不直接认定定位故障。
命令缓存只反映本次首次查询，多个检查不组成同一时刻的 ROS 图快照。
不主动启停 daemon 或定位栈；调用方必须先加载正确的 ROS/DDS 环境。
测试注入命令结果和探针，不要求运行 ROS 图。
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
    """表示话题详情输出中的一个发布或订阅端点.

    Attributes:
        node_name: 不含命名空间的节点名称。
        node_namespace: 输出中的节点命名空间。
        endpoint_type: 端点类型，常见为 PUBLISHER 或 SUBSCRIPTION。
        gid: 原样点分 GID 文本，本对象不校验其完整性。
    """

    node_name: str
    node_namespace: str
    endpoint_type: str
    gid: str

    @property
    def qualified_node(self) -> str:
        """str: 去除命名空间末尾斜杠后拼接的节点名，不验证 ROS 名称语法."""
        return f"{self.node_namespace.rstrip('/')}/{self.node_name}"


@dataclass(frozen=True)
class LiveContext:
    """表示一轮在线诊断的端口、帧与话题配置，不在构造时查询图.

    Attributes:
        runner: 执行 ros2 命令并返回退出码/文本的端口，环境与超时由外层负责。
        command_topics: 检查发布者数量是否超过一的指令话题。
        watched_topics: 用 GID 主机片段比较发布来源的话题。
        map_frame: 定位变换的目标父帧。
        odom_frame: 定位变换的源子帧。
        transform_probe: 返回变换可用性或 None 的探针，缺省不查询。
        spin_time_sec: 无 daemon 查询的发现等待时间，不是整轮硬期限。
    """

    runner: manifest.CommandRunner
    command_topics: Tuple[str, ...] = ()
    watched_topics: Tuple[str, ...] = ()
    map_frame: str = 'map'
    odom_frame: str = 'odom'
    transform_probe: Optional[TransformProbe] = None
    spin_time_sec: float = 3.0


def caching_runner(runner: manifest.CommandRunner) -> manifest.CommandRunner:
    """按完整 argv 缓存首次返回的命令结果，不缓存抛出的异常.

    非零退出结果同样缓存；重复调用不反映后续图变化。缓存无并发同步或过期机制，
    应按一次串行诊断创建，不作为持续观察器。

    Args:
        runner: 被包装的同步命令端口。

    Returns:
        按参数元组复用首次返回值的闭包，异常后的同参数调用会重试。
    """
    results: Dict[Tuple[str, ...], Tuple[int, str]] = {}

    def run(argv: Sequence[str]) -> Tuple[int, str]:
        key = tuple(argv)
        if key not in results:
            results[key] = runner(argv)
        return results[key]

    return run


def default_runner(timeout_sec: float = 30.0) -> manifest.CommandRunner:
    """为一次诊断创建带命令超时的结果缓存端口.

    Args:
        timeout_sec: 传给 run_command 的单条命令超时秒数，不是整轮预算。

    Returns:
        包装 manifest.run_command 的缓存闭包，不在创建时执行命令。
    """
    return caching_runner(functools.partial(manifest.run_command, timeout_sec=timeout_sec))


def parse_daemon_status(output: str) -> Optional[bool]:
    """按既有英文子串识别 daemon 状态，优先判断未运行.

    Args:
        output: 命令返回文本，不在此检查退出码或执行 ROS 命令。

    Returns:
        识别到运行/未运行分别为 True/False，否则为 None。
    """
    if 'is not running' in output:
        return False
    if 'is running' in output:
        return True
    return None


def parse_topic_info(output: str) -> Tuple[Optional[str], List[Endpoint]]:
    """解析话题详情中的类型和端点，忽略缩进的 QoS 等内容.

    仅保留同时有节点名和端点类型的块，命名空间缺省为根、GID 缺省为空。
    不核对声明的端点总数、完整 GID 或节点名称是否合法。

    Args:
        output: ros2 topic info --verbose 的文本。

    Returns:
        首个 Type 字段和按出现次序排列的端点；缺少类型时第一项为 None。
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
    """按现有 Fast DDS 厂商前缀提取 GID 的第三、第四段作为主机线索.

    仅识别小写 01.0f 且至少四段的文本，不验证十六进制、完整长度或唯一性；
    相同片段不能证明同一主机，不同片段也不能直接证明外部机器人接入。

    Args:
        gid: 点分 GID 字符串，不在此归一化大小写。

    Returns:
        两段原文本，前缀或段数不符时为 None。
    """
    parts = gid.split('.')
    if len(parts) < 4 or tuple(parts[:2]) != FAST_DDS_VENDOR:
        return None
    return '.'.join(parts[2:4])


def check_cli_daemon(context: LiveContext) -> CheckResult:
    """比较直接发现的节点是否被已运行的 daemon 漏报.

    只检查直接发现减去 daemon 结果的差集，不检测 daemon 独有的过期节点，
    也不保证两次发现同时发生。这里只查询，不执行提示中的重启操作。

    Args:
        context: 命令端口与发现等待配置。

    Returns:
        漏报时 WARN，daemon 未运行或未发现漏报时 OK，查询失败时 SKIP；
        OK 不证明图完全一致或节点健康。
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
    """报告指令话题中多个发布端点，不判断实际指令流或底盘状态.

    Args:
        context: 命令端口、指令话题与发现等待配置。

    Returns:
        多发布者时 WARN，未配置话题时 SKIP，其余为 OK。
        查询失败沿用“不存在”摘要且不使结果失败；OK 不是单一控制权证明。
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
    """按发布端点的 GID 主机片段汇总来源，忽略订阅端点.

    监视与指令话题先按次序去重；查询失败或不能识别的 GID 不计入。
    不同来源只产生警告，不更改域号、白名单或任何进程。

    Args:
        context: 命令端口及要观察的话题。

    Returns:
        发现多个片段时 WARN，未配置话题或没有可识别发布端点时 SKIP，
        只有一个片段时 OK；结果不是网络隔离或主机归属的完整证明。
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
    """调用注入探针检查指定定位变换是否可用，不检查其时间戳.

    Args:
        context: 父/子帧名称及可选变换探针。

    Returns:
        变换缺失为 WARN，未配置探针或探针返回 None 为 SKIP，存在为 OK。
        探针异常由外层 run_live_checks 统一转换，本函数不自行吞掉异常。
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
    """按顺序执行诊断，将单项普通异常隔离为 FAIL 结果.

    Args:
        context: 共享诊断上下文，缓存端口可能复用先前查询结果。
        selected: 要执行的检查序列，缺省为全部在线检查。

    Returns:
        每项检查对应的结果列表，不因单项普通异常停止其余检查。
    """
    return checks.run_checks(context, selected)


def ros_transform_probe(wait_sec: float = 3.0) -> TransformProbe:
    """构造使用短命 ROS 节点的 TF 可用性探针，调用时才导入 ROS.

    使用默认 context；外层须避免重复初始化冲突并加载正确 DDS 环境。
    等待预算仅覆盖查询循环，不能抢占同步初始化、spin 或清理操作。
    不创建定位提供者，也不保证变换新鲜或任意异常路径完整释放资源。

    Args:
        wait_sec: 查询循环的单调时钟预算秒数。

    Returns:
        接受父/子帧名的闭包；观察到变换为 True，到期为 False，
        仅导入依赖失败返回 None，其余初始化/查询/清理异常继续传播。
    """
    def probe(parent: str, child: str) -> Optional[bool]:
        """临时驱动 ROS TF 查询，返回可用性；缺少导入依赖时返回 None.

        初始化默认 context 并创建短命节点，不启停定位进程或 daemon。
        外层须提供正确 DDS 环境且避免已有默认 context 冲突。查询体退出时
        依次释放 listener、节点和 context；初始化/节点构造早于 try，
        销毁节点失败也可能阻止 try_shutdown，不能承诺任意异常完整回收。

        Args:
            parent: 目标父帧。
            child: 查询的源子帧。

        Returns:
            预算内观察到变换为 True，到期未观察到为 False，导入失败为 None。
            spin_once 或同步查询不可抢占，预算不包含初始化与最终清理。

        Raises:
            Exception: 初始化、节点创建、TF 查询或清理异常。
        """
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
