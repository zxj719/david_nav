"""用一个监控节点采集导航运行的行为树、话题、TF 与事件观测.

需要 ROS 图的观察共用 spin 创建的节点：行为树触发器查询最新 TF 位姿，
话题用 best-effort、raw 订阅计数，周期追加 metrics/topics.csv。
raw 不反序列化消息，但仍有传输开销，调用方应优先选择轻量话题。

可按动作状态话题的发布者数触发一次参数转储，子进程使用独立会话。
该图观察不证明动作服务可用、生命周期激活或导航健康。运行目录存在时，
同一节点还接收 /nav/events，并投射本地运行事件，通过身份前缀避免回写。

ROS 包在 spin 内导入；模块导入本身不创建 DDS 参与者。
资源清理有初始化及顺序异常边界，不能承诺任意失败下都完整关闭。
"""

from __future__ import annotations

from dataclasses import dataclass
import subprocess
import sys
import time
from typing import Any, List, Optional, Sequence, TextIO, Tuple

from nav_bringup.recorder import bt_log_watcher
from nav_bringup.recorder import bt_triggers
from nav_bringup.recorder import events
from nav_bringup.recorder.nav_events import NavEventRecorder
from nav_bringup.recorder.run_event_stream import RunEventStream
from nav_bringup.recorder import run_dir
from nav_bringup.recorder import sessions
from nav_bringup.recorder import topic_stats

DEFAULT_BT_TOPIC = '/behavior_tree_log'
EVENT_SOURCE = 'monitor'
DUMP_LOG_NAME = 'dump_params.log'
DUMP_STOP_TIMEOUT_SEC = 10.0


@dataclass(frozen=True)
class MonitorOptions:
    """保存监控话题、参考系、采样周期及参数转储选项.

    Attributes:
        base_frame: 位姿和 TF 时间差的机器人帧，必须显式从档案提供。
        map_frame: 位姿和 TF 时间差的全局参考系。
        bt_topic: 行为树日志话题，None 时不创建该订阅。
        topics: 要记录接收频率与发布者数的话题。
        period_sec: 周期采样的秒数，不保证实际回调严格按期执行。
        min_trigger_interval_sec: 同一触发规则的最小树时间报告间隔。
        node_name: 监控 ROS 节点名。
        ready_action: 用其状态话题观察首次发布者的动作名；None 关闭此检查。
        dump_params_on_ready: 首次观察到发布者且有运行目录时尝试启动参数转储。
    """

    base_frame: str
    map_frame: str = 'map'
    bt_topic: Optional[str] = DEFAULT_BT_TOPIC
    topics: Tuple[str, ...] = ()
    period_sec: float = topic_stats.DEFAULT_PERIOD_SEC
    min_trigger_interval_sec: float = 0.5
    node_name: str = 'navrun_monitor'
    ready_action: Optional[str] = None
    dump_params_on_ready: bool = False

    def __post_init__(self) -> None:
        """检查参考系、周期及参数转储的基本配置关系.

        不做 ROS 名称语法或数值有限性验证，空白字符串也不自动剥除。

        Raises:
            ValueError: 参考系为空、周期非正、触发间隔为负，
                或请求转储但未提供 ready_action。
        """
        if not self.base_frame or not self.map_frame:
            raise ValueError('map and base frames must not be empty')
        if self.period_sec <= 0:
            raise ValueError(f'period must be positive, got {self.period_sec}')
        if self.min_trigger_interval_sec < 0:
            raise ValueError('minimum trigger interval must not be negative')
        if self.dump_params_on_ready and not self.ready_action:
            raise ValueError('dumping parameters on ready needs a ready action')


def resolve_message_type(
    topic: str, names_and_types: Sequence[Tuple[str, Sequence[str]]],
) -> Optional[str]:
    """从图列表找到精确匹配话题的首个非空类型列表.

    Args:
        topic: 要匹配的话题名。
        names_and_types: get_topic_names_and_types 返回的名称和类型列表。

    Returns:
        首个匹配且非空类型列表的第一项；没有则为 None。
        不检测同名话题的类型冲突，也不验证消息类型可导入。
    """
    for name, types in names_and_types:
        if name == topic and types:
            return types[0]
    return None


def action_status_topic(action: str) -> str:
    """拼出动作状态话题名，不查询图或校验动作名称.

    标准动作服务端发布此话题；监控器只用发布者存在作为观察条件，
    不能把它等同于服务就绪、生命周期激活或发送目标能力。

    Args:
        action: 调用方提供的动作名，预期为完整名称。

    Returns:
        去掉结尾斜杠后拼接 /_action/status 的字符串。
    """
    return action.rstrip('/') + '/_action/status'


def dump_params_command(run: run_dir.RunDirectory, python: str = sys.executable) -> List[str]:
    """构造向指定运行目录转储参数的命令列表，不启动进程.

    Args:
        run: params/effective 接收转储结果的运行目录。
        python: 执行 navrun 模块的解释器路径。

    Returns:
        解释器及 navrun dump-params 的参数列表，不经 shell 展开。
    """
    return [python, '-m', 'nav_bringup.navrun.cli', 'dump-params', '--run-dir', str(run.path)]


def spin(
    run: Optional[run_dir.RunDirectory],
    options: MonitorOptions,
    stdout: TextIO = sys.stdout,
) -> int:
    """运行监控节点，直到被中断或 ROS 停止.

    使用默认 rclpy 上下文。初始化、文件打开、初次订阅和定时器创建有部分
    位于 try/finally 之前，失败不保证回滚。进入 finally 后按顺序清理，
    前一步异常仍可能阻止后续资源释放。参数转储停止的十秒仅为 SIGINT 等待，
    不是整个退出流程的硬时限；不接管定位或其它外部进程。

    Args:
        run: 记录目标目录；None 时不写运行文件，但仍可订阅并执行周期观察。
        options: 监控配置。
        stdout: 接收触发文本和提示的流，默认标准输出。

    Returns:
        spin 正常返回或捕获 KeyboardInterrupt/ExternalShutdownException，
        且清理完成后返回零；其它异常可传播，不保证总是返回状态码。
    """
    from nav2_msgs.msg import BehaviorTreeLog
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from rosidl_runtime_py.utilities import get_message
    import tf2_ros

    rclpy.init()
    node = rclpy.create_node(options.node_name)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)
    transform_name = f'{options.map_frame}->{options.base_frame}'

    def lookup() -> Any:
        """查询 map 到机器人帧的最新 TF，不检查新鲜度.

        Returns:
            查询到的变换；仅 tf2 TransformException 被转为 None，
            其它异常仍传播。
        """
        try:
            return tf_buffer.lookup_transform(options.map_frame, options.base_frame, Time())
        except tf2_ros.TransformException:
            return None

    def lookup_pose() -> Optional[bt_triggers.Pose2D]:
        """从最新 TF 的平移和四元数构造平面位姿.

        Returns:
            查询缺失时为 None，否则返回位姿。不归一化四元数、核验时间同步
            或吞掉变换对象字段访问异常。
        """
        transform = lookup()
        if transform is None:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = bt_triggers.yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        return bt_triggers.Pose2D(translation.x, translation.y, yaw, options.map_frame)

    sinks: List[TextIO] = [stdout]
    event_log = None
    writer = None
    log_file = None
    if run is not None:
        run.logs.mkdir(parents=True, exist_ok=True)
        run.metrics.mkdir(parents=True, exist_ok=True)
        log_file = (run.logs / bt_log_watcher.LOG_FILE_NAME).open('a', encoding='utf-8')
        sinks.append(log_file)
        event_log = events.EventLog(run.events)
        writer = topic_stats.MetricsCsvWriter(run.metrics / topic_stats.TOPICS_CSV_NAME)

    def say(message: str) -> None:
        """按顺序向所有输出流写提示并刷新.

        Args:
            message: 添加监控前缀和换行的文本。写入或刷新失败直接传播，
                不保证其余流仍收到该提示。
        """
        for sink in sinks:
            sink.write(f'navrun monitor: {message}\n')
            sink.flush()

    event_recorder = None
    event_stream = None

    if options.bt_topic:
        reporter = bt_log_watcher.TriggerReporter(
            bt_triggers.TriggerDetector(min_interval_sec=options.min_trigger_interval_sec),
            event_log, sinks, pose_provider=lookup_pose)
        node.create_subscription(
            BehaviorTreeLog, options.bt_topic,
            lambda msg: reporter.handle(bt_log_watcher.status_changes_from_msg(msg)), 10)

    counter = topic_stats.RateCounter(options.topics)
    pending = list(counter.topics)
    subscriptions: List[Any] = []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    ready_topic = action_status_topic(options.ready_action) if options.ready_action else None
    ready_seen: List[bool] = []
    dump_processes: List[subprocess.Popen] = []

    def subscribe_discovered() -> None:
        """为已发现类型的待订阅话题创建 raw best-effort 订阅.

        没有发现类型的项保留到下次；一旦发现即先移出 pending。
        类型导入的指定异常记录提示但不重新入队，订阅创建异常则传播；
        不支持运行中类型变化后的自动重建。
        """
        if not pending:
            return
        names_and_types = node.get_topic_names_and_types()
        for topic in list(pending):
            type_name = resolve_message_type(topic, names_and_types)
            if type_name is None:
                continue
            pending.remove(topic)
            try:
                message_type = get_message(type_name)
            except (AttributeError, ImportError, ValueError) as error:
                say(f'cannot count {topic} ({type_name}): {error}')
                continue
            subscriptions.append(node.create_subscription(
                message_type, topic,
                lambda _message, name=topic: counter.record(name), qos, raw=True))

    def check_ready() -> None:
        """首次观察到状态话题发布者时记录事件并可选启动转储.

        ready_seen 在事件写入和启动进程之前置位，失败不会自动重试。
        转储用自有独立会话；日志打开或 Popen 的 OSError 记录告警，
        其它异常直接传播。这里只观察发布者数量，不调用动作服务。
        """
        if ready_topic is None or ready_seen or node.count_publishers(ready_topic) == 0:
            return
        ready_seen.append(True)
        if event_log is not None:
            event_log.append(
                EVENT_SOURCE, 'ready_action_available', data={'action': options.ready_action})
        if not options.dump_params_on_ready or run is None:
            return
        say(f'{options.ready_action} is available; dumping effective parameters')
        try:
            with (run.logs / DUMP_LOG_NAME).open('ab') as log:
                dump_processes.append(subprocess.Popen(
                    dump_params_command(run), stdin=subprocess.DEVNULL, stdout=log,
                    stderr=subprocess.STDOUT, start_new_session=True))
        except OSError as error:
            say(f'cannot start the parameter dump: {error}')
            if event_log is not None:
                event_log.append(
                    EVENT_SOURCE, 'params_dump_start_failed', 'warn', {'error': str(error)})

    def sample() -> None:
        """依次投射事件、检查就绪、补订阅并生成话题/TF 周期行.

        样本 stamp 用墙钟，计数间隔用 RateCounter 的时钟，
        TF 时间差用节点 ROS 时钟与变换头时间；不构成同步快照。
        TF 查询缺失记为 None，只有配置了 writer 才追加 CSV；
        任何未处理异常都会中断本次回调，未写部分不回滚或补写。
        """
        if event_stream is not None:
            event_stream.pump()
        check_ready()
        subscribe_discovered()
        stamp = time.time()
        publishers = {topic: node.count_publishers(topic) for topic in counter.topics}
        rows: List[Any] = list(counter.sample(stamp, publishers))
        transform = lookup()
        lag = None
        if transform is not None:
            header = transform.header.stamp
            now_sec = node.get_clock().now().nanoseconds * 1e-9
            lag = topic_stats.transform_lag_ms(now_sec, header.sec + header.nanosec * 1e-9)
        rows.append(topic_stats.TfSample(stamp, transform_name, lag))
        if writer is not None:
            writer.write(rows)

    subscribe_discovered()
    node.create_timer(options.period_sec, sample)
    try:
        if event_log is not None:
            prefixes = ()
            try:
                from david_nav_msgs.msg import NavEvent

                publisher = node.create_publisher(NavEvent, '/nav/events', 10)
                event_stream = RunEventStream(
                    run.events, lambda values: publisher.publish(NavEvent(**values)), say)
                prefixes = (event_stream.identity_prefix,)
            except Exception as error:
                say(f'cannot publish navrun events ({type(error).__name__})')
            event_recorder = NavEventRecorder(event_log, say, ignored_prefixes=prefixes)
            try:
                from david_nav_msgs.msg import NavEvent

                node.create_subscription(NavEvent, '/nav/events', event_recorder.receive, 10)
            except Exception as error:
                event_recorder.close()
                say(f'cannot subscribe to navigation events ({type(error).__name__})')
        if event_stream is not None:
            event_stream.pump()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if event_stream is not None:
            event_stream.pump()
            event_stream.close()
        if event_recorder is not None:
            event_recorder.close()
        for process in dump_processes:
            sessions.stop_session(process, DUMP_STOP_TIMEOUT_SEC)
        del tf_listener
        node.destroy_node()
        rclpy.try_shutdown()
        if log_file is not None:
            log_file.close()
    return 0
