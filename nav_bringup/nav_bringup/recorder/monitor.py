"""One ROS node that watches a navigation run (``navrun monitor``).

The recorder probes that need a ROS graph share this node, so a run costs a
single extra DDS participant (refactor plan §10.3, §13.2):

* behavior tree escape and recovery triggers, reported through
  :class:`~nav_bringup.recorder.bt_log_watcher.TriggerReporter` with the robot
  pose looked up in TF;
* message rates and publisher counts of key topics, and the lag of the map to
  robot transform, appended to ``metrics/topics.csv`` once per sampling
  period (:mod:`~nav_bringup.recorder.topic_stats`).

Counted topics are subscribed with best-effort QoS, which matches reliable
and best-effort publishers, and as raw bytes, so messages are never
deserialized. Choose light topics: heavy ones still cost transport.

ROS packages are imported only inside :func:`spin`, so the rest of
``nav_bringup`` and its unit tests do not need a ROS environment.
"""

from __future__ import annotations

from dataclasses import dataclass
import sys
import time
from typing import Any, List, Optional, Sequence, TextIO, Tuple

from nav_bringup.recorder import bt_log_watcher
from nav_bringup.recorder import bt_triggers
from nav_bringup.recorder import events
from nav_bringup.recorder import run_dir
from nav_bringup.recorder import topic_stats

DEFAULT_BT_TOPIC = '/behavior_tree_log'


@dataclass(frozen=True)
class MonitorOptions:
    """What the monitor watches.

    Attributes:
        base_frame: Robot frame for the pose and TF lag; from the robot
            profile (``frames.nav_base``), never defaulted (plan §5.4).
        map_frame: Global frame for the pose and TF lag.
        bt_topic: Behavior tree log topic, or ``None`` to skip triggers.
        topics: Topics whose rates and publishers are recorded.
        period_sec: Seconds between ``topics.csv`` samples.
        min_trigger_interval_sec: Minimum tree time between reports of one
            trigger rule.
        node_name: Name of the monitor node.
    """

    base_frame: str
    map_frame: str = 'map'
    bt_topic: Optional[str] = DEFAULT_BT_TOPIC
    topics: Tuple[str, ...] = ()
    period_sec: float = topic_stats.DEFAULT_PERIOD_SEC
    min_trigger_interval_sec: float = 0.5
    node_name: str = 'navrun_monitor'

    def __post_init__(self) -> None:
        """Validate the options.

        Raises:
            ValueError: If a frame is empty, the period is not positive or the
                trigger interval is negative.
        """
        if not self.base_frame or not self.map_frame:
            raise ValueError('map and base frames must not be empty')
        if self.period_sec <= 0:
            raise ValueError(f'period must be positive, got {self.period_sec}')
        if self.min_trigger_interval_sec < 0:
            raise ValueError('minimum trigger interval must not be negative')


def resolve_message_type(
    topic: str, names_and_types: Sequence[Tuple[str, Sequence[str]]],
) -> Optional[str]:
    """Find the message type of a topic in a graph listing.

    Args:
        topic: Topic name.
        names_and_types: Output of ``Node.get_topic_names_and_types()``.

    Returns:
        The first type advertised for the topic, or ``None`` if it is unknown.
    """
    for name, types in names_and_types:
        if name == topic and types:
            return types[0]
    return None


def spin(
    run: Optional[run_dir.RunDirectory],
    options: MonitorOptions,
    stdout: TextIO = sys.stdout,
) -> int:
    """Monitor until interrupted or ROS shuts down.

    Args:
        run: Run directory to record into, or ``None`` to only print triggers.
        options: What to watch.
        stdout: Terminal stream for trigger lines and warnings.

    Returns:
        Exit status 0.
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
        """Look up the latest map to robot transform.

        Returns:
            The transform, or ``None`` while it is unavailable.
        """
        try:
            return tf_buffer.lookup_transform(options.map_frame, options.base_frame, Time())
        except tf2_ros.TransformException:
            return None

    def lookup_pose() -> Optional[bt_triggers.Pose2D]:
        """Return the robot pose from TF.

        Returns:
            The pose, or ``None`` while the transform is unavailable.
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

    def warn(message: str) -> None:
        """Write a warning line to every sink.

        Args:
            message: Warning text.
        """
        for sink in sinks:
            sink.write(f'navrun monitor: {message}\n')
            sink.flush()

    def subscribe_discovered() -> None:
        """Subscribe to counted topics whose type has been discovered."""
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
                warn(f'cannot count {topic} ({type_name}): {error}')
                continue
            subscriptions.append(node.create_subscription(
                message_type, topic,
                lambda _message, name=topic: counter.record(name), qos, raw=True))

    def sample() -> None:
        """Append one sampling period to ``topics.csv``."""
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
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        del tf_listener
        node.destroy_node()
        rclpy.try_shutdown()
        if log_file is not None:
            log_file.close()
    return 0
