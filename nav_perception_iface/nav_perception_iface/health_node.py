"""Five-hertz navigation-side health observer; no IMU, motion or provider control."""

import json
import math
import sys
import time
from typing import Optional, Sequence
import uuid

from david_nav_msgs.msg import NavEvent
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from nav_perception_iface.health_core import HealthMonitor, HealthPolicy, PoseSample

_LEVELS = {'info': DiagnosticStatus.OK, 'warn': DiagnosticStatus.WARN,
           'error': DiagnosticStatus.ERROR}


def _stamp(value):
    return float(value.sec) + float(value.nanosec) * 1e-9


def _yaw(value):
    values = (value.x, value.y, value.z, value.w)
    if not all(type(item) in (int, float) and math.isfinite(item) for item in values):
        raise ValueError('orientation must be finite')
    norm = sum(item * item for item in values)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('orientation must have finite nonzero norm')
    return math.atan2(2 * (value.w * value.z + value.x * value.y),
                      norm - 2 * (value.y * value.y + value.z * value.z))


class LocalizationMonitor(Node):
    """Own only TF/odometry observation and diagnostic publication on one ROS node."""

    def __init__(self) -> None:
        """Read ROS parameters, validate warn-only policy, and create bounded observations.

        All frame/topic names come from prepared robot configuration. A failed
        construction destroys its own node; it never touches a provider process.

        Raises:
            ValueError: A required name is missing or health policy is invalid.
            RuntimeError: ROS entities cannot be constructed.
        """
        super().__init__('localization_monitor')
        self._closed = False
        self._destroyed = False
        self._listener = None
        self._wheel = None
        self._last_status = None
        self._last_localized = False
        self._next_event = 0.0
        self._next_failure = 0.0
        self._identity = uuid.uuid4().hex
        self._sequence = 0
        try:
            defaults = dict(map_frame='', odom_frame='', nav_frame='', chassis_odom_topic='',
                            chassis_odom_frame='',
                            max_tf_age_sec=0.5, max_divergence_m=1.0,
                            divergence_window_sec=2.0, action='warn')
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            values = {name: self.get_parameter(name).value for name in defaults}
            for name in ('map_frame', 'odom_frame', 'nav_frame', 'chassis_odom_topic',
                         'chassis_odom_frame'):
                if not isinstance(values[name], str) or not values[name].strip():
                    raise ValueError('health observation frame/topic must be explicit')
            self._frames = tuple(values[name] for name in ('map_frame', 'odom_frame', 'nav_frame'))
            self._wheel_frame = values['chassis_odom_frame']
            self._core = HealthMonitor(HealthPolicy(**{
                name: values[name] for name in (
                    'max_tf_age_sec', 'max_divergence_m', 'divergence_window_sec', 'action')}))
            self._buffer = Buffer()
            self._listener = TransformListener(self._buffer, self, spin_thread=False)
            self._diagnostics = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
            self._events = self.create_publisher(NavEvent, '/nav/events', 10)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Odometry, values['chassis_odom_topic'], self._odometry, qos)
            self.create_timer(0.2, self._sample)
        except BaseException:
            self.destroy_node()
            raise

    def _odometry(self, message):
        if self._closed:
            return
        try:
            if message.header.frame_id != self._wheel_frame or not message.child_frame_id:
                raise ValueError('wheel odometry frame does not match its configured frame')
            value = message.pose.pose
            yaw = _yaw(value.orientation)
            x, y = value.position.x, value.position.y
            if message.child_frame_id != self._frames[2]:
                offset = self._buffer.lookup_transform(
                    message.child_frame_id, self._frames[2], Time.from_msg(message.header.stamp))
                shift = offset.transform.translation
                x += math.cos(yaw) * shift.x - math.sin(yaw) * shift.y
                y += math.sin(yaw) * shift.x + math.cos(yaw) * shift.y
                yaw += _yaw(offset.transform.rotation)
            self._wheel = PoseSample(_stamp(message.header.stamp), x, y, yaw)
        except Exception:
            self._wheel = None  # Never keep a previous good sample after bad/new evidence.

    def _sample(self):
        if self._closed:
            return
        clock = self.get_clock().now()
        now = clock.nanoseconds * 1e-9
        try:
            localized = self._buffer.can_transform(self._frames[0], self._frames[1], Time())
        except Exception:
            localized = False
        pose = None
        if localized:
            try:
                transform = self._buffer.lookup_transform(self._frames[0], self._frames[2], Time())
                value = transform.transform
                pose = PoseSample(_stamp(transform.header.stamp), value.translation.x,
                                  value.translation.y, _yaw(value.rotation))
            except Exception:
                pass
        report = self._core.sample(now, bool(localized), pose, self._wheel)
        if localized and not self._last_localized:
            self._event('localization_ready', 'info', {})
        self._last_localized = bool(localized)
        status = (report.code, report.severity)
        if status != self._last_status or time.monotonic() >= self._next_event:
            self._event(report.code, report.severity, report.values)
            self._last_status = status
            self._next_event = time.monotonic() + 5.0
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = clock.to_msg()
        details = [KeyValue(key=key, value=str(value)) for key, value in report.values.items()]
        diagnostic.status = [DiagnosticStatus(
            level=_LEVELS[report.severity], name='localization_health', hardware_id='navigation',
            message=report.code, values=details)]
        try:
            self._diagnostics.publish(diagnostic)
        except Exception as error:
            self._publication_failed(error)

    def _event(self, name, severity, values):
        self._sequence += 1
        payload = dict(values, action='warn')
        try:
            stamp = time.time()
            if not math.isfinite(stamp):
                raise ValueError('event timestamp must be finite')
            self._events.publish(NavEvent(
                event_id='health:' + self._identity + ':' + str(self._sequence),
                stamp=stamp, source='localization_health', event_type=name,
                severity=severity, data_json=json.dumps(payload, allow_nan=False)))
        except Exception as error:
            self._publication_failed(error)
        message = '等待在 RViz 手动给初始位姿' if name == 'localization_waiting' else name
        logger = self.get_logger()
        # Humble fixes each logging call site's severity after its first use.
        # Keep separate sites so waiting -> ready or divergence -> recovery
        # changes the diagnostic level without terminating this observer.
        if severity == 'error':
            logger.error(message)
        elif severity == 'warn':
            logger.warning(message)
        else:
            logger.info(message)

    def _publication_failed(self, error):
        if time.monotonic() >= self._next_failure:
            self.get_logger().warning('health observation publication failed: ' +
                                      type(error).__name__)
            self._next_failure = time.monotonic() + 5.0

    def destroy_node(self) -> None:
        """Seal callbacks and remove only owned observation resources."""
        if self._destroyed:
            return
        self._closed = True
        try:
            if self._listener is not None:
                self._listener.unregister()
                self._listener = None
        finally:
            super().destroy_node()
        self._destroyed = True


def main(args: Optional[Sequence[str]] = None) -> int:
    """Run the navigation-side observer, with no automatic provider or motion control.

    Args:
        args: Standard ROS arguments; None uses process arguments.

    Returns:
        Zero for normal shutdown, two for configuration/runtime failure.
    """
    node = None
    try:
        rclpy.init(args=args)
        node = LocalizationMonitor()
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except Exception as error:
        print('localization health failed: ' + type(error).__name__, file=sys.stderr)
        return 2
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
