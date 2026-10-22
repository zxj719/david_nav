"""五赫兹导航侧健康观察节点，不读取 IMU，也不控制运动或定位提供方.

只订阅配置的轮式里程计并查询 TF，发布诊断与只告警策略的事件。
"""

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

from nav_perception_iface.health_core import (
    HealthMonitor, HealthPolicy, MapOdomLagMonitor, PoseSample)

_LEVELS = {'info': DiagnosticStatus.OK, 'warn': DiagnosticStatus.WARN,
           'error': DiagnosticStatus.ERROR}


def _stamp(value):
    return float(value.sec) + float(value.nanosec) * 1e-9


def _yaw(value):
    """从有限且非退化的四元数计算导航健康观察所需的 yaw.

    使用平方范数修正公式，不要求输入单位化；拒绝平方范数小于 1e-12
    或溢出的输入。只做数值转换，不操作定位提供方。

    Args:
        value: 含 x、y、z、w 数值字段的四元数。

    Returns:
        atan2 给出的偏航角，单位弧度。

    Raises:
        ValueError: 分量非有限、类型不符，或平方范数过小/非有限。
        OverflowError: 超大整数在有限性检查时无法转换。
    """
    values = (value.x, value.y, value.z, value.w)
    if not all(type(item) in (int, float) and math.isfinite(item) for item in values):
        raise ValueError('orientation must be finite')
    norm = sum(item * item for item in values)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('orientation must have finite nonzero norm')
    return math.atan2(2 * (value.w * value.z + value.x * value.y),
                      norm - 2 * (value.y * value.y + value.z * value.z))


class LocalizationMonitor(Node):
    """仅持有 TF、里程计观察与诊断发布资源的导航节点."""

    def __init__(self) -> None:
        """读取显式 ROS 参数，校验只告警策略并创建有界观察资源.

        帧与话题名称来自准备好的机器人配置。父类构造之后的资源创建失败会尝试
        销毁自己的节点，不触碰定位提供方；清理失败可能覆盖原创建错误。

        Raises:
            ValueError: 必需名称为空或健康策略不合法。
            Exception: ROS 实体创建或本节点清理失败。
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
        self._lag_warn = None
        try:
            defaults = dict(map_frame='', odom_frame='', nav_frame='', chassis_odom_topic='',
                            chassis_odom_frame='',
                            max_tf_age_sec=0.5, max_divergence_m=1.0,
                            divergence_window_sec=2.0, action='warn',
                            max_map_odom_lag_sec=0.3, map_odom_lag_samples=25)
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            values = {name: self.get_parameter(name).value for name in defaults}
            for name in ('map_frame', 'odom_frame', 'nav_frame', 'chassis_odom_topic',
                         'chassis_odom_frame'):
                if not isinstance(values[name], str) or not values[name].strip():
                    raise ValueError('health observation frame/topic must be explicit')
            self._frames = tuple(values[name] for name in ('map_frame', 'odom_frame', 'nav_frame'))
            self._wheel_frame = values['chassis_odom_frame']
            policy = HealthPolicy(**{name: values[name] for name in (
                'max_tf_age_sec', 'max_divergence_m', 'divergence_window_sec', 'action',
                'max_map_odom_lag_sec', 'map_odom_lag_samples')})
            self._core = HealthMonitor(policy)
            self._lag = MapOdomLagMonitor(policy)
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
        """把轮式里程计样本换算到配置的导航基准，坏样本立即清空缓存.

        Args:
            message: 轮式里程计消息，位置为米，时间为 ROS 秒，角度由四元数
                求得；子帧不同于导航基准时使用该样本时间的 TF 偏移。

        关闭后忽略消息；帧不符、转换不可用或数值转换异常不保留旧好样本。
        此回调只观察，不修改发布方或 Lightning/uslam 的话题接线。
        """
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
        """以当前 ROS 时间采样健康状态，只发布诊断和告警，不执行停车或重定位.

        定位就绪转换时单独发事件；状态变化立即上报，稳定状态按单调时钟
        最多每五秒重报一次。诊断消息每次定时回调发布，失败走节流告警。
        TF 查询失败按不可用样本处理，不伪造健康状态。map→odom 滞后是第二个
        诊断状态，只在进入告警和首次恢复时发事件，不改变主报告。

        Raises:
            Exception: 健康核心、消息构造或日志操作失败；诊断发布异常由
                _publication_failed 处理，不操作定位或运动资源。
        """
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
        edge_stamp = None
        if localized:
            try:
                edge = self._buffer.lookup_transform(self._frames[0], self._frames[1], Time())
                edge_stamp = _stamp(edge.header.stamp)
            except Exception:
                pass
        report = self._core.sample(now, bool(localized), pose, self._wheel)
        lag = self._lag.sample(now, edge_stamp)
        if localized and not self._last_localized:
            self._event('localization_ready', 'info', {})
        self._last_localized = bool(localized)
        status = (report.code, report.severity)
        if status != self._last_status or time.monotonic() >= self._next_event:
            self._event(report.code, report.severity, report.values)
            self._last_status = status
            self._next_event = time.monotonic() + 5.0
        if lag.severity != 'info':
            if lag.code != self._lag_warn:
                self._event(lag.code, lag.severity, lag.values)
                self._lag_warn = lag.code
        elif lag.code == 'map_odom_lag_ok' and self._lag_warn is not None:
            self._event('map_odom_lag_recovered', 'info', lag.values)
            self._lag_warn = None
        diagnostic = DiagnosticArray()
        diagnostic.header.stamp = clock.to_msg()
        diagnostic.status = [DiagnosticStatus(
            level=_LEVELS[item.severity], name=name, hardware_id='navigation',
            message=item.code,
            values=[KeyValue(key=key, value=str(value)) for key, value in item.values.items()])
            for name, item in (('localization_health', report),
                               ('localization_map_odom_lag', lag))]
        try:
            self._diagnostics.publish(diagnostic)
        except Exception as error:
            self._publication_failed(error)

    def _event(self, name, severity, values):
        """发布带唯一递增标识的 warn 策略事件，并在独立日志调用点输出等级.

        Args:
            name: 健康事件名称；等待定位时日志提示人工初始化。
            severity: info、warn 或 error 的事件等级，不改变只告警策略。
            values: 观察数据映射，序列化时拒绝非有限数值。

        时间戳使用墙钟秒；序列化/发布失败只节流报告，不回滚序号或重发。
        不同日志级别使用独立调用位置以满足 Humble 日志器约束。

        Raises:
            Exception: 数据映射构造或日志操作失败，发布异常本身会被捕获。
        """
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
        """按单调时钟限频记录健康观察发布失败，不重发事件或控制定位.

        仅报告异常类型；一次成功写出警告后，至少五秒才允许下一次警告。

        Args:
            error: 发布失败时捕获的异常对象。
        """
        if time.monotonic() >= self._next_failure:
            self.get_logger().warning('health observation publication failed: ' +
                                      type(error).__name__)
            self._next_failure = time.monotonic() + 5.0

    def destroy_node(self) -> None:
        """封闭回调，注销自有 TF 监听器并调用父类销毁节点.

        即使注销失败，finally 仍调用父类销毁；仅在整个路径未抛异常时设置已销毁标志，
        但不核对父类的返回值。失败后的再次调用不等于全部资源已成功释放的证明。

        Raises:
            Exception: 监听器注销或父类节点销毁失败。
        """
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
    """运行导航侧只读观察器，不自动控制定位提供方或运动.

    主体错误只输出异常类型；finally 尝试销毁本节点并关闭本进程 ROS 上下文，
    不信号或关闭其它进程。

    Args:
        args: 标准 ROS 参数；None 使用进程参数。

    Returns:
        正常结束、键盘中断或外部 ROS 关闭返回零；
        主体中捕获的配置或运行错误返回 2。

    Raises:
        Exception: 最终清理失败，可能覆盖主体的返回码。
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
