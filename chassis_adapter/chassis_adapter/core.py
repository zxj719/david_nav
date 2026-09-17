"""Legacy-equivalent limit, deadzone and one-shot timeout logic without ROS."""

from dataclasses import asdict, dataclass
import math
from typing import Dict, Optional, Tuple, Union


@dataclass(frozen=True)
class AdapterConfig:
    """Node parameters; supplied profiles override these legacy standalone defaults.

    Attributes:
        input_cmd_vel_topic: Mode-selected command input.
        output_cmd_vel_topic: Adapted command output.
        cmd_vel_timeout: ROS-time silence timeout; nonpositive disables it.
        min_linear_x: Minimum nonzero forward/reverse command magnitude.
        max_linear_x: Linear magnitude ceiling.
        min_angular_z: Minimum angular magnitude while moving or below turn threshold.
        min_stationary_angular_z: Minimum deliberate stationary-turn magnitude.
        stationary_angular_threshold: Threshold selecting the stationary floor.
        max_angular_z: Angular magnitude ceiling.
        publish_hz: Requested timer frequency, floored at one hertz as in legacy.
    """

    input_cmd_vel_topic: str = 'cmd_vel_safe'
    output_cmd_vel_topic: str = '/cmd_vel'
    cmd_vel_timeout: float = 2.0
    min_linear_x: float = 0.20
    max_linear_x: float = 0.20
    min_angular_z: float = 0.0
    min_stationary_angular_z: float = 0.35
    stationary_angular_threshold: float = 0.025
    max_angular_z: float = 0.50
    publish_hz: float = 50.0

    def validate(self) -> None:
        """Reject malformed settings before constructing any ROS endpoints.

        Raises:
            ValueError: Topics are empty, numbers are nonfinite or limits are negative.
        """
        for key, value in asdict(self).items():
            if key.endswith('_topic'):
                if not isinstance(value, str) or not value.strip():
                    raise ValueError(f'{key} must be a nonempty topic')
            elif (isinstance(value, bool) or not isinstance(value, (int, float))
                  or not math.isfinite(value)):
                raise ValueError(f'{key} must be a finite number')
            elif key not in ('cmd_vel_timeout', 'publish_hz') and value < 0:
                raise ValueError(f'{key} must be nonnegative')

    @property
    def period_sec(self) -> float:
        """Return the timer period with the legacy one-hertz lower bound."""
        return 1.0 / max(self.publish_hz, 1.0)


def _clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    return max(min(float(value), limit), -limit)


def _lift(value: float, minimum: float) -> float:
    """把非零小幅命令提升到最小幅值，保留方向且零始终为零.

    本助手不执行上限裁剪；调用方负责先限幅，并保证最小值不超过上限。
    输入有限性由调用方负责，不在此验证。

    Args:
        value: 已限幅的线速度或角速度，与 minimum 使用相同单位。
        minimum: 最小非零幅值，使用其绝对值；零表示不提升。

    Returns:
        保留原方向的浮点命令，大于等于最小幅值时保持原值。
    """
    value = float(value)
    minimum = abs(float(minimum))
    if value == 0.0 or minimum <= 0.0 or abs(value) >= minimum:
        return value
    return minimum if value > 0.0 else -minimum


def shape_command(
    linear_x: float, linear_y: float, angular_z: float, config: AdapterConfig,
) -> Tuple[float, float]:
    """Apply the unchanged clamp-then-lift and raw-input stationary-turn decisions.

    Args:
        linear_x: Raw forward command in meters per second.
        linear_y: Raw lateral input, only used to select the angular floor.
        angular_z: Raw yaw command in radians per second.
        config: Validated limits and floors.

    Returns:
        Adapted forward and yaw velocities; all other Twist axes are zero.
    """
    minimum = config.min_angular_z
    if angular_z == 0.0:
        minimum = 0.0
    elif not (abs(linear_x) > 0.0 or abs(linear_y) > 0.0):
        if abs(angular_z) >= abs(config.stationary_angular_threshold):
            minimum = config.min_stationary_angular_z
    return (
        _lift(_clamp(linear_x, config.max_linear_x),
              min(config.min_linear_x, config.max_linear_x)),
        _lift(_clamp(angular_z, config.max_angular_z), min(minimum, config.max_angular_z)))


class AdapterState:
    """Latest command and one-shot stale transition, with no clock or I/O ownership.

    Attributes:
        config: 已校验的速度整形和超时参数。
        input_count: 已接收输入命令的累计次数。
        last_input: 最后一次输入的 ROS 时间（整数纳秒）；初始值刻意陈旧。
        latest: 最近整形后的前向速度（米/秒）和偏航速度（弧度/秒）。
    """

    def __init__(self, config: AdapterConfig, now: int) -> None:
        """Initialize the same stale startup state as the original adapter.

        Args:
            config: Validated adapter parameters.
            now: Initial ROS time in integer nanoseconds.
        """
        config.validate()
        self.config = config
        self.latest = (0.0, 0.0)
        self.last_input = now - 3600_000_000_000
        self.input_count = 0
        self._stale_stop_sent = True

    def receive(self, x: float, y: float, yaw: float, now: int) -> None:
        """Store one shaped command and reset the stale transition.

        Args:
            x: Raw forward velocity.
            y: Raw lateral velocity used by the legacy floor decision.
            yaw: Raw yaw velocity.
            now: Receipt ROS time in integer nanoseconds.
        """
        self.latest = shape_command(x, y, yaw, self.config)
        self.last_input = now
        self.input_count += 1
        self._stale_stop_sent = False

    def is_stale(self, now: int) -> bool:
        """Evaluate the strict greater-than ROS-time timeout.

        Args:
            now: Current ROS time in integer nanoseconds.

        Returns:
            False for a disabled timeout, otherwise whether input age exceeds it.
        """
        return (self.config.cmd_vel_timeout > 0.0
                and (now - self.last_input) / 1e9 > self.config.cmd_vel_timeout)

    def output(self, now: int) -> Optional[Tuple[float, float]]:
        """Return this timer's command, or None when the output must remain silent.

        Args:
            now: Current ROS time in integer nanoseconds.

        Returns:
            Latest command while fresh; one zero when becoming stale; then None.
        """
        if self.is_stale(now):
            if not self._stale_stop_sent:
                self._stale_stop_sent = True
                return (0.0, 0.0)
            return None
        return self.latest

    def diagnostics(self, now: int) -> Dict[str, Union[bool, int, float]]:
        """Describe limits and current command without consuming a transition.

        Args:
            now: Current ROS time in integer nanoseconds.

        Returns:
            Counters, timeout state, shaped output and configured clamp values.
        """
        stale = self.is_stale(now)
        x, yaw = (0.0, 0.0) if stale else self.latest
        return {
            'input_count': self.input_count, 'timed_out': stale,
            'output_linear_x': x, 'output_angular_z': yaw,
            'max_linear_x': self.config.max_linear_x,
            'max_angular_z': self.config.max_angular_z,
            'min_linear_x': self.config.min_linear_x,
            'min_angular_z': self.config.min_angular_z,
            'min_stationary_angular_z': self.config.min_stationary_angular_z,
        }
