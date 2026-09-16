"""Pure planar localization health observations; never own motion or localization."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Mapping, Optional


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


@dataclass(frozen=True)
class HealthPolicy:
    """Navigation-side warning policy; automatic stopping is uncommissioned TODO-H1.

    Attributes:
        max_tf_age_sec: Maximum age/future skew for paired transform/odometry samples.
        max_divergence_m: Strict displacement residual threshold in metres.
        divergence_window_sec: Maximum comparison window in ROS-clock seconds.
        action: Only warn is commissioned; stop must not silently change behavior.
    """

    max_tf_age_sec: float = 0.5
    max_divergence_m: float = 1.0
    divergence_window_sec: float = 2.0
    action: str = 'warn'

    def __post_init__(self) -> None:
        """Reject invalid policy before allocating observation resources.

        Raises:
            ValueError: Thresholds are not positive finite numbers or action is not warn.
        """
        if any(not _finite(value) or value <= 0 for value in (
                self.max_tf_age_sec, self.max_divergence_m, self.divergence_window_sec)):
            raise ValueError('health thresholds must be positive finite numbers')
        if self.action != 'warn':
            raise ValueError('only warn is commissioned; automatic stop is TODO-H1')


@dataclass(frozen=True)
class PoseSample:
    """One planar pose of the same robot reference point in either coordinate system.

    Attributes:
        stamp: ROS-clock source timestamp in seconds.
        x: Position in metres in that source's coordinate system.
        y: Position in metres in that source's coordinate system.
        yaw: Heading in radians in that source's coordinate system.
    """

    stamp: float
    x: float
    y: float
    yaw: float

    def __post_init__(self) -> None:
        """Validate observations before they enter a history or diagnostic message.

        Raises:
            ValueError: Any value is nonnumeric or nonfinite.
        """
        if not all(_finite(value) for value in (self.stamp, self.x, self.y, self.yaw)):
            raise ValueError('pose sample values must be finite numbers')


@dataclass(frozen=True)
class HealthReport:
    """An observation, not a stop request or a provider-fault attribution.

    Attributes:
        severity: info, warn or error; compatible with NavEvent.
        code: Stable event/diagnostic status name.
        values: Finite numerical evidence, copied into outbound observations.
    """

    severity: str
    code: str
    values: Mapping[str, float]


class HealthMonitor:
    """Compare paired pose increments in a bounded, caller-loop-confined history."""

    def __init__(self, policy: HealthPolicy) -> None:
        """Bind policy without ROS, clocks, threads, subscribers or commands.

        Args:
            policy: Validated warning policy.
        """
        self.policy = policy
        self._history = deque(maxlen=4096)
        self._last_now = None

    @property
    def history_size(self) -> int:
        """int: Number of retained paired observations, never more than 4096."""
        return len(self._history)

    def sample(
        self, now: float, localized: bool, map_pose: Optional[PoseSample],
        wheel_pose: Optional[PoseSample],
    ) -> HealthReport:
        """Evaluate fresh paired poses; stale data and clock jumps reset comparison.

        At the oldest retained pair, align the wheel axes using the heading
        difference. Compare both displacement vectors in that fixed orientation,
        not their absolute positions or scalar path lengths. The wheel pose is
        already integrated chassis odometry, transformed to the navigation point.
        Short startup windows are evaluated as soon as two pairs exist. This
        detects inconsistency, not which provider is correct or whether to stop.

        Args:
            now: Current ROS/simulation time in seconds, not an elapsed-wall clock.
            localized: Whether the configured map-to-odometry transform is available.
            map_pose: Latest map-to-navigation-point pose, or None.
            wheel_pose: Latest integrated wheel pose of that same point, or None.

        Returns:
            Finite evidence and the most urgent currently evaluable health status.

        Raises:
            ValueError: Current time or localization availability has an invalid type.
        """
        if not _finite(now) or type(localized) is not bool:
            raise ValueError('health time must be finite and localization state boolean')
        previous, self._last_now = self._last_now, now
        if previous is not None and now < previous:
            return self._reset('localization_clock_reset')
        if not localized:
            return self._reset('localization_waiting')
        evidence = {}
        for label, pose in (('tf', map_pose), ('odom', wheel_pose)):
            if pose is None:
                return self._reset('localization_' + label + '_missing', evidence)
            age = now - pose.stamp
            if not math.isfinite(age):
                return self._reset('localization_numeric_range', evidence)
            evidence[label + '_age_sec'] = age
            if age < -self.policy.max_tf_age_sec:
                return self._reset('localization_' + label + '_future', evidence)
            if age > self.policy.max_tf_age_sec:
                return self._reset('localization_' + label + '_stale', evidence)
        if previous is not None and now - previous > self.policy.divergence_window_sec:
            self._history.clear()
        cutoff = now - self.policy.divergence_window_sec
        while self._history and self._history[0][0] < cutoff:
            self._history.popleft()
        if self._history and self._history[-1][0] == now:
            self._history.pop()
        self._history.append((now, map_pose, wheel_pose))
        stamp, initial_map, initial_wheel = self._history[0]
        angle = math.remainder(initial_map.yaw, 2 * math.pi) - math.remainder(
            initial_wheel.yaw, 2 * math.pi)
        cosine, sine = math.cos(angle), math.sin(angle)
        dx, dy = wheel_pose.x - initial_wheel.x, wheel_pose.y - initial_wheel.y
        residual = math.hypot(map_pose.x - initial_map.x - (cosine * dx - sine * dy),
                              map_pose.y - initial_map.y - (sine * dx + cosine * dy))
        if not math.isfinite(residual):
            return self._reset('localization_numeric_range', evidence)
        evidence.update(divergence_m=residual, window_sec=now - stamp)
        if residual > self.policy.max_divergence_m:
            return HealthReport('error', 'localization_divergence', evidence)
        return HealthReport('info', 'localization_health_ok', evidence)

    def _reset(self, code, values=None):
        self._history.clear()
        return HealthReport('warn', code, dict(values or {}))
