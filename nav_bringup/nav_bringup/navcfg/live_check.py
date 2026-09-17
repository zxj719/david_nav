"""Assess read-only runtime observations and wait for operator-owned localization."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Callable, Mapping, Optional, Tuple


@dataclass(frozen=True)
class Finding:
    """One operational finding, also suitable for a run event.

    Attributes:
        severity: info, warn or error.
        code: Stable event/check identifier.
        message: Human-readable evidence or operator hint.
    """

    severity: str
    code: str
    message: str


@dataclass(frozen=True)
class TopicRule:
    """Expected publisher multiplicity for one resolved topic.

    Attributes:
        topic: Absolute topic name.
        minimum: Required number of publishers.
        maximum: Maximum permitted publishers, or None without an upper bound.
        message_type: Expected ROS interface name, or empty to check only multiplicity.
    """

    topic: str
    minimum: int = 1
    maximum: Optional[int] = None
    message_type: str = ''


@dataclass(frozen=True)
class LiveSpec:
    """Inputs obtained from the prepared run, never from robot-specific literals.

    Attributes:
        map_frame: Global localization frame.
        odom_frame: Localization odometry frame.
        nav_frame: Navigation reference frame, distinct from the body frame.
        topics: Publisher-count contracts.
        parameters: Exact flattened rendered parameters, keyed by qualified node.
        max_tf_age_sec: Age above which the navigation transform is a warning.
    """

    map_frame: str
    odom_frame: str
    nav_frame: str
    topics: Tuple[TopicRule, ...]
    parameters: Mapping[str, Mapping[str, Any]]
    max_tf_age_sec: float = 0.5

    def __post_init__(self) -> None:
        """Reject malformed or unbounded configuration before any ROS sampling.

        Raises:
            ValueError: A frame, topic rule, parameter selector or threshold is invalid.
        """
        if not all(isinstance(frame, str) and frame.strip() for frame in (
                self.map_frame, self.odom_frame, self.nav_frame)):
            raise ValueError('live checks require explicit nonempty frames')
        if (isinstance(self.max_tf_age_sec, bool)
                or not isinstance(self.max_tf_age_sec, (int, float))
                or not math.isfinite(self.max_tf_age_sec) or self.max_tf_age_sec < 0):
            raise ValueError('max_tf_age_sec must be finite and nonnegative')
        for rule in self.topics:
            if (not rule.topic.startswith('/') or type(rule.minimum) is not int
                    or rule.minimum < 0 or (rule.maximum is not None and (
                        type(rule.maximum) is not int or rule.maximum < rule.minimum))):
                raise ValueError('invalid resolved topic publisher rule')
        if any(not name.startswith('/') or not isinstance(values, Mapping)
               for name, values in self.parameters.items()):
            raise ValueError('parameters require qualified node selectors and mappings')


@dataclass(frozen=True)
class LiveSnapshot:
    """Evidence from one bounded observation, with failed queries kept distinguishable.

    Attributes:
        localized: Whether the global-to-odometry transform can be queried.
        tf_age_sec: Navigation transform age, or None if absent.
        publishers: Observed topic publisher counts; missing keys mean unavailable.
        parameters: Successfully fetched parameter values; missing nodes/keys are failures.
        node_counts: Number of discovered nodes for each qualified name.
        topic_types: Discovered message types, including conflicting topic types.
    """

    localized: bool
    tf_age_sec: Optional[float]
    publishers: Mapping[str, int]
    parameters: Mapping[str, Mapping[str, Any]]
    node_counts: Mapping[str, int]
    topic_types: Mapping[str, Tuple[str, ...]] = field(default_factory=dict)


def _equal(expected: Any, actual: Any) -> bool:
    """比较参数类型和值，列表元素逐层核对以区分 bool 与 int.

    顶层类型必须相同；只对 list 特别递归，其它容器沿用自身等值运算。

    Args:
        expected: 期望的 ROS 参数值。
        actual: 实际读回的参数值。

    Returns:
        类型与当前规则下的值是否一致。
    """
    if type(expected) is not type(actual):
        return False
    if isinstance(expected, list):
        return len(expected) == len(actual) and all(
            _equal(left, right) for left, right in zip(expected, actual))
    return expected == actual


def assess(spec: LiveSpec, snapshot: LiveSnapshot) -> Tuple[Finding, ...]:
    """Compare the live graph with rendered inputs without changing either.

    Args:
        spec: Validated expected graph and flattened parameters.
        snapshot: Observations; absent evidence is never a successful match.

    Returns:
        All findings. Before localization, only an informational waiting hint.
    """
    if not snapshot.localized:
        return (Finding('info', 'localization_waiting', '等待在 RViz 手动给初始位姿'),)
    findings = []
    age = snapshot.tf_age_sec
    if age is None:
        findings.append(Finding('error', 'navigation_tf_missing',
                                f'cannot transform {spec.map_frame} <- {spec.nav_frame}'))
    elif not math.isfinite(age) or age < 0:
        findings.append(Finding('warn', 'navigation_tf_clock', f'invalid TF age: {age} s'))
    elif age > spec.max_tf_age_sec:
        findings.append(Finding('warn', 'navigation_tf_stale',
                                f'TF age {age:.3f} s exceeds {spec.max_tf_age_sec:g} s'))
    for rule in spec.topics:
        count = snapshot.publishers.get(rule.topic)
        if count is None or count < rule.minimum or (
                rule.maximum is not None and count > rule.maximum):
            findings.append(Finding('error', 'topic_publishers',
                                    f'{rule.topic}: observed {count}; expected '
                                    f'{rule.minimum}..{rule.maximum} publishers'))
        if rule.message_type and set(snapshot.topic_types.get(rule.topic, ())) != {
                rule.message_type}:
            findings.append(Finding('error', 'topic_type',
                                    f'{rule.topic}: expected only {rule.message_type}'))
    for node, expected in spec.parameters.items():
        count = snapshot.node_counts.get(node, 0)
        if count != 1:
            findings.append(Finding('error', 'node_identity',
                                    f'{node}: expected one node, observed {count}'))
            continue
        actual = snapshot.parameters.get(node)
        if actual is None:
            findings.append(Finding('error', 'parameters_unavailable',
                                    f'{node}: parameter query failed'))
            continue
        for name, value in expected.items():
            if name not in actual:
                findings.append(Finding('error', 'parameter_missing',
                                        f'{node}/{name}: rendered but not declared'))
            elif not _equal(value, actual[name]):
                # Avoid dumping complete values, which may include robot descriptions.
                findings.append(Finding('error', 'parameter_mismatch',
                                        f'{node}/{name}: runtime type or value differs'))
    return tuple(findings)


def wait_localization(
    probe: Callable[[], bool], spin: Callable[[float], None], clock: Callable[[], float],
    stopped: Callable[[], bool], emit: Callable[[Finding], None], timeout_sec: float = 0.0,
) -> bool:
    """Wait without initializing localization or imposing a default startup deadline.

    Args:
        probe: Read-only test for the global-to-odometry transform.
        spin: Discover ROS data or wait for at most the supplied seconds.
        clock: Monotonic wall clock for hint cadence and an optional diagnostic bound.
        stopped: Whether the caller is shutting down.
        emit: Print/record informational waiting and ready events.
        timeout_sec: Zero waits indefinitely; positive bounds a standalone diagnostic.

    Returns:
        True on localization availability, False on interruption or bounded pending wait.

    Raises:
        ValueError: The timeout is negative or non-finite.
    """
    if isinstance(timeout_sec, bool) or not math.isfinite(timeout_sec) or timeout_sec < 0:
        raise ValueError('localization wait timeout must be finite and nonnegative')
    started = clock()
    next_hint = started
    while not stopped():
        if probe():
            if stopped():
                return False
            emit(Finding('info', 'localization_ready', 'localization ready'))
            return True
        now = clock()
        if timeout_sec and now - started >= timeout_sec:
            return False
        if now >= next_hint:
            emit(Finding('info', 'localization_waiting', '等待在 RViz 手动给初始位姿'))
            next_hint = now + 5.0
        duration = min(0.2, next_hint - now)
        if timeout_sec:
            duration = min(duration, timeout_sec - (now - started))
        spin(duration)
    return False
