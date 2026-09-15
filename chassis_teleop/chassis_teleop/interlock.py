"""Startup command-publisher exclusion, independent of any ROS process ownership."""

import math
from typing import Callable, Sequence


class CommandConflict(RuntimeError):
    """Existing publishers or incomplete graph visibility reject keyboard startup."""


def wait_for_clear_topics(
    topics: Sequence[str], query: Callable[[str], Sequence[object]],
    spin: Callable[[float], None], clock: Callable[[], float], duration_sec: float = 1.0,
) -> None:
    """Require an empty command graph throughout a bounded discovery window.

    The caller holds the navigation lock and has not created a velocity publisher.
    ROS discovery cannot rule out an uncoordinated process starting later; this
    check is not an atomic graph barrier and never exempts a publisher by its name.

    Args:
        topics: Resolved absolute command topics, including keyboard output.
        query: Read-only publisher-info query; failures reject startup.
        spin: Process graph discovery for at most the given duration.
        clock: Monotonic wall clock.
        duration_sec: Finite positive discovery window.

    Raises:
        ValueError: Topics or duration are invalid.
        CommandConflict: A publisher exists or discovery failed.
    """
    if (not topics or isinstance(topics, str) or any(
            not isinstance(topic, str) or not topic.startswith('/') for topic in topics)):
        raise ValueError('guard requires resolved absolute command topics')
    if (isinstance(duration_sec, bool) or not isinstance(duration_sec, (int, float))
            or not math.isfinite(duration_sec) or duration_sec <= 0.0):
        raise ValueError('discovery window must be finite and positive')
    deadline = clock() + duration_sec
    while True:
        for topic in dict.fromkeys(topics):
            try:
                publishers = query(topic)
                if publishers is None:
                    raise RuntimeError('missing publisher observation')
                count = len(publishers)
            except Exception as error:
                message = f'publisher discovery failed for {topic}: {error}'
                raise CommandConflict(message) from error
            if count:
                raise CommandConflict(f'command topic {topic} already has {count} publisher(s)')
        remaining = deadline - clock()
        if remaining <= 0.0:
            return
        spin(min(remaining, 0.05))
