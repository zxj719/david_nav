"""Topic rates and TF lag sampled during a run (``metrics/topics.csv``).

A monitor node counts the messages of key topics and measures the age of the
latest map to robot transform. Every sampling period (5 s, that is 0.2 Hz,
refactor plan §13.2) it appends one row per topic and one per transform, and
the run summary reports rates and TF lag percentiles. Both exposed real
problems before: a topic without publishers looks like "the robot ignores
commands", and delayed TF made the controller replay stale commands.

This module is pure Python; the ROS subscriber lives in
:mod:`nav_bringup.recorder.monitor`.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from typing import Callable, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple, Union

TOPICS_CSV_NAME = 'topics.csv'
CSV_COLUMNS = ('stamp', 'kind', 'name', 'messages', 'rate_hz', 'publishers', 'lag_ms')
DEFAULT_PERIOD_SEC = 5.0


@dataclass(frozen=True)
class TopicSample:
    """Message rate of one topic over one sampling period.

    Attributes:
        stamp: Wall-clock time of the sample in seconds since the epoch.
        name: Topic name.
        messages: Messages received during the period.
        rate_hz: ``messages`` divided by the period length.
        publishers: Publishers of the topic at sampling time, or ``None`` if unknown.
    """

    stamp: float
    name: str
    messages: int
    rate_hz: float
    publishers: Optional[int] = None


@dataclass(frozen=True)
class TfSample:
    """Age of the latest transform between two frames.

    Attributes:
        stamp: Wall-clock time of the sample in seconds since the epoch.
        name: ``<parent frame>-><child frame>``.
        lag_ms: Sampling time minus transform stamp in milliseconds, or
            ``None`` while the transform is unavailable.
    """

    stamp: float
    name: str
    lag_ms: Optional[float]


@dataclass(frozen=True)
class TopicStats:
    """Rates of one topic over a run.

    Attributes:
        name: Topic name.
        samples: Number of sampling periods.
        rate_avg_hz: Mean rate over the periods.
        rate_min_hz: Lowest rate of a period.
        silent_samples: Periods without any message.
        publishers_max: Most publishers seen at once, or ``None`` if never known.
    """

    name: str
    samples: int
    rate_avg_hz: float
    rate_min_hz: float
    silent_samples: int
    publishers_max: Optional[int]


@dataclass(frozen=True)
class TfStats:
    """Lag of one transform over a run.

    Attributes:
        name: ``<parent frame>-><child frame>``.
        samples: Number of sampling periods.
        missing_samples: Periods in which the transform was unavailable.
        lag_p50_ms: Median lag, or ``None`` without measurements.
        lag_p95_ms: 95th percentile lag, or ``None`` without measurements.
        lag_max_ms: Largest lag, or ``None`` without measurements.
    """

    name: str
    samples: int
    missing_samples: int
    lag_p50_ms: Optional[float]
    lag_p95_ms: Optional[float]
    lag_max_ms: Optional[float]


@dataclass(frozen=True)
class TopicsSummary:
    """Aggregated ``topics.csv``.

    Attributes:
        topics: Per-topic statistics in order of first appearance.
        transforms: Per-transform statistics in order of first appearance.
        skipped_rows: Rows that could not be parsed.
    """

    topics: Tuple[TopicStats, ...] = ()
    transforms: Tuple[TfStats, ...] = ()
    skipped_rows: int = 0


class RateCounter:
    """Counts messages per topic between samples; safe to share between threads."""

    def __init__(self, topics: Sequence[str], clock: Callable[[], float] = time.monotonic) -> None:
        """Create a counter whose first period starts now.

        Args:
            topics: Topics to count, in report order; duplicates are ignored.
            clock: Monotonic time source, replaceable in tests.
        """
        self._topics = list(dict.fromkeys(topics))
        self._clock = clock
        self._lock = threading.Lock()
        self._counts: Dict[str, int] = {topic: 0 for topic in self._topics}
        self._period_start = clock()

    @property
    def topics(self) -> List[str]:
        """List[str]: Counted topics in report order."""
        return list(self._topics)

    def record(self, topic: str) -> None:
        """Count one message.

        Args:
            topic: Topic the message arrived on; topics not counted are ignored.
        """
        with self._lock:
            if topic in self._counts:
                self._counts[topic] += 1

    def sample(
        self, stamp: float, publishers: Optional[Mapping[str, int]] = None,
    ) -> List[TopicSample]:
        """Close the current period and start the next one.

        Args:
            stamp: Wall-clock time to put on the samples.
            publishers: Publisher count per topic, if known.

        Returns:
            One sample per topic in report order.
        """
        with self._lock:
            now = self._clock()
            elapsed = now - self._period_start
            counts = self._counts
            self._counts = {topic: 0 for topic in self._topics}
            self._period_start = now
        known = publishers or {}
        return [
            TopicSample(
                stamp=stamp,
                name=topic,
                messages=counts[topic],
                rate_hz=counts[topic] / elapsed if elapsed > 0 else 0.0,
                publishers=known.get(topic),
            )
            for topic in self._topics
        ]


class MetricsCsvWriter:
    """Appends samples to ``topics.csv``; safe to share between threads."""

    def __init__(self, path: Union[str, Path]) -> None:
        """Create a writer.

        Args:
            path: The CSV file; its directory must exist.
        """
        self._path = Path(path)
        self._lock = threading.Lock()

    @property
    def path(self) -> Path:
        """Path: The CSV file."""
        return self._path

    def write(self, samples: Iterable[Union[TopicSample, TfSample]]) -> int:
        """Append samples, writing the header first if the file is new or empty.

        Args:
            samples: Samples to append.

        Returns:
            The number of rows written.
        """
        rows = [_row(sample) for sample in samples]
        with self._lock:
            new_file = not self._path.exists() or self._path.stat().st_size == 0
            with self._path.open('a', encoding='utf-8', newline='') as handle:
                writer = csv.writer(handle, lineterminator='\n')
                if new_file:
                    writer.writerow(CSV_COLUMNS)
                writer.writerows(rows)
        return len(rows)


def transform_lag_ms(now_sec: float, transform_stamp_sec: float) -> float:
    """Compute the age of a transform.

    Args:
        now_sec: Current time in seconds, on the clock of the stamps.
        transform_stamp_sec: Header stamp of the transform in seconds.

    Returns:
        The age in milliseconds rounded to 0.1 ms; negative when the stamp lies
        in the future, which indicates clock skew between machines.
    """
    return round((now_sec - transform_stamp_sec) * 1000.0, 1)


def percentile(values: Sequence[float], fraction: float) -> float:
    """Compute a percentile with linear interpolation between ranks.

    Args:
        values: Measurements in any order.
        fraction: Percentile as a fraction, for example ``0.95``.

    Returns:
        The interpolated percentile.

    Raises:
        ValueError: If ``values`` is empty or ``fraction`` is outside ``[0, 1]``.
    """
    if not values:
        raise ValueError('percentile of no values')
    if not 0.0 <= fraction <= 1.0:
        raise ValueError(f'fraction must be within [0, 1], got {fraction}')
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def summarize_topics(path: Union[str, Path]) -> TopicsSummary:
    """Aggregate a ``topics.csv`` file.

    Args:
        path: The CSV file. A missing file yields an empty summary.

    Returns:
        Per-topic rates and per-transform lag percentiles.
    """
    file_path = Path(path)
    if not file_path.is_file():
        return TopicsSummary()
    rates: Dict[str, List[float]] = {}
    silent: Dict[str, int] = {}
    publishers: Dict[str, List[int]] = {}
    tf_samples: Dict[str, int] = {}
    lags: Dict[str, List[float]] = {}
    skipped = 0
    with file_path.open('r', encoding='utf-8', newline='') as handle:
        for row in csv.DictReader(handle):
            try:
                kind, name = row['kind'], row['name']
                if kind == 'topic':
                    rate = float(row['rate_hz'])
                    messages = int(row['messages'])
                    count = int(row['publishers']) if row['publishers'] else None
                    rates.setdefault(name, []).append(rate)
                    silent[name] = silent.get(name, 0) + (1 if messages == 0 else 0)
                    if count is not None:
                        publishers.setdefault(name, []).append(count)
                elif kind == 'tf':
                    lag = float(row['lag_ms']) if row['lag_ms'] else None
                    tf_samples[name] = tf_samples.get(name, 0) + 1
                    if lag is not None:
                        lags.setdefault(name, []).append(lag)
                else:
                    skipped += 1
            except (KeyError, TypeError, ValueError):
                skipped += 1
    topics = tuple(
        TopicStats(
            name=name,
            samples=len(values),
            rate_avg_hz=sum(values) / len(values),
            rate_min_hz=min(values),
            silent_samples=silent.get(name, 0),
            publishers_max=max(publishers[name]) if name in publishers else None,
        )
        for name, values in rates.items()
    )
    transforms = tuple(
        TfStats(
            name=name,
            samples=count,
            missing_samples=count - len(lags.get(name, [])),
            lag_p50_ms=percentile(lags[name], 0.5) if name in lags else None,
            lag_p95_ms=percentile(lags[name], 0.95) if name in lags else None,
            lag_max_ms=max(lags[name]) if name in lags else None,
        )
        for name, count in tf_samples.items()
    )
    return TopicsSummary(topics=topics, transforms=transforms, skipped_rows=skipped)


def _row(sample: Union[TopicSample, TfSample]) -> List[object]:
    """按冻结的 CSV 列布局序列化话题或 TF 采样，缺失字段留空.

    时间戳和话题频率保留三位小数，TF 延迟保留一位；
    话题计数与 TF 延迟占不同列，不能把两种样本混作同一量。

    Args:
        sample: 已校验的 TopicSample 或 TfSample，不在此判定新鲜度。

    Returns:
        按时间、类型、名称、计数、频率、发布者数、延迟排列的七列值。
    """
    stamp = f'{sample.stamp:.3f}'
    if isinstance(sample, TopicSample):
        publishers = '' if sample.publishers is None else sample.publishers
        return [stamp, 'topic', sample.name, sample.messages, f'{sample.rate_hz:.3f}',
                publishers, '']
    lag = '' if sample.lag_ms is None else f'{sample.lag_ms:.1f}'
    return [stamp, 'tf', sample.name, '', '', '', lag]
