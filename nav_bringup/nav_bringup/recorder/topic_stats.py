"""计数并汇总运行期间的话题接收频率与 TF 时间差.

监控节点提供话题计数、发布者数和最新 TF 的观测，本模块负责按周期
生成样本、追加 metrics/topics.csv 及计算摘要，不导入 ROS。
接收频率不是发布端发送频率；缺消息、缺发布者或 TF 时间差均只作观测，
不能单独证明导航健康或定位故障。数据类本身不执行字段或有限性校验。
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
    """保存一个话题采样周期的接收统计.

    Attributes:
        stamp: 样本标签的 Unix 墙钟时间，单位秒，不用于计算周期长度。
        name: 话题名称。
        messages: 本周期收到的消息数。
        rate_hz: 消息数除以周期秒数；RateCounter 对非正周期返回零。
        publishers: 采样时外部提供的发布者数；未知为 None，不等同零。
    """

    stamp: float
    name: str
    messages: int
    rate_hz: float
    publishers: Optional[int] = None


@dataclass(frozen=True)
class TfSample:
    """保存一次变换时间差观测.

    Attributes:
        stamp: 采样标签的 Unix 墙钟时间，单位秒。
        name: 变换名称，通常为 <父帧>-><子帧>。
        lag_ms: ROS 采样时刻减变换时间戳的毫秒数；变换未知为 None。
    """

    stamp: float
    name: str
    lag_ms: Optional[float]


@dataclass(frozen=True)
class TopicStats:
    """保存一个话题的有效 CSV 行统计.

    Attributes:
        name: 话题名称。
        samples: 成功解析的采样行数，不含跳过的坏行。
        rate_avg_hz: 各行频率的算术均值，不按周期时长加权。
        rate_min_hz: 所有采样行的最小频率。
        silent_samples: 消息数恰为零的采样行数。
        publishers_max: 已知发布者数的最大值；全部未知为 None。
    """

    name: str
    samples: int
    rate_avg_hz: float
    rate_min_hz: float
    silent_samples: int
    publishers_max: Optional[int]


@dataclass(frozen=True)
class TfStats:
    """保存一个变换的有效 CSV 行数与延迟分布.

    Attributes:
        name: 变换名称，通常为 <父帧>-><子帧>。
        samples: 成功解析的 TF 行数，含延迟空白的行。
        missing_samples: 延迟空白的行数，不含解析失败而跳过的行。
        lag_p50_ms: 已知延迟的中位数；没有测量为 None。
        lag_p95_ms: 已知延迟的线性插值 95 分位数；没有测量为 None。
        lag_max_ms: 已知延迟最大值；没有测量为 None。
    """

    name: str
    samples: int
    missing_samples: int
    lag_p50_ms: Optional[float]
    lag_p95_ms: Optional[float]
    lag_max_ms: Optional[float]


@dataclass(frozen=True)
class TopicsSummary:
    """保存 topics.csv 的按名聚合结果.

    Attributes:
        topics: 按各话题首次有效出现顺序排列的统计。
        transforms: 按各变换首次有效出现顺序排列的统计。
        skipped_rows: 类型未知或字段转换失败等被跳过的行数。
    """

    topics: Tuple[TopicStats, ...] = ()
    transforms: Tuple[TfStats, ...] = ()
    skipped_rows: int = 0


class RateCounter:
    """用实例内线程锁累计话题计数并切换采样周期.

    record 与 sample 共享计数锁；外部时钟和发布者映射仍由调用方负责，
    这里不建立 ROS 图或与发布者查询一致的原子快照。
    """

    def __init__(self, topics: Sequence[str], clock: Callable[[], float] = time.monotonic) -> None:
        """按当前时钟启动第一个计数周期.

        Args:
            topics: 报告顺序中的话题列表，重复名称只保留第一次。
            clock: 秒制时钟，默认 monotonic；注入时钟可抛异常，不在此校验。
        """
        self._topics = list(dict.fromkeys(topics))
        self._clock = clock
        self._lock = threading.Lock()
        self._counts: Dict[str, int] = {topic: 0 for topic in self._topics}
        self._period_start = clock()

    @property
    def topics(self) -> List[str]:
        """List[str]: 按报告顺序返回话题列表的副本."""
        return list(self._topics)

    def record(self, topic: str) -> None:
        """在锁内累计一个已登记话题的消息.

        Args:
            topic: 收到消息的话题名；未登记名称被忽略。
        """
        with self._lock:
            if topic in self._counts:
                self._counts[topic] += 1

    def sample(
        self, stamp: float, publishers: Optional[Mapping[str, int]] = None,
    ) -> List[TopicSample]:
        """在锁内截取并清空当前计数，开始下一个周期.

        即使时钟间隔非正，也会清空计数并更新周期起点，此时报告频率为零。
        锁外读取外部发布者映射并构造样本，不保证与计数同一时刻的图状态。

        Args:
            stamp: 输出样本的墙钟标签，不参与频率计算。
            publishers: 可选的话题发布者数映射，缺少的项为 None。

        Returns:
            按登记顺序生成的样本列表；消息数与间隔来自刚结束的周期。
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
    """用实例内线程锁向 CSV 追加样本.

    锁只串行化同一实例的写入，不协调其它实例或进程。没有事务回滚、
    fsync 或来源文件身份保护，调用方应保证目标目录及写入所有权。
    """

    def __init__(self, path: Union[str, Path]) -> None:
        """保存 CSV 目标路径并创建实例锁，不立即创建文件.

        Args:
            path: CSV 文件路径；真正写入前其父目录必须已存在。
        """
        self._path = Path(path)
        self._lock = threading.Lock()

    @property
    def path(self) -> Path:
        """Path: 保存的 CSV 文件路径."""
        return self._path

    def write(self, samples: Iterable[Union[TopicSample, TfSample]]) -> int:
        """序列化并追加样本；文件缺失或为空时先写表头.

        先在锁外消费全部输入并转换行，再在锁内打开和写入文件。
        空样本序列仍可能创建文件及表头；写入失败可能留下部分内容。

        Args:
            samples: 要追加的话题或 TF 样本，不做完整字段校验。

        Returns:
            追加的数据行数，不含表头。

        Raises:
            OSError: 目标文件检查、打开或写入失败。
            Exception: 输入迭代或字段格式化的未处理异常。
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
    """计算同一时钟域下采样时间与变换时间戳之差.

    Args:
        now_sec: 当前采样时间，单位秒，应与变换时间戳使用同一时钟。
        transform_stamp_sec: 变换头时间戳，单位秒。

    Returns:
        四舍五入到 0.1 毫秒的时间差；负值仅表示变换时间晚于采样时间，
        不能单独证明跨机器时钟偏差；不校验输入有限性。
    """
    return round((now_sec - transform_stamp_sec) * 1000.0, 1)


def percentile(values: Sequence[float], fraction: float) -> float:
    """对排序后的测量值按相邻秩线性插值计算分位数.

    Args:
        values: 任意顺序的数值序列，不验证各元素有限性。
        fraction: [0, 1] 内的分位比例，例如 0.95。

    Returns:
        插值结果，不修改输入序列。

    Raises:
        ValueError: 序列为空或分位比例不在闭区间内。
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
    """按名称汇总 CSV 中可解析的话题与 TF 行.

    忽略 stamp 字段，不按时长加权，也不拒绝可解析的负数或非有限值。
    整文件顺序读取但无读取预算或跨进程锁，不保证与并发写入的一致快照。

    Args:
        path: CSV 路径；缺失或不是文件时返回空摘要。

    Returns:
        话题的行频率均值/最小值，以及已知 TF 延迟的分位数和最大值；
        未知类型与已处理的字段错误计入 skipped_rows。

    Raises:
        OSError: 文件打开或读取失败。
        UnicodeError: 文件无法按 UTF-8 解码。
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
    话题计数与 TF 延迟占不同列，不能混作同一量。不做输入校验，
    非 TopicSample 对象会按 TF 字段读取，缺字段或格式错误直接传播。

    Args:
        sample: 调用方提供的话题或 TF 样本，不在此判定类型合法性或新鲜度。

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
