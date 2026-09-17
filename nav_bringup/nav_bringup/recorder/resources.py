"""Per-process CPU and memory sampling for navigation runs.

:class:`ResourceSampler` periodically samples every process in the process
trees of a run (for example a ``ros2 launch`` process and its children) plus a
whole-system row, and appends them to ``metrics/resources.csv``.
:func:`summarize_resources` aggregates that file for the run summary
(refactor plan §13.2, §15).

Process CPU percentages follow ``psutil``: 100 means one fully used core. The
system row reports overall CPU usage from 0 to 100 and used memory.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path
import threading
import time
from typing import Any, Callable, Dict, Iterable, List, Optional, Union

import psutil

CSV_COLUMNS = ('stamp', 'pid', 'name', 'cpu_percent', 'rss_mb', 'num_threads')
SYSTEM_ROW_NAME = '__system__'

_BYTES_PER_MB = 1024 * 1024


@dataclass(frozen=True)
class ProcessSample:
    """Resource usage of one process at one instant.

    Attributes:
        stamp: Wall-clock time in seconds since the epoch.
        pid: Process id; 0 for the whole-system row.
        name: Process name, or :data:`SYSTEM_ROW_NAME`.
        cpu_percent: CPU usage since the previous sample.
        rss_mb: Resident memory in MiB; used system memory for the system row.
        num_threads: Thread count; 0 for the system row.
    """

    stamp: float
    pid: int
    name: str
    cpu_percent: float
    rss_mb: float
    num_threads: int


class ResourceSampler:
    """Samples the resource usage of process trees into a CSV file."""

    def __init__(
        self,
        root_pids: Callable[[], Iterable[int]],
        csv_path: Union[str, Path],
        clock: Callable[[], float] = time.time,
        psutil_module: Any = psutil,
    ) -> None:
        """Create a sampler.

        Args:
            root_pids: Returns the pids whose process trees are sampled. It is
                called on every sample so late-starting processes are included.
            csv_path: Output CSV file; created with a header on first write.
            clock: Time source, replaceable in tests.
            psutil_module: ``psutil`` or a test double with the same interface.
        """
        self._root_pids = root_pids
        self._csv_path = Path(csv_path)
        self._clock = clock
        self._psutil = psutil_module
        self._errors = (
            psutil_module.NoSuchProcess,
            psutil_module.AccessDenied,
            psutil_module.ZombieProcess,
        )
        self._processes: Dict[int, Any] = {}

    @property
    def csv_path(self) -> Path:
        """Path: The CSV file samples are appended to."""
        return self._csv_path

    def sample(self) -> List[ProcessSample]:
        """Take one sample and append it to the CSV file.

        CPU percentages are measured since the previous call, so the first
        sample of a newly seen process reports 0.

        Returns:
            The samples written, system row first.
        """
        stamp = self._clock()
        memory = self._psutil.virtual_memory()
        samples = [
            ProcessSample(
                stamp=stamp,
                pid=0,
                name=SYSTEM_ROW_NAME,
                cpu_percent=float(self._psutil.cpu_percent(interval=None)),
                rss_mb=memory.used / _BYTES_PER_MB,
                num_threads=0,
            )
        ]
        alive: Dict[int, Any] = {}
        for pid in self._tree_pids():
            process = self._processes.get(pid) or self._open_process(pid)
            if process is None:
                continue
            try:
                samples.append(
                    ProcessSample(
                        stamp=stamp,
                        pid=pid,
                        name=str(process.name()),
                        cpu_percent=float(process.cpu_percent(interval=None)),
                        rss_mb=process.memory_info().rss / _BYTES_PER_MB,
                        num_threads=int(process.num_threads()),
                    ))
            except self._errors:
                continue
            alive[pid] = process
        self._processes = alive
        self._append(samples)
        return samples

    def run(self, stop_event: threading.Event, interval_sec: float = 1.0) -> None:
        """Sample until ``stop_event`` is set.

        Args:
            stop_event: Set it to stop sampling; the current wait ends at once.
            interval_sec: Seconds between samples.
        """
        while not stop_event.is_set():
            self.sample()
            stop_event.wait(interval_sec)

    def _open_process(self, pid: int) -> Optional[Any]:
        try:
            return self._psutil.Process(pid)
        except self._errors:
            return None

    def _tree_pids(self) -> List[int]:
        """收集选定根进程及递归后代的唯一 PID，观察权限/消失错误保守跳过.

        Returns:
            按发现顺序去重的 PID 列表；不读取任意无关进程，
            根或后代扫描失败可导致样本缺失而非证明它们不存在。
        """
        pids: Dict[int, None] = {}
        for root_pid in self._root_pids():
            root = self._processes.get(root_pid) or self._open_process(root_pid)
            if root is None:
                continue
            pids[root_pid] = None
            try:
                for child in root.children(recursive=True):
                    pids[child.pid] = None
            except self._errors:
                continue
        return list(pids)

    def _append(self, samples: List[ProcessSample]) -> None:
        """追加资源样本到 CSV，只在文件不存在或为空时写表头.

        Args:
            samples: 秒制时间、PID、进程名、CPU 百分比、内存及线程数快照。
                数值沿用既有三位时间/一位 CPU 和内存显示精度。

        Raises:
            OSError: 文件状态读取、追加写入或关闭失败；不回滚已写入的行。
        """
        write_header = not self._csv_path.is_file() or self._csv_path.stat().st_size == 0
        with self._csv_path.open('a', encoding='utf-8', newline='') as handle:
            writer = csv.writer(handle)
            if write_header:
                writer.writerow(CSV_COLUMNS)
            for sample in samples:
                writer.writerow([
                    f'{sample.stamp:.3f}',
                    sample.pid,
                    sample.name,
                    f'{sample.cpu_percent:.1f}',
                    f'{sample.rss_mb:.1f}',
                    sample.num_threads,
                ])


@dataclass(frozen=True)
class ProcessStats:
    """Aggregated usage of one process name over a run.

    Attributes:
        name: Process name, or :data:`SYSTEM_ROW_NAME`.
        samples: Number of samples aggregated.
        cpu_avg: Mean CPU percentage.
        cpu_peak: Maximum CPU percentage.
        rss_peak_mb: Maximum resident memory in MiB.
    """

    name: str
    samples: int
    cpu_avg: float
    cpu_peak: float
    rss_peak_mb: float


@dataclass(frozen=True)
class ResourceSummary:
    """Aggregated resource usage of a run.

    Attributes:
        system: Statistics of the whole-system rows, or ``None`` without samples.
        processes: Statistics per process name, by descending peak CPU.
    """

    system: Optional[ProcessStats] = None
    processes: List[ProcessStats] = field(default_factory=list)


def summarize_resources(csv_path: Union[str, Path]) -> ResourceSummary:
    """Aggregate a resources CSV file.

    Args:
        csv_path: File written by :class:`ResourceSampler`. A missing file
            yields an empty summary.

    Returns:
        The aggregated usage.
    """
    path = Path(csv_path)
    if not path.is_file():
        return ResourceSummary()
    grouped: Dict[str, List[Dict[str, str]]] = {}
    with path.open('r', encoding='utf-8', newline='') as handle:
        for row in csv.DictReader(handle):
            grouped.setdefault(row['name'], []).append(row)
    stats = {name: _aggregate(name, rows) for name, rows in grouped.items()}
    system = stats.pop(SYSTEM_ROW_NAME, None)
    processes = sorted(stats.values(), key=lambda item: (-item.cpu_peak, item.name))
    return ResourceSummary(system=system, processes=processes)


def _aggregate(name: str, rows: List[Dict[str, str]]) -> ProcessStats:
    """按已有资源采样行计算均值和峰值，不再次查询进程.

    调用方须提供非空且可转数值的行；CPU 为采样百分比，RSS 字段单位为 MB。

    Args:
        name: 汇总结果中使用的进程名称。
        rows: 已读取的非空 CSV 记录，含 cpu_percent 与 rss_mb。

    Returns:
        样本数、CPU 平均值/峰值和 RSS 峰值组成的 ProcessStats。

    Raises:
        KeyError: 必需列不存在。
        ValueError: 采样文本不能转换为数值。
        ZeroDivisionError: 调用方违反非空行前提。
    """
    cpu_values = [float(row['cpu_percent']) for row in rows]
    rss_values = [float(row['rss_mb']) for row in rows]
    return ProcessStats(
        name=name,
        samples=len(rows),
        cpu_avg=sum(cpu_values) / len(cpu_values),
        cpu_peak=max(cpu_values),
        rss_peak_mb=max(rss_values),
    )
