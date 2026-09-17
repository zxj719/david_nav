"""采样指定进程树与系统的 CPU/内存，并追加到 resources.csv.

ResourceSampler 每轮重新发现根与后代，系统行在前，进程消失/无权限等会跳过。
进程 CPU 沿用 psutil 的每核口径，100 表示一核；系统行使用全机百分比。
汇总按进程名称合并各行，均值不是同一时刻所有同名进程的总 CPU。
时间是墙钟，RSS 与系统已用内存以 MiB 表示，采样不是原子宿主快照。
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
    """表示一轮资源观察中的进程或系统行，不自行校验数值.

    Attributes:
        stamp: 墙钟秒数，同轮共用一个读到的时间。
        pid: 进程号，系统行用零。
        name: 进程名称，系统行用 __system__。
        cpu_percent: 由采样后端给出的 CPU 百分比，进程与系统口径不同。
        rss_mb: 进程 RSS 的 MiB；系统行是全机已用内存 MiB。
        num_threads: 线程数，系统行用零。
    """

    stamp: float
    pid: int
    name: str
    cpu_percent: float
    rss_mb: float
    num_threads: int


class ResourceSampler:
    """表示向一个 CSV 追加进程树资源观察的采样器.

    缓存后端进程对象以获得跨轮 CPU 变化，不提供多线程同步或全进程树原子快照。
    调用方负责单一写者、停止与异常管理；本对象不启动线程或信号进程。
    """

    def __init__(
        self,
        root_pids: Callable[[], Iterable[int]],
        csv_path: Union[str, Path],
        clock: Callable[[], float] = time.time,
        psutil_module: Any = psutil,
    ) -> None:
        """绑定进程发现、输出与观察端口，不立即查询系统或创建 CSV.

        Args:
            root_pids: 每轮提供根 PID 的回调，允许纳入后启动的进程。
            csv_path: 追加目标，首次空文件写表头，父目录由外层准备。
            clock: 墙钟时间端口，测试可替换。
            psutil_module: psutil 或等接口假件，提供进程和系统观察。
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
        """Path: 资源样本 CSV 的追加路径."""
        return self._csv_path

    def sample(self) -> List[ProcessSample]:
        """观察系统及当前进程树，追加本轮样本并更新进程缓存.

        后端的非阻塞 CPU 统计依赖此前观察；真实 psutil 新对象首轮值通常无代表性。
        进程消失、无权限或僵尸错误会跳过该项，系统端口和其它异常继续传播。
        CSV 写入失败时内存缓存可能已更新、部分行可能已写入，不能回滚本轮。

        Returns:
            实际交给追加器的样本列表，系统行在前，进程按发现次序去重。

        Raises:
            OSError: CSV 状态检查、打开或写入失败。
            Exception: 根 PID、时钟或观察后端发生未被隔离的错误。
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
        """在调用线程重复采样，停止事件可唤醒间隔等待，不抢占正在采样的调用.

        不是定频调度：每轮采样完成后再等待 interval_sec；采样异常会直接终止循环。
        本函数不验证间隔，调用方应传合理的正值并管理执行线程。

        Args:
            stop_event: 设置后不再开始下一轮；已设置时不会采样。
            interval_sec: 每轮完成后的等待秒数，默认一秒。

        Raises:
            Exception: sample 或等待端口失败，不在此吞掉或自动重试。
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
    """表示同名采样行的算术统计，不代表按 PID 或同时刻汇总.

    Attributes:
        name: 进程名称或系统行名称。
        samples: 纳入统计的行数，不是唯一进程数。
        cpu_avg: 所有行 CPU 百分比的算术平均，不按时间间隔加权。
        cpu_peak: 单行 CPU 百分比最大值。
        rss_peak_mb: 单行内存 MiB 最大值。
    """

    name: str
    samples: int
    cpu_avg: float
    cpu_peak: float
    rss_peak_mb: float


@dataclass(frozen=True)
class ResourceSummary:
    """表示一次 CSV 汇总结果，不深冻结内部列表.

    Attributes:
        system: 系统行统计，没有系统样本时为 None。
        processes: 按 CPU 峰值降序、同峰按名字排序的同名进程统计列表。
    """

    system: Optional[ProcessStats] = None
    processes: List[ProcessStats] = field(default_factory=list)


def summarize_resources(csv_path: Union[str, Path]) -> ResourceSummary:
    """读取整个资源 CSV 并按名称合并，不再次采样或按时间计算并发总量.

    不限制文件大小、不校验非有限数值，不隔离损坏行；解析或数值错误直接传播。
    同名不同 PID 的行进入同一组，结果不是单进程身份连续性的证明。

    Args:
        csv_path: 资源 CSV 路径，缺失或不是文件时为空汇总。

    Returns:
        系统行和排序后的进程名统计；CPU 均值为行均值，RSS 为单行峰值。

    Raises:
        OSError: CSV 无法打开或读取。
        UnicodeError: 文件不是有效 UTF-8。
        KeyError: 缺少必要列。
        ValueError: 数值文本无法转换。
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
    """按同名资源采样行计算算术均值和峰值，不再次查询进程.

    调用方须提供非空且可转数值的行；CPU 为采样百分比，RSS 字段单位为 MiB。
    不按时间加权，不按 PID 区分，也不将同一时刻的多个进程 CPU 相加。

    Args:
        name: 汇总结果中使用的进程名称。
        rows: 已读取的非空 CSV 记录，含 cpu_percent 与 rss_mb。

    Returns:
        样本行数、CPU 平均值/峰值和 RSS 峰值组成的 ProcessStats。

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
