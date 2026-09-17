"""Passive Linux process CPU profiling; never start, signal or configure a target."""

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Sequence


@dataclass(frozen=True)
class _Counters:
    start_ticks: int
    cpu_ticks: int
    rss_bytes: int
    threads: int
    state: str


def _read_counters(pid, proc_root=Path('/proc')):
    """只读解析 Linux 进程计数器，不记录命令名、命令行或环境.

    Args:
        pid: 目标进程号，必须与 stat 中的身份一致。
        proc_root: proc 根目录，可注入离线测试目录。

    Returns:
        启动和 CPU tick、RSS 字节数、线程数及进程状态快照。

    Raises:
        OSError: 无法读取 stat 或系统页大小。
        ValueError: 身份、字段、数值或进程存活状态不合法。
    """
    # comm may contain spaces and parentheses. Never record comm/cmdline/environ.
    text = (proc_root / str(pid) / 'stat').read_text(encoding='utf-8')
    end = text.rfind(')')
    if end < 0 or text.split(' ', 1)[0] != str(pid):
        raise ValueError('invalid process stat identity')
    fields = text[end + 2:].split()
    if len(fields) < 22:
        raise ValueError('truncated process stat')
    value = _Counters(int(fields[19]), int(fields[11]) + int(fields[12]),
                      int(fields[21]) * os.sysconf('SC_PAGE_SIZE'),
                      int(fields[17]), fields[0])
    if (min(value.start_ticks, value.cpu_ticks, value.rss_bytes) < 0 or value.threads < 1
            or value.state in ('Z', 'X', 'x')):
        raise ValueError('invalid or exited process counters')
    return value


def _positive(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError('must be positive and finite')
    return result


def _collect(source, seconds, interval, tick_hz, emit, *, idle_confirmed=False,
             clock=time.monotonic, sleep=time.sleep):
    """被动采样同一进程的 CPU 增量，给出完整性与 R6 资格而非自动认定空闲.

    OSError、ValueError 和键盘中断转为不完整原因；其它注入函数异常透传。
    CPU 百分比以单核为 100%，包含进程线程，不包含子进程。

    Args:
        source: 返回进程计数器快照的只读函数。
        seconds: 请求采样时长（正数秒）。
        interval: 采样间隔（正数秒，调用方保证不超过总时长）。
        tick_hz: 每秒 CPU tick 数，调用方保证为正。
        emit: 接收每条样本字典的函数；首条百分比为 None。
        idle_confirmed: 操作者是否确认目标在整个窗口内空闲。
        clock: 返回单调时钟秒数的函数。
        sleep: 秒制阻塞等待函数，可由测试替换。

    Returns:
        含实际时长、均值、间隔质量、不完整原因与 R6 资格的汇总字典。
    """
    started, last_stamp, baseline, previous = None, None, None, None
    rows, elapsed, cpu_delta, max_gap = 0, 0.0, 0, 0.0
    reason = None
    try:
        while True:
            current, stamp = source(), clock()
            if not math.isfinite(stamp):
                raise ValueError('nonfinite monotonic clock')
            if baseline is None:
                baseline, previous = current, current
                started = last_stamp = stamp
                percent = None  # Baseline is not an artificial zero-CPU sample.
            else:
                if current.start_ticks != baseline.start_ticks:
                    raise ValueError('process identity changed (PID reused)')
                gap = stamp - last_stamp
                if gap <= 0 or current.cpu_ticks < previous.cpu_ticks:
                    raise ValueError('clock or CPU counter did not advance monotonically')
                elapsed = stamp - started
                cpu_delta = current.cpu_ticks - baseline.cpu_ticks
                max_gap = max(max_gap, gap)
                percent = 100.0 * (current.cpu_ticks - previous.cpu_ticks) / tick_hz / gap
                previous, last_stamp = current, stamp
            emit(dict(elapsed_sec=elapsed, cpu_ticks=current.cpu_ticks, cpu_percent=percent,
                      rss_bytes=current.rss_bytes, threads=current.threads, state=current.state))
            rows += 1
            if elapsed >= seconds:
                break
            sleep(min(interval, seconds - elapsed))
    except (OSError, ValueError) as error:
        reason = f'{type(error).__name__}: {error}'
    except KeyboardInterrupt:
        reason = 'interrupted'
    complete = reason is None and elapsed >= seconds and rows > 1
    expected_intervals = math.ceil(seconds / interval)
    cadence_ok = max_gap <= interval * 2.0 and rows - 1 >= expected_intervals
    eligible = complete and seconds >= 60.0 and interval <= 1.0 and cadence_ok and idle_confirmed
    mean = 100.0 * cpu_delta / tick_hz / elapsed if elapsed > 0 else None
    return dict(
        complete=complete, reason=reason, rows=rows, intervals=max(0, rows - 1),
        elapsed_sec=elapsed, cpu_delta_ticks=cpu_delta, cpu_mean_percent=mean,
        max_gap_sec=max_gap, cadence_ok=cadence_ok, expected_intervals=expected_intervals,
        process_start_ticks=None if baseline is None else baseline.start_ticks,
        r6_eligible=eligible,
        r6_at_or_above_three_percent=(mean >= 3.0 if eligible else None))


def main(args: Sequence[str] | None = None) -> int:
    """Write a new evidence directory for one explicitly selected Linux process.

    Args:
        args: CLI arguments, or None for the current command line.

    Returns:
        Zero for a complete window, one for incomplete evidence, two for invalid
        arguments/output, or 130 for an interrupted measurement. Eligibility for
        R6 additionally requires at least 60 seconds, cadence and confirmed idle.

    Raises:
        SystemExit: Argument parsing handles help or rejects invalid arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True,
                        help='new directory; never overwrite')
    parser.add_argument('--seconds', type=_positive, default=60.0)
    parser.add_argument('--interval', type=_positive, default=1.0)
    parser.add_argument('--confirm-idle', action='store_true',
                        help='operator confirms the target is idle throughout this window')
    options = parser.parse_args(args)
    if options.pid <= 0 or options.interval > options.seconds:
        parser.error('positive PID and interval <= seconds are required')
    try:
        tick_hz = os.sysconf('SC_CLK_TCK')
        if tick_hz <= 0:
            raise ValueError('invalid system clock tick frequency')
        metadata = dict(
            schema_version=1, pid=options.pid, requested_seconds=options.seconds,
            interval_seconds=options.interval, clock_ticks_per_second=tick_hz,
            cpu_units='percent of one CPU core; process threads included, children excluded',
            idle_confirmed=options.confirm_idle,
            started_utc=datetime.now(timezone.utc).isoformat(),
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            method='Linux /proc/PID/stat utime+stime deltas; monotonic wall time',
            r6_threshold_percent=3.0)
        options.output.mkdir()  # Refuse existing paths, including symlinks.
        with (options.output / 'metadata.json').open('x', encoding='utf-8') as handle:
            json.dump(metadata, handle, ensure_ascii=False, indent=2)
        with (options.output / 'samples.csv').open('x', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=(
                'elapsed_sec', 'cpu_ticks', 'cpu_percent', 'rss_bytes', 'threads', 'state'))
            writer.writeheader()

            def emit(row):
                writer.writerow(row)
                handle.flush()

            result = _collect(
                lambda: _read_counters(options.pid), options.seconds, options.interval,
                tick_hz, emit, idle_confirmed=options.confirm_idle,
                clock=time.monotonic, sleep=time.sleep)
        with (options.output / 'summary.json').open('x', encoding='utf-8') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
        print(json.dumps(result, ensure_ascii=False, allow_nan=False))
        return 130 if result['reason'] == 'interrupted' else (0 if result['complete'] else 1)
    except (OSError, ValueError) as error:
        print(f'profile failed: {type(error).__name__}: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
