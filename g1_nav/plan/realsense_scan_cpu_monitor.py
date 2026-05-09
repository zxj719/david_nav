#!/usr/bin/env python3
"""Monitor CPU usage for the external Realsense camera and depth-to-scan bridge.

This script does not start any ROS nodes. It only watches the already-running
processes:

- realsense2_camera_node
- depthimage_to_laserscan_node

Example:
    python3 src/g1_nav/plan/realsense_scan_cpu_monitor.py --samples 30 --interval 1.0
"""

import argparse
import csv
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional


TARGET_LABELS = {
    'realsense_camera': 'Realsense Camera',
    'depth_to_scan': 'Depth To Scan',
}


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    command_name: str
    command_line: str
    category: str


def classify_process(command_name: str, command_line: str) -> Optional[str]:
    text = f'{command_name} {command_line}'
    if 'realsense2_camera_node' in text:
        return 'realsense_camera'
    if 'depthimage_to_laserscan_node' in text:
        return 'depth_to_scan'
    return None


def summarize_category_samples(category_samples: Dict[str, List[float]]) -> Dict[str, Dict[str, float]]:
    summary = {}
    for category, samples in category_samples.items():
        if not samples:
            summary[category] = {
                'samples': 0,
                'avg_cpu_percent': 0.0,
                'max_cpu_percent': 0.0,
            }
            continue

        avg_cpu = round(sum(samples) / len(samples), 2)
        max_cpu = round(max(samples), 2)
        summary[category] = {
            'samples': len(samples),
            'avg_cpu_percent': avg_cpu,
            'max_cpu_percent': max_cpu,
        }

    return summary


def read_total_cpu_jiffies() -> int:
    with Path('/proc/stat').open() as handle:
        first_line = handle.readline().strip().split()
    if not first_line or first_line[0] != 'cpu':
        raise RuntimeError('Failed to read total CPU jiffies from /proc/stat.')
    return sum(int(value) for value in first_line[1:])


def read_process_cpu_jiffies(pid: int) -> Optional[int]:
    stat_path = Path('/proc') / str(pid) / 'stat'
    try:
        content = stat_path.read_text()
    except FileNotFoundError:
        return None

    closing_paren = content.rfind(')')
    if closing_paren == -1:
        return None

    fields_after_name = content[closing_paren + 2:].split()
    if len(fields_after_name) < 13:
        return None

    utime = int(fields_after_name[11])
    stime = int(fields_after_name[12])
    return utime + stime


def iter_target_processes() -> Iterable[ProcessInfo]:
    proc_root = Path('/proc')
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue

        pid = int(entry.name)
        try:
            command_name = (entry / 'comm').read_text().strip()
            raw_cmdline = (entry / 'cmdline').read_bytes()
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue

        if raw_cmdline:
            command_line = raw_cmdline.replace(b'\x00', b' ').decode(errors='replace').strip()
        else:
            command_line = command_name

        category = classify_process(command_name, command_line)
        if category is None:
            continue

        yield ProcessInfo(
            pid=pid,
            command_name=command_name,
            command_line=command_line,
            category=category,
        )


def snapshot_processes() -> Dict[int, ProcessInfo]:
    return {process.pid: process for process in iter_target_processes()}


def build_cpu_snapshot(processes: Dict[int, ProcessInfo]) -> Dict[int, int]:
    return {
        pid: jiffies
        for pid, jiffies in (
            (pid, read_process_cpu_jiffies(pid))
            for pid in processes
        )
        if jiffies is not None
    }


def wait_for_required_processes(timeout_seconds: float) -> Dict[int, ProcessInfo]:
    deadline = time.time() + timeout_seconds
    required = set(TARGET_LABELS)

    while True:
        processes = snapshot_processes()
        seen_categories = {process.category for process in processes.values()}
        missing = required - seen_categories
        if not missing:
            return processes

        if time.time() >= deadline:
            missing_text = ', '.join(sorted(missing))
            raise TimeoutError(
                f'Timed out waiting for processes: {missing_text}. '
                'Please make sure the external Realsense camera and the depth-to-scan bridge are running.'
            )

        print(
            f'Waiting for target processes... missing: {", ".join(sorted(missing))}',
            flush=True,
        )
        time.sleep(1.0)


def format_command(command_line: str, width: int = 72) -> str:
    if len(command_line) <= width:
        return command_line
    return command_line[: width - 3] + '...'


def print_sample_table(sample_index: int, total_samples: int, rows: List[Dict[str, object]]) -> None:
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
    print(f'\n[{timestamp}] Sample {sample_index}/{total_samples}')
    print(f'{"Category":18} {"PID":>7} {"CPU%":>8} Command')
    print('-' * 96)
    for row in rows:
        print(
            f'{row["label"]:18} {row["pid"]:>7} {row["cpu_percent"]:>8.2f} '
            f'{format_command(str(row["command_line"]))}'
        )


def print_summary(category_samples: Dict[str, List[float]]) -> None:
    summary = summarize_category_samples(category_samples)
    print('\nSummary')
    print(f'{"Category":18} {"Samples":>8} {"Avg CPU%":>10} {"Max CPU%":>10}')
    print('-' * 52)
    for category in TARGET_LABELS:
        stats = summary[category]
        print(
            f'{TARGET_LABELS[category]:18} '
            f'{int(stats["samples"]):>8} '
            f'{stats["avg_cpu_percent"]:>10.2f} '
            f'{stats["max_cpu_percent"]:>10.2f}'
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Monitor CPU usage for Realsense camera and depth-to-scan nodes.',
    )
    parser.add_argument(
        '--interval',
        type=float,
        default=1.0,
        help='Sampling interval in seconds.',
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=20,
        help='Number of samples to capture after the target processes appear.',
    )
    parser.add_argument(
        '--wait-timeout',
        type=float,
        default=120.0,
        help='Maximum seconds to wait for both target processes before exiting.',
    )
    parser.add_argument(
        '--csv',
        type=Path,
        default=None,
        help='Optional CSV output path for per-sample category totals.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.interval <= 0.0:
        print('--interval must be greater than 0.', file=sys.stderr)
        return 2
    if args.samples <= 0:
        print('--samples must be greater than 0.', file=sys.stderr)
        return 2

    try:
        current_processes = wait_for_required_processes(args.wait_timeout)
    except TimeoutError as error:
        print(str(error), file=sys.stderr)
        return 1

    cpu_count = os.cpu_count() or 1
    csv_rows: List[Dict[str, object]] = []
    category_samples = {category: [] for category in TARGET_LABELS}

    previous_total = read_total_cpu_jiffies()
    previous_process_jiffies = build_cpu_snapshot(current_processes)

    for sample_index in range(1, args.samples + 1):
        time.sleep(args.interval)
        current_processes = snapshot_processes()
        current_total = read_total_cpu_jiffies()
        current_process_jiffies = build_cpu_snapshot(current_processes)
        total_delta = current_total - previous_total

        if total_delta <= 0:
            print('Skipping sample because total CPU delta was not positive.', file=sys.stderr)
            previous_total = current_total
            previous_process_jiffies = current_process_jiffies
            continue

        rows = []
        category_totals = {category: 0.0 for category in TARGET_LABELS}
        for pid, process in sorted(current_processes.items(), key=lambda item: (item[1].category, item[0])):
            current_jiffies = current_process_jiffies.get(pid)
            previous_jiffies = previous_process_jiffies.get(pid, current_jiffies)
            if current_jiffies is None or previous_jiffies is None:
                continue

            cpu_percent = max(
                0.0,
                ((current_jiffies - previous_jiffies) / total_delta) * cpu_count * 100.0,
            )
            category_totals[process.category] += cpu_percent
            rows.append({
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'sample_index': sample_index,
                'category': process.category,
                'label': TARGET_LABELS[process.category],
                'pid': pid,
                'cpu_percent': round(cpu_percent, 2),
                'command_line': process.command_line,
            })

        print_sample_table(sample_index, args.samples, rows)

        for category in TARGET_LABELS:
            category_total = round(category_totals[category], 2)
            category_samples[category].append(category_total)
            csv_rows.append({
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'sample_index': sample_index,
                'category': category,
                'cpu_percent': category_total,
            })

        previous_total = current_total
        previous_process_jiffies = current_process_jiffies

    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open('w', newline='') as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=['timestamp', 'sample_index', 'category', 'cpu_percent'],
            )
            writer.writeheader()
            writer.writerows(csv_rows)
        print(f'\nSaved CSV to: {args.csv}')

    print_summary(category_samples)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
