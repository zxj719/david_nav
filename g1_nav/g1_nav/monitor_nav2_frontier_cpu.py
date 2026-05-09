#!/usr/bin/env python3
"""Monitor CPU usage for Nav2 and frontier exploration processes.

This script reads process CPU time directly from /proc so it does not require
extra Python packages such as psutil. CPU values are reported in two forms:

- raw: Linux multi-core process percentage. On an 8-core machine the total
  system capacity is 800%.
- sys: normalized system percentage, i.e. raw / logical_cores.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set


NAV2_EXECUTABLES = {
    "controller_server",
    "planner_server",
    "smoother_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
}

FRONTIER_EXECUTABLES = {
    "frontier_explorer_cpp",
}

NAV2_CONTAINER_EXECUTABLES = {
    "component_container",
    "component_container_mt",
    "component_container_isolated",
}


@dataclass(frozen=True)
class ProcInfo:
    pid: int
    comm: str
    cmdline: str


def executable_name(cmdline: str) -> str:
    if not cmdline:
        return ""
    return os.path.basename(cmdline.split()[0])


def matches_process_name(proc: ProcInfo, target: str) -> bool:
    exec_name = executable_name(proc.cmdline)
    return (
        proc.comm == target
        or proc.comm == target[:15]
        or exec_name == target
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor CPU usage for Nav2 and frontier explorer processes.",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="Sampling interval in seconds. Default: 1.0",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Optional run duration in seconds. 0 means run until Ctrl+C.",
    )
    parser.add_argument(
        "--cores",
        type=int,
        default=os.cpu_count() or 1,
        help="Logical CPU core count used for normalization. Default: auto-detect.",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=None,
        help="Optional CSV output path. Default: auto-generate in current directory.",
    )
    parser.add_argument(
        "--no-wait",
        action="store_true",
        help="Start sampling immediately instead of waiting for both groups to appear.",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=0.0,
        help="Optional wait timeout in seconds while waiting for target processes. 0 means no timeout.",
    )
    return parser.parse_args()


def default_csv_path() -> Path:
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path.cwd() / f"nav2_frontier_cpu_{timestamp}.csv"


def read_text(path: Path) -> Optional[str]:
    try:
        return path.read_text().strip()
    except OSError:
        return None


def read_cmdline(path: Path) -> Optional[str]:
    try:
        data = path.read_bytes()
    except OSError:
        return None
    if not data:
        return ""
    return data.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()


def iter_processes() -> Iterable[ProcInfo]:
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue

        pid = int(entry.name)
        base = Path(entry.path)
        comm = read_text(base / "comm")
        if not comm:
            continue

        cmdline = read_cmdline(base / "cmdline")
        if cmdline is None:
            continue

        yield ProcInfo(pid=pid, comm=comm, cmdline=cmdline)


def classify_group(proc: ProcInfo) -> Optional[str]:
    if any(matches_process_name(proc, name) for name in FRONTIER_EXECUTABLES):
        return "frontier"

    if any(matches_process_name(proc, name) for name in NAV2_EXECUTABLES):
        return "nav2"

    if (
        matches_process_name(proc, "lifecycle_manager")
        and "lifecycle_manager_navigation" in proc.cmdline
    ):
        return "nav2"

    if (
        any(matches_process_name(proc, name) for name in NAV2_CONTAINER_EXECUTABLES)
        and "nav2_container" in proc.cmdline
    ):
        return "nav2"

    return None


def discover_groups() -> Dict[str, Set[int]]:
    groups: Dict[str, Set[int]] = {"nav2": set(), "frontier": set()}
    for proc in iter_processes():
        group = classify_group(proc)
        if group is not None:
            groups[group].add(proc.pid)
    return groups


def read_total_cpu_ticks() -> int:
    with open("/proc/stat", "r", encoding="utf-8") as handle:
        first_line = handle.readline().strip()
    parts = first_line.split()
    if not parts or parts[0] != "cpu":
        raise RuntimeError("Unexpected /proc/stat format")
    return sum(int(value) for value in parts[1:])


def read_process_ticks(pid: int) -> Optional[int]:
    try:
        with open(f"/proc/{pid}/stat", "r", encoding="utf-8") as handle:
            stat = handle.read()
    except OSError:
        return None

    closing = stat.rfind(")")
    if closing == -1:
        return None

    fields = stat[closing + 2 :].split()
    if len(fields) < 13:
        return None

    utime = int(fields[11])
    stime = int(fields[12])
    return utime + stime


def read_target_ticks(pids: Iterable[int]) -> Dict[int, int]:
    ticks: Dict[int, int] = {}
    for pid in pids:
        value = read_process_ticks(pid)
        if value is not None:
            ticks[pid] = value
    return ticks


def percentile(values: Sequence[float], pct: float) -> float:
    if not values:
        return 0.0
    if len(values) == 1:
        return values[0]

    ordered = sorted(values)
    position = (len(ordered) - 1) * (pct / 100.0)
    lower_index = math.floor(position)
    upper_index = math.ceil(position)
    if lower_index == upper_index:
        return ordered[lower_index]

    lower = ordered[lower_index]
    upper = ordered[upper_index]
    weight = position - lower_index
    return lower + (upper - lower) * weight


def summarize(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {"avg": 0.0, "max": 0.0, "p95": 0.0}
    return {
        "avg": sum(values) / len(values),
        "max": max(values),
        "p95": percentile(values, 95.0),
    }


def format_pct(value: float) -> str:
    return f"{value:6.1f}%"


def wait_for_groups(timeout: float) -> Dict[str, Set[int]]:
    start = time.monotonic()
    last_status: Optional[str] = None

    while True:
        groups = discover_groups()
        status = f"nav2={len(groups['nav2'])} frontier={len(groups['frontier'])}"
        if status != last_status:
            print(f"Waiting for target processes: {status}", flush=True)
            last_status = status

        if groups["nav2"] and groups["frontier"]:
            print("Target processes detected. Starting sampling.", flush=True)
            return groups

        if timeout > 0.0 and (time.monotonic() - start) >= timeout:
            raise TimeoutError(
                "Timed out while waiting for both Nav2 and frontier processes."
            )

        time.sleep(0.5)


def main() -> int:
    args = parse_args()
    if args.interval <= 0.0:
        raise SystemExit("--interval must be > 0")
    if args.cores <= 0:
        raise SystemExit("--cores must be > 0")

    csv_path = args.csv or default_csv_path()
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    if args.no_wait:
        initial_groups = discover_groups()
        print(
            "Starting immediately without waiting for both process groups. "
            f"Initial match: nav2={len(initial_groups['nav2'])} "
            f"frontier={len(initial_groups['frontier'])}",
            flush=True,
        )
    else:
        try:
            initial_groups = wait_for_groups(args.wait_timeout)
        except KeyboardInterrupt:
            print("\nStopped while waiting for target processes.", flush=True)
            return 130
        except TimeoutError as exc:
            print(str(exc), flush=True)
            return 1

    prev_total_ticks = read_total_cpu_ticks()
    prev_proc_ticks = read_target_ticks(initial_groups["nav2"] | initial_groups["frontier"])

    start_wall = time.monotonic()
    next_sample_time = start_wall + args.interval

    nav2_raw_values: List[float] = []
    frontier_raw_values: List[float] = []
    total_raw_values: List[float] = []

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "timestamp",
                "elapsed_sec",
                "nav2_cpu_raw_pct",
                "frontier_cpu_raw_pct",
                "total_cpu_raw_pct",
                "nav2_cpu_sys_pct",
                "frontier_cpu_sys_pct",
                "total_cpu_sys_pct",
                "nav2_pids",
                "frontier_pids",
            ]
        )

        print(f"CSV logging: {csv_path}", flush=True)
        print(
            f"Logical cores: {args.cores} "
            f"(100% raw = 1 core, 100% sys = full machine)",
            flush=True,
        )

        try:
            while True:
                sleep_time = next_sample_time - time.monotonic()
                if sleep_time > 0.0:
                    time.sleep(sleep_time)

                sample_wall = time.monotonic()
                elapsed = sample_wall - start_wall
                if args.duration > 0.0 and elapsed > args.duration:
                    break

                groups = discover_groups()
                active_pids = groups["nav2"] | groups["frontier"]
                proc_ticks = read_target_ticks(active_pids)
                total_ticks = read_total_cpu_ticks()
                delta_total_ticks = total_ticks - prev_total_ticks

                if delta_total_ticks <= 0:
                    prev_total_ticks = total_ticks
                    prev_proc_ticks = proc_ticks
                    next_sample_time += args.interval
                    continue

                nav2_delta = sum(
                    proc_ticks[pid] - prev_proc_ticks.get(pid, proc_ticks[pid])
                    for pid in groups["nav2"]
                    if pid in proc_ticks
                )
                frontier_delta = sum(
                    proc_ticks[pid] - prev_proc_ticks.get(pid, proc_ticks[pid])
                    for pid in groups["frontier"]
                    if pid in proc_ticks
                )

                nav2_raw = 100.0 * args.cores * nav2_delta / delta_total_ticks
                frontier_raw = 100.0 * args.cores * frontier_delta / delta_total_ticks
                total_raw = nav2_raw + frontier_raw

                nav2_sys = nav2_raw / args.cores
                frontier_sys = frontier_raw / args.cores
                total_sys = total_raw / args.cores

                timestamp = dt.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                nav2_pid_str = ";".join(str(pid) for pid in sorted(groups["nav2"]))
                frontier_pid_str = ";".join(str(pid) for pid in sorted(groups["frontier"]))

                print(
                    f"{timestamp} | "
                    f"nav2 raw/sys {format_pct(nav2_raw)} / {format_pct(nav2_sys)} | "
                    f"frontier raw/sys {format_pct(frontier_raw)} / {format_pct(frontier_sys)} | "
                    f"total raw/sys {format_pct(total_raw)} / {format_pct(total_sys)}",
                    flush=True,
                )

                writer.writerow(
                    [
                        timestamp,
                        f"{elapsed:.3f}",
                        f"{nav2_raw:.3f}",
                        f"{frontier_raw:.3f}",
                        f"{total_raw:.3f}",
                        f"{nav2_sys:.3f}",
                        f"{frontier_sys:.3f}",
                        f"{total_sys:.3f}",
                        nav2_pid_str,
                        frontier_pid_str,
                    ]
                )
                csv_file.flush()

                nav2_raw_values.append(nav2_raw)
                frontier_raw_values.append(frontier_raw)
                total_raw_values.append(total_raw)

                prev_total_ticks = total_ticks
                prev_proc_ticks = proc_ticks
                next_sample_time += args.interval

        except KeyboardInterrupt:
            print("\nStopping monitor.", flush=True)

    duration = time.monotonic() - start_wall
    nav2_raw_summary = summarize(nav2_raw_values)
    frontier_raw_summary = summarize(frontier_raw_values)
    total_raw_summary = summarize(total_raw_values)

    print("Summary", flush=True)
    print(f"  duration_sec={duration:.1f}", flush=True)
    print(f"  samples={len(total_raw_values)}", flush=True)
    print(f"  csv={csv_path}", flush=True)
    print(
        "  nav2_raw_avg={avg:.1f}% nav2_raw_max={max:.1f}% nav2_raw_p95={p95:.1f}%".format(
            **nav2_raw_summary
        ),
        flush=True,
    )
    print(
        "  frontier_raw_avg={avg:.1f}% frontier_raw_max={max:.1f}% frontier_raw_p95={p95:.1f}%".format(
            **frontier_raw_summary
        ),
        flush=True,
    )
    print(
        "  total_raw_avg={avg:.1f}% total_raw_max={max:.1f}% total_raw_p95={p95:.1f}%".format(
            **total_raw_summary
        ),
        flush=True,
    )
    print(
        "  nav2_sys_avg={avg:.1f}% nav2_sys_max={max:.1f}% nav2_sys_p95={p95:.1f}%".format(
            avg=nav2_raw_summary["avg"] / args.cores,
            max=nav2_raw_summary["max"] / args.cores,
            p95=nav2_raw_summary["p95"] / args.cores,
        ),
        flush=True,
    )
    print(
        "  frontier_sys_avg={avg:.1f}% frontier_sys_max={max:.1f}% frontier_sys_p95={p95:.1f}%".format(
            avg=frontier_raw_summary["avg"] / args.cores,
            max=frontier_raw_summary["max"] / args.cores,
            p95=frontier_raw_summary["p95"] / args.cores,
        ),
        flush=True,
    )
    print(
        "  total_sys_avg={avg:.1f}% total_sys_max={max:.1f}% total_sys_p95={p95:.1f}%".format(
            avg=total_raw_summary["avg"] / args.cores,
            max=total_raw_summary["max"] / args.cores,
            p95=total_raw_summary["p95"] / args.cores,
        ),
        flush=True,
    )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
