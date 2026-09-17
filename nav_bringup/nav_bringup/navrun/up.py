"""Unified startup CLI with a pure preview and a lock-scoped recorded runtime."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, Tuple

from nav_bringup.launching.executables import resolve_executable
from nav_bringup.navcfg import schema
from nav_bringup.navrun.preflight import PreflightError, ProcessInfo
from nav_bringup.navrun.locking import DEFAULT_LOCK_PATH
from nav_bringup.navrun.recorded_startup import run_recorded_startup, StartupOptions
from nav_bringup.navrun.runtime_commands import bind_commands
from nav_bringup.navrun.runtime_host import bind_host_steps
from nav_bringup.navrun.teleop_options import TeleopOptions
from nav_bringup.navrun.up_planning import discover_inputs, plan_up, report_plan, UpRequest
from nav_bringup.recorder.run_dir import default_runs_root


def _processes() -> Tuple[ProcessInfo, ...]:
    from nav_bringup.navrun.process_backend import snapshot_processes

    return snapshot_processes()


def preview(args: argparse.Namespace) -> Dict[str, Any]:
    """Read resource files and processes and return the startup preview.

    Args:
        args: Parsed up arguments; source_root optionally overrides installed assets.

    Returns:
        JSON-compatible plan. The synthetic dry-run directory is never created,
        and no git, ROS CLI, platform or other subprocess is started.

    Raises:
        ValueError: Invalid configuration or selectors.
        LookupError: Required installed resources are unavailable.
        OSError: A required source cannot be read.
        PreflightError: Process visibility is insufficient for chassis auto detection.
    """
    root = (args.runs_root or default_runs_root()).expanduser().absolute()
    plan = _plan(args, root / 'dry_run_preview')
    report = report_plan(plan)
    if not plan.errors:
        try:
            report['runtime_commands'] = asdict(bind_commands(
                plan, strict=args.strict, resolve=resolve_executable))
        except (ValueError, LookupError, OSError) as error:
            report['errors'].append(str(error))
    return report


def _plan(args, run_dir):
    """解析资源与 schema，并基于只读进程快照生成启动计划.

    Args:
        args: CLI 的模式、机器人/场地、底盘、记录及可选源码根配置。
        run_dir: 计划所属运行目录路径。

    Returns:
        plan_up 生成的未执行计划；不在本函数创建 ROS 或发送信号。

    Raises:
        ValueError: 没有 schema 或请求/配置验证失败。
        LookupError: 资源包或所需可执行文件未找到。
        OSError: 输入资源或进程快照无法读取。
    """
    request = UpRequest(args.mode, args.robot, args.site, run_dir,
                        args.chassis, args.record, args.no_executor,
                        TeleopOptions(args.layout, args.linear_speed, args.angular_speed),
                        getattr(args, 'mode_file', None), getattr(args, 'composition', False))

    def share(package: str) -> Path:
        if args.source_root is not None:
            path = args.source_root.expanduser().absolute() / package
            if not (path / 'package.xml').is_file():
                raise LookupError(f'package resources not found: {path}')
            return path
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package))

    inputs = discover_inputs(request, share)
    paths = (schema.find_source_schemas(args.source_root.expanduser())
             if args.source_root is not None
             else schema.find_installed_schemas(schema.ament_prefixes(os.environ)))
    if not paths:
        raise ValueError('no package schemas found')
    return plan_up(request, inputs, schema.load_catalog(paths), _processes(), os.getpid(),
                   resolve_executable)


def start(args: argparse.Namespace) -> int:
    """Execute the prepared recorded startup without modifying production aliases.

    Args:
        args: Parsed mode, profile, recording, repository and strict-check selectors.

    Returns:
        Recorded shell-compatible exit status after cleanup and finalization.

    Raises:
        ValueError: An installed copy needs an explicit repository for traceability.
        OSError: A required file, executable or recording operation is unavailable.
        RuntimeError: Ownership, host preparation or recording cannot be established.
    """
    repo = args.repo or args.source_root
    if repo is None:
        candidate = Path(__file__).resolve().parents[3]
        if not (candidate / '.git').exists():
            raise ValueError('cannot locate source repository; provide --repo explicitly')
        repo = candidate
    options = StartupOptions(
        args.mode, args.robot, 'none' if args.mode in ('chassis', 'teleop') else args.site,
        (args.runs_root or default_runs_root()).expanduser().absolute(),
        repo.expanduser().absolute(), DEFAULT_LOCK_PATH,
        command=getattr(args, 'invocation', ('navrun', 'up', '--mode', args.mode)))
    result = run_recorded_startup(
        options, lambda path: _plan(args, path),
        lambda plan: bind_commands(plan, strict=args.strict, resolve=resolve_executable),
        bind_host_steps(Path('/dev/shm')))
    print(f'navrun: run record {result.run.path} '
          f'({result.exit_reason}, exit {result.exit_code})', file=sys.stderr)
    return result.exit_code


def run(args: argparse.Namespace) -> int:
    """Dispatch either a read-only preview or the explicit recorded startup.

    Args:
        args: Parsed startup arguments.

    Returns:
        Zero for a valid preview, two for invalid setup, or the runtime exit code.
        A valid preview never certifies hardware or deployment acceptance.
    """
    try:
        if not args.dry_run:
            return start(args)
        report = preview(args)
    except (ValueError, LookupError, OSError, PreflightError, RuntimeError) as error:
        print(f'navrun up: {error}', file=sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
    return 2 if report['errors'] else 0
