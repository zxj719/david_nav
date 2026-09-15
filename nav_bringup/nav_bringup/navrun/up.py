"""Startup CLI preview; actual startup remains disabled until lifecycle wiring is complete."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any, Dict, Tuple

from nav_bringup.launching.executables import resolve_executable
from nav_bringup.navcfg import schema
from nav_bringup.navrun.preflight import PreflightError, ProcessInfo
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
    request = UpRequest(args.mode, args.robot, args.site, root / 'dry_run_preview',
                        args.chassis, args.record, args.no_executor)

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
    plan = plan_up(request, inputs, schema.load_catalog(paths), _processes(), os.getpid(),
                   resolve_executable)
    return report_plan(plan)


def run(args: argparse.Namespace) -> int:
    """Dispatch an up request without enabling unfinished runtime orchestration.

    Args:
        args: Parsed startup arguments.

    Returns:
        Zero for a valid preview, two for missing dependencies or unsupported live
        startup. A valid preview does not imply the runtime path is implemented.
    """
    if not args.dry_run:
        print('navrun up: live startup is not yet enabled; use --dry-run to inspect '
              'the plan. Production aliases are unchanged.', file=sys.stderr)
        return 2
    try:
        report = preview(args)
    except (ValueError, LookupError, OSError, PreflightError) as error:
        print(f'navrun up: {error}', file=sys.stderr)
        return 2
    print(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2))
    return 2 if report['errors'] else 0
