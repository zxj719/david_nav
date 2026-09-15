"""Command line entry point ``navrun``.

Example:
    navrun wrap --mode tour --alias 4 -- ~/ros2_ws/src/david_nav/start_navigation_headless.sh
    navrun doctor
    navrun watch-bt --run-dir ~/nav_runs/latest --base-frame <frames.nav_base>
    navrun dump-params --run-dir ~/nav_runs/latest
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Dict, Optional, Sequence

from nav_bringup.doctor import checks
from nav_bringup.navrun import wrap
from nav_bringup.recorder import bt_log_watcher
from nav_bringup.recorder import events
from nav_bringup.recorder import param_dump
from nav_bringup.recorder import run_dir

DEFAULT_REPO_DIR = '~/ros2_ws/src'


def build_parser() -> argparse.ArgumentParser:
    """Create the ``navrun`` argument parser.

    Returns:
        The parser with every subcommand registered.
    """
    parser = argparse.ArgumentParser(
        prog='navrun', description='Navigation run supervisor and recorder.')
    subparsers = parser.add_subparsers(dest='subcommand', required=True)

    wrap_parser = subparsers.add_parser(
        'wrap', help='record a run around an existing launch command')
    wrap_parser.add_argument('--mode', required=True, help='run mode, e.g. tour or precise')
    wrap_parser.add_argument(
        '--alias', default=None, help='shell alias that started the run (default: $NAVRUN_ALIAS)')
    wrap_parser.add_argument('--robot', default=None, help='robot profile name')
    wrap_parser.add_argument('--site', default=None, help='site profile name')
    wrap_parser.add_argument(
        '--runs-root', type=Path, default=None, help='default: $NAV_RUNS_ROOT or ~/nav_runs')
    wrap_parser.add_argument(
        '--repo', type=Path, default=None, help='git repository to record (default: '
        '$NAVRUN_REPO or ~/ros2_ws/src)')
    wrap_parser.add_argument(
        '--file', action='append', default=[], metavar='NAME=PATH',
        help='input file to fingerprint in the manifest; repeatable')
    wrap_parser.add_argument(
        '--sample-interval', type=float, default=1.0, help='seconds between resource samples')
    wrap_parser.add_argument(
        '--no-gc', action='store_true', help='do not prune old runs before starting')
    wrap_parser.add_argument(
        'command', nargs=argparse.REMAINDER, help='command to run, placed after --')

    doctor_parser = subparsers.add_parser(
        'doctor', help='read-only checks for known navigation failure modes')
    doctor_parser.add_argument(
        '--json', action='store_true', help='print results as JSON instead of text')

    watch_parser = subparsers.add_parser(
        'watch-bt', help='report behavior tree escape and recovery triggers')
    watch_parser.add_argument(
        '--run-dir', type=Path, default=None,
        help='run directory to record into (default: $NAVRUN_RUN_DIR; unset prints only)')
    watch_parser.add_argument('--topic', default='/behavior_tree_log', help='tree log topic')
    watch_parser.add_argument('--map-frame', default='map', help='frame of the reported pose')
    watch_parser.add_argument(
        '--base-frame', required=True,
        help='robot frame whose pose is reported (robot profile frames.nav_base)')
    watch_parser.add_argument(
        '--min-interval', type=float, default=0.5,
        help='minimum seconds between reports of the same trigger')

    dump_defaults = param_dump.DumpSettings()
    dump_parser = subparsers.add_parser(
        'dump-params', help='dump the effective parameters of every running node')
    dump_parser.add_argument(
        '--run-dir', type=Path, default=None,
        help='run directory; dumps go to its params/effective (default: $NAVRUN_RUN_DIR)')
    dump_parser.add_argument(
        '--output-dir', type=Path, default=None,
        help='write the dumps here instead of into a run directory')
    dump_parser.add_argument(
        '--spin-time', type=float, default=dump_defaults.spin_time_sec,
        help='discovery seconds for each ros2 call')
    dump_parser.add_argument(
        '--timeout', type=float, default=dump_defaults.timeout_sec,
        help='seconds before one ros2 call is abandoned')
    dump_parser.add_argument(
        '--attempts', type=int, default=dump_defaults.attempts, help='dump attempts per node')
    return parser


def parse_file_arguments(values: Sequence[str]) -> Dict[str, Path]:
    """Parse repeated ``NAME=PATH`` arguments.

    Args:
        values: Raw argument values.

    Returns:
        Mapping of each name to its expanded path.

    Raises:
        ValueError: If a value has no ``=`` or an empty name or path.
    """
    files: Dict[str, Path] = {}
    for value in values:
        name, separator, path = value.partition('=')
        if not separator or not name or not path:
            raise ValueError(f'--file expects NAME=PATH, got {value!r}')
        files[name] = Path(path).expanduser()
    return files


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the ``navrun`` command line.

    Args:
        argv: Arguments without the program name; defaults to ``sys.argv[1:]``.

    Returns:
        The process exit status.
    """
    parser = build_parser()
    args = parser.parse_args(sys.argv[1:] if argv is None else list(argv))
    if args.subcommand == 'doctor':
        return _doctor(args)
    if args.subcommand == 'watch-bt':
        return _watch_bt(args)
    if args.subcommand == 'dump-params':
        return _dump_params(parser, args)
    return _wrap(parser, args)


def _run_directory(explicit: Optional[Path]) -> Optional[run_dir.RunDirectory]:
    directory = explicit
    if directory is None and os.environ.get('NAVRUN_RUN_DIR'):
        directory = Path(os.environ['NAVRUN_RUN_DIR'])
    if directory is None:
        return None
    directory = directory.expanduser()
    return run_dir.RunDirectory(directory.parent, directory.name)


def _doctor(args: argparse.Namespace) -> int:
    results = checks.run_checks(checks.default_context(os.environ))
    if args.json:
        print(json.dumps([result.to_dict() for result in results], ensure_ascii=False, indent=2))
    else:
        print(checks.format_report(results), end='')
    return 1 if checks.worst_status(results) is checks.Status.FAIL else 0


def _watch_bt(args: argparse.Namespace) -> int:
    return bt_log_watcher.spin(
        _run_directory(args.run_dir),
        base_frame=args.base_frame,
        topic=args.topic,
        map_frame=args.map_frame,
        min_interval_sec=args.min_interval,
    )


def _dump_params(parser: argparse.ArgumentParser, args: argparse.Namespace) -> int:
    if args.attempts < 1:
        parser.error('--attempts must be at least 1')
    run = None if args.output_dir is not None else _run_directory(args.run_dir)
    if args.output_dir is not None:
        output_dir = args.output_dir.expanduser()
    elif run is not None:
        output_dir = run.params_effective
    else:
        parser.error('dump-params needs --output-dir, --run-dir or $NAVRUN_RUN_DIR')
    settings = param_dump.DumpSettings(
        spin_time_sec=args.spin_time, timeout_sec=args.timeout, attempts=args.attempts)
    report = param_dump.dump_graph(output_dir, settings=settings)
    if run is not None:
        events.EventLog(run.events).append(
            wrap.EVENT_SOURCE, 'params_dumped', 'info' if report.ok else 'warn',
            report.to_dict())
    problems = ', node list failed' if report.list_failed else ''
    print(
        f'navrun: dumped {len(report.dumped)} of {len(report.nodes)} nodes into {output_dir} '
        f'(skipped {len(report.skipped)}, failed {len(report.failed)}{problems})',
        file=sys.stderr,
    )
    return 0 if report.ok else 1


def _wrap(parser: argparse.ArgumentParser, args: argparse.Namespace) -> int:
    command = list(args.command)
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('wrap needs a command after --')
    try:
        files = parse_file_arguments(args.file)
    except ValueError as error:
        parser.error(str(error))
    repo_dir = args.repo or Path(os.environ.get('NAVRUN_REPO', DEFAULT_REPO_DIR))
    options = wrap.WrapOptions(
        command=command,
        mode=args.mode,
        runs_root=(args.runs_root or run_dir.default_runs_root()).expanduser(),
        repo_dir=repo_dir.expanduser(),
        alias=args.alias,
        robot=args.robot,
        site=args.site,
        files=files,
        sample_interval_sec=args.sample_interval,
        gc=not args.no_gc,
    )
    try:
        result = wrap.run_wrapped(options)
    except ValueError as error:
        parser.error(str(error))
    print(
        f'navrun: run record {result.run.path} '
        f'({result.exit_reason}, exit {result.exit_code})',
        file=sys.stderr,
    )
    return result.exit_code


if __name__ == '__main__':
    sys.exit(main())
