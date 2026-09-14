"""Command line entry point ``navrun``.

Example:
    navrun wrap --mode tour --alias 4 -- ~/ros2_ws/src/david_nav/start_navigation_headless.sh
    navrun doctor
    navrun watch-bt --run-dir ~/nav_runs/latest --base-frame <frames.nav_base>
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
    return _wrap(parser, args)


def _watch_bt(args: argparse.Namespace) -> int:
    directory = args.run_dir
    if directory is None and os.environ.get('NAVRUN_RUN_DIR'):
        directory = Path(os.environ['NAVRUN_RUN_DIR'])
    run = None
    if directory is not None:
        directory = directory.expanduser()
        run = run_dir.RunDirectory(directory.parent, directory.name)
    return bt_log_watcher.spin(
        run,
        base_frame=args.base_frame,
        topic=args.topic,
        map_frame=args.map_frame,
        min_interval_sec=args.min_interval,
    )


def _doctor(args: argparse.Namespace) -> int:
    results = checks.run_checks(checks.default_context(os.environ))
    if args.json:
        print(json.dumps([result.to_dict() for result in results], ensure_ascii=False, indent=2))
    else:
        print(checks.format_report(results), end='')
    return 1 if checks.worst_status(results) is checks.Status.FAIL else 0


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
