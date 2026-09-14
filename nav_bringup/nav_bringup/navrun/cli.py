"""Command line entry point ``navrun``.

Example:
    navrun wrap --mode tour --alias 4 -- ~/ros2_ws/src/david_nav/start_navigation_headless.sh
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
from typing import Dict, Optional, Sequence

from nav_bringup.navrun import wrap
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
    return _wrap(parser, args)


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
