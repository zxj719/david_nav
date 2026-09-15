"""Command line of navcfg, the robot configuration tool (refactor plan §12).

``navcfg render`` writes the Nav2 parameter file of a robot profile and mode.
``validate`` and ``doc`` follow (PROGRESS P2.5, P2.6).
"""

import argparse
from pathlib import Path
import sys
from typing import Optional, Sequence

from nav_bringup.navcfg import render

PACKAGE_NAME = 'nav_bringup'
BASE_FILE = 'nav2_base.yaml'
BINDINGS_FILE = 'nav2_bindings.yaml'


def _installed_config(file_name: str) -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(PACKAGE_NAME)) / 'config' / file_name


def build_parser() -> argparse.ArgumentParser:
    """Build the parser of the navcfg command line.

    Returns:
        The argument parser.
    """
    parser = argparse.ArgumentParser(prog='navcfg', description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest='command', required=True)
    render_parser = commands.add_parser(
        'render', help='write the Nav2 parameter file of a robot profile and mode')
    render_parser.add_argument(
        '--profile', required=True, type=Path, help='robot profile, e.g. robot/david.yaml')
    render_parser.add_argument(
        '--mode', type=Path, help='mode overlay, e.g. modes/precise.yaml (default: none)')
    render_parser.add_argument(
        '--base', type=Path,
        help=f'base parameter tree (default: share/{PACKAGE_NAME}/config/{BASE_FILE})')
    render_parser.add_argument(
        '--bindings', type=Path,
        help=f'binding table (default: share/{PACKAGE_NAME}/config/{BINDINGS_FILE})')
    render_parser.add_argument(
        '--output', required=True, type=Path, help='parameter file to write')
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run navcfg.

    Args:
        argv: Arguments without the program name; ``sys.argv[1:]`` when None.

    Returns:
        0 on success, 2 when the inputs are missing, malformed or inconsistent.
    """
    args = build_parser().parse_args(argv)
    try:
        base = args.base or _installed_config(BASE_FILE)
        bindings = args.bindings or _installed_config(BINDINGS_FILE)
        tree = render.render_files(base, bindings, args.profile, args.mode)
        render.write_parameters(args.output, tree)
    except (render.RenderError, LookupError, OSError) as error:
        print(f'navcfg render: {error}', file=sys.stderr)
        return 2
    print(f'navcfg render: wrote {args.output}')
    return 0
