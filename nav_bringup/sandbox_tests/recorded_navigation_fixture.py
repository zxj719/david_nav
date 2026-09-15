"""Actual recorded Nav2 up with a fictional robot and private host boundaries."""

import argparse
from pathlib import Path
import sys

from nav_bringup.navrun import cli
from recorded_chassis_fixture import configure


def main() -> int:
    """Launch goal-free example navigation; never enable the WS executor or real driver.

    Returns:
        Status from the real recorded navigation CLI after owned cleanup.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--mode', choices=('tour', 'precise'), default='tour')
    parser.add_argument('--strict', action='store_true')
    parser.add_argument('--mode-file', type=Path)
    args = parser.parse_args()
    configure(args.root)
    source = Path(__file__).resolve().parents[2]
    return cli.main([
        'up', '--mode', args.mode, '--robot', 'example_diffdrive', '--site', 'offline',
        '--record', 'off', '--no-executor', '--source-root', str(source), '--repo', str(source),
        '--runs-root', str(args.root / 'runs'), *(['--strict'] if args.strict else []),
        *(['--mode-file', str(args.mode_file)] if args.mode_file else [])])


if __name__ == '__main__':
    sys.exit(main())
