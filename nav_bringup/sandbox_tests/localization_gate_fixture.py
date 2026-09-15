"""Run recorded navigation waiting with private host operations and a forbidden launch."""

import argparse
from dataclasses import replace
from pathlib import Path
import sys

from nav_bringup.navrun import cli, up
from recorded_chassis_fixture import configure


def main() -> int:
    """Wait through the real CLI while preventing any navigation launch.

    Returns:
        The actual recorded CLI status after localization wait or interruption.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    configure(args.root)
    original = up.bind_commands

    def bind(plan, **kwargs):
        commands = original(plan, **kwargs)
        return replace(
            commands, launch=(sys.executable, '-c', 'raise RuntimeError("unexpected launch")'),
            gates=tuple(replace(gate, timeout_sec=0.2) for gate in commands.gates))

    up.bind_commands = bind
    source = Path(__file__).resolve().parents[2]
    return cli.main([
        'up', '--mode', 'tour', '--robot', 'example_diffdrive', '--site', 'offline',
        '--record', 'off', '--no-executor', '--source-root', str(source), '--repo', str(source),
        '--runs-root', str(args.root / 'runs')])


if __name__ == '__main__':
    sys.exit(main())
