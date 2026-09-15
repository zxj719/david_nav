"""Execute real up/launch with fake chassis and private, fake host operations only."""

import argparse
from dataclasses import replace
import os
from pathlib import Path
import sys

from nav_bringup.navrun import cli, up
from nav_bringup.navrun.dds_hygiene import MappingView
from nav_bringup.navrun.preflight import ProcessInfo
from nav_bringup.navrun.runtime_host import bind_host_steps


def configure(root: Path, tty: str = '') -> None:
    """Replace host boundaries only inside the domain-99 fake-profile fixture.

    Args:
        root: Absolute test-owned directory for DDS, records and the lease.
        tty: Private terminal path, or empty for chassis-only startup.

    Raises:
        ValueError: The fixture is outside domain 99 or its root is not absolute.
    """
    if os.environ.get('ROS_DOMAIN_ID') != '99' or not root.is_absolute():
        raise ValueError('this fixture requires domain 99 and an absolute private root')
    root.mkdir(parents=True, exist_ok=True)
    (root / 'dds').mkdir()

    def own_view():
        return (ProcessInfo(os.getpid(), os.getppid(), 1.0, ('navrun',), os.getuid()),)

    def forbidden(*values):
        raise AssertionError('sandbox must not configure platform or signal residuals')

    def private_host(_directory):
        return bind_host_steps(root / 'dds', forbidden, own_view, forbidden,
                               lambda: MappingView(frozenset(), True))

    up.DEFAULT_LOCK_PATH = root / 'lease'
    up._processes = own_view
    up.bind_host_steps = private_host
    if tty:
        original_bind = up.bind_commands

        def private_keyboard(plan, **kwargs):
            commands = original_bind(plan, **kwargs)
            helper = Path(__file__).with_name('teleop_delegation_fixture.py')
            return replace(commands, tty_device=tty, keyboard=(
                sys.executable, str(helper), str(root / 'lease'), *commands.keyboard[1:]))

        up.bind_commands = private_keyboard


def main() -> int:
    """Run the sandbox CLI fixture after replacing every host mutation boundary.

    Returns:
        The actual navrun CLI exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--mode', choices=('chassis', 'teleop'), default='chassis')
    parser.add_argument('--tty', default='')
    args = parser.parse_args()
    configure(args.root, args.tty if args.mode == 'teleop' else '')
    source = Path(__file__).resolve().parents[2]
    return cli.main([
        'up', '--mode', args.mode, '--robot', 'example_diffdrive', '--site', 'offline',
        '--record', 'off', '--no-executor', '--source-root', str(source), '--repo', str(source),
        '--runs-root', str(args.root / 'runs')])


if __name__ == '__main__':
    sys.exit(main())
