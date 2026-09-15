"""Execute real up/launch with fake chassis and private, fake host operations only."""

import argparse
import os
from pathlib import Path
import sys

from nav_bringup.navrun import cli, up
from nav_bringup.navrun.dds_hygiene import MappingView
from nav_bringup.navrun.preflight import ProcessInfo
from nav_bringup.navrun.runtime_host import bind_host_steps


def main() -> int:
    """Run the sandbox CLI fixture after replacing every host mutation boundary.

    Returns:
        The actual navrun CLI exit code.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '99' or not args.root.is_absolute():
        parser.error('this fixture requires domain 99 and an absolute private root')
    args.root.mkdir(parents=True, exist_ok=True)
    (args.root / 'dds').mkdir()
    source = Path(__file__).resolve().parents[2]

    def own_view():
        return (ProcessInfo(os.getpid(), os.getppid(), 1.0, ('navrun',), os.getuid()),)

    def forbidden(*values):
        raise AssertionError('sandbox must not configure platform or signal residuals')

    def private_host(_directory):
        return bind_host_steps(args.root / 'dds', forbidden, own_view, forbidden,
                               lambda: MappingView(frozenset(), True))

    up.DEFAULT_LOCK_PATH = args.root / 'lease'
    up._processes = own_view
    up.bind_host_steps = private_host
    return cli.main([
        'up', '--mode', 'chassis', '--robot', 'example_diffdrive', '--site', 'offline',
        '--record', 'off', '--no-executor', '--source-root', str(source), '--repo', str(source),
        '--runs-root', str(args.root / 'runs')])


if __name__ == '__main__':
    sys.exit(main())
