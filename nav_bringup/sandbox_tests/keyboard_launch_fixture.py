#!/usr/bin/env python3
"""Run installed convenience launch with a private controlling TTY and host shim."""

import argparse
import fcntl
import importlib.util
import os
from pathlib import Path
import sys
import termios

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import SetLaunchConfiguration

from nav_bringup.navrun import cli
from recorded_chassis_fixture import configure


def main() -> int:
    """Exercise actual launch/CLI while replacing only test-owned host boundaries.

    Returns:
        The actual launch result or delegated navrun exit code.

    Raises:
        ValueError: The fixture is not in domain 99 or was passed a non-test robot.
        OSError: The test-owned terminal cannot become this session's controlling TTY.
    """
    if os.environ.get('ROS_DOMAIN_ID') != '99':
        raise ValueError('fixture requires domain 99')
    if sys.argv[1:2] == ['up']:
        args = cli.build_parser().parse_args(sys.argv[1:])
        if args.robot != 'example_diffdrive' or args.mode != 'teleop':
            raise ValueError('fixture only permits fake chassis teleop')
        configure(Path(os.environ['DAVID_TELEOP_FIXTURE_ROOT']), '/dev/tty')
        return cli.main(sys.argv[1:])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--tty-fd', type=int)
    args = parser.parse_args()
    if not args.root.is_absolute():
        parser.error('root must be absolute')
    if args.tty_fd is not None:
        fcntl.ioctl(args.tty_fd, termios.TIOCSCTTY, 0)
        os.close(args.tty_fd)
    os.environ['DAVID_TELEOP_FIXTURE_ROOT'] = str(args.root)
    path = Path(get_package_share_directory('chassis_teleop')) / 'launch'
    spec = importlib.util.spec_from_file_location(
        'installed_keyboard_launch', path / 'keyboard_teleop.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.resolve_executable = lambda *_: str(Path(__file__).resolve())
    source = Path(__file__).resolve().parents[2]
    service = LaunchService()
    selectors = {
        'robot': 'example_diffdrive', 'record': 'off', 'layout': 'uiojkl',
        'linear_speed': '0.1', 'angular_speed': '0.2', 'source_root': str(source),
        'repo': str(source), 'runs_root': str(args.root / 'runs')}
    service.include_launch_description(LaunchDescription([
        *(SetLaunchConfiguration(name, value) for name, value in selectors.items()),
        *module.generate_launch_description().entities]))
    return service.run()


if __name__ == '__main__':
    sys.exit(main())
