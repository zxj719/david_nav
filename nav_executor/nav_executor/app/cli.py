"""Frozen help and argparse surface; ROS startup is owned by the outer runtime."""

import argparse
from typing import Sequence

from nav_tasks.api import ConfigError


def build_help_text() -> str:
    """Return the exact legacy help text without importing ROS or reading configuration.

    Returns:
        Frozen G16 help, including the historical executable name.
    """
    return '\n'.join([
        'usage: navigation_executor.py [--server-uri URI] [--poi-store-file PATH] '
        '[--pose-file PATH] [ROS args...]',
        '',
        'Robot-side WebSocket navigation executor for david_nav.',
        '',
        'options:',
        '  --server-uri URI        Agent WebSocket server (nav WS v1). Default:',
        '                          $NAV_WS_URL or ws://127.0.0.1:8101',
        '  --poi-store-file PATH   Local YAML file used to persist executor-owned POIs',
        '  --pose-file PATH        named_poses.json to import into the POI store at startup',
        '  --action-name NAME      NavigateToPose action server to drive',
        '                          (default: /navigate_to_pose; waypoint stack uses',
        '                          /waypoint_follower/navigate)',
        '  --reconnect-delay SEC   Delay between WebSocket reconnect attempts (default: 3.0)',
        '  --nav-retry-count N     Silent re-publish attempts after a nav failure before',
        '                          reporting on_error to the server (default: 1)',
        '  --nav-retry-delay SEC   Wait between a nav failure and the re-publish (default: 1.0)',
        '  --dock-poi-id ID        Navigate this POI with two-stage precise docking through',
        '                          the docking action (repeatable; never snapped or retried)',
        '  --dock-action-name NAME Docking action server (default: /dock_to_pose)',
        '  -h, --help              Show this help message and exit',
    ])


class _Parser(argparse.ArgumentParser):
    def error(self, message):
        raise ConfigError(message)


def parse_arguments(raw_args: Sequence[str]) -> tuple[dict, list[str]]:
    """Parse known application flags without abbreviating or consuming ROS arguments.

    Args:
        raw_args: Arguments excluding the executable name.

    Returns:
        Explicit overrides and untouched unknown arguments in input order.

    Raises:
        ConfigError: A known flag lacks a value or has an invalid numeric value.
    """
    from nav_executor.app.config import ExecutorConfig
    from dataclasses import fields

    parser = _Parser(add_help=False, allow_abbrev=False, argument_default=argparse.SUPPRESS)
    parser.add_argument('-h', '--help', action='store_true')
    parser.add_argument('--config')
    parser.add_argument('--robot-profile')
    for field in fields(ExecutorConfig):
        if field.name == 'dock_poi_ids':
            parser.add_argument('--dock-poi-id', dest=field.name, action='append')
        else:
            flag = '--' + field.name.replace('_', '-')
            if field.name == 'nav_retry_delay_sec':
                flag = '--nav-retry-delay'
            parser.add_argument(flag, dest=field.name,
                                type=field.type if field.type in (int, float) else str)
    parsed, remaining = parser.parse_known_args(list(raw_args))
    return vars(parsed), remaining
