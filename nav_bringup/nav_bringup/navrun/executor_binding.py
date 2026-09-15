"""Translate frozen site/profile settings into the unchanged legacy executor CLI."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import urlsplit

from nav_bringup.launching.graph import GraphOptions
from nav_bringup.navcfg import render


def _path(value: Any, directory: Path) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError('executor site paths must be nonempty strings')
    path = Path(value).expanduser()
    return str((path if path.is_absolute() else directory / path).resolve())


def _duration(value: Any, name: str, allow_zero: bool = False) -> float:
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero)):
        requirement = 'nonnegative' if allow_zero else 'positive'
        raise ValueError(f'{name} must be finite and {requirement}')
    return float(value)


def bind_executor(
    options: GraphOptions, profile: Mapping[str, Any], site: Mapping[str, Any],
    site_directory: Path,
) -> GraphOptions:
    """Bind server, point paths, retry interval and precise-only dock routing.

    Args:
        options: Selected graph; disabled or already explicit executors are left untouched.
        profile: Effective robot/mode settings for executor and startup timing.
        site: Frozen site mapping, including agent_server and pois.
        site_directory: Original site directory for relative point/store paths.

    Returns:
        Updated immutable graph options. No point/store file is opened or written,
        no runtime implementation imported, and every value remains one argv item.

    Raises:
        ValueError: Server, paths, timing, routing or selected mode is invalid.
    """
    if not options.start_executor or options.executor_arguments:
        return options
    if options.mode not in ('tour', 'precise'):
        raise ValueError('legacy executor requires tour or precise mode')
    uri = render.profile_value(site, 'agent_server.uri')
    if not isinstance(uri, str) or not uri.strip():
        raise ValueError('agent_server.uri must be a nonempty WebSocket URI')
    parsed = urlsplit(uri)
    if parsed.scheme not in ('ws', 'wss') or not parsed.hostname:
        raise ValueError('agent_server.uri must use ws/wss and include a host')
    # Accessing port validates malformed explicit ports without making a connection.
    if parsed.port is not None and parsed.port == 0:
        raise ValueError('agent_server.uri port must be positive')
    settings, startup = profile.get('executor', {}), profile.get('startup', {})
    if not isinstance(settings, Mapping) or not isinstance(startup, Mapping):
        raise ValueError('executor and startup settings must be mappings')
    delay = _duration(settings.get('reconnect_delay_sec', 3.0), 'reconnect delay', True)
    timeout = _duration(startup.get('nav_ready_timeout_sec', 120.0), 'navigation ready timeout')
    args = [
        '--server-uri', uri, '--reconnect-delay', f'{delay:g}',
        '--pose-file', _path(render.profile_value(site, 'pois.named_poses_file'), site_directory),
        '--poi-store-file', _path(render.profile_value(site, 'pois.store_file'), site_directory),
        '--action-name', options.ready_action,
    ]
    if options.mode == 'precise':
        identifiers = site.get('pois', {}).get('dock_poi_ids', [])
        if not isinstance(identifiers, list) or not all(
                isinstance(identifier, str) and identifier.strip() for identifier in identifiers):
            raise ValueError('pois.dock_poi_ids must be a list of nonempty strings')
        if identifiers:
            args += ['--dock-action-name', '/dock_to_pose']
            for identifier in dict.fromkeys(item.strip() for item in identifiers):
                args += ['--dock-poi-id', identifier]
    return replace(options, executor_arguments=tuple(args), ready_timeout_sec=timeout)
