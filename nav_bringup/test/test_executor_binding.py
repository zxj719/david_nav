"""Bind the legacy executor from site/profile data without importing its runtime."""

from dataclasses import replace
from pathlib import Path

import pytest

from nav_bringup.launching.graph import GraphOptions
from nav_bringup.navrun.executor_binding import bind_executor


def _site():
    return {'agent_server': {'uri': 'ws://127.0.0.1:18101'}, 'pois': {
        'named_poses_file': 'poses/named poses.json', 'store_file': 'state/store.yaml',
        'dock_poi_ids': ['DOCK_A', 'DOCK_A', 'DOCK_B']}}


def test_tour_keeps_legacy_routing_and_exact_path_arguments(tmp_path):
    """Spaces remain inside one argv item and tour does not silently become docking mode."""
    options = GraphOptions('tour', str(tmp_path / 'run'), start_executor=True)
    bound = bind_executor(options, {}, _site(), tmp_path)
    assert bound.executor_arguments == (
        '--server-uri', 'ws://127.0.0.1:18101', '--reconnect-delay', '3',
        '--pose-file', str(tmp_path / 'poses/named poses.json'),
        '--poi-store-file', str(tmp_path / 'state/store.yaml'),
        '--action-name', '/navigate_to_pose')
    assert bound.ready_timeout_sec == 120.0
    assert not (tmp_path / 'run').exists()
    assert not (tmp_path / 'state').exists()


def test_precise_adds_site_specific_dock_ids_without_duplicates(tmp_path):
    """Only precise routing carries dock IDs, preserving the U-1 alias behavior."""
    options = GraphOptions('precise', str(tmp_path / 'run'), start_executor=True)
    bound = bind_executor(options, {'executor': {'reconnect_delay_sec': 4.0}}, _site(), tmp_path)
    assert bound.executor_arguments[-6:] == (
        '--dock-action-name', '/dock_to_pose', '--dock-poi-id', 'DOCK_A',
        '--dock-poi-id', 'DOCK_B')
    assert bound.executor_arguments[3] == '4'


def test_disabled_executor_does_not_require_site_configuration(tmp_path):
    """Bench/chassis tools retain no server or POI dependency."""
    options = GraphOptions('chassis', str(tmp_path / 'run'))
    assert bind_executor(options, {}, {}, tmp_path) is options


def test_explicit_low_level_arguments_remain_frozen(tmp_path):
    """A supervisor-prepared explicit command is not rewritten by a second binding pass."""
    options = GraphOptions('tour', str(tmp_path / 'run'), start_executor=True,
                           executor_arguments=('--explicit', 'value'))
    assert bind_executor(options, {}, {}, tmp_path) is options


@pytest.mark.parametrize('uri', ['', 'https://example.test', 'ws://', 'ws://host:invalid'])
def test_invalid_server_rejected_before_startup(tmp_path, uri):
    """A malformed server cannot reach launch as an unvalidated generic argument."""
    site = _site()
    site['agent_server']['uri'] = uri
    with pytest.raises(ValueError):
        bind_executor(GraphOptions('tour', str(tmp_path), start_executor=True), {}, site, tmp_path)


@pytest.mark.parametrize('value', [-1, float('nan'), True, 'three'])
def test_bad_reconnect_delay_is_rejected(tmp_path, value):
    """Non-finite or nonnumeric intervals cannot disable retry timing accidentally."""
    with pytest.raises(ValueError):
        bind_executor(GraphOptions('tour', str(tmp_path), start_executor=True),
                      {'executor': {'reconnect_delay_sec': value}}, _site(), tmp_path)


def test_site_paths_expand_home_and_preserve_explicit_action(tmp_path):
    """Input and mutable POI-store paths are resolved, but neither is opened or changed."""
    site = _site()
    site['pois']['store_file'] = '~/runtime/pois.yaml'
    options = replace(GraphOptions('tour', str(tmp_path), start_executor=True),
                      ready_action='/custom/navigate')
    bound = bind_executor(options, {}, site, Path('/site/data'))
    assert str(Path.home() / 'runtime/pois.yaml') in bound.executor_arguments
    assert bound.executor_arguments[-1] == '/custom/navigate'
