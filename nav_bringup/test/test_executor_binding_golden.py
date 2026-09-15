"""Prove legacy parsed settings remain equal without importing or executing its runtime."""

import ast
import os
from pathlib import Path
import re

import pytest

from nav_bringup.launching.graph import GraphOptions
from nav_bringup.navcfg import render
from nav_bringup.navrun.executor_binding import bind_executor


SOURCE = Path(__file__).resolve().parents[2]


def _old_parser():
    source = SOURCE / 'david_nav/david_nav/navigation_executor.py'
    tree = ast.parse(source.read_text())
    selected = [node for node in tree.body if (
        isinstance(node, ast.Assign) and all(
            isinstance(target, ast.Name) and target.id.startswith('DEFAULT_')
            for target in node.targets)) or (
        isinstance(node, ast.FunctionDef) and node.name == '_parse_cli_args')]
    namespace = {'os': os}
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['_parse_cli_args']


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_david_settings_equal_old_entry_after_legacy_parser(tmp_path, mode):
    """Explicitly supplied store/action defaults do not alter any legacy configuration field."""
    package = SOURCE / 'david_config'
    profile = render.load_yaml(package / 'robot/david.yaml')
    site_path = package / 'sites/daolan/site.yaml'
    site = render.load_yaml(site_path)
    options = GraphOptions(mode, str(tmp_path), start_executor=True)
    result = bind_executor(options, profile, site, site_path.parent)
    old_arguments = [
        '--server-uri', site['agent_server']['uri'], '--reconnect-delay', '3',
        '--pose-file', str(Path(site['pois']['named_poses_file']).expanduser())]
    if mode == 'precise':
        launch = (SOURCE / 'david_nav/launch/precise_nav_bringup.launch.py').read_text()
        match = re.search(r"'dock_poi_ids', default_value='([^']*)'", launch)
        assert match
        identifiers = [value.strip() for value in match.group(1).split(',') if value.strip()]
        assert site['pois']['dock_poi_ids'] == identifiers
        for identifier in identifiers:
            old_arguments.extend(['--dock-poi-id', identifier])
    parse = _old_parser()
    assert parse(list(result.executor_arguments)) == parse(old_arguments)
    assert result.ready_timeout_sec == 120.0
