"""The Nav2 base parameter tree agrees with its binding table (PROGRESS P2.3)."""

from pathlib import Path

import yaml

from nav_bringup.lint import literals
from nav_bringup.navcfg import render
from nav_bringup.navcfg import schema

CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'
OTHER_ROBOT = {
    'frames': {'map': 'world', 'odom': 'wheel_odom', 'nav_base': 'chassis_center'},
    'footprint': [[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]],
    'limits': {'max_vel_x': 0.5, 'max_vel_theta': 1.0},
    'goal': {'xy_tolerance': 0.2, 'yaw_tolerance': 0.3},
    'nav2': {'odom_topic': '/wheel/odom', 'scan_topic': '/front/scan', 'map_topic': '/map'},
    'chassis': {'odom_topic': '/wheel/odom'},
}


def _load(name):
    return yaml.safe_load((CONFIG_DIR / name).read_text(encoding='utf-8'))


def _bindings():
    return render.parse_bindings(_load('nav2_bindings.yaml'), 'nav2_bindings.yaml')


def _string_values(tree):
    if isinstance(tree, str):
        return [tree]
    if isinstance(tree, dict):
        tree = list(tree.values())
    if not isinstance(tree, list):
        return []
    return [value for child in tree for value in _string_values(child)]


def test_markers_are_exactly_the_bound_parameters():
    """Every marker in nav2_base.yaml is bound and every bound parameter holds its marker."""
    bound = {path: key for key, binding in _bindings().items() for path in binding.paths}

    assert render.find_markers(_load('nav2_base.yaml')) == bound


def test_base_names_no_robot_frames_or_robot_topics():
    """Frame names and robot topic prefixes come from the profile (plan §5.4)."""
    offending = [
        value for value in _string_values(_load('nav2_base.yaml'))
        if value in literals.FORBIDDEN_EXACT
        or any(part in value for part in literals.FORBIDDEN_SUBSTRINGS)]

    assert not offending


def test_bound_profile_keys_are_required_by_the_schema():
    """The schema check rejects a profile that lacks a bound key, before rendering."""
    catalog = schema.load_catalog([CONFIG_DIR / 'schema.yaml'])

    for key in _bindings():
        spec = catalog.get(key)
        assert spec is not None, key
        assert spec.required, key


def test_another_robot_renders_from_its_profile_alone():
    """A different profile fills every bound parameter and leaves no marker (plan §12.6)."""
    tree = render.render(_load('nav2_base.yaml'), _bindings(), OTHER_ROBOT)

    assert render.find_markers(tree) == {}
    local_costmap = tree['local_costmap']['local_costmap']['ros__parameters']
    assert local_costmap['robot_base_frame'] == 'chassis_center'
    assert local_costmap['global_frame'] == 'wheel_odom'
    assert yaml.safe_load(local_costmap['footprint']) == OTHER_ROBOT['footprint']
    assert tree['behavior_server']['ros__parameters']['global_frame'] == 'wheel_odom'
    follow_path = tree['controller_server']['ros__parameters']['FollowPath']
    assert (follow_path['max_vel_x'], follow_path['max_speed_xy']) == (0.5, 0.5)
    assert tree['velocity_smoother']['ros__parameters']['odom_topic'] == '/wheel/odom'
