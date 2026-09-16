"""Legacy Dock parameter binding without a ROS context or motion publication."""

import ast
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from nav_tasks.api import ConfigError, declare_dock_parameters, dock_config_from_parameters


def parameters():
    """Return a complete non-David ROS parameter mapping for adapter tests.

    Returns:
        Mutable caller-owned settings, including the string-valued footprint.
    """
    return dict(
        approach_dist=0.5, xy_ok=0.1, yaw_ok=0.4363, max_yaw_retries=0,
        goal_topic='/custom_dock', action_name='custom_dock_action',
        robot_base_frame='navigation_center', global_frame='world',
        nav_timeout_sec=120.0, spin_timeout_sec=30.0, static_map_topic='/static_grid',
        static_occupied_threshold=50, footprint='[[-.2,-.2],[.2,-.2],[.2,.2],[-.2,.2]]',
        settle_max_rate_rad_s=0.05, settle_window_sec=0.4, settle_timeout_sec=8.0)


class _Node:
    def __init__(self, overrides=None):
        self.overrides = overrides or {}
        self.values = {}
        self.declarations = []

    def declare_parameter(self, name, value):
        self.declarations.append((name, value))
        self.values[name] = self.overrides.get(name, value)

    def get_parameter(self, name):
        return NS(value=self.values[name])


def test_parameter_names_equal_old_node_and_keep_distinct_wiring():
    """Every old declaration is represented, without substituting POI capture frames."""
    root = Path(__file__).resolve().parents[2]
    path = root / 'david_nav/test/golden/legacy_entries/dock_navigator.py.txt'
    source = ast.parse(path.read_text())
    old = {node.args[0].value for node in ast.walk(source)
           if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
           and node.func.attr == 'declare_parameter'}
    raw = parameters()
    assert old == set(raw)
    value = dock_config_from_parameters(raw)
    assert value.goal_topic == '/custom_dock'
    assert value.action_name == 'custom_dock_action'
    assert value.runtime.robot_base_frame == 'navigation_center'
    assert value.runtime.static_map_topic == '/static_grid'
    assert value.runtime.global_frame == 'world'
    assert value.runtime.spin_action_name == 'spin'
    assert value.runtime.pose_timeout_sec == 2.0
    assert value.runtime.yaw_ok == 0.4363 and value.runtime.max_yaw_retries == 0
    assert value.runtime.settle_max_rate_rad_s == 0.05
    raw['yaw_ok'] = 99
    assert value.runtime.yaw_ok == 0.4363
    with pytest.raises(FrozenInstanceError):
        value.goal_topic = '/changed'


def test_declares_original_defaults_then_uses_ros_overrides():
    """Existing launch/ROS CLI values take effect, including the old footprint string."""
    raw = parameters()
    node = _Node(dict(yaw_ok=0.2, max_yaw_retries=3, robot_base_frame='other_center',
                      goal_topic='/goal_pose', action_name='/other_action'))
    value = declare_dock_parameters(node, raw)
    assert dict(node.declarations) == raw
    assert value.runtime.yaw_ok == 0.2 and value.runtime.max_yaw_retries == 3
    assert value.runtime.robot_base_frame == 'other_center'
    assert value.goal_topic == '/goal_pose'  # R8 changes the default, not explicit overrides.
    assert value.action_name == '/other_action'
    assert node.values['footprint'] == raw['footprint']
    assert value.runtime.footprint[0] == (-0.2, -0.2)


def test_optional_backend_wiring_is_explicit_not_an_extra_legacy_parameter():
    """Spin remapping and TF budget can be supplied without growing the ROS parameter list."""
    node = _Node()
    value = declare_dock_parameters(node, parameters(), spin_action_name='/custom_spin',
                                    pose_timeout_sec=0.3)
    assert value.runtime.spin_action_name == '/custom_spin'
    assert value.runtime.pose_timeout_sec == 0.3
    assert len(node.declarations) == 16


@pytest.mark.parametrize('name,value', [
    ('goal_topic', ''), ('action_name', None), ('robot_base_frame', True),
    ('global_frame', ''), ('static_map_topic', ' '), ('approach_dist', -1),
    ('xy_ok', float('nan')), ('yaw_ok', float('inf')), ('max_yaw_retries', 1.5),
    ('static_occupied_threshold', 101), ('static_occupied_threshold', True),
    ('footprint', [[0, 0], [1, 0], [0, 1]]), ('footprint', '[[0,0],[1,1]]'),
    ('nav_timeout_sec', -1), ('spin_timeout_sec', False), ('settle_window_sec', -1),
    ('settle_timeout_sec', None), ('settle_max_rate_rad_s', -1),
])
def test_invalid_defaults_fail_before_parameter_declaration(name, value):
    """Invalid geometry, policy or endpoints cannot open a partially configured ingress."""
    raw, node = parameters(), _Node()
    raw[name] = value
    with pytest.raises(ConfigError):
        declare_dock_parameters(node, raw)
    assert not node.declarations


@pytest.mark.parametrize('name', list(parameters()))
def test_missing_parameter_is_not_replaced_by_helper_defaults(name):
    """All sixteen legacy values must come from explicit defaults or rendered mode data."""
    raw = parameters()
    del raw[name]
    with pytest.raises(ConfigError, match=name):
        dock_config_from_parameters(raw)


def test_unknown_parameter_and_invalid_ros_override_are_rejected():
    """Misspellings are not ignored and overrides are validated after declaration."""
    with pytest.raises(ConfigError, match='yaw_tolerance'):
        dock_config_from_parameters(dict(parameters(), yaw_tolerance=0.2))
    node = _Node({'settle_max_rate_rad_s': float('nan')})
    with pytest.raises(ConfigError):
        declare_dock_parameters(node, parameters())
    assert len(node.declarations) == 16


@pytest.mark.parametrize('options', [dict(spin_action_name=''), dict(pose_timeout_sec=-1)])
def test_bad_backend_options_fail_without_ros(options):
    """Explicit optional client/TF wiring is validated by the same runtime policy check."""
    with pytest.raises(ConfigError):
        dock_config_from_parameters(parameters(), **options)


@pytest.mark.parametrize('value', [None, [], 'not a mapping'])
def test_invalid_mapping_has_config_error_before_declaration(value):
    """The public declaration API has the same malformed-input contract as the pure binder."""
    node = _Node()
    with pytest.raises(ConfigError):
        declare_dock_parameters(node, value)
    assert not node.declarations
