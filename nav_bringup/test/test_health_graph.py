"""Navigation health must be present in the actual graph and frozen parameter bundle."""

import pytest
import yaml

from nav_bringup.launching.graph import build_graph, GraphOptions
from test_run_preparation import _prepare


@pytest.mark.parametrize('mode', ['tour', 'precise', 'bench_slamgt'])
def test_navigation_graph_owns_one_warn_only_health_observer(tmp_path, mode):
    """Health is noncritical observation, not a localization provider or a motion owner."""
    graph = build_graph(GraphOptions(mode, str(tmp_path / 'run')))
    nodes = [node for node in graph.processes if node.name == 'localization_monitor']
    assert len(nodes) == 1
    assert nodes[0].package == 'nav_perception_iface'
    assert nodes[0].executable == 'localization_monitor' and not nodes[0].critical
    bundle = _prepare(tmp_path, mode)
    tree = yaml.safe_load(dict(bundle.artifacts)['params/rendered/localization_health.yaml'])
    parameters = tree['localization_monitor']['ros__parameters']
    assert parameters['map_frame'] == 'demo_world' and parameters['nav_frame'] == 'demo_axle'
    assert parameters['odom_frame'] == 'demo_encoder'
    assert parameters['chassis_odom_frame'] == 'demo_encoder'
    assert parameters['chassis_odom_topic'] == '/demo/wheel_odom'
    assert parameters['action'] == 'warn'
    assert parameters['max_tf_age_sec'] == 0.5
    assert parameters['max_divergence_m'] == 1.0
    assert parameters['divergence_window_sec'] == 2.0


@pytest.mark.parametrize('mode', ['chassis', 'teleop'])
def test_non_navigation_modes_do_not_add_localization_dependency(tmp_path, mode):
    """Standalone chassis/teleop stay independent of positioning and its health observer."""
    graph = build_graph(GraphOptions(mode, str(tmp_path / 'run')))
    assert all(node.name != 'localization_monitor' for node in graph.processes)


@pytest.mark.parametrize('key,value', [
    ('max_tf_age_sec', 0), ('max_divergence_m', float('inf')),
    ('divergence_window_sec', True), ('action', 'stop'),
])
def test_bad_health_policy_is_rejected_during_pure_parameter_preparation(tmp_path, key, value):
    """A bad warning policy must not wait until after the rest of navigation has launched."""
    from nav_bringup.navrun.prepared_parameters import node_parameters

    bundle = _prepare(tmp_path)
    profile = yaml.safe_load(dict(bundle.artifacts)['params/inputs/robot.yaml'])
    profile['localization_health'][key] = value
    with pytest.raises(ValueError, match='localization_health'):
        node_parameters(bundle.options, profile, {'nav2': {}})


def test_david_wheel_frame_is_bound_without_changing_localization_or_driver(tmp_path):
    """The navigation observer must not conflate raw wheel integration and SLAM odometry."""
    from pathlib import Path
    from nav_bringup.navrun.prepared_parameters import node_parameters

    source = Path(__file__).resolve().parents[2]
    profile = yaml.safe_load((source / 'david_config/robot/david.yaml').read_text())
    options = GraphOptions('tour', str(tmp_path), start_chassis=False,
                           robot_state=False, map_server=False)
    tree = node_parameters(options, profile, {'nav2': {}})
    health = tree['localization_health']['localization_monitor']['ros__parameters']
    driver = profile['chassis']['driver']['parameters']
    assert health['chassis_odom_frame'] == driver['odom_frame_id']
    assert health['odom_frame'] == profile['frames']['odom']
    assert health['chassis_odom_frame'] != health['odom_frame']


def test_missing_wheel_parent_name_is_rejected_before_startup(tmp_path):
    """An empty interface name must not become a late noncritical node failure."""
    from nav_bringup.navrun.prepared_parameters import node_parameters

    bundle = _prepare(tmp_path)
    profile = yaml.safe_load(dict(bundle.artifacts)['params/inputs/robot.yaml'])
    profile['chassis']['odom_frame'] = ''
    with pytest.raises(ValueError, match='localization_health'):
        node_parameters(bundle.options, profile, {'nav2': {}})
