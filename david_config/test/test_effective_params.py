"""Rendered parameters match the effective P0.7 Nav2 parameter golden files."""

from pathlib import Path

import pytest
import yaml

from nav_bringup.navcfg import render

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_DIR.parent
PROFILE = PACKAGE_DIR / 'robot' / 'david.yaml'
MODES_DIR = PACKAGE_DIR / 'modes'
NAV2_CONFIG_DIR = SOURCE_DIR / 'nav_bringup' / 'config'
GOLDEN_DIR = SOURCE_DIR / 'david_nav' / 'test' / 'golden' / 'params'

NODES = (
    'behavior_server',
    'bt_navigator',
    'controller_server',
    'global_costmap__global_costmap',
    'local_costmap__local_costmap',
    'planner_server',
    'robot_state_publisher',
    'velocity_smoother',
)

EXPECTED_SHARED_COUNTS = {
    'behavior_server': 17,
    'bt_navigator': 13,
    'controller_server': {'tour': 48, 'precise': 50},
    'global_costmap__global_costmap': 32,
    'local_costmap__local_costmap': 34,
    'planner_server': 6,
    'robot_state_publisher': 1,
    'velocity_smoother': 12,
}

EXPECTED_UNDECLARED = {
    'behavior_server': set(),
    'bt_navigator': set(),
    'controller_server': {
        'common': {
            'FollowPath/BaseObstacle.scale',
            'FollowPath/GoalAlign.forward_point_distance',
            'FollowPath/GoalAlign.scale',
            'FollowPath/GoalDist.scale',
            'FollowPath/PathAlign.forward_point_distance',
            'FollowPath/PathAlign.scale',
            'FollowPath/PathDist.scale',
            'FollowPath/min_vel_theta',
            'FollowPath/stateful',
            'global_frame',
            'precise_goal_checker/plugin',
            'precise_goal_checker/stateful',
            'precise_goal_checker/xy_goal_tolerance',
            'precise_goal_checker/yaw_goal_tolerance',
        },
        'tour_only': {
            'FollowPath/trans_stopped_velocity',
            'FollowPath/xy_goal_tolerance',
        },
    },
    'global_costmap__global_costmap': {
        'static_layer/lethal_cost_threshold',
        'static_layer/trinary_costmap',
        'static_layer/unknown_cost_value',
    },
    'local_costmap__local_costmap': {
        'obstacle_layer/enabled',
        'obstacle_layer/observation_sources',
        'obstacle_layer/plugin',
        'obstacle_layer/scan/clearing',
        'obstacle_layer/scan/data_type',
        'obstacle_layer/scan/inf_is_valid',
        'obstacle_layer/scan/marking',
        'obstacle_layer/scan/max_obstacle_height',
        'obstacle_layer/scan/obstacle_max_range',
        'obstacle_layer/scan/obstacle_min_range',
        'obstacle_layer/scan/raytrace_max_range',
        'obstacle_layer/scan/raytrace_min_range',
        'obstacle_layer/scan/topic',
        'static_layer/lethal_cost_threshold',
        'static_layer/trinary_costmap',
        'static_layer/unknown_cost_value',
    },
    'planner_server': {
        'GridBased/analytic_expansion_max_length',
        'GridBased/analytic_expansion_ratio',
        'GridBased/angle_quantization_bins',
        'GridBased/cache_obstacle_heuristic',
        'GridBased/change_penalty',
        'GridBased/cost_penalty',
        'GridBased/cost_travel_factor',
        'GridBased/downsample_costmap',
        'GridBased/downsampling_factor',
        'GridBased/lookup_table_size',
        'GridBased/max_iterations',
        'GridBased/max_planning_time',
        'GridBased/minimum_turning_radius',
        'GridBased/motion_model_for_search',
        'GridBased/non_straight_penalty',
        'GridBased/retrospective_penalty',
        'GridBased/reverse_penalty',
        'GridBased/smooth_path',
        'GridBased/smoother/do_refinement',
        'GridBased/smoother/max_iterations',
        'GridBased/smoother/tolerance',
        'GridBased/smoother/w_data',
        'GridBased/smoother/w_smooth',
    },
    'robot_state_publisher': set(),
    'velocity_smoother': set(),
}

BT_PATH_PARAMETERS = {
    'default_bt_xml_filename',
    'default_nav_through_poses_bt_xml',
    'default_nav_to_pose_bt_xml',
}


def _load(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def _flatten(value, prefix=''):
    if not isinstance(value, dict):
        return {prefix: value}
    result = {}
    for key, child in value.items():
        child_prefix = f'{prefix}/{key}' if prefix else str(key)
        result.update(_flatten(child, child_prefix))
    return result


def _rendered_node(tree, node):
    if node == 'local_costmap__local_costmap':
        return tree['local_costmap']['local_costmap']['ros__parameters']
    if node == 'global_costmap__global_costmap':
        return tree['global_costmap']['global_costmap']['ros__parameters']
    return tree[node]['ros__parameters']


def _golden_node(mode, node):
    document = _load(GOLDEN_DIR / mode / f'{node}.yaml')
    return next(iter(document.values()))['ros__parameters']


def _normalized(path, value):
    if path == 'footprint' and isinstance(value, str):
        return yaml.safe_load(value)
    if path in BT_PATH_PARAMETERS:
        return Path(value).name
    return value


def _expected_undeclared(mode, node):
    expected = EXPECTED_UNDECLARED[node]
    if node != 'controller_server':
        return expected
    return expected['common'] | (expected['tour_only'] if mode == 'tour' else set())


@pytest.mark.parametrize('mode', ('tour', 'precise'))
@pytest.mark.parametrize('node', NODES)
def test_rendered_parameters_equal_effective_golden(mode, node):
    """Every declared rendered key equals the P0.7 effective value.

    Nav2 parameter dumps contain only declared parameters and add defaults.
    The exact set of rendered-but-undeclared keys is frozen here so a typo or
    plugin change cannot silently reduce the compared surface. Footprints are
    compared as polygons, and launch-resolved BT paths by file name.
    """
    tree = render.render_files(
        NAV2_CONFIG_DIR / 'nav2_base.yaml',
        NAV2_CONFIG_DIR / 'nav2_bindings.yaml',
        PROFILE,
        MODES_DIR / f'{mode}.yaml',
    )
    rendered = _flatten(_rendered_node(tree, node))
    effective = _flatten(_golden_node(mode, node))
    shared = sorted(rendered.keys() & effective.keys())
    expected_count = EXPECTED_SHARED_COUNTS[node]
    if isinstance(expected_count, dict):
        expected_count = expected_count[mode]

    assert len(shared) == expected_count
    assert rendered.keys() - effective.keys() == _expected_undeclared(mode, node)
    assert {
        path: (_normalized(path, rendered[path]), _normalized(path, effective[path]))
        for path in shared
        if _normalized(path, rendered[path]) != _normalized(path, effective[path])
    } == {}
