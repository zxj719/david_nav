"""Tests for rendering parameter trees from a robot profile."""

import copy

import pytest
import yaml

from nav_bringup.navcfg import render

BASE = {
    'bt_navigator': {'ros__parameters': {
        'robot_base_frame': '<bind:frames.nav_base>',
        'bt_loop_duration': 50,
    }},
    'local_costmap': {'local_costmap': {'ros__parameters': {
        'robot_base_frame': '<bind:frames.nav_base>',
        'footprint': '<bind:footprint>',
        'plugins': ['static_layer', 'obstacle_layer'],
    }}},
    'controller_server': {'ros__parameters': {
        'FollowPath': {
            'max_vel_x': '<bind:limits.max_vel_x>',
            'BaseObstacle.scale': 0.2,
            'critics': ['Oscillation', 'GoalDist'],
        },
    }},
    'velocity_smoother': {'ros__parameters': {
        'max_velocity': ['<bind:limits.max_vel_x>', 0.0, 0.5],
    }},
}
BINDING_TABLE = {
    'schema_version': 1,
    'bindings': {
        'frames.nav_base': [
            'bt_navigator/ros__parameters/robot_base_frame',
            'local_costmap/local_costmap/ros__parameters/robot_base_frame',
        ],
        'footprint': {
            'format': 'footprint_string',
            'paths': ['local_costmap/local_costmap/ros__parameters/footprint'],
        },
        'limits.max_vel_x': [
            'controller_server/ros__parameters/FollowPath/max_vel_x',
            'velocity_smoother/ros__parameters/max_velocity/0',
        ],
    },
}
PROFILE = {
    'frames': {'nav_base': 'robot_base'},
    'footprint': [[0.25, 0.3], [0.25, -0.3], [-0.45, -0.3], [-0.45, 0.3]],
    'limits': {'max_vel_x': 0.15},
}


def _bindings(table=None):
    return render.parse_bindings(table or BINDING_TABLE, 'bindings.yaml')


def _mode(**fields):
    data = {'schema_version': 1, 'mode': 'precise'}
    data.update(fields)
    return render.parse_mode(data, 'modes/precise.yaml')


def test_render_writes_every_bound_value():
    """Every marker is replaced with the formatted profile value; inputs stay untouched."""
    base = copy.deepcopy(BASE)

    tree = render.render(base, _bindings(), PROFILE)

    assert tree['bt_navigator']['ros__parameters']['robot_base_frame'] == 'robot_base'
    costmap = tree['local_costmap']['local_costmap']['ros__parameters']
    assert costmap['robot_base_frame'] == 'robot_base'
    assert yaml.safe_load(costmap['footprint']) == PROFILE['footprint']
    assert tree['controller_server']['ros__parameters']['FollowPath']['max_vel_x'] == 0.15
    assert tree['velocity_smoother']['ros__parameters']['max_velocity'] == [0.15, 0.0, 0.5]
    assert render.find_markers(tree) == {}
    assert base == BASE


def test_mode_overlay_changes_profile_values_and_overrides_tuning():
    """Mode profile values flow through bindings; overrides replace or add parameters."""
    mode = _mode(
        profile={'limits': {'max_vel_x': 0.1}},
        nav2_overrides={
            'controller_server/ros__parameters/FollowPath/critics':
                ['RotateToGoal', 'Oscillation', 'GoalDist'],
            'controller_server/ros__parameters/FollowPath/RotateToGoal.scale': 1.0,
        })

    tree = render.render(BASE, _bindings(), PROFILE, mode)

    follow_path = tree['controller_server']['ros__parameters']['FollowPath']
    assert follow_path['max_vel_x'] == 0.1
    assert follow_path['critics'] == ['RotateToGoal', 'Oscillation', 'GoalDist']
    assert follow_path['RotateToGoal.scale'] == 1.0
    assert tree['velocity_smoother']['ros__parameters']['max_velocity'][0] == 0.1


def test_markers_and_bindings_must_agree():
    """Unbound markers, bound paths without markers and key mismatches are rejected."""
    unbound = copy.deepcopy(BASE)
    unbound['bt_navigator']['ros__parameters']['global_frame'] = '<bind:frames.map>'
    missing_marker = copy.deepcopy(BASE)
    missing_marker['bt_navigator']['ros__parameters']['robot_base_frame'] = 'robot_base'
    wrong_key = copy.deepcopy(BASE)
    wrong_key['bt_navigator']['ros__parameters']['robot_base_frame'] = '<bind:frames.body>'

    with pytest.raises(render.RenderError, match='without a binding'):
        render.render(unbound, _bindings(), PROFILE)
    with pytest.raises(render.RenderError, match='do not hold their marker'):
        render.render(missing_marker, _bindings(), PROFILE)
    with pytest.raises(render.RenderError, match='frames.body'):
        render.render(wrong_key, _bindings(), PROFILE)


@pytest.mark.parametrize('path, message', [
    ('controller_server/ros__parameters/FollowPath/max_vel_x', 'covers bound parameters'),
    ('velocity_smoother/ros__parameters/max_velocity', 'covers bound parameters'),
    ('controller_server/ros__parameters/Missing/value', 'does not exist'),
])
def test_overrides_cannot_touch_bound_or_missing_parameters(path, message):
    """Overrides of bound parameters, or of parameters under missing parents, fail."""
    with pytest.raises(render.RenderError, match=message):
        render.render(BASE, _bindings(), PROFILE, _mode(nav2_overrides={path: 1.0}))


def test_profile_values_must_exist_and_have_the_right_shape():
    """Missing profile keys and malformed footprints name the key."""
    without_limits = {key: value for key, value in PROFILE.items() if key != 'limits'}
    bad_footprint = dict(PROFILE, footprint=[[0.1, 0.1], [0.2, 0.2]])

    with pytest.raises(render.RenderError, match='limits.max_vel_x'):
        render.render(BASE, _bindings(), without_limits)
    with pytest.raises(render.RenderError, match='footprint: expected a polygon'):
        render.render(BASE, _bindings(), bad_footprint)


@pytest.mark.parametrize('table, message', [
    ({'schema_version': 2, 'bindings': {'a.b': ['x/y']}}, 'schema_version'),
    ({'schema_version': 1, 'bindings': {}}, 'non-empty mapping'),
    ({'schema_version': 1, 'bindings': {'Bad.Key': ['x/y']}}, 'invalid profile key'),
    ({'schema_version': 1, 'bindings': {'a.b': []}}, 'non-empty list'),
    ({'schema_version': 1, 'bindings': {'a.b': {'paths': ['x/y'], 'format': 'nope'}}},
     'format must be one of'),
    ({'schema_version': 1, 'bindings': {'a.b': {'paths': ['x/y'], 'unit': 'm'}}},
     'unknown fields'),
    ({'schema_version': 1, 'bindings': {'a.b': ['x//y']}}, 'invalid parameter path'),
    ({'schema_version': 1, 'bindings': {'a.b': ['x/y'], 'a.c': ['x/y']}}, 'bound by both'),
])
def test_malformed_binding_tables_are_rejected(table, message):
    """Every format violation of a binding table raises RenderError."""
    with pytest.raises(render.RenderError, match=message):
        render.parse_bindings(table, 'bindings.yaml')


@pytest.mark.parametrize('data, message', [
    ({'schema_version': 1}, 'non-empty name'),
    ({'schema_version': 1, 'mode': 'tour', 'extra': 1}, 'unknown fields'),
    ({'schema_version': 1, 'mode': 'tour', 'profile': []}, 'profile must be a mapping'),
    ({'schema_version': 1, 'mode': 'tour', 'nav2_overrides': {'a//b': 1}},
     'invalid parameter path'),
])
def test_malformed_mode_overlays_are_rejected(data, message):
    """Every format violation of a mode overlay raises RenderError."""
    with pytest.raises(render.RenderError, match=message):
        render.parse_mode(data, 'modes/tour.yaml')


def test_paths_handle_dotted_names_and_list_indices():
    """Parameter names may contain dots; integer segments index lists."""
    assert render.get_path(
        BASE, 'controller_server/ros__parameters/FollowPath/BaseObstacle.scale') == 0.2
    assert render.get_path(
        BASE, 'local_costmap/local_costmap/ros__parameters/plugins/1') == 'obstacle_layer'
    with pytest.raises(render.RenderError, match='is not an index'):
        render.get_path(BASE, 'local_costmap/local_costmap/ros__parameters/plugins/5')
    with pytest.raises(render.RenderError, match='does not exist'):
        render.set_path(copy.deepcopy(BASE), 'bt_navigator/ros__parameters/new_value', 1)


def test_malformed_markers_and_slashed_names_are_rejected():
    """Typos in markers and parameter names containing / are caught."""
    with pytest.raises(render.RenderError, match='malformed binding marker'):
        render.find_markers({'node': {'value': '<bind:Frames.Base>'}})
    with pytest.raises(render.RenderError, match='contains'):
        render.find_markers({'node': {'a/b': 1}})


def test_deep_merge_replaces_lists_and_keeps_inputs():
    """Mappings merge recursively; lists and scalars are replaced."""
    base = {'a': {'b': 1, 'c': [1, 2]}, 'd': 1}
    overlay = {'a': {'c': [3]}, 'e': {'f': 2}}

    merged = render.deep_merge(base, overlay)

    assert merged == {'a': {'b': 1, 'c': [3]}, 'd': 1, 'e': {'f': 2}}
    assert base == {'a': {'b': 1, 'c': [1, 2]}, 'd': 1}


def test_render_files_round_trip(tmp_path):
    """Files are read, rendered and written back as equivalent YAML."""
    paths = {}
    for name, content in (
            ('base', BASE), ('bindings', BINDING_TABLE), ('profile', PROFILE),
            ('mode', {'schema_version': 1, 'mode': 'tour'})):
        paths[name] = tmp_path / f'{name}.yaml'
        paths[name].write_text(yaml.safe_dump(content), encoding='utf-8')
    output = tmp_path / 'rendered.yaml'

    tree = render.render_files(
        paths['base'], paths['bindings'], paths['profile'], paths['mode'])
    render.write_parameters(output, tree)

    assert yaml.safe_load(output.read_text(encoding='utf-8')) == tree
    with pytest.raises(render.RenderError, match='cannot read'):
        render.load_yaml(tmp_path / 'missing.yaml')
