"""Tests for cross-file navigation configuration validation."""

import yaml

from nav_bringup.navcfg import cli
from nav_bringup.navcfg import render
from nav_bringup.navcfg import schema
from nav_bringup.navcfg import validation

URDF = '''
<robot name="demo">
  <link name="base_link"/>
  <link name="base_footprint"/>
  <joint name="body_to_nav" type="fixed">
    <parent link="base_link"/>
    <child link="base_footprint"/>
    <origin xyz="0.165 0 -0.4075" rpy="0 0 0"/>
  </joint>
</robot>
'''


def _profile():
    return {
        'kinematics': {'type': 'diff_drive'},
        'frames': {'poi_capture': 'base_link', 'nav_base': 'base_footprint'},
        'limits': {'max_vel_x': 0.15, 'max_vel_theta': 0.5},
        'goal': {'yaw_tolerance': 0.4},
        'chassis': {'adapter': {
            'hard_max_linear_x': 0.2,
            'hard_max_angular_z': 0.6,
            'min_stationary_angular_z': 0.55,
        }},
        'teleop': {'max_linear_x': 0.2, 'max_angular_z': 0.5},
    }


def _tree():
    return {
        'behavior_server': {'ros__parameters': {'global_frame': 'odom'}},
        'local_costmap': {'local_costmap': {'ros__parameters': {'global_frame': 'odom'}}},
        'controller_server': {'ros__parameters': {
            'general_goal_checker': {'xy_goal_tolerance': 0.1},
            'FollowPath': {
                'xy_goal_tolerance': 0.1,
                'critics': ['RotateToGoal', 'GoalDist'],
            },
        }},
    }


def _mode():
    return render.parse_mode({'schema_version': 1, 'mode': 'precise'}, 'mode.yaml')


def _catalog():
    return schema.ParameterCatalog([])


def _issues(profile=None, tree=None, map_data=None, behavior_trees=None, urdf=URDF):
    return validation.validate_configuration(
        _catalog(),
        profile or _profile(),
        _mode(),
        tree or _tree(),
        map_data or {'mode': 'trinary'},
        behavior_trees or {'tree.xml': '<root><Escape lateral_speed="0.20"/></root>'},
        urdf,
    )


def test_valid_configuration_only_warns_about_known_david_geometry():
    """David's lateral BT escape and POI frame offset warn without rejecting it."""
    issues = _issues()

    assert not [issue for issue in issues if issue.severity == 'error']
    assert {issue.rule for issue in issues if issue.severity == 'warn'} >= {
        'diff_drive_bt_lateral', 'poi_capture_offset'}
    offset = next(issue for issue in issues if issue.rule == 'poi_capture_offset')
    assert 'xyz=[0.1650, 0.0000, -0.4075]' in offset.message


def test_frame_and_navigation_limit_violations_are_errors():
    """Recovery-frame and all three hard-limit relationships reject bad values."""
    profile = _profile()
    profile['limits'] = {'max_vel_x': 0.25, 'max_vel_theta': 0.7}
    profile['chassis']['adapter']['min_stationary_angular_z'] = 0.65
    tree = _tree()
    tree['behavior_server']['ros__parameters']['global_frame'] = 'map'

    errors = [issue for issue in _issues(profile=profile, tree=tree)
              if issue.severity == 'error']

    assert [issue.rule for issue in errors].count('navigation_hard_limits') == 3
    assert any(issue.rule == 'behavior_local_global_frame' for issue in errors)


def test_precise_mode_requires_equal_xy_tolerances_and_rotate_critic():
    """A constrained heading requires consistent position and heading control."""
    tree = _tree()
    tree['controller_server']['ros__parameters']['FollowPath'] = {
        'xy_goal_tolerance': 0.2,
        'critics': ['GoalDist'],
    }

    errors = [issue for issue in _issues(tree=tree) if issue.rule == 'precise_controller']

    assert len(errors) == 2
    assert any('values differ' in issue.message for issue in errors)
    assert any('omit RotateToGoal' in issue.message for issue in errors)


def test_map_and_teleop_limits_reject_unsafe_values():
    """Scale-mode maps and teleop gears above adapter limits are errors."""
    profile = _profile()
    profile['teleop'] = {'max_linear_x': 0.25, 'max_angular_z': 0.7}

    errors = [issue for issue in _issues(profile=profile, map_data={'mode': 'scale'})
              if issue.severity == 'error']

    assert [issue.rule for issue in errors].count('teleop_hard_limits') == 2
    assert any(issue.rule == 'map_trinary' for issue in errors)


def test_malformed_relationship_values_become_errors_instead_of_crashing():
    """Rule inputs not yet covered by a package schema still fail cleanly."""
    profile = _profile()
    profile['chassis']['adapter']['hard_max_linear_x'] = 'fast'
    profile['goal']['yaw_tolerance'] = 'strict'

    errors = [issue for issue in _issues(profile=profile) if issue.severity == 'error']

    assert any('must be numeric' in issue.message for issue in errors)
    assert {issue.rule for issue in errors} >= {
        'navigation_hard_limits', 'precise_controller', 'teleop_hard_limits'}


def test_lateral_speed_warning_only_applies_to_diff_drive():
    """A nonzero lateral BT parameter warns for diff-drive but not other kinematics."""
    diff_issues = _issues()
    holonomic = _profile()
    holonomic['kinematics']['type'] = 'holonomic'
    holonomic_issues = _issues(profile=holonomic)

    assert any(issue.rule == 'diff_drive_bt_lateral' for issue in diff_issues)
    assert not any(issue.rule == 'diff_drive_bt_lateral' for issue in holonomic_issues)


def test_frame_offset_composes_and_inverts_fixed_joints():
    """URDF offsets work in both directions and through a fixed-joint chain."""
    urdf = '''<robot name="chain">
      <joint name="a_b" type="fixed"><parent link="a"/><child link="b"/>
        <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/></joint>
      <joint name="b_c" type="fixed"><parent link="b"/><child link="c"/>
        <origin xyz="1 0 0" rpy="0 0 0"/></joint>
    </robot>'''

    forward = validation.frame_offset(urdf, 'a', 'c')
    reverse = validation.frame_offset(urdf, 'c', 'a')

    assert forward.xyz == (1.0, 1.0, 0.0)
    assert all(abs(actual - expected) < 1e-12 for actual, expected in zip(
        reverse.xyz, (-1.0, 1.0, 0.0)))


def _write_cli_fixture(tmp_path):
    """Write a complete small navcfg validate input set."""
    base = _tree()
    base['behavior_server']['ros__parameters']['global_frame'] = '<bind:frames.odom>'
    base['local_costmap']['local_costmap']['ros__parameters']['global_frame'] = (
        '<bind:frames.odom>')
    base['bt_navigator'] = {'ros__parameters': {
        'default_bt_xml_filename': 'tree.xml',
        'default_nav_to_pose_bt_xml': 'tree.xml',
        'default_nav_through_poses_bt_xml': 'tree.xml',
    }}
    files = {
        'base.yaml': base,
        'bindings.yaml': {
            'schema_version': 1,
            'bindings': {'frames.odom': [
                'behavior_server/ros__parameters/global_frame',
                'local_costmap/local_costmap/ros__parameters/global_frame',
            ]},
        },
        'profile.yaml': dict(_profile(), frames={
            'odom': 'odom', 'poi_capture': 'base_footprint', 'nav_base': 'base_footprint'}),
        'mode.yaml': {'schema_version': 1, 'mode': 'precise'},
        'schema.yaml': {'schema_version': 1, 'package': 'demo', 'parameters': {}},
        'map.yaml': {'mode': 'trinary'},
        'site.yaml': {'map': {'nav_yaml': 'map.yaml'}},
    }
    for name, data in files.items():
        (tmp_path / name).write_text(yaml.safe_dump(data), encoding='utf-8')
    (tmp_path / 'robot.urdf').write_text('<robot name="demo"/>', encoding='utf-8')
    (tmp_path / 'tree.xml').write_text(
        '<root><Escape lateral_speed="0.0"/></root>', encoding='utf-8')


def _cli_args(tmp_path):
    return [
        'validate', '--profile', str(tmp_path / 'profile.yaml'),
        '--mode', str(tmp_path / 'mode.yaml'), '--site', str(tmp_path / 'site.yaml'),
        '--urdf', str(tmp_path / 'robot.urdf'), '--bt-dir', str(tmp_path),
        '--base', str(tmp_path / 'base.yaml'),
        '--bindings', str(tmp_path / 'bindings.yaml'),
        '--schema', str(tmp_path / 'schema.yaml'),
    ]


def test_validate_command_exit_codes(tmp_path, capsys):
    """The CLI returns zero for warnings and one when a validation rule fails."""
    _write_cli_fixture(tmp_path)

    assert cli.main(_cli_args(tmp_path)) == 0
    assert '0 error(s)' in capsys.readouterr().out

    (tmp_path / 'map.yaml').write_text('mode: scale\n', encoding='utf-8')
    assert cli.main(_cli_args(tmp_path)) == 1
    output = capsys.readouterr().out
    assert 'ERROR [map_trinary]' in output
    assert '1 error(s)' in output
