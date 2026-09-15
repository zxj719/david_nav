"""假接口快照验证定位就绪顺序和底盘/点云契约."""

from dataclasses import replace

import pytest

from nav_perception_iface.contracts import check_contracts, InterfaceSnapshot


def _profile(scan=False):
    return {
        'frames': {'map': 'world', 'odom': 'wheel_world', 'body': 'body_axis',
                   'nav_base': 'rotation_center', 'scan_target': 'scan_axis'},
        'localization': {'odom_topic': '/loc_odom'},
        'nav2': {'odom_topic': '/legacy_odom'},
        'chassis': {'cmd_topic': '/drive_cmd', 'odom_topic': '/wheel_odom'},
        'sensors': {'scan_from_cloud': {'enabled': scan, 'cloud_topic': '/cloud'}},
    }


def _ready():
    topics = ['/loc_odom', '/wheel_odom', '/cloud']
    return InterfaceSnapshot(
        frozenset({('world', 'wheel_world'), ('wheel_world', 'body_axis'),
                   ('body_axis', 'rotation_center'), ('scan_axis', 'sensor_axis')}),
        {topic: 1 for topic in topics}, {'/drive_cmd': 1}, frozenset(topics),
        {'/loc_odom': frozenset({'nav_msgs/msg/Odometry'}),
         '/wheel_odom': frozenset({'nav_msgs/msg/Odometry'}),
         '/cloud': frozenset({'sensor_msgs/msg/PointCloud2'})}, 'sensor_axis')


def test_waiting_for_manual_localization_defers_odom_checks():
    """Missing map-to-odom means waiting, not broken localization odometry."""
    snapshot = InterfaceSnapshot(frozenset(), {}, {}, frozenset(), {})
    findings = check_contracts(_profile(), snapshot, chassis_required=False)
    assert [(f.severity, f.code) for f in findings] == [('info', 'localization_waiting')]


def test_ready_contract_and_optional_scan_pass_without_using_nav2_legacy_odom():
    """Checks use actual localization odom while preserving Nav2's separate wiring."""
    profile = _profile(scan=True)
    assert check_contracts(profile, _ready()) == ()
    assert profile['nav2']['odom_topic'] == '/legacy_odom'


def test_odom_without_messages_fails_only_after_localization_is_ready():
    """A publisher endpoint alone does not prove a live data stream."""
    snapshot = replace(_ready(), samples=frozenset({'/wheel_odom'}))
    assert [f.code for f in check_contracts(_profile(), snapshot)] == ['localization_odom_missing']


def test_localization_transform_chain_is_checked():
    """A map-to-odom transform alone is insufficient for the full robot frame chain."""
    snapshot = replace(_ready(), transforms=frozenset({('world', 'wheel_world')}))
    assert [f.code for f in check_contracts(_profile(), snapshot)] == [
        'localization_tf_missing', 'localization_tf_missing']


def test_body_and_navigation_frame_may_coincide_on_another_robot():
    """An identity transform needs no separate TF publisher."""
    profile = _profile()
    profile['frames']['nav_base'] = profile['frames']['body']
    assert check_contracts(profile, _ready()) == ()


def test_chassis_contract_can_be_disabled_for_a_chassis_off_run():
    """No command subscriber or chassis data is required when the driver is excluded."""
    snapshot = replace(_ready(), subscribers={}, publishers={'/loc_odom': 1})
    assert check_contracts(_profile(), snapshot, chassis_required=False) == ()
    assert [f.code for f in check_contracts(_profile(), snapshot)] == [
        'chassis_cmd_missing', 'chassis_odom_missing']


def test_disabled_scan_does_not_require_any_cloud_data():
    """Optional preprocessing introduces no readiness requirement while disabled."""
    snapshot = replace(_ready(), publishers={'/loc_odom': 1, '/wheel_odom': 1}, cloud_frame=None)
    assert check_contracts(_profile(), snapshot) == ()
    assert [f.code for f in check_contracts(_profile(scan=True), snapshot)] == [
        'scan_cloud_missing', 'scan_cloud_tf_missing']


def test_missing_enable_defaults_false_and_wrong_type_fails():
    """The contract checker shares the conservative configuration-enable semantics."""
    profile = _profile()
    del profile['sensors']['scan_from_cloud']['enabled']
    assert check_contracts(profile, _ready()) == ()
    profile['sensors']['scan_from_cloud']['enabled'] = 'false'
    with pytest.raises(ValueError, match='boolean'):
        check_contracts(profile, _ready())


def test_cloud_message_type_and_target_transform_are_checked():
    """Wrong sensor message types and a disconnected sensor frame are rejected."""
    snapshot = _ready()
    types = dict(snapshot.types, **{'/cloud': frozenset({'sensor_msgs/msg/LaserScan'})})
    findings = check_contracts(_profile(scan=True), replace(
        snapshot, types=types, cloud_frame='disconnected_sensor'))
    assert [f.code for f in findings] == ['scan_cloud_missing', 'scan_cloud_tf_missing']
