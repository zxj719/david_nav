"""Fake snapshots verify localization and chassis interface contracts."""

from dataclasses import replace

from nav_perception_iface.contracts import check_contracts, InterfaceSnapshot


def _profile():
    return {
        'frames': {'map': 'world', 'odom': 'wheel_world', 'body': 'body_axis',
                   'nav_base': 'rotation_center'},
        'localization': {'odom_topic': '/loc_odom'},
        'nav2': {'odom_topic': '/nav_odom'},
        'chassis': {'cmd_topic': '/drive_cmd', 'odom_topic': '/wheel_odom'},
    }


def _ready():
    topics = ['/loc_odom', '/wheel_odom']
    return InterfaceSnapshot(
        frozenset({('world', 'wheel_world'), ('wheel_world', 'body_axis'),
                   ('body_axis', 'rotation_center')}),
        {topic: 1 for topic in topics}, {'/drive_cmd': 1}, frozenset(topics),
        {topic: frozenset({'nav_msgs/msg/Odometry'}) for topic in topics})


def test_waiting_for_manual_localization_defers_odom_checks():
    """Missing localization reports waiting before downstream data failures."""
    snapshot = InterfaceSnapshot(frozenset(), {}, {}, frozenset(), {})
    findings = check_contracts(_profile(), snapshot, chassis_required=False)
    assert [(f.severity, f.code) for f in findings] == [('info', 'localization_waiting')]


def test_ready_contract_uses_localization_odom_not_nav2_odom():
    """The interface contract observes the localization-owned odometry topic."""
    profile = _profile()
    assert check_contracts(profile, _ready()) == ()
    assert profile['nav2']['odom_topic'] == '/nav_odom'


def test_odom_without_messages_fails_only_after_localization_is_ready():
    """A silent localization topic fails once the transform chain is ready."""
    snapshot = replace(_ready(), samples=frozenset({'/wheel_odom'}))
    assert [f.code for f in check_contracts(_profile(), snapshot)] == [
        'localization_odom_missing']


def test_localization_transform_chain_is_checked():
    """Every required edge in the localization transform chain is checked."""
    snapshot = replace(_ready(), transforms=frozenset({('world', 'wheel_world')}))
    assert [f.code for f in check_contracts(_profile(), snapshot)] == [
        'localization_tf_missing', 'localization_tf_missing']


def test_body_and_navigation_frame_may_coincide():
    """Coincident body and navigation frames need no artificial transform."""
    profile = _profile()
    profile['frames']['nav_base'] = profile['frames']['body']
    assert check_contracts(profile, _ready()) == ()


def test_chassis_contract_can_be_disabled():
    """Chassis ownership findings disappear only when explicitly disabled."""
    snapshot = replace(_ready(), subscribers={}, publishers={'/loc_odom': 1})
    assert check_contracts(_profile(), snapshot, chassis_required=False) == ()
    assert [f.code for f in check_contracts(_profile(), snapshot)] == [
        'chassis_cmd_missing', 'chassis_odom_missing']
