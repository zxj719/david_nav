"""Read-only sampling on injected ROS boundaries, without initializing a context."""

from types import SimpleNamespace

from builtin_interfaces.msg import Time as TimeMessage
import pytest

from nav_perception_iface import contract_probe
from test_contracts import _profile


def test_constructor_is_pure_and_sampling_waits_for_localization(monkeypatch):
    """No odometry subscription is created while manual localization is pending."""
    monkeypatch.setattr(contract_probe.rclpy, 'init', lambda **_: pytest.fail('ROS init'))
    probe = contract_probe.ContractProbe(_profile())
    probe.node = SimpleNamespace(create_subscription=lambda *_: pytest.fail('subscribed'))
    monkeypatch.setattr(probe, 'localized', lambda: False)
    with pytest.raises(ValueError, match='localization'):
        probe.sample(1.0, lambda: False)


def test_disabled_scan_and_command_topics_are_never_subscribed():
    """Only data topics are observed, so the observer cannot satisfy driver subscription."""
    probe = contract_probe.ContractProbe(_profile())
    assert probe.data_topics == {'/loc_odom': 'nav_msgs/msg/Odometry',
                                 '/wheel_odom': 'nav_msgs/msg/Odometry'}
    assert '/drive_cmd' not in probe.data_topics


def test_chassis_off_omits_only_chassis_data_not_localization():
    """An explicit standalone diagnostic switch never disables localization checks."""
    probe = contract_probe.ContractProbe(_profile(scan=True), chassis_required=False)
    assert probe.data_topics == {'/loc_odom': 'nav_msgs/msg/Odometry',
                                 '/cloud': 'sensor_msgs/msg/PointCloud2'}


@pytest.mark.parametrize('topic', ['/loc_odom', 'loc_odom'])
def test_command_alias_cannot_be_satisfied_by_the_observers_own_subscription(topic):
    """A malformed command/data alias cannot turn the observer into a fake driver."""
    profile = _profile()
    profile['chassis']['cmd_topic'] = topic
    with pytest.raises(ValueError, match='command'):
        contract_probe.ContractProbe(profile)


def test_cloud_and_odometry_aliases_cannot_hide_a_type_collision():
    """Relative and absolute spellings resolve to the same root-namespace topic."""
    profile = _profile(scan=True)
    profile['sensors']['scan_from_cloud']['cloud_topic'] = 'loc_odom'
    with pytest.raises(ValueError, match='cloud'):
        contract_probe.ContractProbe(profile)


@pytest.mark.parametrize('topic', ['~odom', '{node}/odom'])
def test_private_or_substituted_topics_are_not_valid_provider_contracts(topic):
    """Provider contracts cannot depend on the observer's node-name expansion."""
    profile = _profile()
    profile['localization']['odom_topic'] = topic
    with pytest.raises(ValueError, match='non-private'):
        contract_probe.ContractProbe(profile)


@pytest.mark.parametrize('duration', [0, -1, float('nan'), float('inf'), True])
def test_invalid_sampling_duration_fails_before_ros(duration):
    """No invalid window can reach node methods or start an unbounded observation."""
    with pytest.raises(ValueError):
        contract_probe.ContractProbe(_profile()).sample(duration, lambda: False)


def test_sample_is_fresh_bounded_and_checks_cloud_at_message_stamp(monkeypatch):
    """Only current-window callbacks count; payloads are not retained and handles close."""
    probe = contract_probe.ContractProbe(_profile(scan=True))
    now, callbacks, destroyed, transforms = [0.0], {}, [], []
    emitting = [True]
    subscriber_type = ['geometry_msgs/msg/Twist']

    def subscribe(message_type, topic, callback, qos):
        assert qos.depth == 1
        assert qos.reliability == contract_probe.ReliabilityPolicy.BEST_EFFORT
        assert qos.durability == contract_probe.DurabilityPolicy.VOLATILE
        callbacks[topic] = callback
        return topic

    def spin(duration):
        now[0] += duration
        if emitting[0]:
            for callback in callbacks.values():
                callback(SimpleNamespace(header=SimpleNamespace(
                    frame_id='sensor_axis', stamp=TimeMessage(sec=12))))

    def transform(target, source, stamp):
        transforms.append((target, source, stamp.nanoseconds))
        return True

    probe.buffer = SimpleNamespace(can_transform=transform)
    probe.node = SimpleNamespace(
        create_subscription=subscribe, destroy_subscription=destroyed.append,
        count_publishers=lambda _: 1, count_subscribers=lambda _: 1,
        get_subscriptions_info_by_topic=lambda _: [SimpleNamespace(topic_type=subscriber_type[0])],
        get_publishers_info_by_topic=lambda topic: [SimpleNamespace(
            topic_type=probe.data_topics[topic])])
    monkeypatch.setattr(probe, 'spin', spin)
    monkeypatch.setattr(contract_probe.time, 'monotonic', lambda: now[0])
    snapshot = probe.sample(0.2, lambda: False)
    assert now[0] == 0.2
    assert snapshot.samples == frozenset(probe.data_topics)
    assert snapshot.cloud_frame == 'sensor_axis'
    assert ('scan_axis', 'sensor_axis', 12000000000) in transforms
    assert sorted(destroyed) == sorted(probe.data_topics)
    assert snapshot.subscribers == {'/drive_cmd': 1}
    emitting[0] = False
    subscriber_type[0] = 'nav_msgs/msg/Odometry'
    snapshot = probe.sample(0.2, lambda: False)
    assert snapshot.samples == frozenset()
    assert snapshot.cloud_frame is None
    assert snapshot.subscribers == {'/drive_cmd': 0}
    assert len(destroyed) == 6


def test_partial_subscription_failure_releases_created_handles(monkeypatch):
    """A later ROS creation error must not leak an earlier subscription."""
    probe = contract_probe.ContractProbe(_profile())
    created, destroyed = [], []

    def subscribe(_type, topic, *_):
        if created:
            raise RuntimeError('subscription unavailable')
        created.append(topic)
        return topic

    probe.node = SimpleNamespace(create_subscription=subscribe,
                                 destroy_subscription=destroyed.append)
    monkeypatch.setattr(probe, 'localized', lambda: True)
    with pytest.raises(RuntimeError, match='subscription unavailable'):
        probe.sample(1.0, lambda: False)
    assert destroyed == created == ['/loc_odom']
