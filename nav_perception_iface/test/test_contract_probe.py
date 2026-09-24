"""Read-only sampling on injected ROS boundaries, without initializing a context."""

from types import SimpleNamespace

import pytest

from nav_perception_iface import contract_probe
from test_contracts import _profile


def test_constructor_is_pure_and_sampling_waits_for_localization(monkeypatch):
    """Construction is side-effect free and sampling requires localization."""
    monkeypatch.setattr(contract_probe.rclpy, 'init', lambda **_: pytest.fail('ROS init'))
    probe = contract_probe.ContractProbe(_profile())
    probe.node = SimpleNamespace(create_subscription=lambda *_: pytest.fail('subscribed'))
    monkeypatch.setattr(probe, 'localized', lambda: False)
    with pytest.raises(ValueError, match='localization'):
        probe.sample(1.0, lambda: False)


def test_command_topic_is_never_subscribed():
    """The probe observes command ownership without consuming commands."""
    probe = contract_probe.ContractProbe(_profile())
    assert probe.data_topics == {'/loc_odom': 'nav_msgs/msg/Odometry',
                                 '/wheel_odom': 'nav_msgs/msg/Odometry'}
    assert '/drive_cmd' not in probe.data_topics


def test_chassis_off_omits_only_chassis_data():
    """Disabling chassis checks retains the localization observation."""
    probe = contract_probe.ContractProbe(_profile(), chassis_required=False)
    assert probe.data_topics == {'/loc_odom': 'nav_msgs/msg/Odometry'}


@pytest.mark.parametrize('topic', ['/loc_odom', 'loc_odom'])
def test_command_alias_cannot_be_satisfied_by_observer(topic):
    """A command topic cannot alias an observed odometry topic."""
    profile = _profile()
    profile['chassis']['cmd_topic'] = topic
    with pytest.raises(ValueError, match='command'):
        contract_probe.ContractProbe(profile)


@pytest.mark.parametrize('topic', ['~odom', '{node}/odom'])
def test_private_or_substituted_topics_are_invalid(topic):
    """Runtime-dependent topic names are rejected before creating ROS handles."""
    profile = _profile()
    profile['localization']['odom_topic'] = topic
    with pytest.raises(ValueError, match='non-private'):
        contract_probe.ContractProbe(profile)


@pytest.mark.parametrize('duration', [0, -1, float('nan'), float('inf'), True])
def test_invalid_sampling_duration_fails_before_ros(duration):
    """Sampling accepts only a positive finite duration."""
    with pytest.raises(ValueError):
        contract_probe.ContractProbe(_profile()).sample(duration, lambda: False)


def test_sample_is_fresh_bounded_and_releases_handles(monkeypatch):
    """Each bounded sample uses fresh messages and releases subscriptions."""
    probe = contract_probe.ContractProbe(_profile())
    now, callbacks, destroyed = [0.0], {}, []
    emitting = [True]
    subscriber_type = ['geometry_msgs/msg/Twist']

    def subscribe(message_type, topic, callback, qos):
        assert message_type is contract_probe.Odometry
        assert qos.depth == 1
        callbacks[topic] = callback
        return topic

    def spin(duration):
        now[0] += duration
        if emitting[0]:
            for callback in callbacks.values():
                callback(SimpleNamespace())

    probe.buffer = SimpleNamespace(can_transform=lambda *_: True)
    probe.node = SimpleNamespace(
        create_subscription=subscribe, destroy_subscription=destroyed.append,
        count_publishers=lambda _: 1,
        get_subscriptions_info_by_topic=lambda _: [SimpleNamespace(topic_type=subscriber_type[0])],
        get_publishers_info_by_topic=lambda topic: [SimpleNamespace(
            topic_type=probe.data_topics[topic])])
    monkeypatch.setattr(probe, 'spin', spin)
    monkeypatch.setattr(contract_probe.time, 'monotonic', lambda: now[0])
    snapshot = probe.sample(0.2, lambda: False)
    assert snapshot.samples == frozenset(probe.data_topics)
    assert sorted(destroyed) == sorted(probe.data_topics)
    assert snapshot.subscribers == {'/drive_cmd': 1}
    emitting[0] = False
    subscriber_type[0] = 'nav_msgs/msg/Odometry'
    snapshot = probe.sample(0.2, lambda: False)
    assert snapshot.samples == frozenset()
    assert snapshot.subscribers == {'/drive_cmd': 0}
    assert len(destroyed) == 4


def test_partial_subscription_failure_releases_created_handles(monkeypatch):
    """A partial setup failure releases every subscription already created."""
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
