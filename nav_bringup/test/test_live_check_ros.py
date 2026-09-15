"""Read-only observer methods tested on service/TF fakes without initializing ROS."""

from types import SimpleNamespace

import pytest

from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.parameter_service import ParameterService

from nav_bringup.navcfg import live_ros
from nav_bringup.navcfg.live_check import LiveSpec, TopicRule


def test_snapshot_uses_only_read_services_and_excludes_undeclared_values(monkeypatch):
    """No parameter writes or action requests are constructed by the real sampler method."""
    spec = LiveSpec('world', 'wheel', 'axle', (TopicRule('/drive'),),
                    {'/adapter': {'limit': 0.2, 'missing': 1}})
    probe = live_ros.RosProbe(spec)
    clients, requests, destroyed = [], [], []

    def create(service, name):
        clients.append((service, name))

        def call(request):
            if service is ListParameters:
                response = SimpleNamespace(result=SimpleNamespace(names=['limit']))
            else:
                requests.append(request.names)
                response = SimpleNamespace(values=[
                    ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.2)])
            return SimpleNamespace(done=lambda: True, result=lambda: response)

        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

    probe.node = SimpleNamespace(
        get_node_names_and_namespaces=lambda: [('adapter', '/')],
        create_client=create, destroy_client=destroyed.append, count_publishers=lambda _: 1,
        get_topic_names_and_types=lambda: [])
    monkeypatch.setattr(probe, 'localized', lambda: True)
    monkeypatch.setattr(probe, '_age', lambda: 0.1)
    monkeypatch.setattr(probe, 'spin', lambda _: None)
    result = probe.snapshot(lambda: False)
    assert clients == [(ListParameters, '/adapter/list_parameters'),
                       (GetParameters, '/adapter/get_parameters')]
    assert requests == [['limit']]
    assert result.parameters == {'/adapter': {'limit': 0.2}}
    assert len(destroyed) == 2


def test_parameter_timeouts_share_one_global_deadline_and_destroy_clients(monkeypatch):
    """Many absent services cannot multiply the supervisor's timeout per node."""
    spec = LiveSpec('world', 'wheel', 'axle', (),
                    {f'/node{index}': {'key': 1} for index in range(10)})
    probe = live_ros.RosProbe(spec)
    now, destroyed = [0.0], []
    probe.node = SimpleNamespace(
        get_node_names_and_namespaces=lambda: [(f'node{index}', '/') for index in range(10)],
        create_client=lambda *_: SimpleNamespace(service_is_ready=lambda: False),
        destroy_client=destroyed.append, get_topic_names_and_types=lambda: [])

    def spin(duration):
        now[0] += duration

    monkeypatch.setattr(live_ros.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(probe, 'spin', spin)
    monkeypatch.setattr(probe, 'localized', lambda: True)
    monkeypatch.setattr(probe, '_age', lambda: None)
    result = probe.snapshot(lambda: False, timeout_sec=0.5)
    assert now[0] == 0.5
    assert len(destroyed) == 10
    assert result.parameters == {}


@pytest.mark.parametrize('arrival', [0.2, None])
def test_node_discovery_uses_the_parameter_budget_without_early_missing(monkeypatch, arrival):
    """Late DDS node discovery is given the same bounded budget as parameter responses."""
    probe = live_ros.RosProbe(LiveSpec('world', 'wheel', 'axle', (), {'/late': {'key': 1}}))
    now, destroyed = [0.0], []
    response = SimpleNamespace(values=[
        ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=1)])

    def create(service, _name):
        result = (SimpleNamespace(result=SimpleNamespace(names=['key']))
                  if service is ListParameters else response)
        future = SimpleNamespace(done=lambda: True, result=lambda: result)
        return SimpleNamespace(service_is_ready=lambda: True, call_async=lambda _: future)

    probe.node = SimpleNamespace(
        get_node_names_and_namespaces=lambda: [('late', '/')] if (
            arrival is not None and now[0] >= arrival) else [],
        create_client=create,
        destroy_client=destroyed.append, get_topic_names_and_types=lambda: [])

    def spin(duration):
        now[0] += duration

    monkeypatch.setattr(live_ros.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(probe, 'spin', spin)
    monkeypatch.setattr(probe, 'localized', lambda: True)
    monkeypatch.setattr(probe, '_age', lambda: 0.1)
    result = probe.snapshot(lambda: False, timeout_sec=0.5)
    if arrival is None:
        assert now[0] == 0.5
        assert result.node_counts == {} and result.parameters == {}
        assert not destroyed
    else:
        assert arrival <= now[0] <= 0.5
        assert result.parameters == {'/late': {'key': 1}}
        assert len(destroyed) == 2


@pytest.mark.parametrize('declared', [['limit'], []])
def test_cpp_batch_rejection_cannot_hide_declared_parameter_values(monkeypatch, declared):
    """An undeclared key must be reported without discarding all other values in a node."""
    probe = live_ros.RosProbe(LiveSpec(
        'world', 'wheel', 'axle', (), {'/cpp': {'limit': 0.2, 'missing': 1}}))
    calls, destroyed, now = [], [], [0.0]

    def create(service, name):
        assert service in (GetParameters, ListParameters)

        def call(request):
            calls.append((service, request))
            if service is ListParameters:
                response = SimpleNamespace(result=SimpleNamespace(names=declared))
            else:
                # Match the C++ service: any undeclared name invalidates the whole batch.
                response = SimpleNamespace(values=[] if 'missing' in request.names else [
                    ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.2)])
            return SimpleNamespace(done=lambda: True, result=lambda: response)

        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

    probe.node = SimpleNamespace(
        get_node_names_and_namespaces=lambda: [('cpp', '/')], create_client=create,
        destroy_client=destroyed.append, get_topic_names_and_types=lambda: [])
    monkeypatch.setattr(probe, 'localized', lambda: True)
    monkeypatch.setattr(probe, '_age', lambda: 0.1)
    monkeypatch.setattr(live_ros.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(probe, 'spin', lambda duration: now.__setitem__(0, now[0] + duration))
    result = probe.snapshot(lambda: False, 0.5)
    assert result.parameters == {'/cpp': {'limit': 0.2} if declared else {}}
    assert [request.names for service, request in calls if service is GetParameters] == (
        [['limit']] if declared else [])
    assert len(destroyed) == (2 if declared else 1)


def test_humble_python_nested_declarations_are_not_lost_to_prefix_semantics(monkeypatch):
    """Use the actual read-only Python service callback without constructing a ROS node."""
    probe = live_ros.RosProbe(LiveSpec(
        'world', 'wheel', 'axle', (), {'/python': {'group.limit': 0.2}}))
    service = SimpleNamespace(_get_node=lambda: SimpleNamespace(_parameters={'group.limit': None}))

    def create(kind, _name):
        def call(request):
            if kind is ListParameters:
                response = ParameterService._list_parameters_callback(
                    service, request, ListParameters.Response())
            else:
                assert request.names == ['group.limit']
                response = SimpleNamespace(values=[
                    ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.2)])
            return SimpleNamespace(done=lambda: True, result=lambda: response)
        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

    probe.node = SimpleNamespace(
        get_node_names_and_namespaces=lambda: [('python', '/')], create_client=create,
        destroy_client=lambda _: None, get_topic_names_and_types=lambda: [])
    monkeypatch.setattr(probe, 'localized', lambda: True)
    monkeypatch.setattr(probe, '_age', lambda: 0.1)
    monkeypatch.setattr(probe, 'spin', lambda _: None)
    assert probe.snapshot(lambda: False).parameters == {'/python': {'group.limit': 0.2}}


@pytest.mark.parametrize('arrival', [0.2, None])
def test_tf_discovery_can_finish_after_parameter_queries_within_same_budget(monkeypatch, arrival):
    """A late static body transform must not be labeled absent before the global bound."""
    probe = live_ros.RosProbe(LiveSpec('world', 'wheel', 'axle', (), {}))
    now = [0.0]
    probe.node = SimpleNamespace(get_node_names_and_namespaces=lambda: [],
                                 get_topic_names_and_types=lambda: [])
    monkeypatch.setattr(probe, 'localized', lambda: True)
    monkeypatch.setattr(probe, '_age', lambda: 0.1 if (
        arrival is not None and now[0] >= arrival) else None)
    monkeypatch.setattr(live_ros.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(probe, 'spin', lambda duration: now.__setitem__(0, now[0] + duration))
    result = probe.snapshot(lambda: False, 0.5)
    if arrival is None:
        assert now[0] == 0.5 and result.tf_age_sec is None
    else:
        assert arrival <= now[0] <= 0.5 and result.tf_age_sec == 0.1
