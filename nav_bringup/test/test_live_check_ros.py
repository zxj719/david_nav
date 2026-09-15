"""Read-only observer methods tested on service/TF fakes without initializing ROS."""

from types import SimpleNamespace

from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters

from nav_bringup.navcfg import live_ros
from nav_bringup.navcfg.live_check import LiveSpec, TopicRule


def test_snapshot_uses_only_get_parameters_and_excludes_undeclared_values(monkeypatch):
    """No parameter writes or action requests are constructed by the real sampler method."""
    spec = LiveSpec('world', 'wheel', 'axle', (TopicRule('/drive'),),
                    {'/adapter': {'limit': 0.2, 'missing': 1}})
    probe = live_ros.RosProbe(spec)
    clients, requests, destroyed = [], [], []
    response = SimpleNamespace(values=[
        ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.2), ParameterValue()])
    future = SimpleNamespace(done=lambda: True, result=lambda: response)

    def create(service, name):
        clients.append((service, name))

        def call(request):
            requests.append(request.names)
            return future

        return SimpleNamespace(service_is_ready=lambda: True, call_async=call)

    probe.node = SimpleNamespace(
        get_node_names_and_namespaces=lambda: [('adapter', '/')],
        create_client=create, destroy_client=destroyed.append, count_publishers=lambda _: 1,
        get_topic_names_and_types=lambda: [])
    monkeypatch.setattr(probe, 'localized', lambda: True)
    monkeypatch.setattr(probe, '_age', lambda: 0.1)
    result = probe.snapshot(lambda: False)
    assert clients == [(GetParameters, '/adapter/get_parameters')]
    assert requests == [['limit', 'missing']]
    assert result.parameters == {'/adapter': {'limit': 0.2}}
    assert len(destroyed) == 1


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
