"""Observation publisher identity, strict payloads and owned teardown with fake ROS."""

import json
import sys
import threading
from types import ModuleType, SimpleNamespace as NS

import pytest

from nav_tasks.domain.tasks import DockGoal
from nav_tasks.domain.types import PoseRecord


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []
        self.fail = False

    def publish(self, message):
        if self.fail:
            raise OSError('transport failure')
        self.messages.append(message)


class _Node:
    def __init__(self):
        self.publishers, self.destroyed = [], []
        self.fail_at, self.fail_destroy = None, False

    def create_publisher(self, message_type, topic, qos):
        assert qos == 10
        if len(self.publishers) + 1 == self.fail_at:
            raise RuntimeError('construction failed')
        publisher = _Publisher(topic)
        self.publishers.append(publisher)
        return publisher

    def destroy_publisher(self, publisher):
        if self.fail_destroy:
            return False
        self.destroyed.append(publisher)
        return True


@pytest.fixture
def message_types(monkeypatch):
    """Only constructors are replaced; no rclpy context or ROS process is started."""
    for package, name in [('david_nav_msgs', 'NavEvent'), ('std_msgs', 'String')]:
        messages = ModuleType(package + '.msg')
        setattr(messages, name, NS)
        root = ModuleType(package)
        root.msg = messages
        monkeypatch.setitem(sys.modules, package, root)
        monkeypatch.setitem(sys.modules, package + '.msg', messages)


def _open(*, dock=False, now=lambda: 1234.5):
    from nav_tasks.adapters.ros.event_publisher import NavigationEvents

    node = _Node()
    owner = NavigationEvents(now=now)
    owner.open(node, dock=dock)
    return owner, node


@pytest.fixture
def warnings(monkeypatch):
    """Capture the adapter sink independently of colcon's process-wide logging hooks."""
    from nav_tasks.adapters.ros import event_publisher

    records = []
    monkeypatch.setattr(event_publisher, '_LOGGER', NS(
        warning=lambda message, *args: records.append(message % args)))
    return records


def test_task_events_keep_ingress_metadata_and_unique_producer_identity(message_types):
    """Global observations use the generated envelope without altering task correlation."""
    owner, node = _open()
    data = {'task_id': 'same', 'generation': 1, 'ingress_source': 'websocket'}
    owner.task('task_admitted', data)
    owner.task('task_failed', dict(data, detail='目标点不存在'))
    first, second = node.publishers[0].messages
    assert len(node.publishers) == 1 and node.publishers[0].topic == '/nav/events'
    assert first.event_id and first.event_id != second.event_id
    assert (first.stamp, first.source, first.event_type, first.severity) == (
        1234.5, 'tasks', 'task_admitted', 'info')
    assert second.severity == 'error' and '目标点不存在' in second.data_json
    assert json.loads(first.data_json) == data
    other, second_node = _open()
    other.task('task_admitted', data)
    assert second_node.publishers[0].messages[0].event_id != first.event_id
    owner.close()
    owner.close()
    owner.task('late', data)
    assert len(node.publishers[0].messages) == 2 and len(node.destroyed) == 1
    other.close()


def test_dock_dual_write_keeps_numbers_stamp_and_identity(message_types):
    """R7 private JSON and NavEvent share the same unrounded measurement and stamp."""
    owner, node = _open(dock=True)
    goal = DockGoal(PoseRecord('world', 1, 2, 0, 0, 1234), 'same', 2, 17)
    values = {'exy': 0.0123456789, 'eyaw': -0.00123456789, 'settled': True}
    owner.dock(goal, 'measured', values)
    global_event = node.publishers[0].messages[0]
    private = json.loads(node.publishers[1].messages[0].data)
    assert node.publishers[1].topic == '~/events'
    assert global_event.event_type == 'dock_measured' and global_event.source == 'dock'
    assert private['event'] == 'measured'
    assert private['stamp'] == global_event.stamp
    assert private['event_id'] == global_event.event_id
    data = json.loads(global_event.data_json)
    assert data == dict(values, task_id='same', sub_id=2, generation=17)
    assert all(private[key] == value for key, value in data.items())
    assert values == {'exy': 0.0123456789, 'eyaw': -0.00123456789, 'settled': True}
    owner.close()


@pytest.mark.parametrize('data', [
    {'x': float('nan')}, {'x': float('inf')}, {'x': object()}, [('x', 1)],
])
def test_invalid_json_is_rejected_without_publishing(message_types, data, warnings):
    """Bad observations are diagnosed rather than publishing nonportable JSON or crashing."""
    owner, node = _open()
    owner.task('event', data)
    assert not node.publishers[0].messages and 'observation' in warnings[0]
    owner.close()


@pytest.mark.parametrize('stamp', [float('nan'), float('inf'), True])
def test_invalid_wall_clock_does_not_emit(message_types, stamp):
    """A broken clock is not a valid Unix timestamp, including bool-as-integer coercion."""
    owner, node = _open(now=lambda: stamp)
    owner.task('event', {})
    assert not node.publishers[0].messages
    owner.close()


@pytest.mark.parametrize('name,data', [(None, {}), (' ', {}), ('measured', None)])
def test_invalid_dock_observation_is_isolated(message_types, name, data):
    """Invalid private observations cannot escape through the worker callback boundary."""
    owner, node = _open(dock=True)
    goal = DockGoal(PoseRecord('world', 1, 2, 0, 0, 1234), 't')
    owner.dock(goal, name, data)
    assert all(not publisher.messages for publisher in node.publishers)
    owner.close()


def test_partial_open_and_failed_destroy_remain_owned_for_retry(message_types):
    """Creating one publisher before the second fails cannot lose the first handle."""
    from nav_tasks.adapters.ros.event_publisher import NavigationEvents

    owner, node = NavigationEvents(), _Node()
    node.fail_at = 2
    with pytest.raises(RuntimeError, match='construction'):
        owner.open(node, dock=True)
    owner.task('must_not_emit', {})
    assert not node.publishers[0].messages
    node.fail_destroy = True
    with pytest.raises(RuntimeError, match='destroy'):
        owner.close()
    assert not node.destroyed
    node.fail_destroy = False
    owner.close()
    owner.close()
    assert node.destroyed == node.publishers
    with pytest.raises(RuntimeError):
        owner.open(node)


def test_failure_on_one_surface_does_not_disable_the_other(message_types, warnings):
    """Observability publication failure never becomes a Dock failure or suppresses R7."""
    owner, node = _open(dock=True)
    node.publishers[0].fail = True
    goal = DockGoal(PoseRecord('world', 1, 2, 0, 0, 1234), 't')
    owner.dock(goal, 'measured', {'settled': True})
    assert len(node.publishers[1].messages) == 1
    assert 'observation' in warnings[0]
    owner.close()


def test_concurrent_worker_and_loop_publication_uses_unique_ids(message_types):
    """The shared publisher has no new worker but safely accepts the existing Dock thread."""
    owner, node = _open()
    threads = [threading.Thread(target=lambda: [owner.task('tick', {}) for _ in range(100)])
               for _ in range(4)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(timeout=2)
        assert not thread.is_alive()
    messages = node.publishers[0].messages
    assert len(messages) == len({message.event_id for message in messages}) == 400
    owner.close()
