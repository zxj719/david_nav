"""ROS host ownership against injected bindings, with no ROS initialization."""

import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

from nav_executor.adapters.ros.host import RosHost


@pytest.fixture
def ros_bindings(monkeypatch):
    """Provide binding doubles recording construction, shutdown and failure stages."""
    state = NS(events=[], fail=None, shutdown_ok=True, node_name='navigation_executor')

    def operation(name):
        state.events.append(name)
        if state.fail == name:
            raise RuntimeError(name)

    class Context:
        def init(self, args):
            state.args = args
            operation('context init')

        def ok(self):
            return True

        def try_shutdown(self):
            operation('context shutdown')

    class Node:
        def __init__(self, name, context):
            assert name == state.node_name and isinstance(context, Context)
            operation('node create')

        def destroy_node(self):
            operation('node destroy')

    class Executor:
        def __init__(self, context):
            assert isinstance(context, Context)
            operation('executor create')

        def add_node(self, node):
            operation('node add')

        def spin_once(self, timeout_sec):
            assert timeout_sec == 0
            operation('spin')

        def shutdown(self, timeout_sec):
            assert timeout_sec == 0
            operation('executor shutdown')
            return state.shutdown_ok

    class Listener:
        def __init__(self, buffer, node, spin_thread):
            assert spin_thread is False
            operation('listener create')

        def unregister(self):
            operation('listener unregister')

    bindings = {
        'rclpy.context': {'Context': Context},
        'rclpy.node': {'Node': Node},
        'rclpy.executors': {'SingleThreadedExecutor': Executor},
        'tf2_ros': {'Buffer': lambda: operation('buffer create'), 'TransformListener': Listener},
    }
    for name, attributes in bindings.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    return state


def test_private_context_executor_and_listener_close_order(ros_bindings):
    """No default executor, rclpy signal handler or listener spin thread is involved."""
    host = RosHost()
    host.open(['--ros-args', '-r', '__node:=example'])
    host.spin_once()
    host.close()
    host.close()
    assert ros_bindings.args == ['--ros-args', '-r', '__node:=example']
    assert ros_bindings.events == [
        'context init', 'node create', 'executor create', 'node add', 'buffer create',
        'listener create', 'spin', 'executor shutdown', 'listener unregister',
        'node destroy', 'context shutdown',
    ]


@pytest.mark.parametrize('stage', [
    'context init', 'node create', 'executor create',
    'node add', 'buffer create', 'listener create'])
def test_partial_open_keeps_completed_stages_owned(ros_bindings, stage):
    """The caller can close the same host after any construction step fails."""
    host = RosHost()
    ros_bindings.fail = stage
    with pytest.raises(RuntimeError, match=stage):
        host.open([])
    ros_bindings.fail = None
    host.close()
    assert ros_bindings.events[-1] == 'context shutdown'
    assert host.node is None and host._context is None


@pytest.mark.parametrize('stage', ['listener unregister', 'node destroy', 'context shutdown'])
def test_failed_teardown_retains_stage_for_retry(ros_bindings, stage):
    """Completed shutdown stages are not repeated when later cleanup fails."""
    host = RosHost()
    host.open([])
    ros_bindings.fail = stage
    with pytest.raises(RuntimeError, match=stage):
        host.close()
    ros_bindings.fail = None
    host.close()
    assert ros_bindings.events.count('executor shutdown') == 1
    assert ros_bindings.events.count(stage) == 2
    assert host.node is None


def test_false_executor_shutdown_cannot_be_mistaken_for_later_success(ros_bindings):
    """Humble's sticky shutdown flag cannot authorize node destruction after a false result."""
    host = RosHost()
    host.open([])
    ros_bindings.shutdown_ok = False
    with pytest.raises(RuntimeError, match='callbacks remain'):
        host.close()
    ros_bindings.shutdown_ok = True
    with pytest.raises(RuntimeError, match='was incomplete'):
        host.close()
    assert 'node destroy' not in ros_bindings.events
    assert 'context shutdown' not in ros_bindings.events
    assert ros_bindings.events.count('executor shutdown') == 1
