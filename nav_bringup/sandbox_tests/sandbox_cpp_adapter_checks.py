"""Compare real adapter endpoints and component teardown using only zero commands."""

from dataclasses import asdict
import os
from pathlib import Path
import signal
import time

from ament_index_python.packages import get_package_prefix
from composition_interfaces.srv import LoadNode, UnloadNode
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
import pytest
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
import yaml

from chassis_adapter.core import AdapterConfig
from conftest import require_sandbox
from nav_bringup.navrun.owned_process import OwnedProcess


def _until(observer, predicate, log, timeout=12.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        observer.executor.spin_once(timeout_sec=0.02)
    pytest.fail(log.read_text())


def _spin(observer, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        observer.executor.spin_once(timeout_sec=0.02)


@pytest.mark.parametrize('backend,finish', [
    ('python', 'int'), ('python', 'term'), ('cpp', 'int'), ('cpp', 'term'),
    ('cpp_integer', 'int'), ('component', 'unload'), ('component', 'term'),
    ('component_mt', 'unload'), ('component_mt', 'term')])
def test_cpp_python_and_component_share_parameters_timeout_and_stop(tmp_path, backend, finish):
    """Real DDS checks exercise zero-only flow, one-shot stale silence and five-stop teardown."""
    require_sandbox()
    context = Context()
    rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
    observer = rclpy.create_node('cpp_adapter_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    process = None
    log = tmp_path / 'adapter.log'
    expected = asdict(AdapterConfig())
    expected.update(input_cmd_vel_topic='/cpp_check/input',
                    output_cmd_vel_topic='/cpp_check/output',
                    cmd_vel_timeout=0.25, publish_hz=20.0, min_linear_x=0.0,
                    min_stationary_angular_z=0.55, max_angular_z=0.6)
    if backend == 'cpp_integer':
        expected['publish_hz'] = 20
    messages, statuses = [], []
    observer.create_subscription(Twist, expected['output_cmd_vel_topic'], messages.append, 20)
    observer.create_subscription(DiagnosticArray, '/diagnostics',
                                 lambda message: statuses.extend(message.status), 10)
    publisher = observer.create_publisher(Twist, expected['input_cmd_vel_topic'], 10)
    name = 'adapter_under_test'
    unique_id = None
    try:
        if backend.startswith('component'):
            container = 'component_container_mt' if backend == 'component_mt' else (
                'component_container')
            executable = Path(get_package_prefix('rclcpp_components')) / (
                'lib/rclcpp_components') / container
            process = OwnedProcess([str(executable), '--ros-args', '-r',
                                    '__node:=adapter_container'], os.environ, log)
            client = observer.create_client(LoadNode, '/adapter_container/_container/load_node')
            _until(observer, client.service_is_ready, log)
            request = LoadNode.Request()
            request.package_name = 'chassis_adapter'
            request.plugin_name = 'chassis_adapter::AdapterNode'
            request.node_name = name
            request.parameters = [Parameter(key, value=value).to_parameter_msg()
                                  for key, value in expected.items()]
            future = client.call_async(request)
            _until(observer, future.done, log)
            response = future.result()
            assert response.success, response.error_message
            unique_id = response.unique_id
        else:
            executable = Path(get_package_prefix('chassis_adapter')) / 'lib/chassis_adapter' / (
                'chassis_adapter' if backend == 'python' else 'chassis_adapter_cpp')
            params = tmp_path / 'params.yaml'
            params.write_text(yaml.safe_dump({'/**': {'ros__parameters': expected}}))
            process = OwnedProcess([str(executable), '--ros-args', '--params-file', str(params),
                                    '-r', '__node:=' + name], os.environ, log)
        _until(observer, lambda: statuses and publisher.get_subscription_count() == 1, log)
        assert process.poll() is None, log.read_text()
        assert not messages, 'no input must mean no startup velocity publication'
        assert statuses[-1].name == '/' + name
        assert dict((item.key, item.value) for item in statuses[-1].values)['timed_out'] == 'True'
        parameters = observer.create_client(GetParameters, '/' + name + '/get_parameters')
        _until(observer, parameters.service_is_ready, log)
        request = GetParameters.Request()
        request.names = list(expected)
        future = parameters.call_async(request)
        _until(observer, future.done, log)
        actual = {}
        for key, value in zip(expected, future.result().values):
            kind = value.type
            actual[key] = (value.string_value if kind == Parameter.Type.STRING.value else
                           value.integer_value if kind == Parameter.Type.INTEGER.value else
                           value.double_value)
        assert actual == expected
        assert isinstance(actual['publish_hz'], int if backend == 'cpp_integer' else float)
        deadline = time.monotonic() + 1.25
        while time.monotonic() < deadline:
            publisher.publish(Twist())  # S1: zero is the only permitted publication.
            _spin(observer, 0.05)
        assert 10 <= len(messages) <= 40
        assert any(dict((item.key, item.value) for item in status.values)['timed_out'] == 'False'
                   for status in statuses)
        _spin(observer, 0.7)
        settled = len(messages)
        _spin(observer, 0.5)
        assert len(messages) == settled, 'stale adapter must release output after stopping'
        final_status = {item.key: item.value for item in statuses[-1].values}
        assert final_status['timed_out'] == 'True'
        if finish == 'unload':
            client = observer.create_client(
                UnloadNode, '/adapter_container/_container/unload_node')
            _until(observer, client.service_is_ready, log)
            request = UnloadNode.Request()
            request.unique_id = unique_id
            future = client.call_async(request)
            _until(observer, future.done, log)
            assert future.result().success, future.result().error_message
            _until(observer, lambda: len(messages) >= settled + 5, log)
            assert process.poll() is None, 'unload must not stop the container'
        else:
            os.kill(process.pid, signal.SIGINT if finish == 'int' else signal.SIGTERM)
            _until(observer, lambda: process.poll() is not None and len(messages) >= settled + 5,
                   log)
            assert process.poll() == 0, log.read_text()
        assert all(message == Twist() for message in messages)
    finally:
        if process is not None:
            process.close()
        executor.shutdown()
        observer.destroy_node()
        rclpy.shutdown(context=context)
