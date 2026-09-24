"""Validate installed adapter parameters and zero-only shutdown in isolated DDS."""

from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
import pytest
import rclpy
import yaml


@pytest.mark.parametrize('mode,signum', [('tour', signal.SIGINT), ('precise', signal.SIGTERM)])
def test_adapter_parameters_and_exit_zero(sessions, tmp_path, mode, signum):
    """No command input is ever published; only the owned node's shutdown zeros appear."""
    source = Path(__file__).resolve().parents[2]
    profile = yaml.safe_load((source / 'david_config/robot/david.yaml').read_text())
    adapter = profile['chassis']['adapter']
    expected = {
        'input_cmd_vel_topic': adapter['input_topic'],
        'output_cmd_vel_topic': profile['chassis']['cmd_topic'],
        'max_linear_x': adapter['hard_max_linear_x'],
        'max_angular_z': adapter['hard_max_angular_z'],
        'min_linear_x': adapter['min_linear_x'],
        'min_angular_z': adapter['min_angular_z'],
        'min_stationary_angular_z': adapter['min_stationary_angular_z'],
        'stationary_angular_threshold': adapter['stationary_angular_threshold'],
        'cmd_vel_timeout': adapter['cmd_timeout_sec'],
        'publish_hz': adapter['publish_hz'],
    }
    parameter_file = tmp_path / 'adapter.yaml'
    parameter_file.write_text(yaml.safe_dump({
        'chassis_adapter': {'ros__parameters': expected}}))
    executable = (Path(get_package_prefix('chassis_adapter'))
                  / 'lib/chassis_adapter/chassis_adapter')
    process = sessions.start('adapter', [str(executable), '--ros-args',
                                         '--params-file', str(parameter_file)])
    rclpy.init(args=[])
    observer = rclpy.create_node('adapter_zero_observer')
    zeros, diagnostics = [], []
    observer.create_subscription(Twist, expected['output_cmd_vel_topic'], zeros.append, 10)
    observer.create_subscription(DiagnosticArray, '/diagnostics', diagnostics.append, 10)
    try:
        deadline = time.monotonic() + 12.0
        while not diagnostics and time.monotonic() < deadline and process.poll() is None:
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert diagnostics, (tmp_path / 'process_logs/adapter.log').read_text()
        assert not zeros, 'startup must remain silent without input'
        result = subprocess.run(['ros2', 'param', 'dump', '/chassis_adapter'],
                                capture_output=True, text=True, timeout=15, check=False)
        assert result.returncode == 0, result.stderr
        actual = yaml.safe_load(result.stdout)['/chassis_adapter']['ros__parameters']
        assert {name: actual[name] for name in expected} == expected
        # Signal only the child returned by our own session fixture.
        process.send_signal(signum)
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and (process.poll() is None or len(zeros) < 5):
            rclpy.spin_once(observer, timeout_sec=0.1)
        assert process.wait(timeout=1) == 0
        assert len(zeros) >= 1, 'no shutdown zero was observed'
        assert all(message == Twist() for message in zeros)
    finally:
        observer.destroy_node()
        rclpy.shutdown()
