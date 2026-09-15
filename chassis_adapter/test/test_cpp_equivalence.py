"""Execute the C++ pure core against Python without ROS or hardware endpoints."""

from dataclasses import replace
import os
from pathlib import Path
import random
import subprocess

import pytest

from chassis_adapter.core import AdapterConfig, AdapterState


KEYS = ('cmd_vel_timeout', 'min_linear_x', 'max_linear_x', 'min_angular_z',
        'min_stationary_angular_z', 'stationary_angular_threshold', 'max_angular_z', 'publish_hz')


@pytest.mark.parametrize('timeout,hz,floor,max_yaw,min_x', [
    (2.0, 50.0, 0.35, 0.5, 0.2), (1.0, 20.0, 0.55, 0.6, 0.0),
    (0.0, 50.0, 0.55, 0.6, 0.0), (-1.0, -2.0, 0.55, 0.6, 0.2),
    (0.000000001, 20.0, 0.8, 0.5, 0.2)])
def test_cpp_core_matches_python_across_commands_clocks_and_stale_transitions(
        timeout, hz, floor, max_yaw, min_x):
    """Mixed/lateral inputs, signed turns, clock jumps and timeout boundaries match."""
    fallback = Path(__file__).resolve().parents[3] / 'build/chassis_adapter/adapter_core_oracle'
    executable = Path(os.environ.get('ADAPTER_CORE_ORACLE', str(fallback)))
    assert executable.is_file(), 'build the C++ oracle before running equivalence tests'
    config = replace(AdapterConfig(), cmd_vel_timeout=timeout, publish_hz=hz,
                     min_stationary_angular_z=floor, max_angular_z=max_yaw, min_linear_x=min_x)
    state = AdapterState(config, 0)
    lines = [' '.join(str(getattr(config, key)) for key in KEYS), 'P', 'O 0']
    expected = [(config.period_sec,), state.output(0)]
    rng = random.Random(23053)
    values = [-1.0, -0.55, -0.025, -0.001, -0.0, 0.0, 0.001, 0.025, 0.55, 1.0]
    for index in range(2500):
        x, y, yaw = (rng.choice(values) for _ in range(3))
        now = index * 10000000
        lines.append(f'R {x} {y} {yaw} {now}')
        state.receive(x, y, yaw, now)
        for offset in (-1, 0, int(max(timeout, 0) * 1e9),
                       int(max(timeout, 0) * 1e9) + 1, 5000000000):
            tick = now + offset
            lines.append(f'O {tick}')
            expected.append(state.output(tick))
        lines.append(f'D {now}')
        values_at = state.diagnostics(now)
        expected.append((state.input_count, int(values_at['timed_out']),
                         values_at['output_linear_x'], values_at['output_angular_z']))
    result = subprocess.run([str(executable)], input='\n'.join(lines) + '\n', text=True,
                            capture_output=True, timeout=20, check=True)
    actual = [None if line == 'none' else tuple(map(float, line.split()))
              for line in result.stdout.splitlines()]
    assert actual == expected
