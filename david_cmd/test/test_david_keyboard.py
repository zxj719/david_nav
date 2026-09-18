# Copyright 2026 xingjian zhang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time
from types import SimpleNamespace

import pytest

from david_cmd.david_keyboard import (
    DavidKeyboardNode,
    apply_min_effective_speed,
    motion_for_key,
)


def make_keyboard(**overrides):
    parameters = {
        "cmd_vel_topic": "/test_cmd_vel_safe_keyboard",
        "linear_speed": 0.10,
        "angular_speed": 0.20,
        "linear_deadzone": 0.20,
        "angular_deadzone": 0.35,
        "max_linear_x": 0.30,
        "max_angular_z": 0.50,
        "publish_hz": 20.0,
        "key_timeout": 0.20,
    }
    parameters.update(overrides)
    node = object.__new__(DavidKeyboardNode)
    for name, value in parameters.items():
        setattr(node, name, value)
    node.outputs = []
    node.cmd_publisher = SimpleNamespace(publish=node.outputs.append)
    node.active_twist = node.build_twist(0.0, 0.0)
    node.last_motion_time = 0.0
    node.destroy_node = lambda: None
    return node


def test_apply_min_effective_speed_lifts_non_zero_commands_out_of_deadzone():
    assert apply_min_effective_speed(0.0, 0.20, 0.30) == 0.0
    assert apply_min_effective_speed(0.10, 0.20, 0.30) == pytest.approx(0.20)
    assert apply_min_effective_speed(-0.10, 0.20, 0.30) == pytest.approx(-0.20)
    assert apply_min_effective_speed(0.40, 0.20, 0.30) == pytest.approx(0.30)


def test_arrow_and_wasd_keys_map_to_base_directions():
    assert motion_for_key("w") == (1.0, 0.0)
    assert motion_for_key("S") == (-1.0, 0.0)
    assert motion_for_key("a") == (0.0, 1.0)
    assert motion_for_key("d") == (0.0, -1.0)
    assert motion_for_key("\x1b[A") == (1.0, 0.0)
    assert motion_for_key("\x1b[B") == (-1.0, 0.0)
    assert motion_for_key("\x1b[D") == (0.0, 1.0)
    assert motion_for_key("\x1b[C") == (0.0, -1.0)


def test_keyboard_node_builds_deadzone_aware_twists():
    node = make_keyboard()

    try:
        forward = node.build_twist(1.0, 0.0)
        left = node.build_twist(0.0, 1.0)

        assert forward.linear.x == pytest.approx(0.20)
        assert forward.angular.z == 0.0
        assert left.linear.x == 0.0
        assert left.angular.z == pytest.approx(0.35)
    finally:
        node.destroy_node()


def test_keyboard_timeout_publishes_zero():
    keyboard = make_keyboard(key_timeout=0.05)
    outputs = keyboard.outputs

    try:
        keyboard.set_motion(1.0, 0.0)
        keyboard.last_motion_time = time.monotonic() - 1.0
        keyboard.publish_current_command()

        assert outputs
        assert outputs[-1].linear.x == 0.0
        assert outputs[-1].angular.z == 0.0
    finally:
        keyboard.destroy_node()
