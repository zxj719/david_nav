"""Check file-backed driver defaults and the navigation-only IMU boundary."""

from pathlib import Path
import re

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE.parent


def test_thirteen_parameters_have_matching_schema_defaults() -> None:
    """Every standalone parameter has exactly one documented profile key."""
    defaults = yaml.safe_load((PACKAGE / 'config/wheeltec_driver.yaml').read_text())
    values = defaults['wheeltec_robot']['ros__parameters']
    schema = yaml.safe_load((PACKAGE / 'config/schema.yaml').read_text())['parameters']
    prefix = 'chassis.driver.parameters.'
    assert len(values) == 13
    assert set(schema) == {prefix + key for key in values}
    for key, value in values.items():
        assert schema[prefix + key]['default'] == value
        assert not schema[prefix + key].get('required', False)


def test_original_nine_defaults_match_the_immutable_vendor_launch() -> None:
    """The standalone node preserves the frozen device, frame and calibration values."""
    launch = (SOURCE / 'third_party/wheeltec_original/launch'
              / 'turn_on_wheeltec_robot.launch.py').read_text()
    original = dict(re.findall(
        r"DeclareLaunchArgument\(\s*'(\w+)',\s*default_value='([^']*)'", launch))
    values = yaml.safe_load((PACKAGE / 'config/wheeltec_driver.yaml').read_text())
    values = values['wheeltec_robot']['ros__parameters']
    for key in ('usart_port_name', 'odom_frame_id', 'robot_frame_id', 'gyro_frame_id'):
        assert values[key] == original[key]
    assert values['serial_baud_rate'] == int(original['serial_baud_rate'])
    for key in ('odom_x_scale', 'odom_y_scale', 'odom_z_scale_positive', 'odom_z_scale_negative'):
        assert values[key] == float(original[key])


def test_david_navigation_does_not_publish_an_additional_chassis_imu() -> None:
    """The navigation profile leaves IMU ownership to manually started alias 3."""
    profile = yaml.safe_load((SOURCE / 'david_config/robot/david.yaml').read_text())
    parameters = profile['chassis']['driver']['parameters']
    assert parameters['publish_imu'] is False
    assert parameters['publish_voltage'] is True
    assert parameters['cmd_timeout_sec'] == 0.0
    assert parameters['log_tx_frames'] is False
