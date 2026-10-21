"""新包代码的格式与机器人常量检查."""

from pathlib import Path

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257

from nav_bringup.lint import literals


def test_flake8():
    """All Python and launch files meet ROS formatting requirements."""
    code, errors = main_with_errors(argv=[])
    assert code == 0, '\n'.join(errors)


def test_pep257():
    """Public documentation follows the Google convention."""
    assert pep257(argv=['.', 'test', '--convention', 'google']) == 0


def test_robot_literals():
    """Runtime code contains no robot-specific frame or topic literals."""
    package = Path(__file__).resolve().parents[1] / 'nav_perception_iface'
    assert not literals.check_paths([package])
