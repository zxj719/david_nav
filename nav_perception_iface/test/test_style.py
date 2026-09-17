"""新包代码的格式、Google 文档与机器人常量门禁."""

from pathlib import Path

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257

from nav_bringup.lint import docstrings, literals


def test_long_private_functions_are_documented():
    """Require nonblank documentation for private definitions over ten lines."""
    paths = [Path(__file__).resolve().parents[1] / 'nav_perception_iface']
    violations = docstrings.check_private_paths(paths)
    assert not violations, docstrings.format_violations(violations)


ROOT = Path(__file__).resolve().parents[1]


def test_flake8():
    """All Python and launch files meet ROS formatting requirements."""
    code, errors = main_with_errors(argv=[])
    assert code == 0, '\n'.join(errors)


def test_pep257():
    """Public documentation follows the Google convention."""
    assert pep257(argv=['.', 'test', '--convention', 'google']) == 0


def test_docstrings_and_literal_boundaries():
    """Runtime code and launch have complete API docs and no robot-specific literals."""
    paths = [ROOT / 'nav_perception_iface', ROOT / 'launch']
    assert not docstrings.check_paths(paths)
    assert not literals.check_paths(paths)
