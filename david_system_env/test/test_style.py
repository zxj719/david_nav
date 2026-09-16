"""Apply ROS Python formatting and Google docstrings to the data package's tests."""

from ament_flake8.main import main_with_errors
from ament_pep257.main import main as pep257


def test_flake8():
    """The small pure test suite satisfies the same Python style gate as runtime packages."""
    code, errors = main_with_errors(argv=['test'])
    assert code == 0, '\n'.join(errors)


def test_google_docstrings():
    """All public test functions explain their contract using Google conventions."""
    assert pep257(argv=['test', '--convention', 'google']) == 0
