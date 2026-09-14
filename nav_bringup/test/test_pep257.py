"""Run ament_pep257 with the Google convention on the package."""

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Docstrings follow the Google convention."""
    rc = main(argv=['.', 'test', '--convention', 'google'])
    assert rc == 0, 'Found code style errors / warnings'
