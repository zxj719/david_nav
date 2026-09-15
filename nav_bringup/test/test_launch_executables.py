"""Resolve package executables without running them or constructing ROS actions."""

from ament_index_python import packages
import pytest

from nav_bringup.launching.executables import resolve_executable


@pytest.mark.parametrize('name', ['', '.', '..', '../other', '/absolute'])
def test_bad_executable_names_are_rejected(name):
    """An entry point cannot escape its package's lib directory."""
    with pytest.raises(ValueError, match='basename'):
        resolve_executable('unused_package', name)


def test_executable_resolution_is_read_only(tmp_path, monkeypatch):
    """A regular executable is resolved but never invoked."""
    path = tmp_path / 'lib/demo/entry'
    path.parent.mkdir(parents=True)
    path.write_text('this is deliberately not a runnable program')
    path.chmod(0o700)
    monkeypatch.setattr(packages, 'get_package_prefix', lambda _: str(tmp_path))
    assert resolve_executable('demo', 'entry') == str(path)
    path.chmod(0o600)
    with pytest.raises(ValueError, match='missing executable'):
        resolve_executable('demo', 'entry')


def test_missing_package_is_not_hidden():
    """Dependency failures remain distinguishable from configuration errors."""
    with pytest.raises(packages.PackageNotFoundError):
        resolve_executable('navrun_test_nonexistent_package', 'entry')
