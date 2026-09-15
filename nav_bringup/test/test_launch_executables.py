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


@pytest.mark.parametrize('registration', [
    '', 'other::Node;lib/node.so', 'demo::Node',
    'demo::Node;lib/node.so\ndemo::Node;lib/other.so',
    'demo::Node;/absolute.so', 'demo::Node;../escape.so', 'demo::Node;lib/absent.so',
])
def test_bad_component_registration_fails_read_only(tmp_path, monkeypatch, registration):
    """Missing, duplicate and unsafe plugin resource entries cannot reach runtime loading."""
    from ament_index_python import resources
    from nav_bringup.launching.executables import resolve_component

    monkeypatch.setattr(resources, 'get_resource', lambda *_: (registration, str(tmp_path)))
    with pytest.raises(ValueError):
        resolve_component('demo', 'demo::Node')
    assert not list(tmp_path.iterdir())


def test_component_resolution_checks_library_without_loading_it(tmp_path, monkeypatch):
    """The read-only planner never executes a constructor or dlopen."""
    from ament_index_python import resources
    from nav_bringup.launching.executables import resolve_component

    library = tmp_path / 'lib/node.so'
    library.parent.mkdir()
    library.write_text('not an executable shared library')
    monkeypatch.setattr(resources, 'get_resource',
                        lambda *_: ('demo::Node;lib/node.so', str(tmp_path)))
    assert resolve_component('demo', 'demo::Node') == str(library)
