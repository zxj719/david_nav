"""Validate named resources through the real offline renderer, never host startup."""

from pathlib import Path
import shutil

import pytest
import yaml

from nav_bringup.navcfg import cli

SOURCE = Path(__file__).resolve().parents[2]


def _invoke(*options):
    try:
        return cli.main(['validate', *map(str, options)])
    except SystemExit as error:
        return error.code


def _source(root, mode='tour', *options):
    return _invoke('--robot', 'example_diffdrive', '--mode', mode,
                   '--source-root', root, *options)


@pytest.fixture
def copied(tmp_path):
    """Copy configuration assets only; mutations never reach repository fixtures."""
    root = tmp_path / 'src'
    shutil.copytree(SOURCE / 'example_diffdrive_config', root / 'example_diffdrive_config')
    for path in SOURCE.glob('*/config/schema.yaml'):
        package = path.parent.parent.name
        (root / package / 'config').mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, root / package / 'config/schema.yaml')
        shutil.copy2(SOURCE / package / 'package.xml', root / package / 'package.xml')
    for package in ('nav_bringup', 'nav_plugins'):
        shutil.copytree(SOURCE / package / 'config', root / package / 'config',
                        dirs_exist_ok=True)
    shutil.copytree(SOURCE / 'nav_plugins/behavior_trees', root / 'nav_plugins/behavior_trees')
    return root


@pytest.mark.parametrize('mode', ['tour', 'precise', 'chassis', 'teleop'])
def test_source_selector_validates_four_modes_without_writes(copied, mode, capsys):
    """The selector renders configuration but never creates a run or modifies its input."""
    robot = copied / 'example_diffdrive_config'
    before = {p: p.read_bytes() for p in robot.rglob('*') if p.is_file()}
    assert _source(copied, mode) == 0
    assert 'robot=example_diffdrive' in capsys.readouterr().out
    assert {p: p.read_bytes() for p in robot.rglob('*') if p.is_file()} == before


def test_installed_resource_resolver_uses_selected_robot(monkeypatch, capsys):
    """The installed lookup requests the selected config package, never a David default."""
    from ament_index_python import packages
    seen = []

    def share(package):
        seen.append(package)
        return str(SOURCE / package)

    monkeypatch.setattr(packages, 'get_package_share_directory', share)
    paths = sorted(SOURCE.glob('*/config/schema.yaml'))
    schemas = [item for path in paths for item in ('--schema', str(path))]
    assert _invoke('--robot', 'example_diffdrive', '--mode', 'tour', *schemas) == 0
    assert 'example_diffdrive_config' in seen and 'david_config' not in seen
    assert 'site=offline' in capsys.readouterr().out


@pytest.mark.parametrize('mutation', ['robot', 'mode', 'site', 'speed', 'map'])
def test_selected_configuration_identity_and_safety_are_rechecked(copied, mutation):
    """Valid YAML or a correct directory name cannot hide identity or safety errors."""
    config = copied / 'example_diffdrive_config'
    if mutation == 'mode':
        path = config / 'modes/tour.yaml'
    elif mutation == 'site':
        path = config / 'sites/offline/site.yaml'
    elif mutation == 'map':
        site = yaml.safe_load((config / 'sites/offline/site.yaml').read_text())
        path = config / 'sites/offline' / site['map']['nav_yaml']
    else:
        path = config / 'robot/example_diffdrive.yaml'
    data = yaml.safe_load(path.read_text())
    if mutation == 'robot':
        data['robot']['name'] = 'unexpected_robot'
    elif mutation == 'mode':
        data['mode'] = 'precise'
    elif mutation == 'site':
        data['site']['name'] = 'unexpected_site'
    elif mutation == 'speed':
        data['limits']['max_vel_x'] = 99.0
    else:
        data['mode'] = 'scale'
    path.write_text(yaml.safe_dump(data))
    assert _source(copied) == 1


def test_multiple_sites_require_explicit_choice(copied, capsys):
    """Porting cannot silently validate an arbitrary first site."""
    sites = copied / 'example_diffdrive_config/sites'
    shutil.copytree(sites / 'offline', sites / 'second')
    assert _source(copied) == 2
    assert '--site' in capsys.readouterr().err
    assert _source(copied, 'tour', '--site', 'offline') == 0


@pytest.mark.parametrize('mode', ['chassis', 'teleop'])
def test_manual_modes_do_not_read_site_map_or_urdf(copied, mode):
    """Manual tools retain their existing independence from localization assets."""
    config = copied / 'example_diffdrive_config'
    (config / 'sites').rename(config / 'saved_sites')
    (config / 'urdf').rename(config / 'saved_urdf')
    assert _source(copied, mode) == 0


@pytest.mark.parametrize('options', [
    ('--robot', '../escape', '--mode', 'tour'),
    ('--robot', 'example_diffdrive', '--mode', '../tour'),
    ('--robot', 'example_diffdrive', '--mode', 'tour', '--site', '../offline'),
    ('--robot', 'example_diffdrive', '--mode', 'tour', '--profile', '/unused'),
    ('--robot', 'example_diffdrive', '--mode', 'tour', '--urdf', '/unused'),
    ('--robot', 'example_diffdrive', '--mode', 'chassis', '--site', 'offline'),
])
def test_ambiguous_or_unsafe_selectors_are_rejected(options):
    """Resource names cannot be paths or mixed with unrelated explicit-file inputs."""
    assert _invoke(*options, '--source-root', SOURCE) == 2


@pytest.mark.parametrize('path', [
    'urdf/example_diffdrive.urdf', 'modes/tour.yaml', 'robot/example_diffdrive.yaml'])
def test_missing_resource_is_an_input_error(copied, path):
    """Missing required assets must not lead to a green validation."""
    (copied / 'example_diffdrive_config' / path).unlink()
    assert _source(copied) == 2


def test_invalid_yaml_is_a_configuration_error(copied):
    """Unreadable YAML content is reported without starting anything or a traceback."""
    (copied / 'example_diffdrive_config/robot/example_diffdrive.yaml').write_text('bad: [')
    assert _source(copied) == 1


def test_absent_sites_are_not_replaced_with_a_david_default(copied):
    """A navigation robot with no site must fail instead of validating another robot."""
    (copied / 'example_diffdrive_config/sites/offline/site.yaml').unlink()
    assert _source(copied) == 2


def test_explicit_profile_still_requires_all_original_files():
    """Keeping the file interface does not silently make its required inputs optional."""
    assert _invoke('--profile', '/unused', '--mode', '/unused_mode') == 2


def test_no_schemas_is_not_a_successful_validation(copied):
    """Schema discovery cannot silently degrade into cross-file-only checks."""
    for path in copied.glob('*/config/schema.yaml'):
        path.rename(path.with_suffix('.disabled'))
    assert _source(copied) == 2


def test_selected_precise_mode_overlay_is_actually_applied(copied):
    """An unsafe mode-only override is not hidden by checking the base profile alone."""
    path = copied / 'example_diffdrive_config/modes/precise.yaml'
    data = yaml.safe_load(path.read_text())
    data.setdefault('profile', {}).setdefault('limits', {})['max_vel_x'] = 99.0
    path.write_text(yaml.safe_dump(data))
    assert _source(copied, 'precise') == 1


def test_selector_never_calls_host_or_process_planning(monkeypatch):
    """Reuse bundle preparation without invoking the broader up workflow."""
    from nav_bringup.navrun import up_planning

    def forbidden(*args, **kwargs):
        pytest.fail('offline validation attempted host/startup planning')

    monkeypatch.setattr(up_planning, 'plan_up', forbidden)
    monkeypatch.setattr(up_planning, 'plan_preflight', forbidden)
    monkeypatch.setattr(up_planning, 'plan_platform', forbidden)
    assert _source(SOURCE) == 0
