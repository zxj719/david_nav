"""Exercise offline checks and actual live dispatch with in-memory ROS substitutes."""

import json
from pathlib import Path
import signal

import pytest
import yaml

from nav_bringup.navrun import cli
from nav_bringup.navrun.preparation import write_bundle
from test_live_check_cli import observer as _observer
from test_run_preparation import _prepare

SOURCE = Path(__file__).resolve().parents[2]
observer = _observer


def _bundle(tmp_path, mode='tour'):
    prepared = _prepare(tmp_path, mode)
    root = Path(prepared.options.run_dir)
    root.mkdir()
    write_bundle(prepared)
    return root


def _run(root, *options):
    return cli.main(['check', '--run-dir', str(root), '--source-root', str(SOURCE), *options])


@pytest.mark.parametrize('mode', ['tour', 'precise', 'bench_slamgt', 'chassis', 'teleop'])
def test_offline_checks_all_five_modes_without_observer_or_writes(tmp_path, observer, mode):
    """Offline validation never instantiates even the fake live observer."""
    root = _bundle(tmp_path, mode)
    before = {path: path.read_bytes() for path in root.rglob('*') if path.is_file()}
    assert _run(root) == 0
    assert not observer.initialized
    assert {path: path.read_bytes() for path in root.rglob('*') if path.is_file()} == before


@pytest.mark.parametrize('artifact', ['params/inputs/robot.urdf', 'params/rendered/nav2.yaml',
                                      'params/rendered/lifecycle.yaml'])
def test_missing_artifact_fails_before_live_observation(tmp_path, observer, artifact):
    """A record cannot receive a green result when required evidence is absent."""
    root = _bundle(tmp_path)
    (root / artifact).unlink()
    assert _run(root, '--live') == 2
    assert not observer.initialized


@pytest.mark.parametrize('mutation', ['map', 'limit', 'frame', 'teleop_limit'])
def test_offline_recomputes_safety_rules_instead_of_trusting_checks_yaml(
        tmp_path, observer, mutation):
    """Changing frozen inputs invalidates the configuration despite an old green report."""
    root = _bundle(tmp_path, 'teleop' if mutation == 'teleop_limit' else 'tour')
    path = root / ('params/inputs/map.yaml' if mutation == 'map' else
                   'params/inputs/robot.yaml')
    data = yaml.safe_load(path.read_text())
    if mutation == 'map':
        data['mode'] = 'scale'
    elif mutation == 'limit':
        data['limits']['max_vel_x'] = 99.0
    elif mutation == 'frame':
        data['frames']['nav_base'] = 'not_a_urdf_link'
    else:
        data['teleop']['max_linear_x'] = 99.0
    path.write_text(yaml.safe_dump(data))
    assert _run(root, '--live') == 1
    assert not observer.initialized


def test_mode_mismatch_is_operational_failure(tmp_path, observer):
    """A different frozen mode cannot silently be checked as the launch mode."""
    root = _bundle(tmp_path)
    path = root / 'params/inputs/mode.yaml'
    data = yaml.safe_load(path.read_text())
    data['mode'] = 'precise'
    path.write_text(yaml.safe_dump(data))
    assert _run(root) == 2


def test_run_environment_default_is_read_only(tmp_path, monkeypatch, observer):
    """The owning supervisor's run directory is used without starting a new run."""
    root = _bundle(tmp_path)
    monkeypatch.setenv('NAVRUN_RUN_DIR', str(root))
    assert cli.main(['check', '--source-root', str(SOURCE)]) == 0
    assert not observer.initialized


def test_latest_default_rejects_escape(tmp_path, monkeypatch, observer):
    """Default latest must select a real record directly below the chosen runs root."""
    monkeypatch.delenv('NAVRUN_RUN_DIR', raising=False)
    (tmp_path / 'latest').symlink_to('/outside-record-root')
    assert cli.main(['check', '--runs-root', str(tmp_path),
                     '--source-root', str(SOURCE)]) == 2
    assert not observer.initialized


@pytest.mark.parametrize('strict,expected', [(False, 0), (True, 1)])
def test_live_dispatch_preserves_strict_error_policy(tmp_path, observer, strict, expected):
    """Call the real shared live handler with its ROS boundary replaced, not a stub."""
    from dataclasses import replace

    root = _bundle(tmp_path)
    observer.snapshot = replace(observer.snapshot, tf_age_sec=None)
    assert _run(root, '--live', *(['--strict'] if strict else [])) == expected
    assert observer.initialized and observer.closed and observer.queried
    assert all(handler == 'original' for handler in observer.handlers.values())


def test_live_localization_pending_is_not_success(tmp_path, observer):
    """A bounded absence of localization propagates code three unchanged."""
    root = _bundle(tmp_path)
    observer.localized = False
    assert _run(root, '--live', '--wait-timeout', '0.2') == 3
    assert observer.closed


@pytest.mark.parametrize('mode', ['chassis', 'teleop'])
def test_live_rejects_non_navigation_modes_without_observation(tmp_path, observer, mode):
    """Do not impose localization on manual chassis tools or fake a live acceptance."""
    root = _bundle(tmp_path, mode)
    assert _run(root, '--live') == 2
    assert not observer.initialized


def test_explicit_paths_do_not_override_each_other_silently(tmp_path, observer):
    """Run-directory and runs-root selectors are mutually exclusive at the CLI."""
    with pytest.raises(SystemExit) as error:
        _run(tmp_path, '--runs-root', str(tmp_path))
    assert error.value.code == 2 and not observer.initialized


def test_missing_default_root_does_not_get_created(tmp_path, monkeypatch, observer):
    """A missing default run is an input error, not a reason to create a new record."""
    monkeypatch.delenv('NAVRUN_RUN_DIR', raising=False)
    root = tmp_path / 'missing'
    assert cli.main(['check', '--runs-root', str(root),
                     '--source-root', str(SOURCE)]) == 2
    assert not root.exists() and not observer.initialized


def test_valid_latest_selection(tmp_path, monkeypatch, observer):
    """The conventional latest selector resolves the saved record identity."""
    monkeypatch.delenv('NAVRUN_RUN_DIR', raising=False)
    root = _bundle(tmp_path, 'chassis')
    record = tmp_path / '20260916_010203_chassis_nogit'
    root.rename(record)
    (tmp_path / 'latest').symlink_to(record.name)
    assert cli.main(['check', '--runs-root', str(tmp_path),
                     '--source-root', str(SOURCE)]) == 0


def test_invalid_lifecycle_order_is_not_hidden_by_saved_checks(tmp_path, observer):
    """Use the actual launch reader's lifecycle invariant as part of checking."""
    root = _bundle(tmp_path)
    path = root / 'params/rendered/lifecycle.yaml'
    data = yaml.safe_load(path.read_text())
    data['lifecycle_manager_navigation']['ros__parameters']['node_names'].reverse()
    path.write_text(yaml.safe_dump(data))
    assert _run(root) == 2
    assert not observer.initialized


def test_check_does_not_accept_broken_launch_identity(tmp_path, observer):
    """Robot identity is checked against the frozen profile as well as JSON shape."""
    root = _bundle(tmp_path)
    path = root / 'launch.json'
    data = json.loads(path.read_text())
    data['robot'] = 'another_robot'
    path.write_text(json.dumps(data))
    assert _run(root) == 2


def test_original_source_files_are_not_used_for_run_validation(tmp_path, observer):
    """Removing mutable source inputs does not invalidate an intact frozen record."""
    root = _bundle(tmp_path)
    (tmp_path / 'robot.yaml').unlink()
    (tmp_path / 'mode.yaml').unlink()
    assert _run(root) == 0
    assert not observer.initialized


@pytest.mark.parametrize('signum', [signal.SIGINT, signal.SIGTERM])
def test_live_interruption_preserves_status_and_cleans_observer(
        tmp_path, observer, monkeypatch, signum):
    """Invoke only an in-memory signal callback, never signal a real process."""
    from nav_bringup.navcfg import live_ros

    root = _bundle(tmp_path)
    original = live_ros.RosProbe.spin

    def interrupt(probe, duration):
        original(probe, duration)
        observer.handlers[signum](signum, None)

    monkeypatch.setattr(live_ros.RosProbe, 'spin', interrupt)
    assert _run(root, '--live') == 128 + signum
    assert observer.closed and not observer.queried
    assert all(handler == 'original' for handler in observer.handlers.values())


@pytest.mark.parametrize('option,value', [
    ('--sample-sec', '0'), ('--parameter-timeout', '-1'), ('--wait-timeout', 'nan')])
def test_bad_live_bounds_do_not_initialize_observer(tmp_path, observer, option, value):
    """Malformed online options are rejected by the shared handler before ROS startup."""
    root = _bundle(tmp_path)
    assert _run(root, '--live', option, value) == 2
    assert not observer.initialized


def test_missing_explicit_schema_is_not_silently_ignored(tmp_path, observer):
    """Explicit schema selection takes priority over otherwise valid source discovery."""
    root = _bundle(tmp_path)
    assert _run(root, '--schema', str(tmp_path / 'missing.yaml')) == 2
    assert not observer.initialized


def test_explicit_runs_root_takes_priority_over_environment(tmp_path, monkeypatch, observer):
    """An unrelated environment run must not override an explicit operator selection."""
    root = _bundle(tmp_path, 'chassis')
    record = tmp_path / '20260916_010203_chassis_nogit'
    root.rename(record)
    (tmp_path / 'latest').symlink_to(record.name)
    monkeypatch.setenv('NAVRUN_RUN_DIR', '/missing-environment-run')
    assert cli.main(['check', '--runs-root', str(tmp_path),
                     '--source-root', str(SOURCE)]) == 0
