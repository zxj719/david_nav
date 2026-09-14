"""Tests for the run directory layout."""

import datetime

import pytest

from nav_bringup.recorder import run_dir

START = datetime.datetime(2026, 9, 14, 21, 13, 42)
SHA = 'fe1ac78d0c4b9a2e5f1d3c7b8a6e4f2d1c0b9a87'


def test_make_run_id_uses_timestamp_mode_and_short_sha():
    """Run ids sort chronologically and carry the mode and commit."""
    assert run_dir.make_run_id('tour', SHA, START) == '20260914_211342_tour_fe1ac78'


def test_make_run_id_without_git_uses_placeholder():
    """Runs outside git are still named consistently."""
    assert run_dir.make_run_id('precise', None, START) == '20260914_211342_precise_nogit'


@pytest.mark.parametrize('mode', ['Tour', 'tour-mode', '', 'tour/x'])
def test_make_run_id_rejects_invalid_mode(mode):
    """Modes are restricted to lowercase letters, digits and underscores."""
    with pytest.raises(ValueError):
        run_dir.make_run_id(mode, SHA, START)


def test_make_run_id_rejects_invalid_sha():
    """A malformed commit hash is an error rather than a strange run id."""
    with pytest.raises(ValueError):
        run_dir.make_run_id('tour', 'not-a-sha', START)


def test_create_run_directory_builds_tree_and_latest_link(tmp_path):
    """The run tree exists and latest points at the new run."""
    run = run_dir.create_run_directory(tmp_path / 'runs', '20260914_211342_tour_fe1ac78')

    for directory in (run.logs, run.metrics, run.bag, run.params_inputs,
                      run.params_rendered, run.params_effective):
        assert directory.is_dir()
    latest = tmp_path / 'runs' / run_dir.LATEST_LINK_NAME
    assert latest.is_symlink()
    assert latest.resolve() == run.path.resolve()


def test_create_run_directory_refuses_existing_run(tmp_path):
    """A run directory is never reused."""
    run_dir.create_run_directory(tmp_path, '20260914_211342_tour_fe1ac78')

    with pytest.raises(FileExistsError):
        run_dir.create_run_directory(tmp_path, '20260914_211342_tour_fe1ac78')


def test_create_run_directory_rejects_invalid_run_id(tmp_path):
    """Only valid run ids become run directories."""
    with pytest.raises(ValueError):
        run_dir.create_run_directory(tmp_path, '../escape')


def test_latest_link_moves_to_newest_run(tmp_path):
    """Creating a second run repoints latest."""
    run_dir.create_run_directory(tmp_path, '20260914_211342_tour_fe1ac78')
    second = run_dir.create_run_directory(tmp_path, '20260914_221342_precise_fe1ac78')

    assert (tmp_path / 'latest').resolve() == second.path.resolve()


def test_list_runs_ignores_unrelated_entries_and_sorts(tmp_path):
    """Only run directories are listed, oldest first."""
    run_dir.create_run_directory(tmp_path, '20260914_221342_precise_fe1ac78')
    run_dir.create_run_directory(tmp_path, '20260914_211342_tour_fe1ac78')
    (tmp_path / 'notes').mkdir()
    (tmp_path / 'readme.txt').write_text('x', encoding='utf-8')

    names = [run.run_id for run in run_dir.list_runs(tmp_path)]

    assert names == ['20260914_211342_tour_fe1ac78', '20260914_221342_precise_fe1ac78']


def test_list_runs_on_missing_root_is_empty(tmp_path):
    """A root that does not exist yet has no runs."""
    assert run_dir.list_runs(tmp_path / 'missing') == []


def test_started_at_parses_run_id():
    """The start time is recoverable from the run id alone."""
    run = run_dir.RunDirectory(root=None, run_id='20260914_211342_tour_fe1ac78')

    assert run.started_at == START


def test_started_at_rejects_foreign_names():
    """Names that are not run ids have no start time."""
    with pytest.raises(ValueError):
        _ = run_dir.RunDirectory(root=None, run_id='latest').started_at


def test_default_runs_root_honours_environment(monkeypatch, tmp_path):
    """NAV_RUNS_ROOT overrides the home directory default."""
    monkeypatch.setenv('NAV_RUNS_ROOT', str(tmp_path))

    assert run_dir.default_runs_root() == tmp_path
