"""Tests for the navcfg command line."""

import yaml

from nav_bringup.navcfg import cli

BASE = {'node': {'ros__parameters': {'frame': '<bind:frames.map>', 'rate': 5.0}}}
BINDINGS = {'schema_version': 1, 'bindings': {'frames.map': ['node/ros__parameters/frame']}}


def _write(path, content):
    path.write_text(yaml.safe_dump(content), encoding='utf-8')
    return str(path)


def _render_args(tmp_path, profile, output):
    return [
        'render',
        '--base', _write(tmp_path / 'base.yaml', BASE),
        '--bindings', _write(tmp_path / 'bindings.yaml', BINDINGS),
        '--profile', _write(tmp_path / 'profile.yaml', profile),
        '--output', str(output),
    ]


def test_render_writes_the_parameter_file(tmp_path, capsys):
    """Render writes the rendered tree and names the written file."""
    output = tmp_path / 'rendered.yaml'

    status = cli.main(_render_args(tmp_path, {'frames': {'map': 'world'}}, output))

    assert status == 0
    assert yaml.safe_load(output.read_text(encoding='utf-8')) == {
        'node': {'ros__parameters': {'frame': 'world', 'rate': 5.0}}}
    assert str(output) in capsys.readouterr().out


def test_render_applies_the_mode_overlay(tmp_path):
    """Profile values and parameter overrides of the mode are applied."""
    output = tmp_path / 'rendered.yaml'
    mode = {
        'schema_version': 1,
        'mode': 'bench',
        'profile': {'frames': {'map': 'bench_map'}},
        'nav2_overrides': {'node/ros__parameters/rate': 1.0},
    }
    args = _render_args(tmp_path, {'frames': {'map': 'world'}}, output)

    status = cli.main(args + ['--mode', _write(tmp_path / 'mode.yaml', mode)])

    assert status == 0
    assert yaml.safe_load(output.read_text(encoding='utf-8')) == {
        'node': {'ros__parameters': {'frame': 'bench_map', 'rate': 1.0}}}


def test_render_reports_invalid_inputs_without_writing(tmp_path, capsys):
    """A profile without a bound key fails with status 2, names the key and writes nothing."""
    output = tmp_path / 'rendered.yaml'

    status = cli.main(_render_args(tmp_path, {'frames': {}}, output))

    assert status == 2
    assert 'frames.map' in capsys.readouterr().err
    assert not output.exists()


def test_render_reports_missing_files(tmp_path, capsys):
    """A missing input file fails with status 2."""
    output = tmp_path / 'rendered.yaml'
    args = _render_args(tmp_path, {'frames': {'map': 'world'}}, output)

    status = cli.main(args + ['--mode', str(tmp_path / 'missing.yaml')])

    assert status == 2
    assert 'missing.yaml' in capsys.readouterr().err
