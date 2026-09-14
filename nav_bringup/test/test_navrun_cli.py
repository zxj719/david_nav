"""Tests for the ``navrun`` command line."""

from pathlib import Path
import sys

import pytest

from nav_bringup.navrun import cli


def test_parse_file_arguments():
    """NAME=PATH pairs become a mapping of expanded paths."""
    files = cli.parse_file_arguments(['map_yaml=/maps/a.yaml', 'pois=~/pois.json'])

    assert files == {'map_yaml': Path('/maps/a.yaml'), 'pois': Path('~/pois.json').expanduser()}


@pytest.mark.parametrize('value', ['no_separator', '=path', 'name='])
def test_parse_file_arguments_rejects_malformed_values(value):
    """Missing names, paths or separators are errors."""
    with pytest.raises(ValueError):
        cli.parse_file_arguments([value])


def test_wrap_requires_a_command(tmp_path):
    """Running wrap without a command is a usage error."""
    with pytest.raises(SystemExit) as raised:
        cli.main(['wrap', '--mode', 'tour', '--runs-root', str(tmp_path)])

    assert raised.value.code == 2


def test_wrap_rejects_invalid_mode(tmp_path):
    """An invalid mode is reported as a usage error before anything is created."""
    with pytest.raises(SystemExit):
        cli.main(['wrap', '--mode', 'Tour!', '--runs-root', str(tmp_path / 'runs'),
                  '--repo', str(tmp_path), '--', sys.executable, '-c', 'pass'])

    assert not (tmp_path / 'runs').exists()


def test_wrap_end_to_end_returns_command_status(tmp_path, capfd):
    """The command's exit status is returned and the run record is announced."""
    code = cli.main([
        'wrap', '--mode', 'precise', '--runs-root', str(tmp_path / 'runs'), '--repo',
        str(tmp_path), '--no-gc', '--file', f'self={__file__}', '--',
        sys.executable, '-c', 'import sys; print("inside"); sys.exit(4)',
    ])
    captured = capfd.readouterr()

    assert code == 4
    assert 'inside' in captured.out
    assert 'navrun: run record' in captured.err
    assert '(failed, exit 4)' in captured.err
