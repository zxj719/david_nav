"""Exercise the actual optional CLI and coordinator with no goals or velocity publishers."""

import json

import pytest

from nav_bringup.navrun.cli import main


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_actual_cli_startup_only_cannot_report_closed_loop(tmp_path, capsys, mode):
    """Real lifecycle/map/TF/odom/WS/cleanup must pass, but complete must remain false."""
    root = tmp_path / 'run'
    status = main(['selftest', '--mode', mode, '--robot', 'example_diffdrive',
                   '--output-dir', str(root), '--startup-only', '--timeout', '90'])
    output = capsys.readouterr()
    assert status == 0, output.err + output.out
    printed = json.loads(output.out)
    saved = json.loads((root / 'result.json').read_text())
    assert saved['passed'] and saved['executed'] and saved['cleanup_complete']
    assert printed['complete'] is saved['complete'] is False
    assert saved['kind'] == 'startup_only' and saved['measurements'] == []
    assert not saved['errors']
    rows = [json.loads(line) for line in (root / 'observations.jsonl').read_text().splitlines()]
    assert not any(row['record'].get('event_type') in ('on_arrived', 'on_error', 'task_admitted')
                   for row in rows)
