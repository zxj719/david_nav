"""Tests for dumping the effective parameters of running nodes."""

from pathlib import Path

import pytest
import yaml

from nav_bringup.navrun import cli
from nav_bringup.recorder import events
from nav_bringup.recorder import param_dump

NODE_LIST = '\n'.join([
    '/bt_navigator',
    '/bt_navigator_navigate_to_pose_rclcpp_node',
    '/controller_server',
    '/global_costmap/global_costmap',
    '/transform_listener_impl_5581d3c0',
    '/controller_server',
    'WARNING: a ros2cli notice',
    '',
])


def _fake_runner(node_list, dumps, list_code=0):
    calls = []
    remaining = {node: list(responses) for node, responses in dumps.items()}

    def runner(argv):
        calls.append(list(argv))
        if list(argv[:3]) == ['ros2', 'node', 'list']:
            return list_code, node_list
        responses = remaining.get(argv[-1], [])
        return responses.pop(0) if responses else (1, '')

    return runner, calls


def test_node_file_name():
    """Slashes become double underscores, as in the golden parameter files."""
    assert param_dump.node_file_name('/global_costmap/global_costmap') == (
        'global_costmap__global_costmap.yaml')
    assert param_dump.node_file_name('/bt_navigator') == 'bt_navigator.yaml'
    with pytest.raises(ValueError):
        param_dump.node_file_name('/')


def test_parse_node_list_reports_duplicates():
    """Names are unique and sorted, other lines are ignored, duplicates are reported."""
    names, duplicates = param_dump.parse_node_list(NODE_LIST)

    assert names == [
        '/bt_navigator',
        '/bt_navigator_navigate_to_pose_rclcpp_node',
        '/controller_server',
        '/global_costmap/global_costmap',
        '/transform_listener_impl_5581d3c0',
    ]
    assert duplicates == ['/controller_server']


def test_default_skip_patterns():
    """Helper nodes without parameter services are skipped."""
    patterns = param_dump.DEFAULT_SKIP_PATTERNS

    assert param_dump.should_skip('/bt_navigator_navigate_to_pose_rclcpp_node', patterns)
    assert param_dump.should_skip('/transform_listener_impl_5581d3c0', patterns)
    assert param_dump.should_skip('/_ros2cli_4242', patterns)
    assert not param_dump.should_skip('/controller_server', patterns)


def test_composition_container_is_reported_but_components_are_still_dumped(tmp_path):
    """Humble's container disables parameter services; its loaded nodes do not."""
    runner, calls = _fake_runner('/nav2_container\n/controller_server\n', {
        '/controller_server': [(0, 'controller_server:\n  ros__parameters: {}\n')]})
    report = param_dump.dump_graph(tmp_path, runner=runner, sleep=lambda _: None)
    assert report.ok
    assert report.nodes == ('/controller_server', '/nav2_container')
    assert report.skipped == ('/nav2_container',)
    assert report.dumped == ('/controller_server',)
    assert len(calls) == 2


def test_dump_graph_retries_and_reports(tmp_path):
    """Dumps are retried, empty output fails, helpers are skipped, a report is written."""
    runner, calls = _fake_runner(NODE_LIST, {
        '/bt_navigator': [(0, 'bt_navigator:\n  ros__parameters: {}\n')],
        '/controller_server': [(1, ''), (0, 'controller_server:\n  ros__parameters: {}\n')],
        '/global_costmap/global_costmap': [(0, ''), (124, ''), (0, '  \n')],
    })
    sleeps = []
    output = tmp_path / 'effective'

    report = param_dump.dump_graph(
        output, runner=runner, settings=param_dump.DumpSettings(retry_delay_sec=0.5),
        sleep=sleeps.append)

    assert report.dumped == ('/bt_navigator', '/controller_server')
    assert report.failed == ('/global_costmap/global_costmap',)
    assert report.skipped == (
        '/bt_navigator_navigate_to_pose_rclcpp_node', '/transform_listener_impl_5581d3c0')
    assert report.duplicates == ('/controller_server',)
    assert not report.ok
    assert sleeps == [0.5, 0.5, 0.5]
    assert (output / 'controller_server.yaml').read_text(encoding='utf-8').startswith(
        'controller_server:')
    assert not (output / 'global_costmap__global_costmap.yaml').exists()
    assert (output / 'nodes.txt').read_text(encoding='utf-8').splitlines()[0] == '/bt_navigator'
    saved = yaml.safe_load((output / 'dump_report.yaml').read_text(encoding='utf-8'))
    assert saved['failed'] == ['/global_costmap/global_costmap']
    assert calls[0] == ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '5']
    assert calls[1] == [
        'ros2', 'param', 'dump', '--no-daemon', '--spin-time', '5', '/bt_navigator']


def test_dump_graph_when_listing_fails(tmp_path):
    """A failed node listing yields a failed report and no node files."""
    runner, calls = _fake_runner('', {}, list_code=1)

    report = param_dump.dump_graph(tmp_path, runner=runner, sleep=lambda seconds: None)

    assert report.list_failed
    assert not report.ok
    assert report.nodes == ()
    assert len(calls) == 1
    assert not (tmp_path / 'nodes.txt').exists()
    saved = yaml.safe_load((tmp_path / 'dump_report.yaml').read_text(encoding='utf-8'))
    assert saved['list_failed'] is True


def test_cli_dump_params_into_run_directory(tmp_path, monkeypatch, capsys):
    """Without --output-dir the dump goes to params/effective of the current run."""
    run_path = tmp_path / '20260915_024941_tour_b19f1d3'
    monkeypatch.setenv('NAVRUN_RUN_DIR', str(run_path))
    captured = {}

    def fake_dump_graph(output_dir, settings):
        captured['output_dir'] = output_dir
        captured['settings'] = settings
        Path(output_dir).mkdir(parents=True)
        return param_dump.DumpReport(
            nodes=('/a', '/b'), dumped=('/a',), skipped=(), failed=('/b',))

    monkeypatch.setattr(param_dump, 'dump_graph', fake_dump_graph)

    code = cli.main(['dump-params', '--attempts', '2', '--timeout', '12'])

    assert code == 1
    assert captured['output_dir'] == run_path / 'params' / 'effective'
    assert (captured['settings'].attempts, captured['settings'].timeout_sec) == (2, 12.0)
    recorded, _ = events.read_events(run_path / 'events.jsonl')
    assert [(event.event_type, event.severity) for event in recorded] == [
        ('params_dumped', 'warn')]
    assert recorded[0].data['failed'] == ['/b']
    assert 'dumped 1 of 2 nodes' in capsys.readouterr().err


def test_cli_dump_params_to_output_directory(tmp_path, monkeypatch):
    """With --output-dir no run directory or event is needed."""
    monkeypatch.delenv('NAVRUN_RUN_DIR', raising=False)
    monkeypatch.setattr(
        param_dump, 'dump_graph',
        lambda output_dir, settings: param_dump.DumpReport(('/a',), ('/a',), (), ()))

    assert cli.main(['dump-params', '--output-dir', str(tmp_path / 'dumps')]) == 0
    assert not (tmp_path / 'events.jsonl').exists()


def test_cli_dump_params_needs_a_destination(monkeypatch):
    """Without a run directory or output directory the command is rejected."""
    monkeypatch.delenv('NAVRUN_RUN_DIR', raising=False)

    with pytest.raises(SystemExit) as raised:
        cli.main(['dump-params'])

    assert raised.value.code == 2


def test_failed_listing_reports_failure_without_removing_old_artifacts(tmp_path):
    """失败报告优先于文件存在性；不把旧参数文件当作本次成功."""
    (tmp_path / 'nodes.txt').write_text('/previous\n', encoding='utf-8')
    (tmp_path / 'previous.yaml').write_text('previous data\n', encoding='utf-8')
    runner, calls = _fake_runner('', {}, list_code=1)
    report = param_dump.dump_graph(tmp_path, runner=runner)
    assert not report.ok and report.list_failed
    assert report.dumped == () and len(calls) == 1
    assert (tmp_path / 'nodes.txt').read_text() == '/previous\n'
    assert (tmp_path / 'previous.yaml').read_text() == 'previous data\n'
    assert yaml.safe_load((tmp_path / 'dump_report.yaml').read_text()) == report.to_dict()
