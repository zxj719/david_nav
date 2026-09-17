"""参数转储文件名冲突回归，仅使用假命令和临时目录."""

import pytest
import yaml

from nav_bringup.navrun import cli
from nav_bringup.recorder import events, param_dump, run_dir


def _runner(names):
    """返回列表与参数假输出，并记录实际请求的节点."""
    dumped = []

    def run(command):
        if command[:3] == ['ros2', 'node', 'list']:
            return 0, '\n'.join(names)
        assert command[:3] == ['ros2', 'param', 'dump']
        dumped.append(command[-1])
        return 0, yaml.safe_dump({command[-1]: {'ros__parameters': {'value': 1}}})

    return run, dumped


@pytest.mark.parametrize('existing', [False, True])
def test_colliding_nodes_are_all_failed_without_overwriting(tmp_path, existing):
    """拒绝所有同名目标，不任意选择胜者，也不覆盖先前记录."""
    runner, requested = _runner(['/a/b', '/a__b', '/safe'])
    collision = tmp_path / 'a__b.yaml'
    if existing:
        collision.write_text('previous dump\n', encoding='utf-8')
    report = param_dump.dump_graph(tmp_path, runner=runner)
    assert report.failed == ('/a/b', '/a__b')
    assert report.dumped == ('/safe',) and not report.ok
    assert requested == ['/safe']
    assert collision.exists() is existing
    if existing:
        assert collision.read_text() == 'previous dump\n'
    saved = yaml.safe_load((tmp_path / param_dump.REPORT_FILE_NAME).read_text())
    assert saved == report.to_dict()
    assert (tmp_path / 'nodes.txt').read_text().splitlines() == ['/a/b', '/a__b', '/safe']


def test_three_way_collision_fails_every_member(tmp_path):
    """多级命名空间映射冲突不能仅拒绝第二个节点."""
    names = ['/a/b__c', '/a__b/c', '/a/b/c']
    runner, requested = _runner(names)
    report = param_dump.dump_graph(tmp_path, runner=runner)
    assert report.failed == tuple(sorted(names))
    assert report.dumped == () and not report.ok
    assert requested == []
    assert not (tmp_path / 'a__b__c.yaml').exists()


def test_report_file_is_reserved_for_metadata(tmp_path):
    """节点不能被报告为转储成功后再被同名汇总覆盖."""
    runner, requested = _runner(['/dump_report', '/safe'])
    report = param_dump.dump_graph(tmp_path, runner=runner)
    assert report.failed == ('/dump_report',)
    assert report.dumped == ('/safe',) and not report.ok
    assert requested == ['/safe']
    assert yaml.safe_load((tmp_path / 'dump_report.yaml').read_text()) == report.to_dict()


def test_skipped_node_does_not_reserve_an_output_name(tmp_path):
    """显式跳过的节点没有写入权，不阻止唯一实际输出."""
    runner, requested = _runner(['/a/b', '/a__b'])
    settings = param_dump.DumpSettings(skip_patterns=('/a/b',))
    report = param_dump.dump_graph(tmp_path, runner=runner, settings=settings)
    assert report.ok and report.skipped == ('/a/b',)
    assert report.dumped == ('/a__b',) and requested == ['/a__b']
    assert '/a__b' in yaml.safe_load((tmp_path / 'a__b.yaml').read_text())


def test_duplicate_discovery_is_not_a_filename_collision(tmp_path):
    """重复发现同名节点保留既有重复报告语义，不把它当成两个文件."""
    runner, requested = _runner(['/safe', '/safe'])
    report = param_dump.dump_graph(tmp_path, runner=runner)
    assert report.ok and report.duplicates == ('/safe',)
    assert report.dumped == ('/safe',) and requested == ['/safe']


def test_cli_propagates_collision_as_failure_and_warning(tmp_path, monkeypatch, capsys):
    """沿用现有 failed 字段使 CLI 返回非零并在运行事件中告警."""
    run = run_dir.create_run_directory(tmp_path, '20260917_120000_tour_nogit')
    runner, requested = _runner(['/a/b', '/a__b'])
    monkeypatch.setattr(param_dump, 'default_runner', lambda settings: runner)
    code = cli.main(['dump-params', '--run-dir', str(run.path)])
    assert code == 1
    assert requested == []
    recorded, skipped = events.read_events(run.events)
    assert skipped == 0 and len(recorded) == 1
    assert recorded[0].event_type == 'params_dumped'
    assert recorded[0].severity == 'warn'
    assert recorded[0].data['failed'] == ['/a/b', '/a__b']
    assert 'failed 2' in capsys.readouterr().err
