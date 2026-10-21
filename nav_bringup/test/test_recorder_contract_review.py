"""记录器的纯文件/假端口边界，不清理实际运行记录."""

import datetime
import math

import pytest

from nav_bringup.recorder import events, manifest, resources, retention, run_dir


def test_git_info_does_not_promote_later_query_errors_to_failure():
    """根查询成功后其它退出码不单独报告，空状态不构成干净证明."""
    responses = {
        ('rev-parse', '--show-toplevel'): (0, '/fixture\n'),
        ('rev-parse', '--abbrev-ref', 'HEAD'): (128, 'partial-branch\n'),
        ('rev-parse', 'HEAD'): (128, ''),
        ('status', '--porcelain'): (128, ''),
    }
    info = manifest.collect_git_info(
        '/fixture', lambda argv: responses[tuple(argv[3:])])
    assert info == manifest.GitInfo('/fixture', 'partial-branch', None, False, 0)


def test_code_diff_records_untracked_name_not_contents(tmp_path):
    """未跟踪文件只有状态行，不能据此声称完整重建工作树."""
    responses = {('status', '--porcelain'): (0, '?? fresh.py\n'),
                 ('diff', 'HEAD'): (0, '')}
    output = tmp_path / 'code.diff'
    assert manifest.write_code_diff(
        '/fixture', output, lambda argv: responses[tuple(argv[3:])])
    assert output.read_text() == '# git status --porcelain\n# ?? fresh.py\n\n'


def test_appended_event_keeps_nested_alias_but_file_has_serialized_value(tmp_path):
    """外层字典复制不复制嵌套值；后续修改不会改已写入的行."""
    nested = {'count': 1}
    log = events.EventLog(tmp_path / 'events.jsonl', clock=lambda: 1.0)
    event = log.append('fixture', 'sample', data={'nested': nested})
    nested['count'] = 2
    assert event.data['nested']['count'] == 2
    recorded, skipped = events.read_events(log.path)
    assert recorded[0].data['nested']['count'] == 1
    assert skipped == 0


def test_local_event_codec_preserves_nonfinite_stamp():
    """本地编码沿用 Python JSON 的非有限值扩展，不是严格线协议校验."""
    encoded = events.Event(float('nan'), 'fixture', 'sample').to_json()
    assert '"stamp": NaN' in encoded
    assert math.isnan(events.Event.from_json(encoded).stamp)


def test_resource_summary_groups_rows_not_simultaneous_cpu_totals(tmp_path):
    """相同名字不同 PID 合并为样本均值，不计算同时刻总 CPU."""
    path = tmp_path / 'resources.csv'
    path.write_text(
        'stamp,pid,name,cpu_percent,rss_mb,num_threads\n'
        '1,11,worker,20,2,1\n'
        '1,12,worker,60,3,2\n', encoding='utf-8')
    summary = resources.summarize_resources(path)
    assert summary.system is None
    assert summary.processes == [resources.ProcessStats('worker', 2, 40.0, 60.0, 3.0)]


@pytest.mark.parametrize('extra_seconds, expired', [(0, False), (1, True)])
def test_bag_age_boundary_uses_strictly_greater_without_deleting(tmp_path, extra_seconds, expired):
    """刚满七天不列入删除，超过一秒才计划删除，规划不修改记录."""
    started = datetime.datetime(2026, 9, 17, 1, 2, 3)
    run = run_dir.create_run_directory(tmp_path, run_dir.make_run_id('tour', None, started))
    run.manifest.write_text(
        f'schema_version: 1\nrun_id: {run.run_id}\nended_at: "2026-09-17T02:00:00"\n',
        encoding='utf-8')
    run.summary.write_text('fixture summary', encoding='utf-8')
    payload = run.bag / 'part'
    payload.write_bytes(b'fixture')
    now = started + datetime.timedelta(days=7, seconds=extra_seconds)
    plan = retention.plan_gc([run], retention.RetentionPolicy(), now)
    assert plan.delete_bags == ([run.bag] if expired else [])
    assert not plan.prune_runs and not plan.skipped
    assert payload.read_bytes() == b'fixture'
