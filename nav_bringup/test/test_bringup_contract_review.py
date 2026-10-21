"""锁定已审模块说明与诊断/记录读取边界，不创建 ROS 或操纵进程."""

import pytest

from nav_bringup.doctor import live
from nav_bringup.doctor.checks import Status
from nav_bringup.navrun import record_reader
from nav_bringup.recorder.safe_retention import Snapshot

RUN = '20260917_010203_tour_abcdef0'


def test_command_cache_retries_exceptions_but_caches_nonzero_results():
    """异常不入缓存；已返回的非零结果按 argv 固定，不自动重试."""
    calls = []

    def runner(argv):
        calls.append(tuple(argv))
        if len(calls) == 1:
            raise OSError('fixture failure')
        return 7, 'not available'

    cached = live.caching_runner(runner)
    with pytest.raises(OSError, match='fixture failure'):
        cached(['probe'])
    assert cached(['probe']) == (7, 'not available')
    assert cached(('probe',)) == (7, 'not available')
    assert calls == [('probe',), ('probe',)]


def test_daemon_comparison_does_not_detect_daemon_only_nodes():
    """原规则只报告直接发现比 daemon 多的节点，不证明两份图完全相等."""
    responses = {
        ('ros2', 'daemon', 'status'): (0, 'The daemon is running'),
        ('ros2', 'node', 'list'): (0, '/seen\n/stale\n'),
        ('ros2', 'node', 'list', '--no-daemon', '--spin-time', '3'): (0, '/seen\n'),
    }
    context = live.LiveContext(runner=lambda argv: responses[tuple(argv)])
    assert live.check_cli_daemon(context).status is Status.OK


@pytest.mark.parametrize('gid, expected', [
    ('01.0f.xx.yy', 'xx.yy'), ('01.0F.aa.bb', None), ('01.0f.aa', None),
])
def test_gid_host_is_only_a_prefix_heuristic(gid, expected):
    """既有解析不验证十六进制或完整 GID，也不归一化大小写."""
    assert live.gid_host(gid) == expected


def test_missing_record_read_does_not_spend_shared_byte_budget(tmp_path, monkeypatch):
    """缺失文件返回 None；成功重复读取仍消耗同一个 reader 的累计预算."""
    run = tmp_path / RUN
    run.mkdir()
    (run / 'sample').write_bytes(b'xx')
    monkeypatch.setattr(record_reader, 'MAX_TOTAL_BYTES', 3)
    with record_reader.open_record(tmp_path, RUN) as reader:
        assert reader.read('missing') is None
        assert reader.read('sample') == 'xx'
        with pytest.raises(ValueError, match='byte budget'):
            reader.read('sample')


def test_invalid_utf8_read_spends_budget_before_decode(tmp_path, monkeypatch):
    """解码失败仍扣除已经读取的字节，不回滚 reader 预算."""
    run = tmp_path / RUN
    run.mkdir()
    (run / 'invalid').write_bytes(b'\xff')
    (run / 'valid').write_bytes(b'x')
    monkeypatch.setattr(record_reader, 'MAX_TOTAL_BYTES', 1)
    with record_reader.open_record(tmp_path, RUN) as reader:
        with pytest.raises(UnicodeError):
            reader.read('invalid')
        with pytest.raises(ValueError, match='byte budget'):
            reader.read('valid')


def test_retention_snapshot_freezes_fields_not_entries(tmp_path):
    """冻结数据类仍持有可变字典；派生属性读当前字典而非不可变副本."""
    sample = tmp_path / 'payload'
    sample.write_bytes(b'abc')
    entries = {}
    snapshot = Snapshot((1, 2), entries, False)
    assert snapshot.size == 0 and snapshot.pruned and not snapshot.has_bag
    entries[('bag', 'part')] = sample.stat()
    assert snapshot.size == 3 and not snapshot.pruned and snapshot.has_bag
    assert not snapshot.complete
