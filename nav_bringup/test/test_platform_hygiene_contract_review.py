"""边界：只用私有目录与无害命令，不改功耗模式、不清理实际 /dev/shm."""

import inspect
from pathlib import Path

import pytest

from nav_bringup.navrun import dds_hygiene, platform


def test_missing_program_is_a_bounded_result_not_an_exception():
    """真实执行一条不存在的命令：返回 127，不抛异常、不重试."""
    code, detail = platform.run_platform_command(
        ('/nonexistent/nav_bringup_platform_probe',), 5.0)
    assert code == 127 and detail


def test_real_command_reads_end_of_file_instead_of_waiting_for_input():
    """标准输入接空设备，需要输入的命令立刻结束，不会挂住启动流程."""
    code, detail = platform.run_platform_command(('/bin/cat',), 5.0)
    assert code == 0 and detail == ''


def test_dry_run_switch_is_validated_before_any_planning():
    """dry-run 开关非法时在规划前就拒绝，执行器不会被调用."""
    def forbidden(argv, timeout):
        pytest.fail('runner must not be called')

    with pytest.raises(ValueError):
        platform.configure_platform({'platform': {'jetson': True, 'maxn': True}},
                                    forbidden, dry_run='yes')


def test_cleanup_has_no_default_directory_or_observer():
    """目录与观察器必须显式传入，导入本模块不可能清到任何默认根."""
    plan_parameters = inspect.signature(dds_hygiene.plan_hygiene).parameters
    cleanup_parameters = inspect.signature(dds_hygiene.cleanup_hygiene).parameters
    assert plan_parameters['directory'].default is inspect.Parameter.empty
    assert plan_parameters['view'].default is inspect.Parameter.empty
    assert cleanup_parameters['plan'].default is inspect.Parameter.empty
    assert cleanup_parameters['observe'].default is inspect.Parameter.empty


def test_relative_directory_is_refused_before_listing(tmp_path, monkeypatch):
    """相对路径一律拒绝，拒绝发生在读取目录内容之前."""
    monkeypatch.chdir(tmp_path)
    (tmp_path / 'fastrtps_port7415').write_bytes(b'')
    view = dds_hygiene.MappingView(frozenset(), True)
    with pytest.raises(ValueError):
        dds_hygiene.plan_hygiene(Path('.'), view)
    assert (tmp_path / 'fastrtps_port7415').exists()


def test_incomplete_evidence_keeps_every_file_and_names_a_reason(tmp_path):
    """观察不完整时不产生候选，并给出保留原因，绝不按“没看见”删除."""
    (tmp_path / 'fastrtps_port7415').write_bytes(b'')
    (tmp_path / 'fastrtps_port7415_el').write_bytes(b'')
    view = dds_hygiene.MappingView(frozenset(), False, ('maps unreadable',))
    plan = dds_hygiene.plan_hygiene(tmp_path, view)
    assert plan.candidates == ()
    assert {name for name, _ in plan.retained} == {'fastrtps_port7415', 'fastrtps_port7415_el'}
    assert all(reason for _, reason in plan.retained)
    assert (tmp_path / 'fastrtps_port7415').exists()
