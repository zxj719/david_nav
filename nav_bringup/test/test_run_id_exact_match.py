"""运行标识必须完整匹配，创建与只读记录入口采用同一边界."""

import datetime

import pytest

from nav_bringup.navrun.record_reader import open_record
from nav_bringup.recorder import run_dir

START = datetime.datetime(2026, 9, 17, 1, 2, 3)
RUN = '20260917_010203_tour_abcdef0'


def test_mode_rejects_trailing_newline():
    """正则末尾锚点不能把最后一个换行当作合法模式的一部分."""
    with pytest.raises(ValueError, match='invalid run mode'):
        run_dir.make_run_id('tour\n', None, START)


def test_invalid_run_id_rejected_before_creating_root(tmp_path):
    """拒绝带换行的目录名，并且失败前不创建根目录或 latest."""
    root = tmp_path / 'not-created'
    with pytest.raises(ValueError, match='not a run id'):
        run_dir.create_run_directory(root, RUN + '\n')
    assert not root.exists()


def test_started_at_rejects_nonexact_id(tmp_path):
    """日期提取不能使其它接口拒绝的 ID 看起来合法."""
    with pytest.raises(ValueError, match='not a run id'):
        _ = run_dir.RunDirectory(tmp_path, RUN + '\n').started_at


def test_listing_ignores_nonexact_id_without_modifying_it(tmp_path):
    """列举忽略异常目录，保留原目录供人工判断，不自动修复或删除."""
    run_dir.create_run_directory(tmp_path, RUN)
    invalid = tmp_path / (RUN + '\n')
    invalid.mkdir()
    assert [item.run_id for item in run_dir.list_runs(tmp_path)] == [RUN]
    assert invalid.is_dir()


@pytest.mark.parametrize('mode', ['tour', 'precise'])
def test_valid_ids_round_trip_through_creation_listing_and_reading(tmp_path, mode):
    """正常模式保持命名格式，并可被严格的记录读取器接收."""
    name = run_dir.make_run_id(mode, 'abcdef0123456789', START)
    run = run_dir.create_run_directory(tmp_path, name)
    assert run.started_at == START
    assert [item.run_id for item in run_dir.list_runs(tmp_path)] == [name]
    with open_record(tmp_path, 'latest') as reader:
        assert reader.run_id == name
        assert reader.read('manifest.yaml') is None
