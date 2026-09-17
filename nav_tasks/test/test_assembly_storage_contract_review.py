"""保护运行时及存储说明，并用内存回调核验构造回滚的失败保留."""

import ast
import asyncio
from pathlib import Path
import re

import pytest

from nav_tasks.app.runtime import _Rollback


@pytest.mark.parametrize('path, qualified_name', [
    ('app/__init__.py', '<module>'),
    ('app/dock_config.py', '<module>'),
    ('app/dock_parameters.py', '<module>'),
    ('app/logging.py', '<module>'),
    ('app/logging.py', 'RetryAudit'),
    ('app/logging.py', 'RetryAudit.__init__'),
    ('app/logging.py', 'RetryAudit.__call__'),
    ('app/loop_closure.py', '<module>'),
    ('app/loop_closure.py', 'watch_loop_closure'),
    ('app/runtime.py', '<module>'),
    ('app/runtime.py', 'TaskRuntime'),
    ('app/runtime.py', 'TaskRuntime.__init__'),
    ('app/runtime.py', 'TaskRuntime.start_loop_closure'),
    ('app/runtime.py', 'TaskRuntime.stop_loop_closure'),
    ('app/runtime.py', 'TaskRuntime.close'),
    ('app/runtime.py', 'assemble_runtime'),
    ('nav_site_data/__init__.py', '<module>'),
    ('nav_site_data/named_poses_importer.py', '<module>'),
    ('nav_site_data/named_poses_importer.py', 'import_named_poses'),
    ('nav_site_data/yaml_poi_repository.py', '<module>'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository.__init__'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository.load'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository.get'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository.upsert'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository.rename_many'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository.prune_session'),
    ('nav_site_data/yaml_poi_repository.py', 'YamlPoiRepository.reproject_session'),
    ('app/runtime.py', '_Rollback'),
    ('app/runtime.py', '_Rollback.__aexit__'),
    ('app/runtime.py', '_Rollback.track'),
    ('app/runtime.py', '_Rollback.cleanup'),
])
def test_reviewed_runtime_and_storage_summary(path, qualified_name):
    """按限定名保护已审中文摘要，不代替真实资源或文件故障验收.

    Args:
        path: 包内模块相对路径。
        qualified_name: 模块、类或方法的限定名。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_tasks'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    if qualified_name != '<module>':
        for name in qualified_name.split('.'):
            matches = [child for child in node.body
                       if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                       and child.name == name]
            assert len(matches) == 1
            node = matches[0]
    doc = ast.get_docstring(node) or ''
    assert doc and re.search('[\\u4e00-\\u9fff]', doc.splitlines()[0]), qualified_name


@pytest.mark.parametrize('error_type', [RuntimeError, asyncio.CancelledError])
def test_rollback_attempts_all_callbacks_and_retries_only_failure(error_type):
    """回滚中的普通异常或取消不跳过剩余资源，成功回调不会重复执行.

    Args:
        error_type: 中间异步回调首次抛出的异常类型。
    """
    async def scenario():
        owner, calls = _Rollback(), []
        failed = False
        error = error_type('fixture')

        async def middle():
            nonlocal failed
            calls.append('middle')
            if not failed:
                failed = True
                raise error

        owner.track(lambda: calls.append('first'))
        owner.track(middle)
        owner.track(lambda: calls.append('last'))
        with pytest.raises(error_type) as caught:
            await owner.cleanup()
        assert caught.value is error
        assert calls == ['last', 'middle', 'first']
        assert owner._callbacks == [middle]
        await owner.cleanup()
        assert calls == ['last', 'middle', 'first', 'middle']
        assert owner._callbacks == []
        await owner.cleanup()
        assert calls == ['last', 'middle', 'first', 'middle']

    asyncio.run(scenario())
