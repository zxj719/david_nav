"""核验启动所有权说明及既有退出边界，不创建ROS或进程."""

import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from nav_bringup.navrun import startup_lifecycle, zero_command


@pytest.mark.parametrize('path, qualified_name', [
    ('navrun/startup_lifecycle.py', '<module>'),
    ('navrun/startup_lifecycle.py', 'StartupGate'),
    ('navrun/startup_lifecycle.py', 'StartupCommands'),
    ('navrun/startup_lifecycle.py', 'LifecycleResult'),
    ('navrun/startup_lifecycle.py', 'supervise'),
    ('navrun/startup_lifecycle.py', 'validate_commands'),
    ('navrun/zero_command.py', '<module>'),
    ('navrun/zero_command.py', 'main'),
    ('navrun/startup_lifecycle.py', '_Finished'),
    ('navrun/startup_lifecycle.py', '_Finished.__init__'),
    ('navrun/startup_lifecycle.py', '_Supervisor'),
    ('navrun/startup_lifecycle.py', '_Supervisor.__init__'),
    ('navrun/startup_lifecycle.py', '_Supervisor.emit'),
    ('navrun/startup_lifecycle.py', '_Supervisor.start'),
    ('navrun/startup_lifecycle.py', '_Supervisor.check'),
    ('navrun/startup_lifecycle.py', '_Supervisor.gate'),
    ('navrun/startup_lifecycle.py', '_Supervisor.execute'),
    ('navrun/startup_lifecycle.py', '_Supervisor.close'),
    ('navrun/startup_lifecycle.py', '_Supervisor.zero'),
    ('navrun/startup_lifecycle.py', '_Supervisor.cleanup'),
    ('navrun/zero_command.py', '_RosZeros'),
    ('navrun/zero_command.py', '_RosZeros.__init__'),
    ('navrun/zero_command.py', '_RosZeros.__enter__'),
    ('navrun/zero_command.py', '_RosZeros.__exit__'),
    ('navrun/zero_command.py', '_RosZeros.spin'),
    ('navrun/zero_command.py', '_RosZeros.publish'),
    ('navrun/zero_command.py', '_RosZeros.receiver_count'),
])
def test_reviewed_startup_contract(path, qualified_name):
    """保护已审说明，语言断言不代替实际语义核对.

    Args:
        path: 包内模块相对路径。
        qualified_name: 模块、类或方法的限定名称。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_bringup'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    if qualified_name != '<module>':
        for name in qualified_name.split('.'):
            matches = [child for child in node.body
                       if isinstance(child, (ast.ClassDef, ast.FunctionDef))
                       and child.name == name]
            assert len(matches) == 1
            node = matches[0]
    doc = ast.get_docstring(node) or ''
    assert doc and any('一' <= char <= '鿿' for char in doc.splitlines()[0])


def test_zero_help_is_frozen_before_ros_initialization(monkeypatch, capsys):
    """模块说明转为中文时仍保留原命令行帮助，不创建传输.

    Args:
        monkeypatch: 固定终端宽度、程序名并禁止传输构造。
        capsys: 捕获完整帮助文本。
    """
    monkeypatch.setenv('COLUMNS', '80')
    monkeypatch.setattr(zero_command.sys, 'argv', ['zero_command.py'])

    def forbidden(*args):
        pytest.fail('help must not initialize ROS')

    monkeypatch.setattr(zero_command, '_RosZeros', forbidden)
    with pytest.raises(SystemExit) as error:
        zero_command.main(['--help'])
    assert error.value.code == 0
    expected = Path(__file__).with_name('golden') / 'zero_command_help.txt'
    assert capsys.readouterr().out == expected.read_text(encoding='utf-8')


@pytest.mark.parametrize('failure', ['none', 'false', 'executor', 'node'])
def test_zero_exit_uses_nested_best_effort_cleanup(failure):
    """记录嵌套finally的清理顺序，不声称这是可重试的静止证明.

    Args:
        failure: 正常、假返回值、executor异常或节点销毁异常。
    """
    trace = []

    def shutdown():
        trace.append('executor')
        if failure == 'executor':
            raise RuntimeError('executor failed')
        return failure != 'false'

    def destroy():
        trace.append('node')
        if failure == 'node':
            raise RuntimeError('node failed')

    owner = zero_command._RosZeros(())
    owner.executor = NS(shutdown=shutdown)
    owner.node = NS(destroy_node=destroy)
    owner.context = NS(ok=lambda: True, shutdown=lambda: trace.append('context'))
    if failure in ('executor', 'node'):
        with pytest.raises(RuntimeError, match=failure + ' failed'):
            owner.__exit__(None, None, None)
    else:
        assert owner.__exit__(None, None, None) is None
    assert trace == ['executor', 'node', 'context']
    assert owner.node is not None and owner.executor is not None


@pytest.mark.parametrize('kind', [OSError, ValueError])
def test_supervisor_event_isolation_only_catches_io_errors(tmp_path, kind):
    """事件记录仅隔离OSError并记首条错误，其它异常直接传播.

    Args:
        tmp_path: 私有目录路径，不创建进程或资源。
        kind: 注入的事件记录异常类型。
    """
    def append(*args):
        raise kind('event failure')

    owner = startup_lifecycle._Supervisor(
        None, tmp_path, NS(append=append), {}, lambda: None, 1, None)
    if kind is OSError:
        owner.emit('first')
        owner.emit('second')
        assert owner.record_broken
        assert owner.cleanup_errors == ['event log: event failure']
    else:
        with pytest.raises(ValueError, match='event failure'):
            owner.emit('first')
        assert not owner.record_broken and owner.cleanup_errors == []
