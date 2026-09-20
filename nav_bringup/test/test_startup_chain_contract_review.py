"""本批已审说明的门禁与纯对象边界：不加锁、不创建运行目录、不启动任何进程."""

import ast
from pathlib import Path
import re
import signal

import pytest

import nav_bringup
from nav_bringup.navrun.recorded_startup import StartupInterrupted

MODULES = ('navrun/recorded_startup.py', 'navrun/preparation.py', 'navrun/up_planning.py')


@pytest.mark.parametrize('name', MODULES)
def test_reviewed_summaries_are_chinese(name):
    """检查本批模块的现有摘要，不声称语言存在性可以证明说明语义."""
    path = Path(nav_bringup.__file__).parent / name
    tree = ast.parse(path.read_text(encoding='utf-8'))
    missing = []
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            doc = ast.get_docstring(node)
            if doc and re.search(r'[一-鿿]', doc.splitlines()[0]) is None:
                missing.append((getattr(node, 'lineno', 1), getattr(node, 'name', '<module>')))
    assert not missing, missing


def test_interruption_carries_its_signal_without_stopping_anything():
    """中断异常只带住第一个信号编号，本身不停止任何进程、不改全局处理器."""
    previous = signal.getsignal(signal.SIGINT)
    error = StartupInterrupted(signal.SIGINT)
    assert error.signum == signal.SIGINT
    assert signal.getsignal(signal.SIGINT) is previous
    with pytest.raises(StartupInterrupted) as caught:
        raise error
    assert caught.value.signum == signal.SIGINT
