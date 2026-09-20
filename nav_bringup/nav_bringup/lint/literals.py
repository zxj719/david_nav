"""包代码里的禁用字符串字面量（方案 §5.4）.

坐标系名与机器人专有的话题前缀属于机器人配置包，这样换一台车只需改配置。
本检查用 :mod:`ast` 扫描 Python 源码，报告等于禁用坐标系名、或包含禁用话题前缀的
字符串常量。说明文本不算在内；测试文件由调用方负责排除。

只看静态字符串常量：拼接、f-string 的变量部分与运行期构造的名字都发现不了，
因此检查通过不等于代码里没有写死的机器人专有名字。
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Set, Union

FORBIDDEN_EXACT = ('base', 'base_link', 'base_footprint')
FORBIDDEN_SUBSTRINGS = ('/lightning/', '/david001/')


@dataclass(frozen=True)
class LiteralViolation:
    """一处禁用字符串字面量.

    Attributes:
        path: 源文件。
        line: 该字面量所在行号，从 1 开始。
        value: 字面量本身。
    """

    path: Path
    line: int
    value: str


def check_source(
    source: str,
    path: Union[str, Path],
    forbidden_exact: Sequence[str] = FORBIDDEN_EXACT,
    forbidden_substrings: Sequence[str] = FORBIDDEN_SUBSTRINGS,
) -> List[LiteralViolation]:
    """在 Python 源码文本里找禁用字面量.

    Args:
        source: Python 源码。
        path: 出现在结果里的文件名。
        forbidden_exact: 整串相等即禁用的字面量。
        forbidden_substrings: 出现在字面量任意位置即禁用的子串。

    Returns:
        按行号排列的违例。

    Raises:
        SyntaxError: 源码无法解析。
    """
    tree = ast.parse(source, filename=str(path))
    docstrings = _docstring_ids(tree)
    violations = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Constant) or not isinstance(node.value, str):
            continue
        if id(node) in docstrings:
            continue
        value = node.value
        if value in forbidden_exact or any(part in value for part in forbidden_substrings):
            violations.append(LiteralViolation(Path(path), node.lineno, value))
    return sorted(violations, key=lambda violation: violation.line)


def check_paths(
    paths: Iterable[Union[str, Path]],
    forbidden_exact: Sequence[str] = FORBIDDEN_EXACT,
    forbidden_substrings: Sequence[str] = FORBIDDEN_SUBSTRINGS,
    exclude: Iterable[Union[str, Path]] = (),
) -> List[LiteralViolation]:
    """在 Python 文件与目录里找禁用字面量.

    Args:
        paths: Python 文件，或按递归查找 ``*.py`` 的目录。
        forbidden_exact: 整串相等即禁用的字面量。
        forbidden_substrings: 出现在字面量任意位置即禁用的子串。
        exclude: 要跳过的文件，例如定义这份策略的模块本身。

    Returns:
        按文件与行号排列的违例。
    """
    excluded = {Path(entry).resolve() for entry in exclude}
    files: List[Path] = []
    for entry in paths:
        path = Path(entry)
        files.extend(sorted(path.rglob('*.py')) if path.is_dir() else [path])
    violations: List[LiteralViolation] = []
    for file_path in files:
        if file_path.resolve() in excluded:
            continue
        violations.extend(check_source(
            file_path.read_text(encoding='utf-8'), file_path,
            forbidden_exact, forbidden_substrings))
    return violations


def format_violations(violations: Sequence[LiteralViolation]) -> str:
    """把违例格式化成断言信息.

    Args:
        violations: 待格式化的违例。

    Returns:
        每条违例一行，格式为 ``path:line: literal``。
    """
    return '\n'.join(
        f'{violation.path}:{violation.line}: forbidden literal {violation.value!r} '
        '(put robot frames and topics in the robot configuration package)'
        for violation in violations)


def _docstring_ids(tree: ast.AST) -> Set[int]:
    """定位真正的定义 docstring 字符串节点，供字面量门禁排除文档.

    Args:
        tree: 已解析的模块 AST。

    Returns:
        模块、类及同步/异步函数首条字符串表达式的节点身份集合，
        不排除普通运行时字符串或后续字符串表达式。
    """
    ids = set()
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        if not node.body:
            continue
        first = node.body[0]
        if (isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant)
                and isinstance(first.value.value, str)):
            ids.add(id(first.value))
    return ids
