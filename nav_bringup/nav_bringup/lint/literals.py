"""Forbidden string literals in package code (refactor plan §5.4).

Robot frame names and robot-specific topic prefixes belong in the robot
configuration package, so that porting to another robot only touches
configuration. This check scans Python sources with :mod:`ast` and reports
string constants that equal a forbidden frame name or contain a forbidden
topic prefix. Docstrings are ignored; tests are expected to be excluded by the
caller.
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
    """A forbidden string literal.

    Attributes:
        path: Source file.
        line: 1-based line of the literal.
        value: The literal.
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
    """Find forbidden literals in Python source text.

    Args:
        source: Python source code.
        path: File name used in the results.
        forbidden_exact: Literals that are forbidden as a whole string.
        forbidden_substrings: Substrings that are forbidden anywhere in a literal.

    Returns:
        Violations ordered by line.

    Raises:
        SyntaxError: If the source cannot be parsed.
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
    """Find forbidden literals in Python files and directories.

    Args:
        paths: Python files, or directories searched recursively for ``*.py``.
        forbidden_exact: Literals that are forbidden as a whole string.
        forbidden_substrings: Substrings that are forbidden anywhere in a literal.
        exclude: Files to skip, for example the module that defines a policy.

    Returns:
        Violations ordered by file and line.
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
    """Format violations for an assertion message.

    Args:
        violations: Violations to format.

    Returns:
        One ``path:line: literal`` line per violation.
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
