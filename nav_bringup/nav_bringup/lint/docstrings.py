"""Google-style docstring section checks based on the Python AST.

``ament_pep257`` validates docstring formatting. It does not check that a
function documents its arguments, return value and raised exceptions, which
the Google Python Style Guide requires. :func:`check_paths` fills that gap so
packages can enforce those sections from a unit test.

Rules for public definitions (names not starting with an underscore):

* every module, class and function has a docstring;
* directly declared public data fields have named ``Attributes:`` entries;
* a function with parameters other than ``self`` / ``cls`` has an ``Args:``
  section that mentions each parameter;
* a function that returns a value, or declares a non-``None`` return
  annotation, has a ``Returns:`` section (``@property`` getters are exempt);
* a generator has a ``Yields:`` section;
* a function containing an explicit ``raise SomeError`` has a ``Raises:``
  section.

Nested functions and members of private classes are not checked. Properties use
their getter docstrings. Attribute discovery covers class assignments, literal
slots, and instance writes/literal setattr in methods, not inherited fields,
computed names, receiver aliases, descriptors or dynamically generated APIs.

Example:
    violations = check_paths([Path('my_pkg')])
    assert not violations, format_violations(violations)
"""

from __future__ import annotations

import ast
from dataclasses import dataclass
from pathlib import Path
import re
from typing import Dict, Iterable, Iterator, List, Sequence, Union

_SECTION_RE = re.compile(
    r'^[ \t]*(Args|Arguments|Returns|Yields|Raises|Attributes|Example|Examples|Note|Notes'
    r'|Todo):[ \t]*$',
    re.MULTILINE,
)
_SKIPPED_ARGS = frozenset({'self', 'cls'})
_PROPERTY_DECORATORS = frozenset({'property', 'cached_property'})

FunctionNode = Union[ast.FunctionDef, ast.AsyncFunctionDef]


@dataclass(frozen=True)
class Violation:
    """A single docstring policy violation.

    Attributes:
        path: File that contains the offending definition.
        line: 1-based line number of the definition.
        name: Qualified name of the definition, or ``<module>``.
        message: Human readable description of the problem.
    """

    path: str
    line: int
    name: str
    message: str

    def __str__(self) -> str:
        """Format the violation as ``path:line name: message``.

        Returns:
            The formatted violation.
        """
        return f'{self.path}:{self.line} {self.name}: {self.message}'


def check_source(source: str, path: str = '<string>') -> List[Violation]:
    """Check the source text of one module.

    Args:
        source: Python source code.
        path: Path used in violation messages.

    Returns:
        Violations in source order; empty when the module complies.

    Raises:
        SyntaxError: If ``source`` is not valid Python.
    """
    tree = ast.parse(source, filename=path)
    violations: List[Violation] = []
    if ast.get_docstring(tree) is None:
        violations.append(Violation(path, 1, '<module>', 'missing module docstring'))
    violations.extend(_check_definitions(tree.body, path, prefix=''))
    return violations


def check_paths(paths: Iterable[Union[str, Path]]) -> List[Violation]:
    """Check every Python file under the given files or directories.

    Args:
        paths: Files or directories. Directories are searched recursively and
            files whose names start with ``test_`` are skipped there.

    Returns:
        All violations, ordered by file path and then by source order.

    Raises:
        SyntaxError: If a file is not valid Python.
    """
    violations: List[Violation] = []
    for file_path in sorted(_iter_python_files(paths)):
        source = file_path.read_text(encoding='utf-8')
        violations.extend(check_source(source, str(file_path)))
    return violations


def format_violations(violations: Sequence[Violation]) -> str:
    """Render violations one per line for assertion messages.

    Args:
        violations: Violations to render.

    Returns:
        A newline separated report, or an empty string when there are none.
    """
    return '\n'.join(str(violation) for violation in violations)


def _iter_python_files(paths: Iterable[Union[str, Path]]) -> Iterator[Path]:
    for raw_path in paths:
        path = Path(raw_path)
        if path.is_dir():
            for candidate in path.rglob('*.py'):
                if not candidate.name.startswith('test_'):
                    yield candidate
        elif path.suffix == '.py':
            yield path


def _is_public(name: str) -> bool:
    return not name.startswith('_')


def _check_definitions(body: Sequence[ast.stmt], path: str, prefix: str) -> List[Violation]:
    violations: List[Violation] = []
    for node in body:
        if isinstance(node, ast.ClassDef) and _is_public(node.name):
            qualified_name = prefix + node.name
            if ast.get_docstring(node) is None:
                violations.append(
                    Violation(path, node.lineno, qualified_name, 'missing class docstring'))
            else:
                section = _split_sections(ast.get_docstring(node)).get('Attributes', '')
                for name in sorted(_class_attributes(node)):
                    pattern = r'^[ \t]*' + re.escape(name) + r'\s*(?:\([^)]*\))?:'
                    if re.search(pattern, section, re.MULTILINE) is None:
                        violations.append(Violation(
                            path, node.lineno, qualified_name,
                            f'attribute {name!r} not documented in Attributes:'))
            violations.extend(_check_definitions(node.body, path, qualified_name + '.'))
        elif isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and _is_public(node.name):
            violations.extend(_check_function(node, path, prefix + node.name))
    return violations


def _class_attributes(node: ast.ClassDef) -> set[str]:
    """Collect fields declared by this class without evaluating or importing it."""
    names = set()
    for child in _walk_own_body(node):
        if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Store):
            names.add(child.id)
        elif isinstance(child, (ast.Assign, ast.AnnAssign)):
            targets = child.targets if isinstance(child, ast.Assign) else [child.target]
            if any(isinstance(target, ast.Name) and target.id == '__slots__'
                   for target in targets):
                value = child.value
                if isinstance(value, ast.Dict):
                    entries = value.keys
                else:
                    entries = value.elts if isinstance(value, (ast.Tuple, ast.List)) else [value]
                names.update(item.value for item in entries
                             if isinstance(item, ast.Constant) and isinstance(item.value, str))
        elif isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
            decorators = {item.id for item in child.decorator_list if isinstance(item, ast.Name)}
            if 'staticmethod' in decorators:
                continue
            args = child.args.posonlyargs + child.args.args
            if not args:
                continue
            receiver = args[0].arg
            for item in _walk_own_body(child):
                if (isinstance(item, ast.Attribute) and isinstance(item.ctx, ast.Store)
                        and isinstance(item.value, ast.Name) and item.value.id == receiver):
                    names.add(item.attr)
                elif (isinstance(item, ast.Call) and isinstance(item.func, ast.Name)
                      and item.func.id == 'setattr' and len(item.args) == 3
                      and isinstance(item.args[0], ast.Name) and item.args[0].id == receiver
                      and isinstance(item.args[1], ast.Constant)
                      and isinstance(item.args[1].value, str)):
                    names.add(item.args[1].value)
    return {name for name in names if _is_public(name)}


def _check_function(node: FunctionNode, path: str, qualified_name: str) -> List[Violation]:
    docstring = ast.get_docstring(node)
    if docstring is None:
        return [Violation(path, node.lineno, qualified_name, 'missing function docstring')]

    problems: List[str] = []
    sections = _split_sections(docstring)

    arg_names = _documentable_args(node)
    if arg_names:
        args_text = sections.get('Args', sections.get('Arguments'))
        if args_text is None:
            problems.append('missing Args: section')
        else:
            for arg_name in arg_names:
                pattern = r'^[ \t]*\**' + re.escape(arg_name) + r'\b'
                if re.search(pattern, args_text, re.MULTILINE) is None:
                    problems.append(f'argument {arg_name!r} not documented in Args:')

    if not _is_property(node):
        if _contains(node, (ast.Yield, ast.YieldFrom)):
            if 'Yields' not in sections:
                problems.append('missing Yields: section')
        elif _returns_value(node) and 'Returns' not in sections:
            problems.append('missing Returns: section')

    if _raises_explicitly(node) and 'Raises' not in sections:
        problems.append('missing Raises: section')

    return [Violation(path, node.lineno, qualified_name, problem) for problem in problems]


def _split_sections(docstring: str) -> Dict[str, str]:
    matches = list(_SECTION_RE.finditer(docstring))
    sections: Dict[str, str] = {}
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(docstring)
        sections[match.group(1)] = docstring[match.end():end]
    return sections


def _documentable_args(node: FunctionNode) -> List[str]:
    arguments = node.args
    names = [arg.arg for arg in arguments.posonlyargs + arguments.args + arguments.kwonlyargs]
    if arguments.vararg is not None:
        names.append(arguments.vararg.arg)
    if arguments.kwarg is not None:
        names.append(arguments.kwarg.arg)
    return [name for name in names if name not in _SKIPPED_ARGS]


def _is_property(node: FunctionNode) -> bool:
    for decorator in node.decorator_list:
        if isinstance(decorator, ast.Name) and decorator.id in _PROPERTY_DECORATORS:
            return True
        if isinstance(decorator, ast.Attribute) and decorator.attr in (
                'setter', 'getter', 'deleter'):
            return True
    return False


def _walk_own_body(node: Union[FunctionNode, ast.ClassDef]) -> Iterator[ast.AST]:
    """Yield nodes of a function body without descending into nested scopes."""
    stack: List[ast.AST] = list(node.body)
    while stack:
        current = stack.pop()
        yield current
        if isinstance(current, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)):
            continue
        stack.extend(ast.iter_child_nodes(current))


def _contains(node: FunctionNode, node_types: tuple) -> bool:
    return any(isinstance(child, node_types) for child in _walk_own_body(node))


def _annotation_is_none(annotation: ast.expr) -> bool:
    if isinstance(annotation, ast.Constant) and annotation.value is None:
        return True
    return isinstance(annotation, ast.Name) and annotation.id in ('None', 'NoReturn')


def _returns_value(node: FunctionNode) -> bool:
    if node.returns is not None:
        return not _annotation_is_none(node.returns)
    for child in _walk_own_body(node):
        if isinstance(child, ast.Return) and child.value is not None:
            if not (isinstance(child.value, ast.Constant) and child.value.value is None):
                return True
    return False


def _raises_explicitly(node: FunctionNode) -> bool:
    return any(
        isinstance(child, ast.Raise) and child.exc is not None
        for child in _walk_own_body(node)
    )
