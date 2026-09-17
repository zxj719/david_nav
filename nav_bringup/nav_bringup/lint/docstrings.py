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

``check_private_paths`` is a separate opt-in presence gate for private functions
over ten physical definition lines, including nested functions/private classes.
It excludes double-underscore names and does not judge prose quality or detect
non-obvious short functions. Those still require review under SPEC 5.3.

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


def check_private_source(source: str, path: str = '<string>') -> List[Violation]:
    """检查超过十行的私有函数是否有非空说明，不替代公共 API 检查.

    行数从 def/async def 到最后一条语句末行计算，含空行、注释和多行签名，
    不含装饰器。检查嵌套定义及私有类方法，排除双下划线开头的名称。
    这里只检查说明存在；短而复杂的函数和说明准确性仍需人工或代码审查。

    Args:
        source: 待解析的 Python 源码，不会导入或执行。
        path: 诊断信息中使用的文件路径。

    Returns:
        按源码位置排序的缺失说明列表。

    Raises:
        SyntaxError: 源码不是有效的 Python。
    """
    tree = ast.parse(source, filename=path)
    violations = []
    for name, node in _private_functions(tree):
        if node.end_lineno - node.lineno + 1 > 10 and not (ast.get_docstring(node) or '').strip():
            violations.append(Violation(
                path, node.lineno, name, 'missing docstring for private function over 10 lines'))
    return sorted(violations, key=lambda item: item.line)


def check_private_paths(paths: Iterable[Union[str, Path]]) -> List[Violation]:
    """按文件顺序检查指定源码中的长私有函数说明.

    Args:
        paths: 文件或目录；目录递归扫描时忽略 test_ 开头的文件，
            显式传入的 Python 测试文件仍检查，与公共 API 门禁一致。

    Returns:
        按路径及源码位置排序的违规列表；没有缺失时为空。

    Raises:
        SyntaxError: 某个文件不是有效的 Python。
        OSError: 无法读取待检查文件。
    """
    violations = []
    for file_path in sorted(_iter_python_files(paths)):
        violations.extend(check_private_source(
            file_path.read_text(encoding='utf-8'), str(file_path)))
    return violations


def _private_functions(node: ast.AST, prefix: str = '') -> Iterator[tuple[str, FunctionNode]]:
    """遍历全部语法子树并保留类与函数限定名，不执行任何源码.

    Args:
        node: 当前语法节点，包含条件分支中的定义。
        prefix: 外层类与函数的点分名称前缀。

    Yields:
        单下划线私有函数的限定名与 AST 节点，包括异步和嵌套定义。
    """
    if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
        name = prefix + node.name
        if (isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                and node.name.startswith('_') and not node.name.startswith('__')):
            yield name, node
        prefix = name + '.'
    for child in ast.iter_child_nodes(node):
        yield from _private_functions(child, prefix)


def format_violations(violations: Sequence[Violation]) -> str:
    """Render violations one per line for assertion messages.

    Args:
        violations: Violations to render.

    Returns:
        A newline separated report, or an empty string when there are none.
    """
    return '\n'.join(str(violation) for violation in violations)


def _iter_python_files(paths: Iterable[Union[str, Path]]) -> Iterator[Path]:
    """展开待检查的 Python 路径，目录递归扫描时跳过 test_ 文件.

    显式给出的 .py 文件仍会返回，包括 test_ 文件；此迭代器不排序或去重。

    Args:
        paths: 文件或目录路径序列。

    Yields:
        应交给上层策略检查的 Python 文件路径。
    """
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
    """递归检查公开类及其直接公开方法，不扫描函数内部定义.

    Args:
        body: 当前模块或公开类的直接语句列表。
        path: 用于诊断的文件路径。
        prefix: 外层公开类的限定名前缀。

    Returns:
        缺模块成员说明、公共属性 Attributes 项或函数分节的违规列表。
        私有函数由独立门禁检查；这里不替代语义审查。
    """
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
    """检查一个公共函数的文档分节与静态可见的参数、返回和异常约定.

    Args:
        node: 普通或异步函数 AST 节点。
        path: 诊断文件路径。
        qualified_name: 诊断中展示的限定名称。

    Returns:
        缺说明或缺必要分节的违规列表；property 返回说明豁免，
        检查显式 raise 而非推断所有下游异常。
    """
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
    """按已识别的 Google 分节标题切出文本，不验证段落内容.

    仅识别独占一行的已知标题；重复标题采用最后一次内容，不合并同名段落。

    Args:
        docstring: 已取得的说明文本。

    Returns:
        原始标题名到标题后、下一标题前文本的映射；保留段内空白。
    """
    matches = list(_SECTION_RE.finditer(docstring))
    sections: Dict[str, str] = {}
    for index, match in enumerate(matches):
        end = matches[index + 1].start() if index + 1 < len(matches) else len(docstring)
        sections[match.group(1)] = docstring[match.end():end]
    return sections


def _documentable_args(node: FunctionNode) -> List[str]:
    """汇总需要 Args 说明的参数名，按名字排除 self 与 cls.

    覆盖仅位置、普通、仅关键字、可变位置和可变关键字参数，
    但不推断装饰器或接收者别名；self/cls 即使不在首位也被排除。

    Args:
        node: 同步或异步函数 AST。

    Returns:
        参数名称列表，普通参数在前，可变参数追加在后。
    """
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
    """用返回注解优先判断是否需要 Returns，没有注解才扫描函数自身.

    已识别的 None/NoReturn 注解免除要求；其它注解保守视作返回值。
    无注解时忽略嵌套作用域，只找非空且不是字面 None 的 return。

    Args:
        node: 待检查的同步或异步函数 AST。

    Returns:
        当前静态规则是否要求返回值说明，不证明真实控制流或注解正确性。
    """
    if node.returns is not None:
        return not _annotation_is_none(node.returns)
    for child in _walk_own_body(node):
        if isinstance(child, ast.Return) and child.value is not None:
            if not (isinstance(child.value, ast.Constant) and child.value.value is None):
                return True
    return False


def _raises_explicitly(node: FunctionNode) -> bool:
    """检测函数自身作用域内显式抛出的异常，不推断间接调用异常.

    裸 raise 不计入，嵌套函数/类体不展开；不判断抛出路径是否可达。

    Args:
        node: 待检查的同步或异步函数 AST。

    Returns:
        是否找到带异常表达式的 raise 语句。
    """
    return any(
        isinstance(child, ast.Raise) and child.exc is not None
        for child in _walk_own_body(node)
    )
