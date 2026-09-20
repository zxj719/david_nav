"""基于 Python AST 的 Google 风格说明分节检查.

``ament_pep257`` 只管说明的格式，不检查函数是否写了参数、返回值与异常这几节，
而 Google Python 风格指南要求写。:func:`check_paths` 补上这块，
使各包可以在单元测试里强制这些分节。

本检查只判断"分节在不在、参数名有没有被提到"，**不判断说明写得对不对**：
分节齐全不代表描述属实，语义仍需人工审查。

对公共定义（名字不以下划线开头）的规则：

* 每个模块、类与函数都有说明；
* 直接声明的公共数据字段在 ``Attributes:`` 里逐项列名；
* 除 ``self`` / ``cls`` 以外还有参数的函数要有 ``Args:``，且逐个提到参数名；
* 有返回值、或声明了非 ``None`` 返回注解的函数要有 ``Returns:``
  （``@property`` 取值器豁免）；
* 生成器要有 ``Yields:``；
* 函数体里有显式 ``raise SomeError`` 的要有 ``Raises:``。

嵌套函数与私有类的成员不在检查范围内。属性用取值器的说明。
属性发现覆盖类赋值、字面量 slots，以及方法里的实例写入和字面量 setattr；
继承来的字段、计算出来的名字、接收者别名、描述符与动态生成的 API 都发现不了。

``check_private_paths`` 是另一道需显式开启的"有没有写"门禁，针对定义超过十个物理行的
私有函数，包含嵌套函数与私有类，但排除双下划线名字。它不判断文字质量，
也发现不了"短但不显然"的函数——这些仍需按 SPEC 5.3 人工审查。

``check_long_paths`` 另行覆盖所有命名、所有作用域的长函数，包括双下划线
方法和普通名字的嵌套定义；只检查非空说明，不替代公共分节或语义审查。

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
    """一条说明规范的违例.

    Attributes:
        path: 出现问题的定义所在文件。
        line: 该定义的行号，从 1 开始。
        name: 定义的限定名，模块本身为 ``<module>``。
        message: 面向人的问题描述。
    """

    path: str
    line: int
    name: str
    message: str

    def __str__(self) -> str:
        """按 ``path:line name: message`` 格式化本条违例.

        Returns:
            格式化后的文本。
        """
        return f'{self.path}:{self.line} {self.name}: {self.message}'


def check_source(source: str, path: str = '<string>') -> List[Violation]:
    """检查一个模块的源码文本.

    只解析不执行：源码中的导入、装饰器与运行期行为都不会被触发。

    Args:
        source: Python 源码。
        path: 出现在违例信息里的路径。

    Returns:
        按源码顺序排列的违例；模块合规时为空。

    Raises:
        SyntaxError: ``source`` 不是合法的 Python。
    """
    tree = ast.parse(source, filename=path)
    violations: List[Violation] = []
    if ast.get_docstring(tree) is None:
        violations.append(Violation(path, 1, '<module>', 'missing module docstring'))
    violations.extend(_check_definitions(tree.body, path, prefix=''))
    return violations


def check_paths(paths: Iterable[Union[str, Path]]) -> List[Violation]:
    """检查给定文件或目录下的每一个 Python 文件.

    Args:
        paths: 文件或目录。目录按递归搜索，其中以 ``test_`` 开头的文件跳过；
            直接点名的文件不受该跳过规则影响。

    Returns:
        全部违例，先按文件路径、再按源码顺序排列。

    Raises:
        SyntaxError: 某个文件不是合法的 Python。
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


def check_long_source(source: str, path: str = '<string>') -> List[Violation]:
    """检查全部作用域内超过十个物理定义行的函数是否有非空说明.

    同步、异步、嵌套函数及所有类方法均检查，不按名称筛选。
    定义跨度包含空行、注释、多行签名，不含装饰器或末尾注释。
    这里只检查说明存在，不判断短函数复杂性或说明内容是否准确。

    Args:
        source: 只解析、不执行的 Python 源码。
        path: 用于诊断的文件路径。

    Returns:
        按源码位置排序的缺失说明列表。

    Raises:
        SyntaxError: 源码不是有效的 Python。
    """
    tree = ast.parse(source, filename=path)
    violations = []
    for name, node in _all_functions(tree):
        if node.end_lineno - node.lineno + 1 > 10 and not (ast.get_docstring(node) or '').strip():
            violations.append(Violation(
                path, node.lineno, name, 'missing docstring for function over 10 lines'))
    return sorted(violations, key=lambda item: item.line)


def check_long_paths(paths: Iterable[Union[str, Path]]) -> List[Violation]:
    """递归检查指定源码的所有长函数，不导入被检查模块.

    Args:
        paths: 文件或目录；目录扫描忽略 test_ 文件，显式测试文件仍检查。

    Returns:
        按文件路径及源码位置排序的违规列表。

    Raises:
        SyntaxError: 某个文件不是有效的 Python。
        OSError: 无法读取待检查的文件。
    """
    violations = []
    for file_path in sorted(_iter_python_files(paths)):
        violations.extend(check_long_source(
            file_path.read_text(encoding='utf-8'), str(file_path)))
    return violations


def _private_functions(node: ast.AST, prefix: str = '') -> Iterator[tuple[str, FunctionNode]]:
    """保持原单下划线函数筛选规则，遍历逻辑由全作用域迭代器共享.

    Args:
        node: 待遍历的语法节点。
        prefix: 外层限定名前缀。

    Yields:
        单下划线开头、非双下划线的函数限定名和节点。
    """
    for name, function in _all_functions(node, prefix):
        if function.name.startswith('_') and not function.name.startswith('__'):
            yield name, function


def _all_functions(node: ast.AST, prefix: str = '') -> Iterator[tuple[str, FunctionNode]]:
    """遍历全部语法子树并保留类与函数限定名，不执行任何源码.

    Args:
        node: 当前语法节点，包含条件分支中的定义。
        prefix: 外层类与函数的点分名称前缀。

    Yields:
        所有函数的限定名与 AST 节点，包括异步、嵌套和双下划线定义。
    """
    if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
        name = prefix + node.name
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            yield name, node
        prefix = name + '.'
    for child in ast.iter_child_nodes(node):
        yield from _all_functions(child, prefix)


def format_violations(violations: Sequence[Violation]) -> str:
    """把违例逐行渲染，便于放进断言信息.

    Args:
        violations: 待渲染的违例。

    Returns:
        以换行分隔的报告；没有违例时返回空串。
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
    """静态收集当前类声明的公共字段，不导入或执行被检查代码.

    覆盖类体赋值、字面 __slots__ 以及方法接收者的赋值和字面 setattr；
    裸 staticmethod 被跳过。嵌套作用域、继承、接收者别名及动态名称不推断，
    结果是语法线索，不保证字段会在实际运行路径上创建。

    Args:
        node: 待检查的类定义 AST 节点。

    Returns:
        去除下划线开头名称后的字段名集合。
    """
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
    """按装饰器 AST 的名称识别属性访问器，不执行或解析装饰器.

    识别裸 property/cached_property 与属性式 getter/setter/deleter；
    不核对别名或 descriptor 的真实身份，结果只是现有文档规则的语法判定。

    Args:
        node: 同步或异步方法 AST。

    Returns:
        是否符合已识别的属性访问器语法。
    """
    for decorator in node.decorator_list:
        if isinstance(decorator, ast.Name) and decorator.id in _PROPERTY_DECORATORS:
            return True
        if isinstance(decorator, ast.Attribute) and decorator.attr in (
                'setter', 'getter', 'deleter'):
            return True
    return False


def _walk_own_body(node: Union[FunctionNode, ast.ClassDef]) -> Iterator[ast.AST]:
    """遍历定义自身的语法树，遇到嵌套作用域时只产出其定义节点.

    使用后进先出的显式栈，不保证源码顺序，也不判断分支能否执行；
    函数、异步函数、类和 lambda 的内部节点不再展开。

    Args:
        node: 作为遍历根的函数、异步函数或类定义。

    Yields:
        自身 body 中的 AST 节点，包括嵌套定义节点但不包括其内部语句。
    """
    stack: List[ast.AST] = list(node.body)
    while stack:
        current = stack.pop()
        yield current
        if isinstance(current, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)):
            continue
        stack.extend(ast.iter_child_nodes(current))


def _contains(node: FunctionNode, node_types: tuple) -> bool:
    """只在函数自身作用域查找指定 AST 节点类型，不展开嵌套定义.

    范围由 _walk_own_body 统一限定，不推断分支可达性。

    Args:
        node: 待检查的函数 AST。
        node_types: 要匹配的 AST 节点类型元组。

    Returns:
        本作用域内是否至少存在一个匹配节点。
    """
    return any(isinstance(child, node_types) for child in _walk_own_body(node))


def _annotation_is_none(annotation: ast.expr) -> bool:
    """识别文档规则中的无返回值注解，仅接受既有裸名称或 None 常量.

    不解析字符串前向注解、限定名或导入别名；NoReturn 也按无需 Returns 处理。

    Args:
        annotation: 返回注解的 AST 表达式。

    Returns:
        是否为 None 常量或名为 None/NoReturn 的名称节点。
    """
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
