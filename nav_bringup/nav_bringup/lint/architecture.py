"""Check split task/executor layers without importing their runtime dependencies.

This AST graph includes relative, conditional and deferred static imports and
package initializers. It expands only the inspected package, not external code
or dynamically computed imports. It is not a replacement for TODO-I1's optional
import-linter installation. Logical adapter aliases retain the existing layout.
"""

import ast
from collections import deque
from pathlib import Path
from typing import Iterable, Mapping

_LAYERS = {'app': 5, 'api': 5, 'adapters': 4, 'nav2_client': 4,
           'nav_site_data': 4, 'core': 3, 'protocol': 2, 'skills': 2, 'domain': 0}
_PURE = {'domain', 'protocol', 'core', 'skills'}
_FACTORIES = {'create_runtime', 'create_dock_ingress', 'declare_dock_parameters'}


def check_package(path: Path) -> list[str]:
    """Read and statically check all Python modules of one navigation package.

    Args:
        path: Package source directory named nav_tasks or nav_executor.

    Returns:
        Sorted violations with dependency-chain witnesses; empty means compliant.

    Raises:
        OSError: A source file cannot be read.
        SyntaxError: A Python source is invalid.
        ValueError: The package is not covered by this policy.
    """
    sources = {}
    packages = set()
    for file_path in sorted(path.rglob('*.py')):
        parts = file_path.relative_to(path).with_suffix('').parts
        is_package = parts[-1] == '__init__'
        module = '.'.join((path.name,) + (parts[:-1] if is_package else parts))
        sources[module] = file_path.read_text(encoding='utf-8')
        if is_package:
            packages.add(module)
    return check_sources(path.name, sources, packages=packages)


def check_sources(
    package: str, sources: Mapping[str, str], *, packages: Iterable[str] = (),
) -> list[str]:
    """Enforce layers, adapter independence and pure/public package boundaries.

    Args:
        package: nav_tasks or nav_executor; other packages are external leaves.
        sources: Fully qualified module names mapped to Python source text.
        packages: Modules representing package initializers, for relative imports.

    Returns:
        Deterministically ordered diagnostics with shortest dependency witnesses.

    Raises:
        SyntaxError: A source is invalid.
        ValueError: The package or module set is outside the policy scope.
    """
    if package not in ('nav_tasks', 'nav_executor'):
        raise ValueError('architecture policy covers nav_tasks and nav_executor only')
    if any(name != package and not name.startswith(package + '.') for name in sources):
        raise ValueError('sources must belong to the inspected package')
    initializers = set(packages)
    graph = {}
    for module, source in sources.items():
        targets = _imports(module, source, module in initializers)
        # Python also executes every present parent __init__ when importing a child.
        for target in tuple(targets):
            parts = target.split('.')
            targets.update('.'.join(parts[:index]) for index in range(1, len(parts))
                           if '.'.join(parts[:index]) in sources)
        graph[module] = targets - {module}
    violations = set()
    for origin in sorted(sources):
        queue = deque([(origin,)])
        visited = {origin}
        while queue:
            chain = queue.popleft()
            for target in sorted(graph.get(chain[-1], ())):
                if target in visited:
                    continue
                visited.add(target)
                witness = chain + (target,)
                for reason in _violations(package, origin, target):
                    violations.add(f'{reason}: {" -> ".join(witness)}')
                if target in graph:
                    queue.append(witness)
    return sorted(violations)


def _imports(module, source, is_package):
    """从整棵源码 AST 收集显式导入目标，展开相对路径但不导入模块.

    Args:
        module: 当前模块点分名称。
        source: Python 源码字符串。
        is_package: 当前源码是否为包的初始化模块。

    Returns:
        可能的模块及 from-import 成员目标集合；不判断分支是否运行。

    Raises:
        SyntaxError: 源码不能解析。
        ValueError: 相对导入层级超出包上下文。
    """
    targets = set()
    context = module.split('.') if is_package else module.split('.')[:-1]
    for node in ast.walk(ast.parse(source, filename=module)):
        if isinstance(node, ast.Import):
            targets.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                if node.level > len(context):
                    raise ValueError(f'{module}: relative import escapes package')
                prefix = context[:len(context) - node.level + 1]
                base = '.'.join(prefix + ([node.module] if node.module else []))
            else:
                base = node.module or ''
            targets.add(base)
            targets.update(f'{base}.{alias.name}' for alias in node.names
                           if alias.name != '*')
    return targets


def _section(module):
    parts = module.split('.')
    return parts[1] if len(parts) > 1 else ''


def _adapter(module):
    parts = module.split('.')
    if _section(module) == 'nav2_client':
        return 'ros'
    if _section(module) == 'nav_site_data':
        return 'storage'
    if _section(module) == 'adapters' and len(parts) > 2:
        return parts[2]
    return None


def _violations(package, origin, target):
    """逐项报告一条静态导入边违反的分层规则.

    Args:
        package: 被检查的包名。
        origin: 发起导入的模块限定名。
        target: 已解析的目标模块或成员限定名。

    Yields:
        可读违规标签；一条边可命中多条规则，不证明动态导入行为。
    """
    root = target.split('.')[0]
    pure = _section(origin) in _PURE
    if root == 'david_nav':
        yield 'reverse legacy dependency'
    if package == 'nav_tasks' and root in ('nav_executor', 'websockets'):
        yield 'task/transport dependency'
    if package == 'nav_executor' and root == 'nav_tasks':
        if target != 'nav_tasks.api' and not target.startswith('nav_tasks.api.'):
            yield 'bypasses public TaskApi'
        if pure and target.removeprefix('nav_tasks.api.') in _FACTORIES:
            yield 'pure-layer factory'
    if pure and (root in ('rclpy', 'websockets', 'yaml') or root.endswith('_msgs')):
        yield 'pure-layer I/O'
    if _adapter(origin) == 'storage' and (root == 'rclpy' or root.endswith('_msgs')):
        yield 'storage ROS dependency'
    if root != package:
        return
    source_layer = _LAYERS.get(_section(origin))
    target_layer = _LAYERS.get(_section(target))
    if source_layer is not None and target_layer is not None and target_layer > source_layer:
        yield 'upward layer'
    if {_section(origin), _section(target)} == {'protocol', 'skills'}:
        yield 'protocol/skills independence'
    source_adapter, target_adapter = _adapter(origin), _adapter(target)
    if source_adapter and target_adapter and source_adapter != target_adapter:
        yield 'adapter independence'
