"""从机器人档案、模式叠加和绑定表生成节点参数树.

基础树用 <bind:frames.nav_base> 等标记引用档案键；绑定表列出各键
写入的参数路径，模式 profile 深合并后再赋值。nav2_overrides 仅可
修改未绑定内容，不得覆盖绑定值本身、祖先或后代；列表下标别名按实际
容器类型归一化，字典数字键仍保持区别。

参数路径按斜杠分隔，保留 Nav2 参数名中的点；列表位置使用整数下标。
本模块不启动 ROS，不自动执行完整档案业务校验，也不保证写文件原子性。
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
import re
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple, Union

import yaml

BINDINGS_SCHEMA_VERSION = 1
MODE_SCHEMA_VERSION = 1
PATH_SEPARATOR = '/'

_KEY_PATTERN = r'[a-z][a-z0-9_]*(?:\.[a-z][a-z0-9_]*)*'
_KEY_RE = re.compile(rf'^{_KEY_PATTERN}$')
_MARKER_PREFIX = '<bind:'
_MARKER_RE = re.compile(rf'^<bind:(?P<key>{_KEY_PATTERN})>$')
_MODE_FIELDS = frozenset({'schema_version', 'mode', 'description', 'profile', 'nav2_overrides'})


class RenderError(ValueError):
    """表示参数渲染的已处理格式或一致性错误，不涵盖所有底层异常."""


def _index(node: List[Any], segment: str, path: str) -> int:
    """把列表路径段转换为现有元素下标，不允许负索引或追加位置.

    Args:
        node: 当前基础参数列表。
        segment: 应表示非负下标的路径段文本。
        path: 错误信息中使用的完整参数路径。

    Returns:
        小于列表长度的整数下标。

    Raises:
        RenderError: 路径段不是数字形状或下标超出列表。
        ValueError: isdigit 接受但 int 无法转换的特殊数字字符。
    """
    if not segment.isdigit() or int(segment) >= len(node):
        raise RenderError(f'{path}: {segment!r} is not an index of a list of {len(node)} items')
    return int(segment)


def _child(node: Any, segment: str, path: str) -> Any:
    """沿字典键或列表下标读取基础树，不创建缺失项.

    Args:
        node: 当前基础树节点。
        segment: 字典键或列表下标文本。
        path: 错误信息中的完整参数路径。

    Returns:
        已存在的子节点，直接引用原对象而非复制。

    Raises:
        RenderError: 字典键缺失、列表下标不合法或当前节点不可继续下钻。
        ValueError: 特殊数字字符无法转换成列表下标。
    """
    if isinstance(node, dict):
        if segment not in node:
            raise RenderError(f'{path}: parameter does not exist in the base tree')
        return node[segment]
    if isinstance(node, list):
        return node[_index(node, segment, path)]
    raise RenderError(f'{path}: cannot descend into {type(node).__name__}')


def _canonical_path(tree: Any, path: str) -> str:
    """按当前树的容器类型规范化列表下标，不改变字典键.

    用于比较覆盖目标和绑定路径；00 与 0 仅在列表位置代表同一个元素。
    允许末尾字典键尚不存在，以保留覆盖新增未绑定参数的既有行为。

    Args:
        tree: 已填入绑定值的参数树，只读访问。
        path: 待覆盖的斜杠分隔路径。

    Returns:
        列表下标统一为十进制字符串后的路径。

    Raises:
        RenderError: 路径无效、父项缺失或列表下标不合法。
        ValueError: 特殊数字字符不能转换为整数下标。
    """
    parts = split_path(path)
    node = tree
    canonical = []
    for index, part in enumerate(parts):
        canonical.append(str(_index(node, part, path)) if isinstance(node, list) else part)
        if index < len(parts) - 1:
            node = _child(node, part, path)
    return PATH_SEPARATOR.join(canonical)


def _format_value(value: Any) -> Any:
    return copy.deepcopy(value)


def _format_footprint_string(value: Any) -> str:
    """把至少三个二维数值点格式化为 Nav2 footprint 字符串.

    拒绝布尔坐标，按 float 的 repr 保留确定格式；此处不验证有限性、
    凸性、面积或碰撞可行性，不替代其它几何与安全检查。

    Args:
        value: 由列表组成的至少三个 [x, y] 点。

    Returns:
        既有带逗号与空格格式的多边形字符串。

    Raises:
        RenderError: 输入不是要求的数值点列表。
        OverflowError: 某个整数坐标无法转换为浮点数。
    """
    points_ok = isinstance(value, list) and len(value) >= 3 and all(
        isinstance(point, list) and len(point) == 2 and all(
            isinstance(coordinate, (int, float)) and not isinstance(coordinate, bool)
            for coordinate in point)
        for point in value)
    if not points_ok:
        raise RenderError(f'expected a polygon of at least three [x, y] points, got {value!r}')
    return '[' + ', '.join('[%r, %r]' % (float(x), float(y)) for x, y in value) + ']'


VALUE_FORMATS: Dict[str, Callable[[Any], Any]] = {
    'value': _format_value,
    'footprint_string': _format_footprint_string,
}


@dataclass(frozen=True)
class Binding:
    """保存一个档案键到参数树路径的绑定，不自行执行校验.

    Attributes:
        key: 点分档案键，例如 frames.nav_base。
        paths: 接收该值的斜杠分隔参数路径元组。
        value_format: VALUE_FORMATS 中选择的格式化函数名称。
    """

    key: str
    paths: Tuple[str, ...]
    value_format: str = 'value'


@dataclass(frozen=True)
class ModeOverlay:
    """保存模式对档案和未绑定参数的差异，冻结但不深度复制映射.

    Attributes:
        name: 非空模式名；parse_mode 不强制命名字符规则。
        profile: 深合并到机器人档案上的嵌套值。
        nav2_overrides: 绑定填充后应用的参数路径到值映射。
    """

    name: str
    profile: Mapping[str, Any]
    nav2_overrides: Mapping[str, Any]


def split_path(path: str) -> List[str]:
    """按斜杠拆分参数路径并拒绝空路径段.

    Args:
        path: 斜杠分隔的字符串，参数名中的点保持原样。

    Returns:
        顺序路径段列表，不剥除空白或在此验证列表下标。

    Raises:
        RenderError: 输入非字符串、为空，或含空路径段。
    """
    segments = path.split(PATH_SEPARATOR) if isinstance(path, str) else []
    if not segments or any(not segment for segment in segments):
        raise RenderError(f'invalid parameter path {path!r}')
    return segments


def get_path(tree: Any, path: str) -> Any:
    """沿 dict 键和列表下标读取参数树，返回原值引用.

    Args:
        tree: 由 dict 和 list 组成的嵌套树。
        path: 斜杠分隔路径。

    Returns:
        路径对应的值，不进行复制。

    Raises:
        RenderError: 路径缺失、容器不能下钻或下标无效。
        ValueError: 特殊数字字符不能转为整数下标。
    """
    node = tree
    for segment in split_path(path):
        node = _child(node, segment, path)
    return node


def set_path(tree: Any, path: str, value: Any, create: bool = False) -> None:
    """原地设置路径值，仅可选允许新增末尾字典键.

    Args:
        tree: 要修改的 dict/list 嵌套树。
        path: 斜杠分隔路径，中间父节点必须存在。
        value: 直接赋入的值，不复制，调用方负责共享引用。
        create: 是否允许新增末尾字典键；不创建父级、不追加列表。

    Raises:
        RenderError: 路径或父节点无效，或不允许新增末尾键。
        ValueError: 特殊数字字符不能转为整数下标。
    """
    segments = split_path(path)
    parent = tree
    for segment in segments[:-1]:
        parent = _child(parent, segment, path)
    last = segments[-1]
    if isinstance(parent, dict):
        if last not in parent and not create:
            raise RenderError(f'{path}: parameter does not exist in the base tree')
        parent[last] = value
    elif isinstance(parent, list):
        parent[_index(parent, last, path)] = value
    else:
        raise RenderError(f'{path}: cannot descend into {type(parent).__name__}')


def find_markers(tree: Any, prefix: str = '') -> Dict[str, str]:
    """递归查找以绑定前缀开头且完整匹配语法的字符串.

    只有以 <bind: 开头的字符串会尝试标记校验；其它字符串保持普通值。
    不处理循环引用或深度预算，字典键转字符串后可能碰撞。

    Args:
        tree: dict/list 嵌套树或叶子值。
        prefix: 当前树在更大树中的路径前缀。

    Returns:
        标记路径到档案键的映射，列表路径按规范十进制下标生成。

    Raises:
        RenderError: 标记格式无效或字典键的字符串包含斜杠。
    """
    if isinstance(tree, str):
        if not tree.startswith(_MARKER_PREFIX):
            return {}
        match = _MARKER_RE.fullmatch(tree)
        if match is None:
            raise RenderError(f'{prefix}: malformed binding marker {tree!r}')
        return {prefix: match.group('key')}
    if isinstance(tree, dict):
        items = []
        for key, value in tree.items():
            if PATH_SEPARATOR in str(key):
                raise RenderError(f'{prefix}: parameter name {key!r} contains {PATH_SEPARATOR!r}')
            items.append((str(key), value))
    elif isinstance(tree, list):
        items = [(str(index), value) for index, value in enumerate(tree)]
    else:
        return {}
    markers: Dict[str, str] = {}
    for key, value in items:
        markers.update(find_markers(value, f'{prefix}{PATH_SEPARATOR}{key}' if prefix else key))
    return markers


def deep_merge(base: Mapping[str, Any], overlay: Mapping[str, Any]) -> Dict[str, Any]:
    """深复制并合并嵌套映射，列表与标量整体替换.

    Args:
        base: 基础值映射。
        overlay: 覆盖值映射；仅两侧同为 Mapping 时递归合并。

    Returns:
        新字典；普通 YAML 输入不被修改。深复制或递归异常直接传播，
        不提供任意自定义对象或循环图的统一处理保证。
    """
    merged = copy.deepcopy(dict(base))
    for key, value in overlay.items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def profile_value(profile: Mapping[str, Any], key: str) -> Any:
    """沿点分键查找档案值，不应用默认值或复制对象.

    Args:
        profile: 嵌套档案映射。
        key: 点分路径，不在此执行 schema 键名格式校验。

    Returns:
        匹配值的直接引用。

    Raises:
        RenderError: 任一路径段缺失或其父节点不是 Mapping。
    """
    node: Any = profile
    for part in key.split('.'):
        if not isinstance(node, Mapping) or part not in node:
            raise RenderError(f'the profile has no value for {key}')
        node = node[part]
    return node


def parse_bindings(data: Any, source: Union[str, Path]) -> Dict[str, Binding]:
    """解析绑定表，校验键语法、格式器及精确路径重复.

    顶层版本沿用相等比较，未检查其它顶层字段。路径重复按原字符串判断；
    基础树标记与绑定路径的一致性在 render 中再验证。

    Args:
        data: YAML 解析的绑定表。
        source: 用于错误消息的来源名称。

    Returns:
        按档案键索引的 Binding 字典，路径保存为元组。

    Raises:
        RenderError: 已处理的表结构、键名、格式名、路径或重复绑定错误。
        TypeError: 例如不可哈希的 format 值等未包装的异常。
    """
    if not isinstance(data, dict) or data.get('schema_version') != BINDINGS_SCHEMA_VERSION:
        raise RenderError(f'{source}: expected schema_version {BINDINGS_SCHEMA_VERSION}')
    entries = data.get('bindings')
    if not isinstance(entries, dict) or not entries:
        raise RenderError(f'{source}: bindings must be a non-empty mapping')
    owners: Dict[str, str] = {}
    bindings: Dict[str, Binding] = {}
    for key, entry in entries.items():
        if not isinstance(key, str) or not _KEY_RE.fullmatch(key):
            raise RenderError(f'{source}: invalid profile key {key!r}')
        if isinstance(entry, list):
            entry = {'paths': entry}
        if not isinstance(entry, dict):
            raise RenderError(f'{source}: {key}: expected a list of paths or a mapping')
        unknown = sorted(str(name) for name in set(entry) - {'paths', 'format'})
        if unknown:
            raise RenderError(f'{source}: {key}: unknown fields {unknown}')
        paths = entry.get('paths')
        if not isinstance(paths, list) or not paths:
            raise RenderError(f'{source}: {key}: paths must be a non-empty list')
        value_format = entry.get('format', 'value')
        if value_format not in VALUE_FORMATS:
            raise RenderError(
                f'{source}: {key}: format must be one of {sorted(VALUE_FORMATS)}, '
                f'got {value_format!r}')
        for path in paths:
            split_path(path)
            if path in owners:
                raise RenderError(f'{source}: {path} is bound by both {owners[path]} and {key}')
            owners[path] = key
        bindings[key] = Binding(key, tuple(paths), value_format)
    return bindings


def parse_mode(data: Any, source: Union[str, Path]) -> ModeOverlay:
    """解析模式差异并校验映射和路径的基本形状.

    模式名只要求非空字符串；版本沿用相等比较，description 内容不验证。
    未验证 profile 的业务语义，也不深复制 profile/nav2_overrides 映射。

    Args:
        data: YAML 解析的模式叠加数据。
        source: 用于错误消息的来源名称。

    Returns:
        保存原映射引用的 ModeOverlay。

    Raises:
        RenderError: 版本、未知顶层字段、名称、映射或路径形状不符。
    """
    if not isinstance(data, dict) or data.get('schema_version') != MODE_SCHEMA_VERSION:
        raise RenderError(f'{source}: expected schema_version {MODE_SCHEMA_VERSION}')
    unknown = sorted(str(name) for name in set(data) - _MODE_FIELDS)
    if unknown:
        raise RenderError(f'{source}: unknown fields {unknown}')
    name = data.get('mode')
    if not isinstance(name, str) or not name:
        raise RenderError(f'{source}: mode must be a non-empty name')
    profile = {} if data.get('profile') is None else data['profile']
    overrides = {} if data.get('nav2_overrides') is None else data['nav2_overrides']
    if not isinstance(profile, dict):
        raise RenderError(f'{source}: profile must be a mapping')
    if not isinstance(overrides, dict):
        raise RenderError(f'{source}: nav2_overrides must be a mapping')
    for path in overrides:
        split_path(path)
    return ModeOverlay(name, profile, overrides)


def render(
    base: Mapping[str, Any],
    bindings: Mapping[str, Binding],
    profile: Mapping[str, Any],
    mode: Optional[ModeOverlay] = None,
) -> Dict[str, Any]:
    """填充基础树的绑定标记，再应用不触及绑定值的模式覆盖.

    绑定表必须已按入口规则解析；基础标记和路径须一致。覆盖检查使用
    实际容器类型规范化列表下标，并拒绝绑定值本身、祖先与后代。
    允许在已有未绑定字典中新增末尾键。这里不执行 schema/业务值校验，
    也不递归展开档案值或覆盖值中新注入的标记文本。

    Args:
        base: 含绑定标记的基础参数映射。
        bindings: 按档案键索引的已解析绑定表。
        profile: 机器人档案。
        mode: 可选模式叠加。

    Returns:
        新参数树，普通 YAML 输入不被修改；基础树原有标记已替换，
        不保证注入值中没有形似标记的普通字符串。

    Raises:
        RenderError: 基础标记不匹配、档案键缺失、格式器形状错误、
            覆盖与绑定重叠或覆盖父路径缺失等已处理错误。
    """
    markers = find_markers(base)
    bound = {path: key for key, binding in bindings.items() for path in binding.paths}
    unbound = sorted(path for path in markers if path not in bound)
    if unbound:
        raise RenderError('binding markers without a binding: ' + ', '.join(unbound))
    wrong = sorted(
        f'{path} (bound to {key}, holds {markers.get(path)!r})'
        for path, key in bound.items() if markers.get(path) != key)
    if wrong:
        raise RenderError('bound parameters that do not hold their marker: ' + ', '.join(wrong))

    effective = deep_merge(profile, mode.profile) if mode is not None else profile
    tree = copy.deepcopy(dict(base))
    for key, binding in bindings.items():
        try:
            value = VALUE_FORMATS[binding.value_format](profile_value(effective, key))
        except RenderError as error:
            raise RenderError(f'{key}: {error}') from error
        for path in binding.paths:
            set_path(tree, path, copy.deepcopy(value))

    if mode is not None:
        for path, value in mode.nav2_overrides.items():
            canonical = _canonical_path(tree, path)
            covered = sorted(
                bound_path for bound_path in bound
                if bound_path == canonical
                or bound_path.startswith(canonical + PATH_SEPARATOR)
                or canonical.startswith(bound_path + PATH_SEPARATOR))
            if covered:
                raise RenderError(
                    f'mode {mode.name}: {path} covers bound parameters {covered}; '
                    'change the profile keys in the mode instead')
            set_path(tree, path, copy.deepcopy(value), create=True)
    return tree


def load_yaml(path: Union[str, Path]) -> Any:
    """读取 UTF-8 文本并使用 safe_load 解析 YAML.

    Args:
        path: 输入文件路径。

    Returns:
        任意 YAML 解析值，可能为 None；不在此要求映射形状。

    Raises:
        RenderError: IO 或 YAML 解析失败。
        UnicodeError: UTF-8 解码失败，不统一包装为 RenderError。
    """
    try:
        return yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        raise RenderError(f'{path}: cannot read: {error}') from error


def render_files(
    base_path: Union[str, Path],
    bindings_path: Union[str, Path],
    profile_path: Union[str, Path],
    mode_path: Optional[Union[str, Path]] = None,
) -> Dict[str, Any]:
    """依次读取基础树、档案、绑定表及可选模式后渲染.

    输入按普通路径依次读取，没有文件快照或跨文件事务。

    Args:
        base_path: 含绑定标记的基础参数 YAML。
        bindings_path: 绑定表 YAML。
        profile_path: 机器人档案 YAML。
        mode_path: 可选模式叠加 YAML。

    Returns:
        渲染参数树，不写文件或启动节点。

    Raises:
        RenderError: 已处理的读取、结构或一致性错误。
        UnicodeError: 输入不能按 UTF-8 解码。
    """
    base = load_yaml(base_path)
    if not isinstance(base, dict):
        raise RenderError(f'{base_path}: the base parameter tree must be a mapping')
    profile = load_yaml(profile_path)
    if not isinstance(profile, dict):
        raise RenderError(f'{profile_path}: the robot profile must be a mapping')
    bindings = parse_bindings(load_yaml(bindings_path), bindings_path)
    mode = parse_mode(load_yaml(mode_path), mode_path) if mode_path is not None else None
    return render(base, bindings, profile, mode)


def write_parameters(path: Union[str, Path], tree: Mapping[str, Any]) -> None:
    """将参数树覆盖写入 YAML，不创建父目录或原子替换.

    Args:
        path: 目标文件，打开时截断已有内容。
        tree: 渲染参数映射；只复制最外层为 dict，不再校验参数。

    Raises:
        OSError: 目标打开或写入失败。
        yaml.YAMLError: 数据不能按 YAML 表示；失败可能已截断或部分写入目标。
    """
    with Path(path).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(dict(tree), handle, allow_unicode=True, sort_keys=False)
