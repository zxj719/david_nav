"""Render node parameter files from a robot profile (refactor plan §12).

Rendering joins three inputs:

* a base parameter tree (``nav_bringup/config/nav2_base.yaml``) in which every
  robot-specific value is a binding marker such as ``<bind:frames.nav_base>``;
* a binding table that lists every parameter path each profile key is written
  to, so parameters that must be equal are equal by construction (plan §12.3);
* the robot profile deep-merged with a mode overlay (``modes/<mode>.yaml``),
  whose ``nav2_overrides`` may set tuning parameters that are not profile keys.

Markers and table must agree exactly: a marker without a binding, a bound path
that does not hold its marker, or an override of a bound parameter is an
error. Parameter paths separate keys with ``/`` because Nav2 parameter names
contain dots (``controller_server/ros__parameters/FollowPath/BaseObstacle.scale``);
integer segments index lists.
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
    """The inputs of a render are malformed or inconsistent."""


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
    """Where one profile key is written.

    Attributes:
        key: Dotted profile key, for example ``frames.nav_base``.
        paths: Parameter paths that receive the value.
        value_format: Name in :data:`VALUE_FORMATS` applied to the profile value.
    """

    key: str
    paths: Tuple[str, ...]
    value_format: str = 'value'


@dataclass(frozen=True)
class ModeOverlay:
    """The differences of one mode from the robot profile.

    Attributes:
        name: Mode name.
        profile: Nested profile values merged over the robot profile.
        nav2_overrides: Parameter path to value, applied after the bindings.
    """

    name: str
    profile: Mapping[str, Any]
    nav2_overrides: Mapping[str, Any]


def split_path(path: str) -> List[str]:
    """Split a parameter path into its segments.

    Args:
        path: ``/``-separated parameter path.

    Returns:
        The segments in order.

    Raises:
        RenderError: If the path or one of its segments is empty.
    """
    segments = path.split(PATH_SEPARATOR) if isinstance(path, str) else []
    if not segments or any(not segment for segment in segments):
        raise RenderError(f'invalid parameter path {path!r}')
    return segments


def get_path(tree: Any, path: str) -> Any:
    """Read a value from a parameter tree.

    Args:
        tree: Nested mappings and lists.
        path: Parameter path.

    Returns:
        The value at the path.

    Raises:
        RenderError: If the path does not exist.
    """
    node = tree
    for segment in split_path(path):
        node = _child(node, segment, path)
    return node


def set_path(tree: Any, path: str, value: Any, create: bool = False) -> None:
    """Write a value into a parameter tree.

    Args:
        tree: Nested mappings and lists, modified in place.
        path: Parameter path.
        value: Value to write.
        create: Whether the last segment may add a new mapping key; parents
            must always exist.

    Raises:
        RenderError: If the path does not exist and may not be created.
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
    """Find binding markers in a parameter tree.

    Args:
        tree: Nested mappings and lists.
        prefix: Path of ``tree`` inside a larger tree.

    Returns:
        Mapping of each marker path to its profile key.

    Raises:
        RenderError: If a marker is malformed or a mapping key contains ``/``.
    """
    if isinstance(tree, str):
        if not tree.startswith(_MARKER_PREFIX):
            return {}
        match = _MARKER_RE.match(tree)
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
    """Merge nested mappings.

    Args:
        base: Values to start from.
        overlay: Values that win; lists and scalars replace, mappings merge.

    Returns:
        A new mapping; the inputs are not modified.
    """
    merged = copy.deepcopy(dict(base))
    for key, value in overlay.items():
        if isinstance(value, Mapping) and isinstance(merged.get(key), Mapping):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def profile_value(profile: Mapping[str, Any], key: str) -> Any:
    """Read a dotted key from a nested profile.

    Args:
        profile: Robot profile.
        key: Dotted profile key.

    Returns:
        The value.

    Raises:
        RenderError: If the profile has no value for the key.
    """
    node: Any = profile
    for part in key.split('.'):
        if not isinstance(node, Mapping) or part not in node:
            raise RenderError(f'the profile has no value for {key}')
        node = node[part]
    return node


def parse_bindings(data: Any, source: Union[str, Path]) -> Dict[str, Binding]:
    """Validate the parsed content of a binding table.

    Args:
        data: Content of a binding table as loaded from YAML.
        source: Where the data came from, used in messages.

    Returns:
        Bindings by profile key.

    Raises:
        RenderError: If the table is malformed or two keys bind the same path.
    """
    if not isinstance(data, dict) or data.get('schema_version') != BINDINGS_SCHEMA_VERSION:
        raise RenderError(f'{source}: expected schema_version {BINDINGS_SCHEMA_VERSION}')
    entries = data.get('bindings')
    if not isinstance(entries, dict) or not entries:
        raise RenderError(f'{source}: bindings must be a non-empty mapping')
    owners: Dict[str, str] = {}
    bindings: Dict[str, Binding] = {}
    for key, entry in entries.items():
        if not isinstance(key, str) or not _KEY_RE.match(key):
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
    """Validate the parsed content of a mode overlay.

    Args:
        data: Content of a mode overlay as loaded from YAML.
        source: Where the data came from, used in messages.

    Returns:
        The overlay.

    Raises:
        RenderError: If the overlay is malformed.
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
    """Render a parameter tree.

    Args:
        base: Base parameter tree with binding markers.
        bindings: Binding table by profile key.
        profile: Robot profile.
        mode: Mode overlay, if any.

    Returns:
        A new parameter tree without markers; the inputs are not modified.

    Raises:
        RenderError: If markers and bindings disagree, a profile value is
            missing or has the wrong shape, or an override targets a bound or
            missing parameter.
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
            covered = sorted(
                bound_path for bound_path in bound
                if bound_path == path or bound_path.startswith(path + PATH_SEPARATOR))
            if covered:
                raise RenderError(
                    f'mode {mode.name}: {path} covers bound parameters {covered}; '
                    'change the profile keys in the mode instead')
            set_path(tree, path, copy.deepcopy(value), create=True)
    return tree


def load_yaml(path: Union[str, Path]) -> Any:
    """Read a YAML file.

    Args:
        path: The file.

    Returns:
        The parsed content.

    Raises:
        RenderError: If the file cannot be read or parsed.
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
    """Render a parameter tree from files.

    Args:
        base_path: Base parameter tree with markers.
        bindings_path: Binding table.
        profile_path: Robot profile.
        mode_path: Mode overlay, if any.

    Returns:
        The rendered parameter tree.

    Raises:
        RenderError: If an input is unreadable, malformed or inconsistent.
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
    """Write a rendered parameter tree as YAML.

    Args:
        path: Destination file.
        tree: Rendered parameter tree.
    """
    with Path(path).open('w', encoding='utf-8') as handle:
        yaml.safe_dump(dict(tree), handle, allow_unicode=True, sort_keys=False)
