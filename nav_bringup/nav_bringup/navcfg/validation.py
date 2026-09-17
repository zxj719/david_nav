"""Cross-file validation of robot navigation configuration.

Schema checks prove that individual profile values have the declared type and
range. This module adds the relationships from refactor plan §12.4: frame and
limit invariants, precise-mode controller consistency, map semantics, teleop
limits, differential-drive behavior-tree warnings, and the POI capture offset.

The module performs no ROS communication and starts no processes.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any, Dict, List, Mapping, Optional, Tuple
import xml.etree.ElementTree as ET

from nav_bringup.navcfg import render
from nav_bringup.navcfg.schema import ParameterCatalog

_MISSING = object()
_LATERAL_ATTRIBUTES = frozenset({'lateral_speed', 'linear_y', 'speed_y', 'velocity_y'})


@dataclass(frozen=True)
class ValidationIssue:
    """One configuration validation result that needs attention.

    Attributes:
        rule: Stable name of the rule that produced the issue.
        severity: ``error`` rejects the configuration; ``warn`` does not.
        message: Human-readable explanation containing the values involved.
    """

    rule: str
    severity: str
    message: str


@dataclass(frozen=True)
class FrameOffset:
    """Pose of a target frame expressed in a source frame.

    Attributes:
        xyz: Translation in metres.
        rpy: Fixed-axis roll, pitch and yaw in radians.
    """

    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]


def validate_configuration(
    catalog: ParameterCatalog,
    profile: Mapping[str, Any],
    mode: render.ModeOverlay,
    rendered: Mapping[str, Any],
    map_data: Mapping[str, Any],
    behavior_trees: Mapping[str, str],
    urdf_text: str,
) -> List[ValidationIssue]:
    """Apply every offline rule from refactor plan §12.4.

    Args:
        catalog: Aggregated per-package parameter declarations.
        profile: Robot profile before the mode overlay.
        mode: Selected mode overlay.
        rendered: Nav2 parameter tree rendered for ``profile`` and ``mode``.
        map_data: Parsed map YAML selected by the site profile.
        behavior_trees: XML text keyed by a path or descriptive file name.
        urdf_text: Robot URDF used to calculate the POI-to-navigation offset.

    Returns:
        Errors and warnings in deterministic rule order.
    """
    effective = render.deep_merge(profile, mode.profile)
    issues = [
        ValidationIssue('schema', issue.severity, issue.message)
        for issue in catalog.check_profile(effective)
    ]
    _check_global_frames(rendered, issues)
    _check_navigation_limits(effective, issues)
    _check_precise_controller(effective, rendered, issues)
    _check_map_mode(map_data, issues)
    _check_teleop_limits(effective, issues)
    _check_behavior_trees(effective, behavior_trees, issues)
    _check_poi_offset(effective, urdf_text, issues)
    return issues


def _profile_value(
    profile: Mapping[str, Any],
    key: str,
    rule: str,
    issues: List[ValidationIssue],
) -> Any:
    """读取点分档案键，缺失时累积错误而非立即中止其余规则.

    Args:
        profile: 待验证的档案映射。
        key: 点分参数键。
        rule: 本条检查规则的诊断标识。
        issues: 原地追加缺失错误的列表。

    Returns:
        原值，或专用 _MISSING 哨兵；不会用 None 混淆合法配置值。
    """
    try:
        return render.profile_value(profile, key)
    except render.RenderError:
        issues.append(ValidationIssue(rule, 'error', f'missing profile key {key}'))
        return _MISSING


def _rendered_value(
    tree: Mapping[str, Any],
    path: str,
    rule: str,
    issues: List[ValidationIssue],
) -> Any:
    """读取渲染树路径，缺失时记录所属规则的错误.

    Args:
        tree: 已渲染参数树。
        path: 目标参数路径。
        rule: 本条检查规则的标识。
        issues: 原地追加缺失诊断的列表。

    Returns:
        原值或 _MISSING 哨兵，不修改参数树。
    """
    try:
        return render.get_path(tree, path)
    except render.RenderError:
        issues.append(ValidationIssue(rule, 'error', f'missing rendered parameter {path}'))
        return _MISSING


def _check_global_frames(
    tree: Mapping[str, Any], issues: List[ValidationIssue]
) -> None:
    """检查恢复行为与局部代价图是否使用相同的全局参考帧.

    Args:
        tree: 已渲染 Nav2 参数树，不在此猜测或替换帧名称。
        issues: 原地追加缺失参数或帧不一致错误的列表。
    """
    rule = 'behavior_local_global_frame'
    behavior = _rendered_value(
        tree, 'behavior_server/ros__parameters/global_frame', rule, issues)
    local = _rendered_value(
        tree, 'local_costmap/local_costmap/ros__parameters/global_frame', rule, issues)
    if _MISSING not in (behavior, local) and behavior != local:
        issues.append(ValidationIssue(
            rule, 'error',
            f'behavior_server.global_frame {behavior!r} != local_costmap.global_frame {local!r}'))


def _check_at_most(
    profile: Mapping[str, Any],
    lower_key: str,
    upper_key: str,
    rule: str,
    issues: List[ValidationIssue],
) -> None:
    """检查同单位的两个配置数值满足下限项不大于上限项.

    Args:
        profile: 已读取的机器人档案。
        lower_key: 应较小或相等的参数键。
        upper_key: 应较大或相等的参数键。
        rule: 诊断规则标识。
        issues: 原地追加缺失、非数值或顺序不符错误的列表。

    不在此转换单位或验证有限性；bool 明确不是数值，缺键后不重复比较。
    """
    lower = _profile_value(profile, lower_key, rule, issues)
    upper = _profile_value(profile, upper_key, rule, issues)
    if _MISSING in (lower, upper):
        return
    invalid = [
        key for key, value in ((lower_key, lower), (upper_key, upper))
        if isinstance(value, bool) or not isinstance(value, (int, float))
    ]
    if invalid:
        issues.append(ValidationIssue(
            rule, 'error', f'{", ".join(invalid)} must be numeric'))
    elif lower > upper:
        issues.append(ValidationIssue(
            rule, 'error', f'{lower_key}={lower!r} exceeds {upper_key}={upper!r}'))


def _check_navigation_limits(
    profile: Mapping[str, Any], issues: List[ValidationIssue]
) -> None:
    """检查导航速度和底盘最小原地旋转速度是否超过底盘硬限.

    这里的 min_stationary_angular_z 是底盘最小原地旋转速度，
    不是 Dock 测量使用的停稳阈值；不修改任一配置值。

    Args:
        profile: 已合并模式的机器人档案；比较双方必须采用相同速度单位。
        issues: 原地追加缺键、非数值或超过硬限错误的列表。
    """
    rule = 'navigation_hard_limits'
    _check_at_most(
        profile, 'limits.max_vel_x', 'chassis.adapter.hard_max_linear_x', rule, issues)
    _check_at_most(
        profile, 'limits.max_vel_theta', 'chassis.adapter.hard_max_angular_z', rule, issues)
    _check_at_most(
        profile, 'chassis.adapter.min_stationary_angular_z',
        'chassis.adapter.hard_max_angular_z', rule, issues)


def _check_precise_controller(
    profile: Mapping[str, Any],
    tree: Mapping[str, Any],
    issues: List[ValidationIssue],
) -> None:
    """按目标角度容差判断是否需要精确模式的位置与朝向检查.

    角度容差小于 2π 时才比较两个位置容差并要求 RotateToGoal；
    这里不按模式名称判断，也不负责数值范围的完整 schema 校验。

    Args:
        profile: 已合并模式的机器人档案，目标角度容差单位为弧度。
        tree: 已渲染 Nav2 参数树，不会原地修正控制器参数。
        issues: 原地追加缺键、类型、位置容差不一致或缺 critic 错误的列表。
    """
    rule = 'precise_controller'
    yaw = _profile_value(profile, 'goal.yaw_tolerance', rule, issues)
    if yaw is _MISSING:
        return
    if isinstance(yaw, bool) or not isinstance(yaw, (int, float)):
        issues.append(ValidationIssue(
            rule, 'error', 'goal.yaw_tolerance must be numeric'))
        return
    if yaw >= math.tau:
        return
    goal_xy = _rendered_value(
        tree,
        'controller_server/ros__parameters/general_goal_checker/xy_goal_tolerance',
        rule,
        issues,
    )
    dwb_xy = _rendered_value(
        tree, 'controller_server/ros__parameters/FollowPath/xy_goal_tolerance', rule, issues)
    if _MISSING not in (goal_xy, dwb_xy) and goal_xy != dwb_xy:
        issues.append(ValidationIssue(
            rule, 'error',
            f'precise xy_goal_tolerance values differ: goal_checker={goal_xy!r}, '
            f'FollowPath={dwb_xy!r}'))
    critics = _rendered_value(
        tree, 'controller_server/ros__parameters/FollowPath/critics', rule, issues)
    if critics is not _MISSING and (
            not isinstance(critics, list) or 'RotateToGoal' not in critics):
        issues.append(ValidationIssue(
            rule, 'error',
            f'goal.yaw_tolerance={yaw!r} is below 2π but critics omit RotateToGoal'))


def _check_map_mode(map_data: Mapping[str, Any], issues: List[ValidationIssue]) -> None:
    """要求地图 YAML 显式选择 trinary，不修改地图或定位配置.

    Args:
        map_data: 已加载的地图 YAML 映射；缺少 mode 或不是映射也记录错误。
        issues: 原地追加模式不符合要求错误的列表。
    """
    mode = map_data.get('mode') if isinstance(map_data, Mapping) else None
    if mode != 'trinary':
        issues.append(ValidationIssue(
            'map_trinary', 'error', f'map YAML mode must be trinary, got {mode!r}'))


def validate_teleop_limits(profile: Mapping[str, Any]) -> List[ValidationIssue]:
    """Check keyboard ceilings without adding map or localization prerequisites.

    Args:
        profile: Fully merged robot and teleop-mode configuration.

    Returns:
        Existing hard-limit rule findings, also used by navigation validation.
    """
    issues: List[ValidationIssue] = []
    _check_teleop_limits(profile, issues)
    return issues


def _check_teleop_limits(
    profile: Mapping[str, Any], issues: List[ValidationIssue]
) -> None:
    """检查遥控最大档位速度是否落在底盘适配器硬限内.

    Args:
        profile: 已合并的档案，线速度单位为米每秒，角速度单位为弧度每秒。
        issues: 原地追加缺键、非数值或超限错误的列表；不裁剪速度配置。
    """
    rule = 'teleop_hard_limits'
    _check_at_most(
        profile, 'teleop.max_linear_x', 'chassis.adapter.hard_max_linear_x', rule, issues)
    _check_at_most(
        profile, 'teleop.max_angular_z', 'chassis.adapter.hard_max_angular_z', rule, issues)


def _check_behavior_trees(
    profile: Mapping[str, Any],
    behavior_trees: Mapping[str, str],
    issues: List[ValidationIssue],
) -> None:
    """对差速机器人 BT 中已知的横向速度属性给出离线告警.

    非差速类型跳过；缺运动学键、没有树、坏 XML 或非数值属性记录错误。
    仅扫描已列举的属性名，不能证明树没有其它横向动作，也不修改树。

    Args:
        profile: 已合并模式的档案，通过 kinematics.type 判断运动学类型。
        behavior_trees: 来源名称到 XML 文本的映射；按名称排序检查。
        issues: 原地追加错误和横向速度不接近零的警告列表；
            零值比较绝对容差为 1e-12。
    """
    rule = 'diff_drive_bt_lateral'
    kinematics = _profile_value(profile, 'kinematics.type', rule, issues)
    if kinematics is _MISSING or kinematics != 'diff_drive':
        return
    if not behavior_trees:
        issues.append(ValidationIssue(rule, 'error', 'no behavior trees were provided'))
        return
    for source, text in sorted(behavior_trees.items()):
        try:
            root = ET.fromstring(text)
        except ET.ParseError as error:
            issues.append(ValidationIssue(rule, 'error', f'{source}: invalid XML: {error}'))
            continue
        for element in root.iter():
            for name, raw_value in sorted(element.attrib.items()):
                if name not in _LATERAL_ATTRIBUTES:
                    continue
                try:
                    value = float(raw_value)
                except ValueError:
                    issues.append(ValidationIssue(
                        rule, 'error',
                        f'{source}: <{element.tag}> {name}={raw_value!r} is not numeric'))
                    continue
                if not math.isclose(value, 0.0, abs_tol=1e-12):
                    issues.append(ValidationIssue(
                        rule, 'warn',
                        f'{source}: diff_drive tree <{element.tag}> has '
                        f'{name}={raw_value!r}'))


def _check_poi_offset(
    profile: Mapping[str, Any], urdf_text: str, issues: List[ValidationIssue]
) -> None:
    """在 POI 采集帧与导航基准不同的情况下报告 URDF 固定偏移.

    只给出诊断，不重投影或写回 POI。缺失或相同帧直接结束；
    帧名类型错误及固定路径解析失败记录错误，成功求出偏移时记录警告。

    Args:
        profile: 已合并档案，分别读取 frames.poi_capture 和 frames.nav_base。
        urdf_text: 用于计算采集帧到导航基准变换的完整 URDF 文本。
        issues: 原地追加缺键、解析错误或偏移警告的列表，输出单位为米和弧度。
    """
    rule = 'poi_capture_offset'
    capture = _profile_value(profile, 'frames.poi_capture', rule, issues)
    nav_base = _profile_value(profile, 'frames.nav_base', rule, issues)
    if _MISSING in (capture, nav_base) or capture == nav_base:
        return
    if not isinstance(capture, str) or not isinstance(nav_base, str):
        issues.append(ValidationIssue(
            rule, 'error', 'frames.poi_capture and frames.nav_base must be strings'))
        return
    try:
        offset = frame_offset(urdf_text, capture, nav_base)
    except ValueError as error:
        issues.append(ValidationIssue(rule, 'error', str(error)))
        return
    xyz = ', '.join(f'{value:.4f}' for value in offset.xyz)
    rpy = ', '.join(f'{value:.4f}' for value in offset.rpy)
    issues.append(ValidationIssue(
        rule,
        'warn',
        f'frames.poi_capture {capture!r} != frames.nav_base {nav_base!r}; '
        f'{capture} -> {nav_base} offset xyz=[{xyz}] m rpy=[{rpy}] rad',
    ))


def frame_offset(urdf_text: str, source: str, target: str) -> FrameOffset:
    """Find the fixed-joint transform from one URDF frame to another.

    Args:
        urdf_text: Complete URDF XML.
        source: Frame in which the result is expressed.
        target: Frame whose pose is returned.

    Returns:
        Pose of ``target`` in ``source``.

    Raises:
        ValueError: If the XML is malformed or no fixed-joint path connects the
            two frames.
    """
    try:
        robot = ET.fromstring(urdf_text)
    except ET.ParseError as error:
        raise ValueError(f'URDF is invalid XML: {error}') from error
    graph: Dict[str, List[Tuple[str, Tuple[List[List[float]], List[float]]]]] = {}
    for joint in robot.findall('joint'):
        if joint.get('type') != 'fixed':
            continue
        parent_element = joint.find('parent')
        child_element = joint.find('child')
        if parent_element is None or child_element is None:
            continue
        parent = parent_element.get('link')
        child = child_element.get('link')
        if not parent or not child:
            continue
        transform = _origin_transform(joint.find('origin'))
        graph.setdefault(parent, []).append((child, transform))
        graph.setdefault(child, []).append((parent, _inverse(transform)))
    identity = (_identity_rotation(), [0.0, 0.0, 0.0])
    queue = deque([(source, identity)])
    visited = {source}
    while queue:
        frame, transform = queue.popleft()
        if frame == target:
            rotation, translation = transform
            return FrameOffset(tuple(translation), _rotation_to_rpy(rotation))
        for neighbor, edge in graph.get(frame, []):
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, _compose(transform, edge)))
    raise ValueError(f'URDF has no fixed-joint path from {source!r} to {target!r}')


def _origin_transform(element: Optional[ET.Element]) -> Tuple[List[List[float]], List[float]]:
    """把 URDF origin 的 xyz 与固定轴 rpy 转换为刚体变换.

    缺失 origin 或任一属性时采用对应的零值；不校验平移值有限性，
    也不验证外部模型的关节语义。

    Args:
        element: 可选的 origin XML 元素；角度采用弧度、平移采用米。

    Returns:
        三乘三旋转矩阵与三维平移列表组成的二元组。

    Raises:
        ValueError: 属性不是数值、分量数不是三，或角度无法进行三角函数计算。
    """
    if element is None:
        return _identity_rotation(), [0.0, 0.0, 0.0]
    try:
        xyz = [float(value) for value in element.get('xyz', '0 0 0').split()]
        rpy = [float(value) for value in element.get('rpy', '0 0 0').split()]
    except ValueError as error:
        raise ValueError('URDF joint origin contains a non-numeric value') from error
    if len(xyz) != 3 or len(rpy) != 3:
        raise ValueError('URDF joint origin xyz and rpy must each contain three values')
    return _rpy_to_rotation(*rpy), xyz


def _identity_rotation() -> List[List[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _rpy_to_rotation(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """按固定轴 roll、pitch、yaw 生成 Rz(yaw) Ry(pitch) Rx(roll) 矩阵.

    Args:
        roll: 绕固定 X 轴的角度，单位弧度。
        pitch: 绕固定 Y 轴的角度，单位弧度。
        yaw: 绕固定 Z 轴的角度，单位弧度。

    Returns:
        按行排列的三乘三旋转矩阵；不修改输入，也不主动校验有限性。

    Raises:
        ValueError: 三角函数无法处理给定角度，例如无穷值。
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rotation_to_rpy(rotation: List[List[float]]) -> Tuple[float, float, float]:
    """从旋转矩阵提取固定轴 rpy，在俯仰奇异处固定 roll 为零.

    反正弦输入先截到 [-1, 1]，不验证整个矩阵的维度或正交性；
    奇异判定沿用 math.isclose 的相对容差及 1e-9 绝对容差。

    Args:
        rotation: 调用方保证合法的三乘三刚体旋转矩阵，按行排列。

    Returns:
        弧度制的 roll、pitch、yaw；pitch 在 [-π/2, π/2]，
        奇异处由剩余矩阵项确定 yaw，不保证欧拉角表示唯一。
    """
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if math.isclose(abs(pitch), math.pi / 2.0, abs_tol=1e-9):
        roll = 0.0
        yaw = math.atan2(-rotation[0][1], rotation[1][1])
    else:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    return roll, pitch, yaw


def _compose(
    first: Tuple[List[List[float]], List[float]],
    second: Tuple[List[List[float]], List[float]],
) -> Tuple[List[List[float]], List[float]]:
    """组合源帧到中间帧、中间帧到目标帧的两个刚体变换.

    先将第二段平移用第一段旋转变换，再加第一段平移；
    调用方保证矩阵和向量维度正确，本函数不做模型校验。

    Args:
        first: 中间帧在源帧中的旋转矩阵与平移向量。
        second: 目标帧在中间帧中的旋转矩阵与平移向量，与第一段采用相同长度单位。

    Returns:
        目标帧在源帧中的旋转矩阵与平移列表，不修改两段输入。
    """
    first_rotation, first_translation = first
    second_rotation, second_translation = second
    rotation = _matrix_multiply(first_rotation, second_rotation)
    shifted = _matrix_vector(first_rotation, second_translation)
    translation = [a + b for a, b in zip(first_translation, shifted)]
    return rotation, translation


def _inverse(
    transform: Tuple[List[List[float]], List[float]]
) -> Tuple[List[List[float]], List[float]]:
    """对正交旋转和平移组成的刚体变换求逆，旋转用转置代替矩阵逆.

    输入必须已满足刚体旋转假设；此函数不做正交性或维度校验。

    Args:
        transform: 旋转矩阵与平移向量组成的二元组。

    Returns:
        转置旋转矩阵与其作用于负平移得到的逆变换，不修改原对象。
    """
    rotation, translation = transform
    inverse_rotation = [list(row) for row in zip(*rotation)]
    inverse_translation = [-value for value in _matrix_vector(inverse_rotation, translation)]
    return inverse_rotation, inverse_translation


def _matrix_multiply(
    left: List[List[float]], right: List[List[float]]
) -> List[List[float]]:
    return [
        [sum(left[row][index] * right[index][column] for index in range(3))
         for column in range(3)]
        for row in range(3)
    ]


def _matrix_vector(matrix: List[List[float]], vector: List[float]) -> List[float]:
    return [sum(row[index] * vector[index] for index in range(3)) for row in matrix]
