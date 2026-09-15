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
    try:
        return render.get_path(tree, path)
    except render.RenderError:
        issues.append(ValidationIssue(rule, 'error', f'missing rendered parameter {path}'))
        return _MISSING


def _check_global_frames(
    tree: Mapping[str, Any], issues: List[ValidationIssue]
) -> None:
    """Require recovery behavior and the local costmap to use one frame."""
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
    """Require navigation speeds and the stationary floor to fit hard limits."""
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
    """Check paired position tolerances and heading control in precise modes."""
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
    """Require trinary loading so the Lightning unknown value stays unknown."""
    mode = map_data.get('mode') if isinstance(map_data, Mapping) else None
    if mode != 'trinary':
        issues.append(ValidationIssue(
            'map_trinary', 'error', f'map YAML mode must be trinary, got {mode!r}'))


def _check_teleop_limits(
    profile: Mapping[str, Any], issues: List[ValidationIssue]
) -> None:
    """Require the highest teleop gears to fit the adapter hard limits."""
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
    """Warn when a differential-drive tree requests nonzero lateral motion."""
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
    """Report the URDF offset when POIs and navigation use different frames."""
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
    """Parse one URDF origin into a rotation matrix and translation."""
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
    """Convert fixed-axis roll, pitch and yaw to a rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _rotation_to_rpy(rotation: List[List[float]]) -> Tuple[float, float, float]:
    """Convert a rotation matrix to fixed-axis roll, pitch and yaw."""
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
    """Compose source-to-middle and middle-to-target transforms."""
    first_rotation, first_translation = first
    second_rotation, second_translation = second
    rotation = _matrix_multiply(first_rotation, second_rotation)
    shifted = _matrix_vector(first_rotation, second_translation)
    translation = [a + b for a, b in zip(first_translation, shifted)]
    return rotation, translation


def _inverse(
    transform: Tuple[List[List[float]], List[float]]
) -> Tuple[List[List[float]], List[float]]:
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
