"""Correlate POI and two-stage Dock observations for the planned closed-loop test.

This is an event acceptance oracle, not a simulator or a claim that Nav2 ran.
The runner must separately prove process isolation, real execution and cleanup.
ROS NavEvent records are normalized to event_id/source/event_type/severity/stamp/
data mappings; WS object frames are retained separately. There is no shared
delivery order across the two transports, and wall time need not be monotonic.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import re
from typing import Mapping, Sequence, Tuple


@dataclass(frozen=True)
class ExpectedCase:
    """One uniquely named experiment case, independent of any ROS client.

    Attributes:
        task_id: Unique request ID for this experiment, not a reused operator ID.
        sub_id: Exact integer protocol correlation field.
        target_id: Planned POI ID, never inferred from a success message.
        kind: Ordinary poi or two-stage dock; the experiment requires one of each.
        goal: Dock target x/y/yaw in metres/radians; unused for the ordinary case.
        approach_dist: Positive Dock pre-approach distance from the run configuration.
        xy_ok: Dock positional tolerance in metres.
        yaw_ok: Dock angular tolerance in radians.
    """

    task_id: str
    sub_id: int
    target_id: str
    kind: str
    goal: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    approach_dist: float = 0.5
    xy_ok: float = 0.1
    yaw_ok: float = 0.4


@dataclass(frozen=True)
class EvidenceResult:
    """Event evidence verdict, explicitly separate from overall selftest success.

    Attributes:
        complete: Both expected cases have complete, consistent event evidence.
        completed_cases: Both case IDs only after the complete event evidence passes.
        errors: Bounded diagnostic labels; no raw event payloads are echoed.
    """

    complete: bool
    completed_cases: Tuple[str, ...]
    errors: Tuple[str, ...]


def _number(value):
    """检查证据值为内置有限数值，排除布尔值与超大整数转换溢出.

    Args:
        value: 待判断的已解码字段，不尝试把字符串转换为数字。

    Returns:
        类型严格为 int 或 float 且可通过有限性检查时返回 True。
    """
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _validate(cases):
    """校验普通 POI 后两段 Dock 的固定实验身份、位姿及容差.

    Args:
        cases: 两个 ExpectedCase；目标位置/距离为米，角度/角容差为弧度。

    Raises:
        ValueError: 用例数量/顺序、标识、整数 sub_id、有限位姿、正容差无效，
            或两个用例复用了请求/POI 标识。
    """
    if (len(cases) != 2 or not all(isinstance(case, ExpectedCase) for case in cases)
            or tuple(case.kind for case in cases) != ('poi', 'dock')):
        raise ValueError('selftest requires one POI followed by one two-stage Dock case')
    for case in cases:
        if (not all(isinstance(value, str) and value.strip() and len(value) <= 128
                    for value in (case.task_id, case.target_id))
                or type(case.sub_id) is not int or not isinstance(case.goal, (tuple, list))
                or len(case.goal) != 3
                or not all(_number(value) for value in case.goal)
                or not all(_number(value) and value > 0 for value in (
                    case.approach_dist, case.xy_ok, case.yaw_ok))):
            raise ValueError('invalid expected identity, pose or tolerance')
    if len({case.task_id for case in cases}) != 2 or len({case.target_id for case in cases}) != 2:
        raise ValueError('ordinary and Dock cases require distinct request and POI IDs')


def _nav_records(cases, records, errors):
    """按任务归组导航观察，检查来源、序号和重复事件而不重排接收顺序.

    Args:
        cases: 已验证的预期用例。
        records: 归一化 NavEvent 序列，data 是映射，stamp 使用秒。
        errors: 原地追加有界标签的错误列表，不写入原始负载。

    Returns:
        请求 ID 到序号、事件名、来源、数据四元组列表的映射。
        相同事件 ID 的一致副本去重；发现错误后仍可保留记录供后续诊断，
        调用方必须同时检查 errors，不能仅凭返回列表判成功。
    """
    selected = {case.task_id: [] for case in cases}
    seen, producer, last_sequence = {}, None, 0
    for record in records:
        if not isinstance(record, Mapping) or not isinstance(record.get('data'), Mapping):
            errors.append('invalid_nav_record')
            continue
        data, source = record['data'], record.get('source')
        task = data.get('task_id')
        if (not isinstance(task, str) or task not in selected) and source not in ('tasks', 'dock'):
            continue
        if not isinstance(task, str) or task not in selected:
            errors.append('unexpected_task_in_isolated_trace')
            continue
        name = record.get('event_type')
        if (not isinstance(name, str) or not _number(record.get('stamp'))
                or source not in ('tasks', 'dock')
                or (source == 'dock') != name.startswith('dock_')):
            errors.append('invalid_nav_envelope')
            continue
        event_id = record.get('event_id')
        if (not isinstance(event_id, str) or len(event_id) > 128
                or re.fullmatch(r'[a-f0-9]{32}:[1-9][0-9]*', event_id) is None):
            errors.append('invalid_event_identity')
            continue
        try:
            encoded = json.dumps(record, sort_keys=True, allow_nan=False)
        except (ValueError, TypeError, RecursionError):
            errors.append('invalid_event_payload')
            continue
        if len(encoded) > 65536:
            errors.append('oversized_event_payload')
            continue
        if event_id in seen:
            if seen[event_id] != encoded:
                errors.append('conflicting_duplicate_event')
            continue
        seen[event_id] = encoded
        identity, sequence = event_id.split(':')
        sequence = int(sequence)
        if producer is not None and producer != identity:
            errors.append('task_owner_changed')
        producer = identity
        if sequence <= last_sequence:
            errors.append('out_of_order_nav_sequence')
        last_sequence = sequence
        if record.get('severity') != 'info':
            errors.append('nonhealthy_task_observation')
        selected[task].append((sequence, name, source, data))
    return selected


def _pose(data, expected):
    """按紧容差比对证据中的目标位姿，角度允许相差整周.

    这是配置/事件目标一致性检查，不是实机到点容差。
    x/y 逐轴比较，yaw 使用 remainder 归一；三者绝对差须不超过 1e-8。

    Args:
        data: 含 x、y、yaw 的候选字段字典，缺失或非有限值判为不匹配。
        expected: 已校验的目标三元组，距离为米、角度为弧度。

    Returns:
        三个字段均有效且在一致性容差内时返回 True。
    """
    values = tuple(data.get(key) for key in ('x', 'y', 'yaw'))
    return (all(_number(value) for value in values)
            and abs(values[0] - expected[0]) <= 1e-8
            and abs(values[1] - expected[1]) <= 1e-8
            and abs(math.remainder(values[2] - expected[2], 2 * math.pi)) <= 1e-8)


def _dock(case, records, errors):
    """核对健康且无需旋转修正的两段 Dock 事件链及最终实测容差.

    Args:
        case: 含目标、预备距离和容差的预期 Dock 用例（米、弧度）。
        records: 已按此任务归组的导航记录，保持接收顺序。
        errors: 原地追加顺序、位姿、静态净空或停稳实测不符的错误标签。

    此函数只验证收到的观察，不独立证明实际运动、进程隔离或资源收尾。
    """
    dock = [(name, data) for _, name, source, data in records if source == 'dock']
    names = [name for name, _ in dock]
    if names != ['dock_started', 'dock_approach_checked', 'dock_stage_started',
                 'dock_stage_started', 'dock_measured']:
        errors.append(case.task_id + ':incomplete_or_unexpected_dock_sequence')
        return
    start, check, approach, final, measured = [data for _, data in dock]
    x, y, yaw = case.goal
    prepose = (x - case.approach_dist * math.cos(yaw),
               y - case.approach_dist * math.sin(yaw), yaw)
    if not (_pose(start, case.goal) and _pose(check, prepose)
            and _pose(approach, prepose) and _pose(final, case.goal)):
        errors.append(case.task_id + ':dock_target_mismatch')
    if check.get('clear') is not True or check.get('occupied_cell') is not None:
        errors.append(case.task_id + ':approach_not_proven_clear')
    if approach.get('stage') != 'approach' or final.get('stage') != 'final':
        errors.append(case.task_id + ':dock_stage_order')
    if (measured.get('settled') is not True
            or not _number(measured.get('exy')) or not _number(measured.get('eyaw'))
            or not 0 <= measured['exy'] <= case.xy_ok
            or abs(measured['eyaw']) > case.yaw_ok):
        errors.append(case.task_id + ':dock_measurement_not_acceptable')


def _case(case, records, errors):
    """验证单个任务的准入、代次、阶段与成功释放所有权边界.

    Args:
        case: 已校验身份及容差的预期用例。
        records: 此任务的归组导航记录，序号顺序已经由上层检查。
        errors: 原地追加关联、入口、阶段或所有权不符的标签列表。

    Returns:
        准入序号、成功序号和代次三元组；缺少唯一边界或有效代次时为 None。
        返回三元组不等于通过，调用方还须检查 errors 是否新增。
    """
    tasks = [(seq, name, data) for seq, name, source, data in records if source == 'tasks']
    names = [name for _, name, _ in tasks]
    if names.count('task_admitted') != 1 or names.count('task_succeeded') != 1:
        errors.append(case.task_id + ':missing_or_duplicate_task_boundary')
        return None
    admission = next(row for row in tasks if row[1] == 'task_admitted')
    success = next(row for row in tasks if row[1] == 'task_succeeded')
    generation = admission[2].get('generation')
    if type(generation) is not int or generation <= 0:
        errors.append(case.task_id + ':invalid_generation')
        return None
    if records[0][0] != admission[0] or records[-1][0] != success[0]:
        errors.append(case.task_id + ':premature_or_late_task_boundary')
    for _, _, _, data in records:
        if (type(data.get('generation')) is not int or data['generation'] != generation
                or type(data.get('sub_id')) is not int or data['sub_id'] != case.sub_id):
            errors.append(case.task_id + ':correlation_changed')
    for _, name, data in tasks:
        if name not in ('task_admitted', 'task_phase', 'task_progress', 'task_succeeded'):
            errors.append(case.task_id + ':unexpected_task_event')
        if data.get('action') != 'navigate_to' or data.get('ingress_source') != 'websocket':
            errors.append(case.task_id + ':wrong_task_ingress')
        if name != 'task_admitted' and data.get('target_id') != case.target_id:
            errors.append(case.task_id + ':wrong_poi')
        if data.get('ownership_released') is not (name == 'task_succeeded'):
            errors.append(case.task_id + ':ownership_not_proven')
    if success[2].get('state') != 'idle':
        errors.append(case.task_id + ':not_idle_after_success')
    phases = [data for _, name, data in tasks if name == 'task_phase']
    if not phases or phases[-1].get('phase') != 'to_anchor':
        errors.append(case.task_id + ':missing_anchor_phase')
    dock = [record for record in records if record[2] == 'dock']
    if case.kind == 'dock':
        _dock(case, records, errors)
        if dock and phases:
            phase_index = next(seq for seq, name, _ in tasks if name == 'task_phase')
            if phase_index >= dock[0][0]:
                errors.append(case.task_id + ':dock_before_task_phase')
    elif dock:
        errors.append(case.task_id + ':ordinary_poi_routed_to_dock')
    return admission[0], success[0], generation


def _ws(cases, records, errors):
    """检查两用例 WS 任务事件的关联与唯一到达顺序，不混入状态/心跳帧.

    Args:
        cases: 按普通 POI、Dock 排列的预期用例。
        records: 独立 WS 接收顺序下的已解码帧，不与 ROS 时间强行合并排序。
        errors: 原地追加未知任务、关联错误、暂停/失败或到达顺序异常标签。
    """
    selected = {case.task_id: case for case in cases}
    arrivals = []
    for record in records:
        if not isinstance(record, Mapping):
            errors.append('invalid_ws_record')
            continue
        name = record.get('event_type')
        if name not in ('on_arrived', 'on_error', 'on_progress'):
            continue  # Status and heartbeat frames are not task-completion evidence.
        task = record.get('request_id')
        if not isinstance(task, str) or task not in selected:
            errors.append('unexpected_ws_request')
            continue
        if type(record.get('sub_id')) is not int or record['sub_id'] != selected[task].sub_id:
            errors.append('wrong_ws_subtask')
        if task in arrivals:
            errors.append('ws_after_terminal')
        if name == 'on_error' or name == 'on_progress' and record.get('status') == 'paused':
            errors.append('ws_task_failure_or_pause')
        if name == 'on_arrived':
            arrivals.append(task)
    if arrivals != [case.task_id for case in cases]:
        errors.append('missing_duplicate_or_out_of_order_arrival')


def check_evidence(
    cases: Sequence[ExpectedCase], nav_events: Sequence[Mapping[str, object]],
    ws_events: Sequence[Mapping[str, object]],
) -> EvidenceResult:
    """Verify the strict healthy, zero-spin POI then two-stage Dock event sequence.

    Args:
        cases: One ordinary case then one Dock case with unique request/POI IDs.
        nav_events: Normalized ROS observations in receive order; duplicates must match.
        ws_events: Decoded object frames from the private agent connection, in order.

    Returns:
        Evidence-only verdict. Missing observations fail closed, even if the
        robot actually moved correctly; this does not claim full selftest success.

    Raises:
        ValueError: The expected experiment is malformed or does not contain both cases.
    """
    _validate(cases)
    if len(nav_events) > 50000 or len(ws_events) > 50000:
        return EvidenceResult(False, (), ('event_budget_exceeded',))
    errors = []
    records = _nav_records(cases, nav_events, errors)
    completed, boundaries = [], []
    for case in cases:
        before = len(errors)
        boundary = _case(case, records[case.task_id], errors)
        if boundary is not None:
            boundaries.append(boundary)
        if len(errors) == before and boundary is not None:
            completed.append(case.task_id)
    if len(boundaries) == 2 and (
            boundaries[0][1] >= boundaries[1][0] or boundaries[0][2] >= boundaries[1][2]):
        errors.append('cases_overlap_or_owner_generation_did_not_advance')
    _ws(cases, ws_events, errors)
    if errors:
        return EvidenceResult(False, (), tuple(dict.fromkeys(errors))[:100])
    return EvidenceResult(True, tuple(completed), ())
