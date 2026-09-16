"""Reject incomplete or mis-correlated selftest traces without ROS or motion."""

import copy

import pytest

from nav_sim.selftest_evidence import check_evidence, ExpectedCase

PRODUCER = 'a' * 32
CASES = (
    ExpectedCase('run-poi', 0, 'poi-a', 'poi'),
    ExpectedCase('run-dock', 1, 'dock-b', 'dock', (2.0, 0.0, 0.0), 0.5, 0.1, 0.4),
)


def _trace():
    nav = []

    def add(case, name, source='tasks', **extra):
        index = CASES.index(case) + 1
        data = dict(task_id=case.task_id, sub_id=case.sub_id, generation=index)
        if source == 'tasks':
            data.update(action='navigate_to', ingress_source='websocket',
                        target_id=case.target_id, phase='to_anchor',
                        ownership_released=False, state='starting')
        data.update(extra)
        nav.append(dict(event_id=f'{PRODUCER}:{len(nav) + 1}', source=source,
                        event_type=name, severity='info', stamp=1.0, data=data))

    for case in CASES:
        add(case, 'task_admitted')
        add(case, 'task_phase')
        if case.kind == 'dock':
            add(case, 'dock_started', 'dock', x=2.0, y=0.0, yaw=0.0)
            add(case, 'dock_approach_checked', 'dock', clear=True,
                x=1.5, y=0.0, yaw=0.0)
            add(case, 'dock_stage_started', 'dock', stage='approach', x=1.5, y=0.0, yaw=0.0)
            add(case, 'dock_stage_started', 'dock', stage='final', x=2.0, y=0.0, yaw=0.0)
            add(case, 'dock_measured', 'dock', settled=True, exy=0.05, eyaw=-0.2)
        add(case, 'task_succeeded', ownership_released=True, state='idle')
    ws = [dict(event_type='on_arrived', request_id=case.task_id, sub_id=case.sub_id)
          for case in CASES]
    return nav, ws


def test_complete_event_evidence_requires_both_poi_and_two_stage_dock():
    """A complete event verdict is explicitly not proof of real Nav2 execution."""
    nav, ws = _trace()
    result = check_evidence(CASES, nav, ws)
    assert result.complete and not result.errors
    assert result.completed_cases == ('run-poi', 'run-dock')


@pytest.mark.parametrize('name', [
    'task_admitted', 'task_phase', 'dock_started', 'dock_approach_checked',
    'dock_stage_started', 'dock_measured', 'task_succeeded'])
def test_missing_required_observation_never_passes(name):
    """An arrival alone is not evidence of the mandatory internal stages."""
    nav, ws = _trace()
    nav = [event for event in nav if event['event_type'] != name]
    assert not check_evidence(CASES, nav, ws).complete


@pytest.mark.parametrize('change', [
    'generation', 'sub_id', 'target_id', 'source', 'producer', 'release',
    'blocked', 'not_settled', 'nan', 'xy', 'yaw', 'pose', 'stage_order',
    'early_success', 'extra_stage', 'duplicate_arrival', 'ws_error', 'wrong_ws_id'])
def test_corrupted_or_premature_success_evidence_is_rejected(change):
    """One plausible success line cannot repair a broken correlated sequence."""
    nav, ws = _trace()
    dock = [event for event in nav if event['data']['task_id'] == 'run-dock']
    if change in ('generation', 'sub_id', 'target_id'):
        dock[-1]['data'][change] = 'wrong'
    elif change == 'source':
        dock[-1]['source'] = 'dock'
    elif change == 'producer':
        dock[-1]['event_id'] = 'b' * 32 + ':100'
    elif change == 'release':
        dock[-1]['data']['ownership_released'] = False
    elif change == 'blocked':
        dock[3]['data']['clear'] = False
    elif change in ('not_settled', 'nan', 'xy', 'yaw'):
        key, value = {'not_settled': ('settled', False), 'nan': ('exy', float('nan')),
                      'xy': ('exy', 1.0), 'yaw': ('eyaw', 1.0)}[change]
        dock[-2]['data'][key] = value
    elif change == 'pose':
        dock[5]['data']['x'] = 99.0
    elif change == 'stage_order':
        dock[4]['data']['stage'], dock[5]['data']['stage'] = 'final', 'approach'
    elif change == 'early_success':
        nav.insert(6, nav.pop())
    elif change == 'extra_stage':
        extra = copy.deepcopy(dock[4])
        extra['event_id'] = PRODUCER + ':100'
        nav.append(extra)
    elif change == 'duplicate_arrival':
        ws.append(dict(ws[-1]))
    elif change == 'ws_error':
        ws[-1]['event_type'] = 'on_error'
    elif change == 'wrong_ws_id':
        ws[-1]['request_id'] = 'different'
    assert not check_evidence(CASES, nav, ws).complete


def test_identical_nav_delivery_duplicates_do_not_double_count_stages():
    """Deduplication uses event identity and requires exactly matching contents."""
    nav, ws = _trace()
    nav.insert(7, copy.deepcopy(nav[6]))
    assert check_evidence(CASES, nav, ws).complete
    nav[7]['data']['clear'] = False
    assert not check_evidence(CASES, nav, ws).complete


def test_other_channel_delivery_order_is_not_compared_to_wall_clock():
    """DDS and WS have no shared receipt order; only each ordered stream is checked."""
    nav, ws = _trace()
    for event in nav:
        event['stamp'] = 1000 - int(event['event_id'].split(':')[1])
    assert check_evidence(CASES, nav, ws).complete
    assert not check_evidence(CASES, nav, ws[::-1]).complete


@pytest.mark.parametrize('name', [
    'task_fault', 'task_failed', 'task_cleanup_failed', 'task_canceled',
    'task_paused', 'dock_no_pose'])
def test_failure_observations_are_not_hidden_by_later_success(name):
    """A healthy selftest must fail on task faults, cancellation or unusable pose."""
    nav, ws = _trace()
    extra = copy.deepcopy(nav[-1])
    extra.update(event_type=name, event_id=PRODUCER + ':100',
                 source='dock' if name.startswith('dock_') else 'tasks')
    nav.append(extra)
    assert not check_evidence(CASES, nav, ws).complete


def test_empty_expectations_cannot_be_a_vacuous_success():
    """The plan requires exactly one ordinary POI and one Dock case."""
    with pytest.raises(ValueError):
        check_evidence((), [], [])
    with pytest.raises(ValueError):
        check_evidence(CASES[:1], [], [])


@pytest.mark.parametrize('field,value', [
    ('sub_id', True), ('xy_ok', float('nan')), ('goal', (1.0, 2.0, float('inf'))), ('goal', None)])
def test_invalid_expectations_are_rejected(field, value):
    """Malformed test configuration cannot loosen an acceptance boundary."""
    from dataclasses import replace

    with pytest.raises(ValueError):
        check_evidence((CASES[0], replace(CASES[1], **{field: value})), [], [])


@pytest.mark.parametrize('field,value', [
    ('task_id', {}), ('generation', 'invalid'), ('generation', True), ('sub_id', [])])
def test_malformed_correlations_are_rejected_without_crashing(field, value):
    """Bad transport input must yield failed evidence rather than a parser exception."""
    nav, ws = _trace()
    nav[0]['data'][field] = value
    assert not check_evidence(CASES, nav, ws).complete


def test_event_budgets_and_non_json_values_fail_closed():
    """Oversized or non-finite evidence cannot make an unbounded acceptance scan."""
    nav, ws = _trace()
    assert not check_evidence(CASES, nav * 50001, ws).complete
    nav[0]['data']['extra'] = object()
    assert not check_evidence(CASES, nav, ws).complete


def test_early_success_fails_even_with_monotonic_producer_sequence():
    """Success cannot occur between admission and the required Dock observations."""
    nav, ws = _trace()
    nav.insert(6, nav.pop())
    for index, record in enumerate(nav, 1):
        record['event_id'] = f'{PRODUCER}:{index}'
    result = check_evidence(CASES, nav, ws)
    assert not result.complete
    assert 'run-dock:premature_or_late_task_boundary' in result.errors


def test_second_case_must_have_a_new_owner_generation():
    """Distinct task names alone do not prove a completed ownership transition."""
    nav, ws = _trace()
    for record in nav:
        record['data']['generation'] = 1
    result = check_evidence(CASES, nav, ws)
    assert not result.complete
    assert 'cases_overlap_or_owner_generation_did_not_advance' in result.errors


def test_unrelated_health_events_do_not_replace_required_task_observations():
    """Other producers may be observed but cannot contribute success evidence."""
    nav, ws = _trace()
    nav.insert(0, dict(source='localization_health', event_type='healthy', data={}))
    assert check_evidence(CASES, nav, ws).complete
    assert not check_evidence(CASES, nav[:1], ws).complete
