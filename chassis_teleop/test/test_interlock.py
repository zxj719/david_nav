"""Check publishers through fake graph views only; no ROS context is initialized."""

from types import SimpleNamespace

import pytest

from chassis_teleop.interlock import CommandConflict, wait_for_clear_topics


def _hooks(views):
    clock = [0.0]
    observed = []

    def query(topic):
        observed.append(topic)
        return views(clock[0], topic)

    def spin(period):
        clock[0] += period

    return SimpleNamespace(query=query, spin=spin, clock=lambda: clock[0], observed=observed)


def test_empty_graph_is_observed_for_full_discovery_window():
    """An initial empty discovery result is not treated as immediate permission."""
    hooks = _hooks(lambda *_: [])
    wait_for_clear_topics(('/drive', '/nav'), hooks.query, hooks.spin, hooks.clock, 1.0)
    assert hooks.clock() >= 1.0
    assert set(hooks.observed) == {'/drive', '/nav'}


def test_late_publisher_rejects_startup():
    """A publisher appearing during discovery cannot be ignored after the first sample."""
    hooks = _hooks(lambda now, topic: ['adapter'] if now >= 0.2 and topic == '/drive' else [])
    with pytest.raises(CommandConflict, match='/drive'):
        wait_for_clear_topics(('/drive', '/nav'), hooks.query, hooks.spin, hooks.clock, 1.0)


def test_any_publisher_is_rejected_without_name_based_exemptions():
    """Even an endpoint named like our adapter is not proof of ownership."""
    hooks = _hooks(lambda *_: ['chassis_adapter'])
    with pytest.raises(CommandConflict):
        wait_for_clear_topics(('/drive',), hooks.query, hooks.spin, hooks.clock, 1.0)


def test_query_failure_is_not_an_empty_graph():
    """Discovery errors fail closed before any keyboard publisher is constructed."""
    def query(_):
        raise RuntimeError('discovery unavailable')

    with pytest.raises(CommandConflict, match='discovery'):
        wait_for_clear_topics(('/drive',), query, lambda _: None, lambda: 0.0, 1.0)


@pytest.mark.parametrize('topics,duration', [
    ([], 1.0), (['relative'], 1.0), (['/drive'], 0.0), (['/drive'], float('nan'))])
def test_invalid_guard_settings_fail_before_queries(topics, duration):
    """Neither an empty topic set nor a zero discovery window disables the guard."""
    with pytest.raises(ValueError):
        wait_for_clear_topics(topics, lambda _: pytest.fail('unexpected query'),
                              lambda _: None, lambda: 0.0, duration)
