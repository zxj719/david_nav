"""Bounded evidence intake and real-process audit, without ROS or signals."""

import json
from types import SimpleNamespace

import pytest

from nav_sim.selftest_trace import audit_processes, EvidenceTrace, reached


@pytest.mark.parametrize('raw', ['[]', '{"x":1,"x":2}', '{"x":NaN}',
                                 '{"x":1e999}', '{"x":' + '[' * 34 + '0' + ']' * 34 + '}'])
def test_invalid_transport_frame_is_never_retained(tmp_path, raw):
    """Reject ambiguous/nonfinite/non-object/deep frames before appending evidence."""
    with EvidenceTrace(tmp_path) as trace:
        with pytest.raises(ValueError):
            trace.receive('ws', raw)
        assert trace.ws == []


def test_trace_limits_and_exclusive_output(tmp_path):
    """A finite trace cannot silently discard evidence or overwrite a prior experiment."""
    with EvidenceTrace(tmp_path, max_bytes=100, max_records=2) as trace:
        trace.receive('ws', '{"event_type":"status"}')
        trace.receive('nav', '{"data":{}}')
        with pytest.raises(ValueError, match='limit'):
            trace.receive('ws', '{}')
        assert len(trace.ws) == len(trace.nav) == 1
    with pytest.raises(FileExistsError):
        EvidenceTrace(tmp_path)
    assert len((tmp_path / 'observations.jsonl').read_text().splitlines()) == 2


def test_trace_single_frame_and_total_bytes_are_bounded(tmp_path):
    """The transport and aggregate memory caps apply before storage."""
    with EvidenceTrace(tmp_path, max_bytes=40) as trace:
        with pytest.raises(ValueError):
            trace.receive('ws', '{"data":"' + 'x' * 65536 + '"}')
        trace.receive('ws', '{"value":"1234567890"}')
        with pytest.raises(ValueError, match='limit'):
            trace.receive('ws', '{"value":"1234567890"}')


def _scene(tmp_path):
    processes = (SimpleNamespace(name='controller', ros_node=True),
                 SimpleNamespace(name='navigation_executor', ros_node=False))
    return SimpleNamespace(root=tmp_path, namespace='private',
                           graph=SimpleNamespace(processes=processes))


def _audit(tmp_path, exits=()):
    starts = [dict(event='start', pid=100, name='controller-1', node='/private/controller'),
              dict(event='start', pid=101, name='navigation_executor-2', node=None)]
    rows = starts + [dict(starts[index], event='exit', returncode=code)
                     for index, code in exits]
    (tmp_path / 'process_events.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in rows))


def test_processes_must_all_start_and_exit_normally(tmp_path):
    """A leader's normal status is not a replacement for per-child completion."""
    scene = _scene(tmp_path)
    assert not audit_processes(scene)
    _audit(tmp_path)
    assert audit_processes(scene)
    with pytest.raises(ValueError):
        audit_processes(scene, final=True)
    _audit(tmp_path, ((0, 0), (1, 130)))
    assert audit_processes(scene, final=True)
    with pytest.raises(ValueError):
        audit_processes(scene)


@pytest.mark.parametrize('exits', [((0, 0), (1, 143)), ((0, 0), (0, 0)), ((0, -9), (1, 0))])
def test_bad_child_exit_never_counts_as_success(tmp_path, exits):
    """TERM/forced kill/duplicate exits invalidate the cleanup evidence."""
    _audit(tmp_path, exits)
    with pytest.raises(ValueError):
        audit_processes(_scene(tmp_path), final=True)


@pytest.mark.parametrize('pose', [(0, 0, 0), (1, 0, 1), (float('nan'), 0, 0), None])
def test_arrival_requires_independent_finite_measured_pose(pose):
    """A success event does not establish the planned navigation-centre position."""
    case = SimpleNamespace(goal=(1, 0, 0), xy_ok=0.1, yaw_ok=0.2)
    assert not reached(case, pose)
    assert reached(case, (1.02, 0.01, 6.283185307179586))
