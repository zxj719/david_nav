"""Keep the Phase 7 isolation proposal explicit without removing working legacy entry points."""

from pathlib import Path, PurePosixPath

import pytest
import yaml

from nav_bringup.launching.graph import build_graph, GraphOptions

SOURCE = Path(__file__).resolve().parents[2]


def _inventory():
    return yaml.safe_load((SOURCE / 'docs/legacy_inventory.yaml').read_text())


def test_proposal_covers_each_plan_family_without_implicit_approval():
    """Every named PLAN family has a concrete list; a proposal never authorizes removal."""
    document = _inventory()
    assert document['schema_version'] == 1
    assert document['baseline_commit'] == '55034e9'
    assert document['state'] == 'proposal_pending_review'
    assert document['approved'] is False
    groups = document['groups']
    assert {group['id'] for group in groups} == {
        'separator', 'warmup', 'frontier', 'collision', 'console', 'semantic',
        'shell_entries', 'waypoint_dead_reckoning',
    }
    for group in groups:
        assert group['files'] and group['reason'] and group['references']
        assert group['decision'] == 'retain_until_review'


def test_proposed_files_and_destinations_are_exact_and_not_already_moved():
    """The review list names existing files and unique in-repository destinations only."""
    sources, destinations = [], []
    for group in _inventory()['groups']:
        for name in group['files']:
            path = PurePosixPath(name)
            assert not path.is_absolute() and '..' not in path.parts
            assert path.parts[0] == 'david_nav' and 'legacy' not in path.parts
            destination = PurePosixPath('david_nav/legacy').joinpath(*path.parts[1:])
            assert (SOURCE / name).is_file()
            assert not (SOURCE / destination).exists()
            sources.append(name)
            destinations.append(str(destination))
    assert len(sources) == len(set(sources)) == len(set(destinations)) == 42


def test_documented_reference_and_retained_paths_still_exist():
    """The proposal preserves evidence and active bench/rollback/compatibility entry paths."""
    document = _inventory()
    for group in document['groups']:
        for path in group['references']:
            assert (SOURCE / path).is_file(), path
    for path in document['retain']:
        assert (SOURCE / path).exists(), path
    assert 'david_nav/scripts/controller_bench' in document['retain']
    assert 'david_nav/david_nav/navigation_executor.py' in document['retain']
    assert 'david_nav/start_navigation_headless.sh' in document['retain']


def test_five_shell_candidates_are_explicitly_unconfirmed():
    """The plan's unnamed five shell entries are proposed identities, not invented approval."""
    group = next(group for group in _inventory()['groups'] if group['id'] == 'shell_entries')
    assert len(group['files']) == 5
    assert all(path.endswith('.sh') for path in group['files'])
    assert group['identity_confirmed'] is False


@pytest.mark.parametrize('mode', ['tour', 'precise', 'bench_slamgt', 'chassis', 'teleop'])
def test_new_graph_does_not_start_candidate_legacy_executables(mode):
    """A cleaner new graph is not evidence that the old optional/bench graph is unused."""
    names = {item.executable for item in build_graph(GraphOptions(mode, '/unused/run')).processes}
    assert not names.intersection({
        'cmd_vel_separator', 'map_warmup_spin', 'frontier_explorer_cpp',
        'semantic_pose_navigator.py', 'waypoint_follower', 'dead_reckoning_executor',
        'collision_monitor',
    })
