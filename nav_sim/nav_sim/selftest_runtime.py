"""Real Nav2 selftest resource ownership; imported only after explicit CLI guards."""

import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Callable, Mapping

import websockets

from nav_bringup.navrun.owned_process import OwnedProcess
from nav_sim.model import finite_number, require_sandbox
from nav_sim.selftest_coordinator import drive, Peer
from nav_sim.selftest_evidence import check_evidence
from nav_sim.selftest_launch import close_fixture
from nav_sim.selftest_plan import load_scene
from nav_sim.selftest_scene import Scene
from nav_sim.selftest_trace import audit_processes, EvidenceTrace


async def _cleanup(scene, child, observer, server, peer, result):
    peer.stopping = True
    clean = child is not None
    try:
        if child is not None:
            code = await close_fixture(child)
            if code not in (0, 130, -2):
                raise RuntimeError('abnormal launcher exit')
            audit_processes(scene, final=True)
    except Exception as error:
        clean = False
        result['errors'].append('cleanup_' + type(error).__name__)
    finally:
        try:
            server.close()
            await server.wait_closed()
        except Exception as error:
            clean = False
            result['errors'].append('server_close_' + type(error).__name__)
        finally:
            if observer is not None:
                try:
                    observer.step()
                except Exception as error:
                    result['errors'].append('observer_' + type(error).__name__)
                try:
                    observer.close()
                except Exception as error:
                    clean = False
                    result['errors'].append('observer_close_' + type(error).__name__)
            result['cleanup_complete'] = clean


def require_idle_host() -> None:
    """Read-only guard against starting ROS beside the user's localization/navigation.

    Raises:
        RuntimeError: Any non-ancestor protected process exists or scanning is uncertain.
        OSError: Process ancestry or the read-only scan cannot be obtained.
    """
    require_sandbox(os.environ)
    ancestors, pid = set(), os.getpid()
    while pid > 1 and pid not in ancestors:
        ancestors.add(pid)
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        pid = int(fields[1])
    scan = subprocess.run([
        'pgrep', '-f', 'run_loc_online|controller_server|bt_navigator|wheeltec_robot_node|'
        'wheeltec_driver_node|navigation_executor|dock_navigator|rviz2|component_container',
    ], capture_output=True, text=True, check=False, timeout=5)
    if scan.returncode not in (0, 1) or {int(value) for value in scan.stdout.split()} - ancestors:
        raise RuntimeError('selftest requires an idle host; no processes were signalled')


async def execute(prepare: Callable[[str], Scene], timeout: float,
                  startup_only: bool, allow_goals: bool) -> Mapping[str, object]:
    """Own a passive loopback server, read-only observer and real private Nav2 graph.

    Args:
        prepare: Renderer called with the actual owned loopback listener URI.
        timeout: Total positive finite experiment deadline.
        startup_only: Request startup/cleanup validation without sending any goal.
        allow_goals: Explicit authorization flag required for a complete experiment.

    Returns:
        Persisted result; complete is True only for both goals, evidence and cleanup.

    Raises:
        RuntimeError: Domain, authority or idle-host guard fails before execution.
        ValueError: Preparation or integrity validation fails.
        OSError: Private artifact creation fails.
        KeyboardInterrupt: Caller interrupted; owned children are drained first.
        asyncio.CancelledError: Caller cancelled; owned children are drained first.
    """
    require_sandbox(os.environ)
    if finite_number(timeout, 'timeout') <= 0:
        raise ValueError('timeout must be positive')
    if type(startup_only) is not bool or type(allow_goals) is not bool:
        raise ValueError('execution switches must be booleans')
    if not startup_only and not allow_goals:
        raise RuntimeError('complete selftest requires explicit --allow-goals')
    require_idle_host()
    from nav_sim.selftest_ros import Observer

    peer = Peer()
    async with websockets.serve(peer.handle, '127.0.0.1', 0, max_size=65536, max_queue=16,
                                close_timeout=2) as server:
        scene = prepare(f'ws://127.0.0.1:{server.sockets[0].getsockname()[1]}')
        load_scene(scene.root, scene.digest)
        result = dict(schema_version=1, backend='real_nav2', executed=False, complete=False,
                      passed=False, kind='startup_only' if startup_only else 'closed_loop',
                      plan_sha256=scene.digest, measurements=[], errors=[], cleanup_complete=False)
        with EvidenceTrace(scene.root) as trace:
            peer.trace = trace
            observer = child = None
            try:
                # Recheck after preparation and before creating the first ROS participant.
                require_idle_host()
                observer = Observer(scene, trace)
                child = OwnedProcess([
                    sys.executable, '-m', 'nav_sim.selftest_launch', '--run-dir', str(scene.root),
                    '--plan-sha256', scene.digest, '--start-verified-fixture',
                ], dict(os.environ, ROS_LOG_DIR=str(scene.root / 'ros_logs')),
                    scene.root / 'launch.log')
                result['executed'] = True
                result['measurements'] = await drive(
                    scene, trace, peer, observer, child, timeout, startup_only)
            except Exception as error:
                result['errors'].append(type(error).__name__ + ': ' + str(error)[:240])
            except BaseException as error:
                result['errors'].append(type(error).__name__)
                raise
            finally:
                # Protect the entire cleanup, including WS and ROS resource closure.
                # Keep ownership until it finishes even after repeated cancellation.
                cleanup = asyncio.create_task(_cleanup(
                    scene, child, observer, server, peer, result))
                cancelled = False
                while not cleanup.done():
                    try:
                        await asyncio.shield(cleanup)
                    except asyncio.CancelledError:
                        cancelled = True
                        if 'CancelledError' not in result['errors']:
                            result['errors'].append('CancelledError')
                cleanup.result()
                if peer.error:
                    result['errors'].append(peer.error)
                if not startup_only:
                    evidence = check_evidence(scene.cases, trace.nav, trace.ws)
                    result['errors'].extend(evidence.errors)
                result['passed'] = (result['executed'] and result['cleanup_complete']
                                    and not result['errors'])
                result['complete'] = result['passed'] and not startup_only
                with (scene.root / 'result.json').open('x', encoding='utf-8') as stream:
                    json.dump(result, stream, indent=2, allow_nan=False)
                    stream.write('\n')
                if cancelled:
                    raise asyncio.CancelledError()
        return result
