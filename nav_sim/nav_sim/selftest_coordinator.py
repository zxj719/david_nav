"""协调普通 POI 后 Dock 的自测顺序，分别检查事件、位姿及里程计证据."""

import asyncio
import json
import math
import time
from typing import Any, Mapping, Sequence

from nav_sim.fake_agent_server import navigate_to
from nav_sim.model import finite_number
from nav_sim.selftest_evidence import check_evidence
from nav_sim.selftest_scene import Scene
from nav_sim.selftest_trace import audit_processes, EvidenceTrace, reached


class Peer:
    """接收一次不可重连重放的 WebSocket 会话，并交给有界证据记录器.

    Attributes:
        error: 已观察到的连接/任务失败原因，尚无失败时为 None。
        socket: 当前唯一连接，断开或尚未接入时为 None。
        stopping: 是否正在主动关闭，用于区分正常收尾和意外断连。
        trace: 必须在启动子进程前绑定的有界证据记录器。
    """

    def __init__(self) -> None:
        """初始化未连接对端；启动任何子进程前还须绑定 trace."""
        self.trace = self.socket = self.error = None
        self.stopping = False
        self._accepted = False

    async def handle(self, socket: Any) -> None:
        """观察唯一客户端并回应心跳；导航命令由 drive 发送.

        无记录器、重复接入或重连均记录错误并拒绝；接收/任务失败写入 error，
        不据此报告成功，也不在本回调发送导航目标。

        Args:
            socket: 回环服务器交来的候选连接。
        """
        if self._accepted or self.trace is None:
            self.error = 'unexpected_or_reconnected_client'
            await socket.close(code=1008, reason='single experiment connection')
            return
        self._accepted, self.socket = True, socket
        try:
            async for raw in socket:
                frame = self.trace.receive('ws', raw)
                if (frame.get('event_type') == 'on_error'
                        or frame.get('event_type') == 'on_progress'
                        and frame.get('status') == 'paused'):
                    self.error = 'task_failed_or_paused'
                if frame.get('type') == 'ping':
                    await socket.send(json.dumps({'type': 'pong'}))
        except Exception as error:
            self.error = 'websocket_' + type(error).__name__
        finally:
            self.socket = None
            if not self.stopping and self.error is None:
                self.error = 'client_disconnected'


async def drive(scene: Scene, trace: EvidenceTrace, peer: Peer, observer: Any,
                child: Any, timeout: float, startup_only: bool) -> Sequence[Mapping[str, object]]:
    """等待启动证据，非仅启动模式下依次执行场景中的普通 POI 与 Dock.

    仅启动模式不发送命令。完整模式要求外层已取得运动授权；
    每个用例同时检查关联到达事件、所有权释放、TF 到点和里程计位移，
    返回后仍须由运行器检查尾部事件及清理结果。

    Args:
        scene: 已验证的私有场景、两用例预期和真实启动图。
        trace: 有界且分通道保留的 WS 与 NavEvent 证据。
        peer: 单次接入的命令传输对端。
        observer: 只读 ROS 观察器；内存替身仅供纯单元测试。
        child: 仍由调用方持有且 poll 不回收 PID 的启动器。
        timeout: 从进入本函数起计算的有限正秒数，含启动等待，不含此前准备和后续清理。
        startup_only: True 时只验证启动并返回空列表，不生成动作完成证据。

    Returns:
        每个已完成用例的独立 TF/里程计测量记录；不代表整体自测成功。

    Raises:
        ValueError: 超时或进程/事件证据不合法。
        RuntimeError: 进程、连接、观察或关联一致性检查失败。
        TimeoutError: 就绪、发送或完成证据超过期限；同步回调本身不被抢占中断。
    """
    if finite_number(timeout, 'timeout') <= 0:
        raise ValueError('timeout must be positive')
    deadline = time.monotonic() + timeout
    inspected = 0

    async def until(predicate):
        nonlocal inspected
        while True:
            if time.monotonic() >= deadline:
                raise TimeoutError('selftest deadline exceeded')
            if child.poll() is not None:
                raise RuntimeError('launcher exited before experiment completion')
            if peer.error:
                raise RuntimeError(peer.error)
            audit_processes(scene)
            observer.step()
            for event in trace.nav[inspected:]:
                source = event.get('source')
                permitted = {
                    'tasks': ('task_admitted', 'task_phase', 'task_progress', 'task_succeeded'),
                    'dock': ('dock_started', 'dock_approach_checked', 'dock_stage_started',
                             'dock_measured'),
                }
                if source in permitted and (event.get('event_type') not in permitted[source]
                                            or event.get('severity') != 'info'):
                    raise RuntimeError('unhealthy task observation')
            inspected = len(trace.nav)
            if predicate():
                return
            await asyncio.sleep(0.02)

    await until(lambda: peer.socket is not None and observer.ready() and audit_processes(scene))
    if startup_only:
        return []
    measurements = []
    for index, case in enumerate(scene.cases):
        await until(observer.ready)
        before = observer.odometry()
        if _arrivals(trace, scene.cases) != [item.task_id for item in scene.cases[:index]]:
            raise RuntimeError('premature or duplicate arrival')
        command = navigate_to(case.task_id, case.target_id, case.sub_id)
        await asyncio.wait_for(peer.socket.send(json.dumps(command)),
                               max(0.001, deadline - time.monotonic()))

        def arrived():
            arrivals = _arrivals(trace, scene.cases)
            if arrivals not in ([item.task_id for item in scene.cases[:index]],
                                [item.task_id for item in scene.cases[:index + 1]]):
                raise RuntimeError('unexpected arrival sequence')
            return (len(arrivals) == index + 1 and reached(case, observer.pose())
                    and observer.odometry() is not None and any(
                        event.get('event_type') == 'task_succeeded'
                        and event.get('data', {}).get('task_id') == case.task_id
                        and event['data'].get('ownership_released') is True
                        for event in trace.nav))

        await until(arrived)
        pose, after = observer.pose(), observer.odometry()
        displacement = math.hypot(after[0] - before[0], after[1] - before[1])
        if displacement <= 0.05:
            raise RuntimeError('arrival without independent chassis displacement')
        measurements.append(dict(task_id=case.task_id, pose=pose,
                                 odometry_start=before, odometry_end=after,
                                 odometry_displacement=displacement))
    await until(lambda: check_evidence(scene.cases, trace.nav, trace.ws).complete)
    return measurements


def _arrivals(trace, cases):
    """按 WS 接收顺序提取到达通知，并拒绝关联错误、失败或终态后的帧.

    Args:
        trace: 已解码的观察记录，读取其 ws 列表，不发送命令。
        cases: 含预期任务 ID 和整数 sub_id 的实验用例。

    Returns:
        到达通知的任务 ID 列表；跨用例的期望顺序由调用方继续校验。

    Raises:
        RuntimeError: 任务关联不符、终态后仍有任务帧，或收到错误/暂停事件。
    """
    expected = {case.task_id: case for case in cases}
    arrivals = []
    for row in trace.ws:
        name = row.get('event_type')
        if name not in ('on_arrived', 'on_progress', 'on_error'):
            continue
        task = row.get('request_id')
        if (not isinstance(task, str) or task not in expected
                or type(row.get('sub_id')) is not int or row['sub_id'] != expected[task].sub_id):
            raise RuntimeError('mismatched task correlation')
        if task in arrivals:
            raise RuntimeError('task frame after terminal arrival')
        if name == 'on_error' or name == 'on_progress' and row.get('status') == 'paused':
            raise RuntimeError('task failed or paused')
        if name == 'on_arrived':
            arrivals.append(task)
    return arrivals
