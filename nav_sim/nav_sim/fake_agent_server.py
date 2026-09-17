"""Loopback-only scripted WebSocket peer for the frozen navigation protocol."""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass, field
import json
import os
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from nav_sim.model import finite_number, require_sandbox


def navigate_to(request_id: str, target_id: str, sub_id: int = 0) -> Dict[str, Any]:
    """Build a v1 POI command, keeping its on-wire insertion order.

    Args:
        request_id: Nonempty task identifier.
        target_id: Nonempty POI identifier, never a direct coordinate.
        sub_id: Integer subtask identifier.

    Returns:
        V1 navigate_to command.

    Raises:
        ValueError: Required identifiers are empty or the subtask is not an integer.
    """
    if (not isinstance(request_id, str) or not request_id.strip()
            or not isinstance(target_id, str) or not target_id.strip()
            or isinstance(sub_id, bool) or not isinstance(sub_id, int)):
        raise ValueError('navigate_to requires request_id, target_id and integer sub_id')
    return {'action': 'navigate_to', 'request_id': request_id,
            'sub_id': sub_id, 'target_id': target_id}


def abort(request_id: str) -> Dict[str, str]:
    """Build the legacy abort_navigation command, not a new action alias.

    Args:
        request_id: Nonempty request identifier.

    Returns:
        V1 cancellation command.

    Raises:
        ValueError: The request identifier is empty.
    """
    if not isinstance(request_id, str) or not request_id.strip():
        raise ValueError('abort requires request_id')
    return {'action': 'abort_navigation', 'request_id': request_id}


@dataclass(frozen=True)
class ScriptStep:
    """One outbound command followed by an optional matching-event barrier.

    Attributes:
        command: Outbound v1 command dictionary.
        wait_event: Event type to await before sending the next command.
    """

    command: Mapping[str, Any]
    wait_event: Optional[str] = None


def load_script(path: Path) -> List[ScriptStep]:
    """Load a finite JSON scenario without permitting arbitrary protocol actions.

    Args:
        path: JSON list of command/wait_event objects.

    Returns:
        Validated ordered steps.

    Raises:
        ValueError: Script structure or command action is unsupported.
        OSError: Script cannot be read.
    """
    entries = json.loads(path.read_text(encoding='utf-8'))
    if not isinstance(entries, list):
        raise ValueError('scenario must be a list')
    result = []
    for entry in entries:
        if not isinstance(entry, dict) or not isinstance(entry.get('command'), dict):
            raise ValueError('scenario step requires command object')
        command = entry['command']
        action = command.get('action')
        if action == 'navigate_to':
            validated = navigate_to(command.get('request_id'), command.get('target_id'),
                                    command.get('sub_id', 0))
        elif action == 'abort_navigation':
            validated = abort(command.get('request_id'))
        else:
            raise ValueError(f'unsupported scenario action: {action}')
        event = entry.get('wait_event')
        if event is not None and (not isinstance(event, str) or not event):
            raise ValueError('wait_event must be a nonempty string or null')
        result.append(ScriptStep(validated, event))
    return result


@dataclass
class AgentSession:
    """A single-client finite scenario, injectable without opening sockets.

    Attributes:
        steps: Commands and event barriers.
        timeout: Per-barrier timeout in seconds.
        events: All received JSON object events in receive order.
        completed: Whether every scripted barrier has completed.
    """

    steps: Sequence[ScriptStep] = ()
    timeout: float = 30.0
    events: List[Dict[str, Any]] = field(default_factory=list)
    completed: bool = False

    async def run(self, websocket: Any) -> None:
        """Send the script, retaining interleaved status and unrelated events.

        Args:
            websocket: Peer implementing asynchronous send, recv and iteration.

        Raises:
            ValueError: Peer sent a non-object JSON frame or timeout is not positive.
            asyncio.TimeoutError: A matching event did not arrive before its deadline.
        """
        if finite_number(self.timeout, 'timeout') <= 0:
            raise ValueError('timeout must be positive')
        for step in self.steps:
            await websocket.send(json.dumps(dict(step.command), ensure_ascii=False))
            if step.wait_event:
                await asyncio.wait_for(self._await_event(websocket, step), self.timeout)
        self.completed = True
        async for frame in websocket:
            await self._receive(websocket, frame)

    async def _await_event(self, websocket: Any, step: ScriptStep) -> None:
        while True:
            event = await self._receive(websocket, await websocket.recv())
            if (event.get('event_type') == step.wait_event
                    and event.get('request_id') == step.command.get('request_id')
                    and ('sub_id' not in step.command
                         or event.get('sub_id') == step.command['sub_id'])):
                return

    async def _receive(self, websocket: Any, frame: str) -> Dict[str, Any]:
        event = json.loads(frame)
        if not isinstance(event, dict):
            raise ValueError('agent peer must send JSON objects')
        self.events.append(event)
        if event.get('type') == 'ping':
            await websocket.send(json.dumps({'type': 'pong'}, ensure_ascii=False))
        return event


async def _serve(options: argparse.Namespace) -> None:
    """监听回环地址并串行接收自测客户端，不自行授予脚本目标执行权限.

    Args:
        options: 已经由 main 校验的端口、脚本、秒制超时和可选事件输出路径；
            有脚本时调用方必须已检查 --allow-goals。事件文件采用追加写入。

    Raises:
        OSError: 脚本读取或监听端口失败；会话事件写入异常由服务回调报告。
        ValueError: 脚本内容不符合命令步骤格式。
        asyncio.CancelledError: 外层终止服务，异步上下文负责关闭监听资源。
    """
    from websockets.legacy.server import serve

    steps = load_script(options.script) if options.script else []
    active = False

    async def handler(websocket):
        nonlocal active
        if active:
            await websocket.close(code=1013, reason='one test client at a time')
            return
        active = True
        session = AgentSession(steps, options.timeout)
        try:
            await session.run(websocket)
        finally:
            active = False
            if options.events:
                with options.events.open('a', encoding='utf-8') as stream:
                    for event in session.events:
                        stream.write(json.dumps(event, ensure_ascii=False) + '\n')
            print(f'client closed: completed={session.completed} events={len(session.events)}',
                  flush=True)

    async with serve(handler, '127.0.0.1', options.port):
        print(f'fake agent listening on 127.0.0.1:{options.port}', flush=True)
        await asyncio.Future()


def main(args: Optional[Sequence[str]] = None) -> None:
    """Run a passive local peer unless explicit script execution is enabled.

    Args:
        args: Optional command-line arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=18101)
    parser.add_argument('--script', type=Path)
    parser.add_argument('--allow-goals', action='store_true')
    parser.add_argument('--events', type=Path)
    parser.add_argument('--timeout', type=float, default=30.0)
    options = parser.parse_args(args)
    require_sandbox(os.environ)
    if options.script and not options.allow_goals:
        parser.error('--script requires explicit --allow-goals; default server is passive')
    if not 1 <= options.port <= 65535:
        parser.error('--port must be in 1..65535')
    try:
        if finite_number(options.timeout, 'timeout') <= 0:
            parser.error('--timeout must be positive')
    except ValueError as error:
        parser.error(str(error))
    try:
        asyncio.run(_serve(options))
    except KeyboardInterrupt:
        pass
