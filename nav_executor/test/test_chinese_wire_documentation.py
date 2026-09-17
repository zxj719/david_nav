"""按限定名保护中文摘要；内容准确性仍需逐段代码审查."""

import ast
from pathlib import Path
import re

import pytest


@pytest.mark.parametrize('path, qualified_name', [
    ('__init__.py', '<module>'),
    ('adapters/__init__.py', '<module>'),
    ('adapters/transport/__init__.py', '<module>'),
    ('adapters/transport/logging.py', '<module>'),
    ('adapters/transport/logging.py', 'log_line'),
    ('adapters/transport/outbox.py', '<module>'),
    ('adapters/transport/outbox.py', 'ReliableOutbox'),
    ('adapters/transport/outbox.py', 'ReliableOutbox.__init__'),
    ('adapters/transport/outbox.py', 'ReliableOutbox.in_flight_count'),
    ('adapters/transport/outbox.py', 'ReliableOutbox.pending_count'),
    ('adapters/transport/outbox.py', 'ReliableOutbox.put'),
    ('adapters/transport/outbox.py', 'ReliableOutbox.drain'),
    ('adapters/transport/proxy.py', '<module>'),
    ('adapters/transport/proxy.py', 'bypass_proxy_for_server'),
    ('adapters/transport/ws_client.py', '<module>'),
    ('adapters/transport/ws_client.py', 'WebSocketLike'),
    ('adapters/transport/ws_client.py', 'WebSocketLike.__aiter__'),
    ('adapters/transport/ws_client.py', 'WebSocketLike.send'),
    ('adapters/transport/ws_client.py', 'WsOptions'),
    ('adapters/transport/ws_client.py', 'receive_messages'),
    ('adapters/transport/ws_client.py', 'send_heartbeat'),
    ('adapters/transport/ws_client.py', 'reconnect'),
    ('protocol/__init__.py', '<module>'),
    ('protocol/codec.py', '<module>'),
    ('protocol/codec.py', 'parse_command'),
    ('protocol/codec.py', 'build_progress_event'),
    ('protocol/codec.py', 'build_mark_poi_success_event'),
    ('protocol/codec.py', 'build_arrived_event'),
    ('protocol/codec.py', 'build_error_event'),
    ('protocol/codec.py', 'build_mark_poi_ack'),
    ('protocol/codec.py', 'build_mark_poi_error'),
    ('protocol/messages.py', '<module>'),
    ('protocol/messages.py', 'NavExecutorError'),
    ('protocol/messages.py', 'ProtocolError'),
    ('protocol/messages.py', 'ProtocolError.__init__'),
    ('protocol/messages.py', 'CommandParseError'),
    ('protocol/messages.py', 'MalformedCommandError'),
    ('protocol/messages.py', 'UnsupportedActionError'),
    ('protocol/messages.py', 'NavigateToCommand'),
    ('protocol/messages.py', 'MarkCurrentPoiCommand'),
    ('protocol/messages.py', 'AbortNavigationCommand'),
    ('protocol/messages.py', 'UpdatePoiListCommand'),
    ('protocol/task_adapter.py', '<module>'),
    ('protocol/task_adapter.py', 'event_to_frame'),
    ('protocol/task_adapter.py', 'TaskCommandAdapter'),
    ('protocol/task_adapter.py', 'TaskCommandAdapter.__init__'),
    ('protocol/task_adapter.py', 'TaskCommandAdapter.handle_message'),
    ('protocol/task_adapter.py', 'TaskCommandAdapter.next_event'),
    ('protocol/task_adapter.py', 'TaskCommandAdapter.pump_events'),
    ('protocol/task_adapter.py', 'TaskCommandAdapter.close'),
])
def test_reviewed_wire_documentation_has_chinese_summary(path, qualified_name):
    """仅检查已审条目的中文首句，不据此声称语义或运行行为已通过验收.

    Args:
        path: 执行器包内的相对模块路径。
        qualified_name: 模块标记或类、函数限定名。
    """
    root = Path(__file__).resolve().parents[1] / 'nav_executor'
    node = ast.parse((root / path).read_text(encoding='utf-8'))
    if qualified_name != '<module>':
        for name in qualified_name.split('.'):
            matches = [child for child in node.body
                       if isinstance(child, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
                       and child.name == name]
            assert len(matches) == 1, qualified_name
            node = matches[0]
    doc = ast.get_docstring(node) or ''
    assert doc.strip(), qualified_name
    assert re.search('[\u4e00-\u9fff]', doc.splitlines()[0]), qualified_name
