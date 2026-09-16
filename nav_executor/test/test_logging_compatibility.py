"""Preserve the frozen logger after removing transport's upward dependency."""

import inspect

from nav_executor.adapters.transport import logging, ws_client
from nav_executor.app import logging as app_logging


def test_default_logger_identity_and_frozen_output(capsys):
    """Existing imports and default callbacks share exactly the same function."""
    assert app_logging.log_line is logging.log_line
    assert ws_client.log_line is logging.log_line
    for function in (ws_client.receive_messages, ws_client.send_heartbeat, ws_client.reconnect):
        assert inspect.signature(function).parameters['emit'].default is logging.log_line
    logging.log_line('probe')
    assert capsys.readouterr().out == '[navigation_executor] probe\n'
