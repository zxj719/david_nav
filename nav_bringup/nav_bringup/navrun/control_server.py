"""Cooperative same-user control, polled by the existing run observer thread.

Linux abstract sequence-packet sockets need no socket file, daemon or extra
thread. A per-incarnation endpoint prevents stale clients from stopping a new
run. Neither discovery metadata nor a peer PID is used to send OS signals.
"""

from __future__ import annotations

import json
import os
import socket
import struct
import time
from typing import Any, Callable, Dict
import uuid

from nav_bringup.navrun.locking import RunLock


class RunControl:
    """Serve bounded status/down requests only while the creating lease is owned."""

    def __init__(self, lease: RunLock, request_stop: Callable[[], None]) -> None:
        """Describe a cooperative server without opening any endpoint.

        Args:
            lease: Acquired lease, retained through finalization.
            request_stop: Idempotent shutdown request, not a process signal or cleanup.
        """
        self._lease = lease
        self._request_stop = request_stop
        self._socket = None
        self._clients: Dict[socket.socket, float] = {}
        instance = uuid.uuid4().hex
        self._metadata: Dict[str, Any] = {
            'version': 1, 'instance': instance, 'address': f'navrun_{os.geteuid()}_{instance}',
            'run_id': None, 'mode': None, 'finished': False}
        self._stopped = False

    def __enter__(self) -> RunControl:
        """Open a private incarnation endpoint and publish discovery under the lease.

        Returns:
            The active server; the caller must arrange periodic poll calls.

        Raises:
            RuntimeError: The server is already open or the lease is not owned.
            OSError: Socket creation or discovery publication fails.
        """
        if self._socket is not None:
            raise RuntimeError('control server already open')
        self._lease.assert_owned()
        listener = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            listener.setblocking(False)
            listener.bind('\0' + self._metadata['address'])
            listener.listen(8)
            self._lease.annotate(self._metadata)
        except BaseException:
            listener.close()
            raise
        self._socket = listener
        return self

    def identify(self, run_id: str, mode: str) -> None:
        """Publish the new record identity before starting the observer thread.

        Args:
            run_id: Record name, not a file-system control target.
            mode: Selected navigation or chassis mode.
        """
        self._metadata.update(run_id=run_id, mode=mode)
        self._lease.annotate(self._metadata)

    def poll(self) -> None:
        """Handle at most eight nonblocking same-user requests per observer pass.

        Malformed, oversized, stale or slow clients are closed without invoking
        shutdown. Only the caller's existing observer thread should call poll.
        """
        if self._socket is None:
            return
        for _ in range(8 - len(self._clients)):
            try:
                client, _ = self._socket.accept()
            except BlockingIOError:
                break
            client.setblocking(False)
            _, uid, _ = struct.unpack('3i', client.getsockopt(
                socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize('3i')))
            if uid != os.geteuid():
                client.close()
            else:
                self._clients[client] = time.monotonic() + 2.0
        for client, deadline in list(self._clients.items()):
            try:
                if time.monotonic() >= deadline:
                    self._close_client(client)
                    continue
                packet, _, flags, _ = client.recvmsg(1024)
                if flags & socket.MSG_TRUNC or not packet:
                    self._close_client(client)
                    continue
                request = json.loads(packet)
                if (not isinstance(request, dict) or set(request) != {'op', 'instance'}
                        or request['instance'] != self._metadata['instance']
                        or request['op'] not in ('status', 'down')):
                    self._close_client(client)
                    continue
                self._lease.assert_owned()
                if request['op'] == 'down' and not self._stopped:
                    self._request_stop()
                    self._stopped = True
                response = dict(self._metadata, pid=os.getpid(),
                                state='stop_requested' if self._stopped else 'active',
                                cleanup_complete=False)
                client.sendall(json.dumps(response).encode('utf-8'))
                self._close_client(client)
            except BlockingIOError:
                continue
            except (ValueError, OSError, RuntimeError):
                self._close_client(client)

    def _close_client(self, client: socket.socket) -> None:
        self._clients.pop(client, None)
        client.close()

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        """Close control before releasing the lease; mark only normal completion.

        The caller must join its polling thread before leaving this context.

        Args:
            exc_type: Escaping exception type, or None after finalized cleanup.
            exc_value: Escaping exception value, unused.
            traceback: Escaping traceback, unused.
        """
        for client in list(self._clients):
            self._close_client(client)
        if self._socket is not None:
            self._socket.close()
            self._socket = None
        if exc_type is None:
            self._metadata['finished'] = True
            self._lease.annotate(self._metadata)
