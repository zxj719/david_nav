"""Run-local console forwarding and low-rate resource recording without ROS."""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Callable, IO, List, Optional

from nav_bringup.recorder.run_dir import RunDirectory


class StartupObserver:
    """Own a short-lived recording thread, never a navigation child or ROS context."""

    def __init__(self, run: RunDirectory, terminal: Optional[IO[bytes]],
                 poll: Optional[Callable[[], None]] = None) -> None:
        """Describe a recorder without starting it or opening output files.

        Args:
            run: New directory owned by the startup supervisor.
            terminal: Optional binary output stream, including redirected stdout.
            poll: Optional bounded nonblocking control callback on the existing thread.
        """
        self.run = run
        self.terminal = terminal
        self.errors: List[str] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._offsets = {}
        self._console = None
        self._event_offset = 0
        self._event_fragment = b''
        self._poll = poll

    def start(self) -> None:
        """Start 1 Hz supervisor-tree sampling and forwarding of run-local logs.

        Raises:
            RuntimeError: This observer has already started.
            OSError: The exclusive console file cannot be opened.
            ImportError: The declared psutil dependency is unavailable.
        """
        if self._thread is not None:
            raise RuntimeError('observer already started')
        from nav_bringup.recorder.resources import ResourceSampler

        # The entry point owns its process tree; sampling is read-only and does
        # not confer signaling authority. Startup helpers are included as well.
        self._sampler = ResourceSampler(lambda: (os.getpid(),),
                                        self.run.metrics / 'resources.csv')
        self._console = (self.run.logs / 'console.log').open('xb')
        self._sampling = True
        try:
            self._sampler.sample()
        except Exception as error:
            self.errors.append(str(error))
            self._sampling = False
        self._thread = threading.Thread(target=self._record, daemon=True)
        try:
            self._thread.start()
        except BaseException:
            self._console.close()
            self._thread = None
            raise

    def close(self) -> None:
        """Stop sampling, drain final output, and close owned files.

        Raises:
            RuntimeError: The observer did not stop within five seconds.
        """
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout=5.0)
        if self._thread.is_alive():
            raise RuntimeError('run observer has not stopped')
        try:
            self._forward(drain=True)
        finally:
            self._console.close()
            self._thread = None

    def _record(self):
        next_sample = time.monotonic() + 1.0
        try:
            while not self._stop.is_set():
                if self._poll is not None:
                    try:
                        self._poll()
                    except Exception as error:
                        self.errors.append('control: ' + type(error).__name__)
                        self._poll = None
                current = time.monotonic()
                if self._sampling and current >= next_sample:
                    try:
                        self._sampler.sample()
                    except Exception as error:
                        self.errors.append(str(error))
                        self._sampling = False
                    next_sample = current + 1.0
                self._forward()
                self._stop.wait(0.1)
        except Exception as error:
            self.errors.append(str(error))

    def _forward(self, drain=False):
        for path in sorted(self.run.logs.glob('*.log')):
            if path.name == 'console.log' or path.is_symlink() or not path.is_file():
                continue
            offset = self._offsets.get(path, 0)
            with path.open('rb') as stream:
                stream.seek(offset)
                # At most 64 KiB/file on each live pass; child files retain the
                # authoritative output even when a terminal is slower than logs.
                remaining = max(0, path.stat().st_size - offset) if drain else 65536
                while remaining:
                    chunk = stream.read(min(65536, remaining))
                    if not chunk:
                        break
                    self._write(chunk)
                    offset += len(chunk)
                    remaining -= len(chunk)
            self._offsets[path] = offset
        self._forward_events(drain)

    def _forward_events(self, drain):
        if not self.run.events.is_file() or self.run.events.is_symlink():
            return
        with self.run.events.open('rb') as stream:
            stream.seek(self._event_offset)
            remaining = (max(0, self.run.events.stat().st_size - self._event_offset)
                         if drain else 65536)
            while remaining:
                chunk = stream.read(min(65536, remaining))
                if not chunk:
                    break
                self._event_offset += len(chunk)
                remaining -= len(chunk)
                lines = (self._event_fragment + chunk).split(b'\n')
                self._event_fragment = lines.pop()
                if len(self._event_fragment) > 1048576:
                    self.errors.append('oversized incomplete event; inspect events.jsonl')
                    self._event_fragment = b''
                for line in lines:
                    try:
                        event = json.loads(line)
                        message = (f'{event["source"]} [{event["severity"]}] '
                                   f'{event["type"]}: '
                                   f'{json.dumps(event["data"], ensure_ascii=False)}\n')
                    except (ValueError, KeyError, TypeError):
                        self.errors.append('malformed event; inspect events.jsonl')
                        continue
                    self._write(message.encode('utf-8'))

    def _write(self, chunk):
        self._console.write(chunk)
        self._console.flush()
        if self.terminal is not None:
            try:
                view = memoryview(chunk)
                while view:
                    written = self.terminal.write(view)
                    if written is None or written <= 0:
                        raise OSError('terminal accepted no output')
                    view = view[written:]
                self.terminal.flush()
            except (OSError, ValueError) as error:
                self.errors.append(f'terminal: {error}')
                self.terminal = None
