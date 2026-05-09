#!/usr/bin/env python3
"""Unit tests for launch_manager.py — runs on Windows/Linux without ROS."""
from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
import unittest
from http.client import HTTPConnection
from pathlib import Path
from unittest.mock import patch, MagicMock

# Add parent dir to path so we can import launch_manager
sys.path.insert(0, str(Path(__file__).parent))

from launch_manager import (
    ManagedProcess,
    ProcessManager,
    load_profiles,
    LaunchManagerHandler,
    LOG_BUFFER_MAX,
)


class TestLoadProfiles(unittest.TestCase):
    def test_load_profiles_from_yaml(self):
        content = textwrap.dedent("""\
            profiles:
              test_profile:
                command: echo hello
                description: A test profile
                env:
                  FOO: bar
              other:
                command: sleep 1
                description: Another profile
        """)
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False, encoding="utf-8"
        ) as f:
            f.write(content)
            f.flush()
            path = Path(f.name)

        try:
            profiles = load_profiles(path)
            self.assertEqual(len(profiles), 2)
            self.assertIn("test_profile", profiles)
            self.assertEqual(profiles["test_profile"]["command"], "echo hello")
            self.assertEqual(profiles["test_profile"]["env"]["FOO"], "bar")
            self.assertIn("other", profiles)
        finally:
            path.unlink()

    def test_load_empty_profiles(self):
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False, encoding="utf-8"
        ) as f:
            f.write("profiles:\n")
            f.flush()
            path = Path(f.name)

        try:
            profiles = load_profiles(path)
            self.assertEqual(len(profiles), 0)
        finally:
            path.unlink()


class TestManagedProcess(unittest.TestCase):
    """Test ManagedProcess with real subprocesses (cross-platform safe commands)."""

    def _make_proc(self, command: str) -> ManagedProcess:
        return ManagedProcess(
            profile_name="test",
            command=command,
            env={},
        )

    def test_start_and_state(self):
        # Use a command that exits quickly
        if sys.platform == "win32":
            proc = self._make_proc("echo hello && exit 0")
        else:
            proc = self._make_proc("echo hello")

        self.assertEqual(proc.state, "stopped")
        result = proc.start()
        self.assertTrue(result["ok"])
        self.assertIn("pid", result)

        # Wait for the short-lived process to finish
        time.sleep(1)
        self.assertEqual(proc.state, "stopped")

    def test_start_already_running(self):
        if sys.platform == "win32":
            proc = self._make_proc("ping -n 10 127.0.0.1 >nul")
        else:
            proc = self._make_proc("sleep 10")

        result1 = proc.start()
        self.assertTrue(result1["ok"])

        result2 = proc.start()
        self.assertFalse(result2["ok"])
        self.assertIn("already running", result2["error"])

        proc.stop()

    def test_stop(self):
        if sys.platform == "win32":
            proc = self._make_proc("ping -n 30 127.0.0.1 >nul")
        else:
            proc = self._make_proc("sleep 30")

        proc.start()
        time.sleep(0.5)
        self.assertIn(proc.state, ("running", "starting"))

        result = proc.stop(grace_seconds=5)
        self.assertTrue(result["ok"])
        self.assertEqual(proc.state, "stopped")

    def test_stop_already_stopped(self):
        proc = self._make_proc("echo hi")
        result = proc.stop()
        self.assertTrue(result["ok"])

    def test_logs_captured(self):
        if sys.platform == "win32":
            proc = self._make_proc("echo line1 && echo line2")
        else:
            proc = self._make_proc("echo line1 && echo line2")

        proc.start()
        time.sleep(1)

        logs = proc.logs(tail=10)
        text = "\n".join(logs)
        self.assertIn("line1", text)
        self.assertIn("line2", text)

    def test_status_fields(self):
        if sys.platform == "win32":
            proc = self._make_proc("ping -n 10 127.0.0.1 >nul")
        else:
            proc = self._make_proc("sleep 10")

        proc.start()
        time.sleep(0.5)

        status = proc.status()
        self.assertEqual(status["profile"], "test")
        self.assertIn(status["state"], ("running", "starting"))
        self.assertIn("pid", status)
        self.assertIn("started_at", status)
        self.assertIn("uptime_seconds", status)

        proc.stop()

    def test_uptime_none_when_stopped(self):
        proc = self._make_proc("echo done")
        self.assertIsNone(proc.uptime_seconds)


class TestProcessManager(unittest.TestCase):
    def test_get_or_create(self):
        profiles = {
            "a": {"command": "echo a", "env": {}},
            "b": {"command": "echo b", "env": {}},
        }
        manager = ProcessManager(profiles)
        proc_a = manager.get_or_create("a")
        proc_a2 = manager.get_or_create("a")
        self.assertIs(proc_a, proc_a2)

        proc_b = manager.get_or_create("b")
        self.assertIsNot(proc_a, proc_b)

    def test_stop_all(self):
        if sys.platform == "win32":
            cmd = "ping -n 30 127.0.0.1 >nul"
        else:
            cmd = "sleep 30"

        profiles = {"x": {"command": cmd, "env": {}}}
        manager = ProcessManager(profiles)
        proc = manager.get_or_create("x")
        proc.start()
        time.sleep(0.5)

        manager.stop_all()
        self.assertEqual(proc.state, "stopped")


class TestHTTPIntegration(unittest.TestCase):
    """Start the HTTP server on a random port and test the API."""

    @classmethod
    def setUpClass(cls):
        from http.server import HTTPServer

        profiles = {
            "test": {
                "command": "echo integration_test_ok",
                "description": "Test profile",
                "env": {},
            },
        }
        cls.manager = ProcessManager(profiles)

        class Handler(LaunchManagerHandler):
            pass

        Handler.manager = cls.manager
        Handler.profiles = profiles

        cls.server = HTTPServer(("127.0.0.1", 0), Handler)
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.manager.stop_all()
        cls.server.shutdown()
        cls.server.server_close()

    def _get(self, path: str) -> dict:
        conn = HTTPConnection("127.0.0.1", self.port, timeout=5)
        conn.request("GET", path)
        resp = conn.getresponse()
        data = json.loads(resp.read())
        conn.close()
        return data

    def _post(self, path: str, body: dict = None) -> dict:
        conn = HTTPConnection("127.0.0.1", self.port, timeout=10)
        headers = {}
        payload = b""
        if body:
            payload = json.dumps(body).encode()
            headers["Content-Type"] = "application/json"
        headers["Content-Length"] = str(len(payload))
        conn.request("POST", path, body=payload, headers=headers)
        resp = conn.getresponse()
        data = json.loads(resp.read())
        conn.close()
        return data

    def test_health(self):
        data = self._get("/api/health")
        self.assertTrue(data["ok"])
        self.assertIn("test", data["services"])

    def test_profiles(self):
        data = self._get("/api/profiles")
        self.assertEqual(len(data["profiles"]), 1)
        self.assertEqual(data["profiles"][0]["name"], "test")

    def test_status_stopped(self):
        data = self._get("/api/profiles/test/status")
        self.assertEqual(data["state"], "stopped")

    def test_unknown_profile(self):
        data = self._get("/api/profiles/nonexistent/status")
        self.assertFalse(data["ok"])

    def test_start_stop_lifecycle(self):
        # Start
        data = self._post("/api/profiles/test/start")
        self.assertTrue(data["ok"])

        time.sleep(1)

        # Logs
        logs = self._get("/api/profiles/test/logs?tail=10")
        self.assertIn("lines", logs)

        # Status (may be stopped already since echo exits fast)
        status = self._get("/api/profiles/test/status")
        self.assertIn(status["state"], ("running", "stopped"))

        # Stop (should be ok even if already stopped)
        data = self._post("/api/profiles/test/stop")
        self.assertTrue(data["ok"])

    def test_404_on_unknown_path(self):
        data = self._get("/api/nonexistent")
        self.assertFalse(data["ok"])


if __name__ == "__main__":
    unittest.main()
