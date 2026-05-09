#!/usr/bin/env python3
"""
G1 Navigation Launch Manager — HTTP API for start/stop/status/logs.

Manages start_navigation_headless.sh as a subprocess.
Designed to run as a systemd service on the G1 robot.
"""
from __future__ import annotations

import argparse
import collections
import datetime
import json
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from typing import Optional

import yaml

logger = logging.getLogger("launch_manager")

# ---------------------------------------------------------------------------
# Profile registry
# ---------------------------------------------------------------------------

DEFAULT_PROFILES_FILE = Path(__file__).parent / "profiles.yaml"


def load_profiles(path: Path) -> dict[str, dict]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    profiles = {}
    for name, entry in (data.get("profiles") or {}).items():
        profiles[name] = {
            "command": entry["command"],
            "description": entry.get("description", ""),
            "env": entry.get("env", {}),
        }
    return profiles


# ---------------------------------------------------------------------------
# Managed process
# ---------------------------------------------------------------------------

LOG_BUFFER_MAX = 2000


class ManagedProcess:
    """Wraps a subprocess with log capture and lifecycle tracking."""

    def __init__(self, profile_name: str, command: str, env: dict[str, str]):
        self.profile_name = profile_name
        self.command = command
        self.env = env
        self.process: Optional[subprocess.Popen] = None
        self.started_at: Optional[datetime.datetime] = None
        self.stopped_at: Optional[datetime.datetime] = None
        self.log_buffer: collections.deque[str] = collections.deque(
            maxlen=LOG_BUFFER_MAX
        )
        self._reader_thread: Optional[threading.Thread] = None
        self._state = "stopped"  # stopped | starting | running | stopping
        self._lock = threading.Lock()

    @property
    def state(self) -> str:
        with self._lock:
            if self._state in ("running", "starting"):
                if self.process and self.process.poll() is not None:
                    self._state = "stopped"
                    self.stopped_at = datetime.datetime.now(datetime.timezone.utc)
            return self._state

    @property
    def pid(self) -> Optional[int]:
        return self.process.pid if self.process else None

    @property
    def uptime_seconds(self) -> Optional[float]:
        if self.started_at and self.state in ("running", "starting"):
            return (
                datetime.datetime.now(datetime.timezone.utc) - self.started_at
            ).total_seconds()
        return None

    def start(self) -> dict:
        with self._lock:
            if self._state in ("running", "starting"):
                if self.process and self.process.poll() is None:
                    return {"ok": False, "error": f"Profile '{self.profile_name}' is already running (pid {self.process.pid})"}
            self._state = "starting"

        run_env = os.environ.copy()
        run_env.update(self.env)

        try:
            self.process = subprocess.Popen(
                ["bash", "-lc", self.command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                env=run_env,
                start_new_session=True,
            )
        except Exception as exc:
            with self._lock:
                self._state = "stopped"
            return {"ok": False, "error": str(exc)}

        self.started_at = datetime.datetime.now(datetime.timezone.utc)
        self.stopped_at = None
        self.log_buffer.clear()

        self._reader_thread = threading.Thread(
            target=self._read_output, daemon=True
        )
        self._reader_thread.start()

        with self._lock:
            self._state = "running"

        logger.info("Started profile '%s' with pid %d", self.profile_name, self.process.pid)
        return {"ok": True, "pid": self.process.pid}

    def stop(self, grace_seconds: float = 10.0) -> dict:
        with self._lock:
            if self._state == "stopped":
                return {"ok": True, "message": "Already stopped"}
            if self._state == "stopping":
                return {"ok": False, "error": "Already stopping"}
            self._state = "stopping"

        if self.process and self.process.poll() is None:
            # Send SIGTERM to the process group so the script's trap handler
            # can clean up all child ROS processes.
            # os.killpg / os.getpgid are POSIX-only; fall back to terminate()
            # on Windows so tests pass, while G1 (Linux) uses process groups.
            if hasattr(os, "getpgid"):
                pgid = None
                try:
                    pgid = os.getpgid(self.process.pid)
                except OSError:
                    pass
                if pgid:
                    try:
                        os.killpg(pgid, signal.SIGTERM)
                    except OSError:
                        pass
                else:
                    self.process.terminate()
            else:
                self.process.terminate()

            try:
                self.process.wait(timeout=grace_seconds)
            except subprocess.TimeoutExpired:
                logger.warning("Grace period expired, sending SIGKILL")
                if hasattr(os, "killpg"):
                    pgid = None
                    try:
                        pgid = os.getpgid(self.process.pid)
                    except OSError:
                        pass
                    if pgid:
                        try:
                            os.killpg(pgid, signal.SIGKILL)
                        except OSError:
                            pass
                self.process.kill()
                self.process.wait(timeout=5)

        with self._lock:
            self._state = "stopped"
        self.stopped_at = datetime.datetime.now(datetime.timezone.utc)
        logger.info("Stopped profile '%s'", self.profile_name)
        return {"ok": True}

    def status(self) -> dict:
        s = self.state
        result: dict = {"profile": self.profile_name, "state": s}
        if self.pid is not None:
            result["pid"] = self.pid
        if self.started_at:
            result["started_at"] = self.started_at.isoformat()
        if s in ("running", "starting") and self.uptime_seconds is not None:
            result["uptime_seconds"] = round(self.uptime_seconds, 1)
        if self.process and self.process.poll() is not None:
            result["exit_code"] = self.process.returncode
        return result

    def logs(self, tail: int = 100) -> list[str]:
        lines = list(self.log_buffer)
        return lines[-tail:]

    def _read_output(self):
        assert self.process and self.process.stdout
        try:
            for raw_line in self.process.stdout:
                try:
                    line = raw_line.decode("utf-8", errors="replace").rstrip("\n")
                except Exception:
                    line = repr(raw_line)
                self.log_buffer.append(line)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Process manager (multi-profile)
# ---------------------------------------------------------------------------

class ProcessManager:
    def __init__(self, profiles: dict[str, dict]):
        self.profiles = profiles
        self._managed: dict[str, ManagedProcess] = {}
        self._lock = threading.Lock()

    def get_or_create(self, profile_name: str) -> ManagedProcess:
        with self._lock:
            if profile_name not in self._managed:
                cfg = self.profiles[profile_name]
                self._managed[profile_name] = ManagedProcess(
                    profile_name=profile_name,
                    command=cfg["command"],
                    env=cfg.get("env", {}),
                )
            return self._managed[profile_name]

    def stop_all(self):
        with self._lock:
            names = list(self._managed.keys())
        for name in names:
            proc = self._managed.get(name)
            if proc and proc.state in ("running", "starting"):
                proc.stop()


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------

class LaunchManagerHandler(BaseHTTPRequestHandler):
    manager: ProcessManager
    profiles: dict[str, dict]

    def log_message(self, fmt, *args):
        logger.debug(fmt, *args)

    def _send_json(self, data: dict, status: int = 200):
        body = json.dumps(data, ensure_ascii=False, indent=2).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_body(self) -> dict:
        length = int(self.headers.get("Content-Length", 0))
        if length == 0:
            return {}
        raw = self.rfile.read(length)
        return json.loads(raw)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        path = self.path.split("?")[0].rstrip("/")
        query = self._parse_query()

        if path == "/api/health":
            health = {}
            for name in self.profiles:
                proc = self.manager.get_or_create(name)
                health[name] = proc.state
            self._send_json({"ok": True, "services": health})
            return

        if path == "/api/profiles":
            result = []
            for name, cfg in self.profiles.items():
                proc = self.manager.get_or_create(name)
                result.append({
                    "name": name,
                    "description": cfg.get("description", ""),
                    "state": proc.state,
                })
            self._send_json({"profiles": result})
            return

        # /api/profiles/{name}/status
        parts = path.split("/")
        if len(parts) == 5 and parts[1] == "api" and parts[2] == "profiles" and parts[4] == "status":
            name = parts[3]
            if name not in self.profiles:
                self._send_json({"ok": False, "error": f"Unknown profile: {name}"}, 404)
                return
            proc = self.manager.get_or_create(name)
            self._send_json(proc.status())
            return

        # /api/profiles/{name}/logs
        if len(parts) == 5 and parts[1] == "api" and parts[2] == "profiles" and parts[4] == "logs":
            name = parts[3]
            if name not in self.profiles:
                self._send_json({"ok": False, "error": f"Unknown profile: {name}"}, 404)
                return
            tail = int(query.get("tail", "100"))
            proc = self.manager.get_or_create(name)
            self._send_json({"lines": proc.logs(tail=tail)})
            return

        self._send_json({"ok": False, "error": "Not found"}, 404)

    def do_POST(self):
        path = self.path.split("?")[0].rstrip("/")
        parts = path.split("/")

        # /api/profiles/{name}/start
        if len(parts) == 5 and parts[1] == "api" and parts[2] == "profiles" and parts[4] == "start":
            name = parts[3]
            if name not in self.profiles:
                self._send_json({"ok": False, "error": f"Unknown profile: {name}"}, 404)
                return
            body = self._read_body()
            proc = self.manager.get_or_create(name)

            # Allow overriding options via request body
            if body:
                cfg = self.profiles[name]
                command = cfg["command"]
                if body.get("no_exploration"):
                    command += " --no-exploration"
                if body.get("no_realsense"):
                    command += " --no-realsense"
                if body.get("rviz"):
                    command += " --rviz"
                proc.command = command

            result = proc.start()
            status_code = 200 if result.get("ok") else 409
            self._send_json(result, status_code)
            return

        # /api/profiles/{name}/stop
        if len(parts) == 5 and parts[1] == "api" and parts[2] == "profiles" and parts[4] == "stop":
            name = parts[3]
            if name not in self.profiles:
                self._send_json({"ok": False, "error": f"Unknown profile: {name}"}, 404)
                return
            proc = self.manager.get_or_create(name)
            result = proc.stop()
            self._send_json(result)
            return

        self._send_json({"ok": False, "error": "Not found"}, 404)

    def _parse_query(self) -> dict[str, str]:
        if "?" not in self.path:
            return {}
        qs = self.path.split("?", 1)[1]
        result = {}
        for part in qs.split("&"):
            if "=" in part:
                k, v = part.split("=", 1)
                result[k] = v
        return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="G1 Navigation Launch Manager")
    parser.add_argument("--host", default="0.0.0.0", help="Listen address (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Listen port (default: 8080)")
    parser.add_argument("--profiles", type=Path, default=DEFAULT_PROFILES_FILE,
                        help="Path to profiles.yaml")
    parser.add_argument("--web-dir", type=Path, default=Path(__file__).parent / "web",
                        help="Directory containing static web files")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="[launch_manager] %(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )

    profiles = load_profiles(args.profiles)
    logger.info("Loaded %d profiles: %s", len(profiles), ", ".join(profiles.keys()))

    manager = ProcessManager(profiles)
    web_dir = args.web_dir

    class Handler(LaunchManagerHandler):
        pass

    Handler.manager = manager
    Handler.profiles = profiles

    # Serve static files from web_dir for non-API paths
    _orig_do_GET = Handler.do_GET

    def do_GET_with_static(self):
        path = self.path.split("?")[0]
        if path.startswith("/api/"):
            return _orig_do_GET(self)

        # Serve static files
        if path == "/" or path == "":
            path = "/index.html"

        file_path = web_dir / path.lstrip("/")
        if file_path.is_file():
            content_type = "text/html; charset=utf-8"
            if path.endswith(".js"):
                content_type = "application/javascript; charset=utf-8"
            elif path.endswith(".css"):
                content_type = "text/css; charset=utf-8"
            elif path.endswith(".json"):
                content_type = "application/json; charset=utf-8"
            elif path.endswith(".png"):
                content_type = "image/png"
            elif path.endswith(".svg"):
                content_type = "image/svg+xml"

            data = file_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(data)
        else:
            self._send_json({"ok": False, "error": "Not found"}, 404)

    Handler.do_GET = do_GET_with_static

    server = HTTPServer((args.host, args.port), Handler)
    logger.info("Listening on http://%s:%d", args.host, args.port)
    logger.info("Web UI: http://%s:%d/", args.host, args.port)
    logger.info("API: http://%s:%d/api/health", args.host, args.port)

    def shutdown_handler(signum, frame):
        logger.info("Shutting down...")
        manager.stop_all()
        server.shutdown()

    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGINT, shutdown_handler)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        manager.stop_all()
        server.server_close()


if __name__ == "__main__":
    main()
