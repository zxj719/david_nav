#!/usr/bin/env python3
"""Regression tests for deploy.sh."""
from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


SCRIPT_DIR = Path(__file__).parent.resolve()
DEPLOY_SCRIPT = SCRIPT_DIR / "deploy.sh"


class TestDeployScript(unittest.TestCase):
    def test_deploy_uses_current_g1_ip_by_default(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            mockbin = tmp / "bin"
            mockbin.mkdir()
            rsync_log = tmp / "rsync.log"

            (mockbin / "ssh").write_text(
                "#!/usr/bin/env bash\n"
                "exit 0\n",
                encoding="utf-8",
            )
            (mockbin / "rsync").write_text(
                "#!/usr/bin/env bash\n"
                "printf '%s\\n' '---' >> \"$MOCK_RSYNC_LOG\"\n"
                "printf '%s\\n' \"$@\" >> \"$MOCK_RSYNC_LOG\"\n"
                "exit 0\n",
                encoding="utf-8",
            )
            os.chmod(mockbin / "ssh", 0o755)
            os.chmod(mockbin / "rsync", 0o755)

            env = os.environ.copy()
            env["PATH"] = f"{mockbin}:{env['PATH']}"
            env["MOCK_RSYNC_LOG"] = str(rsync_log)

            result = subprocess.run(
                ["bash", str(DEPLOY_SCRIPT)],
                cwd=SCRIPT_DIR,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn(
                "unitree@172.16.22.130:/home/unitree/ros2_ws/src/g1_nav/operator_console/",
                rsync_log.read_text(encoding="utf-8"),
            )

    def test_deploy_syncs_full_operator_console_directory(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            tmp = Path(tmpdir)
            mockbin = tmp / "bin"
            mockbin.mkdir()
            rsync_log = tmp / "rsync.log"

            (mockbin / "ssh").write_text(
                "#!/usr/bin/env bash\n"
                "exit 0\n",
                encoding="utf-8",
            )
            (mockbin / "rsync").write_text(
                "#!/usr/bin/env bash\n"
                "printf '%s\\n' '---' >> \"$MOCK_RSYNC_LOG\"\n"
                "printf '%s\\n' \"$@\" >> \"$MOCK_RSYNC_LOG\"\n"
                "exit 0\n",
                encoding="utf-8",
            )
            os.chmod(mockbin / "ssh", 0o755)
            os.chmod(mockbin / "rsync", 0o755)

            env = os.environ.copy()
            env["PATH"] = f"{mockbin}:{env['PATH']}"
            env["MOCK_RSYNC_LOG"] = str(rsync_log)
            env["G1_USER"] = "tester"

            result = subprocess.run(
                ["bash", str(DEPLOY_SCRIPT), "192.0.2.10"],
                cwd=SCRIPT_DIR,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)

            rsync_calls: list[list[str]] = []
            current: list[str] = []
            for line in rsync_log.read_text(encoding="utf-8").splitlines():
                if line == "---":
                    if current:
                        rsync_calls.append(current)
                    current = []
                    continue
                current.append(line)
            if current:
                rsync_calls.append(current)

            self.assertEqual(
                rsync_calls,
                [[
                    "-avz",
                    "--delete",
                    f"{SCRIPT_DIR}/",
                    "tester@192.0.2.10:/home/tester/ros2_ws/src/g1_nav/operator_console/",
                ]],
            )


if __name__ == "__main__":
    unittest.main()
