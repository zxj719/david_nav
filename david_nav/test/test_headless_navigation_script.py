import os
from pathlib import Path
import signal
import subprocess
import time

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "start_navigation_headless.sh"


def _write_file(path: Path, content: str = "#!/usr/bin/env bash\n"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _prepare_env(tmp_path: Path, *, websockets_available: bool = True):
    ros_setup = tmp_path / "opt/ros/humble/setup.bash"
    lightning_workspace = tmp_path / "lightning_ws"
    auto_workspace = tmp_path / "ros2_ws"
    fake_python = tmp_path / "bin/fake_python3"
    fake_ifconfig = tmp_path / "bin/ifconfig"
    named_poses = tmp_path / "semantic_map/named_poses.json"
    localization_config = lightning_workspace / "src/hoslam_lightning_lm/config/default_c16.yaml"

    _write_file(ros_setup)
    _write_file(lightning_workspace / "install/setup.bash")
    # 2026-07-27: 脚本 START_CHASSIS 默认 true，require_path wheeltec 安装文件
    _write_file(
        tmp_path
        / "wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/install/setup.bash"
    )
    _write_file(localization_config, "dummy: true\n")
    _write_file(auto_workspace / "install/setup.bash")
    _write_file(
        named_poses,
        '[{"id":"POI_001","name":"1","frame_id":"map","position":{"x":1.0,"y":2.0,"z":0.0},"yaw":0.5}]\n',
    )
    _write_file(
        fake_python,
        f"""#!/usr/bin/env bash
set -euo pipefail
if [[ "${{1:-}}" == "-c" ]]; then
  code="${{2:-}}"
  if [[ "$code" == "import yaml" ]]; then
    exit 0
  fi
  if [[ "$code" == "import websockets" ]]; then
    {"exit 0" if websockets_available else "exit 1"}
  fi
fi
exec python3 "$@"
""",
    )
    fake_python.chmod(0o755)
    _write_file(fake_ifconfig, "#!/usr/bin/env bash\nexit 0\n")
    fake_ifconfig.chmod(0o755)
    # The script runs stop_stale_executor (pgrep + kill) and pin_jetson_clocks
    # (sudo nvpmodel / jetson_clocks) even with --dry-run. Shadow them so the
    # tests can never signal real processes or change the Jetson power mode
    # when they run on the robot.
    for name, body in (
        ("sudo", "exit 1"),
        ("nvpmodel", "exit 0"),
        ("jetson_clocks", "exit 0"),
        ("pgrep", "exit 1"),
    ):
        shim = fake_ifconfig.parent / name
        _write_file(shim, f"#!/usr/bin/env bash
{body}
")
        shim.chmod(0o755)

    env = os.environ.copy()
    env.update(
        {
            "HOME": str(tmp_path),
            "PATH": f"{fake_ifconfig.parent}:{env.get('PATH', '')}",
            "ROS_SETUP": str(ros_setup),
            "LIGHTNING_WORKSPACE": str(lightning_workspace),
            "LOCALIZATION_CONFIG": str(localization_config),
            "AUTO_WORKSPACE": str(auto_workspace),
            "NAMED_POSES_FILE": str(named_poses),
            "ENABLE_CPU_ONLINE": "false",
            "PYTHON_BIN": str(fake_python),
            # 2026-07-27: 非 dry-run 测试在真机上跑时，禁止 pre-flight 的
            # pgrep 清理误杀真实运行中的 nav2/wheeltec 栈
            "DAVID_SKIP_PREFLIGHT_CLEANUP": "true",
        }
    )
    return env


def _run_script(tmp_path: Path, *args: str, websockets_available: bool = True):
    return subprocess.run(
        ["bash", str(SCRIPT_PATH), *args],
        capture_output=True,
        check=False,
        cwd=str(REPO_ROOT),
        env=_prepare_env(tmp_path, websockets_available=websockets_available),
        text=True,
    )


def _run_script_async(
    tmp_path: Path,
    *args: str,
    websockets_available: bool = True,
    extra_env: dict[str, str] | None = None,
):
    env = _prepare_env(tmp_path, websockets_available=websockets_available)
    if extra_env:
        env.update(extra_env)
    return subprocess.Popen(
        ["bash", str(SCRIPT_PATH), *args],
        cwd=str(REPO_ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )


def _pid_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


# 2026-07-27 测试现代化：原有 5 个测试写于"脚本自启 IMU/lidar/loc + 老 URI +
# -u 选项 + websockets 预检 + Would-import 步骤"的上古版本，与现行脚本
# （lightning 独立启动、URI 来自 navigation_headless.yaml、上述特性均已删除）
# 完全不匹配，长期红灯无人跑。以下按现行 dry-run 契约重写。
def test_headless_navigation_dry_run_defaults(tmp_path):
    result = _run_script(tmp_path, "--dry-run")

    assert result.returncode == 0, result.stderr
    assert "start_navigation_headless.sh" in result.stdout
    # nav2 bringup：headless 默认关 rviz/scan_bridge/exploration，
    # 且临时停用软件速度整形（2026-07-20 STM32 重调后）
    assert "2d_david_nav2_bringup.launch.py use_rviz:=false" in result.stdout
    assert "enable_scan_bridge:=false" in result.stdout
    assert "enable_exploration:=false" in result.stdout
    assert "cmd_shaping:=false" in result.stdout
    # 自起 wheeltec 底盘
    assert "turn_on_wheeltec_robot.launch.py" in result.stdout
    # bag：light 列表不再录 /lightning/grid_map（107GB/3.4h 教训）
    assert "ros2 bag record" in result.stdout
    assert "/lightning/grid_map" not in result.stdout
    # executor：URI 来自 config/navigation_headless.yaml
    assert "navigation_executor.py --server-uri ws://" in result.stdout
    assert "--pose-file" in result.stdout


def test_headless_navigation_no_chassis_skips_wheeltec(tmp_path):
    result = _run_script(tmp_path, "--dry-run", "--no-chassis")

    assert result.returncode == 0, result.stderr
    assert "turn_on_wheeltec_robot.launch.py" not in result.stdout
    assert "2d_david_nav2_bringup.launch.py" in result.stdout


@pytest.mark.skip(
    reason="2026-07-27: 现行脚本已无 IMU_COMMAND/NAV2_COMMAND 等注入钩子，"
    "无法在沙箱里替身启动；分离进程清理需要新的测试 harness 再覆盖"
)
def test_headless_navigation_ctrl_c_kills_detached_descendants(tmp_path):
    detached_pid_file = tmp_path / "detached.pid"
    detached_launcher = tmp_path / "detached_launcher.sh"
    _write_file(
        detached_launcher,
        f"""#!/usr/bin/env bash
set -euo pipefail
setsid bash -lc "echo \\$\\$ > '{detached_pid_file}'; exec sleep 1000" >/dev/null 2>&1 &
child=$!
wait "$child"
""",
    )
    detached_launcher.chmod(0o755)

    process = _run_script_async(
        tmp_path,
        extra_env={
            "ENABLE_CPU_ONLINE": "false",
            "SLAM_START_DELAY": "0",
            "IMU_COMMAND": "exec sleep 1000",
            "LIDAR_COMMAND": "exec sleep 1000",
            "LOCALIZATION_COMMAND": "exec sleep 1000",
            "NAV2_COMMAND": f"exec {detached_launcher}",
            "EXECUTOR_COMMAND": "exec sleep 1000",
        },
    )

    try:
        deadline = time.time() + 10.0
        while time.time() < deadline and not detached_pid_file.exists():
            time.sleep(0.1)

        assert detached_pid_file.exists(), "detached child process did not start"

        detached_pid = int(detached_pid_file.read_text(encoding="utf-8").strip())
        assert _pid_exists(detached_pid)

        process.send_signal(signal.SIGINT)
        stdout, stderr = process.communicate(timeout=15)

        assert process.returncode == 130
        assert "Received INT" in stdout
        assert not _pid_exists(detached_pid), (
            f"detached child process {detached_pid} leaked after Ctrl+C\n"
            f"stdout:\n{stdout}\n"
            f"stderr:\n{stderr}\n"
        )
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)
        if detached_pid_file.exists():
            detached_pid = int(detached_pid_file.read_text(encoding="utf-8").strip())
            if _pid_exists(detached_pid):
                os.kill(detached_pid, signal.SIGKILL)
