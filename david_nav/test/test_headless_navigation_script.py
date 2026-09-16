import json
import os
from pathlib import Path
import select
import shlex
import signal
import subprocess
import sys
import time

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "start_navigation_headless.sh"


def test_headless_fixture_uses_current_private_overrides(tmp_path):
    env = _prepare_env(tmp_path)
    for key in (
        "DAVID_WS_DIR", "DAVID_ROS_SETUP_FILE", "DAVID_INSTALL_SETUP_FILE",
        "DAVID_ROS2_BIN", "DAVID_PYTHON_BIN", "DAVID_EXECUTOR_BIN",
        "POSE_FILE", "BAG_ROOT", "WHEELTEC_WS_DIR",
    ):
        assert Path(env[key]).is_relative_to(tmp_path), key
    assert env["ROS_DOMAIN_ID"] == "99"
    assert env["DAVID_SKIP_PREFLIGHT_CLEANUP"] == "true"


def _write_file(path: Path, content: str = "#!/usr/bin/env bash\n"):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _prepare_env(tmp_path: Path):
    ros_setup = tmp_path / "opt/ros/humble/setup.bash"
    auto_workspace = tmp_path / "ros2_ws"
    fake_python = tmp_path / "bin/fake_python3"
    fake_ifconfig = tmp_path / "bin/ifconfig"
    named_poses = tmp_path / "semantic_map/named_poses.json"
    # The real shell sources this after assigning its legacy lock constant.
    # Redirect only the path, keeping the actual lock cleanup function intact.
    _write_file(ros_setup, (
        'EXECUTOR_LOCK_FILE=' + shlex.quote(str(tmp_path / 'executor.lock')) + '\n'
        'printf "%s" "$$" > ' + shlex.quote(str(tmp_path / 'shell.pid')) + '\n'))
    _write_file(
        tmp_path
        / "wheelhumanoidrobot/workspace_wheeltec/wheeltec_robot_ros2/install/setup.bash"
    )
    _write_file(auto_workspace / "install/setup.bash")
    _write_file(
        named_poses,
        '[{"id":"POI_001","name":"1","frame_id":"map",'
        '"position":{"x":1.0,"y":2.0,"z":0.0},"yaw":0.5}]\n',
    )
    helper = Path(__file__).with_name('headless_process_fixture.py')
    for name, role in (
        ('fake_python3', 'python'), ('ros2', 'ros2'),
        ('navigation_executor.py', 'executor'), ('pgrep', 'pgrep'),
    ):
        path = tmp_path / 'bin' / name
        _write_file(path, '#!/bin/bash\nexec ' + shlex.join(
            [sys.executable, str(helper), role]) + ' "$@"\n')
        path.chmod(0o755)
    _write_file(fake_ifconfig, "#!/usr/bin/env bash\nexit 0\n")
    fake_ifconfig.chmod(0o755)
    # The script runs stop_stale_executor (pgrep + kill) and pin_jetson_clocks
    # (sudo nvpmodel / jetson_clocks) even with --dry-run. Shadow them so the
    # tests can never signal real processes or change the Jetson power mode
    # when they run on the robot.
    for name, exit_code in (
        ("sudo", 1),
        ("nvpmodel", 0),
        ("jetson_clocks", 0),
    ):
        shim = fake_ifconfig.parent / name
        _write_file(shim, f"#!/usr/bin/env bash\nexit {exit_code}\n")
        shim.chmod(0o755)

    env = os.environ.copy()
    env.update(
        {
            "PATH": f"{fake_ifconfig.parent}:{env.get('PATH', '')}",
            "DAVID_WS_DIR": str(auto_workspace),
            "DAVID_ROS_SETUP_FILE": str(ros_setup),
            "DAVID_INSTALL_SETUP_FILE": str(auto_workspace / 'install/setup.bash'),
            "DAVID_ROS2_BIN": str(tmp_path / 'bin/ros2'),
            "DAVID_PYTHON_BIN": str(fake_python),
            "DAVID_EXECUTOR_BIN": str(tmp_path / 'bin/navigation_executor.py'),
            "DAVID_CONFIG_HELPER_BIN": str(REPO_ROOT / 'scripts/navigation_headless_config.py'),
            "DAVID_HEADLESS_CONFIG_FILE": str(REPO_ROOT / 'config/navigation_headless.yaml'),
            "DAVID_HEADLESS_PROFILE": "navigation_no_explore",
            "POSE_FILE": str(named_poses),
            "BAG_ROOT": str(tmp_path / 'bags'),
            "WHEELTEC_WS_DIR": str(tmp_path / 'wheelhumanoidrobot/workspace_wheeltec/'
                                   'wheeltec_robot_ros2'),
            "HEADLESS_TEST_PACKAGE": str(REPO_ROOT),
            "HEADLESS_TEST_ROOT": str(tmp_path),
            "HEADLESS_TEST_STAGE": "executor",
            "ROS_DOMAIN_ID": "99",
            "ZERO_CMD_VEL_ON_SHUTDOWN": "false",
            "SHUTDOWN_GRACE_SECONDS": "1",
            "BRINGUP_LIVENESS_INTERVAL_SEC": "0.1",
            "ENABLE_CPU_ONLINE": "false",
            # 2026-07-27: 非 dry-run 测试在真机上跑时，禁止 pre-flight 的
            # pgrep 清理误杀真实运行中的 nav2/wheeltec 栈
            "DAVID_SKIP_PREFLIGHT_CLEANUP": "true",
        }
    )
    return env


def _run_script(tmp_path: Path, *args: str):
    return subprocess.run(
        ["bash", str(SCRIPT_PATH), *args],
        capture_output=True,
        check=False,
        cwd=str(REPO_ROOT),
        env=_prepare_env(tmp_path),
        text=True,
        timeout=10,
    )


def _run_script_async(
    tmp_path: Path,
    *args: str,
    extra_env: dict[str, str] | None = None,
):
    env = _prepare_env(tmp_path)
    if extra_env:
        env.update(extra_env)
    return subprocess.Popen(
        ["bash", str(SCRIPT_PATH), *args],
        cwd=str(REPO_ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )


def _pid_exists(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    return True


def _remember_children(tmp_path, handles):
    for label in ('detached', 'nav'):
        if label in handles:
            continue
        try:
            pid = int((tmp_path / (label + '.pid')).read_text())
            handle = os.pidfd_open(pid)
        except (OSError, ValueError):
            continue
        try:
            expected = ('HEADLESS_TEST_ROOT=' + str(tmp_path)).encode()
            belongs = expected in Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
        except OSError:
            os.close(handle)
            continue
        if not belongs:
            os.close(handle)
            continue
        handles[label] = handle


@pytest.mark.parametrize('role,args,code', [
    ('ros2', ['daemon', 'stop'], 97),
    ('ros2', ['topic', 'pub', '/cmd_vel'], 97),
    ('ros2', ['launch', 'turn_on_wheeltec_robot', 'anything.launch.py'], 97),
    ('python', ['-c', 'import rclpy'], 97),
    ('python', ['/tmp/not-a-test-program.py'], 97),
    ('pgrep', ['-f', 'navigation_executor.py'], 1),
    ('pgrep', ['-P', str(os.getpid())], 1),
])
def test_headless_fixture_refuses_unrecognized_host_operations(tmp_path, role, args, code):
    env = _prepare_env(tmp_path)
    helper = Path(__file__).with_name('headless_process_fixture.py')
    result = subprocess.run([sys.executable, str(helper), role, *args],
                            env=env, capture_output=True, text=True, timeout=3)
    assert result.returncode == code, result.stderr
    assert result.stdout == ''


def test_headless_fixture_rejects_non_sandbox_domain(tmp_path):
    env = _prepare_env(tmp_path)
    env['ROS_DOMAIN_ID'] = '0'
    result = subprocess.run([env['DAVID_ROS2_BIN'], 'daemon', 'stop'],
                            env=env, capture_output=True, text=True, timeout=3)
    assert result.returncode == 98
    assert not (tmp_path / 'calls.jsonl').exists()


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


@pytest.mark.parametrize('stage', ['startup', 'probe', 'executor'])
def test_headless_navigation_ctrl_c_kills_detached_descendants(tmp_path, stage):
    detached_pid_file = tmp_path / "detached.pid"
    process = _run_script_async(
        tmp_path,
        '--no-chassis', '--no-bag', '--skip-runtime-config-check',
        extra_env={'HEADLESS_TEST_STAGE': stage},
    )
    child_handles = {}
    early_output = b''
    try:
        deadline = time.monotonic() + 10.0
        ready = tmp_path / ('detached.pid' if stage == 'startup' else stage + '.pid')
        marker = b'Nav2 bringup started with pid'
        while time.monotonic() < deadline:
            _remember_children(tmp_path, child_handles)
            assert process.poll() is None, process.communicate(timeout=2)
            if ready.exists() and marker in early_output:
                break
            readable, _, _ = select.select([process.stdout], [], [], 0.05)
            if readable:
                early_output += os.read(process.stdout.fileno(), 4096)
        assert ready.exists(), 'foreground test replacement did not start'
        assert marker in early_output, 'script did not register the managed child'
        assert detached_pid_file.exists(), "detached child process did not start"
        detached_pid = int(detached_pid_file.read_text(encoding="utf-8").strip())
        assert _pid_exists(detached_pid)
        nav_pid = int((tmp_path / 'nav.pid').read_text())
        assert os.getpgid(detached_pid) == detached_pid
        assert os.getpgid(nav_pid) == nav_pid
        assert len({process.pid, detached_pid, nav_pid}) == 3
        # pidfds make failure-only cleanup immune to PID reuse. Open before
        # signalling; normal success must be established before this fallback.
        _remember_children(tmp_path, child_handles)
        assert set(child_handles) == {'detached', 'nav'}
        if stage == 'startup':
            # Retain the old single-PID injection while the script starts up.
            assert not (tmp_path / 'executor.pid').exists()
            process.send_signal(signal.SIGINT)
        else:
            # Terminal Ctrl+C reaches both the shell and foreground pipeline.
            os.killpg(process.pid, signal.SIGINT)
        stdout, stderr = process.communicate(timeout=15)
        stdout = (early_output + stdout).decode()
        stderr = stderr.decode()
        assert process.returncode == 130
        assert "Received INT" in stdout
        assert not _pid_exists(detached_pid), (
            f"detached child process {detached_pid} leaked after Ctrl+C\n"
            f"stdout:\n{stdout}\n"
            f"stderr:\n{stderr}\n"
        )
        assert not _pid_exists(nav_pid)
        calls = [json.loads(line) for line in (tmp_path / 'calls.jsonl').read_text().splitlines()]
        assert ['pgrep', '-P', str(nav_pid)] in calls
        assert ['pgrep', '-g', str(detached_pid)] in calls
        assert [call[1:4] for call in calls if call[0] == 'ros2'] == [
            ['launch', 'david_nav', '2d_david_nav2_bringup.launch.py']]
    finally:
        _remember_children(tmp_path, child_handles)
        for handle in child_handles.values():
            try:
                signal.pidfd_send_signal(handle, signal.SIGINT)
            except ProcessLookupError:
                pass
        # Allow the fake Nav2 parent to reap the detached child before forcing
        # anything. The assertions above never rely on this fallback cleanup.
        time.sleep(0.2)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5)
        for handle in child_handles.values():
            try:
                signal.pidfd_send_signal(handle, signal.SIGKILL)
            except ProcessLookupError:
                pass
            os.close(handle)
