import asyncio
import os
from pathlib import Path
import xml.etree.ElementTree as ET

from legacy_entry_reference import load_entry


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "david_nav" / "navigation_executor.py"


def _load_module():
    return load_entry('navigation_executor')


def test_navigation_executor_cli_parses_server_uri_and_store_path():
    module = _load_module()

    config, ros_args = module._parse_cli_args(
        [
            "--server-uri",
            "ws://10.0.0.5:8100/ws/navigation/executor",
            "--poi-store-file",
            "/tmp/poi_store.yaml",
            "--ros-args",
            "-r",
            "__ns:=/robot",
        ]
    )

    assert config["server_uri"] == "ws://10.0.0.5:8100/ws/navigation/executor"
    assert config["poi_store_file"] == "/tmp/poi_store.yaml"
    assert ros_args == ["--ros-args", "-r", "__ns:=/robot"]


def test_navigation_executor_cli_parses_nav_retry_options():
    # 失败消抖(2026-07-24):默认失败后静默重发 1 次、间隔 1s;可用
    # --nav-retry-count / --nav-retry-delay 调整。
    module = _load_module()

    default_config, _ = module._parse_cli_args([])
    assert default_config["nav_retry_count"] == 1
    assert default_config["nav_retry_delay_sec"] == 1.0

    config, ros_args = module._parse_cli_args(
        ["--nav-retry-count", "2", "--nav-retry-delay", "0.5"]
    )
    assert config["nav_retry_count"] == 2
    assert config["nav_retry_delay_sec"] == 0.5
    assert ros_args == []


def test_navigation_executor_uses_nav_ws_v1_default_server_uri():
    # nav WS v1 checklist:默认 $NAV_WS_URL,否则 ws://127.0.0.1:8101。
    module = _load_module()

    assert module.DEFAULT_SERVER_URI == os.environ.get(
        "NAV_WS_URL", "ws://127.0.0.1:8101"
    )


def test_navigation_executor_log_prefix_is_stable(capsys):
    module = _load_module()

    module._log("connected")

    captured = capsys.readouterr()
    assert captured.out == "[navigation_executor] connected\n"


def test_navigation_executor_patches_asyncio_for_legacy_websockets():
    module = _load_module()

    module._patch_asyncio_for_legacy_websockets()
    lock = asyncio.Lock(loop=None)

    assert hasattr(lock, "acquire")


def test_navigation_executor_spins_ros_node_in_async_loop():
    module = _load_module()
    spin_calls = []

    def fake_spin_once(node, timeout_sec):
        spin_calls.append((node, timeout_sec))

    async def run_scenario():
        task = asyncio.create_task(
            module._spin_ros_node("node", fake_spin_once, interval_sec=0.0)
        )
        await asyncio.sleep(0)
        await asyncio.sleep(0)
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    asyncio.run(run_scenario())

    assert spin_calls
    assert spin_calls[0] == ("node", 0.0)


def test_navigation_executor_drain_outbound_queue_sends_without_inbound_message():
    module = _load_module()
    sent_messages = []

    class FakeWebSocket:
        async def send(self, payload):
            sent_messages.append(payload)

    async def run_scenario():
        outbound_queue = asyncio.Queue()
        pending: list = []
        await outbound_queue.put({"event_type": "on_error", "request_id": "req_1"})
        drain_task = asyncio.create_task(
            module._drain_outbound_queue(FakeWebSocket(), outbound_queue, pending)
        )
        await asyncio.sleep(0)
        drain_task.cancel()
        try:
            await drain_task
        except asyncio.CancelledError:
            pass
        return pending

    pending_after = asyncio.run(run_scenario())

    assert sent_messages == [
        '{"event_type": "on_error", "request_id": "req_1"}'
    ]
    # 发送成功后 pending 应清空
    assert pending_after == []


def test_navigation_executor_drain_outbound_queue_resends_after_disconnect():
    """2026-07-27 送达保证：断联瞬间(send 抛异常)事件保留在 pending_holder，
    重连后新 drain 任务优先重发 —— 修复"到点了但 on_arrived 落在断联窗口
    丢失 → 服务端超时判导航失败"。"""
    module = _load_module()
    sent_messages = []

    class DeadWebSocket:
        async def send(self, payload):
            raise ConnectionError("connection lost mid-send")

    class LiveWebSocket:
        async def send(self, payload):
            sent_messages.append(payload)

    async def run_scenario():
        outbound_queue = asyncio.Queue()
        pending: list = []
        await outbound_queue.put({"event_type": "on_arrived", "request_id": "req_1"})
        # 第一次连接：send 失败，事件必须留在 pending
        dead_task = asyncio.create_task(
            module._drain_outbound_queue(DeadWebSocket(), outbound_queue, pending)
        )
        try:
            await dead_task
        except ConnectionError:
            pass
        assert pending == [{"event_type": "on_arrived", "request_id": "req_1"}]
        # "重连"后：新 drain 任务优先重发 pending
        live_task = asyncio.create_task(
            module._drain_outbound_queue(LiveWebSocket(), outbound_queue, pending)
        )
        await asyncio.sleep(0)
        live_task.cancel()
        try:
            await live_task
        except asyncio.CancelledError:
            pass
        return pending

    pending_after = asyncio.run(run_scenario())

    assert sent_messages == [
        '{"event_type": "on_arrived", "request_id": "req_1"}'
    ]
    assert pending_after == []


def test_navigation_executor_help_does_not_require_runtime_imports(capsys, monkeypatch):
    module = _load_module()

    def fail_asyncio_run(_coroutine):
        raise AssertionError("asyncio.run should not be reached for --help")

    monkeypatch.setattr(module.asyncio, "run", fail_asyncio_run)

    module.main(["--help"])

    captured = capsys.readouterr()
    assert "usage: navigation_executor.py" in captured.out
    assert "--server-uri URI" in captured.out
    assert "--poi-store-file PATH" in captured.out


def test_package_xml_declares_python3_websockets():
    root = ET.parse(REPO_ROOT / "package.xml").getroot()
    exec_depends = [node.text for node in root.findall("exec_depend")]

    assert "python3-websockets" in exec_depends


def test_cmake_installs_navigation_executor_script():
    cmake_text = (REPO_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    assert "david_nav/navigation_executor.py" in cmake_text
    assert "RENAME navigation_executor.py" in cmake_text
