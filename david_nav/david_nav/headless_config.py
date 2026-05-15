from __future__ import annotations

from pathlib import Path

import yaml


def _resolve_default_headless_config_path():
    module_path = Path(__file__).resolve()
    source_candidate = module_path.parents[1] / "config" / "navigation_headless.yaml"
    if source_candidate.is_file():
        return source_candidate

    for parent in module_path.parents:
        share_candidate = (
            parent / "share" / "david_nav" / "config" / "navigation_headless.yaml"
        )
        if share_candidate.is_file():
            return share_candidate

    return source_candidate


DEFAULT_HEADLESS_CONFIG_PATH = _resolve_default_headless_config_path()


class HeadlessConfigError(ValueError):
    pass


def load_headless_config(config_path=DEFAULT_HEADLESS_CONFIG_PATH):
    config_path = Path(config_path).expanduser()
    if not config_path.is_file():
        raise HeadlessConfigError(f"headless config file not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    if not isinstance(data, dict):
        raise HeadlessConfigError(
            f"invalid headless config root in {config_path}"
        )

    data["_config_path"] = str(config_path)
    return data


def resolve_headless_profile(config, profile_name=None):
    profiles = config.get("profiles") or {}
    if not isinstance(profiles, dict) or not profiles:
        raise HeadlessConfigError("headless config must define profiles")

    selected_name = profile_name or config.get("default_profile")
    selected = profiles.get(selected_name)
    if not isinstance(selected, dict):
        raise HeadlessConfigError(f"unknown headless profile: {selected_name}")

    return {
        "name": selected_name,
        "enable_exploration": bool(selected.get("enable_exploration", True)),
        "enable_scan_bridge": bool(selected.get("enable_scan_bridge", False)),
        "enable_realsense_scan_bridge": bool(
            selected.get("enable_realsense_scan_bridge", True)
        ),
    }


def build_executor_settings(config, profile_name=None):
    profile = resolve_headless_profile(config, profile_name)
    executor_cfg = config.get("executor") or {}
    launch_cfg = config.get("launch") or {}
    cleanup_cfg = config.get("cleanup") or {}

    process_matchers = cleanup_cfg.get("process_matchers", [])
    if not isinstance(process_matchers, list):
        raise HeadlessConfigError("cleanup.process_matchers must be a list")

    return {
        "profile": profile["name"],
        "enable_exploration": profile["enable_exploration"],
        "enable_scan_bridge": profile["enable_scan_bridge"],
        "enable_realsense_scan_bridge": profile["enable_realsense_scan_bridge"],
        "server_uri": str(executor_cfg.get("server_uri", "")).strip(),
        "server_timeout": float(executor_cfg.get("server_timeout", 10.0)),
        "heartbeat_interval": float(executor_cfg.get("heartbeat_interval", 30.0)),
        "reconnect_delay": float(executor_cfg.get("reconnect_delay", 3.0)),
        "nav_event_timeout": float(executor_cfg.get("nav_event_timeout", 15.0)),
        "show_logs": bool(executor_cfg.get("show_logs", True)),
        "record_bag": bool(executor_cfg.get("record_bag", False)),
        "use_rviz": bool(launch_cfg.get("use_rviz", False)),
        "nav2_wait_timeout_sec": int(launch_cfg.get("nav2_wait_timeout_sec", 120)),
        "slam_start_delay_sec": int(launch_cfg.get("slam_start_delay_sec", 5)),
        "process_matchers": [str(m).strip() for m in process_matchers],
        "term_timeout_sec": int(cleanup_cfg.get("term_timeout_sec", 5)),
        "kill_timeout_sec": int(cleanup_cfg.get("kill_timeout_sec", 2)),
        "config_path": str(config.get("_config_path", DEFAULT_HEADLESS_CONFIG_PATH)),
    }
