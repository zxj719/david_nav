import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / 'plan' / 'realsense_scan_cpu_monitor.py'


def _load_module():
    spec = importlib.util.spec_from_file_location(
        'realsense_scan_cpu_monitor',
        MODULE_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_classify_process_matches_camera_and_scan_bridge_nodes():
    module = _load_module()

    assert module.classify_process(
        'realsense2_camera_node',
        '/home/unitree/ros2_ws/install/realsense2_camera/lib/realsense2_camera/realsense2_camera_node',
    ) == 'realsense_camera'
    assert module.classify_process(
        'depthimage_to_laserscan_node',
        '/opt/ros/humble/lib/depthimage_to_laserscan/depthimage_to_laserscan_node --ros-args -r __node:=realsense_depth_to_scan',
    ) == 'depth_to_scan'
    assert module.classify_process(
        'python3',
        '/usr/bin/python3 /opt/ros/humble/bin/ros2 launch g1_nav g1_auto_explore.launch.py',
    ) is None


def test_summarize_category_samples_reports_average_and_peak_cpu():
    module = _load_module()

    summary = module.summarize_category_samples({
        'realsense_camera': [11.0, 14.0, 7.0],
        'depth_to_scan': [4.5, 6.0],
        'missing': [],
    })

    assert summary['realsense_camera'] == {
        'samples': 3,
        'avg_cpu_percent': 10.67,
        'max_cpu_percent': 14.0,
    }
    assert summary['depth_to_scan'] == {
        'samples': 2,
        'avg_cpu_percent': 5.25,
        'max_cpu_percent': 6.0,
    }
    assert summary['missing'] == {
        'samples': 0,
        'avg_cpu_percent': 0.0,
        'max_cpu_percent': 0.0,
    }
