import importlib.util
from pathlib import Path
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "g1_nav" / "odometry_tf_broadcaster.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "g1_nav_odometry_tf_broadcaster",
        MODULE_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_transform_uses_odometry_frames_and_pose():
    module = _load_module()
    odom_msg = SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=12, nanosec=34),
            frame_id="odom",
        ),
        child_frame_id="base",
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.25, y=-0.5, z=0.1),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.1, w=0.99),
            )
        ),
    )

    transform = module.build_transform_from_odometry(odom_msg)

    assert transform.header.frame_id == "odom"
    assert transform.child_frame_id == "base"
    assert transform.header.stamp == odom_msg.header.stamp
    assert transform.transform.translation.x == 1.25
    assert transform.transform.translation.y == -0.5
    assert transform.transform.translation.z == 0.1
    assert transform.transform.rotation.z == 0.1
    assert transform.transform.rotation.w == 0.99
