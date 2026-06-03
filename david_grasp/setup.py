from setuptools import setup
from glob import glob

package_name = "david_grasp"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agx",
    maintainer_email="agx@example.com",
    description="David vision-based grasping pipeline",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # Stage 3-5 入口将逐步填充
            "pick_at_pose = david_grasp.pick_at_pose:main",
            "greet = david_grasp.greet:main",
            "pick_cartesian = david_grasp.pick_cartesian:main",
            "mock_grasp_pose = david_grasp.mock_grasp_pose:main",
            "goto_pose = david_grasp.goto_pose:main",
            "pick_from_cam = david_grasp.pick_from_cam:main",
            "grab_server = david_grasp.grab_server:main",
            "arm_grab_ws_client = david_grasp.arm_grab_ws_client:main",
            # "force_grasp = david_grasp.force_grasp:main",
            # "vision_pick = david_grasp.vision_pick:main",
            # "grasp_pipeline = david_grasp.grasp_pipeline:main",
        ],
    },
)
