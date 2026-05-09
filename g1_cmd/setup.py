import os
from glob import glob

from setuptools import find_packages, setup


package_name = "g1_cmd"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.*")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
        (
            os.path.join("share", package_name, "map"),
            glob(os.path.join("map", "*.*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FLYivan",
    maintainer_email="luoyifan902008@126.com",
    description="Wheeltec base command bridge and evaluation tools",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "g1_move = g1_cmd.g1_move:main",
            "wheeltec_base_eval = g1_cmd.wheeltec_base_eval:main",
            "wheeltec_imu_cmd_probe = g1_cmd.wheeltec_imu_cmd_probe:main",
            "wheeltec_micro_eval = g1_cmd.wheeltec_micro_eval:main",
        ],
    },
)
