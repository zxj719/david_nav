from setuptools import setup
from glob import glob

package_name = "david_sdk_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agx",
    maintainer_email="agx@example.com",
    description="Rokae xCoreSDK <-> ROS 2 bridge for MoveIt",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_node = david_sdk_bridge.arm_node:main",
        ],
    },
)
