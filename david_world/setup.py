from setuptools import setup
from glob import glob
package_name = "david_world"
setup(
    name=package_name, version="0.1.0", packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/scenes", glob("scenes/*.yaml")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="agx", maintainer_email="agx@example.com",
    description="David MoveIt scene loader",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "scene_loader = david_world.scene_loader:main",
        ],
    },
)
