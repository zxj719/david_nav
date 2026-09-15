"""Install the guarded standalone keyboard and reusable control/TTY components."""

from setuptools import find_packages, setup

PACKAGE = 'chassis_teleop'

setup(
    name=PACKAGE, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
        ('share/' + PACKAGE + '/config', ['config/schema.yaml']),
        ('share/' + PACKAGE + '/launch', ['launch/keyboard_teleop.launch.py']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zhangxingjian', maintainer_email='xingjian.zhang@amicro.com.cn',
    description='Keyboard motion state and terminal input without ROS or hardware ownership.',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'chassis_teleop = chassis_teleop.node:main',
        'chassis_teleop_managed = chassis_teleop.node:managed_main',
    ]},
)
