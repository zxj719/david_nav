"""Package setup for nav_perception_iface."""

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = 'nav_perception_iface'

setup(
    name=PACKAGE_NAME, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md']),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE_NAME + '/config', ['config/schema.yaml']),
        ('share/' + PACKAGE_NAME + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zhangxingjian', maintainer_email='xingjian.zhang@amicro.com.cn',
    description='Localization and perception contracts for robot navigation.',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'nav_interface_check = nav_perception_iface.contract_cli:main',
        'localization_monitor = nav_perception_iface.health_node:main',
    ]},
)
