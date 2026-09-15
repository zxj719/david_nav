"""Package setup for nav_bringup."""

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'nav_bringup'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md']),
        ('share/' + PACKAGE_NAME + '/config', ['config/schema.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhangxingjian',
    maintainer_email='xingjian.zhang@amicro.com.cn',
    description='Bringup, run recording and operations tooling for navigation stacks.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navcfg = nav_bringup.navcfg.cli:main',
            'navrun = nav_bringup.navrun.cli:main',
        ],
    },
)
