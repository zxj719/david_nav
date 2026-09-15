"""Package setup for david_config."""

from glob import glob

from setuptools import setup

PACKAGE_NAME = 'david_config'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md']),
        ('share/' + PACKAGE_NAME + '/robot', glob('robot/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhangxingjian',
    maintainer_email='xingjian.zhang@amicro.com.cn',
    description='Configuration data of the David robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
