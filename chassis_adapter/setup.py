"""Install the adapter executable and its profile-key documentation."""

from setuptools import find_packages, setup

PACKAGE = 'chassis_adapter'

setup(
    name=PACKAGE, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
        ('share/' + PACKAGE + '/config', ['config/schema.yaml']),
        ('share/' + PACKAGE + '/launch', ['launch/chassis.launch.py']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zhangxingjian', maintainer_email='xingjian.zhang@amicro.com.cn',
    description='Differential command adapter with legacy-equivalent timeout behavior.',
    license='Apache-2.0', tests_require=['pytest'],
    entry_points={'console_scripts': ['chassis_adapter = chassis_adapter.node:main']},
)
