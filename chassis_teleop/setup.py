"""Install keyboard state and terminal components; graph-facing runtime is pending."""

from setuptools import find_packages, setup

PACKAGE = 'chassis_teleop'

setup(
    name=PACKAGE, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
        ('share/' + PACKAGE + '/config', ['config/schema.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zhangxingjian', maintainer_email='xingjian.zhang@amicro.com.cn',
    description='Keyboard motion state and terminal input without ROS or hardware ownership.',
    license='Apache-2.0', tests_require=['pytest'],
)
