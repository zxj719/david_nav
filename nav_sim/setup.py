"""Install configurable, explicitly isolated navigation simulation tools."""

from setuptools import find_packages, setup

PACKAGE = 'nav_sim'

setup(
    name=PACKAGE,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
        ('share/' + PACKAGE + '/config', ['config/schema.yaml']),
    ],
    install_requires=['setuptools', 'websockets==16.1'],
    zip_safe=True,
    maintainer='zhangxingjian',
    maintainer_email='xingjian.zhang@amicro.com.cn',
    description='Offline chassis, localization and WebSocket test doubles.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'fake_chassis = nav_sim.fake_chassis:main',
        'fake_localization = nav_sim.fake_localization:main',
        'fake_agent_server = nav_sim.fake_agent_server:main',
    ]},
)
