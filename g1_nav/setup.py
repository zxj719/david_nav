from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'g1_nav'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 基础元数据文件
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 安装 launch 文件到 share/g1_nav/launch
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*.launch.py'))),

        # 安装 RViz 配置到 share/g1_nav/rviz
        (os.path.join('share', package_name, 'rviz'), 
         glob(os.path.join('rviz', '*.rviz'))),

        # 安装参数文件到 share/g1_nav/config
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),

        # 安装地图文件到 share/g1_nav/maps（包含 YAML 和 PGM）
        (os.path.join('share', package_name, 'maps'), 
         glob(os.path.join('maps', '*.yaml') ) +  # 修改为 maps 目录
         glob(os.path.join('maps', '*.pgm')) + 
         glob(os.path.join('maps', '*.png'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FLYivan',
    maintainer_email='luoyifan902008@126.com',
    description='Navigation package for Unitree G1 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
)
