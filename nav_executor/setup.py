"""Install nav_executor pure contracts and package-owned documentation."""

from glob import glob

from setuptools import find_packages, setup

PACKAGE = 'nav_executor'

setup(
    name=PACKAGE, version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md', 'CHANGELOG.rst']),
        ('share/' + PACKAGE + '/config', ['config/schema.yaml']),
        ('share/' + PACKAGE + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zhangxingjian', maintainer_email='xingjian.zhang@amicro.com.cn',
    description='nav_executor pure navigation contracts.',
    license='Apache-2.0', tests_require=['pytest'],
)
