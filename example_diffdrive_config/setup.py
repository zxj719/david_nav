"""Install only data for a fictional second differential-drive robot."""

from pathlib import Path

from setuptools import setup

PACKAGE = 'example_diffdrive_config'


def _data_files():
    destinations = {}
    for directory in ('robot', 'modes', 'urdf', 'sites'):
        for path in sorted(Path(directory).rglob('*')):
            if path.is_file():
                target = str(Path('share') / PACKAGE / path.parent)
                destinations.setdefault(target, []).append(str(path))
    return sorted(destinations.items())


setup(
    name=PACKAGE,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml', 'README.md']),
    ] + _data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhangxingjian',
    maintainer_email='xingjian.zhang@amicro.com.cn',
    description='Config-only portability acceptance for a fictional differential-drive robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
