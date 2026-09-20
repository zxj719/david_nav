"""只读的入口点校验，启动规划与 ROS launch 共用同一套判断.

只检查"装没装、是不是可执行文件"，不加载库、不启动进程、不初始化 ROS。
"""

import os
from pathlib import Path


def resolve_executable(package: str, executable: str) -> str:
    """解析已安装的可执行文件，不导入 launch 也不启动 ROS.

    结果取决于当前 ament 环境：换一个 shell 或换一份 overlay 结论可能不同。

    Args:
        package: 当前 ament 环境中的包名。
        executable: 该包 lib/包名 目录下的文件名。

    Returns:
        可执行普通文件的绝对路径。

    Raises:
        ValueError: 名字非法，或该可执行文件不可用。
        LookupError: 当前环境中没有这个包。
    """
    from ament_index_python.packages import get_package_prefix

    if not executable or Path(executable).name != executable or executable in ('.', '..'):
        raise ValueError('package executable must be a basename')
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ValueError(f'missing executable: {path}')
    return str(path)


def resolve_component(package: str, plugin: str) -> str:
    """在改动主机之前先核对 ament 组件注册与共享库.

    Args:
        package: 拥有该组件资源的 ROS 包。
        plugin: 注册时使用的精确 C++ 类名。

    Returns:
        共享库的绝对路径；过程中不加载库，也不创建任何 ROS 节点，
        因此"能找到"不等于"能成功加载"。

    Raises:
        ValueError: 注册缺失、歧义、不安全，或指向不存在的库。
        LookupError: 该组件资源没有安装。
    """
    from ament_index_python.resources import get_resource

    content, prefix = get_resource('rclcpp_components', package)
    matches = [line.split(';') for line in content.splitlines()
               if line.split(';')[0] == plugin]
    if len(matches) != 1 or len(matches[0]) != 2:
        raise ValueError(f'missing or ambiguous component registration: {package}/{plugin}')
    relative = Path(matches[0][1])
    if relative.is_absolute() or '..' in relative.parts or not relative.parts:
        raise ValueError('component library must be a package-relative path')
    path = Path(prefix) / relative
    if not path.is_file() or not os.access(path, os.R_OK):
        raise ValueError(f'missing component library: {path}')
    return str(path)
