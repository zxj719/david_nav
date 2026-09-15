"""Read-only entry-point validation shared by startup planning and ROS launch."""

import os
from pathlib import Path


def resolve_executable(package: str, executable: str) -> str:
    """Resolve an installed executable without importing launch or starting ROS.

    Args:
        package: Package name in the current ament environment.
        executable: Basename under its lib/package directory.

    Returns:
        Absolute path to an executable regular file.

    Raises:
        ValueError: Invalid name or unavailable executable.
        LookupError: The package is absent from the current environment.
    """
    from ament_index_python.packages import get_package_prefix

    if not executable or Path(executable).name != executable or executable in ('.', '..'):
        raise ValueError('package executable must be a basename')
    path = Path(get_package_prefix(package)) / 'lib' / package / executable
    if not path.is_file() or not os.access(path, os.X_OK):
        raise ValueError(f'missing executable: {path}')
    return str(path)


def resolve_component(package: str, plugin: str) -> str:
    """Check the ament component registration and shared library before host changes.

    Args:
        package: ROS package owning the component resource.
        plugin: Exact registered C++ class name.

    Returns:
        Absolute library path; no library is loaded and no ROS node is created.

    Raises:
        ValueError: Registration is absent, ambiguous, unsafe or points at a missing library.
        LookupError: The component resource is not installed.
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
