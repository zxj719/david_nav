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
