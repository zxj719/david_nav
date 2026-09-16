"""Keep legacy import paths as identity-preserving pure Dock reexports."""

from nav_tasks.app import dock_config, dock_parameters
from nav_tasks.skills.docking import config, parameters


def test_configuration_compatibility_exports_are_identical():
    """No wrapper changes validation, defaults, errors or declaration ordering."""
    assert dock_config.workflow_config is config.workflow_config
    assert dock_parameters.from_parameters is parameters.from_parameters
    assert dock_parameters.PARAMETER_NAMES is parameters.PARAMETER_NAMES
