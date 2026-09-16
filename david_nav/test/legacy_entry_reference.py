"""Load immutable pre-shim entry fixtures, never the live compatibility executables."""

from importlib.machinery import SourceFileLoader
import importlib.util
from pathlib import Path


def load_entry(name):
    """Return a historical module for legacy behavior assertions without running main."""
    path = Path(__file__).parent / 'golden' / 'legacy_entries' / (name + '.py.txt')
    loader = SourceFileLoader('legacy_reference_' + name, str(path))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module
