"""Pin pre-shim reference bytes independently of moving runtime implementations."""

import ast
import hashlib
from pathlib import Path
import runpy

import pytest

ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT / 'david_nav/test/golden/legacy_entries'


@pytest.mark.parametrize('name,digest', [
    ('navigation_executor', '5f7bdf437c06ba886d4d23812c0767a336de2535a97503b343b75b2800d2da53'),
    ('dock_navigator', 'cf5883eb3e3b468a15bedb90c70d12f31c2f56f4aad5d78f760b39f22a8314c6'),
])
def test_reference_bytes_are_pinned_to_pre_shim_commit(name, digest):
    """Later entry changes cannot silently rewrite the oracle into the tested implementation."""
    path = FIXTURES / f'{name}.py.txt'
    data = path.read_bytes()
    assert hashlib.sha256(data).hexdigest() == digest
    ast.parse(data, filename=str(path))
    assert path.suffix == '.txt'


def test_config_golden_loads_snapshot_even_when_runtime_source_is_unavailable(monkeypatch):
    """A shim may remove old helpers without weakening the CLI behavior comparison."""
    from test_config import test_default_configuration_and_help_match_old_executor

    original = runpy.run_path

    def checked(path, *args, **kwargs):
        assert Path(path) == FIXTURES / 'navigation_executor.py.txt'
        return original(path, *args, **kwargs)

    monkeypatch.setattr(runpy, 'run_path', checked)
    test_default_configuration_and_help_match_old_executor(monkeypatch)
