"""Run the dock_bench script-style self check under pytest.

``scripts/dock_bench/test_dock_common.py`` is a standalone script: it runs its
checks at module level, prints a summary and exits non-zero on failure. It has
no ``test_*`` functions, so registering it directly with pytest collects zero
tests and ctest reports a failure. This wrapper executes it as a subprocess
and asserts on its exit status instead.
"""

from pathlib import Path
import subprocess
import sys


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'dock_bench' / 'test_dock_common.py'


def test_dock_bench_common_self_check_passes():
    """The dock_bench log parser and metrics self check exits cleanly."""
    result = subprocess.run(
        [sys.executable, str(SCRIPT)],
        capture_output=True,
        check=False,
        cwd=str(SCRIPT.parent),
        text=True,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert '全部通过' in result.stdout
