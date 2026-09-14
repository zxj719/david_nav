# nav_bringup

Bringup, run recording and operations tooling for robot navigation stacks.
This package is robot independent: robot-specific values live in a
`<robot>_config` package.

## Status

| Component | State | Plan reference |
|---|---|---|
| `nav_bringup.lint.docstrings` | available | refactor plan §5.4, RUNBOOK §1 |
| `nav_bringup.recorder`: run directory, manifest, `code.diff`, events, resource samples, summary, retention | available (PROGRESS P1.2a) | plan §13.2 |
| Recorder ROS side: effective parameter dump, topic rates and TF lag | planned (P1.2b) | plan §13.2 |
| `navrun wrap -- <command>`: record a run around an existing launch command | available (P1.3) | plan §13.1, §16.2 Phase 1 |
| `navrun doctor`: read-only checks (ROS/DDS environment, orphan Fast DDS segments, map yaml mode, duplicate robot_state_publisher, stale copied scripts, chassis serial device) | available (P1.5a) | plan §13.3 |
| `navrun doctor` live checks (ros2 daemon, duplicate publishers, map→odom TF) | planned (P1.5b) | plan §13.3 |
| `navrun up` | planned (P4.2) | plan §13.1 |
| `navcfg render` / `validate` / `doc` | planned (P2.3 – P2.6) | plan §12 |
| `nav.launch.py` | planned (P4.1) | plan §7.1 |

## Layout

```
nav_bringup/
├── nav_bringup/
│   ├── __init__.py
│   └── lint/
│       └── docstrings.py   # Google-style Args/Returns/Yields/Raises checks
├── test/
│   ├── test_docstring_policy.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml
├── setup.cfg
└── setup.py
```

## Code style

New navigation packages follow the Google Python Style Guide:

* `ament_flake8` with the ROS default configuration (99 columns, Google import order).
* `ament_pep257 --convention google` (summary on the first docstring line).
* `nav_bringup.lint.docstrings.check_paths` in a unit test, for the sections that
  pep257 does not check.

To apply the docstring policy to another package, add a test:

```python
from pathlib import Path

from nav_bringup.lint import docstrings


def test_docstring_policy():
    """Public API documents its arguments, returns and raised exceptions."""
    package_dir = Path(__file__).resolve().parents[1] / 'my_package'
    violations = docstrings.check_paths([package_dir])
    assert not violations, docstrings.format_violations(violations)
```

## Testing

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws_refactor
colcon build --symlink-install --metas src/colcon.meta --packages-select nav_bringup
ROS_DOMAIN_ID=99 colcon test --packages-select nav_bringup --event-handlers console_direct+
colcon test-result --verbose
```
