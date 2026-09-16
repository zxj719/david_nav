# Headless shell cleanup tests

`test_headless_navigation_script.py` executes the actual, unchanged headless
shell, not a copied implementation of its cleanup functions. These are pure
host-process tests: they never initialize ROS or open robot hardware.

- Current `DAVID_*` injection paths select private empty setup files, fake
  ROS/Python/executor executables, a private POI file and output directory.
  No test rewrites `HOME`. The fake sourced ROS setup redirects the legacy
  executor lock to a private path before its real cleanup function is called.
- `sudo`, `nvpmodel`, `jetson_clocks` and `ifconfig` are inert replacements.
  Host preflight cleanup and shutdown velocity publication are disabled.
  Live test execution also selects `--no-chassis --no-bag`.
- The Python replacement delegates only the repository's pure configuration
  reader. The readiness probe and executor are in-memory waiting processes.
  Unknown ROS commands, chassis launch, arbitrary Python, broad `pgrep -f`
  and non-99 domains fail closed. No command falls back to the host ROS CLI.
- The scoped pgrep replacement supports only `-P` and `-g` for descendants of
  the exact test shell. A fake Nav2 parent waits for a child in a **separate
  session**, so signalling just the Nav2 process group cannot pass the test.
- Ctrl+C is sent to the owned foreground process group, as by a terminal,
  both while the probe waits and while the foreground executor runs. The
  original single-shell-PID injection is also retained during startup, after
  confirming that the script registered its managed child. The
  original exit-130, `Received INT`, and detached-PID-disappeared assertions
  remain; additionally the parent must be reaped and scoped descendant/group
  queries must have occurred. Success is checked before fallback cleanup.
- Failure cleanup uses pidfds for recorded private children to avoid signalling
  reused PIDs. The fake parent reaps its detached child. No hardware shutdown,
  real ROS process handling or signal sent to only a foreground-blocked shell PID is
  claimed by these tests.

Run through the normal `david_nav` colcon tests. The former missing-harness
skip is removed; this does not resolve the independent `david_cmd` copyright
skip or constitute physical navigation acceptance.
