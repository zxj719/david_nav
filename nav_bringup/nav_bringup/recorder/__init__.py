"""Run recorder: a self-describing record of every navigation run.

Modules (refactor plan §13.2):

* :mod:`~nav_bringup.recorder.run_dir`: run directory layout and ``latest`` link.
* :mod:`~nav_bringup.recorder.manifest`: git state, versions, environment and
  input file fingerprints of a run.
* :mod:`~nav_bringup.recorder.events`: structured ``events.jsonl`` timeline.
* :mod:`~nav_bringup.recorder.resources`: per-process CPU and memory samples.
* :mod:`~nav_bringup.recorder.summary`: human readable ``summary.md``.
* :mod:`~nav_bringup.recorder.retention`: bounded disk usage that never loses
  a run's manifest or summary.
"""
