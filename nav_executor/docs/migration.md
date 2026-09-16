# Protocol ownership — P6.1 / SPEC P1

PLAN §5.3 takes precedence over the earlier all-in-one SPEC package tree.
This package owns wire protocol and transport only. Domain values, skills and
the single task owner live in nav_tasks; their public surface is nav_tasks.api.

The initial codec relocation preserved function bodies. The separate R1 repair
now normalizes malformed fields while retaining valid-command values/wire bytes.
Command classes and the CommandParseError compatibility base remain available.
The old module re-exports them so existing isinstance checks remain correct.
POI data is only read to format an event; the codec performs no repository I/O.

TaskApi integration, registered translation, ReliableOutbox and the shared
runtime/CLI are connected. Retained navigation_executor.py and dock_navigator.py
now forward to this package. Old algorithm tests load immutable pre-shim fixtures;
separate shim tests exercise real paths/argv/exit status, and new-chain tests
retain the frozen wire contract. Historical supporting classes remain available
for imports pending P7 legacy isolation. R1/R2 are implemented; R3 is verified
against the synchronous feedback path introduced with the new backend (see
nav_tasks ADR-0009). R8 now defaults standalone Dock to /dock_goal; explicit ROS
overrides remain supported. The R6 passive measurement tool is available; actual
idle measurements and any architecture decision remain separate manual review.
