# Protocol ownership — P6.1 / SPEC P1

PLAN §5.3 takes precedence over the earlier all-in-one SPEC package tree.
This package owns wire protocol and transport only. Domain values, skills and
the single task owner live in nav_tasks; their public surface is nav_tasks.api.

The codec is split out of the existing file without changing function bodies.
Command classes and CommandParseError keep their construction/exception behavior.
The old module re-exports them so existing isinstance checks remain correct.
POI data is only read to format an event; the codec performs no repository I/O.

TaskApi integration, registered translation, ReliableOutbox and the shared
runtime/CLI are connected. Retained navigation_executor.py and dock_navigator.py
now forward to this package. Old algorithm tests load immutable pre-shim fixtures;
separate shim tests exercise real paths/argv/exit status, and new-chain tests
retain the frozen wire contract. Historical supporting classes remain available
for imports pending P7 legacy isolation. Robustness repairs remain separate work.
