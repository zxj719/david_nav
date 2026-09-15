# Protocol ownership — P6.1 / SPEC P1

PLAN §5.3 takes precedence over the earlier all-in-one SPEC package tree.
This package owns wire protocol and transport only. Domain values, skills and
the single task owner live in nav_tasks; their public surface is nav_tasks.api.

The codec is split out of the existing file without changing function bodies.
Command classes and CommandParseError keep their construction/exception behavior.
The old module re-exports them so existing isinstance checks remain correct.
POI data is only read to format an event; the codec performs no repository I/O.

The next steps still require TaskApi integration, registered command translation,
ReliableOutbox, runtime/CLI composition and separate robustness fixes. Until
those are verified, the legacy executor core/runtime is intentionally retained.
The existing G1–G16 files are not rewritten to accept a new wire contract.
