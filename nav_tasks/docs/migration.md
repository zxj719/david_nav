# Pure contract migration — P6.1 / SPEC P1

PLAN §5.3–§5.4 refines the older single-package SPEC layout: domain, task core
and docking skills belong in `nav_tasks`; wire protocol and transport belong in
`nav_executor`. `nav_tasks` never imports `nav_executor` or WebSockets. Ingress
adapters consume public values from `nav_tasks.api`; future task operations go
only through TaskApi, never by importing core implementation classes.

| Old module | Owning implementation |
|---|---|
| david_nav.navigation_types | nav_tasks.domain.types |
| david_nav.grid_map_view | nav_tasks.domain.grid |
| david_nav.dock_settle | nav_tasks.skills.docking.settle |
| david_nav.dock_finish | nav_tasks.skills.docking.finish |
| david_nav.navigation_protocol | nav_executor.protocol.messages + codec |

Documentation/annotations were prepared in a separate commit. The relocation
keeps function bodies, values, quoting, wire key order and algorithms unchanged;
only imports and compatibility exports change. No decoder robustness repair,
snap-policy change, dock-log change or task-state repair is bundled here.

Now added separately: domain geometry/errors, task result/request values, ports,
state/ownership, registry, retry, supervisor and public TaskApi. See
[architecture.md](architecture.md) for the operational contracts. Still required
for P6.1: handlers, snap extraction and replacement-core golden integration.
P6.2–P6.4 then connect the shared Nav2 backend, Dock and WS ingress.
The old executor core stays active until the replacement passes its golden
contracts. Do not treat this package installation as deployment of that manager.

Imports must remain side-effect free. Pure layers use standard-library imports;
ROS/storage adapters will remain outside them. AST dependency and docstring
tests are mandatory without installing import-linter. All moved pure tests may
run on a PC without ROS. Hardware acceptance and production entry changes remain
manual; alias 3 and the localization implementation are outside this work.
