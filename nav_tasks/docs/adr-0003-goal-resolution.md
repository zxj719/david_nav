# ADR-0003: keep snap policy, remove unused static/dynamic distinction

The extracted resolver is a pure search over GridMapView snapshots. ROS
subscription ownership belongs in a later adapter, not in the search algorithm.
Ring radii, tolerance, candidate order, zero-cost checks, clearance and fallback
behavior are unchanged and compared with the legacy implementation in tests.

After arrival at a snap point, the current policy accepts success whenever the
anchor remains inadmissible, whether the obstacle is static or dynamic. It only
starts the anchor segment if a fresh admissibility check passes. Therefore the
old `is_anchor_static_blocked` helper is not part of the new GoalResolver port or
implementation. It has no caller controlling this policy. The old class remains
untouched until runtime integration; this ADR does not claim it has been deleted
from the legacy source already.

The Dock skill is separate: dock POIs must not use point snapping, and the static
map footprint validation for their preparation pose remains mandatory.
