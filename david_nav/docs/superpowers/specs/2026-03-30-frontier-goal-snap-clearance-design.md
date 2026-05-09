# Frontier Goal Snap And Clearance Design

## Context

`frontier_explorer` currently discovers frontiers from `/lightning/grid_map`, then snaps each frontier centroid onto a nearby pure-free cell before sending `NavigateToPose`.

That behavior now has three problems for the current exploration workflow:

- final goal selection only checks `grid_map`, not the latest `/global_costmap/costmap`
- the snap result overwrites the frontier identity, so the same physical frontier region can be regenerated and snapped again
- the current search is based on a broad local radius, while the desired behavior is a fixed-radius ring search tied to frontier suppression

In addition, the navigation goal should stay farther away from inflated obstacles than the costmap alone currently guarantees, and Nav2 should treat the robot as a circular platform with radius `0.35 m`.

## Goals

- Only send a new frontier navigation goal after the node has received the latest cached `/global_costmap/costmap`.
- Require every frontier goal candidate to satisfy `grid_map == 0` and `global_costmap/costmap == 0`.
- Keep the final navigation goal at least one robot radius farther from any non-zero global-costmap cell.
- Replace the current goal snap search radius mechanism with a fixed snap ring.
- Use the same snap radius as the non-maximum-suppression radius for frontier anchors.
- Preserve frontier identity separately from the snapped navigation goal.
- Change Nav2 footprint semantics to a circular robot with radius `0.35 m`.

## Non-Goals

- No change to the frontier-discovery definition itself; frontier search still comes from `grid_map`.
- No change to planner plugin selection or behavior-tree structure.
- No attempt to synchronize `frontier_explorer` parameters automatically from Nav2 costmap parameters.
- No change to local or global inflation parameters in this change.

## Agreed Runtime Semantics

### Frontier Identity Versus Navigation Goal

Each selected frontier will carry two positions:

- `frontier_anchor`: the original frontier centroid returned by frontier search
- `nav_goal`: the actual pose sent to Nav2

`frontier_anchor` is the identity used for:

- blacklist checks
- completion checks
- preemption comparisons
- frontier suppression
- continuity with `last_frontier_target_`

`nav_goal` is used only for navigation dispatch and progress tracking.

This prevents a frontier from being rediscovered at nearly the same anchor location and then re-snapped as if it were a different target.

### Global Costmap Gating

`frontier_explorer` will subscribe to `/global_costmap/costmap` through a new configurable parameter:

- `global_costmap_topic: /global_costmap/costmap`

Every time the node is about to send a new frontier goal, it must re-evaluate the chosen candidate against the latest cached global costmap message currently held by the node.

If no global costmap has been received yet, the node waits and does not send a new frontier goal.

The node does not require that the costmap message be newer than the previous planning tick; it only requires that each new goal decision be made from the most recently received cached message.

### Goal Admissibility

A candidate world point is admissible only if all of the following are true:

1. The point maps inside `/lightning/grid_map` and that cell value is exactly `0`.
2. The point maps inside `/global_costmap/costmap` and that cell value is exactly `0`.
3. Every global-costmap cell whose center lies within `goal_clearance_radius` of the candidate also has value `0`.

`goal_clearance_radius` will be configured in `frontier_explorer` and will default to `0.35`.

This third rule makes the goal stand off from the inflated region by one robot radius beyond the candidate cell itself. Unknown cells and any non-zero costmap cells fail the clearance check.

### Snap Radius And Ring Search

The existing `direct_frontier_goal_search_radius` mechanism is removed.

It is replaced with:

- `frontier_snap_radius: 1.0`

This parameter is the only snap-distance limit.

For each frontier anchor:

1. If the anchor itself is goal-admissible, use it directly as `nav_goal`.
2. Otherwise, search only on the discrete ring centered at the anchor with radius `frontier_snap_radius`.
3. If one or more admissible ring candidates exist, choose the candidate with the smallest distance to the current robot pose.
4. If the ring contains no admissible candidate, blacklist the `frontier_anchor` immediately.

There is no expanding search radius and no fallback to the whole map.

### Discrete Ring Definition

The ring search operates in world coordinates using the global costmap grid as the candidate generator.

A global-costmap cell center belongs to the ring if its distance to the `frontier_anchor` lies within:

- `[frontier_snap_radius - 0.5 * global_costmap_resolution, frontier_snap_radius + 0.5 * global_costmap_resolution]`

Each ring candidate is then projected back into both maps for admissibility checks.

Using global-costmap cell centers as candidates keeps the ring aligned with the map that determines collision semantics and clearance.

### Frontier Non-Maximum Suppression

Frontier suppression is added before goal snapping.

The suppression radius is exactly the same `frontier_snap_radius` used by the ring search.

Process:

1. Score and sort observed frontiers using the existing frontier ranking logic.
2. Keep the highest-ranked frontier anchor.
3. Remove every lower-ranked frontier whose anchor lies within `frontier_snap_radius` of any already kept anchor.
4. Continue until all frontier anchors are either kept or suppressed.

This guarantees that if one frontier anchor already occupies a snap neighborhood, another frontier anchor cannot survive in the same neighborhood and trigger a second snap around nearly the same place.

## Data And State Changes

`FrontierExplorerNode` will retain separate state for:

- current frontier anchor
- current navigation goal
- last frontier anchor

Existing blacklist and completion storage will continue to store world coordinates, but those coordinates will now always refer to frontier anchors, never snapped navigation goals.

Marker publication may continue showing the chosen navigation targets, but the selection pipeline must preserve anchor identity internally even when visualization stays goal-centric.

## Planning Flow

Each planning tick will follow this sequence:

1. Wait for live-map readiness when enabled.
2. Wait for `/lightning/grid_map`.
3. Wait for `/global_costmap/costmap`.
4. Wait for robot pose.
5. Discover frontiers from `grid_map`.
6. Filter frontiers by blacklist and completion using frontier anchors.
7. Apply frontier non-maximum suppression using `frontier_snap_radius`.
8. Rank the remaining frontier anchors.
9. For each candidate anchor in rank order:
   - attempt direct admissibility at the anchor
   - otherwise attempt ring snapping
   - if a `nav_goal` is found, compare against current navigation state and dispatch or preempt as needed
   - if no `nav_goal` is found, blacklist that frontier anchor and continue

The node still sends at most one goal per planning cycle.

## Configuration Changes

### `frontier_explorer_params.yaml`

Add:

- `global_costmap_topic: /global_costmap/costmap`
- `frontier_snap_radius: 1.0`
- `goal_clearance_radius: 0.35`

Remove:

- `direct_frontier_goal_search_radius`

`frontier_snap_radius` must serve both:

- the snap ring radius
- the frontier non-maximum-suppression radius

### `nav2_params_2d.yaml`

For both `local_costmap` and `global_costmap`:

- remove the polygon `footprint`
- set `robot_radius: 0.35`
- set `footprint_padding: 0.0`

This makes Nav2 footprint semantics explicitly circular and aligns the configured robot size with `goal_clearance_radius`.

## Error Handling

- If `global_costmap` has not been received yet, planning waits rather than falling back to `grid_map`-only decisions.
- If a frontier anchor and its snap ring both fail admissibility, blacklist the anchor immediately.
- If all remaining frontiers are suppressed, blacklisted, completed, or non-admissible, the node reports that no reachable frontier is available.

## Risks And Follow-Up Notes

- Changing Nav2 to `robot_radius: 0.35` while leaving the current `inflation_radius: 0.25` untouched may produce warnings or planner behavior that remains tighter than ideal. This change intentionally leaves inflation tuning out of scope, but that interaction should be monitored during bring-up.
- Because ring candidates are generated from global-costmap cell centers, small map-resolution differences between `grid_map` and `global_costmap` may cause edge candidates to pass one map and fail the other. That is acceptable and desired because the final goal must satisfy both maps at the same world position.
- The new suppression step intentionally reduces duplicate frontiers, so the visible frontier count may drop even when the explored boundary has not changed.

## Testing Strategy

Add focused tests for the new goal-selection logic:

- planning waits for global costmap before sending a new frontier goal
- a frontier anchor that is already admissible is used directly as the navigation goal
- a non-admissible frontier anchor can snap only to a candidate on the `frontier_snap_radius` ring
- ring candidates must satisfy `grid_map == 0`
- ring candidates must satisfy `global_costmap == 0`
- ring candidates must satisfy the `goal_clearance_radius` disk check
- if the ring has no admissible candidate, the frontier anchor is blacklisted
- blacklist and completion operate on frontier anchors, not snapped goals
- frontier non-maximum suppression keeps only the highest-ranked anchor within one snap radius neighborhood

Also update integration-style configuration tests to verify:

- `direct_frontier_goal_search_radius` is gone from `frontier_explorer_params.yaml`
- `frontier_snap_radius` and `goal_clearance_radius` exist
- both Nav2 costmaps use `robot_radius: 0.35`
- both Nav2 costmaps no longer declare the previous polygon footprint
