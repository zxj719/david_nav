# Probability-Map Frontier Semantics Design

## Context

`g1_nav` now consumes a SLAM occupancy grid with the following semantics:

- `-1`: unknown
- `0..100`: known occupancy probability
- `0`: pure free cell
- `100`: pure obstacle cell

The current frontier search implementation still assumes the older rule "`0` is free, `1..100` is occupied" in several places. That incorrectly shrinks the traversable set for frontier discovery when using a probability map and can suppress valid frontiers.

At the same time, the final navigation goal sent to Nav2 should remain conservative: it should stay on pure `0` cells so the existing planner behavior is not loosened.

## Goals

- Update frontier search to work correctly on probability maps.
- Preserve the definition of a frontier as a boundary between known free space and unknown space.
- Keep the final navigation goal on pure `0` cells.
- Avoid changing Nav2 planner configuration or goal semantics.

## Non-Goals

- No change to Nav2 planner parameters, costmap layers, or inflation settings.
- No clearance ring or additional safety-margin filter around goal cells.
- No parameterization of the occupancy threshold in this change; use the fixed empirical threshold `50`.

## Map Semantics

The codebase will use these shared occupancy categories:

- `unknown`: value `< 0`
- `search-free`: known cell with value `<= 50`
- `occupied`: known cell with value `> 50`
- `goal-free`: value `== 0`

`search-free` is used only for frontier discovery and frontier revalidation. `goal-free` is used only when choosing the final navigation goal that will be sent to Nav2.

## Design

### Shared Grid Helpers

`GridMapView` will expose helper predicates so occupancy semantics are defined once and reused consistently:

- `is_unknown(x, y)` / `is_unknown_value(value)`
- `is_search_free(x, y)` / `is_search_free_value(value)`
- `is_goal_free(x, y)` / `is_goal_free_value(value)`
- `is_obstacle(x, y)` / `is_obstacle_value(value)`

These helpers make the distinction between frontier-search traversability and final-goal admissibility explicit in code.

### Frontier Detection

A frontier cell remains:

- a `search-free` cell
- with at least one 8-connected neighboring `unknown` cell

This preserves the exploration boundary behavior while adapting free-space detection to probability-map values.

### BFS Expansion

Frontier search BFS will:

- start from the robot grid cell if it is `search-free`
- otherwise snap to the nearest `search-free` cell before BFS begins
- expand only through `search-free` cells
- never expand into `unknown` or `occupied` cells

This allows the robot to continue discovering frontiers in probability-mapped traversable space even when nearby free cells are represented by values such as `5`, `18`, or `42`.

### Frontier Revalidation

`revalidate_nearby_frontier()` will use the same frontier and `search-free` rules as the main search path so a locked frontier stays valid only if it is still anchored in admissible free space.

### Navigation Goal Selection

`find_navigation_goal()` will remain conservative:

- only cells with value `== 0` are eligible navigation goals
- no new clearance filter is added in this change

This keeps the final goal semantics unchanged for Nav2, minimizing planner-behavior risk while still broadening frontier discovery.

## Expected Behavior

- Frontiers can be discovered adjacent to unknown space even when the frontier cell has a probability value between `1` and `50`.
- The robot can recover from starting on a non-`0` but still traversable `search-free` cell by snapping BFS to the nearest `search-free` cell.
- A frontier may be discoverable in `search-free` terrain but still be discarded later if there is no pure `0` navigation goal near its centroid. That frontier will continue to be blacklisted by the existing logic.

## Error Handling And Risks

- If a frontier exists in `search-free` space but no nearby pure `0` goal exists, the current blacklist-and-replan flow remains the fallback behavior.
- Because the final goal still uses pure `0`, exploration may be intentionally more conservative than the expanded frontier-search space. This is acceptable and desired for this change.
- The split between `search-free` and `goal-free` must be reflected clearly in helper names to avoid future regressions.

## Testing Strategy

Add focused tests for the frontier-search semantics:

- a cell with value `<= 50` can serve as frontier/search-free space
- a cell with value `> 50` cannot
- a frontier still requires an adjacent `unknown` neighbor
- robot-start snapping uses nearest `search-free`, not only pure `0`

Add or refactor goal-selection tests to confirm:

- navigation goals still require pure `0`
- non-zero `search-free` cells are not accepted as final navigation goals

Run package tests and relevant build verification after implementation.
