# Frontier Completion Regression Report

## Symptom

After the robot reached one frontier, nearby new frontiers sometimes did not get
selected. The planner instead jumped back to an older, farther frontier.

## Root Cause

`frontier_explorer` treated any candidate frontier within the robot's
`frontier_reached_radius()` as already completed during target selection.
Later, completed-history filtering also reused that same broad radius.

With the current configuration, that made completion suppression much wider than
"the same frontier anchor". As a result, legitimately new nearby frontiers
could be filtered out as if they had already been explored.

## Fix

The completion semantics were narrowed in two places:

1. Proximity-based auto-completion now applies only to the current tracked
   frontier identity.
2. Completed-history matching now uses a small anchor-drift tolerance instead
   of the full reached radius.

This keeps minor centroid drift stable for the same frontier while allowing new
nearby frontier anchors to survive candidate filtering.

## Tests

Added `frontier_completion_policy_test` to lock in:

- small anchor drift still matches the same completed frontier
- a nearby but different frontier is not auto-completed
- the current frontier can still complete by proximity

## Verification

Verified with:

```bash
colcon test --packages-select g1_nav --event-handlers console_direct+ --ctest-args -R 'frontier_completion_policy_test|frontier_goal_selector_test|frontier_navigation_result_policy_test'
```

Result: 3/3 targeted gtest targets passed.
