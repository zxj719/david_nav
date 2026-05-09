# Frontier Debug Visualization Design

## Context

`frontier_explorer` currently publishes only snapped navigation-goal markers to RViz.
That is useful for runtime goal inspection, but it is not enough to tune
`search_free_threshold`, because the user cannot see which raw frontier cells were
actually detected by `FrontierSearch`.

## Goal

Publish raw frontier-search output to RViz so the user can visually inspect:

- every frontier cell returned by `FrontierSearch`
- every frontier cluster centroid
- the existing snapped navigation goals

All three views must come from the same frontier-search pipeline used by runtime
selection, so the visualization reflects the real exploration decision inputs.

## Non-Goals

- No separate debug-only frontier search pass
- No change to blacklist, NMS, or goal-selection semantics
- No new RViz panel or custom message type

## Design

### Shared Frontier Data

Extend `g1_nav::Frontier` so each frontier carries the member grid cells that
formed the cluster. The search algorithm already visits these cells while building
the frontier, so this is the natural place to preserve them.

Store member cells in grid coordinates, not world coordinates. This keeps the
search result lightweight and allows the latest map geometry to convert cells to
RViz points when publishing.

### Marker Layers

Keep the existing marker publisher and publish three namespaces:

- `frontier_cells`: one `POINTS` marker containing every raw frontier cell
- `frontier_centroids`: one `SPHERE_LIST` marker containing every raw frontier centroid
- `frontier_goals`: the existing snapped navigation-goal markers

`frontier_cells` and `frontier_centroids` must be built from `observed_frontiers`
before blacklist filtering, completion filtering, NMS, and goal snapping.

`frontier_goals` must keep using the filtered `targets` pipeline so the operator can
still see which snapped goal the node is actually considering.

### Runtime Semantics

- If raw frontiers exist but no admissible goals exist, RViz should still show the
  raw frontier cells and centroids.
- If no raw frontiers exist, all frontier markers should be cleared.
- The existing `visualize` parameter continues to gate all frontier markers.

## Testing Strategy

Add TDD coverage at the frontier-search layer so we can prove that the returned
frontier cluster now preserves its member cells. Runtime marker publication will be
verified through build/test plus manual RViz use, because the node currently does
not expose a pure marker-building helper.
