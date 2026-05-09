#pragma once

namespace g1_nav
{

inline bool should_blacklist_on_navigation_abort(
  bool preempt_requested,
  bool has_pending_goal)
{
  return !(preempt_requested && has_pending_goal);
}

inline bool should_accept_new_frontier_goal_while_navigating(bool navigating)
{
  return !navigating;
}

inline bool should_retry_same_frontier_after_failure(
  bool has_current_frontier,
  bool retry_used,
  bool has_retry_goal)
{
  return has_current_frontier && !retry_used && has_retry_goal;
}

}  // namespace g1_nav
