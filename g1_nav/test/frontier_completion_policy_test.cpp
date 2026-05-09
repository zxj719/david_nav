#include <gtest/gtest.h>

#include <optional>
#include <utility>

#include "g1_nav/frontier_completion_policy.hpp"

TEST(FrontierCompletionPolicy, MatchesOnlySmallAnchorDriftForCompletedHistory)
{
  const std::pair<double, double> completed_anchor{1.0, 1.0};
  const std::pair<double, double> same_frontier_drifted_anchor{1.18, 1.0};
  const std::pair<double, double> nearby_new_frontier{1.35, 1.0};

  EXPECT_TRUE(g1_nav::is_same_frontier_anchor(
    completed_anchor, same_frontier_drifted_anchor, 0.25));
  EXPECT_FALSE(g1_nav::is_same_frontier_anchor(
    completed_anchor, nearby_new_frontier, 0.25));
}

TEST(FrontierCompletionPolicy, ProximityCompletionOnlyAppliesToCurrentFrontierIdentity)
{
  const std::optional<std::pair<double, double>> current_frontier{
    std::pair<double, double>{1.0, 1.0}};
  const std::pair<double, double> robot_xy{1.30, 1.0};
  const std::pair<double, double> nearby_new_frontier{1.30, 1.0};

  EXPECT_FALSE(g1_nav::should_complete_candidate_frontier_by_proximity(
    current_frontier, nearby_new_frontier, robot_xy, 1.0, 0.25));
}

TEST(FrontierCompletionPolicy, ProximityCompletionAllowsSmallAnchorDriftForCurrentFrontier)
{
  const std::optional<std::pair<double, double>> current_frontier{
    std::pair<double, double>{1.0, 1.0}};
  const std::pair<double, double> robot_xy{1.15, 1.0};
  const std::pair<double, double> slightly_shifted_current_frontier{1.15, 1.0};

  EXPECT_TRUE(g1_nav::should_complete_candidate_frontier_by_proximity(
    current_frontier,
    slightly_shifted_current_frontier,
    robot_xy,
    1.0,
    0.25));
}
