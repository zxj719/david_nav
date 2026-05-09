#include <gtest/gtest.h>

#include "g1_nav/frontier_navigation_result_policy.hpp"

TEST(FrontierNavigationResultPolicy, DoesNotBlacklistAbortDuringGoalHandoff)
{
  EXPECT_FALSE(g1_nav::should_blacklist_on_navigation_abort(true, true));
}

TEST(FrontierNavigationResultPolicy, BlacklistsAbortWhenNoHandoffPending)
{
  EXPECT_TRUE(g1_nav::should_blacklist_on_navigation_abort(false, false));
  EXPECT_TRUE(g1_nav::should_blacklist_on_navigation_abort(true, false));
}

TEST(FrontierNavigationResultPolicy, AcceptsNewFrontierGoalWhileNavigating)
{
  EXPECT_FALSE(g1_nav::should_accept_new_frontier_goal_while_navigating(true));
  EXPECT_TRUE(g1_nav::should_accept_new_frontier_goal_while_navigating(false));
}

TEST(FrontierNavigationResultPolicy, RetriesSameFrontierOnlyOnceWhenRetryGoalExists)
{
  EXPECT_TRUE(g1_nav::should_retry_same_frontier_after_failure(true, false, true));
  EXPECT_FALSE(g1_nav::should_retry_same_frontier_after_failure(true, true, true));
  EXPECT_FALSE(g1_nav::should_retry_same_frontier_after_failure(false, false, true));
  EXPECT_FALSE(g1_nav::should_retry_same_frontier_after_failure(true, false, false));
}
