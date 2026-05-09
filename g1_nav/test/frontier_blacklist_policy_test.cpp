#include <gtest/gtest.h>

#include "g1_nav/frontier_blacklist_policy.hpp"

TEST(FrontierBlacklistPolicy, RecoversWhenNoActiveFrontierExists)
{
  EXPECT_TRUE(g1_nav::should_recover_blacklist(0, 1));
}

TEST(FrontierBlacklistPolicy, DoesNotRecoverWhenActiveFrontierExists)
{
  EXPECT_FALSE(g1_nav::should_recover_blacklist(2, 1));
}

TEST(FrontierBlacklistPolicy, DoesNotRecoverWithoutBlacklistedEntries)
{
  EXPECT_FALSE(g1_nav::should_recover_blacklist(0, 0));
}
