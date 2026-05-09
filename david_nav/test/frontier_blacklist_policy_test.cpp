#include <gtest/gtest.h>

#include "david_nav/frontier_blacklist_policy.hpp"

TEST(FrontierBlacklistPolicy, RecoversWhenNoActiveFrontierExists)
{
  EXPECT_TRUE(david_nav::should_recover_blacklist(0, 1));
}

TEST(FrontierBlacklistPolicy, DoesNotRecoverWhenActiveFrontierExists)
{
  EXPECT_FALSE(david_nav::should_recover_blacklist(2, 1));
}

TEST(FrontierBlacklistPolicy, DoesNotRecoverWithoutBlacklistedEntries)
{
  EXPECT_FALSE(david_nav::should_recover_blacklist(0, 0));
}
