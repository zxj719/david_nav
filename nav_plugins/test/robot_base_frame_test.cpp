// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <stdexcept>

#include "nav_plugins/robot_base_frame.hpp"

TEST(RobotBaseFrame, DefaultsToTheParentCostmapFrame)
{
  EXPECT_EQ(nav_plugins::resolveRobotBaseFrame("rotation_center"), "rotation_center");
  EXPECT_EQ(nav_plugins::resolveRobotBaseFrame("other_robot"), "other_robot");
}

TEST(RobotBaseFrame, ExplicitLayerOverrideWins)
{
  EXPECT_EQ(nav_plugins::resolveRobotBaseFrame("parent", "override"), "override");
  EXPECT_EQ(nav_plugins::resolveRobotBaseFrame("", "override"), "override");
}

TEST(RobotBaseFrame, EmptyEffectiveFrameIsRejected)
{
  EXPECT_THROW(nav_plugins::resolveRobotBaseFrame(""), std::invalid_argument);
  EXPECT_THROW(nav_plugins::resolveRobotBaseFrame("parent", ""), std::invalid_argument);
}
