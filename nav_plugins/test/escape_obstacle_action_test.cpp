#include <gtest/gtest.h>
#include <vector>

#include "nav_plugins/escape_obstacle_action.hpp"

TEST(EscapeObstacleAction, FrontOverlapCommandsNegativeX)
{
  const std::vector<nav_plugins::EscapeOverlapPoint> overlaps{{0.18, 0.02}, {0.15, -0.03}};
  const auto command = nav_plugins::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, -0.25);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
}

TEST(EscapeObstacleAction, RearOverlapCommandsPositiveX)
{
  const std::vector<nav_plugins::EscapeOverlapPoint> overlaps{{-0.17, 0.01}, {-0.14, -0.02}};
  const auto command = nav_plugins::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.25);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
}

TEST(EscapeObstacleAction, LeftOverlapCommandsNegativeY)
{
  const std::vector<nav_plugins::EscapeOverlapPoint> overlaps{{0.02, 0.16}, {-0.01, 0.12}};
  const auto command = nav_plugins::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, -0.20);
}

TEST(EscapeObstacleAction, RightOverlapCommandsPositiveY)
{
  const std::vector<nav_plugins::EscapeOverlapPoint> overlaps{{0.03, -0.18}, {-0.02, -0.11}};
  const auto command = nav_plugins::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.20);
}

TEST(EscapeObstacleAction, ClearTrackerNeedsThreeConsecutiveClearObservations)
{
  nav_plugins::ClearObservationTracker tracker(3);

  EXPECT_FALSE(tracker.update(true));
  EXPECT_FALSE(tracker.update(false));
  EXPECT_FALSE(tracker.update(false));
  EXPECT_TRUE(tracker.update(false));
}

TEST(EscapeObstacleAction, StopTwistZeroesAllComponents)
{
  const auto twist = nav_plugins::make_stop_twist();

  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST(EscapeObstacleAction, ObservationGenerationTrackerOnlyAdvancesOnNewMessages)
{
  nav_plugins::ObservationGenerationTracker tracker;

  EXPECT_TRUE(tracker.update(1, 1));
  EXPECT_FALSE(tracker.update(1, 1));
  EXPECT_TRUE(tracker.update(2, 1));
  EXPECT_FALSE(tracker.update(2, 1));
  EXPECT_TRUE(tracker.update(2, 2));
}
