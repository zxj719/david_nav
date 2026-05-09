#include <gtest/gtest.h>
#include <vector>

#include "g1_nav/escape_obstacle_action.hpp"

TEST(EscapeObstacleAction, FrontOverlapCommandsNegativeX)
{
  const std::vector<g1_nav::EscapeOverlapPoint> overlaps{{0.18, 0.02}, {0.15, -0.03}};
  const auto command = g1_nav::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, -0.25);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
}

TEST(EscapeObstacleAction, RearOverlapCommandsPositiveX)
{
  const std::vector<g1_nav::EscapeOverlapPoint> overlaps{{-0.17, 0.01}, {-0.14, -0.02}};
  const auto command = g1_nav::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.25);
  EXPECT_DOUBLE_EQ(command.vy, 0.0);
}

TEST(EscapeObstacleAction, LeftOverlapCommandsNegativeY)
{
  const std::vector<g1_nav::EscapeOverlapPoint> overlaps{{0.02, 0.16}, {-0.01, 0.12}};
  const auto command = g1_nav::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, -0.20);
}

TEST(EscapeObstacleAction, RightOverlapCommandsPositiveY)
{
  const std::vector<g1_nav::EscapeOverlapPoint> overlaps{{0.03, -0.18}, {-0.02, -0.11}};
  const auto command = g1_nav::choose_escape_command(overlaps, 0.25, 0.20);

  EXPECT_TRUE(command.valid);
  EXPECT_DOUBLE_EQ(command.vx, 0.0);
  EXPECT_DOUBLE_EQ(command.vy, 0.20);
}

TEST(EscapeObstacleAction, ClearTrackerNeedsThreeConsecutiveClearObservations)
{
  g1_nav::ClearObservationTracker tracker(3);

  EXPECT_FALSE(tracker.update(true));
  EXPECT_FALSE(tracker.update(false));
  EXPECT_FALSE(tracker.update(false));
  EXPECT_TRUE(tracker.update(false));
}

TEST(EscapeObstacleAction, StopTwistZeroesAllComponents)
{
  const auto twist = g1_nav::make_stop_twist();

  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.z, 0.0);
}

TEST(EscapeObstacleAction, ObservationGenerationTrackerOnlyAdvancesOnNewMessages)
{
  g1_nav::ObservationGenerationTracker tracker;

  EXPECT_TRUE(tracker.update(1, 1));
  EXPECT_FALSE(tracker.update(1, 1));
  EXPECT_TRUE(tracker.update(2, 1));
  EXPECT_FALSE(tracker.update(2, 1));
  EXPECT_TRUE(tracker.update(2, 2));
}
