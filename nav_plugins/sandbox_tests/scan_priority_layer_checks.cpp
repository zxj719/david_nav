// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace
{

class ScanPriorityLayerChecks : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    if (!domain || std::string(domain) != "99") {
      throw std::runtime_error("This check requires the guarded domain 99 launcher");
    }
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(ScanPriorityLayerChecks, DefaultFollowsTheCostmapFrame)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("costmap_frame_check");
  node->declare_parameter("robot_base_frame", "configured_rotation_center");
  nav2_costmap_2d::LayeredCostmap parent("test_world", false, false);
  parent.resizeMap(10, 10, 0.1, 0.0, 0.0);
  tf2_ros::Buffer buffer(node->get_clock());
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("nav_plugins::ScanPriorityLayer");
  layer->initialize(&parent, "scan_layer", &buffer, node, nullptr);
  ASSERT_TRUE(node->has_parameter("scan_layer.robot_base_frame"));
  EXPECT_EQ(
    node->get_parameter("scan_layer.robot_base_frame").as_string(),
    "configured_rotation_center");
}

TEST_F(ScanPriorityLayerChecks, CompatibilityAliasAcceptsAnOverride)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("costmap_override_check");
  node->declare_parameter("robot_base_frame", "parent_frame");
  node->declare_parameter("scan_layer.robot_base_frame", "explicit_rotation_center");
  nav2_costmap_2d::LayeredCostmap parent("test_world", false, false);
  parent.resizeMap(10, 10, 0.1, 0.0, 0.0);
  tf2_ros::Buffer buffer(node->get_clock());
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("david_nav::ScanPriorityLayer");
  layer->initialize(&parent, "scan_layer", &buffer, node, nullptr);
  EXPECT_EQ(
    node->get_parameter("scan_layer.robot_base_frame").as_string(),
    "explicit_rotation_center");
}

TEST_F(ScanPriorityLayerChecks, EmptyOverrideFailsClosed)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("costmap_invalid_frame_check");
  node->declare_parameter("robot_base_frame", "parent_frame");
  node->declare_parameter("scan_layer.robot_base_frame", "");
  nav2_costmap_2d::LayeredCostmap parent("test_world", false, false);
  parent.resizeMap(10, 10, 0.1, 0.0, 0.0);
  tf2_ros::Buffer buffer(node->get_clock());
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader(
    "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  auto layer = loader.createSharedInstance("nav_plugins::ScanPriorityLayer");
  EXPECT_THROW(
    layer->initialize(&parent, "scan_layer", &buffer, node, nullptr), std::invalid_argument);
}

}  // namespace
