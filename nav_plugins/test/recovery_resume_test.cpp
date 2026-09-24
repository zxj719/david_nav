// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"

namespace
{

struct Scenario
{
  bool embedded{false};
  bool escape_complete{false};
  bool tracking_complete{false};
  std::map<std::string, int> starts;
  std::map<std::string, int> halts;
};

// Only action/condition leaves are faked. Nav2's actual RecoveryNode,
// PipelineSequence, RateController and RoundRobin control the execution.
class ScriptedLeaf : public BT::StatefulActionNode
{
public:
  ScriptedLeaf(
    const std::string & name, const BT::NodeConfiguration & config,
    std::string kind, std::shared_ptr<Scenario> scenario)
  : BT::StatefulActionNode(name, config), kind_(std::move(kind)), scenario_(std::move(scenario)) {}

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports;
    for (const auto & name : {
        "confirmations_required", "costmap_topic", "footprint_topic",
        "lethal_cost_threshold", "transform_tolerance", "goal", "goals", "path",
        "planner_id", "controller_id", "service_name", "clear_confirmations",
        "cmd_vel_topic", "robot_base_frame", "longitudinal_speed", "lateral_speed",
        "wait_duration", "backup_dist", "backup_speed", "time_allowance", "spin_dist",
        "input_goals", "output_goals", "radius", "global_frame"})
    {
      ports.insert(BT::InputPort<std::string>(name));
    }
    return ports;
  }

  BT::NodeStatus onStart() override
  {
    ++scenario_->starts[kind_];
    return nextStatus();
  }

  BT::NodeStatus onRunning() override
  {
    return nextStatus();
  }

  void onHalted() override
  {
    ++scenario_->halts[kind_];
  }

private:
  BT::NodeStatus nextStatus()
  {
    if (kind_ == "RobotEmbeddedInObstacle") {
      return scenario_->embedded ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    if (kind_ == "GoalUpdated" || kind_ == "IsStuck") {
      return BT::NodeStatus::FAILURE;
    }
    if (kind_ == "FollowPath") {
      return scenario_->tracking_complete ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
    }
    if (kind_ == "EscapeObstacle") {
      if (!scenario_->escape_complete) {
        return BT::NodeStatus::RUNNING;
      }
      scenario_->embedded = false;
    }
    return BT::NodeStatus::SUCCESS;
  }

  std::string kind_;
  std::shared_ptr<Scenario> scenario_;
};

class RecoveryResume : public ::testing::TestWithParam<std::string> {};

TEST_P(RecoveryResume, EscapeSuccessRequiresASecondPlanningAndTrackingGoal)
{
  auto scenario = std::make_shared<Scenario>();
  BT::BehaviorTreeFactory factory;
  for (const auto & library : {
      "nav2_recovery_node_bt_node", "nav2_pipeline_sequence_bt_node",
      "nav2_round_robin_node_bt_node", "nav2_rate_controller_bt_node"})
  {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(library));
  }
  for (const std::string kind : {
      "RobotEmbeddedInObstacle", "ComputePathToPose", "ComputePathThroughPoses",
      "ClearEntireCostmap", "FollowPath", "GoalUpdated", "CancelControl", "EscapeObstacle",
      "Wait", "BackUp", "IsStuck", "Spin", "RemovePassedGoals"})
  {
    factory.registerBuilder<ScriptedLeaf>(
      kind, [scenario, kind](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<ScriptedLeaf>(name, config, kind, scenario);
      });
  }
  auto tree = factory.createTreeFromFile(GetParam());
  const std::string planner = GetParam().find("through_poses") == std::string::npos ?
    "ComputePathToPose" : "ComputePathThroughPoses";

  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  ASSERT_EQ(scenario->starts[planner], 1);
  ASSERT_EQ(scenario->starts["FollowPath"], 1);

  scenario->embedded = true;
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(scenario->starts["EscapeObstacle"], 1);
  EXPECT_EQ(scenario->halts["FollowPath"], 1);

  scenario->escape_complete = true;
  // Recovery finishing is not navigation finishing. The old controller goal
  // must be halted and a newly planned path submitted to a fresh FollowPath.
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(scenario->starts[planner], 2);
  EXPECT_EQ(scenario->starts["FollowPath"], 2);

  scenario->tracking_complete = true;
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

INSTANTIATE_TEST_SUITE_P(
  Templates, RecoveryResume,
  ::testing::Values(
    TEMPLATE_BT_DIR "/navigate_to_pose_w_david_embedded_recovery.xml.tmpl",
    TEMPLATE_BT_DIR "/navigate_through_poses_w_david_embedded_recovery.xml.tmpl"));

}  // namespace
