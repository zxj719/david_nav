// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <iostream>
#include <memory>

#include "chassis_adapter/adapter_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<chassis_adapter::AdapterNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    std::cerr << "chassis_adapter: " << error.what() << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
