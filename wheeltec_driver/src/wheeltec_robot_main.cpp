// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "wheeltec_driver/driver_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<wheeltec_driver::WheeltecDriverNode>();
    wheeltec_driver::registerShutdownHook(node);
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("wheeltec_robot"), "driver failed: %s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
