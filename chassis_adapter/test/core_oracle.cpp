// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <iomanip>
#include <iostream>
#include <string>

#include "chassis_adapter/core.hpp"

// Test-only stdin/stdout harness. No ROS initialization or command publication.
int main()
{
  chassis_adapter::AdapterConfig config;
  std::cin >> config.cmd_vel_timeout >> config.min_linear_x >> config.max_linear_x >>
  config.min_angular_z >> config.min_stationary_angular_z >>
  config.stationary_angular_threshold >> config.max_angular_z >> config.publish_hz;
  if (!std::cin) {
    return 2;
  }
  chassis_adapter::AdapterState state(config, 0);
  std::cout << std::setprecision(17);
  char operation;
  while (std::cin >> operation) {
    int64_t now;
    if (operation == 'R') {
      double x, y, yaw;
      std::cin >> x >> y >> yaw >> now;
      state.receive(x, y, yaw, now);
    } else if (operation == 'O') {
      std::cin >> now;
      const auto output = state.output(now);
      if (output) {
        std::cout << output->first << " " << output->second << "\n";
      } else {
        std::cout << "none\n";
      }
    } else if (operation == 'D') {
      std::cin >> now;
      const auto output = state.diagnosticOutput(now);
      std::cout << state.inputCount() << " " << state.isStale(now) << " " <<
        output.first << " " << output.second << "\n";
    } else if (operation == 'P') {
      std::cout << config.periodSeconds() << "\n";
    } else {
      return 3;
    }
    if (!std::cin) {
      return 4;
    }
  }
  return 0;
}
