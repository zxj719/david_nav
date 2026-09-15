// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef SUPPORT__LOG_CAPTURE_HPP_
#define SUPPORT__LOG_CAPTURE_HPP_

#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rcutils/logging.h"

namespace wheeltec_driver
{
namespace test
{

/// Scoped capture for serialized sandbox tests; restores the process-global handler.
class LogCapture
{
public:
  LogCapture()
  : previous_(rcutils_logging_get_output_handler()),
    level_(rcutils_logging_get_logger_level("wheeltec_robot"))
  {
    const auto result = rcutils_logging_set_logger_level(
      "wheeltec_robot", RCUTILS_LOG_SEVERITY_DEBUG);
    (void)result;
    messages_.clear();
    rcutils_logging_set_output_handler(&LogCapture::record);
  }
  ~LogCapture()
  {
    rcutils_logging_set_output_handler(previous_);
    const auto result = rcutils_logging_set_logger_level("wheeltec_robot", level_);
    (void)result;
  }
  std::size_t count(const std::string & needle, int severity)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::size_t result = 0;
    for (const auto & entry : messages_) {
      result += entry.first == severity && entry.second.find(needle) != std::string::npos;
    }
    return result;
  }

private:
  static void record(
    const rcutils_log_location_t *, int severity, const char * name,
    rcutils_time_point_value_t, const char * format, va_list * args)
  {
    if (name == nullptr || std::string(name) != "wheeltec_robot") {
      return;
    }
    char buffer[2048];
    std::vsnprintf(buffer, sizeof(buffer), format, *args);
    std::lock_guard<std::mutex> lock(mutex_);
    messages_.emplace_back(severity, buffer);
  }
  rcutils_logging_output_handler_t previous_;
  int level_;
  inline static std::mutex mutex_;
  inline static std::vector<std::pair<int, std::string>> messages_;
};

}  // namespace test
}  // namespace wheeltec_driver

#endif  // SUPPORT__LOG_CAPTURE_HPP_
