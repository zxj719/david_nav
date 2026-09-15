// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "support/fake_stream.hpp"
#include "wheeltec_driver/driver_node.hpp"

namespace
{

struct Trace
{
  std::atomic<unsigned int> stops{};
  std::atomic<unsigned int> closes{};
  std::atomic<bool> stopped_before_context_shutdown{};
};

class SignalStream : public wheeltec_driver::test::FakeStream
{
public:
  explicit SignalStream(Trace & trace)
  : trace_(trace) {connected = false;}
  std::size_t write(const uint8_t * data, std::size_t size) override
  {
    const auto count = FakeStream::write(data, size);
    const auto zero = wheeltec_driver::encodeCommand(0.0, 0.0, 0.0);
    if (std::vector<uint8_t>(data, data + count) ==
      std::vector<uint8_t>(zero.begin(), zero.end()))
    {
      ++trace_.stops;
      trace_.stopped_before_context_shutdown = rclcpp::ok();
    }
    return count;
  }
  void close() override
  {
    FakeStream::close();
    ++trace_.closes;
  }

private:
  Trace & trace_;
};

}  // namespace

int main(int argc, char ** argv)
{
  const auto domain = std::getenv("ROS_DOMAIN_ID");
  if (domain == nullptr || std::string(domain) != "99" || argc != 2 ||
    (std::string(argv[1]) != "INT" && std::string(argv[1]) != "TERM"))
  {
    std::cerr << "requires guarded domain 99 and INT or TERM argument\n";
    return 2;
  }
  const int signal = std::string(argv[1]) == "INT" ? SIGINT : SIGTERM;
  rclcpp::init(0, nullptr);
  Trace trace;
  auto node = std::make_shared<wheeltec_driver::WheeltecDriverNode>(
    rclcpp::NodeOptions(), [&trace](const std::string &, uint32_t) {
      return std::make_unique<SignalStream>(trace);
    });
  wheeltec_driver::registerShutdownHook(node);
  // Signal only this test process. No external process is discovered or signaled.
  std::thread trigger([signal]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::raise(signal);
    });
  rclcpp::spin(node);
  trigger.join();
  // The asynchronous shutdown callback may still be completing after spin exits.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (trace.closes.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const bool callback_stopped = trace.stops == 1 && trace.closes == 1 &&
    trace.stopped_before_context_shutdown;
  node->stop();
  node.reset();
  const bool idempotent = trace.stops == 1 && trace.closes == 1;
  rclcpp::shutdown();
  std::cout << argv[1] << " callback_stop=" << callback_stopped <<
    " idempotent=" << idempotent << '\n';
  return callback_stopped && idempotent ? 0 : 1;
}
