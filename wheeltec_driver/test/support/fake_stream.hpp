// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef SUPPORT__FAKE_STREAM_HPP_
#define SUPPORT__FAKE_STREAM_HPP_

#include <algorithm>
#include <deque>
#include <limits>
#include <stdexcept>
#include <vector>

#include "wheeltec_driver/byte_stream.hpp"

namespace wheeltec_driver
{
namespace test
{

/// Memory-only serial double with deterministic short reads and injected errors.
class FakeStream : public ByteStream
{
public:
  void open() override
  {
    ++opens;
    if (fail_open) {
      throw std::runtime_error("fake open error");
    }
    connected = true;
  }
  void close() override
  {
    ++closes;
    connected = false;
    if (fail_close) {
      throw std::runtime_error("fake close error");
    }
  }
  bool isOpen() const override {return connected;}
  std::size_t available() override
  {
    if (fail_available) {
      throw std::runtime_error("fake available error");
    }
    return incoming.size();
  }
  std::size_t read(uint8_t * data, std::size_t size) override
  {
    if (reads++ == fail_read_at) {
      throw std::runtime_error("fake read error");
    }
    if (short_read) {
      return 0;
    }
    const auto count = std::min(size, incoming.size());
    for (std::size_t index = 0; index < count; ++index) {
      data[index] = incoming.front();
      incoming.pop_front();
    }
    return count;
  }
  std::size_t write(const uint8_t * data, std::size_t size) override
  {
    if (fail_write || !connected) {
      throw std::runtime_error("fake write error");
    }
    const auto count = std::min(size, write_limit);
    writes.emplace_back(data, data + count);
    return count;
  }

  bool connected{true};
  bool fail_open{};
  bool fail_close{};
  bool fail_available{};
  bool fail_write{};
  bool short_read{};
  std::size_t fail_read_at{std::numeric_limits<std::size_t>::max()};
  std::size_t write_limit{std::numeric_limits<std::size_t>::max()};
  std::size_t reads{};
  std::size_t opens{};
  std::size_t closes{};
  std::deque<uint8_t> incoming;
  std::vector<std::vector<uint8_t>> writes;
};

}  // namespace test
}  // namespace wheeltec_driver

#endif  // SUPPORT__FAKE_STREAM_HPP_
