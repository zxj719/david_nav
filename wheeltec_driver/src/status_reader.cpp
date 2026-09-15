// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include "wheeltec_driver/status_reader.hpp"

#include <exception>

namespace wheeltec_driver
{

std::optional<ControllerState> StatusReader::poll(ByteStream & stream)
{
  std::optional<ControllerState> latest;
  try {
    auto remaining = stream.available();
    while (remaining-- > 0) {
      uint8_t byte{};
      if (stream.read(&byte, 1) != 1) {
        break;
      }
      ++counters_.bytes;
      const auto state = consume(byte);
      if (state) {
        latest = state;
      }
    }
  } catch (const std::exception & error) {
    ++counters_.io_errors;
    last_error_ = error.what();
    resetPartial();
    try {
      if (stream.isOpen()) {
        stream.close();
      }
    } catch (const std::exception &) {
      // Preserve the original read error even if closing a dead port also fails.
    }
    return std::nullopt;
  }
  return latest;
}

std::optional<ControllerState> StatusReader::consume(uint8_t byte)
{
  // Exactly the old Get_Sensor_Data_New framing rule: a header starts a
  // 24-byte candidate, embedded sentinel bytes are ordinary payload, and an
  // invalid candidate is dropped whole (no new resynchronization heuristic).
  if (count_ == 0 && byte != kFrameHeader) {
    return std::nullopt;
  }
  frame_[count_++] = byte;
  if (count_ < frame_.size()) {
    return std::nullopt;
  }
  count_ = 0;
  if (frame_.back() != kFrameTail) {
    ++counters_.framing_errors;
    return std::nullopt;
  }
  const auto state = decodeStatus(frame_);
  if (state) {
    ++counters_.valid_frames;
  } else {
    ++counters_.checksum_errors;
  }
  return state;
}

void StatusReader::resetPartial()
{
  count_ = 0;
}

const ReceiveCounters & StatusReader::counters() const
{
  return counters_;
}

const std::string & StatusReader::lastError() const
{
  return last_error_;
}

}  // namespace wheeltec_driver
