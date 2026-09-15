// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef WHEELTEC_DRIVER__STATUS_READER_HPP_
#define WHEELTEC_DRIVER__STATUS_READER_HPP_

#include <optional>
#include <string>

#include "wheeltec_driver/byte_stream.hpp"
#include "wheeltec_driver/protocol.hpp"

namespace wheeltec_driver
{

/// Cumulative counters for later diagnostics; resetPartial() does not reset them.
struct ReceiveCounters
{
  uint64_t bytes{};
  uint64_t valid_frames{};
  uint64_t framing_errors{};
  uint64_t checksum_errors{};
  uint64_t io_errors{};
};

/// Preserve the legacy byte accumulator and drain-all, use-last-frame policy.
/// This class is instance-local and not thread-safe; the owning driver serializes access.
class StatusReader
{
public:
  /// Drain only the bytes reported available at entry and return the last valid sample.
  /// Partial frames survive a short read; I/O exceptions discard them and close the port.
  /// Like the old driver, an I/O exception discards even a valid earlier sample in this poll.
  std::optional<ControllerState> poll(ByteStream & stream);
  /// Forget partial bytes after a disconnect, retaining lifetime counters.
  void resetPartial();
  /// Return cumulative transport and framing evidence, without clearing it.
  const ReceiveCounters & counters() const;
  /// Return the most recent transport error, empty until the first failure.
  const std::string & lastError() const;

private:
  std::optional<ControllerState> consume(uint8_t byte);
  StatusFrame frame_{};
  std::size_t count_{};
  ReceiveCounters counters_{};
  std::string last_error_;
};

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__STATUS_READER_HPP_
