// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef WHEELTEC_DRIVER__BYTE_STREAM_HPP_
#define WHEELTEC_DRIVER__BYTE_STREAM_HPP_

#include <cstddef>
#include <cstdint>

namespace wheeltec_driver
{

/// Injectable byte transport. Construction must not open a device.
/// Implementations may throw std::exception for transport failures.
class ByteStream
{
public:
  virtual ~ByteStream() = default;
  /// Open the configured stream; no-op semantics are implementation-specific.
  virtual void open() = 0;
  /// Close the transport without writing a command.
  virtual void close() = 0;
  /// Report whether the transport currently considers itself connected.
  virtual bool isOpen() const = 0;
  /// Return the current buffered byte count without waiting for a full frame.
  virtual std::size_t available() = 0;
  /// Read up to size bytes and return the count actually transferred.
  virtual std::size_t read(uint8_t * data, std::size_t size) = 0;
  /// Write up to size bytes and return the count actually transferred.
  virtual std::size_t write(const uint8_t * data, std::size_t size) = 0;
};

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__BYTE_STREAM_HPP_
