// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef WHEELTEC_DRIVER__SERIAL_STREAM_HPP_
#define WHEELTEC_DRIVER__SERIAL_STREAM_HPP_

#include <memory>
#include <string>

#include "wheeltec_driver/byte_stream.hpp"

namespace serial
{
class Serial;
}  // namespace serial

namespace wheeltec_driver
{

/// Serial implementation using the vendored library, with no implicit open.
class SerialStream final : public ByteStream
{
public:
  /// Save explicit device/baud configuration; only open() touches the port.
  /// @throws std::invalid_argument for an empty port or zero baud rate.
  explicit SerialStream(const std::string & port, uint32_t baud, uint32_t timeout_ms = 2000);
  ~SerialStream() override;
  SerialStream(const SerialStream &) = delete;
  SerialStream & operator=(const SerialStream &) = delete;

  void open() override;
  void close() override;
  bool isOpen() const override;
  std::size_t available() override;
  std::size_t read(uint8_t * data, std::size_t size) override;
  std::size_t write(const uint8_t * data, std::size_t size) override;

private:
  std::unique_ptr<serial::Serial> serial_;
  std::string port_;
  uint32_t baud_;
  uint32_t timeout_ms_;
};

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__SERIAL_STREAM_HPP_
