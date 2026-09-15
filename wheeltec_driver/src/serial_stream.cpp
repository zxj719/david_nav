// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include "wheeltec_driver/serial_stream.hpp"

#include <stdexcept>

#include "serial/serial.h"

namespace wheeltec_driver
{

SerialStream::SerialStream(const std::string & port, uint32_t baud, uint32_t timeout_ms)
: serial_(std::make_unique<serial::Serial>()), port_(port), baud_(baud), timeout_ms_(timeout_ms)
{
  if (port.empty() || baud == 0) {
    throw std::invalid_argument("serial port and nonzero baud rate are required");
  }
}

SerialStream::~SerialStream() = default;

void SerialStream::open()
{
  serial_->setPort(port_);
  serial_->setBaudrate(baud_);
  auto timeout = serial::Timeout::simpleTimeout(timeout_ms_);
  serial_->setTimeout(timeout);
  serial_->open();
}

void SerialStream::close()
{
  serial_->close();
}

bool SerialStream::isOpen() const
{
  return serial_->isOpen();
}

std::size_t SerialStream::available()
{
  return serial_->available();
}

std::size_t SerialStream::read(uint8_t * data, std::size_t size)
{
  return serial_->read(data, size);
}

std::size_t SerialStream::write(const uint8_t * data, std::size_t size)
{
  return serial_->write(data, size);
}

}  // namespace wheeltec_driver
