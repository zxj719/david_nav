// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include "wheeltec_driver/protocol.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace wheeltec_driver
{
namespace
{

constexpr float kAccelerometerRatio = 1671.84f;
constexpr float kGyroscopeRatio = 0.00026644f;

int16_t encodeMilli(double value)
{
  const auto scaled = std::trunc(value * 1000.0);
  if (!std::isfinite(scaled) || scaled < std::numeric_limits<int16_t>::min() ||
    scaled > std::numeric_limits<int16_t>::max())
  {
    throw std::invalid_argument("controller command must fit a finite signed int16 milli-unit");
  }
  return static_cast<int16_t>(scaled);
}

}  // namespace

uint8_t xorChecksum(const uint8_t * data, std::size_t size)
{
  if (data == nullptr && size != 0) {
    throw std::invalid_argument("nonempty checksum buffer must not be null");
  }
  uint8_t result = 0;
  for (std::size_t index = 0; index < size; ++index) {
    result ^= data[index];
  }
  return result;
}

int16_t decodeSigned(uint8_t high, uint8_t low)
{
  const auto raw = static_cast<int32_t>(high) * 256 + low;
  return static_cast<int16_t>(raw >= 32768 ? raw - 65536 : raw);
}

float decodeMilli(int16_t raw)
{
  // Keep integer quotient/remainder and the final float rounding exactly as in
  // Odom_Trans/Power_voltage; raw / 1000.0f is not equivalent for every int16.
  return static_cast<float>((raw / 1000) + (raw % 1000) * 0.001);
}

CommandFrame encodeCommand(double vx, double vy, double wz)
{
  // Validate all fields before making the frame available to the caller.
  const std::array<int16_t, 3> values{encodeMilli(vx), encodeMilli(vy), encodeMilli(wz)};
  CommandFrame frame{};
  frame.front() = kFrameHeader;
  frame.back() = kFrameTail;
  for (std::size_t axis = 0; axis < values.size(); ++axis) {
    const auto bits = static_cast<uint16_t>(values[axis]);
    frame[3 + axis * 2] = static_cast<uint8_t>(bits >> 8);
    frame[4 + axis * 2] = static_cast<uint8_t>(bits & 0xFF);
  }
  frame[9] = xorChecksum(frame.data(), 9);
  return frame;
}

std::optional<ControllerState> decodeStatus(const StatusFrame & frame)
{
  if (frame.front() != kFrameHeader || frame.back() != kFrameTail ||
    frame[22] != xorChecksum(frame.data(), 22))
  {
    return std::nullopt;
  }
  ControllerState state;
  state.flag_stop = frame[1];
  for (std::size_t axis = 0; axis < 3; ++axis) {
    state.velocity_raw[axis] = decodeSigned(frame[2 + axis * 2], frame[3 + axis * 2]);
    state.velocity[axis] = decodeMilli(state.velocity_raw[axis]);
    state.accelerometer_raw[axis] = decodeSigned(frame[8 + axis * 2], frame[9 + axis * 2]);
    state.gyroscope_raw[axis] = decodeSigned(frame[14 + axis * 2], frame[15 + axis * 2]);
    state.acceleration[axis] = state.accelerometer_raw[axis] / kAccelerometerRatio;
    state.angular_velocity[axis] = state.gyroscope_raw[axis] * kGyroscopeRatio;
  }
  state.voltage_raw = decodeSigned(frame[20], frame[21]);
  state.voltage = decodeMilli(state.voltage_raw);
  return state;
}

}  // namespace wheeltec_driver
