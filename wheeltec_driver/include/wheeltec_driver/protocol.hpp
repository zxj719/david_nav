// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#ifndef WHEELTEC_DRIVER__PROTOCOL_HPP_
#define WHEELTEC_DRIVER__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace wheeltec_driver
{

constexpr std::size_t kCommandSize = 11;
constexpr std::size_t kStatusSize = 24;
constexpr uint8_t kFrameHeader = 0x7B;
constexpr uint8_t kFrameTail = 0x7D;
using CommandFrame = std::array<uint8_t, kCommandSize>;
using StatusFrame = std::array<uint8_t, kStatusSize>;

/// One decoded controller sample, without ROS messages or hardware resources.
struct ControllerState
{
  uint8_t flag_stop{};
  std::array<int16_t, 3> velocity_raw{};  ///< mm/s, mm/s, mrad/s.
  std::array<float, 3> velocity{};  ///< m/s, m/s, rad/s; legacy float precision.
  std::array<int16_t, 3> accelerometer_raw{};
  std::array<int16_t, 3> gyroscope_raw{};
  std::array<double, 3> acceleration{};  ///< m/s^2, after legacy float division.
  std::array<double, 3> angular_velocity{};  ///< rad/s, after legacy float product.
  int16_t voltage_raw{};  ///< Preserve legacy signed-int16 interpretation.
  float voltage{};  ///< V; no guessed correction of observed controller values.
};

/// XOR exactly size bytes. Empty buffers are valid; nonempty buffers need data.
/// @throws std::invalid_argument if data is null and size is nonzero.
uint8_t xorChecksum(const uint8_t * data, std::size_t size);

/// Decode two big-endian bytes with defined signed two's-complement semantics.
int16_t decodeSigned(uint8_t high, uint8_t low);

/// Convert a signed milli-unit value using the original driver's float formula.
float decodeMilli(int16_t raw);

/// Encode vx/vy in m/s and wz in rad/s, truncating each scaled value toward zero.
/// @throws std::invalid_argument for nonfinite or unrepresentable int16 values.
/// No bytes are emitted and no hardware resource is accessed by this function.
CommandFrame encodeCommand(double vx, double vy, double wz);

/// Validate header, tail and XOR before decoding one complete 24-byte sample.
/// Returns nullopt on malformed framing/checksum; never changes caller state.
std::optional<ControllerState> decodeStatus(const StatusFrame & frame);

}  // namespace wheeltec_driver

#endif  // WHEELTEC_DRIVER__PROTOCOL_HPP_
