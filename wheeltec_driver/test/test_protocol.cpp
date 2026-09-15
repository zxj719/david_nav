// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <array>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "wheeltec_driver/protocol.hpp"

namespace wheeltec_driver
{
namespace
{

std::vector<StatusFrame> recordedFrames()
{
  std::ifstream input(UPLINK_GOLDEN_FILE);
  if (!input) {
    throw std::runtime_error("cannot read the recorded P0.8 uplink golden file");
  }
  std::vector<StatusFrame> frames;
  std::string line;
  while (std::getline(input, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }
    std::istringstream row(line.substr(0, line.find('|')));
    StatusFrame frame{};
    for (auto & byte : frame) {
      unsigned int value;
      if (!(row >> std::hex >> value) || value > 255) {
        throw std::runtime_error("malformed recorded uplink golden row");
      }
      byte = static_cast<uint8_t>(value);
    }
    frames.push_back(frame);
  }
  return frames;
}

void setField(StatusFrame * frame, std::size_t offset, int16_t value)
{
  const auto bits = static_cast<uint16_t>(value);
  (*frame)[offset] = static_cast<uint8_t>(bits >> 8);
  (*frame)[offset + 1] = static_cast<uint8_t>(bits & 0xFF);
  (*frame)[22] = xorChecksum(frame->data(), 22);
}

TEST(Protocol, ZeroCommandHasFrozenBytes)
{
  const CommandFrame golden{0x7B, 0, 0, 0, 0, 0, 0, 0, 0, 0x7B, 0x7D};
  EXPECT_EQ(encodeCommand(0.0, 0.0, 0.0), golden);
}

TEST(Protocol, ObservedStationaryRotationHasFrozenBytes)
{
  const CommandFrame golden{0x7B, 0, 0, 0, 0, 0, 0, 0x02, 0x26, 0x5F, 0x7D};
  EXPECT_EQ(encodeCommand(0.0, 0.0, 0.55), golden);
}

TEST(Protocol, SignedCommandsRetainBigEndianAxisOrder)
{
  const CommandFrame golden{0x7B, 0, 0, 0xFF, 0x38, 0, 0x96, 0xFD, 0xDA, 0x0D, 0x7D};
  EXPECT_EQ(encodeCommand(-0.2, 0.15, -0.55), golden);
}

TEST(Protocol, FractionalMilliUnitsTruncateTowardZero)
{
  EXPECT_EQ(encodeCommand(0.0009, -0.0009, -0.0), encodeCommand(0.0, 0.0, 0.0));
  EXPECT_EQ(encodeCommand(1.2349, -1.2349, 0.0), encodeCommand(1.234, -1.234, 0.0));
}

TEST(Protocol, SignedFieldAndLegacyVelocityFormulaAreExhaustive)
{
  for (int32_t raw = -32768; raw <= 32767; ++raw) {
    const auto bits = static_cast<uint16_t>(raw);
    EXPECT_EQ(decodeSigned(bits >> 8, bits & 0xFF), raw);
    // Frozen Odom_Trans expression from the immutable Wheeltec snapshot.
    const float legacy = (raw / 1000) + (raw % 1000) * 0.001;
    EXPECT_EQ(decodeMilli(static_cast<int16_t>(raw)), legacy);
  }
}

TEST(Protocol, RepresentableCommandEndpointsWork)
{
  const auto frame = encodeCommand(-32.768, 32.767, 32.7679);
  EXPECT_EQ(decodeSigned(frame[3], frame[4]), -32768);
  EXPECT_EQ(decodeSigned(frame[5], frame[6]), 32767);
  EXPECT_EQ(decodeSigned(frame[7], frame[8]), 32767);
}

TEST(Protocol, InvalidCommandsAreRejectedBeforeAnyFrameIsReturned)
{
  for (const auto value : {32.768, -32.769, std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::max()})
  {
    EXPECT_THROW(encodeCommand(value, 0.0, 0.0), std::invalid_argument);
    EXPECT_THROW(encodeCommand(0.0, value, 0.0), std::invalid_argument);
    EXPECT_THROW(encodeCommand(0.0, 0.0, value), std::invalid_argument);
  }
}

TEST(Protocol, RecordedStationaryFramesKeepRawAndScaledValues)
{
  const auto frames = recordedFrames();
  ASSERT_EQ(frames.size(), 3u);
  const std::array<std::array<int16_t, 3>, 3> accelerometers{{
    {22, -12, 16386}, {-22, -4, 16288}, {-8, -6, 16340}}};
  const std::array<std::array<int16_t, 3>, 3> gyroscopes{{
    {1, -1, 0}, {3, -2, 0}, {-2, -1, 0}}};
  for (std::size_t index = 0; index < frames.size(); ++index) {
    const auto state = decodeStatus(frames[index]);
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->flag_stop, 1);
    EXPECT_EQ(state->voltage_raw, 4338);
    EXPECT_EQ(state->voltage, 4.338f);
    EXPECT_EQ(state->accelerometer_raw, accelerometers[index]);
    EXPECT_EQ(state->gyroscope_raw, gyroscopes[index]);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      EXPECT_EQ(state->velocity[axis], 0.0f);
      EXPECT_EQ(
        state->acceleration[axis],
        static_cast<double>(accelerometers[index][axis] / 1671.84f));
      EXPECT_EQ(
        state->angular_velocity[axis],
        static_cast<double>(gyroscopes[index][axis] * 0.00026644f));
    }
  }
}

TEST(Protocol, SyntheticSignedMotionFieldsKeepLegacyScaling)
{
  auto frame = recordedFrames().front();
  setField(&frame, 2, -1234);
  setField(&frame, 4, 4321);
  setField(&frame, 6, -550);
  const auto state = decodeStatus(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->velocity[0], -1.234f);
  EXPECT_EQ(state->velocity[1], 4.321f);
  EXPECT_EQ(state->velocity[2], -0.55f);
}

TEST(Protocol, VoltageSignednessIsNotSilentlyChanged)
{
  auto frame = recordedFrames().front();
  setField(&frame, 20, -1);
  const auto state = decodeStatus(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->voltage_raw, -1);
  EXPECT_EQ(state->voltage, -0.001f);
}

TEST(Protocol, EverySingleByteCorruptionIsRejected)
{
  const auto golden = recordedFrames().front();
  for (std::size_t index = 0; index < golden.size(); ++index) {
    for (unsigned int bit = 0; bit < 8; ++bit) {
      auto frame = golden;
      frame[index] ^= static_cast<uint8_t>(1u << bit);
      EXPECT_FALSE(decodeStatus(frame).has_value()) << "index=" << index << " bit=" << bit;
    }
  }
}

TEST(Protocol, FrameSentinelsInsidePayloadAreOrdinaryData)
{
  auto frame = recordedFrames().front();
  setField(&frame, 8, 0x7B7D);
  const auto state = decodeStatus(frame);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->accelerometer_raw[0], 0x7B7D);
}

TEST(Protocol, DecodeDoesNotMutateInput)
{
  const auto frame = recordedFrames().front();
  const auto before = frame;
  ASSERT_TRUE(decodeStatus(frame).has_value());
  EXPECT_EQ(frame, before);
}

TEST(Protocol, ChecksumSupportsEmptyButNotNullNonemptyBuffers)
{
  EXPECT_EQ(xorChecksum(nullptr, 0), 0);
  EXPECT_THROW(xorChecksum(nullptr, 1), std::invalid_argument);
  const std::array<uint8_t, 3> data{0x7B, 0x7D, 0xFF};
  EXPECT_EQ(xorChecksum(data.data(), data.size()), 0xF9);
}

}  // namespace
}  // namespace wheeltec_driver
