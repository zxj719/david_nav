// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include <stdexcept>

#include "gtest/gtest.h"
#include "support/fake_stream.hpp"
#include "wheeltec_driver/serial_stream.hpp"
#include "wheeltec_driver/status_reader.hpp"

namespace wheeltec_driver
{
namespace
{

// First immutable P0.8 recorded frame, no device reads in this test suite.
const StatusFrame kGolden{
  0x7B, 0x01, 0, 0, 0, 0, 0, 0, 0, 0x16, 0xFF, 0xF4,
  0x40, 0x02, 0, 0x01, 0xFF, 0xFF, 0, 0, 0x10, 0xF2, 0xC6, 0x7D};

void append(test::FakeStream * stream, const StatusFrame & frame)
{
  stream->incoming.insert(stream->incoming.end(), frame.begin(), frame.end());
}

TEST(StatusReader, NoBytesProduceNoSampleOrIo)
{
  test::FakeStream stream;
  StatusReader reader;
  EXPECT_FALSE(reader.poll(stream));
  EXPECT_EQ(stream.reads, 0u);
  EXPECT_EQ(reader.counters().valid_frames, 0u);
  EXPECT_TRUE(stream.writes.empty());
}

TEST(StatusReader, PartialFrameSurvivesEveryPossibleSplit)
{
  for (std::size_t split = 1; split < kGolden.size(); ++split) {
    test::FakeStream stream;
    StatusReader reader;
    stream.incoming.insert(stream.incoming.end(), kGolden.begin(), kGolden.begin() + split);
    EXPECT_FALSE(reader.poll(stream));
    stream.incoming.insert(stream.incoming.end(), kGolden.begin() + split, kGolden.end());
    const auto state = reader.poll(stream);
    ASSERT_TRUE(state);
    EXPECT_EQ(state->voltage_raw, 4338);
    EXPECT_EQ(reader.counters().valid_frames, 1u);
    EXPECT_EQ(reader.counters().bytes, 24u);
  }
}

TEST(StatusReader, DrainAllAvailableFramesAndReturnOnlyLast)
{
  test::FakeStream stream;
  StatusReader reader;
  append(&stream, kGolden);
  auto second = kGolden;
  second[1] = 0;
  second[22] ^= 1;
  append(&stream, second);
  const auto state = reader.poll(stream);
  ASSERT_TRUE(state);
  EXPECT_EQ(state->flag_stop, 0);
  EXPECT_TRUE(stream.incoming.empty());
  EXPECT_EQ(reader.counters().valid_frames, 2u);
  EXPECT_EQ(stream.reads, 48u);
}

TEST(StatusReader, NoiseBeforeHeaderIsIgnored)
{
  test::FakeStream stream;
  stream.incoming = {0, 0xFF, 0x7D, 1};
  append(&stream, kGolden);
  StatusReader reader;
  EXPECT_TRUE(reader.poll(stream));
  EXPECT_EQ(reader.counters().bytes, 28u);
  EXPECT_EQ(reader.counters().framing_errors, 0u);
}

TEST(StatusReader, BadTailAndChecksumAreCountedSeparately)
{
  test::FakeStream stream;
  auto bad_tail = kGolden;
  bad_tail.back() = 0;
  append(&stream, bad_tail);
  auto bad_checksum = kGolden;
  bad_checksum[22] ^= 1;
  append(&stream, bad_checksum);
  append(&stream, kGolden);
  StatusReader reader;
  EXPECT_TRUE(reader.poll(stream));
  EXPECT_EQ(reader.counters().framing_errors, 1u);
  EXPECT_EQ(reader.counters().checksum_errors, 1u);
  EXPECT_EQ(reader.counters().valid_frames, 1u);
}

TEST(StatusReader, InvalidTrailingFrameDoesNotDiscardEarlierValidSample)
{
  test::FakeStream stream;
  append(&stream, kGolden);
  auto invalid = kGolden;
  invalid.back() = 0;
  append(&stream, invalid);
  StatusReader reader;
  EXPECT_TRUE(reader.poll(stream));
  EXPECT_EQ(reader.counters().valid_frames, 1u);
}

TEST(StatusReader, ShortReadKeepsPartialAccumulator)
{
  test::FakeStream stream;
  StatusReader reader;
  stream.incoming.insert(stream.incoming.end(), kGolden.begin(), kGolden.begin() + 7);
  EXPECT_FALSE(reader.poll(stream));
  stream.incoming.insert(stream.incoming.end(), kGolden.begin() + 7, kGolden.end());
  stream.short_read = true;
  EXPECT_FALSE(reader.poll(stream));
  stream.short_read = false;
  EXPECT_TRUE(reader.poll(stream));
  EXPECT_EQ(reader.counters().io_errors, 0u);
}

TEST(StatusReader, AvailableFailureClosesAndClearsPartialFrame)
{
  test::FakeStream stream;
  StatusReader reader;
  stream.incoming.insert(stream.incoming.end(), kGolden.begin(), kGolden.begin() + 3);
  EXPECT_FALSE(reader.poll(stream));
  stream.fail_available = true;
  EXPECT_FALSE(reader.poll(stream));
  EXPECT_FALSE(stream.connected);
  EXPECT_EQ(stream.closes, 1u);
  EXPECT_EQ(reader.lastError(), "fake available error");
  stream.fail_available = false;
  stream.open();
  append(&stream, kGolden);
  EXPECT_TRUE(reader.poll(stream));
  EXPECT_EQ(reader.counters().io_errors, 1u);
}

TEST(StatusReader, ReadFailureAfterValidFrameDiscardsThisPollLikeLegacy)
{
  test::FakeStream stream;
  append(&stream, kGolden);
  append(&stream, kGolden);
  stream.fail_read_at = 26;
  stream.fail_close = true;
  StatusReader reader;
  EXPECT_FALSE(reader.poll(stream));
  EXPECT_EQ(reader.counters().valid_frames, 1u);
  EXPECT_EQ(reader.counters().io_errors, 1u);
  EXPECT_EQ(reader.lastError(), "fake read error");
  EXPECT_FALSE(stream.connected);
}

TEST(StatusReader, IndependentInstancesNeverSharePartialBytes)
{
  test::FakeStream first, second;
  StatusReader one, two;
  first.incoming.insert(first.incoming.end(), kGolden.begin(), kGolden.begin() + 8);
  second.incoming.insert(second.incoming.end(), kGolden.begin() + 8, kGolden.end());
  EXPECT_FALSE(one.poll(first));
  EXPECT_FALSE(two.poll(second));
  append(&second, kGolden);
  EXPECT_TRUE(two.poll(second));
  EXPECT_EQ(one.counters().valid_frames, 0u);
}

TEST(StatusReader, ResetPartialRetainsLifetimeCounters)
{
  test::FakeStream stream;
  append(&stream, kGolden);
  StatusReader reader;
  ASSERT_TRUE(reader.poll(stream));
  stream.incoming = {kFrameHeader, 0};
  EXPECT_FALSE(reader.poll(stream));
  reader.resetPartial();
  append(&stream, kGolden);
  EXPECT_TRUE(reader.poll(stream));
  EXPECT_EQ(reader.counters().valid_frames, 2u);
}

TEST(SerialStream, ConstructionNeverOpensEvenAnImpossibleDevice)
{
  SerialStream stream("/does/not/exist/nav-refactor-test", 115200);
  EXPECT_FALSE(stream.isOpen());
  // Deliberately never call open(): this is a constructor side-effect contract.
}

TEST(SerialStream, MissingExplicitConfigurationIsRejected)
{
  EXPECT_THROW(SerialStream("", 115200), std::invalid_argument);
  EXPECT_THROW(SerialStream("/does/not/exist/nav-refactor-test", 0), std::invalid_argument);
}

}  // namespace
}  // namespace wheeltec_driver
