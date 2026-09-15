// Copyright 2026 David navigation contributors
// SPDX-License-Identifier: Apache-2.0

#include "wheeltec_driver/driver_core.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>

namespace wheeltec_driver
{

DriverCore::DriverCore(
  ByteStream & stream, const std::array<float, 4> & scales, double initial_time)
: stream_(stream), scales_(scales), last_time_(initial_time)
{
  if (!std::isfinite(initial_time) ||
    !std::all_of(scales.begin(), scales.end(), [](float value) {return std::isfinite(value);}))
  {
    throw std::invalid_argument("driver calibration and initial time must be finite");
  }
}

DriverCore::~DriverCore()
{
  shutdown();
}

bool DriverCore::connect()
{
  if (stopped_) {
    return false;
  }
  ++counters_.connection_attempts;
  reader_.resetPartial();
  try {
    if (stream_.isOpen()) {
      stream_.close();
    }
  } catch (const std::exception &) {
    // As in the old reconnect routine, still attempt open after a close failure.
  }
  try {
    stream_.open();
    if (stream_.isOpen()) {
      ++counters_.successful_connections;
      return true;
    }
    last_error_ = "serial open returned with a closed stream";
  } catch (const std::exception & error) {
    last_error_ = error.what();
  }
  return false;
}

void DriverCore::resetSamplingTime(double now)
{
  if (!std::isfinite(now)) {
    throw std::invalid_argument("sampling time must be finite");
  }
  last_time_ = now;
}

std::optional<DriverSample> DriverCore::tick(double now)
{
  if (!std::isfinite(now)) {
    throw std::invalid_argument("sampling time must be finite");
  }
  if (stopped_) {
    return std::nullopt;
  }
  if (!stream_.isOpen()) {
    if (reconnect_divider_++ % 100 == 0) {
      connect();
    }
    return std::nullopt;
  }
  const auto previous_errors = reader_.counters().io_errors;
  const auto sensor = reader_.poll(stream_);
  if (reader_.counters().io_errors != previous_errors) {
    last_error_ = reader_.lastError();
  }
  if (!sensor) {
    return std::nullopt;
  }
  DriverSample sample;
  sample.sensor = *sensor;
  sample.velocity = sensor->velocity;
  sample.velocity[0] *= scales_[0];
  sample.velocity[1] *= scales_[1];
  sample.velocity[2] *= sample.velocity[2] >= 0 ? scales_[2] : scales_[3];
  const double dt = now - last_time_;
  // Preserve legacy float += double integration and the old heading for both axes.
  pose_[0] += (sample.velocity[0] * std::cos(pose_[2]) -
    sample.velocity[1] * std::sin(pose_[2])) * dt;
  pose_[1] += (sample.velocity[0] * std::sin(pose_[2]) +
    sample.velocity[1] * std::cos(pose_[2])) * dt;
  pose_[2] += sample.velocity[2] * dt;
  sample.pose = pose_;
  sample.publish_voltage = voltage_divider_++ > 10;
  if (sample.publish_voltage) {
    voltage_divider_ = 0;
  }
  last_time_ = now;
  latest_ = sample;
  return sample;
}

bool DriverCore::command(double vx, double vy, double wz)
{
  if (stopped_) {
    return false;
  }
  try {
    return writeFrame(encodeCommand(vx, vy, wz));
  } catch (const std::invalid_argument & error) {
    ++counters_.rejected_commands;
    last_error_ = error.what();
    writeFrame(encodeCommand(0.0, 0.0, 0.0));
    return false;
  }
}

bool DriverCore::writeFrame(const CommandFrame & frame)
{
  try {
    if (stream_.write(frame.data(), frame.size()) == frame.size()) {
      ++counters_.transmitted_frames;
      return true;
    }
    last_error_ = "short serial command write";
  } catch (const std::exception & error) {
    last_error_ = error.what();
  }
  ++counters_.transmit_errors;
  return false;
}

bool DriverCore::shutdown()
{
  if (stopped_) {
    return false;
  }
  stopped_ = true;
  bool sent = false;
  try {
    if (stream_.isOpen()) {
      sent = writeFrame(encodeCommand(0.0, 0.0, 0.0));
      stream_.close();
    }
  } catch (const std::exception & error) {
    last_error_ = error.what();
  }
  reader_.resetPartial();
  return sent;
}

const std::optional<DriverSample> & DriverCore::latest() const
{
  return latest_;
}

const DriverCounters & DriverCore::counters() const
{
  return counters_;
}

const ReceiveCounters & DriverCore::receiveCounters() const
{
  return reader_.counters();
}

const std::string & DriverCore::lastError() const
{
  return last_error_;
}

bool DriverCore::online() const
{
  return stream_.isOpen();
}

}  // namespace wheeltec_driver
