#include "cup_car_serial/protocol.hpp"

#include <cerrno>
#include <climits>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

namespace cup_car_serial
{
namespace
{
std::vector<std::string> split_fields(const std::string & line)
{
  std::vector<std::string> fields;
  size_t start = 0;
  while (start <= line.size()) {
    const size_t comma = line.find(',', start);
    fields.push_back(line.substr(start, comma - start));
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  return fields;
}

bool parse_int32(const std::string & text, int32_t * value)
{
  if (text.empty()) {
    return false;
  }
  char * end = nullptr;
  errno = 0;
  const long long parsed = std::strtoll(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0' ||
    parsed < INT32_MIN || parsed > INT32_MAX)
  {
    return false;
  }
  *value = static_cast<int32_t>(parsed);
  return true;
}

bool parse_uint32(const std::string & text, uint32_t * value)
{
  if (text.empty() || text.front() == '-') {
    return false;
  }
  char * end = nullptr;
  errno = 0;
  const unsigned long long parsed = std::strtoull(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0' ||
    parsed > std::numeric_limits<uint32_t>::max())
  {
    return false;
  }
  *value = static_cast<uint32_t>(parsed);
  return true;
}

bool parse_flag(const std::string & text, bool * value)
{
  uint32_t parsed;
  if (!parse_uint32(text, &parsed) || parsed > 1U) {
    return false;
  }
  *value = parsed == 1U;
  return true;
}

bool parse_int16(const std::string & text, int16_t * value)
{
  int32_t parsed;
  if (!parse_int32(text, &parsed) || parsed < INT16_MIN || parsed > INT16_MAX) {
    return false;
  }
  *value = static_cast<int16_t>(parsed);
  return true;
}

bool parse_uint8(const std::string & text, uint8_t * value)
{
  uint32_t parsed;
  if (!parse_uint32(text, &parsed) || parsed > UINT8_MAX) {
    return false;
  }
  *value = static_cast<uint8_t>(parsed);
  return true;
}
}  // namespace

bool parse_encoder_frame(const std::string & line, std::vector<int32_t> * values)
{
  if (values == nullptr || line.rfind("ENC,", 0) != 0) {
    return false;
  }
  const auto fields = split_fields(line);
  if (fields.size() != 5 && fields.size() != 3) {
    return false;
  }

  values->clear();
  values->reserve(4);
  if (fields.size() == 3) {
    int32_t left_total;
    int32_t right_total;
    if (!parse_int32(fields[1], &left_total) || !parse_int32(fields[2], &right_total)) {
      return false;
    }
    *values = {0, 0, left_total, right_total};
    return true;
  }

  for (size_t index = 1; index < fields.size(); ++index) {
    int32_t value;
    if (!parse_int32(fields[index], &value)) {
      values->clear();
      return false;
    }
    values->push_back(value);
  }
  return true;
}

bool parse_control_telemetry_frame(const std::string & line, ControlTelemetryFrame * frame)
{
  if (frame == nullptr || line.rfind("CTL,", 0) != 0) {
    return false;
  }
  const auto fields = split_fields(line);
  if (fields.size() != 15) {
    return false;
  }

  ControlTelemetryFrame parsed;
  uint32_t mode;
  if (!parse_uint32(fields[1], &parsed.mcu_time_ms) ||
    !parse_uint32(fields[2], &parsed.sample_sequence) ||
    !parse_uint32(fields[3], &mode) || mode > 1U ||
    !parse_flag(fields[4], &parsed.emergency_stop) ||
    !parse_flag(fields[5], &parsed.command_valid) ||
    !parse_uint32(fields[6], &parsed.command_age_ms) ||
    !parse_int32(fields[7], &parsed.received_linear_velocity_milli) ||
    !parse_int32(fields[8], &parsed.received_angular_velocity_milli) ||
    !parse_int32(fields[9], &parsed.target_left_velocity_milli) ||
    !parse_int32(fields[10], &parsed.target_right_velocity_milli) ||
    !parse_int32(fields[11], &parsed.measured_left_velocity_milli) ||
    !parse_int32(fields[12], &parsed.measured_right_velocity_milli) ||
    !parse_int16(fields[13], &parsed.pwm_left) ||
    !parse_int16(fields[14], &parsed.pwm_right))
  {
    return false;
  }
  parsed.mode = static_cast<uint8_t>(mode);
  *frame = parsed;
  return true;
}

bool parse_bmi088_imu_frame(const std::string & line, Bmi088ImuFrame * frame)
{
  if (frame == nullptr || line.rfind("IMU,", 0) != 0) {
    return false;
  }
  const auto fields = split_fields(line);
  if (fields.size() != 8) {
    return false;
  }

  Bmi088ImuFrame parsed;
  if (!parse_uint32(fields[1], &parsed.mcu_time_ms) ||
    !parse_uint32(fields[2], &parsed.sample_sequence) ||
    !parse_uint8(fields[3], &parsed.status) ||
    !parse_int16(fields[4], &parsed.gyro_x_counts) ||
    !parse_int16(fields[5], &parsed.gyro_y_counts) ||
    !parse_int16(fields[6], &parsed.gyro_z_counts) ||
    !parse_int16(fields[7], &parsed.temperature_centideg_c))
  {
    return false;
  }

  *frame = parsed;
  return true;
}

bool parse_bmi088_attitude_frame(const std::string & line, Bmi088AttitudeFrame * frame)
{
  if (frame == nullptr || line.rfind("ATT,", 0) != 0) {
    return false;
  }
  const auto fields = split_fields(line);
  if (fields.size() != 8) {
    return false;
  }

  Bmi088AttitudeFrame parsed;
  if (!parse_uint32(fields[1], &parsed.mcu_time_ms) ||
    !parse_uint32(fields[2], &parsed.sample_sequence) ||
    !parse_uint8(fields[3], &parsed.status) ||
    !parse_int32(fields[4], &parsed.yaw_mdeg) ||
    !parse_int32(fields[5], &parsed.gyro_z_mdps) ||
    !parse_int32(fields[6], &parsed.bias_z_mdps) ||
    !parse_uint32(fields[7], &parsed.startup_samples))
  {
    return false;
  }

  *frame = parsed;
  return true;
}

bool control_state_is_healthy(const ControlTelemetryFrame & frame)
{
  return frame.mode == 1U && !frame.emergency_stop && frame.command_valid;
}

SampleSequenceDisposition classify_sample_sequence(
  uint32_t previous_sequence, uint32_t previous_mcu_time_ms,
  uint32_t sequence, uint32_t mcu_time_ms)
{
  const uint32_t sequence_advance = sequence - previous_sequence;
  if (sequence_advance == 0U) {
    return SampleSequenceDisposition::DUPLICATE;
  }

  const uint32_t mcu_time_advance = mcu_time_ms - previous_mcu_time_ms;
  const bool sequence_is_forward = sequence_advance < (uint32_t{1} << 31U);
  const bool mcu_time_is_forward = mcu_time_advance < (uint32_t{1} << 31U);
  if (sequence_is_forward && mcu_time_is_forward) {
    return SampleSequenceDisposition::NEW_SAMPLE;
  }

  // A source restart normally resets both counters. A wrap of either uint32
  // counter remains a forward modular advance and is accepted above.
  if (!sequence_is_forward && !mcu_time_is_forward) {
    return SampleSequenceDisposition::SOURCE_RESTART;
  }
  return SampleSequenceDisposition::OUT_OF_ORDER;
}
}  // namespace cup_car_serial
