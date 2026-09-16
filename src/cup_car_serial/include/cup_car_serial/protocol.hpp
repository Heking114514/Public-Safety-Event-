#ifndef CUP_CAR_SERIAL__PROTOCOL_HPP_
#define CUP_CAR_SERIAL__PROTOCOL_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace cup_car_serial
{
struct ControlTelemetryFrame
{
  uint32_t mcu_time_ms{};
  uint32_t sample_sequence{};
  uint8_t mode{};
  bool emergency_stop{};
  bool command_valid{};
  uint32_t command_age_ms{};
  int32_t received_linear_velocity_milli{};
  int32_t received_angular_velocity_milli{};
  int32_t target_left_velocity_milli{};
  int32_t target_right_velocity_milli{};
  int32_t measured_left_velocity_milli{};
  int32_t measured_right_velocity_milli{};
  int16_t pwm_left{};
  int16_t pwm_right{};
};

struct Bmi088ImuFrame
{
  uint32_t mcu_time_ms{};
  uint32_t sample_sequence{};
  uint8_t status{};
  int16_t gyro_x_counts{};
  int16_t gyro_y_counts{};
  int16_t gyro_z_counts{};
  int16_t temperature_centideg_c{};
};

struct Bmi088AttitudeFrame
{
  uint32_t mcu_time_ms{};
  uint32_t sample_sequence{};
  uint8_t status{};
  int32_t yaw_mdeg{};
  int32_t gyro_z_mdps{};
  int32_t bias_z_mdps{};
  uint32_t startup_samples{};
};

enum class SampleSequenceDisposition
{
  NEW_SAMPLE,
  DUPLICATE,
  OUT_OF_ORDER,
  SOURCE_RESTART
};

bool parse_encoder_frame(const std::string & line, std::vector<int32_t> * values);
bool parse_control_telemetry_frame(const std::string & line, ControlTelemetryFrame * frame);
bool parse_bmi088_imu_frame(const std::string & line, Bmi088ImuFrame * frame);
bool parse_bmi088_attitude_frame(const std::string & line, Bmi088AttitudeFrame * frame);
bool control_state_is_healthy(const ControlTelemetryFrame & frame);
SampleSequenceDisposition classify_sample_sequence(
  uint32_t previous_sequence, uint32_t previous_mcu_time_ms,
  uint32_t sequence, uint32_t mcu_time_ms);
}  // namespace cup_car_serial

#endif  // CUP_CAR_SERIAL__PROTOCOL_HPP_
