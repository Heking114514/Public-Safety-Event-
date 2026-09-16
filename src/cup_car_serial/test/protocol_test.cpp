#include <cstdint>
#include <limits>
#include <vector>

#include "cup_car_serial/protocol.hpp"
#include "gtest/gtest.h"

TEST(Protocol, ParsesCurrentEncoderFrame)
{
  std::vector<int32_t> values;
  ASSERT_TRUE(cup_car_serial::parse_encoder_frame("ENC,1250,125,1234,-1188", &values));
  EXPECT_EQ(values, (std::vector<int32_t>{1250, 125, 1234, -1188}));
}

TEST(Protocol, ParsesControlTelemetryFrame)
{
  cup_car_serial::ControlTelemetryFrame frame;
  ASSERT_TRUE(cup_car_serial::parse_control_telemetry_frame(
      "CTL,1250,125,1,0,1,12,250,-650,-83,83,-158,79,-612,605", &frame));
  EXPECT_EQ(frame.mcu_time_ms, 1250U);
  EXPECT_EQ(frame.sample_sequence, 125U);
  EXPECT_EQ(frame.mode, 1U);
  EXPECT_FALSE(frame.emergency_stop);
  EXPECT_TRUE(frame.command_valid);
  EXPECT_EQ(frame.command_age_ms, 12U);
  EXPECT_EQ(frame.received_linear_velocity_milli, 250);
  EXPECT_EQ(frame.received_angular_velocity_milli, -650);
  EXPECT_EQ(frame.measured_left_velocity_milli, -158);
  EXPECT_EQ(frame.pwm_left, -612);
}

TEST(Protocol, ParsesBmi088ImuFrame)
{
  cup_car_serial::Bmi088ImuFrame frame;
  ASSERT_TRUE(cup_car_serial::parse_bmi088_imu_frame(
      "IMU,1250,125,29,12,-34,56,3175", &frame));
  EXPECT_EQ(frame.mcu_time_ms, 1250U);
  EXPECT_EQ(frame.sample_sequence, 125U);
  EXPECT_EQ(frame.status, 29U);
  EXPECT_EQ(frame.gyro_x_counts, 12);
  EXPECT_EQ(frame.gyro_y_counts, -34);
  EXPECT_EQ(frame.gyro_z_counts, 56);
  EXPECT_EQ(frame.temperature_centideg_c, 3175);
}

TEST(Protocol, ParsesBmi088AttitudeFrame)
{
  cup_car_serial::Bmi088AttitudeFrame frame;
  ASSERT_TRUE(cup_car_serial::parse_bmi088_attitude_frame(
      "ATT,1250,125,29,90512,16968,345,600", &frame));
  EXPECT_EQ(frame.mcu_time_ms, 1250U);
  EXPECT_EQ(frame.sample_sequence, 125U);
  EXPECT_EQ(frame.status, 29U);
  EXPECT_EQ(frame.yaw_mdeg, 90512);
  EXPECT_EQ(frame.gyro_z_mdps, 16968);
  EXPECT_EQ(frame.bias_z_mdps, 345);
  EXPECT_EQ(frame.startup_samples, 600U);
}

TEST(Protocol, RejectsMalformedBmi088Frames)
{
  cup_car_serial::Bmi088ImuFrame imu;
  cup_car_serial::Bmi088AttitudeFrame attitude;
  EXPECT_FALSE(cup_car_serial::parse_bmi088_imu_frame(
      "IMU,1,2,300,0,0,0,0", &imu));
  EXPECT_FALSE(cup_car_serial::parse_bmi088_imu_frame(
      "IMU,1,2,1,0,0,40000,0", &imu));
  EXPECT_FALSE(cup_car_serial::parse_bmi088_attitude_frame(
      "ATT,1,2,1,0,0,0", &attitude));
}

TEST(Protocol, AcceptsNeverReceivedCommandAge)
{
  cup_car_serial::ControlTelemetryFrame frame;
  ASSERT_TRUE(cup_car_serial::parse_control_telemetry_frame(
      "CTL,0,0,0,0,0,4294967295,0,0,0,0,0,0,0,0", &frame));
  EXPECT_EQ(frame.command_age_ms, std::numeric_limits<uint32_t>::max());
  EXPECT_FALSE(frame.command_valid);
}

TEST(Protocol, RejectsInvalidFlagsModesAndFieldCounts)
{
  cup_car_serial::ControlTelemetryFrame frame;
  EXPECT_FALSE(cup_car_serial::parse_control_telemetry_frame(
      "CTL,1,2,2,0,1,3,4,5,6,7,8,9,10,11", &frame));
  EXPECT_FALSE(cup_car_serial::parse_control_telemetry_frame(
      "CTL,1,2,1,3,1,3,4,5,6,7,8,9,10,11", &frame));
  EXPECT_FALSE(cup_car_serial::parse_control_telemetry_frame("CTL,1,2", &frame));
}

TEST(Protocol, RejectsPwmOverflow)
{
  cup_car_serial::ControlTelemetryFrame frame;
  EXPECT_FALSE(cup_car_serial::parse_control_telemetry_frame(
      "CTL,1,2,1,0,1,3,4,5,6,7,8,9,32768,11", &frame));
}

TEST(Protocol, ActuatorHealthRequiresNavigationNoEstopAndValidCommand)
{
  cup_car_serial::ControlTelemetryFrame frame;
  frame.mode = 1U;
  frame.command_valid = true;
  EXPECT_TRUE(cup_car_serial::control_state_is_healthy(frame));

  frame.mode = 0U;
  EXPECT_FALSE(cup_car_serial::control_state_is_healthy(frame));
  frame.mode = 1U;
  frame.emergency_stop = true;
  EXPECT_FALSE(cup_car_serial::control_state_is_healthy(frame));
  frame.emergency_stop = false;
  frame.command_valid = false;
  EXPECT_FALSE(cup_car_serial::control_state_is_healthy(frame));
}

TEST(Protocol, ClassifiesTelemetrySequenceWithoutTreatingDuplicatesAsFresh)
{
  using cup_car_serial::SampleSequenceDisposition;

  EXPECT_EQ(
    cup_car_serial::classify_sample_sequence(10U, 100U, 11U, 110U),
    SampleSequenceDisposition::NEW_SAMPLE);
  EXPECT_EQ(
    cup_car_serial::classify_sample_sequence(10U, 100U, 10U, 110U),
    SampleSequenceDisposition::DUPLICATE);
  EXPECT_EQ(
    cup_car_serial::classify_sample_sequence(10U, 100U, 9U, 110U),
    SampleSequenceDisposition::OUT_OF_ORDER);
}

TEST(Protocol, AcceptsSequenceWrapAndRecognizesMcuRestart)
{
  using cup_car_serial::SampleSequenceDisposition;

  EXPECT_EQ(
    cup_car_serial::classify_sample_sequence(
      std::numeric_limits<uint32_t>::max(), 500U, 0U, 510U),
    SampleSequenceDisposition::NEW_SAMPLE);
  EXPECT_EQ(
    cup_car_serial::classify_sample_sequence(91679U, 916793U, 10U, 100U),
    SampleSequenceDisposition::SOURCE_RESTART);
}
