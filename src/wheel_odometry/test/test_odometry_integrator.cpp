#include <cmath>
#include <cstdint>
#include <limits>

#include "gtest/gtest.h"

#include "wheel_odometry/odometry_integrator.hpp"

namespace wheel_odometry
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

TEST(OdometryIntegratorConfig, DefaultsMatchMeasuredChassis)
{
  const IntegratorConfig config;
  EXPECT_DOUBLE_EQ(config.wheel_radius_m, 0.0302);
  EXPECT_DOUBLE_EQ(config.wheel_track_m, 0.1466);
  EXPECT_DOUBLE_EQ(config.left_encoder_counts_per_revolution, 294912.0);
  EXPECT_DOUBLE_EQ(config.right_encoder_counts_per_revolution, 294912.0);
  EXPECT_DOUBLE_EQ(config.left_distance_scale, -1.0);
  EXPECT_DOUBLE_EQ(config.right_distance_scale, -1.0);
}

IntegratorConfig test_config()
{
  // Deliberately simple fixture values make the integration math exact.
  IntegratorConfig config;
  config.wheel_radius_m = 0.0325;
  config.left_encoder_counts_per_revolution = 1925.0;
  config.right_encoder_counts_per_revolution = 1925.0;
  config.wheel_track_m = 0.254;
  config.left_distance_scale = 1.0;
  config.right_distance_scale = 1.0;
  config.max_wheel_speed_mps = 10.0;
  config.nominal_sample_period_s = 0.02;
  config.nominal_sequence_increment = 1;
  return config;
}

EncoderSample sample(uint32_t time_ms, uint32_t sequence, int32_t left, int32_t right)
{
  return {time_ms, sequence, left, right};
}
}  // namespace

TEST(OdometryIntegrator, InitializesThenIntegratesStraightMotion)
{
  OdometryIntegrator integrator(test_config());
  EXPECT_EQ(integrator.update(sample(1000, 10, 0, 0)).status, UpdateStatus::kInitialized);

  const UpdateResult result = integrator.update(sample(2000, 11, 1925, 1925));
  const double expected_distance = 2.0 * kPi * 0.0325;
  EXPECT_TRUE(result.publish);
  EXPECT_NEAR(integrator.state().x_m, expected_distance, 1.0e-12);
  EXPECT_NEAR(integrator.state().y_m, 0.0, 1.0e-12);
  EXPECT_NEAR(integrator.state().yaw_rad, 0.0, 1.0e-12);
  EXPECT_NEAR(integrator.state().linear_velocity_mps, expected_distance, 1.0e-12);
}

TEST(OdometryIntegrator, UsesIndependentEncoderCountsPerRevolution)
{
  IntegratorConfig config = test_config();
  config.wheel_radius_m = 0.05;
  config.left_encoder_counts_per_revolution = 1000.0;
  config.right_encoder_counts_per_revolution = 2000.0;
  OdometryIntegrator integrator(config);
  integrator.update(sample(0, 1, 0, 0));

  ASSERT_TRUE(integrator.update(sample(1000, 2, 1000, 2000)).publish);
  EXPECT_NEAR(integrator.state().x_m, 2.0 * kPi * 0.05, 1.0e-12);
  EXPECT_NEAR(integrator.state().y_m, 0.0, 1.0e-12);
  EXPECT_NEAR(integrator.state().yaw_rad, 0.0, 1.0e-12);
}

TEST(OdometryIntegrator, SignedDistanceScaleHandlesEncoderPolarity)
{
  IntegratorConfig config = test_config();
  config.left_distance_scale = -1.0;
  config.right_distance_scale = -1.0;
  OdometryIntegrator integrator(config);
  integrator.update(sample(0, 1, 0, 0));

  const UpdateResult result = integrator.update(sample(1000, 2, -1925, -1925));
  const double expected_distance = 2.0 * kPi * 0.0325;
  ASSERT_TRUE(result.publish);
  EXPECT_NEAR(integrator.state().x_m, expected_distance, 1.0e-12);
  EXPECT_NEAR(integrator.state().linear_velocity_mps, expected_distance, 1.0e-12);
}

TEST(OdometryIntegrator, UsesMidpointHeadingForArc)
{
  OdometryIntegrator integrator(test_config());
  integrator.update(sample(0, 1, 0, 0));
  integrator.update(sample(1000, 2, 0, 1000));

  const double right_distance = 1000.0 * 2.0 * kPi * 0.0325 / 1925.0;
  const double center_distance = 0.5 * right_distance;
  const double delta_yaw = right_distance / 0.254;
  EXPECT_NEAR(integrator.state().x_m, center_distance * std::cos(delta_yaw / 2.0), 1.0e-12);
  EXPECT_NEAR(integrator.state().y_m, center_distance * std::sin(delta_yaw / 2.0), 1.0e-12);
  EXPECT_NEAR(integrator.state().yaw_rad, delta_yaw, 1.0e-12);
}

TEST(OdometryIntegrator, ModelsTurnSlipWithoutChangingPhysicalWheelTrack)
{
  IntegratorConfig config = test_config();
  config.yaw_slip_scale = 0.61;
  OdometryIntegrator integrator(config);
  integrator.update(sample(0, 1, 0, 0));
  integrator.update(sample(1000, 2, -1000, 1000));

  const double wheel_distance = 1000.0 * 2.0 * kPi * 0.0325 / 1925.0;
  const double expected_yaw = 2.0 * wheel_distance / 0.254 * 0.61;
  EXPECT_NEAR(integrator.state().yaw_rad, expected_yaw, 1.0e-12);
  EXPECT_NEAR(integrator.state().x_m, 0.0, 1.0e-12);
  EXPECT_NEAR(integrator.state().y_m, 0.0, 1.0e-12);
}

TEST(OdometryIntegrator, RejectsInvalidTurnSlipScale)
{
  IntegratorConfig config = test_config();
  config.yaw_slip_scale = 0.0;
  EXPECT_THROW(OdometryIntegrator integrator(config), std::invalid_argument);
}

TEST(OdometryIntegrator, IntegratesAcrossDroppedMessages)
{
  OdometryIntegrator integrator(test_config());
  integrator.update(sample(100, 20, 100, 100));
  const UpdateResult result = integrator.update(sample(400, 24, 400, 400));

  EXPECT_TRUE(result.publish);
  EXPECT_EQ(result.sequence_delta, 4U);
  EXPECT_EQ(result.left_delta_ticks, 300);
  EXPECT_NEAR(result.dt_s, 0.3, 1.0e-12);
  EXPECT_EQ(integrator.statistics().sequence_gap_events, 1U);
  EXPECT_EQ(integrator.statistics().missing_samples, 3U);
  EXPECT_GT(integrator.uncertainty().pose_xy_variance, test_config().pose_xy_variance);
  EXPECT_GT(
    integrator.uncertainty().twist_linear_variance,
    test_config().twist_linear_variance);
}

TEST(OdometryIntegrator, SupportsMcuSequenceStrideWithoutFalseMissingSamples)
{
  IntegratorConfig config = test_config();
  config.nominal_sample_period_s = 0.05;
  config.nominal_sequence_increment = 5;
  OdometryIntegrator integrator(config);
  integrator.update(sample(100, 20, 100, 100));

  const UpdateResult normal = integrator.update(sample(150, 25, 150, 150));
  ASSERT_TRUE(normal.publish);
  EXPECT_EQ(integrator.statistics().sequence_gap_events, 0U);
  EXPECT_EQ(integrator.statistics().missing_samples, 0U);

  const UpdateResult one_report_missing = integrator.update(sample(250, 35, 250, 250));
  ASSERT_TRUE(one_report_missing.publish);
  EXPECT_EQ(integrator.statistics().sequence_gap_events, 1U);
  EXPECT_EQ(integrator.statistics().missing_samples, 1U);
}

TEST(OdometryIntegrator, AcceptsSequenceAndMcuTimeWrap)
{
  OdometryIntegrator integrator(test_config());
  integrator.update(sample(0xfffffff0U, 0xfffffffeU, 100, 100));
  const UpdateResult result = integrator.update(sample(0x00000018U, 1U, 120, 120));

  EXPECT_TRUE(result.publish);
  EXPECT_EQ(result.sequence_delta, 3U);
  EXPECT_NEAR(result.dt_s, 0.040, 1.0e-12);
}

TEST(OdometryIntegrator, AcceptsSignedEncoderCounterWrap)
{
  OdometryIntegrator integrator(test_config());
  integrator.update(sample(100, 1, std::numeric_limits<int32_t>::max() - 4, 0));
  const UpdateResult result = integrator.update(
    sample(200, 2, std::numeric_limits<int32_t>::min() + 5, 0));

  EXPECT_TRUE(result.publish);
  EXPECT_EQ(result.left_delta_ticks, 10);
}

TEST(OdometryIntegrator, RebasesOnMcuRestartWithoutResettingPose)
{
  OdometryIntegrator integrator(test_config());
  integrator.update(sample(1000, 100, 0, 0));
  integrator.update(sample(1100, 101, 10, 10));
  const double x_before_restart = integrator.state().x_m;

  EXPECT_EQ(
    integrator.update(sample(20, 1, 0, 0)).status,
    UpdateStatus::kRebasedSequenceRegression);
  EXPECT_DOUBLE_EQ(integrator.state().x_m, x_before_restart);
  EXPECT_EQ(integrator.statistics().rebase_count, 1U);
  EXPECT_GT(integrator.uncertainty().pose_xy_variance, test_config().pose_xy_variance);
  EXPECT_TRUE(integrator.update(sample(120, 2, 10, 10)).publish);
  EXPECT_GT(integrator.state().x_m, x_before_restart);
}

TEST(OdometryIntegrator, RebasesOnTimeRegressionOrNonPositiveDt)
{
  OdometryIntegrator integrator(test_config());
  integrator.update(sample(1000, 10, 0, 0));
  EXPECT_EQ(
    integrator.update(sample(900, 11, 10, 10)).status,
    UpdateStatus::kRebasedTimeRegression);
  EXPECT_EQ(
    integrator.update(sample(900, 12, 20, 20)).status,
    UpdateStatus::kRebasedInvalidDt);
  EXPECT_TRUE(integrator.update(sample(1000, 13, 30, 30)).publish);
}

TEST(OdometryIntegrator, RejectsImplausibleTickJumpAndRecovers)
{
  IntegratorConfig config = test_config();
  config.max_wheel_speed_mps = 0.5;
  OdometryIntegrator integrator(config);
  integrator.update(sample(100, 1, 0, 0));

  EXPECT_EQ(
    integrator.update(sample(200, 2, 10000, 10000)).status,
    UpdateStatus::kRebasedTickJump);
  EXPECT_DOUBLE_EQ(integrator.state().x_m, 0.0);
  EXPECT_EQ(integrator.statistics().tick_jump_count, 1U);
  EXPECT_EQ(integrator.statistics().rebase_count, 1U);
  EXPECT_TRUE(integrator.update(sample(300, 3, 10010, 10010)).publish);
  EXPECT_GT(integrator.state().x_m, 0.0);
}

TEST(OdometryIntegrator, IgnoresDuplicateSequence)
{
  OdometryIntegrator integrator(test_config());
  integrator.update(sample(100, 1, 0, 0));
  EXPECT_EQ(
    integrator.update(sample(200, 1, 100, 100)).status,
    UpdateStatus::kDuplicate);
  EXPECT_TRUE(integrator.update(sample(300, 2, 20, 20)).publish);
  EXPECT_EQ(integrator.state().linear_velocity_mps > 0.0, true);
  EXPECT_EQ(integrator.statistics().duplicate_samples, 1U);
}

TEST(OdometryIntegrator, PoseVarianceGrowsOverLongTravel)
{
  IntegratorConfig config = test_config();
  config.nominal_sample_period_s = 0.1;
  OdometryIntegrator integrator(config);
  integrator.update(sample(0, 1, 0, 0));

  double previous_xy_variance = integrator.uncertainty().pose_xy_variance;
  double previous_yaw_variance = integrator.uncertainty().pose_yaw_variance;
  for (int index = 1; index <= 100; ++index) {
    ASSERT_TRUE(integrator.update(sample(
      static_cast<uint32_t>(index * 100), static_cast<uint32_t>(index + 1),
      index * 10, index * 10)).publish);
    EXPECT_GT(integrator.uncertainty().pose_xy_variance, previous_xy_variance);
    EXPECT_GT(integrator.uncertainty().pose_yaw_variance, previous_yaw_variance);
    previous_xy_variance = integrator.uncertainty().pose_xy_variance;
    previous_yaw_variance = integrator.uncertainty().pose_yaw_variance;
  }
  EXPECT_GT(integrator.statistics().accumulated_wheel_travel_m, 0.0);
  EXPECT_EQ(integrator.statistics().integrated_samples, 100U);
}

TEST(OdometryIntegrator, TurningAndWheelDifferenceRaiseYawUncertainty)
{
  IntegratorConfig config = test_config();
  config.nominal_sample_period_s = 1.0;
  OdometryIntegrator straight(config);
  OdometryIntegrator turning(config);
  straight.update(sample(0, 1, 0, 0));
  turning.update(sample(0, 1, 0, 0));

  ASSERT_TRUE(straight.update(sample(1000, 2, 100, 100)).publish);
  ASSERT_TRUE(turning.update(sample(1000, 2, 0, 200)).publish);

  EXPECT_GT(
    turning.uncertainty().pose_yaw_variance,
    straight.uncertainty().pose_yaw_variance);
  EXPECT_GT(
    turning.uncertainty().twist_yaw_variance,
    straight.uncertainty().twist_yaw_variance);
  EXPECT_GT(turning.statistics().accumulated_abs_turn_rad, 0.0);
}

TEST(OdometryIntegrator, SamplingDelayRaisesTwistButNotPoseWithoutMotion)
{
  IntegratorConfig config = test_config();
  config.nominal_sample_period_s = 0.02;
  OdometryIntegrator nominal(config);
  OdometryIntegrator delayed(config);
  nominal.update(sample(0, 1, 0, 0));
  delayed.update(sample(0, 1, 0, 0));

  ASSERT_TRUE(nominal.update(sample(20, 2, 0, 0)).publish);
  ASSERT_TRUE(delayed.update(sample(200, 2, 0, 0)).publish);
  EXPECT_GT(
    delayed.uncertainty().twist_linear_variance,
    nominal.uncertainty().twist_linear_variance);
  EXPECT_DOUBLE_EQ(
    delayed.uncertainty().pose_xy_variance,
    nominal.uncertainty().pose_xy_variance);
}

}  // namespace wheel_odometry
