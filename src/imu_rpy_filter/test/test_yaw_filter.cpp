#include <cmath>

#include "gtest/gtest.h"

#include "imu_rpy_filter/yaw_filter.hpp"

namespace imu_rpy_filter
{
namespace
{

double radians(double degrees)
{
  return degrees * kPi / 180.0;
}

TEST(StationaryDetector, RejectsShortSpikeButExitsOnSustainedMotion)
{
  StationaryDetector detector(0.5, 0.15, 0.20, 0.35, 0.02, 0.05);

  EXPECT_FALSE(detector.update(0.0, 0.01, 0.001));
  EXPECT_TRUE(detector.has_candidate());
  EXPECT_TRUE(detector.update(0.51, 0.01, 0.001));
  EXPECT_TRUE(detector.active());

  EXPECT_TRUE(detector.update(0.60, 0.01, 0.10));
  EXPECT_TRUE(detector.exit_pending());
  EXPECT_TRUE(detector.update(0.68, 0.01, 0.001));
  EXPECT_FALSE(detector.exit_pending());
  EXPECT_EQ(detector.rejected_transient_count(), 1U);

  EXPECT_TRUE(detector.update(0.70, 0.01, 0.10));
  EXPECT_FALSE(detector.update(0.86, 0.01, 0.10));
  EXPECT_FALSE(detector.active());
}

TEST(StationaryDetector, RequiresFreshDwellAfterCandidateBreaks)
{
  StationaryDetector detector(0.5, 0.15, 0.20, 0.35, 0.02, 0.05);

  EXPECT_FALSE(detector.update(0.0, 0.01, 0.001));
  EXPECT_FALSE(detector.update(0.4, 0.01, 0.03));
  EXPECT_FALSE(detector.has_candidate());
  EXPECT_FALSE(detector.update(0.5, 0.01, 0.001));
  EXPECT_TRUE(detector.update(1.01, 0.01, 0.001));
}

TEST(YawBiasKalman, DirectZeroRateUpdatesEstimateStationaryBias)
{
  YawBiasKalman filter(radians(0.6), radians(0.03));
  const double measured_bias = radians(0.45);
  const double dt = 0.005;

  for (int sample = 0; sample < 2000; ++sample) {
    filter.predict(measured_bias, dt);
    if (sample % 10 == 0) {
      filter.update(0.0, radians(0.5));
      filter.update_bias(measured_bias, radians(0.15));
    }
  }

  EXPECT_NEAR(filter.bias(), measured_bias, radians(0.02));
  EXPECT_NEAR(filter.yaw(), 0.0, radians(0.05));
  EXPECT_LT(filter.bias_variance(), std::pow(radians(0.05), 2));
}

TEST(YawBiasKalman, HoldingInternalStationaryAnchorDoesNotShrinkCovariance)
{
  YawBiasKalman filter(radians(0.6), radians(0.03));
  filter.predict(radians(0.4), 0.1);
  const double variance_before = filter.yaw_variance();

  filter.hold_yaw(0.0);

  EXPECT_DOUBLE_EQ(filter.yaw(), 0.0);
  EXPECT_DOUBLE_EQ(filter.yaw_variance(), variance_before);
}

TEST(ExternalMotionDiagnostics, DriftingWheelCannotBlockImuStationaryDetection)
{
  const auto observation = collect_external_motion_diagnostics(
    true, true, 0.0, 0.0, 0.08, 0.0, 0.02, 0.03, 0.02, 0.05);
  EXPECT_FALSE(observation.stationary);
  EXPECT_TRUE(observation.moving);

  StationaryDetector detector(0.5, 0.15, 0.20, 0.35, 0.02, 0.05);
  EXPECT_FALSE(detector.update(0.0, 0.01, 0.004));
  EXPECT_TRUE(detector.update(0.51, 0.01, 0.004));
}

TEST(ExternalMotionDiagnostics, WheelDeadzoneDoesNotSuppressCommandedTurnRate)
{
  const auto observation = collect_external_motion_diagnostics(
    true, true, 0.0, 0.4, 0.0, 0.0, 0.02, 0.03, 0.02, 0.05);
  EXPECT_FALSE(observation.stationary);
  EXPECT_FALSE(observation.moving);
  EXPECT_NEAR(bias_corrected_yaw_rate(0.42, 0.02), 0.40, 1e-12);
}

TEST(ExternalMotionDiagnostics, ZeroWheelCannotHideUncommandedRealRotation)
{
  const auto observation = collect_external_motion_diagnostics(
    true, true, 0.0, 0.0, 0.0, 0.0, 0.02, 0.03, 0.02, 0.05);
  ASSERT_TRUE(observation.stationary);

  StationaryDetector detector(0.5, 0.15, 0.20, 0.35, 0.02, 0.05);
  EXPECT_FALSE(detector.update(0.0, 0.01, 0.002));
  ASSERT_TRUE(detector.update(0.51, 0.01, 0.002));
  EXPECT_TRUE(detector.update(0.52, 0.01, 0.40));
  EXPECT_TRUE(detector.exit_pending());

  // The debounce may still report stationary, but the measured turn remains
  // visible to downstream fusion immediately and uses moving uncertainty.
  EXPECT_NEAR(bias_corrected_yaw_rate(0.40, 0.01), 0.39, 1e-12);
  EXPECT_DOUBLE_EQ(yaw_rate_variance(
      detector.active() || detector.has_candidate(), detector.exit_pending(),
      0.01, 0.25),
    0.25);
}

TEST(YawRateOutput, StationaryNoiseIsBiasCorrectedRatherThanHardZeroed)
{
  EXPECT_NEAR(bias_corrected_yaw_rate(0.008, 0.006), 0.002, 1e-12);
  EXPECT_NE(bias_corrected_yaw_rate(0.008, 0.006), 0.0);
  EXPECT_DOUBLE_EQ(yaw_rate_variance(true, false, 0.01, 0.25), 0.01);
}

}  // namespace
}  // namespace imu_rpy_filter
