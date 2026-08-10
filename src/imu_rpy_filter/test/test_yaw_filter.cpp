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

}  // namespace
}  // namespace imu_rpy_filter
