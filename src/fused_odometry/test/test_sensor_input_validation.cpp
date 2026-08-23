#include <gtest/gtest.h>

#include "fused_odometry/sensor_input_validation.hpp"

using fused_odometry::StampValidation;
using fused_odometry::ValidateMeasurementStamp;

TEST(SensorInputValidation, AcceptsIncreasingFreshStamp)
{
  EXPECT_EQ(ValidateMeasurementStamp(2'000'000'000, true, 1'000'000'000,
    2'100'000'000, 0.5, 0.2), StampValidation::kAccepted);
}

TEST(SensorInputValidation, RejectsUnsetNonMonotonicStaleAndFuture)
{
  EXPECT_EQ(ValidateMeasurementStamp(0, false, 0, 0, 0.5, 0.2), StampValidation::kUnset);
  EXPECT_EQ(ValidateMeasurementStamp(1, true, 1, 0, 0.5, 0.2), StampValidation::kNonMonotonic);
  EXPECT_EQ(ValidateMeasurementStamp(1'000'000'000, false, 0,
    2'000'000'000, 0.5, 0.2), StampValidation::kStale);
  EXPECT_EQ(ValidateMeasurementStamp(2'000'000'000, false, 0,
    1'000'000'000, 0.2, 0.2), StampValidation::kFuture);
}

TEST(SensorInputValidation, SharesFrameAndFiniteValueRules)
{
  EXPECT_TRUE(fused_odometry::FramePairMatches("odom", "base_link", "odom", "base_link"));
  EXPECT_FALSE(fused_odometry::FramePairMatches("map", "base_link", "odom", "base_link"));
  EXPECT_TRUE(fused_odometry::FiniteAndWithin(-0.5, 1.0));
  EXPECT_FALSE(fused_odometry::FiniteAndWithin(1.5, 1.0));
}
