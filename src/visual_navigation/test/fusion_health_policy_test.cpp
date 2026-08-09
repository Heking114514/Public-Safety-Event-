#include <gtest/gtest.h>

#include <vector>

#include "visual_navigation/fusion_health_policy.hpp"

namespace
{

const visual_navigation::FusionHealthPolicy kPolicy(
  {"FULL", "DEGRADED_NO_VISION", "DEGRADED_NO_IMU"}, 0.4);

TEST(FusionHealthPolicy, FullRunsAtNormalSpeed)
{
  const auto decision = kPolicy.Evaluate("FULL");
  EXPECT_TRUE(decision.allowed);
  EXPECT_FALSE(decision.fault);
  EXPECT_DOUBLE_EQ(decision.speed_scale, 1.0);
}

TEST(FusionHealthPolicy, AllowedDegradedStateIsSpeedLimited)
{
  const auto decision = kPolicy.Evaluate("DEGRADED_NO_VISION");
  EXPECT_TRUE(decision.allowed);
  EXPECT_FALSE(decision.fault);
  EXPECT_DOUBLE_EQ(decision.speed_scale, 0.4);
}

TEST(FusionHealthPolicy, AppliesPerModeSpeedLimits)
{
  visual_navigation::FusionSpeedScales scales;
  scales.no_vision = 0.5;
  scales.no_imu = 0.65;
  scales.no_wheel = 0.6;
  scales.vision_only = 0.4;
  scales.wheel_only = 0.25;
  scales.visual_realigned = 0.45;
  const visual_navigation::FusionHealthPolicy policy(
    {"DEGRADED_NO_VISION", "DEGRADED_NO_IMU", "DEGRADED_NO_WHEEL",
      "DEGRADED_VISION_ONLY", "DEGRADED_WHEEL_ONLY", "DEGRADED_VISUAL_REALIGNED"},
    scales);

  EXPECT_DOUBLE_EQ(policy.Evaluate("DEGRADED_NO_VISION").speed_scale, 0.5);
  EXPECT_DOUBLE_EQ(policy.Evaluate("DEGRADED_NO_IMU").speed_scale, 0.65);
  EXPECT_DOUBLE_EQ(policy.Evaluate("DEGRADED_NO_WHEEL").speed_scale, 0.6);
  EXPECT_DOUBLE_EQ(policy.Evaluate("DEGRADED_VISION_ONLY").speed_scale, 0.4);
  EXPECT_DOUBLE_EQ(policy.Evaluate("DEGRADED_WHEEL_ONLY").speed_scale, 0.25);
  EXPECT_DOUBLE_EQ(policy.Evaluate("DEGRADED_VISUAL_REALIGNED").speed_scale, 0.45);
}

TEST(FusionHealthPolicy, UnknownAndDisallowedStatesStop)
{
  EXPECT_FALSE(kPolicy.Evaluate("DEGRADED_WHEEL_ONLY").allowed);
  EXPECT_FALSE(kPolicy.Evaluate("UNKNOWN").allowed);
}

TEST(FusionHealthPolicy, FaultCannotBeAllowedByConfiguration)
{
  const visual_navigation::FusionHealthPolicy unsafe_configuration(
    {"FULL", "FAULT", "FAULT_CAMERA"}, 0.5);
  EXPECT_TRUE(unsafe_configuration.Evaluate("FAULT").fault);
  EXPECT_FALSE(unsafe_configuration.Evaluate("FAULT").allowed);
  EXPECT_TRUE(unsafe_configuration.Evaluate("FAULT_CAMERA").fault);
  EXPECT_FALSE(unsafe_configuration.Evaluate("FAULT_CAMERA").allowed);
}

TEST(FusionHealthPolicy, SpeedScaleIsClamped)
{
  const visual_navigation::FusionHealthPolicy too_high({"DEGRADED_NO_IMU"}, 2.0);
  const visual_navigation::FusionHealthPolicy negative({"DEGRADED_NO_IMU"}, -1.0);
  EXPECT_DOUBLE_EQ(too_high.Evaluate("DEGRADED_NO_IMU").speed_scale, 1.0);
  EXPECT_DOUBLE_EQ(negative.Evaluate("DEGRADED_NO_IMU").speed_scale, 0.0);
}

TEST(FusionHealthPolicy, StartRequiresAllCurrentInputsToBeValid)
{
  const auto full = kPolicy.Evaluate("FULL");
  EXPECT_TRUE(visual_navigation::LocalizationCanStart(true, full, true));
  EXPECT_FALSE(visual_navigation::LocalizationCanStart(false, full, true));
  EXPECT_FALSE(visual_navigation::LocalizationCanStart(true, {}, true));
  EXPECT_FALSE(visual_navigation::LocalizationCanStart(true, full, false));
}

TEST(FusionHealthPolicy, RuntimeLossLatchesButStartupWaitDoesNot)
{
  const auto full = kPolicy.Evaluate("FULL");
  const auto degraded = kPolicy.Evaluate("DEGRADED_NO_VISION");
  EXPECT_FALSE(
    visual_navigation::ShouldLatchFusedLocalizationLoss(false, false, {}));
  EXPECT_TRUE(
    visual_navigation::ShouldLatchFusedLocalizationLoss(true, false, full));
  EXPECT_TRUE(
    visual_navigation::ShouldLatchFusedLocalizationLoss(true, true, {}));
  EXPECT_FALSE(
    visual_navigation::ShouldLatchFusedLocalizationLoss(true, true, degraded));
}

}  // namespace
