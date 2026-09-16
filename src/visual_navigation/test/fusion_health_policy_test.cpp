#include <gtest/gtest.h>

#include "visual_navigation/fusion_health_policy.hpp"

namespace
{

const visual_navigation::FusionHealthPolicy kPolicy(
  {"FULL", "DEGRADED_NO_VISION", "DEGRADED_NO_IMU"});

TEST(FusionHealthPolicy, FullIsAllowed)
{
  const auto decision = kPolicy.Evaluate("FULL");
  EXPECT_TRUE(decision.allowed);
  EXPECT_FALSE(decision.fault);
}

TEST(FusionHealthPolicy, AllowedDegradedStateIsAllowed)
{
  const auto decision = kPolicy.Evaluate("DEGRADED_NO_VISION");
  EXPECT_TRUE(decision.allowed);
  EXPECT_FALSE(decision.fault);
}

TEST(FusionHealthPolicy, AllowedDegradedModesUseTheSameRuntimeContract)
{
  const visual_navigation::FusionHealthPolicy policy(
    {"DEGRADED_NO_VISION", "DEGRADED_NO_IMU", "DEGRADED_NO_WHEEL",
      "DEGRADED_VISION_ONLY", "DEGRADED_WHEEL_ONLY",
      "DEGRADED_VISUAL_REALIGNED"});

  for (const auto &state : {
      "DEGRADED_NO_VISION", "DEGRADED_NO_IMU", "DEGRADED_NO_WHEEL",
      "DEGRADED_VISION_ONLY", "DEGRADED_WHEEL_ONLY",
      "DEGRADED_VISUAL_REALIGNED"}) {
    const auto decision = policy.Evaluate(state);
    EXPECT_TRUE(decision.allowed);
    EXPECT_FALSE(decision.fault);
  }
}

TEST(FusionHealthPolicy, UnknownAndDisallowedStatesStop)
{
  EXPECT_FALSE(kPolicy.Evaluate("DEGRADED_WHEEL_ONLY").allowed);
  EXPECT_FALSE(kPolicy.Evaluate("UNKNOWN").allowed);
  const auto initializing = kPolicy.Evaluate("WAITING_FOR_INITIALIZATION");
  EXPECT_FALSE(initializing.allowed);
  EXPECT_FALSE(initializing.fault);
}

TEST(FusionHealthPolicy, FaultCannotBeAllowedByConfiguration)
{
  const visual_navigation::FusionHealthPolicy unsafe_configuration(
    {"FULL", "FAULT", "FAULT_CAMERA"});
  EXPECT_TRUE(unsafe_configuration.Evaluate("FAULT").fault);
  EXPECT_FALSE(unsafe_configuration.Evaluate("FAULT").allowed);
  EXPECT_TRUE(unsafe_configuration.Evaluate("FAULT_CAMERA").fault);
  EXPECT_FALSE(unsafe_configuration.Evaluate("FAULT_CAMERA").allowed);
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
