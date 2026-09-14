#include <chrono>

#include "gtest/gtest.h"
#include "visual_navigation/waypoint_brake_controller.hpp"

TEST(WaypointBrakeController, RequiresDwellAndMinimumStopTime)
{
  visual_navigation::WaypointBrakeController controller;
  controller.configure(0.03, 0.10, 0.20, 0.60);
  const auto start = visual_navigation::WaypointBrakeController::Clock::now();
  controller.begin(start);
  EXPECT_FALSE(controller.check(start + std::chrono::milliseconds(50), true, 0.0));
  EXPECT_TRUE(controller.check(start + std::chrono::milliseconds(250), true, 0.0));
}

TEST(WaypointBrakeController, TimesOutWhenMotionDoesNotSettle)
{
  visual_navigation::WaypointBrakeController controller;
  controller.configure(0.03, 0.10, 0.20, 0.60);
  const auto start = visual_navigation::WaypointBrakeController::Clock::now();
  controller.begin(start);
  EXPECT_FALSE(controller.check(start + std::chrono::milliseconds(600), true, 0.2));
  EXPECT_TRUE(controller.timed_out());
}

TEST(WaypointBrakeController, TimeoutCanRestartAndThenConfirmStop)
{
  visual_navigation::WaypointBrakeController controller;
  controller.configure(0.03, 0.10, 0.20, 0.60);
  const auto start = visual_navigation::WaypointBrakeController::Clock::now();
  controller.begin(start);
  EXPECT_FALSE(controller.check(start + std::chrono::milliseconds(600), true, 0.2));
  ASSERT_TRUE(controller.timed_out());

  const auto retry = start + std::chrono::milliseconds(600);
  controller.begin(retry);
  EXPECT_FALSE(controller.timed_out());
  EXPECT_FALSE(controller.check(
    retry + std::chrono::milliseconds(50), true, 0.0));
  EXPECT_TRUE(controller.check(retry + std::chrono::milliseconds(250), true, 0.0));
}

TEST(WaypointBrakeController, FreshWheelStopOverridesVisualPoseJitter)
{
  const auto speed = visual_navigation::SelectWaypointBrakeSpeed(
    true, 0.02, 0.30, 0.0, 0.0, true, 0.036);
  EXPECT_TRUE(speed.valid);
  EXPECT_TRUE(speed.from_wheel_telemetry);
  EXPECT_DOUBLE_EQ(speed.absolute_speed, 0.0);
}

TEST(WaypointBrakeController, StaleOrPreBrakeTelemetryFallsBackToPoseSpeed)
{
  const auto stale = visual_navigation::SelectWaypointBrakeSpeed(
    true, 0.31, 0.30, 0.0, 0.0, true, 0.04);
  EXPECT_TRUE(stale.valid);
  EXPECT_FALSE(stale.from_wheel_telemetry);
  EXPECT_DOUBLE_EQ(stale.absolute_speed, 0.04);

  const auto old_sample = visual_navigation::SelectWaypointBrakeSpeed(
    false, 0.01, 0.30, 0.0, 0.0, true, 0.05);
  EXPECT_TRUE(old_sample.valid);
  EXPECT_FALSE(old_sample.from_wheel_telemetry);
  EXPECT_DOUBLE_EQ(old_sample.absolute_speed, 0.05);
}

TEST(WaypointBrakeController, MovingWheelsCannotBeHiddenByStationaryPose)
{
  const auto speed = visual_navigation::SelectWaypointBrakeSpeed(
    true, 0.02, 0.30, 0.12, -0.15, true, 0.0);
  EXPECT_TRUE(speed.valid);
  EXPECT_TRUE(speed.from_wheel_telemetry);
  EXPECT_DOUBLE_EQ(speed.absolute_speed, 0.15);
}

TEST(WaypointBrakeController, RequiresTwoIndependentStoppedTelemetrySamples)
{
  const double one_sample = visual_navigation::IndependentWheelStopEvidence(
    0.0, 0.0, 0.0, 0.0, 0.03, 1, 2);
  const double two_samples = visual_navigation::IndependentWheelStopEvidence(
    0.0, 0.0, 0.0, 0.0, 0.03, 2, 2);
  EXPECT_GT(one_sample, 0.03);
  EXPECT_DOUBLE_EQ(two_samples, 0.0);
}

TEST(WaypointBrakeController, RejectsDuplicateTelemetryButAcceptsWrapAndRestart)
{
  EXPECT_FALSE(visual_navigation::ControlTelemetrySampleIsFresh(
    10U, 100U, 10U, 110U));
  EXPECT_FALSE(visual_navigation::ControlTelemetrySampleIsFresh(
    10U, 100U, 9U, 110U));
  EXPECT_TRUE(visual_navigation::ControlTelemetrySampleIsFresh(
    10U, 100U, 11U, 110U));
  EXPECT_TRUE(visual_navigation::ControlTelemetrySampleIsFresh(
    std::numeric_limits<std::uint32_t>::max(), 500U, 0U, 510U));
  EXPECT_TRUE(visual_navigation::ControlTelemetrySampleIsFresh(
    91679U, 916793U, 10U, 100U));
}

TEST(WaypointBrakeController, FallbackRequiresBothStablePoseAndFreshImu)
{
  EXPECT_FALSE(visual_navigation::BrakeFallbackMayConfirmStop(
    1, 2, true, 0.01, 0.08, true, 0.01, 0.12));
  EXPECT_FALSE(visual_navigation::BrakeFallbackMayConfirmStop(
    2, 2, true, 0.09, 0.08, true, 0.01, 0.12));
  EXPECT_FALSE(visual_navigation::BrakeFallbackMayConfirmStop(
    2, 2, true, 0.01, 0.08, false, 0.01, 0.12));
  EXPECT_TRUE(visual_navigation::BrakeFallbackMayConfirmStop(
    2, 2, true, 0.01, 0.08, true, 0.01, 0.12));
}
