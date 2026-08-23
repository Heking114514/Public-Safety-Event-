#include <chrono>

#include "gtest/gtest.h"
#include "visual_navigation/path_tracking_controller.hpp"

TEST(PathTrackingController, PidResetRemovesAccumulatedState)
{
  visual_navigation::PathTrackingController controller;
  controller.configure(1.0, 1.0, 0.0, 0.0, 1.0, 2.0);
  const auto start = visual_navigation::PathTrackingController::Clock::now();
  controller.update_pid(0.5, -0.5, true, 0.0, start);
  const double accumulated = controller.update_pid(
    0.5, -0.5, true, 0.0, start + std::chrono::milliseconds(100));
  controller.reset_pid();
  const double reset = controller.update_pid(
    0.5, -0.5, true, 0.0, start + std::chrono::milliseconds(200));
  EXPECT_GT(accumulated, reset);
}

TEST(PathTrackingController, EstimatesObservedSpeedFromPositionWindow)
{
  visual_navigation::PathTrackingController controller;
  const auto start = visual_navigation::PathTrackingController::Clock::now();
  controller.update_observed_speed(0.0, 0.0, start, 0.2);
  controller.update_observed_speed(0.1, 0.0, start + std::chrono::milliseconds(200), 0.2);
  EXPECT_TRUE(controller.observed_speed_valid());
  EXPECT_NEAR(controller.observed_speed(), 0.5, 1.0e-6);
}
