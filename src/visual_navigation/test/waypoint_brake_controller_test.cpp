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
