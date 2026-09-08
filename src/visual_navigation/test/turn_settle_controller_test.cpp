#include <chrono>

#include "gtest/gtest.h"
#include "visual_navigation/turn_settle_controller.hpp"

TEST(TurnSettleController, RequiresContinuousLowActivity)
{
  visual_navigation::TurnSettleController controller;
  controller.begin_settling();
  const auto start = visual_navigation::TurnSettleController::Clock::now();
  EXPECT_FALSE(controller.update(start, 0.2, 0.12, 0.3));
  EXPECT_FALSE(controller.update(start + std::chrono::milliseconds(100), 0.05, 0.12, 0.3));
  EXPECT_TRUE(controller.update(start + std::chrono::milliseconds(400), 0.05, 0.12, 0.3));
}

TEST(TurnSettleController, HighActivityResetsDwell)
{
  visual_navigation::TurnSettleController controller;
  controller.begin_settling();
  const auto start = visual_navigation::TurnSettleController::Clock::now();
  controller.update(start, 0.05, 0.12, 0.3);
  EXPECT_FALSE(controller.update(start + std::chrono::milliseconds(200), 0.2, 0.12, 0.3));
  EXPECT_FALSE(controller.update(start + std::chrono::milliseconds(300), 0.05, 0.12, 0.3));
}

TEST(TurnSettleController, RepeatedBeginCallsPreserveDwellAtThirtyHertz)
{
  visual_navigation::TurnSettleController controller;
  const auto start = visual_navigation::TurnSettleController::TimePoint{};

  // The navigator calls begin_settling() once per control cycle while settling.
  for (int cycle = 0; cycle < 10; ++cycle) {
    const auto now = start + std::chrono::milliseconds(33 * cycle);
    controller.begin_settling();
    EXPECT_FALSE(controller.update(now, 0.05, 0.12, 0.3));
  }

  controller.begin_settling();
  EXPECT_TRUE(controller.update(start + std::chrono::milliseconds(333),
                                0.05, 0.12, 0.3));
}
