#include "visual_navigation/obstacle_recovery.hpp"

#include <chrono>

#include "gtest/gtest.h"

namespace {

using visual_navigation::ObstacleRecoveryAction;
using visual_navigation::ObstacleRecoveryConfig;
using visual_navigation::ObstacleRecoveryController;
using visual_navigation::RecoveryPoint;

ObstacleRecoveryController::TimePoint At(double seconds) {
  return ObstacleRecoveryController::TimePoint{} +
         std::chrono::duration_cast<
             ObstacleRecoveryController::Clock::duration>(
             std::chrono::duration<double>(seconds));
}

TEST(ObstacleClassification, IgnoresKnownWallBeyondPlannedTurn) {
  EXPECT_FALSE(visual_navigation::UnexpectedObstacle(
      false, true, 0.20, true, 0.30, 0.096, 0.0717, 0.05));
  EXPECT_FALSE(visual_navigation::UnexpectedObstacle(
      true, true, 0.30, true, 0.25, 0.096, 0.0717, 0.05));

  EXPECT_TRUE(visual_navigation::UnexpectedObstacle(true, true, 0.10, true,
                                                    0.40, 0.096, 0.0717, 0.05));
  EXPECT_TRUE(visual_navigation::UnexpectedObstacle(true, false, 0.30, true,
                                                    0.25, 0.096, 0.0717, 0.05));
  EXPECT_TRUE(visual_navigation::UnexpectedObstacle(true, true, 0.30, false,
                                                    0.25, 0.096, 0.0717, 0.05));
}

TEST(ObstacleRecovery, ReversesRecordedTraceToSelectedAnchor) {
  ObstacleRecoveryConfig config;
  config.breadcrumb_spacing = 0.10;
  config.reverse_lookahead = 0.10;
  config.target_tolerance = 0.06;
  config.anchor_tolerance = 0.05;
  ObstacleRecoveryController controller(config);
  controller.record_pose(0.0, 0.0);
  controller.record_pose(0.05, 0.0); // Below spacing; deliberately omitted.
  controller.record_pose(0.20, 0.0);
  controller.record_pose(0.40, 0.0);
  controller.record_pose(0.60, 0.0);

  ASSERT_TRUE(controller.trigger(0.60, 0.0, 0.20, 0.0));
  EXPECT_EQ(controller.breadcrumb_count(), 3U);
  EXPECT_NEAR(controller.anchor().x, 0.20, 1.0e-12);
  controller.confirm_stopped(0.60, 0.0, At(0.2));
  ASSERT_TRUE(controller.reversing());

  RecoveryPoint target;
  ASSERT_TRUE(controller.reverse_target(target));
  EXPECT_LT(target.x, 0.60);
  const auto command = visual_navigation::ComputeReverseMotionCommand(
      0.60, 0.0, 0.0, target, 0.10, 1.2, 0.0, 0.4, 0.3, 0.7);
  ASSERT_TRUE(command.valid);
  EXPECT_NEAR(command.linear_velocity, -0.10, 1.0e-12);
  EXPECT_NEAR(command.angular_velocity, 0.0, 1.0e-12);

  const auto complete = controller.update_reverse(0.20, 0.0, At(1.0));
  EXPECT_EQ(complete.action, ObstacleRecoveryAction::REPLAN_REQUIRED);
  EXPECT_TRUE(controller.replan_required());
}

TEST(ObstacleRecovery, IgnoresOneLargeFusionJumpInBreadcrumbTrace) {
  ObstacleRecoveryConfig config;
  config.breadcrumb_spacing = 0.05;
  config.maximum_breadcrumb_step = 0.25;
  ObstacleRecoveryController controller(config);
  controller.record_pose(0.0, 0.0);
  controller.record_pose(0.10, 0.0);
  controller.record_pose(2.0, 1.0); // Isolated localization discontinuity.
  controller.record_pose(0.20, 0.0);

  EXPECT_EQ(controller.breadcrumb_count(), 3U);
  ASSERT_TRUE(controller.trigger(0.20, 0.0, 0.0, 0.0));
  EXPECT_EQ(controller.breadcrumb_count(), 3U);
  EXPECT_NEAR(controller.anchor().x, 0.0, 1.0e-12);
}

TEST(ObstacleRecovery, NoProgressStopsBeforeRetryThenFaults) {
  ObstacleRecoveryConfig config;
  config.no_progress_timeout = 1.0;
  config.attempt_timeout = 10.0;
  config.maximum_recovery_attempts = 1;
  ObstacleRecoveryController controller(config);
  controller.record_pose(0.0, 0.0);
  controller.record_pose(1.0, 0.0);
  ASSERT_TRUE(controller.trigger(1.0, 0.0, 0.0, 0.0));
  controller.confirm_stopped(1.0, 0.0, At(0.0));

  auto decision = controller.update_reverse(1.0, 0.0, At(1.1));
  EXPECT_EQ(decision.action, ObstacleRecoveryAction::RETRY_BRAKING);
  EXPECT_TRUE(controller.braking());
  EXPECT_EQ(decision.recovery_attempt, 1);

  controller.confirm_stopped(1.0, 0.0, At(1.2));
  decision = controller.update_reverse(1.0, 0.0, At(2.3));
  EXPECT_EQ(decision.action, ObstacleRecoveryAction::FAULT_NO_PROGRESS);
  EXPECT_TRUE(controller.faulted());
}

TEST(ObstacleRecovery, RetryKeepsTargetTowardAnchorAfterPartialRetreat) {
  ObstacleRecoveryConfig config;
  config.breadcrumb_spacing = 0.10;
  config.reverse_lookahead = 0.01;
  config.no_progress_timeout = 1.0;
  config.attempt_timeout = 10.0;
  config.maximum_recovery_attempts = 1;
  ObstacleRecoveryController controller(config);
  for (double x : {0.0, 0.2, 0.4, 0.6, 0.8, 1.0})
    controller.record_pose(x, 0.0);
  ASSERT_TRUE(controller.trigger(1.0, 0.0, 0.0, 0.0));
  controller.confirm_stopped(1.0, 0.0, At(0.0));

  // The first attempt reached x=0.55 before becoming stuck.
  controller.update_reverse(0.55, 0.0, At(0.2));
  const auto retry = controller.update_reverse(0.55, 0.0, At(1.3));
  ASSERT_EQ(retry.action, ObstacleRecoveryAction::RETRY_BRAKING);
  controller.confirm_stopped(0.55, 0.0, At(1.4));

  RecoveryPoint target;
  ASSERT_TRUE(controller.reverse_target(target));
  EXPECT_LT(target.x, 0.55);
}

TEST(ObstacleRecovery, AttemptTimeoutIsDistinctFromNoProgress) {
  ObstacleRecoveryConfig config;
  config.no_progress_timeout = 100.0;
  config.attempt_timeout = 1.0;
  config.maximum_recovery_attempts = 0;
  ObstacleRecoveryController controller(config);
  controller.record_pose(0.0, 0.0);
  controller.record_pose(1.0, 0.0);
  ASSERT_TRUE(controller.trigger(1.0, 0.0, 0.0, 0.0));
  controller.confirm_stopped(1.0, 0.0, At(0.0));

  const auto decision = controller.update_reverse(0.99, 0.0, At(1.1));
  EXPECT_EQ(decision.action, ObstacleRecoveryAction::FAULT_TIMEOUT);
  EXPECT_TRUE(controller.route_may_take_over());
  controller.reset();
  EXPECT_TRUE(controller.idle());
  EXPECT_FALSE(controller.route_may_take_over());
}

TEST(ObstacleRecovery, LatchedNavigationFaultMakesReverseRouteReplaceable) {
  ObstacleRecoveryController controller;
  controller.record_pose(0.0, 0.0);
  controller.record_pose(0.5, 0.0);
  ASSERT_TRUE(controller.trigger(0.5, 0.0, 0.0, 0.0));
  controller.confirm_stopped(0.5, 0.0, At(0.0));
  ASSERT_TRUE(controller.reversing());

  controller.require_route_takeover();

  EXPECT_FALSE(controller.reversing());
  EXPECT_TRUE(controller.replan_required());
  EXPECT_TRUE(controller.route_may_take_over());
}

TEST(ReverseMotionCommand, UsesReverseSpecificHeadingAndSpeedSigns) {
  const auto command = visual_navigation::ComputeReverseMotionCommand(
      1.0, 0.0, 0.0, {0.0, 0.10}, 0.12, 1.0, 0.0, 0.0, 0.25, 0.70);
  ASSERT_TRUE(command.valid);
  EXPECT_LT(command.linear_velocity, 0.0);
  EXPECT_LT(command.angular_velocity, 0.0);

  const auto badlyMisaligned = visual_navigation::ComputeReverseMotionCommand(
      1.0, 0.0, 1.0, {0.0, 0.0}, 0.12, 1.0, 0.0, 0.0, 0.25, 0.70);
  ASSERT_TRUE(badlyMisaligned.valid);
  EXPECT_LT(badlyMisaligned.linear_velocity, 0.0);
  EXPECT_LE(std::abs(badlyMisaligned.angular_velocity), 0.15);

  const auto noSafeReverse = visual_navigation::ComputeReverseMotionCommand(
      1.0, 0.0, 1.5, {0.0, 0.0}, 0.12, 1.0, 0.0, 0.0, 0.25, 0.70);
  ASSERT_TRUE(noSafeReverse.valid);
  EXPECT_DOUBLE_EQ(noSafeReverse.linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(noSafeReverse.angular_velocity, 0.0);
}

} // namespace
