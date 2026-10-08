#include "visual_navigation/obstacle_recovery.hpp"

#include <chrono>

#include "gtest/gtest.h"

namespace {

using visual_navigation::ObstacleRecoveryAction;
using visual_navigation::ObstacleRecoveryConfig;
using visual_navigation::ObstacleRecoveryController;
using visual_navigation::RecoveryPoint;
using visual_navigation::RecoveryReplayStep;
using visual_navigation::RetreatExtensionResult;

ObstacleRecoveryController::TimePoint At(double seconds) {
  return ObstacleRecoveryController::TimePoint{} +
         std::chrono::duration_cast<
             ObstacleRecoveryController::Clock::duration>(
             std::chrono::duration<double>(seconds));
}

TEST(ObstacleClassification, IgnoresKnownWallBeyondPlannedTurn) {
  EXPECT_FALSE(visual_navigation::UnexpectedObstacle(
      false, true, 0.20, true, 0.30, 0.096, 0.07425, 0.05));
  EXPECT_FALSE(visual_navigation::UnexpectedObstacle(
      true, true, 0.30, true, 0.25, 0.096, 0.07425, 0.05));

  EXPECT_TRUE(visual_navigation::UnexpectedObstacle(
      true, true, 0.10, true, 0.40, 0.096, 0.07425, 0.05));
  EXPECT_TRUE(visual_navigation::UnexpectedObstacle(
      true, false, 0.30, true, 0.25, 0.096, 0.07425, 0.05));
  EXPECT_TRUE(visual_navigation::UnexpectedObstacle(
      true, true, 0.30, false, 0.25, 0.096, 0.07425, 0.05));
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
  EXPECT_EQ(controller.breadcrumb_count(), 4U);
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

TEST(ObstacleRecovery, PlannerMayRequestAnEarlierRetreatAnchor) {
  ObstacleRecoveryConfig config;
  config.breadcrumb_spacing = 0.10;
  config.reverse_lookahead = 0.10;
  config.target_tolerance = 0.05;
  config.anchor_tolerance = 0.05;
  ObstacleRecoveryController controller(config);
  for (double x : {0.0, 0.2, 0.4, 0.6, 0.8})
    controller.record_pose(x, 0.0);

  ASSERT_TRUE(controller.trigger(0.8, 0.0, 0.4, 0.0));
  controller.confirm_stopped(0.8, 0.0, At(0.0));
  ASSERT_EQ(controller.update_reverse(0.4, 0.0, At(1.0)).action,
            ObstacleRecoveryAction::REPLAN_REQUIRED);
  ASSERT_EQ(controller.extend_retreat(0.30), RetreatExtensionResult::ACCEPTED);
  EXPECT_TRUE(controller.braking());
  EXPECT_NEAR(controller.anchor().x, 0.0, 1.0e-12);
  EXPECT_EQ(controller.remaining_breadcrumbs_before_anchor(), 0U);

  controller.confirm_stopped(0.4, 0.0, At(1.1));
  for (int index = 0; index < 2; ++index) {
    const auto replay =
        controller.replay_step(0.4, 0.0, 0.0, 0.0, 0.0, At(1.2));
    ASSERT_TRUE(replay.command_valid);
  }
  EXPECT_EQ(controller.replay_step(0.0, 0.0, 0.0, 0.0, 0.0, At(2.0)).action,
            ObstacleRecoveryAction::REPLAN_REQUIRED);
  EXPECT_EQ(controller.extend_retreat(0.30), RetreatExtensionResult::EXHAUSTED);
}

TEST(ObstacleRecovery, ExtendedRetreatReplaysCrossedTurnInTraceOrder) {
  ObstacleRecoveryConfig config;
  config.breadcrumb_spacing = 0.01;
  config.maximum_breadcrumb_step = 0.40;
  config.target_tolerance = 0.02;
  config.anchor_tolerance = 0.08;
  ObstacleRecoveryController controller(config);
  controller.record_sample(0.00, 0.00, 0.0, 0.10, 0.00);
  controller.record_sample(0.20, 0.00, 0.0, 0.10, 0.00);
  controller.record_sample(0.40, 0.00, 0.0, 0.10, 0.00);     // Turn approach.
  controller.record_sample(0.44, -0.04, -0.4, 0.02, -0.20);  // Forward shunt.
  controller.record_sample(0.42, -0.08, -0.8, -0.02, -0.20); // Reverse shunt.
  controller.record_sample(0.40, -0.07, -1.57, 0.02, -0.20); // Turn exit.
  controller.record_sample(0.40, -0.20, -1.57, 0.10, 0.00);
  controller.record_sample(0.40, -0.40, -1.57, 0.10, 0.00);
  controller.record_sample(0.40, -0.60, -1.57, 0.10, 0.00);
  controller.record_sample(0.40, -0.80, -1.57, 0.10, 0.00);

  ASSERT_TRUE(controller.trigger(0.40, -0.80, -1.57, 0.10, 0.00, 0.40, 0.00));
  EXPECT_NEAR(controller.anchor().y, -0.07, 1.0e-12);
  controller.confirm_stopped(0.40, -0.80, At(0.0));
  ASSERT_EQ(controller.update_reverse(0.40, -0.07, At(1.0)).action,
            ObstacleRecoveryAction::REPLAN_REQUIRED);

  ASSERT_EQ(controller.extend_retreat(0.30), RetreatExtensionResult::ACCEPTED);
  controller.confirm_stopped(0.40, -0.07, At(1.1));

  const std::vector<std::pair<double, double>> expected_commands{
      {-0.02, 0.20}, {0.02, 0.20}, {-0.02, 0.20}, {-0.10, 0.00}};
  double applied_linear = 0.0;
  double applied_angular = 0.0;
  auto alignment =
      controller.replay_step(0.40, -0.07, -1.30, 0.0, 0.0, At(1.15));
  ASSERT_TRUE(alignment.command_valid);
  EXPECT_DOUBLE_EQ(alignment.linear_velocity, 0.0);
  EXPECT_LT(alignment.angular_velocity, 0.0);

  RecoveryReplayStep replay = controller.replay_step(
      0.40, -0.07, -1.57, applied_linear, applied_angular, At(1.2));
  ASSERT_TRUE(replay.command_valid);
  EXPECT_FALSE(replay.publish_recorded);
  EXPECT_NEAR(replay.linear_velocity, expected_commands[0].first, 1.0e-12);
  EXPECT_NEAR(replay.angular_velocity, expected_commands[0].second, 1.0e-12);
  applied_linear = expected_commands[0].first;
  applied_angular = expected_commands[0].second;
  replay = controller.replay_step(0.40, -0.07, -1.57, applied_linear,
                                  applied_angular, At(1.3));
  ASSERT_TRUE(replay.command_valid);
  EXPECT_FALSE(replay.publish_recorded);

  for (std::size_t index = 1; index < expected_commands.size(); ++index) {
    const auto &expected = expected_commands[index];
    replay = controller.replay_step(0.40, -0.07, -1.57, applied_linear,
                                    applied_angular, At(1.4));
    ASSERT_TRUE(replay.command_valid);
    EXPECT_TRUE(replay.publish_recorded);
    EXPECT_NEAR(replay.linear_velocity, expected.first, 1.0e-12);
    EXPECT_NEAR(replay.angular_velocity, expected.second, 1.0e-12);
    applied_linear = expected.first;
    applied_angular = expected.second;
  }
  replay = controller.replay_step(0.20, 0.00, 0.0, applied_linear,
                                  applied_angular, At(1.5));
  EXPECT_EQ(replay.action, ObstacleRecoveryAction::REPLAN_REQUIRED);
}

TEST(ObstacleRecovery, ContinueRetreatRequiresReplanState) {
  ObstacleRecoveryController controller;
  controller.record_pose(0.0, 0.0);
  controller.record_pose(0.5, 0.0);

  EXPECT_EQ(controller.extend_retreat(0.20),
            RetreatExtensionResult::INVALID_STATE);
}

TEST(ObstacleRecovery, CompletedRouteMayBeginBoundedRetreat) {
  ObstacleRecoveryConfig config;
  config.breadcrumb_spacing = 0.05;
  ObstacleRecoveryController controller(config);
  controller.record_sample(0.0, 0.0, 0.0, 0.10, 0.0);
  controller.record_sample(0.2, 0.0, 0.0, 0.10, 0.0);
  controller.record_sample(0.4, 0.0, 0.0, 0.10, 0.0);

  EXPECT_EQ(controller.begin_retreat(0.20, 0.4, 0.0, 0.0),
            RetreatExtensionResult::ACCEPTED);
  EXPECT_TRUE(controller.braking());
  EXPECT_NEAR(controller.replay_goal().point.x, 0.2, 1.0e-12);
}

TEST(ObstacleRecovery, RouteHandoffPreservesEarlierRetreatTrace) {
  ObstacleRecoveryConfig config;
  config.breadcrumb_spacing = 0.05;
  config.maximum_breadcrumb_step = 0.30;
  config.target_tolerance = 0.04;
  config.anchor_tolerance = 0.06;
  ObstacleRecoveryController controller(config);
  controller.record_sample(0.0, 0.0, 0.0, 0.1, 0.0);
  controller.record_sample(0.2, 0.0, 0.0, 0.1, 0.0);
  controller.record_sample(0.4, 0.0, 0.0, 0.1, 0.0);

  controller.prepare_for_route(true);
  controller.record_sample(0.6, 0.0, 0.0, 0.1, 0.0);
  controller.record_sample(0.8, 0.0, 0.0, 0.1, 0.0);

  ASSERT_TRUE(controller.trigger(0.8, 0.0, 0.0, 0.1, 0.0, 0.6, 0.0));
  controller.confirm_stopped(0.8, 0.0, At(0.0));
  ASSERT_EQ(controller.update_reverse(0.6, 0.0, At(1.0)).action,
            ObstacleRecoveryAction::REPLAN_REQUIRED);
  EXPECT_EQ(controller.extend_retreat(0.30), RetreatExtensionResult::ACCEPTED);
  EXPECT_GT(controller.remaining_breadcrumbs_before_anchor(), 0U);
}

TEST(ObstacleRecovery, ReplayRejectsLargeStartHeadingMismatch) {
  ObstacleRecoveryConfig config;
  config.breadcrumb_spacing = 0.10;
  config.anchor_tolerance = 0.05;
  config.replay_start_max_yaw_error = 0.35;
  ObstacleRecoveryController controller(config);
  controller.record_sample(0.0, 0.0, 0.0, 0.10, 0.0);
  controller.record_sample(0.2, 0.0, 0.0, 0.10, 0.0);
  controller.record_sample(0.4, 0.0, 0.0, 0.10, 0.0);
  ASSERT_TRUE(controller.trigger(0.4, 0.0, 0.0, 0.10, 0.0, 0.2, 0.0));
  controller.confirm_stopped(0.4, 0.0, At(0.0));
  ASSERT_EQ(controller.update_reverse(0.2, 0.0, At(1.0)).action,
            ObstacleRecoveryAction::REPLAN_REQUIRED);
  ASSERT_EQ(controller.extend_retreat(0.10, 0.2, 0.0, 0.0),
            RetreatExtensionResult::ACCEPTED);
  controller.confirm_stopped(0.2, 0.0, At(1.1));

  const auto replay = controller.replay_step(0.2, 0.0, 1.0, 0.0, 0.0, At(1.2));
  EXPECT_EQ(replay.action, ObstacleRecoveryAction::FAULT_NO_PROGRESS);
  EXPECT_GT(replay.yaw_error, config.replay_start_max_yaw_error);
}

TEST(ObstacleRecovery, ReplayAlignsModerateRouteHandoffHeadingMismatch) {
  ObstacleRecoveryConfig config;
  config.breadcrumb_spacing = 0.10;
  config.anchor_tolerance = 0.05;
  config.replay_start_max_yaw_error = 0.70;
  ObstacleRecoveryController controller(config);
  controller.record_sample(0.0, 0.0, 0.0, 0.10, 0.0);
  controller.record_sample(0.2, 0.0, 0.0, 0.10, 0.0);
  controller.record_sample(0.4, 0.0, 0.0, 0.10, 0.0);
  ASSERT_TRUE(controller.trigger(0.4, 0.0, 0.0, 0.10, 0.0, 0.2, 0.0));
  controller.confirm_stopped(0.4, 0.0, At(0.0));
  ASSERT_EQ(controller.update_reverse(0.2, 0.0, At(1.0)).action,
            ObstacleRecoveryAction::REPLAN_REQUIRED);
  ASSERT_EQ(controller.extend_retreat(0.10, 0.2, 0.0, 0.0),
            RetreatExtensionResult::ACCEPTED);
  controller.confirm_stopped(0.2, 0.0, At(1.1));

  const auto replay = controller.replay_step(0.2, 0.0, 0.58, 0.0, 0.0, At(1.2));
  EXPECT_EQ(replay.action, ObstacleRecoveryAction::NONE);
  EXPECT_TRUE(replay.command_valid);
  EXPECT_DOUBLE_EQ(replay.linear_velocity, 0.0);
  EXPECT_LT(replay.angular_velocity, 0.0);
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
