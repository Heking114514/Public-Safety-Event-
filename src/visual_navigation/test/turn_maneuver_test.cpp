#include "visual_navigation/turn_maneuver.hpp"

#include <cmath>

#include "gtest/gtest.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

visual_navigation::CornerShuntPose
Integrate(visual_navigation::CornerShuntPose pose,
          const visual_navigation::CornerShuntTwist &command,
          double elapsed_s) {
  if (std::abs(command.angular_z) <= 1.0e-6) {
    pose.x += command.linear_x * elapsed_s * std::cos(pose.yaw);
    pose.y += command.linear_x * elapsed_s * std::sin(pose.yaw);
    return pose;
  }
  const double next_yaw = pose.yaw + command.angular_z * elapsed_s;
  const double radius = command.linear_x / command.angular_z;
  pose.x += radius * (std::sin(next_yaw) - std::sin(pose.yaw));
  pose.y -= radius * (std::cos(next_yaw) - std::cos(pose.yaw));
  pose.yaw = next_yaw;
  return pose;
}

visual_navigation::TurnManeuverConfig Config() {
  visual_navigation::TurnManeuverConfig config;
  // Test fixture values mirror vehicle_physical.yaml; production has no
  // physical-parameter defaults.
  config.corner.vehicle_length = 0.132;
  config.corner.vehicle_width = 0.132;
  config.corner.base_offset_x = 0.030;
  config.corner.base_offset_y = 0.0;
  config.corner.plan_linear_speed = 0.035;
  config.corner.plan_reverse_speed = -0.035;
  config.corner.closed_loop_min_speed = 0.035;
  config.corner.closed_loop_max_angular_speed = 0.65;
  config.corner.closed_loop_bearing_gain = 2.20;
  config.corner.closed_loop_final_yaw_gain = 1.00;
  config.corner.plan_angular_speed = 0.40;
  config.corner.entry_base_x = -0.050;
  config.corner.max_turn_duration = 60.0;
  config.trim_speed = 0.035;
  config.trim_max_distance = 0.20;
  return config;
}

visual_navigation::TurnManeuverInput LeftTurn(double x = -0.05,
                                               double y = 0.0,
                                               double yaw = 0.0) {
  visual_navigation::TurnManeuverInput input;
  input.context.pose = {x, y, yaw};
  input.context.junction_x = 0.0;
  input.context.junction_y = 0.0;
  input.context.geometry_heading = 0.0;
  input.context.arrival_heading = yaw;
  input.context.target_heading = 0.5 * kPi;
  return input;
}

visual_navigation::TurnManeuverInput RightTurn(double x = -0.05,
                                                double y = 0.0,
                                                double yaw = 0.0) {
  auto input = LeftTurn(x, y, yaw);
  input.context.target_heading = -0.5 * kPi;
  return input;
}

void ApplyDecision(visual_navigation::TurnManeuverInput *input,
                   const visual_navigation::TurnManeuverDecision &decision,
                   const visual_navigation::TurnManeuverConfig &config,
                   double elapsed_s) {
  input->context.applied_twist.linear_x = visual_navigation::LimitRate(
      decision.command.linear_x, input->context.applied_twist.linear_x,
      config.corner.linear_acceleration, config.corner.linear_deceleration,
      elapsed_s);
  input->context.applied_twist.angular_z = visual_navigation::LimitRate(
      decision.command.angular_z, input->context.applied_twist.angular_z,
      config.corner.angular_acceleration, config.corner.angular_deceleration,
      elapsed_s);
  input->context.measured_twist = input->context.applied_twist;
  input->context.pose = Integrate(input->context.pose,
                                  input->context.applied_twist, elapsed_s);
}

visual_navigation::TurnManeuverDecision RunToTerminal(
    visual_navigation::TurnManeuverController *controller,
    visual_navigation::TurnManeuverInput *input,
    const visual_navigation::TurnManeuverConfig &config,
    bool *saw_reverse = nullptr) {
  constexpr double kStep = 1.0 / 30.0;
  visual_navigation::TurnManeuverDecision decision;
  for (int index = 0; index < 1800; ++index) {
    decision = controller->update(*input, index * kStep);
    if (saw_reverse && decision.command.linear_x < 0.0)
      *saw_reverse = true;
    EXPECT_FALSE(std::abs(decision.command.linear_x) <= 1.0e-9 &&
                 std::abs(decision.command.angular_z) > 1.0e-9);
    if (decision.result != visual_navigation::TurnManeuverResult::RUNNING)
      return decision;
    ApplyDecision(input, decision, config, kStep);
  }
  return decision;
}

} // namespace

TEST(TurnManeuver, NominalTurnCompletesWithoutReverse) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.070);
  controller.begin(0.0);
  bool saw_reverse = false;

  const auto decision =
      RunToTerminal(&controller, &input, config, &saw_reverse);

  EXPECT_EQ(decision.result,
            visual_navigation::TurnManeuverResult::COMPLETED)
      << "phase=" << visual_navigation::TurnManeuverStatus(decision.phase)
      << " pose=(" << input.context.pose.x << ", " << input.context.pose.y
      << ", " << input.context.pose.yaw << ") trims="
      << controller.trim_attempts();
  EXPECT_FALSE(saw_reverse);
}

TEST(TurnManeuver, NearJunctionTrimsBeforeStartingArc) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.050);
  controller.begin(0.0);

  bool saw_reverse = false;
  bool saw_forward_arc = false;
  for (int index = 0; index < 30; ++index) {
    const auto decision = controller.update(input, index / 30.0);
    saw_reverse = saw_reverse || decision.command.linear_x < 0.0;
    saw_forward_arc = saw_forward_arc ||
                      (decision.phase ==
                           visual_navigation::TurnManeuverPhase::ARC &&
                       decision.command.linear_x > 0.0);
    if (saw_reverse)
      break;
    ApplyDecision(&input, decision, config, 1.0 / 30.0);
  }

  EXPECT_TRUE(saw_reverse);
  EXPECT_FALSE(saw_forward_arc);
}

TEST(TurnManeuver, RightTurnMirrorsNominalLeftTurn) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = RightTurn(-0.070);
  controller.begin(0.0);
  bool saw_reverse = false;

  const auto decision =
      RunToTerminal(&controller, &input, config, &saw_reverse);

  EXPECT_EQ(decision.result,
            visual_navigation::TurnManeuverResult::COMPLETED)
      << "phase=" << visual_navigation::TurnManeuverStatus(decision.phase)
      << " pose=(" << input.context.pose.x << ", " << input.context.pose.y
      << ", " << input.context.pose.yaw << ")";
  EXPECT_FALSE(saw_reverse);
  EXPECT_LT(input.context.pose.y, 0.0);
}

TEST(TurnManeuver, CompletesAcrossMeasuredEntryOffsets) {
  const auto config = Config();
  const double lateral_offsets[] = {-0.010, 0.0, 0.010};
  const double yaw_offsets[] = {-0.08, 0.0, 0.08};
  for (const double lateral : lateral_offsets) {
    for (const double yaw : yaw_offsets) {
      visual_navigation::TurnManeuverController controller(config);
      auto input = LeftTurn(-0.080, lateral, yaw);
      controller.begin(0.0);

      const auto decision = RunToTerminal(&controller, &input, config);

      EXPECT_EQ(decision.result,
                visual_navigation::TurnManeuverResult::COMPLETED)
          << "entry lateral=" << lateral << " yaw=" << yaw
          << " phase=" << visual_navigation::TurnManeuverStatus(decision.phase)
          << " pose=(" << input.context.pose.x << ", "
          << input.context.pose.y << ", " << input.context.pose.yaw << ")";
    }
  }
}

TEST(TurnManeuver, RecordedSecondCornerUsesIncomingRoadHeading) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn();
  input.context.junction_x = 0.2;
  input.context.junction_y = -0.5;
  input.context.geometry_heading = -0.5 * kPi;
  input.context.arrival_heading = -86.0 * kPi / 180.0;
  input.context.target_heading = 0.0;
  input.context.pose = {0.2, -0.45, input.context.arrival_heading};
  controller.begin(0.0);

  const auto decision = RunToTerminal(&controller, &input, config);

  EXPECT_EQ(decision.result,
            visual_navigation::TurnManeuverResult::COMPLETED)
      << "phase=" << visual_navigation::TurnManeuverStatus(decision.phase)
      << " pose=(" << input.context.pose.x << ", " << input.context.pose.y
      << ", " << input.context.pose.yaw << ") trims="
      << controller.trim_attempts();
}

TEST(TurnManeuver, RfidAnchorRightTurnUsesAxleOffsetEntry) {
  auto config = Config();
  config.corner.base_offset_x = 0.03;
  visual_navigation::TurnManeuverController controller(config);
  auto input = RightTurn();
  input.context.junction_x = 0.2;
  input.context.junction_y = -0.5;
  input.context.geometry_heading = -87.3 * kPi / 180.0;
  input.context.arrival_heading = -87.3 * kPi / 180.0;
  input.context.target_heading = 0.0;
  // RFID correction anchors base_link at the junction. The controller
  // converts it to the axle midpoint using the measured 30 mm offset.
  input.context.pose = {0.2, -0.47, input.context.arrival_heading};
  controller.begin(0.0);

  const auto decision = RunToTerminal(&controller, &input, config);

  EXPECT_EQ(decision.result,
            visual_navigation::TurnManeuverResult::COMPLETED)
      << "phase=" << visual_navigation::TurnManeuverStatus(decision.phase)
      << " pose=(" << input.context.pose.x << ", " << input.context.pose.y
      << ", " << input.context.pose.yaw << ") trims="
      << controller.trim_attempts();
}

TEST(TurnManeuver, UnsafeMidTurnCanRetreatIntoSafeEnvelope) {
  auto config = Config();
  config.corner.base_offset_x = 0.03;
  visual_navigation::TurnManeuverController controller(config);
  auto input = RightTurn();
  input.context.junction_x = 0.2;
  input.context.junction_y = -0.5;
  input.context.geometry_heading = -87.2 * kPi / 180.0;
  input.context.arrival_heading = -87.2 * kPi / 180.0;
  input.context.target_heading = 0.0;
  input.context.pose = {0.225, -0.485, -37.4 * kPi / 180.0};
  controller.begin(0.0);

  const auto decision = RunToTerminal(&controller, &input, config);

  EXPECT_EQ(decision.result,
            visual_navigation::TurnManeuverResult::COMPLETED)
      << "phase=" << visual_navigation::TurnManeuverStatus(decision.phase)
      << " pose=(" << input.context.pose.x << ", " << input.context.pose.y
      << ", " << input.context.pose.yaw << ") trims="
      << controller.trim_attempts();
}

TEST(TurnManeuver, LargeArrivalErrorRequestsClosedLoopReverseTrim) {
  auto config = Config();
  config.direct_entry_yaw_tolerance = 0.10;
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.02, 0.0, 0.22);
  controller.begin(0.0);

  bool saw_reverse = false;
  for (int index = 0; index < 30; ++index) {
    const auto decision = controller.update(input, index / 30.0);
    if (decision.command.linear_x < 0.0) {
      saw_reverse = true;
      EXPECT_NEAR(decision.command.linear_x, -0.035, 1.0e-12);
      EXPECT_NE(decision.phase,
                visual_navigation::TurnManeuverPhase::FAULT);
      break;
    }
    ApplyDecision(&input, decision, config, 1.0 / 30.0);
  }
  EXPECT_TRUE(saw_reverse);
  EXPECT_GT(controller.trim_attempts(), 0U);
}

TEST(TurnManeuver, TrimDirectionUsesHysteresisAfterTargetOvershoot) {
  auto config = Config();
  config.trim_min_direction_hold = 0.30;
  config.trim_min_direction_distance = 0.010;
  config.trim_reverse_hysteresis = 0.015;
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.050, 0.0, 0.22);
  controller.begin(0.0);

  visual_navigation::TurnManeuverDecision decision;
  for (int index = 0; index < 30; ++index) {
    decision = controller.update(input, index / 30.0);
    if (decision.phase == visual_navigation::TurnManeuverPhase::REVERSE_TRIM)
      break;
    ApplyDecision(&input, decision, config, 1.0 / 30.0);
  }
  ASSERT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::REVERSE_TRIM);
  const double target_x = controller.trim_target_x();

  // Move beyond the target far enough to request the opposite direction, but
  // before the minimum hold time has elapsed.
  input.context.pose.x = target_x - 0.030;
  input.context.pose.y = 0.0;
  input.context.pose.yaw = 0.0;
  decision = controller.update(input, 0.20);
  EXPECT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::REVERSE_TRIM);
  EXPECT_LT(decision.command.linear_x, 0.0);

  // Once the hold time is satisfied, the controller changes direction only
  // after a zero-command cycle, rather than immediately injecting forward vx.
  decision = controller.update(input, 0.60);
  EXPECT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::FORWARD_TRIM);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);
}

TEST(TurnManeuver, TrimDirectionSwitchesAreBounded) {
  auto config = Config();
  config.trim_min_direction_hold = 0.0;
  config.trim_min_direction_distance = 0.0;
  config.trim_reverse_hysteresis = 0.0;
  config.max_trim_direction_switches = 1;
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.050, 0.0, 0.22);
  controller.begin(0.0);

  visual_navigation::TurnManeuverDecision decision;
  for (int index = 0; index < 30; ++index) {
    decision = controller.update(input, index / 30.0);
    if (decision.phase == visual_navigation::TurnManeuverPhase::REVERSE_TRIM)
      break;
    ApplyDecision(&input, decision, config, 1.0 / 30.0);
  }
  ASSERT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::REVERSE_TRIM);
  const double target_x = controller.trim_target_x();
  input.context.pose.x = target_x - 0.030;
  decision = controller.update(input, 1.0);
  ASSERT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::FORWARD_TRIM);
  EXPECT_EQ(decision.trim_direction_switches, 1U);

  input.context.pose.x = target_x + 0.030;
  decision = controller.update(input, 2.0);
  EXPECT_EQ(decision.result,
            visual_navigation::TurnManeuverResult::NEED_REPLAN);
  EXPECT_TRUE(decision.trim_limit_reached);
}

TEST(TurnManeuver, NoTrimCandidateRequestsReplanInsteadOfFault) {
  auto config = Config();
  config.direct_entry_yaw_tolerance = 0.01;
  config.trim_max_distance = 0.005;
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.02, 0.0, 0.30);
  controller.begin(0.0);

  visual_navigation::TurnManeuverDecision decision;
  for (int index = 0; index < 10; ++index) {
    decision = controller.update(input, index / 30.0);
    if (decision.result != visual_navigation::TurnManeuverResult::RUNNING)
      break;
  }

  EXPECT_EQ(decision.phase,
            visual_navigation::TurnManeuverPhase::NEED_REPLAN);
  EXPECT_EQ(decision.result,
            visual_navigation::TurnManeuverResult::NEED_REPLAN);
}

TEST(TurnManeuver, ReportsFootprintCornerOutsideRoadEnvelope) {
  const auto config = Config();
  visual_navigation::CornerShuntController checker(config.corner);
  visual_navigation::CornerShuntContext context;
  context.pose = {0.5, 0.0, 0.0};
  context.target_heading = 0.5 * kPi;

  visual_navigation::CornerShuntSafetyReport report;
  EXPECT_FALSE(checker.pose_is_safe(context, context.pose, &report));
  EXPECT_EQ(report.failure,
            visual_navigation::CornerShuntSafetyFailure::
                FOOTPRINT_OUTSIDE_ENVELOPE);
  EXPECT_GE(report.footprint_corner, 0);
  EXPECT_GT(report.corner_local.x, 0.5 * config.corner.road_width -
                                      config.corner.road_margin);
}

TEST(TurnManeuver, ObstaclePauseResumesFromMeasuredPose) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.070);
  controller.begin(0.0);
  controller.update(input, 0.0);
  input.obstacle_blocked = true;

  auto decision = controller.update(input, 0.1);
  EXPECT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::PAUSED);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);

  input.obstacle_blocked = false;
  input.context.pose.x += 0.005;
  decision = controller.update(input, 0.2);
  EXPECT_TRUE(decision.phase == visual_navigation::TurnManeuverPhase::ENTRY_CHECK ||
              decision.phase == visual_navigation::TurnManeuverPhase::ARC);
}

TEST(TurnManeuver, PauseDurationDoesNotConsumeManeuverTimeout) {
  auto config = Config();
  config.corner.max_turn_duration = 1.0;
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.070);
  controller.begin(0.0);
  controller.update(input, 0.0);
  input.motion_allowed = false;

  auto decision = controller.update(input, 0.1);
  EXPECT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::PAUSED);
  decision = controller.update(input, 10.0);
  EXPECT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::PAUSED);

  input.motion_allowed = true;
  decision = controller.update(input, 10.1);
  EXPECT_EQ(decision.result, visual_navigation::TurnManeuverResult::RUNNING);
  EXPECT_NE(decision.phase, visual_navigation::TurnManeuverPhase::FAULT);
}

TEST(TurnManeuver, CapturedGeometryDoesNotFollowLaterRouteFrames) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.070);
  controller.begin(0.0);
  controller.update(input, 0.0);

  input.context.geometry_heading = kPi;
  input.context.target_heading = -0.5 * kPi;
  const auto decision = controller.update(input, 0.1);

  EXPECT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::ARC);
  EXPECT_EQ(decision.result, visual_navigation::TurnManeuverResult::RUNNING);
}

TEST(TurnManeuver, ContextChoosesForwardTrimWhenAlreadyPastEntry) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.12, 0.0, 0.25);
  controller.begin(0.0);

  visual_navigation::TurnManeuverDecision decision;
  for (int index = 0; index < 10; ++index) {
    decision = controller.update(input, index / 30.0);
    if (decision.phase == visual_navigation::TurnManeuverPhase::FORWARD_TRIM)
      break;
    ApplyDecision(&input, decision, config, 1.0 / 30.0);
  }

  ASSERT_EQ(decision.phase,
            visual_navigation::TurnManeuverPhase::FORWARD_TRIM);
  decision = controller.update(input, 0.4);
  EXPECT_EQ(decision.phase,
            visual_navigation::TurnManeuverPhase::FORWARD_TRIM);
  EXPECT_GT(decision.command.linear_x, 0.0);
  EXPECT_EQ(decision.suggested_recovery_direction,
            visual_navigation::TurnManeuverRecoveryDirection::FORWARD_TO_EXIT);
}

TEST(TurnManeuver, DecisionCarriesLastSafePoseAfterMeasuredOvershoot) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.070);
  controller.begin(0.0);
  controller.update(input, 0.0);

  const auto safe_pose = input.context.pose;
  input.context.pose = {0.25, 0.0, 0.0};
  const auto decision = controller.update(input, 0.1);

  EXPECT_TRUE(decision.has_last_safe_pose);
  EXPECT_DOUBLE_EQ(decision.last_safe_pose.x, safe_pose.x);
  EXPECT_DOUBLE_EQ(decision.last_safe_pose.y, safe_pose.y);
}

TEST(TurnManeuver, ExitAlignIsObservedBeforeCompletion) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.070);
  controller.begin(0.0);

  bool saw_exit_align = false;
  visual_navigation::TurnManeuverDecision decision;
  constexpr double kStep = 1.0 / 30.0;
  for (int index = 0; index < 1800; ++index) {
    decision = controller.update(input, index * kStep);
    saw_exit_align = saw_exit_align ||
                     decision.phase == visual_navigation::TurnManeuverPhase::EXIT_ALIGN;
    if (decision.result != visual_navigation::TurnManeuverResult::RUNNING)
      break;
    ApplyDecision(&input, decision, config, kStep);
  }

  EXPECT_TRUE(saw_exit_align);
  EXPECT_EQ(decision.result,
            visual_navigation::TurnManeuverResult::COMPLETED);
}

TEST(TurnManeuver, CancelStopsAndRequiresNewBegin) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn(-0.070);
  controller.begin(0.0);
  controller.update(input, 0.0);

  input.cancel = true;
  auto decision = controller.update(input, 0.1);
  EXPECT_EQ(decision.result, visual_navigation::TurnManeuverResult::CANCELED);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);
  EXPECT_FALSE(controller.active());

  input.cancel = false;
  decision = controller.update(input, 0.2);
  EXPECT_EQ(decision.result, visual_navigation::TurnManeuverResult::CANCELED);
  EXPECT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::IDLE);

  controller.begin(0.3);
  decision = controller.update(input, 0.3);
  EXPECT_EQ(decision.result, visual_navigation::TurnManeuverResult::RUNNING);
  EXPECT_EQ(decision.phase, visual_navigation::TurnManeuverPhase::ENTRY_CHECK);
}

TEST(TurnManeuver, RejectsNonFiniteTwistAtCapture) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn();
  input.context.applied_twist.angular_z =
      std::numeric_limits<double>::quiet_NaN();
  controller.begin(0.0);

  const auto decision = controller.update(input, 0.0);

  EXPECT_EQ(decision.result, visual_navigation::TurnManeuverResult::FAULT);
  EXPECT_EQ(decision.failure_reason,
            visual_navigation::TurnManeuverFailureReason::INVALID_CONTEXT);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);
}

TEST(TurnManeuver, RejectsNonFiniteControlTime) {
  const auto config = Config();
  visual_navigation::TurnManeuverController controller(config);
  auto input = LeftTurn();
  controller.begin(std::numeric_limits<double>::quiet_NaN());

  const auto decision = controller.update(input, 0.0);

  EXPECT_EQ(decision.result, visual_navigation::TurnManeuverResult::FAULT);
  EXPECT_EQ(decision.failure_reason,
            visual_navigation::TurnManeuverFailureReason::INVALID_CONTEXT);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);
}
