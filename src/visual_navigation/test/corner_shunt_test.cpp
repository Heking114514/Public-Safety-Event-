#include "visual_navigation/corner_shunt.hpp"
#include "gtest/gtest.h"

#include <chrono>
#include <cmath>
#include <thread>

namespace {

constexpr double kPi = 3.14159265358979323846;

visual_navigation::CornerShuntContext LeftCornerContext(double x, double y,
                                                        double yaw) {
  visual_navigation::CornerShuntContext context;
  context.pose = {x, y, yaw};
  context.junction_x = 0.0;
  context.junction_y = 0.0;
  context.geometry_heading = 0.0;
  context.arrival_heading = yaw;
  context.target_heading = kPi / 2.0;
  return context;
}

visual_navigation::CornerShuntContext RightCornerContext(double x, double y,
                                                         double yaw) {
  visual_navigation::CornerShuntContext context;
  context.pose = {x, y, yaw};
  context.junction_x = 0.0;
  context.junction_y = 0.0;
  context.geometry_heading = 0.0;
  context.arrival_heading = yaw;
  context.target_heading = -kPi / 2.0;
  return context;
}

visual_navigation::CornerShuntContext ReverseArrivalContext() {
  visual_navigation::CornerShuntContext context;
  context.pose = {0.005, 0.0, -0.5 * kPi};
  context.junction_x = 0.0;
  context.junction_y = 0.0;
  context.geometry_heading = 0.5 * kPi;
  context.arrival_heading = -0.5 * kPi;
  context.target_heading = kPi;
  context.reverse_arrival = true;
  return context;
}

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

bool CommandIsSafe(const visual_navigation::CornerShuntController &controller,
                   const visual_navigation::CornerShuntContext &context,
                   const visual_navigation::CornerShuntTwist &command) {
  if (!controller.pose_is_safe(context, context.pose))
    return false;
  auto pose = context.pose;
  auto applied = context.applied_twist;
  for (int index = 0; index < 5; ++index) {
    applied.linear_x = visual_navigation::LimitRate(
        command.linear_x, applied.linear_x, 0.25, 0.80, 0.04);
    applied.angular_z = visual_navigation::LimitRate(
        command.angular_z, applied.angular_z, 1.80, 3.00, 0.04);
    pose = Integrate(pose, applied, 0.04);
    if (!controller.pose_is_safe(context, pose))
      return false;
  }
  return true;
}

void ApplyLimitedDecision(
    visual_navigation::CornerShuntContext *context,
    const visual_navigation::CornerShuntDecision &decision, double elapsed_s) {
  if (decision.phase == visual_navigation::CornerShuntPhase::STOP ||
      decision.phase == visual_navigation::CornerShuntPhase::CHECK ||
      decision.phase == visual_navigation::CornerShuntPhase::BLOCKED) {
    context->applied_twist = {};
    return;
  }
  context->applied_twist.linear_x = visual_navigation::LimitRate(
      decision.command.linear_x, context->applied_twist.linear_x, 0.25, 0.80,
      elapsed_s);
  context->applied_twist.angular_z = visual_navigation::LimitRate(
      decision.command.angular_z, context->applied_twist.angular_z, 1.80, 3.00,
      elapsed_s);
  context->pose = Integrate(context->pose, context->applied_twist, elapsed_s);
}

visual_navigation::CornerShuntDecision
AwaitPlannedMotion(visual_navigation::CornerShuntController &controller,
                   const visual_navigation::CornerShuntContext &context,
                   double *now) {
  visual_navigation::CornerShuntDecision decision;
  for (int attempt = 0; attempt < 300; ++attempt) {
    decision = controller.update(context, *now);
    if (decision.phase != visual_navigation::CornerShuntPhase::CHECK)
      return decision;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    *now += 0.01;
  }
  return decision;
}

} // namespace

TEST(CornerShunt, StartsWithStopThenMovesForwardAcrossJunction) {
  visual_navigation::CornerShuntController controller;
  controller.begin(0.0);

  auto decision = controller.update(LeftCornerContext(-0.05, 0.0, 0.0), 0.05);
  EXPECT_EQ(decision.phase, visual_navigation::CornerShuntPhase::STOP);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);

  decision = controller.update(LeftCornerContext(-0.05, 0.0, 0.0), 0.30);
  EXPECT_EQ(decision.phase, visual_navigation::CornerShuntPhase::CHECK);

  double now = 0.36;
  decision =
      AwaitPlannedMotion(controller, LeftCornerContext(-0.05, 0.0, 0.0), &now);
  EXPECT_NE(decision.phase, visual_navigation::CornerShuntPhase::REVERSE);
  EXPECT_GE(decision.command.linear_x, 0.0);
}

TEST(CornerShunt, BuildsRollingLocalTrajectoryForForwardTurn) {
  visual_navigation::CornerShuntConfig config;
  config.local_trajectory_length = 0.80;
  visual_navigation::CornerShuntController controller(config);
  controller.begin(0.0);
  controller.update(LeftCornerContext(-0.05, 0.0, 0.0), 0.30);
  double now = 0.36;
  auto decision = AwaitPlannedMotion(
      controller, LeftCornerContext(-0.05, 0.0, 0.0), &now);
  for (int attempt = 0; attempt < 300 &&
       controller.local_trajectory_size() < 2U; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    now += 0.01;
    decision = controller.update(LeftCornerContext(-0.05, 0.0, 0.0), now);
  }

  EXPECT_FALSE(decision.blocked);
  EXPECT_GE(controller.local_trajectory_size(), 2U);
  EXPECT_GT(controller.local_trajectory_length(), 0.10);
  EXPECT_LE(controller.local_trajectory_length(), 0.80 + 1.0e-6);
}

TEST(CornerShunt, MirrorsRightTurns) {
  visual_navigation::CornerShuntController controller;
  controller.begin(0.0);

  controller.update(RightCornerContext(-0.05, 0.0, 0.0), 0.30);
  double now = 0.36;
  auto decision =
      AwaitPlannedMotion(controller, RightCornerContext(-0.05, 0.0, 0.0), &now);
  EXPECT_NE(decision.phase, visual_navigation::CornerShuntPhase::REVERSE);
  EXPECT_GE(decision.command.linear_x, 0.0);
}

TEST(CornerShunt, KeepsConfiguredRoadMarginWith132MillimeterChassis) {
  visual_navigation::CornerShuntController controller;
  const auto safe = LeftCornerContext(-0.07, 0.031, 0.0);
  const auto unsafe = LeftCornerContext(-0.07, 0.034, 0.0);

  EXPECT_TRUE(controller.pose_is_safe(safe, safe.pose));
  EXPECT_FALSE(controller.pose_is_safe(unsafe, unsafe.pose));
}

TEST(CornerShunt, CompletesWhenYawMatchesExitLane) {
  visual_navigation::CornerShuntController controller;
  controller.begin(0.0);

  const auto decision =
      controller.update(LeftCornerContext(-0.0075, 0.1225, kPi / 2.0), 0.30);

  EXPECT_TRUE(decision.complete);
  EXPECT_EQ(decision.phase, visual_navigation::CornerShuntPhase::COMPLETE);
}

TEST(CornerShunt, DoesNotHandOffWithLargeResidualYaw) {
  visual_navigation::CornerShuntController controller;
  controller.begin(0.0);

  const auto decision =
      controller.update(LeftCornerContext(-0.036, 0.021, 1.12), 0.30);

  EXPECT_FALSE(decision.complete);
  EXPECT_NE(decision.phase, visual_navigation::CornerShuntPhase::COMPLETE);
}

TEST(CornerShunt, DoesNotHandOffBeyondExitYawTolerance) {
  visual_navigation::CornerShuntController controller;
  controller.begin(0.0);

  const auto decision =
      controller.update(LeftCornerContext(-0.0075, 0.1225, 1.45), 0.30);

  EXPECT_FALSE(decision.complete);
  EXPECT_NE(decision.phase, visual_navigation::CornerShuntPhase::COMPLETE);
}

TEST(CornerShunt, SeparatesRoadGeometryFromResidualArrivalYaw) {
  visual_navigation::CornerShuntController controller;
  auto context = LeftCornerContext(-0.05, 0.0, 0.0);
  context.geometry_heading = 0.0;
  context.arrival_heading = -1.50;
  context.target_heading = 0.5 * kPi;
  auto nominal = context;
  nominal.arrival_heading = nominal.geometry_heading;
  EXPECT_TRUE(controller.pose_is_safe(context, context.pose));
  EXPECT_TRUE(controller.pose_is_safe(nominal, nominal.pose));

  controller.begin(0.0);
  const auto decision = controller.update(context, 0.30);
  EXPECT_FALSE(decision.blocked);
}

TEST(CornerShunt, DoesNotHandOffBeforeHtmlExitWindow) {
  visual_navigation::CornerShuntController controller;
  controller.begin(0.0);

  const auto decision =
      controller.update(LeftCornerContext(-0.020, 0.040, 1.40), 0.30);

  EXPECT_FALSE(decision.complete);
  EXPECT_NE(decision.phase, visual_navigation::CornerShuntPhase::COMPLETE);
}

TEST(CornerShunt, SimulatedWaypointStopPoseReachesSafeExit) {
  visual_navigation::CornerShuntController controller;
  auto context = LeftCornerContext(-0.05, 0.0, 0.0);
  controller.begin(0.0);

  bool complete = false;
  bool blocked = false;
  bool all_poses_safe = true;
  visual_navigation::CornerShuntPose first_unsafe_pose;
  visual_navigation::CornerShuntDecision last_decision;
  int arc_steps = 0;
  int check_steps = 0;
  int stop_steps = 0;
  int rotate_steps = 0;
  int creep_steps = 0;
  int reverse_steps = 0;
  int idle_steps = 0;
  int complete_steps = 0;
  int blocked_steps = 0;
  int other_steps = 0;
  double angular_motion = 0.0;
  double now = 0.0;
  constexpr double kStep = 1.0 / 30.0;
  for (int index = 0; index < 1800; ++index) {
    const auto decision = controller.update(context, now);
    last_decision = decision;
    complete = decision.complete;
    blocked = decision.blocked;
    if (complete || blocked)
      break;
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (decision.phase == visual_navigation::CornerShuntPhase::STOP)
      ++stop_steps;
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      ++check_steps;
    if (decision.phase == visual_navigation::CornerShuntPhase::ARC)
      ++arc_steps;
    if (decision.phase == visual_navigation::CornerShuntPhase::ROTATE)
      ++rotate_steps;
    if (decision.phase == visual_navigation::CornerShuntPhase::CREEP)
      ++creep_steps;
    if (decision.phase == visual_navigation::CornerShuntPhase::REVERSE)
      ++reverse_steps;
    if (decision.phase == visual_navigation::CornerShuntPhase::IDLE)
      ++idle_steps;
    if (decision.phase == visual_navigation::CornerShuntPhase::COMPLETE)
      ++complete_steps;
    if (decision.phase == visual_navigation::CornerShuntPhase::BLOCKED)
      ++blocked_steps;
    if (decision.phase != visual_navigation::CornerShuntPhase::STOP &&
        decision.phase != visual_navigation::CornerShuntPhase::CHECK &&
        decision.phase != visual_navigation::CornerShuntPhase::ARC &&
        decision.phase != visual_navigation::CornerShuntPhase::ROTATE &&
        decision.phase != visual_navigation::CornerShuntPhase::CREEP &&
        decision.phase != visual_navigation::CornerShuntPhase::REVERSE)
      ++other_steps;
    angular_motion += std::abs(decision.command.angular_z) * kStep;
    ApplyLimitedDecision(&context, decision, kStep);
    if (all_poses_safe && !controller.pose_is_safe(context, context.pose)) {
      all_poses_safe = false;
      first_unsafe_pose = context.pose;
    }
    now += kStep;
  }

  EXPECT_TRUE(complete)
      << "phase=" << visual_navigation::CornerShuntStatus(last_decision.phase)
      << " command=(" << last_decision.command.linear_x << ", "
      << last_decision.command.angular_z << ") pose=(" << context.pose.x << ", "
      << context.pose.y << ", " << context.pose.yaw << ") time=" << now
      << " arc_steps=" << arc_steps << " check_steps=" << check_steps
      << " stop_steps=" << stop_steps << " rotate_steps=" << rotate_steps
      << " creep_steps=" << creep_steps << " reverse_steps=" << reverse_steps
      << " idle_steps=" << idle_steps << " complete_steps=" << complete_steps
      << " blocked_steps=" << blocked_steps << " other_steps=" << other_steps
      << " angular_motion=" << angular_motion
      << " pose_safe=" << controller.pose_is_safe(context, context.pose)
      << " rotate_safe=" << CommandIsSafe(controller, context, {0.0, 0.20})
      << " creep_safe=" << CommandIsSafe(controller, context, {0.020, 0.0})
      << " arc_safe=" << CommandIsSafe(controller, context, {0.024, 0.05})
      << " reverse_safe=" << CommandIsSafe(controller, context, {-0.008, 0.0});
  EXPECT_FALSE(blocked);
  EXPECT_TRUE(all_poses_safe)
      << "first unsafe pose=(" << first_unsafe_pose.x << ", "
      << first_unsafe_pose.y << ", " << first_unsafe_pose.yaw << ")";
  EXPECT_EQ(reverse_steps, 0);
  EXPECT_LE(controller.plan_count(), 2);
}

TEST(CornerShunt, SimulatedRightTurnMirrorsSafeExit) {
  visual_navigation::CornerShuntController controller;
  auto context = RightCornerContext(-0.05, 0.0, 0.0);
  controller.begin(0.0);

  bool complete = false;
  bool blocked = false;
  bool all_poses_safe = true;
  visual_navigation::CornerShuntPose first_unsafe_pose;
  double now = 0.0;
  constexpr double kStep = 1.0 / 30.0;
  for (int index = 0; index < 1800; ++index) {
    const auto decision = controller.update(context, now);
    complete = decision.complete;
    blocked = decision.blocked;
    if (complete || blocked)
      break;
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ApplyLimitedDecision(&context, decision, kStep);
    if (all_poses_safe && !controller.pose_is_safe(context, context.pose)) {
      all_poses_safe = false;
      first_unsafe_pose = context.pose;
    }
    now += kStep;
  }

  EXPECT_TRUE(complete) << "pose=(" << context.pose.x << ", " << context.pose.y
                        << ", " << context.pose.yaw << ")";
  EXPECT_FALSE(blocked);
  EXPECT_TRUE(all_poses_safe)
      << "first unsafe pose=(" << first_unsafe_pose.x << ", "
      << first_unsafe_pose.y << ", " << first_unsafe_pose.yaw << ")";
  EXPECT_TRUE(controller.pose_is_safe(context, context.pose));
  EXPECT_LE(controller.plan_count(), 2);
}

TEST(CornerShunt, ClosedLoopSweepStaysSafeAcrossEntryErrors) {
  constexpr double kStep = 1.0 / 30.0;
  for (double start_x : {-0.075, -0.060, -0.050}) {
    for (double lateral : {-0.015, 0.0, 0.015}) {
      for (double yaw : {-0.06, 0.0, 0.06}) {
        for (int direction : {-1, 1}) {
          visual_navigation::CornerShuntController controller;
          auto context = direction > 0
                             ? LeftCornerContext(start_x, lateral, yaw)
                             : RightCornerContext(start_x, -lateral, -yaw);
          ASSERT_TRUE(controller.pose_is_safe(context, context.pose));
          controller.begin(0.0);

          bool complete = false;
          double now = 0.0;
          for (int index = 0; index < 1800; ++index) {
            const auto decision = controller.update(context, now);
            ASSERT_FALSE(decision.blocked)
                << "start=(" << start_x << ", " << lateral << ", " << yaw
                << ") direction=" << direction;
            if (decision.complete) {
              complete = true;
              break;
            }
            ASSERT_GE(decision.command.linear_x, 0.0);
            for (int sample = 0; sample < 5; ++sample) {
              ApplyLimitedDecision(&context, decision, kStep / 5.0);
              ASSERT_TRUE(controller.pose_is_safe(context, context.pose))
                  << "start=(" << start_x << ", " << lateral << ", " << yaw
                  << ") direction=" << direction << " pose=("
                  << context.pose.x << ", " << context.pose.y << ", "
                  << context.pose.yaw << ")";
            }
            now += kStep;
          }
          EXPECT_TRUE(complete)
              << "start=(" << start_x << ", " << lateral << ", " << yaw
              << ") direction=" << direction;
        }
      }
    }
  }
}

TEST(CornerShunt, OrdinaryTurnReactsToMeasuredEntryOffset) {
  visual_navigation::CornerShuntController left_controller;
  visual_navigation::CornerShuntController right_controller;
  auto left_offset = LeftCornerContext(-0.05, 0.012, 0.0);
  auto right_offset = LeftCornerContext(-0.05, -0.012, 0.0);
  left_controller.begin(0.0);
  right_controller.begin(0.0);

  left_controller.update(left_offset, 0.30);
  right_controller.update(right_offset, 0.30);
  const auto left_decision = left_controller.update(left_offset, 0.36);
  const auto right_decision = right_controller.update(right_offset, 0.36);

  EXPECT_GT(left_decision.command.linear_x, 0.0);
  EXPECT_GT(right_decision.command.linear_x, 0.0);
  EXPECT_LT(left_decision.command.angular_z,
            right_decision.command.angular_z);
  EXPECT_EQ(left_controller.planned_command_count(), 0U);
  EXPECT_EQ(right_controller.planned_command_count(), 0U);
}

TEST(CornerShunt, OrdinaryTurnUpdatesSteeringFromMeasuredYaw) {
  visual_navigation::CornerShuntController early_controller;
  visual_navigation::CornerShuntController late_controller;
  auto early = LeftCornerContext(-0.040, 0.0, 0.35);
  auto late = LeftCornerContext(-0.021584, 0.059336, 1.2453);
  early_controller.begin(0.0);
  late_controller.begin(0.0);

  const auto entry = LeftCornerContext(-0.05, 0.0, 0.0);
  early_controller.update(entry, 0.30);
  late_controller.update(entry, 0.30);
  early_controller.update(entry, 0.36);
  late_controller.update(entry, 0.36);
  ASSERT_TRUE(early_controller.pose_is_safe(early, early.pose));
  ASSERT_TRUE(late_controller.pose_is_safe(late, late.pose));
  const auto early_decision = early_controller.update(early, 0.40);
  const auto late_decision = late_controller.update(late, 0.40);

  EXPECT_GT(early_decision.command.linear_x, 0.0);
  EXPECT_GT(late_decision.command.linear_x, 0.0);
  EXPECT_GT(early_decision.command.angular_z,
            late_decision.command.angular_z);
}

TEST(CornerShunt, OrdinaryTurnCannotReenterEntryAlignment) {
  visual_navigation::CornerShuntController controller;
  const auto entry = LeftCornerContext(-0.05, 0.0, 0.0);
  auto committed = LeftCornerContext(-0.040, 0.0, 0.35);
  auto drifted_back = LeftCornerContext(-0.050, 0.0, 0.35);
  controller.begin(0.0);

  controller.update(entry, 0.30);
  controller.update(entry, 0.36);
  const auto turning = controller.update(committed, 0.40);
  const auto after_drift = controller.update(drifted_back, 0.44);

  ASSERT_FALSE(turning.blocked);
  ASSERT_FALSE(after_drift.blocked);
  EXPECT_GT(turning.command.angular_z, 0.0);
  EXPECT_GE(after_drift.command.angular_z, 0.0);
}

TEST(CornerShunt, DelayedDrivetrainDoesNotOscillateAcrossEntryLine) {
  visual_navigation::CornerShuntController controller;
  auto context = LeftCornerContext(-0.05, 0.0, 0.0);
  context.measured_yaw_rate_valid = true;
  controller.begin(0.0);

  constexpr double kStep = 1.0 / 30.0;
  constexpr double kResponseTime = 0.20;
  visual_navigation::CornerShuntTwist measured;
  int last_sign = 0;
  int sign_changes = 0;
  bool complete = false;
  double now = 0.0;
  for (int index = 0; index < 1800; ++index) {
    context.measured_twist = measured;
    const auto decision = controller.update(context, now);
    ASSERT_FALSE(decision.blocked);
    if (decision.complete) {
      complete = true;
      break;
    }
    const int sign = decision.command.angular_z > 0.02 ? 1 :
      (decision.command.angular_z < -0.02 ? -1 : 0);
    if (sign != 0 && last_sign != 0 && sign != last_sign)
      ++sign_changes;
    if (sign != 0)
      last_sign = sign;

    context.applied_twist = decision.command;
    const double alpha = kStep / (kResponseTime + kStep);
    measured.linear_x += alpha *
      (decision.command.linear_x - measured.linear_x);
    measured.angular_z += alpha *
      (decision.command.angular_z - measured.angular_z);
    context.pose = Integrate(context.pose, measured, kStep);
    ASSERT_TRUE(controller.pose_is_safe(context, context.pose));
    now += kStep;
  }

  EXPECT_TRUE(complete);
  EXPECT_LE(sign_changes, 1);
}

TEST(CornerShunt, StalledManeuverReplansThenReturnsBoundedFault) {
  visual_navigation::CornerShuntConfig config;
  config.max_plan_count = 2;
  config.no_progress_timeout = 0.5;
  visual_navigation::CornerShuntController controller(config);
  const auto context = LeftCornerContext(-0.05, 0.0, 0.0);
  controller.begin(0.0);

  double now = 0.0;
  visual_navigation::CornerShuntDecision decision;
  for (int index = 0; index < 1000; ++index) {
    decision = controller.update(context, now);
    if (decision.blocked)
      break;
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (decision.command.linear_x > 0.0 ||
        std::abs(decision.command.angular_z) > 0.0) {
      now += config.no_progress_timeout + 0.01;
    } else {
      now += 0.05;
    }
  }

  EXPECT_TRUE(decision.blocked);
  EXPECT_EQ(controller.plan_count(), config.max_plan_count);
}

TEST(CornerShunt, ReverseArrivalCanLeaveDeadEndJunctionSafely) {
  visual_navigation::CornerShuntController controller;
  auto context = ReverseArrivalContext();
  controller.begin(0.0);

  bool complete = false;
  bool blocked = false;
  double now = 0.0;
  constexpr double kStep = 1.0 / 30.0;
  for (int index = 0; index < 1800; ++index) {
    const auto decision = controller.update(context, now);
    complete = decision.complete;
    blocked = decision.blocked;
    if (complete || blocked)
      break;
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ApplyLimitedDecision(&context, decision, kStep);
    now += kStep;
  }

  EXPECT_TRUE(complete) << "pose=(" << context.pose.x << ", " << context.pose.y
                        << ", " << context.pose.yaw
                        << ") plans=" << controller.plan_count();
  EXPECT_FALSE(blocked);
  EXPECT_TRUE(controller.pose_is_safe(context, context.pose));
}

TEST(CornerShunt, LateTurnPoseCanReplanToSafeExit) {
  visual_navigation::CornerShuntController controller;
  auto context = LeftCornerContext(-0.021584, 0.059336, 1.2453);
  controller.begin(0.0);

  bool complete = false;
  bool blocked = false;
  double now = 0.0;
  constexpr double kStep = 1.0 / 30.0;
  for (int index = 0; index < 1800; ++index) {
    const auto decision = controller.update(context, now);
    complete = decision.complete;
    blocked = decision.blocked;
    if (complete || blocked)
      break;
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ApplyLimitedDecision(&context, decision, kStep);
    now += kStep;
  }

  EXPECT_TRUE(complete) << "pose=(" << context.pose.x << ", " << context.pose.y
                        << ", " << context.pose.yaw << ")";
  EXPECT_FALSE(blocked);
  EXPECT_TRUE(controller.pose_is_safe(context, context.pose));
  EXPECT_LE(controller.plan_count(), 2);
}

TEST(CornerShunt, RecordedOffsetRightTurnReachesSafeExit) {
  visual_navigation::CornerShuntController controller;
  auto context = RightCornerContext(-0.075, 0.005, -0.021);
  controller.begin(0.0);

  bool complete = false;
  bool blocked = false;
  double now = 0.0;
  constexpr double kStep = 1.0 / 30.0;
  for (int index = 0; index < 1800; ++index) {
    const auto decision = controller.update(context, now);
    complete = decision.complete;
    blocked = decision.blocked;
    if (complete || blocked)
      break;
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ApplyLimitedDecision(&context, decision, kStep);
    now += kStep;
  }

  EXPECT_TRUE(complete) << "pose=(" << context.pose.x << ", " << context.pose.y
                        << ", " << context.pose.yaw << ")";
  EXPECT_FALSE(blocked);
  EXPECT_TRUE(controller.pose_is_safe(context, context.pose));
  EXPECT_LE(controller.plan_count(), 2);
}

TEST(CornerShunt, RecordedTangentPoseFindsFineRecovery) {
  visual_navigation::CornerShuntController controller;
  auto context = LeftCornerContext(-0.02521632034127061, 0.05001098729551654,
                                   1.239131515872503);
  controller.begin(0.0);

  bool complete = false;
  bool blocked = false;
  double now = 0.0;
  constexpr double kStep = 1.0 / 30.0;
  for (int index = 0; index < 1800; ++index) {
    const auto decision = controller.update(context, now);
    complete = decision.complete;
    blocked = decision.blocked;
    if (complete || blocked)
      break;
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ApplyLimitedDecision(&context, decision, kStep);
    now += kStep;
  }

  EXPECT_TRUE(complete) << "pose=(" << context.pose.x << ", " << context.pose.y
                        << ", " << context.pose.yaw << ")";
  EXPECT_FALSE(blocked);
  EXPECT_TRUE(controller.pose_is_safe(context, context.pose));
  EXPECT_LE(controller.plan_count(), 2);
}

TEST(CornerShunt, OutsideTangentPoseFailsClosed) {
  visual_navigation::CornerShuntController controller;
  auto context = LeftCornerContext(-0.0500, 0.0506, 1.248);
  ASSERT_FALSE(controller.pose_is_safe(context, context.pose));
  const auto initial_pose = context.pose;
  controller.begin(0.0);

  double now = 0.0;
  auto decision = controller.update(context, now);
  for (int index = 0; index < 600 && !decision.blocked; ++index) {
    if (decision.phase == visual_navigation::CornerShuntPhase::CHECK)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ApplyLimitedDecision(&context, decision, 1.0 / 30.0);
    now += 1.0 / 30.0;
    decision = controller.update(context, now);
  }

  EXPECT_TRUE(decision.blocked);
  EXPECT_FALSE(decision.complete);
  EXPECT_DOUBLE_EQ(context.pose.x, initial_pose.x);
  EXPECT_DOUBLE_EQ(context.pose.y, initial_pose.y);
  EXPECT_DOUBLE_EQ(context.pose.yaw, initial_pose.yaw);
}

TEST(CornerShunt, ResetDoesNotWaitForBackgroundPlan) {
  visual_navigation::CornerShuntController controller;
  const auto context = RightCornerContext(-0.075, 0.005, -0.021);
  controller.begin(0.0);
  controller.update(context, 0.30);
  controller.update(context, 0.36);

  const auto started = std::chrono::steady_clock::now();
  controller.reset();
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started)
          .count();

  EXPECT_LT(elapsed, 0.10);
  EXPECT_FALSE(controller.active());
}
