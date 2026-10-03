#include "visual_navigation/corner_shunt.hpp"
#include "gtest/gtest.h"

#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;

visual_navigation::CornerShuntContext LeftCornerContext(double x, double y,
                                                        double yaw) {
  visual_navigation::CornerShuntContext context;
  context.pose = {x, y, yaw};
  context.junction_x = 0.0;
  context.junction_y = 0.0;
  context.incoming_heading = 0.0;
  context.target_heading = kPi / 2.0;
  return context;
}

visual_navigation::CornerShuntContext RightCornerContext(double x, double y,
                                                         double yaw) {
  visual_navigation::CornerShuntContext context;
  context.pose = {x, y, yaw};
  context.junction_x = 0.0;
  context.junction_y = 0.0;
  context.incoming_heading = 0.0;
  context.target_heading = -kPi / 2.0;
  return context;
}

visual_navigation::CornerShuntPose Integrate(
    visual_navigation::CornerShuntPose pose,
    const visual_navigation::CornerShuntTwist &command, double elapsed_s) {
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
  for (int index = 0; index < 5; ++index) {
    pose = Integrate(pose, command, 0.04);
    if (!controller.pose_is_safe(context, pose))
      return false;
  }
  return true;
}

} // namespace

TEST(CornerShunt, StartsWithStopThenChoosesSafeArc) {
  visual_navigation::CornerShuntController controller;
  controller.begin(0.0);

  auto decision = controller.update(LeftCornerContext(-0.07, 0.0, 0.0), 0.05);
  EXPECT_EQ(decision.phase, visual_navigation::CornerShuntPhase::STOP);
  EXPECT_DOUBLE_EQ(decision.command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(decision.command.angular_z, 0.0);

  decision = controller.update(LeftCornerContext(-0.07, 0.0, 0.0), 0.30);
  EXPECT_EQ(decision.phase, visual_navigation::CornerShuntPhase::CHECK);

  decision = controller.update(LeftCornerContext(-0.07, 0.0, 0.0), 0.36);
  EXPECT_EQ(decision.phase, visual_navigation::CornerShuntPhase::ARC);
  EXPECT_NEAR(decision.command.linear_x, 0.010, 1.0e-9);
  EXPECT_NEAR(decision.command.angular_z, 0.24, 1.0e-9);
}

TEST(CornerShunt, MirrorsRightTurns) {
  visual_navigation::CornerShuntController controller;
  controller.begin(0.0);

  controller.update(RightCornerContext(-0.07, 0.0, 0.0), 0.30);
  auto decision = controller.update(RightCornerContext(-0.07, 0.0, 0.0), 0.36);
  EXPECT_EQ(decision.phase, visual_navigation::CornerShuntPhase::ARC);
  EXPECT_NEAR(decision.command.linear_x, 0.010, 1.0e-9);
  EXPECT_NEAR(decision.command.angular_z, -0.24, 1.0e-9);
}

TEST(CornerShunt, UsesHtmlEightMillimeterSafetyMargin) {
  visual_navigation::CornerShuntController controller;
  const auto safe = LeftCornerContext(-0.07, 0.015, 0.0);
  const auto unsafe = LeftCornerContext(-0.07, 0.016, 0.0);

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

TEST(CornerShunt, DoesNotHandOffBeforeHtmlExitWindow) {
  visual_navigation::CornerShuntController controller;
  controller.begin(0.0);

  const auto decision =
      controller.update(LeftCornerContext(-0.020, 0.040, 1.40), 0.30);

  EXPECT_FALSE(decision.complete);
  EXPECT_NE(decision.phase, visual_navigation::CornerShuntPhase::COMPLETE);
}

TEST(CornerShunt, DISABLED_SimulatedWaypointStopPoseReachesHtmlExit) {
  visual_navigation::CornerShuntController controller;
  auto context = LeftCornerContext(-0.05, 0.0, 0.0);
  controller.begin(0.0);

  bool complete = false;
  bool blocked = false;
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
    context.pose = Integrate(context.pose, decision.command, kStep);
    now += kStep;
  }

  EXPECT_TRUE(complete) << "phase="
                        << visual_navigation::CornerShuntStatus(
                               last_decision.phase)
                        << " command=(" << last_decision.command.linear_x
                        << ", " << last_decision.command.angular_z
                        << ") pose=(" << context.pose.x << ", "
                        << context.pose.y << ", " << context.pose.yaw
                        << ") time=" << now << " arc_steps=" << arc_steps
                        << " check_steps=" << check_steps
                        << " stop_steps=" << stop_steps
                        << " rotate_steps=" << rotate_steps
                        << " creep_steps=" << creep_steps
                        << " reverse_steps=" << reverse_steps
                        << " idle_steps=" << idle_steps
                        << " complete_steps=" << complete_steps
                        << " blocked_steps=" << blocked_steps
                        << " other_steps=" << other_steps
                        << " angular_motion=" << angular_motion
                        << " pose_safe="
                        << controller.pose_is_safe(context, context.pose)
                        << " rotate_safe="
                        << CommandIsSafe(controller, context, {0.0, 0.20})
                        << " creep_safe="
                        << CommandIsSafe(controller, context, {0.020, 0.0})
                        << " arc_safe="
                        << CommandIsSafe(controller, context, {0.024, 0.05})
                        << " reverse_safe="
                        << CommandIsSafe(controller, context, {-0.008, 0.0});
  EXPECT_FALSE(blocked);
}
