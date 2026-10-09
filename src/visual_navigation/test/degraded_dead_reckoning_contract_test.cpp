#include <gtest/gtest.h>

#include <string>

#include "visual_navigation/fusion_health_policy.hpp"
#include "visual_navigation/navigation_supervisor.hpp"

namespace
{

visual_navigation::NavigationInputStatus HealthyInput()
{
  visual_navigation::NavigationInputStatus input;
  input.odometry_received = true;
  input.odometry_frame_valid = true;
  input.odometry_pose_valid = true;
  input.odometry_fresh = true;
  input.fusion_status_received = true;
  input.fusion_status_fresh = true;
  input.fusion_health = {true, false};
  return input;
}

TEST(DegradedDeadReckoningContract, AllowedStateKeepsNavigationDriving)
{
  const visual_navigation::FusionHealthPolicy policy(
    {"FULL", "DEGRADED_DEAD_RECKONING"});
  auto input = HealthyInput();
  input.fusion_status = "DEGRADED_DEAD_RECKONING";
  input.fusion_health = policy.Evaluate(input.fusion_status);

  EXPECT_TRUE(input.fusion_health.allowed);
  EXPECT_FALSE(input.fusion_health.fault);
  const auto start = visual_navigation::EvaluateNavigationStart(input);
  EXPECT_TRUE(start.ready);
  EXPECT_TRUE(start.failure_state.empty());

  visual_navigation::NavigationSupervisor supervisor({false});
  const auto runtime = supervisor.Evaluate(
    input, visual_navigation::NavigationSupervisor::TimePoint{});
  EXPECT_EQ(runtime.action, visual_navigation::NavigationRuntimeAction::DRIVE);
  EXPECT_TRUE(runtime.state.empty());
  EXPECT_FALSE(runtime.reset_path_control);
}

TEST(DegradedDeadReckoningContract, StateNameDoesNotCreateTurnOrMotionGate)
{
  // Fusion health is an input-source contract. It must not be translated into
  // TURN_NEED_REPLAN, TURN_BLOCKED, or motion_allowed=false by supervision.
  const visual_navigation::FusionHealthPolicy policy(
    {"DEGRADED_DEAD_RECKONING"});
  const auto health = policy.Evaluate("DEGRADED_DEAD_RECKONING");
  ASSERT_TRUE(health.allowed);

  auto input = HealthyInput();
  input.fusion_status = "DEGRADED_DEAD_RECKONING";
  input.fusion_health = health;
  visual_navigation::NavigationSupervisor supervisor({false});
  const auto runtime = supervisor.Evaluate(
    input, visual_navigation::NavigationSupervisor::TimePoint{});

  EXPECT_EQ(runtime.action, visual_navigation::NavigationRuntimeAction::DRIVE);
  EXPECT_NE(runtime.state, "TURN_NEED_REPLAN");
  EXPECT_NE(runtime.state, "FAULT_TURN_BLOCKED");
}

TEST(DegradedDeadReckoningContract, MissingOrFaultedInputsStillStop)
{
  const visual_navigation::FusionHealthPolicy policy(
    {"FULL", "DEGRADED_DEAD_RECKONING"});
  visual_navigation::NavigationSupervisor supervisor({false});
  auto input = HealthyInput();
  input.fusion_status = "DEGRADED_DEAD_RECKONING";
  input.fusion_health = policy.Evaluate(input.fusion_status);
  ASSERT_EQ(
    supervisor.Evaluate(input,
      visual_navigation::NavigationSupervisor::TimePoint{}).action,
    visual_navigation::NavigationRuntimeAction::DRIVE);

  input.fusion_status_fresh = false;
  input.fusion_health = {};
  auto runtime = supervisor.Evaluate(
    input, visual_navigation::NavigationSupervisor::TimePoint{});
  EXPECT_EQ(runtime.action, visual_navigation::NavigationRuntimeAction::STOP_AND_WAIT);

  input.fusion_status_fresh = true;
  input.fusion_status = "FAULT_IMU";
  input.fusion_health = {false, true};
  runtime = supervisor.Evaluate(
    input, visual_navigation::NavigationSupervisor::TimePoint{});
  EXPECT_EQ(runtime.action, visual_navigation::NavigationRuntimeAction::STOP_AND_LATCH);
}

}  // namespace
