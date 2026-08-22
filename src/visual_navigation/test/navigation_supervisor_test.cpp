#include <gtest/gtest.h>

#include <chrono>

#include "visual_navigation/navigation_supervisor.hpp"

namespace
{

using visual_navigation::NavigationInputStatus;
using visual_navigation::NavigationRuntimeAction;
using visual_navigation::NavigationSupervisor;
using visual_navigation::NavigationSupervisorConfig;

NavigationInputStatus HealthyInput()
{
  NavigationInputStatus input;
  input.odometry_received = true;
  input.odometry_frame_valid = true;
  input.odometry_pose_valid = true;
  input.odometry_fresh = true;
  input.fusion_status_received = true;
  input.fusion_status_fresh = true;
  input.fusion_status = "FULL";
  input.fusion_health = {true, false, 1.0};
  return input;
}

TEST(NavigationSupervisor, StartRequiresLocalizationBeforeActuator)
{
  auto input = HealthyInput();
  input.odometry_fresh = false;
  input.actuator_required = true;

  const auto decision = visual_navigation::EvaluateNavigationStart(input);

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(decision.failure_state, "WAITING_FOR_ODOMETRY");
}

TEST(NavigationSupervisor, StartReportsInvalidOdometryBeforeOtherInputs)
{
  auto input = HealthyInput();
  input.odometry_frame_valid = false;
  input.fusion_status_fresh = false;
  input.tracking_required = true;

  const auto decision = visual_navigation::EvaluateNavigationStart(input);

  EXPECT_FALSE(decision.ready);
  EXPECT_EQ(decision.failure_state, "FAULT_ODOMETRY_FRAME");
}

TEST(NavigationSupervisor, StartReportsStaleAndDisconnectedActuator)
{
  auto input = HealthyInput();
  input.actuator_required = true;

  auto decision = visual_navigation::EvaluateNavigationStart(input);
  EXPECT_EQ(decision.failure_state, "FAULT_ACTUATOR_STALE");

  input.actuator_status_received = true;
  input.actuator_status_fresh = true;
  decision = visual_navigation::EvaluateNavigationStart(input);
  EXPECT_EQ(decision.failure_state, "FAULT_ACTUATOR_DISCONNECTED");

  input.actuator_connected = true;
  decision = visual_navigation::EvaluateNavigationStart(input);
  EXPECT_TRUE(decision.ready);
  EXPECT_TRUE(decision.failure_state.empty());
}

TEST(NavigationSupervisor, BriefLocalizationLossDrivesAtTransientScale)
{
  NavigationSupervisor supervisor({2.0, 0.25, false});
  const auto start = NavigationSupervisor::TimePoint{};
  auto input = HealthyInput();
  EXPECT_EQ(
    supervisor.Evaluate(input, start).action,
    NavigationRuntimeAction::DRIVE);

  input.odometry_fresh = false;
  const auto decision = supervisor.Evaluate(input, start + std::chrono::milliseconds(1900));

  EXPECT_EQ(decision.action, NavigationRuntimeAction::DRIVE);
  EXPECT_EQ(decision.state, "DEGRADED_TRANSIENT_LOCALIZATION");
  EXPECT_DOUBLE_EQ(decision.fusion_health.speed_scale, 0.25);
}

TEST(NavigationSupervisor, ExpiredLocalizationGraceStopsWithoutLatching)
{
  NavigationSupervisor supervisor({2.0, 0.25, false});
  const auto start = NavigationSupervisor::TimePoint{};
  auto input = HealthyInput();
  supervisor.Evaluate(input, start);

  input.odometry_fresh = false;
  const auto decision = supervisor.Evaluate(input, start + std::chrono::milliseconds(2100));

  EXPECT_EQ(decision.action, NavigationRuntimeAction::STOP_AND_WAIT);
  EXPECT_EQ(decision.state, "WAITING_FOR_ODOMETRY");
  EXPECT_TRUE(decision.reset_path_control);
}

TEST(NavigationSupervisor, FusionFaultStopsAndLatchesImmediately)
{
  NavigationSupervisor supervisor({2.0, 0.25, false});
  const auto start = NavigationSupervisor::TimePoint{};
  auto input = HealthyInput();
  supervisor.Evaluate(input, start);

  input.fusion_status = "FAULT_STALLED";
  input.fusion_health = {false, true, 0.0};
  const auto decision = supervisor.Evaluate(input, start + std::chrono::milliseconds(10));

  EXPECT_EQ(decision.action, NavigationRuntimeAction::STOP_AND_LATCH);
  EXPECT_EQ(decision.state, "FAULT_FUSION_STATUS");
}

TEST(NavigationSupervisor, TrackingLossOnlyLatchesWhenConfiguredAfterValidRun)
{
  const auto start = NavigationSupervisor::TimePoint{};
  auto input = HealthyInput();
  input.tracking_required = true;
  input.tracking_state_received = true;
  input.tracking_state = 2;

  NavigationSupervisor waiting_supervisor({0.0, 0.25, false});
  waiting_supervisor.Evaluate(input, start);
  input.tracking_state = 3;
  EXPECT_EQ(
    waiting_supervisor.Evaluate(input, start + std::chrono::milliseconds(1)).action,
    NavigationRuntimeAction::STOP_AND_WAIT);

  input.tracking_state = 2;
  NavigationSupervisor latching_supervisor({0.0, 0.25, true});
  latching_supervisor.Evaluate(input, start);
  input.tracking_state = 3;
  EXPECT_EQ(
    latching_supervisor.Evaluate(input, start + std::chrono::milliseconds(1)).action,
    NavigationRuntimeAction::STOP_AND_LATCH);
}

TEST(NavigationSupervisor, ActuatorLossDoesNotConsumeLocalizationGrace)
{
  NavigationSupervisor supervisor({2.0, 0.25, false});
  const auto start = NavigationSupervisor::TimePoint{};
  auto input = HealthyInput();
  input.actuator_required = true;
  input.actuator_status_received = true;
  input.actuator_status_fresh = true;
  input.actuator_connected = true;
  supervisor.Evaluate(input, start);

  input.actuator_connected = false;
  input.odometry_fresh = false;
  auto decision = supervisor.Evaluate(input, start + std::chrono::seconds(10));
  EXPECT_EQ(decision.action, NavigationRuntimeAction::STOP_AND_WAIT);
  EXPECT_EQ(decision.state, "WAITING_FOR_ACTUATOR_RECOVERY");
  EXPECT_FALSE(decision.reset_path_control);

  input.actuator_connected = true;
  decision = supervisor.Evaluate(input, start + std::chrono::seconds(10));
  EXPECT_EQ(decision.action, NavigationRuntimeAction::STOP_AND_WAIT);
  EXPECT_EQ(decision.state, "WAITING_FOR_ODOMETRY");
}

TEST(NavigationSupervisor, FusionCheckCanBeDisabledWithoutChangingOtherChecks)
{
  auto input = HealthyInput();
  input.fusion_required = false;
  input.fusion_status_received = false;
  input.fusion_status_fresh = false;
  input.fusion_health = {};

  const auto start_decision = visual_navigation::EvaluateNavigationStart(input);

  EXPECT_TRUE(start_decision.ready);
  const auto health = visual_navigation::EffectiveFusionHealth(input);
  EXPECT_TRUE(health.allowed);
  EXPECT_DOUBLE_EQ(health.speed_scale, 1.0);
}

TEST(NavigationSupervisor, ResetClearsTransientLocalizationHistory)
{
  NavigationSupervisor supervisor({2.0, 0.25, false});
  const auto start = NavigationSupervisor::TimePoint{};
  auto input = HealthyInput();
  EXPECT_EQ(supervisor.Evaluate(input, start).action, NavigationRuntimeAction::DRIVE);

  supervisor.ResetLocalizationHistory();
  input.odometry_fresh = false;
  const auto decision = supervisor.Evaluate(input, start + std::chrono::milliseconds(10));

  EXPECT_EQ(decision.action, NavigationRuntimeAction::STOP_AND_WAIT);
  EXPECT_EQ(decision.state, "WAITING_FOR_ODOMETRY");
}

}  // namespace
