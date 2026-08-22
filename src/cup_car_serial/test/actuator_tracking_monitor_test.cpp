#include <gtest/gtest.h>

#include <chrono>

#include "cup_car_serial/actuator_tracking_monitor.hpp"

namespace
{

using cup_car_serial::ActuatorTrackingMonitor;
using cup_car_serial::ActuatorTrackingState;
using cup_car_serial::WheelTrackingIssue;

ActuatorTrackingMonitor::TimePoint At(double seconds)
{
  return ActuatorTrackingMonitor::TimePoint{} +
    std::chrono::duration_cast<ActuatorTrackingMonitor::Clock::duration>(
    std::chrono::duration<double>(seconds));
}

TEST(ActuatorTrackingMonitor, IgnoresEncoderDriftBelowCommandDeadband)
{
  ActuatorTrackingMonitor monitor;

  const auto decision = monitor.Update(0.079, -0.079, -0.20, 0.20, At(10.0));

  EXPECT_TRUE(decision.healthy);
  EXPECT_FALSE(decision.command_eligible);
  EXPECT_EQ(decision.state, ActuatorTrackingState::HEALTHY);
}

TEST(ActuatorTrackingMonitor, StartupGraceAndPersistenceRejectInstantDelay)
{
  ActuatorTrackingMonitor monitor;

  auto decision = monitor.Update(0.20, 0.20, 0.0, 0.0, At(0.0));
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(decision.state, ActuatorTrackingState::STARTUP_GRACE);
  EXPECT_EQ(decision.left_issue, WheelTrackingIssue::NO_RESPONSE);
  EXPECT_EQ(decision.right_issue, WheelTrackingIssue::NO_RESPONSE);

  decision = monitor.Update(0.20, 0.20, 0.18, 0.19, At(0.8));
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(decision.state, ActuatorTrackingState::HEALTHY);

  decision = monitor.Update(0.20, 0.20, 0.0, 0.0, At(2.0));
  EXPECT_EQ(decision.state, ActuatorTrackingState::SUSPECT);
  decision = monitor.Update(0.20, 0.20, 0.0, 0.0, At(3.49));
  EXPECT_TRUE(decision.healthy);
  decision = monitor.Update(0.20, 0.20, 0.0, 0.0, At(3.50));
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.state, ActuatorTrackingState::UNHEALTHY);
}

TEST(ActuatorTrackingMonitor, DetectsSingleWheelDirectionAndSevereError)
{
  ActuatorTrackingMonitor monitor({0.08, 0.03, 0.15, 0.8, 0.0, 1.5, 0.5, 3.0, 2});

  auto decision = monitor.Update(0.20, 0.20, -0.10, 0.20, At(0.0));
  EXPECT_EQ(decision.left_issue, WheelTrackingIssue::WRONG_DIRECTION);
  EXPECT_EQ(decision.right_issue, WheelTrackingIssue::NONE);
  decision = monitor.Update(0.20, 0.20, -0.10, 0.20, At(1.5));
  EXPECT_FALSE(decision.healthy);

  monitor.Reset();
  decision = monitor.Update(0.20, 0.20, 0.40, 0.20, At(2.0));
  EXPECT_EQ(decision.left_issue, WheelTrackingIssue::SEVERE_ERROR);
  EXPECT_EQ(decision.right_issue, WheelTrackingIssue::NONE);
}

TEST(ActuatorTrackingMonitor, IsolatedBagScaleErrorRemainsHealthy)
{
  ActuatorTrackingMonitor monitor;

  monitor.Update(0.20, 0.20, 0.20, 0.20, At(0.0));
  auto decision = monitor.Update(0.20, 0.20, 0.031, 0.20, At(2.0));
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(decision.state, ActuatorTrackingState::SUSPECT);
  decision = monitor.Update(0.20, 0.20, 0.20, 0.20, At(2.1));
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(decision.state, ActuatorTrackingState::HEALTHY);
}

TEST(ActuatorTrackingMonitor, StopDwellAllowsRetryWithoutImmediateOscillation)
{
  ActuatorTrackingMonitor monitor({0.08, 0.03, 0.15, 0.8, 0.0, 1.0, 0.5, 3.0, 2});

  monitor.Update(0.20, 0.20, 0.0, 0.0, At(0.0));
  auto decision = monitor.Update(0.20, 0.20, 0.0, 0.0, At(1.0));
  ASSERT_FALSE(decision.healthy);
  EXPECT_EQ(decision.recovery_attempts, 1);

  decision = monitor.Update(0.0, 0.0, 0.0, 0.0, At(1.1));
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.state, ActuatorTrackingState::RECOVERING);
  decision = monitor.Update(0.0, 0.0, 0.0, 0.0, At(1.59));
  EXPECT_FALSE(decision.healthy);
  decision = monitor.Update(0.0, 0.0, 0.0, 0.0, At(1.60));
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(decision.recovery_attempts, 1);
}

TEST(ActuatorTrackingMonitor, PersistentFailureExhaustsFiniteRetries)
{
  ActuatorTrackingMonitor monitor({0.08, 0.03, 0.15, 0.8, 0.0, 1.0, 0.5, 3.0, 2});

  double time = 0.0;
  for (int attempt = 1; attempt <= 2; ++attempt)
  {
    monitor.Update(0.20, 0.20, 0.0, 0.0, At(time));
    auto decision = monitor.Update(0.20, 0.20, 0.0, 0.0, At(time + 1.0));
    ASSERT_EQ(decision.state, ActuatorTrackingState::UNHEALTHY);
    EXPECT_EQ(decision.recovery_attempts, attempt);
    monitor.Update(0.0, 0.0, 0.0, 0.0, At(time + 1.1));
    decision = monitor.Update(0.0, 0.0, 0.0, 0.0, At(time + 1.6));
    ASSERT_TRUE(decision.healthy);
    time += 2.0;
  }

  monitor.Update(0.20, 0.20, 0.0, 0.0, At(time));
  const auto decision = monitor.Update(0.20, 0.20, 0.0, 0.0, At(time + 1.0));
  EXPECT_FALSE(decision.healthy);
  EXPECT_EQ(decision.state, ActuatorTrackingState::LATCHED);
  EXPECT_FALSE(monitor.Update(0.0, 0.0, 0.0, 0.0, At(time + 10.0)).healthy);
}

TEST(ActuatorTrackingMonitor, SustainedGoodTrackingRearmsRetryBudget)
{
  ActuatorTrackingMonitor monitor({0.08, 0.03, 0.15, 0.8, 0.0, 1.0, 0.5, 3.0, 1});

  monitor.Update(0.20, 0.20, 0.0, 0.0, At(0.0));
  monitor.Update(0.20, 0.20, 0.0, 0.0, At(1.0));
  monitor.Update(0.0, 0.0, 0.0, 0.0, At(1.1));
  monitor.Update(0.0, 0.0, 0.0, 0.0, At(1.6));

  auto decision = monitor.Update(0.20, 0.20, 0.20, 0.20, At(2.0));
  EXPECT_EQ(decision.recovery_attempts, 1);
  decision = monitor.Update(0.20, 0.20, 0.20, 0.20, At(5.0));
  EXPECT_EQ(decision.recovery_attempts, 0);

  monitor.Update(0.20, 0.20, 0.0, 0.0, At(6.0));
  decision = monitor.Update(0.20, 0.20, 0.0, 0.0, At(7.0));
  EXPECT_EQ(decision.state, ActuatorTrackingState::UNHEALTHY);
}

TEST(ActuatorTrackingMonitor, PauseDropsPartialFaultEvidence)
{
  ActuatorTrackingMonitor monitor({0.08, 0.03, 0.15, 0.8, 0.0, 1.5, 0.5, 3.0, 2});

  monitor.Update(0.20, 0.20, 0.0, 0.0, At(0.0));
  monitor.Pause();
  const auto decision = monitor.Update(0.20, 0.20, 0.0, 0.0, At(10.0));
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(decision.state, ActuatorTrackingState::SUSPECT);
}

TEST(ActuatorTrackingMonitor, CommandReversalStartsANewFaultWindow)
{
  ActuatorTrackingMonitor monitor({0.08, 0.03, 0.15, 0.8, 0.0, 1.0, 0.5, 3.0, 2});

  monitor.Update(0.20, 0.20, 0.0, 0.0, At(0.0));
  EXPECT_EQ(
    monitor.Update(0.20, 0.20, 0.0, 0.0, At(0.8)).state,
    ActuatorTrackingState::SUSPECT);

  EXPECT_EQ(
    monitor.Update(-0.20, -0.20, 0.20, 0.20, At(0.9)).state,
    ActuatorTrackingState::SUSPECT);
  EXPECT_TRUE(monitor.Update(-0.20, -0.20, 0.20, 0.20, At(1.8)).healthy);
  EXPECT_FALSE(monitor.Update(-0.20, -0.20, 0.20, 0.20, At(1.9)).healthy);
}

TEST(ActuatorTrackingMonitor, ResetClearsLatchAndRetryHistory)
{
  ActuatorTrackingMonitor monitor({0.08, 0.03, 0.15, 0.8, 0.0, 1.0, 0.5, 3.0, 0});

  monitor.Update(0.20, 0.20, 0.0, 0.0, At(0.0));
  ASSERT_EQ(
    monitor.Update(0.20, 0.20, 0.0, 0.0, At(1.0)).state,
    ActuatorTrackingState::LATCHED);

  monitor.Reset();
  const auto decision = monitor.Update(0.20, 0.20, 0.20, 0.20, At(2.0));
  EXPECT_TRUE(decision.healthy);
  EXPECT_EQ(decision.state, ActuatorTrackingState::HEALTHY);
  EXPECT_EQ(decision.recovery_attempts, 0);
}

}  // namespace
