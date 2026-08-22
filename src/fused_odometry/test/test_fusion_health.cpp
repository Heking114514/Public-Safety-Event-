#include <gtest/gtest.h>

#include "fused_odometry/fusion_health.hpp"

namespace
{

fused_odometry::FusionHealthMonitor make_monitor()
{
  return fused_odometry::FusionHealthMonitor(
    fused_odometry::MotionClassifierConfig{0.08, 0.04, 0.02, 0.6, 1.0},
    fused_odometry::AngularStallConfig{0.30, 0.10, 0.8, 1.0});
}

fused_odometry::FusionHealthInput healthy_input(double now_seconds)
{
  fused_odometry::FusionHealthInput input;
  input.now_seconds = now_seconds;
  input.initialized = true;
  input.vision = true;
  input.wheel_measurement_fresh = true;
  input.wheel_available = true;
  input.imu = true;
  input.wheel_velocity = 0.2;
  input.visual_velocity = 0.2;
  input.visual_motion_valid = true;
  return input;
}

TEST(FusionHealthMonitor, HealthySourcesProduceOneConsistentSnapshot)
{
  auto monitor = make_monitor();
  const auto result = monitor.evaluate(healthy_input(1.0), {});

  EXPECT_EQ(result.mode, fused_odometry::FusionMode::kFull);
  EXPECT_EQ(result.motion_fault, fused_odometry::MotionFault::kNone);
  EXPECT_TRUE(result.wheel_healthy);
  EXPECT_FALSE(result.angular_stalled);
}

TEST(FusionHealthMonitor, DeadReckoningUsesValidatedWheelSpeed)
{
  auto monitor = make_monitor();
  monitor.start_vision_outage(10.0);
  auto input = healthy_input(10.0);
  input.vision = false;

  monitor.evaluate(input, {});
  input.now_seconds = 10.5;
  auto result = monitor.evaluate(input, {});
  EXPECT_NEAR(result.dead_reckoning_distance, 0.10, 1e-9);

  input.wheel_available = false;
  input.now_seconds = 11.0;
  result = monitor.evaluate(input, {});
  EXPECT_NEAR(result.dead_reckoning_distance, 0.10, 1e-9);

  input.wheel_available = true;
  input.now_seconds = 11.5;
  result = monitor.evaluate(input, {});
  EXPECT_NEAR(result.dead_reckoning_distance, 0.10, 1e-9);

  input.now_seconds = 12.0;
  result = monitor.evaluate(input, {});
  EXPECT_NEAR(result.dead_reckoning_distance, 0.20, 1e-9);
}

TEST(FusionHealthMonitor, WheelOnlyDistanceLimitChangesModeWithoutFusedPose)
{
  auto monitor = make_monitor();
  monitor.start_vision_outage(0.0);
  auto input = healthy_input(0.0);
  input.vision = false;
  input.imu = false;
  input.wheel_yaw_backup = true;
  fused_odometry::FusionHealthLimits limits;
  limits.max_dead_reckoning_time = 10.0;
  limits.max_wheel_only_time = 10.0;
  limits.max_dead_reckoning_distance = 0.15;
  limits.max_wheel_only_distance = 0.15;

  monitor.evaluate(input, limits);
  input.now_seconds = 0.5;
  EXPECT_EQ(
    monitor.evaluate(input, limits).mode,
    fused_odometry::FusionMode::kWheelOnly);
  input.now_seconds = 1.0;
  EXPECT_EQ(
    monitor.evaluate(input, limits).mode,
    fused_odometry::FusionMode::kFault);
}

TEST(FusionHealthMonitor, VisualRecoveryClearsOutageState)
{
  auto monitor = make_monitor();
  monitor.start_vision_outage(2.0);
  auto input = healthy_input(2.0);
  input.vision = false;
  monitor.evaluate(input, {});
  input.now_seconds = 3.0;
  EXPECT_GT(monitor.evaluate(input, {}).dead_reckoning_distance, 0.0);

  monitor.finish_vision_outage();
  input.vision = true;
  const auto result = monitor.evaluate(input, {});
  EXPECT_DOUBLE_EQ(result.vision_outage_seconds, 0.0);
  EXPECT_DOUBLE_EQ(result.dead_reckoning_distance, 0.0);
  EXPECT_EQ(result.mode, fused_odometry::FusionMode::kFull);
}

TEST(FusionHealthMonitor, WheelSlipRemainsDiagnosticDuringHealthyVisualFusion)
{
  auto monitor = make_monitor();
  auto input = healthy_input(0.0);
  input.command_fresh = true;
  input.command_velocity = 0.2;
  input.wheel_velocity = 0.2;
  input.visual_velocity = 0.0;
  monitor.evaluate(input, {});
  input.now_seconds = 0.7;
  const auto result = monitor.evaluate(input, {});

  EXPECT_EQ(result.motion_fault, fused_odometry::MotionFault::kSlip);
  EXPECT_FALSE(result.wheel_healthy);
  EXPECT_EQ(result.mode, fused_odometry::FusionMode::kFull);
}

TEST(FusionHealthMonitor, EncoderFailureRemainsDiagnosticDuringHealthyVisualFusion)
{
  auto monitor = make_monitor();
  auto input = healthy_input(0.0);
  input.command_fresh = true;
  input.command_velocity = 0.2;
  input.wheel_velocity = 0.0;
  input.visual_velocity = 0.2;
  monitor.evaluate(input, {});
  input.now_seconds = 0.7;
  const auto result = monitor.evaluate(input, {});

  EXPECT_EQ(result.motion_fault, fused_odometry::MotionFault::kEncoderFailure);
  EXPECT_FALSE(result.wheel_healthy);
  EXPECT_EQ(result.mode, fused_odometry::FusionMode::kFull);
}

TEST(FusionHealthMonitor, MissingWheelDoesNotDegradeHealthyVisualFusion)
{
  auto monitor = make_monitor();
  auto input = healthy_input(1.0);
  input.wheel_measurement_fresh = false;
  input.wheel_available = false;
  const auto result = monitor.evaluate(input, {});

  EXPECT_EQ(result.motion_fault, fused_odometry::MotionFault::kNone);
  EXPECT_FALSE(result.wheel_healthy);
  EXPECT_EQ(result.mode, fused_odometry::FusionMode::kFull);
}

TEST(FusionHealthMonitor, MissingImuHasOneModeIndependentOfWheelHealth)
{
  auto monitor = make_monitor();
  auto input = healthy_input(1.0);
  input.imu = false;
  EXPECT_EQ(
    monitor.evaluate(input, {}).mode,
    fused_odometry::FusionMode::kNoImu);

  input.now_seconds = 1.1;
  input.wheel_measurement_fresh = false;
  input.wheel_available = false;
  EXPECT_EQ(
    monitor.evaluate(input, {}).mode,
    fused_odometry::FusionMode::kNoImu);
}

TEST(FusionHealthMonitor, MechanicalStallStillOverridesHealthyVisualFusion)
{
  auto monitor = make_monitor();
  auto input = healthy_input(0.0);
  input.command_fresh = true;
  input.command_velocity = 0.2;
  input.wheel_velocity = 0.0;
  input.visual_velocity = 0.0;
  monitor.evaluate(input, {});
  input.now_seconds = 0.7;
  const auto result = monitor.evaluate(input, {});

  EXPECT_EQ(result.motion_fault, fused_odometry::MotionFault::kStalled);
  EXPECT_EQ(result.mode, fused_odometry::FusionMode::kFaultStalled);
}

TEST(FusionHealthMonitor, AngularStallStillOverridesHealthyVisualFusion)
{
  auto monitor = make_monitor();
  auto input = healthy_input(0.0);
  input.command_fresh = true;
  input.command_yaw_rate = 0.5;
  input.visual_yaw_valid = true;
  input.visual_yaw_rate = 0.0;
  input.imu_yaw_rate = 0.0;
  monitor.evaluate(input, {});
  input.now_seconds = 0.9;
  const auto result = monitor.evaluate(input, {});

  EXPECT_TRUE(result.angular_stalled);
  EXPECT_EQ(result.angular_source_count, 2U);
  EXPECT_EQ(result.mode, fused_odometry::FusionMode::kFaultStalled);
}

}  // namespace
