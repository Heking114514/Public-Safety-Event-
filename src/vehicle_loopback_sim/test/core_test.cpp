#include <cmath>

#include <gtest/gtest.h>

#include "vehicle_loopback_sim/motion.hpp"
#include "vehicle_loopback_sim/obstacle.hpp"
#include "vehicle_loopback_sim/route_control.hpp"

namespace loopback = vehicle_loopback_sim;

TEST(Motion, IntegratesStraightArcAndBaseOffset)
{
  const auto straight = loopback::integrate(
    {1.0, 2.0, 0.5 * 3.14159265358979323846}, {0.2, 0.0}, 0.5);
  EXPECT_NEAR(straight.x, 1.0, 1.0e-12);
  EXPECT_NEAR(straight.y, 2.1, 1.0e-12);

  const auto arc = loopback::integrate({0.0, 0.0, 0.0}, {0.2, 1.0}, 0.5);
  EXPECT_NEAR(arc.x, 0.2 * std::sin(0.5), 1.0e-12);
  EXPECT_NEAR(arc.y, 0.2 * (1.0 - std::cos(0.5)), 1.0e-12);

  const auto base = loopback::integrate_base_link(
    {0.0, 0.0, 0.0}, {0.0, 1.0}, 0.5, 0.05, 0.0);
  EXPECT_NEAR(base.x, 0.05 * (std::cos(0.5) - 1.0), 1.0e-12);
  EXPECT_NEAR(base.y, 0.05 * std::sin(0.5), 1.0e-12);
}

TEST(Motion, AppliesLimitsAndWheelGeometry)
{
  const auto limited = loopback::limited_twist(
    {1.0, -2.0}, {}, 0.5, 0.4, 1.0, 0.2, 0.6);
  EXPECT_NEAR(limited.linear_x, 0.1, 1.0e-12);
  EXPECT_NEAR(limited.angular_z, -0.3, 1.0e-12);
  const auto wheels = loopback::wheel_speeds({0.10, 1.0}, 0.12);
  EXPECT_NEAR(wheels.first, 0.04, 1.0e-12);
  EXPECT_NEAR(wheels.second, 0.16, 1.0e-12);
}

TEST(Obstacle, DetectsForwardManualObstacle)
{
  loopback::ObstacleConfig config;
  config.sensor_x_m = 0.055;
  config.sensor_y_m = -0.015;
  config.trigger_range_m = 0.25;
  config.lateral_tolerance_m = 0.08;
  config.min_range_m = 0.03;
  config.activation_floor_m = 0.03;
  loopback::SimulatedObstacleDetector detector(
    {{{0.15, -0.10}, {0.25, -0.10}, {0.25, 0.10}, {0.15, 0.10}}}, config);
  const auto detection = detector.detect({});
  ASSERT_TRUE(detection.has_value());
  EXPECT_NEAR(detection->range_m, 0.095, 1.0e-12);
}

TEST(Obstacle, StableGateRequiresContinuousDetection)
{
  loopback::StableObstacleGate gate(2.0);
  EXPECT_FALSE(gate.update(loopback::ObstacleDetection{3, 0.4, 0.5}, 10.0).has_value());
  EXPECT_FALSE(gate.update(loopback::ObstacleDetection{3, 0.38, 0.5}, 11.9).has_value());
  const auto detection = gate.update(loopback::ObstacleDetection{3, 0.36, 0.5}, 12.0);
  ASSERT_TRUE(detection.has_value());
  EXPECT_EQ(detection->obstacle_index, 3U);
}

TEST(RouteControl, ProjectsRemainingDistanceAndLeftCrossTrack)
{
  const loopback::RoutePose start{0.0, 0.0, 0.0, false};
  const loopback::RoutePose target{1.0, 0.0, 0.0, false};
  const loopback::RoutePose pose{0.25, 0.10, 0.0, false};
  const auto projection = loopback::project(start, target, pose);
  EXPECT_TRUE(projection.valid);
  EXPECT_NEAR(projection.remaining, 0.75, 1.0e-12);
  EXPECT_NEAR(projection.cross_track, 0.10, 1.0e-12);
}

TEST(RouteControl, ParsesTurnJunctionMarker)
{
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;
  pose.pose.position.z = 1.0e-3;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);
  const auto route = loopback::parse_path(path);
  ASSERT_EQ(route.size(), 1U);
  EXPECT_TRUE(route.front().turn_junction);
}
