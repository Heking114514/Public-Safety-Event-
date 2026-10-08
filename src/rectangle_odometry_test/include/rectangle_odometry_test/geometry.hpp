#pragma once

#include <vector>

namespace rectangle_odometry_test
{

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct RectangleWaypoint
{
  Pose2d pose;
  bool turn_junction{false};
};

double wrap_angle(double angle);
Pose2d local_to_world(const Pose2d & origin, double local_x, double local_y);
Pose2d local_pose_to_world(
  const Pose2d & origin, double local_x, double local_y, double local_yaw);
std::vector<RectangleWaypoint> build_clockwise_rectangle(
  const Pose2d & origin, double side_length_m);
std::vector<RectangleWaypoint> build_clockwise_rounded_rectangle(
  const Pose2d & origin, double side_length_m, double corner_radius_m,
  int corner_samples, double straight_step_m);

}  // namespace rectangle_odometry_test
