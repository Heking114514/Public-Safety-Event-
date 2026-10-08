#pragma once

#include <utility>

namespace vehicle_loopback_sim
{

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Twist2d
{
  double linear_x{0.0};
  double angular_z{0.0};
};

double normalize_angle(double angle);
double clamp(double value, double minimum, double maximum);
double limit_rate(double desired, double current, double limit_per_s, double elapsed_s);
Twist2d limited_twist(
  const Twist2d & desired, const Twist2d & current, double elapsed_s,
  double max_linear_speed, double max_angular_speed,
  double max_linear_accel, double max_angular_accel);
Pose2d integrate(const Pose2d & pose, const Twist2d & twist, double elapsed_s);
Pose2d integrate_base_link(
  const Pose2d & pose, const Twist2d & twist, double elapsed_s,
  double base_offset_x_m, double base_offset_y_m);
std::pair<double, double> wheel_speeds(const Twist2d & twist, double wheel_track_m);

}  // namespace vehicle_loopback_sim
