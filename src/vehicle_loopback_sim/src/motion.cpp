#include "vehicle_loopback_sim/motion.hpp"

#include <algorithm>
#include <cmath>

namespace vehicle_loopback_sim
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double clamp(double value, double minimum, double maximum)
{
  return std::clamp(value, minimum, maximum);
}

double limit_rate(double desired, double current, double limit_per_s, double elapsed_s)
{
  if (limit_per_s <= 0.0 || elapsed_s <= 0.0) {
    return desired;
  }
  const double step = limit_per_s * elapsed_s;
  return current + clamp(desired - current, -step, step);
}

Twist2d limited_twist(
  const Twist2d & desired, const Twist2d & current, double elapsed_s,
  double max_linear_speed, double max_angular_speed,
  double max_linear_accel, double max_angular_accel)
{
  double linear = desired.linear_x;
  double angular = desired.angular_z;
  if (max_linear_speed > 0.0) {
    linear = clamp(linear, -max_linear_speed, max_linear_speed);
  }
  if (max_angular_speed > 0.0) {
    angular = clamp(angular, -max_angular_speed, max_angular_speed);
  }
  return {
    limit_rate(linear, current.linear_x, max_linear_accel, elapsed_s),
    limit_rate(angular, current.angular_z, max_angular_accel, elapsed_s)};
}

Pose2d integrate(const Pose2d & pose, const Twist2d & twist, double elapsed_s)
{
  if (elapsed_s <= 0.0) {
    return pose;
  }
  if (std::abs(twist.angular_z) <= 1.0e-9) {
    return {
      pose.x + twist.linear_x * elapsed_s * std::cos(pose.yaw),
      pose.y + twist.linear_x * elapsed_s * std::sin(pose.yaw),
      pose.yaw};
  }
  const double next_yaw = pose.yaw + twist.angular_z * elapsed_s;
  const double radius = twist.linear_x / twist.angular_z;
  return {
    pose.x + radius * (std::sin(next_yaw) - std::sin(pose.yaw)),
    pose.y - radius * (std::cos(next_yaw) - std::cos(pose.yaw)),
    normalize_angle(next_yaw)};
}

Pose2d integrate_base_link(
  const Pose2d & pose, const Twist2d & twist, double elapsed_s,
  double base_offset_x_m, double base_offset_y_m)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const Pose2d wheel_pose{
    pose.x - cosine * base_offset_x_m + sine * base_offset_y_m,
    pose.y - sine * base_offset_x_m - cosine * base_offset_y_m,
    pose.yaw};
  const Pose2d next_wheel = integrate(wheel_pose, twist, elapsed_s);
  const double next_cosine = std::cos(next_wheel.yaw);
  const double next_sine = std::sin(next_wheel.yaw);
  return {
    next_wheel.x + next_cosine * base_offset_x_m - next_sine * base_offset_y_m,
    next_wheel.y + next_sine * base_offset_x_m + next_cosine * base_offset_y_m,
    next_wheel.yaw};
}

std::pair<double, double> wheel_speeds(const Twist2d & twist, double wheel_track_m)
{
  const double half_track = 0.5 * std::max(0.0, wheel_track_m);
  return {
    twist.linear_x - twist.angular_z * half_track,
    twist.linear_x + twist.angular_z * half_track};
}

}  // namespace vehicle_loopback_sim
