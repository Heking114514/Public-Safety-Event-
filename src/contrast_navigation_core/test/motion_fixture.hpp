#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

namespace test_motion
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

inline double normalize_angle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle <= -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

inline Pose2d integrate(const Pose2d & pose, const Twist2d & twist, double dt)
{
  if (dt <= 0.0) {
    return pose;
  }
  if (std::abs(twist.angular_z) <= 1.0e-9) {
    return {pose.x + twist.linear_x * dt * std::cos(pose.yaw),
            pose.y + twist.linear_x * dt * std::sin(pose.yaw), pose.yaw};
  }
  const double next_yaw = pose.yaw + twist.angular_z * dt;
  const double radius = twist.linear_x / twist.angular_z;
  return {pose.x + radius * (std::sin(next_yaw) - std::sin(pose.yaw)),
          pose.y - radius * (std::cos(next_yaw) - std::cos(pose.yaw)),
          normalize_angle(next_yaw)};
}

inline Pose2d integrate_base_link(
  const Pose2d & pose, const Twist2d & twist, double dt,
  double offset_x, double offset_y)
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const Pose2d wheel_pose{pose.x - cosine * offset_x + sine * offset_y,
                          pose.y - sine * offset_x - cosine * offset_y,
                          pose.yaw};
  const Pose2d next_wheel = integrate(wheel_pose, twist, dt);
  const double next_cosine = std::cos(next_wheel.yaw);
  const double next_sine = std::sin(next_wheel.yaw);
  return {next_wheel.x + next_cosine * offset_x - next_sine * offset_y,
          next_wheel.y + next_sine * offset_x + next_cosine * offset_y,
          next_wheel.yaw};
}

inline std::pair<double, double> wheel_speeds(const Twist2d & twist, double track)
{
  const double half_track = 0.5 * std::max(0.0, track);
  return {twist.linear_x - twist.angular_z * half_track,
          twist.linear_x + twist.angular_z * half_track};
}

}  // namespace test_motion
