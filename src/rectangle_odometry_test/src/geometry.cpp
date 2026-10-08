#include "rectangle_odometry_test/geometry.hpp"

#include <cmath>
#include <stdexcept>

namespace rectangle_odometry_test
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Pose2d local_to_world(const Pose2d & origin, double local_x, double local_y)
{
  const double cosine = std::cos(origin.yaw);
  const double sine = std::sin(origin.yaw);
  return {
    origin.x + cosine * local_x - sine * local_y,
    origin.y + sine * local_x + cosine * local_y,
    origin.yaw};
}

Pose2d local_pose_to_world(
  const Pose2d & origin, double local_x, double local_y, double local_yaw)
{
  const Pose2d position = local_to_world(origin, local_x, local_y);
  return {position.x, position.y, wrap_angle(origin.yaw + local_yaw)};
}

std::vector<RectangleWaypoint> build_clockwise_rectangle(
  const Pose2d & origin, double side_length_m)
{
  if (!std::isfinite(side_length_m) || side_length_m <= 0.0) {
    throw std::invalid_argument("side_length_m must be finite and positive");
  }
  struct Target {double x; double y; double yaw; bool junction;};
  const std::vector<Target> targets{
    {side_length_m, 0.0, 0.0, true},
    {side_length_m, -side_length_m, -0.5 * kPi, true},
    {0.0, -side_length_m, kPi, true},
    {0.0, 0.0, 0.5 * kPi, false}};
  std::vector<RectangleWaypoint> result;
  result.reserve(targets.size());
  for (const auto & target : targets) {
    const Pose2d position = local_to_world(origin, target.x, target.y);
    result.push_back({
      {position.x, position.y, wrap_angle(origin.yaw + target.yaw)}, target.junction});
  }
  return result;
}

std::vector<RectangleWaypoint> build_clockwise_rounded_rectangle(
  const Pose2d & origin, double side_length_m, double corner_radius_m,
  int corner_samples, double straight_step_m)
{
  if (!std::isfinite(side_length_m) || side_length_m <= 0.0) {
    throw std::invalid_argument("side_length_m must be finite and positive");
  }
  if (!std::isfinite(corner_radius_m) || corner_radius_m <= 0.0 ||
    corner_radius_m * 2.0 >= side_length_m)
  {
    throw std::invalid_argument("corner_radius_m must be less than half side_length_m");
  }
  if (corner_samples < 2) {
    throw std::invalid_argument("corner_samples must be at least 2");
  }
  if (!std::isfinite(straight_step_m) || straight_step_m <= 0.0) {
    throw std::invalid_argument("straight_step_m must be finite and positive");
  }

  const double radius = corner_radius_m;
  const double straight = side_length_m - 2.0 * radius;
  std::vector<RectangleWaypoint> result;
  auto append = [&result, &origin](double x, double y, double yaw) {
      result.push_back({local_pose_to_world(origin, x, y, yaw), false});
    };
  auto append_straight = [&append, straight_step_m](
      double start_x, double start_y, double end_x, double end_y, double yaw) {
      const double length = std::hypot(end_x - start_x, end_y - start_y);
      const int samples = std::max(1, static_cast<int>(std::ceil(length / straight_step_m)));
      for (int sample = 1; sample <= samples; ++sample) {
        const double ratio = static_cast<double>(sample) / samples;
        append(
          start_x + (end_x - start_x) * ratio,
          start_y + (end_y - start_y) * ratio, yaw);
      }
    };
  auto append_arc = [&append, radius, corner_samples](
      double center_x, double center_y, double start_theta, double end_theta) {
      const double step = (end_theta - start_theta) / corner_samples;
      for (int sample = 1; sample <= corner_samples; ++sample) {
        const double theta = start_theta + step * sample;
        append(
          center_x + radius * std::cos(theta),
          center_y + radius * std::sin(theta),
          theta - 0.5 * kPi);
      }
    };

  append_straight(0.0, 0.0, straight, 0.0, 0.0);
  append_arc(straight, -radius, 0.5 * kPi, 0.0);
  append_straight(
    straight + radius, -radius, straight + radius, -(straight + radius), -0.5 * kPi);
  append_arc(straight, -(straight + radius), 0.0, -0.5 * kPi);
  append_straight(straight, -side_length_m, 0.0, -side_length_m, kPi);
  append_arc(0.0, -(straight + radius), -0.5 * kPi, -kPi);
  append_straight(-radius, -(straight + radius), -radius, -radius, 0.5 * kPi);
  append_arc(0.0, -radius, kPi, 0.5 * kPi);
  return result;
}

}  // namespace rectangle_odometry_test
