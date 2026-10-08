#include "navigation_mission_manager/obstacles.hpp"

#include <cmath>
#include <stdexcept>

namespace navigation_mission_manager
{

bool obstacle_memory_expired(
  const std::optional<double> & observed_at, double now, double retention_s)
{
  return observed_at.has_value() && retention_s > 0.0 &&
         now - *observed_at >= retention_s;
}

Polygon2d front_obstacle_polygon(
  const Pose2d & pose, double measured_range, double sensor_forward_offset,
  double obstacle_width, double obstacle_depth)
{
  if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw) ||
    !std::isfinite(measured_range) || !std::isfinite(sensor_forward_offset) ||
    !std::isfinite(obstacle_width) || !std::isfinite(obstacle_depth))
  {
    throw std::invalid_argument("obstacle projection values must be finite");
  }
  if (measured_range <= 0.0 || sensor_forward_offset < 0.0) {
    throw std::invalid_argument("obstacle range and sensor offset are invalid");
  }
  if (obstacle_width <= 0.0 || obstacle_depth <= 0.0) {
    throw std::invalid_argument("obstacle dimensions must be positive");
  }

  const Point2d forward{std::cos(pose.yaw), std::sin(pose.yaw)};
  const Point2d side{-forward.second, forward.first};
  const double distance = sensor_forward_offset + measured_range + 0.5 * obstacle_depth;
  const Point2d center{
    pose.x + forward.first * distance,
    pose.y + forward.second * distance};
  Polygon2d result;
  for (const auto & [depth, width] :
    {Point2d{-1.0, -1.0}, Point2d{1.0, -1.0},
      Point2d{1.0, 1.0}, Point2d{-1.0, 1.0}})
  {
    result.emplace_back(
      center.first + forward.first * depth * 0.5 * obstacle_depth +
      side.first * width * 0.5 * obstacle_width,
      center.second + forward.second * depth * 0.5 * obstacle_depth +
      side.second * width * 0.5 * obstacle_width);
  }
  return result;
}

Polygon2d valid_polygon(const Polygon2d & points)
{
  if (points.size() < 3U) {
    throw std::invalid_argument("dynamic obstacle polygon is invalid");
  }
  for (const auto & point : points) {
    if (!std::isfinite(point.first) || !std::isfinite(point.second)) {
      throw std::invalid_argument("dynamic obstacle polygon is invalid");
    }
  }
  return points;
}

Point2d polygon_center(const Polygon2d & polygon)
{
  if (polygon.empty()) {
    throw std::invalid_argument("polygon must not be empty");
  }
  Point2d center{};
  for (const auto & point : polygon) {
    center.first += point.first;
    center.second += point.second;
  }
  center.first /= static_cast<double>(polygon.size());
  center.second /= static_cast<double>(polygon.size());
  return center;
}

void merge_obstacle(
  std::vector<Polygon2d> & obstacles, const Polygon2d & candidate,
  double center_tolerance)
{
  if (center_tolerance <= 0.0) {
    throw std::invalid_argument("obstacle merge tolerance must be positive");
  }
  const Point2d center = polygon_center(candidate);
  for (auto & obstacle : obstacles) {
    const Point2d existing = polygon_center(obstacle);
    if (std::hypot(existing.first - center.first, existing.second - center.second) <=
      center_tolerance)
    {
      obstacle = candidate;
      return;
    }
  }
  obstacles.push_back(candidate);
}

}  // namespace navigation_mission_manager
