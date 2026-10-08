#pragma once

#include <optional>
#include <utility>
#include <vector>

namespace navigation_mission_manager
{

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

using Point2d = std::pair<double, double>;
using Polygon2d = std::vector<Point2d>;

bool obstacle_memory_expired(
  const std::optional<double> & observed_at, double now, double retention_s);
Polygon2d front_obstacle_polygon(
  const Pose2d & pose, double measured_range, double sensor_forward_offset,
  double obstacle_width, double obstacle_depth);
Polygon2d valid_polygon(const Polygon2d & points);
void merge_obstacle(
  std::vector<Polygon2d> & obstacles, const Polygon2d & candidate,
  double center_tolerance);
Point2d polygon_center(const Polygon2d & polygon);

}  // namespace navigation_mission_manager
