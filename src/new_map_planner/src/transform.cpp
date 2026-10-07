#include "new_map_planner/transform.hpp"

#include "new_map_planner/geometry.hpp"

#include <cmath>

namespace new_map_planner {

Point MapToArena(const Point &point, const MapTransform &transform) {
  const double cosine = std::cos(transform.arena_yaw);
  const double sine = std::sin(transform.arena_yaw);
  return {transform.arena_origin.x + cosine * point.x - sine * point.y,
          transform.arena_origin.y + sine * point.x + cosine * point.y};
}

Point ArenaToMap(const Point &point, const MapTransform &transform) {
  const double delta_x = point.x - transform.arena_origin.x;
  const double delta_y = point.y - transform.arena_origin.y;
  const double cosine = std::cos(transform.arena_yaw);
  const double sine = std::sin(transform.arena_yaw);
  return {cosine * delta_x + sine * delta_y,
          -sine * delta_x + cosine * delta_y};
}

Pose MapToArena(const Pose &pose, const MapTransform &transform) {
  return {MapToArena(pose.position, transform),
          NormalizeAngle(pose.yaw + transform.arena_yaw)};
}

Pose ArenaToMap(const Pose &pose, const MapTransform &transform) {
  return {ArenaToMap(pose.position, transform),
          NormalizeAngle(pose.yaw - transform.arena_yaw)};
}

Polygon MapToArena(const Polygon &polygon, const MapTransform &transform) {
  Polygon converted;
  converted.points.reserve(polygon.points.size());
  for (const Point &point : polygon.points) {
    converted.points.push_back(MapToArena(point, transform));
  }
  return converted;
}

} // namespace new_map_planner
