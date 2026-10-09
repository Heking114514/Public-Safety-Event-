#include "new_map_planner/collision.hpp"

#include "new_map_planner/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace new_map_planner {
namespace {

bool PointInRectangle(const Point &point, const Rectangle &rectangle) {
  constexpr double tolerance = 1.0e-9;
  return point.x >= rectangle.minimum_x - tolerance &&
         point.x <= rectangle.maximum_x + tolerance &&
         point.y >= rectangle.minimum_y - tolerance &&
         point.y <= rectangle.maximum_y + tolerance;
}

navigation_safety::Polygon ConvertPolygon(const Polygon &polygon) {
  navigation_safety::Polygon converted;
  converted.points.reserve(polygon.points.size());
  for (const Point &point : polygon.points) {
    converted.points.push_back({point.x, point.y});
  }
  return converted;
}

std::vector<navigation_safety::Polygon>
ConvertPolygons(const std::vector<Polygon> &polygons) {
  std::vector<navigation_safety::Polygon> converted;
  converted.reserve(polygons.size());
  for (const Polygon &polygon : polygons) {
    converted.push_back(ConvertPolygon(polygon));
  }
  return converted;
}

navigation_safety::Pose ConvertPose(const Pose &pose) {
  return {{pose.position.x, pose.position.y}, pose.yaw};
}

} // namespace

CollisionChecker::CollisionChecker(const PlannerConfig &config)
    : config_(config),
      footprint_{config.vehicle_length, config.vehicle_width,
                 config.footprint_center_offset_x, 0.0,
                 config.safety_margin} {
  free_regions_.reserve(config.free_regions.size());
  for (const Rectangle &region : config.free_regions) {
    free_regions_.push_back({region.minimum_x, region.minimum_y,
                             region.maximum_x, region.maximum_y});
  }
  static_obstacles_.reserve(config.static_obstacles.size());
  for (const Rectangle &obstacle : config.static_obstacles) {
    static_obstacles_.push_back(navigation_safety::rectangle_polygon(
        {obstacle.minimum_x, obstacle.minimum_y, obstacle.maximum_x,
         obstacle.maximum_y}));
  }
}

bool CollisionChecker::pose_is_free(
    const Pose &pose, const std::vector<Polygon> &dynamic_obstacles) const {
  const auto dynamic = ConvertPolygons(dynamic_obstacles);
  return navigation_safety::pose_is_safe(ConvertPose(pose), footprint_,
                                         free_regions_, static_obstacles_,
                                         dynamic);
}

bool CollisionChecker::segment_is_free(
    const Point &begin, const Point &end,
    const std::vector<Polygon> &dynamic_obstacles) const {
  return segment_is_free(begin, end,
                         std::atan2(end.y - begin.y, end.x - begin.x),
                         dynamic_obstacles);
}

bool CollisionChecker::segment_is_free(
    const Point &begin, const Point &end, double heading,
    const std::vector<Polygon> &dynamic_obstacles) const {
  const double length = Distance(begin, end);
  if (length <= 1.0e-9) {
    return false;
  }
  return sweep_is_free({{{begin.x, begin.y}, heading},
                        {{end.x, end.y}, heading}},
                       dynamic_obstacles);
}

bool CollisionChecker::sweep_is_free(
    const std::vector<Pose> &poses,
    const std::vector<Polygon> &dynamic_obstacles) const {
  if (poses.empty()) {
    return false;
  }
  std::vector<navigation_safety::Pose> converted;
  converted.reserve(poses.size());
  for (const Pose &pose : poses) {
    converted.push_back(ConvertPose(pose));
  }
  const auto dynamic = ConvertPolygons(dynamic_obstacles);
  return navigation_safety::sweep_is_safe(
      converted, footprint_, free_regions_, static_obstacles_, dynamic,
      config_.collision_sample_step, 0.04);
}

std::vector<int8_t> CollisionChecker::occupancy_data() const {
  const int columns =
      static_cast<int>(std::ceil(config_.width / config_.resolution));
  const int rows =
      static_cast<int>(std::ceil(config_.height / config_.resolution));
  std::vector<int8_t> data(static_cast<std::size_t>(columns * rows), 100);
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < columns; ++column) {
      const Point point{(static_cast<double>(column) + 0.5) *
                            config_.resolution,
                        (static_cast<double>(row) + 0.5) * config_.resolution};
      const bool inside =
          std::any_of(config_.free_regions.begin(), config_.free_regions.end(),
                      [&point](const Rectangle &region) {
                        return PointInRectangle(point, region);
                      });
      bool occupied = !inside;
      for (const Rectangle &obstacle : config_.static_obstacles) {
        occupied = occupied || PointInRectangle(point, obstacle);
      }
      data[static_cast<std::size_t>(row * columns + column)] =
          occupied ? 100 : 0;
    }
  }
  return data;
}

} // namespace new_map_planner
