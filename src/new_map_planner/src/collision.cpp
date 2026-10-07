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

} // namespace

CollisionChecker::CollisionChecker(const PlannerConfig &config)
    : config_(config),
      footprint_length_(config.vehicle_length + 2.0 * config.safety_margin),
      footprint_width_(config.vehicle_width + 2.0 * config.safety_margin) {
  static_obstacles_.reserve(config.static_obstacles.size());
  for (const Rectangle &obstacle : config.static_obstacles) {
    static_obstacles_.push_back(RectanglePolygon(obstacle));
  }
}

bool CollisionChecker::footprint_in_free_space(const Polygon &footprint) const {
  for (const Point &corner : footprint.points) {
    const bool inside =
        std::any_of(config_.free_regions.begin(), config_.free_regions.end(),
                    [&corner](const Rectangle &region) {
                      return PointInRectangle(corner, region);
                    });
    if (!inside) {
      return false;
    }
  }
  return true;
}

bool CollisionChecker::footprint_hits_obstacle(
    const Polygon &footprint,
    const std::vector<Polygon> &dynamic_obstacles) const {
  for (const Polygon &obstacle : static_obstacles_) {
    if (PolygonsIntersect(footprint, obstacle)) {
      return true;
    }
  }
  for (const Polygon &obstacle : dynamic_obstacles) {
    if (PolygonsIntersect(footprint, obstacle)) {
      return true;
    }
  }
  return false;
}

bool CollisionChecker::pose_is_free(
    const Pose &pose, const std::vector<Polygon> &dynamic_obstacles) const {
  const Polygon footprint =
      FootprintPolygon(pose, footprint_length_, footprint_width_);
  return footprint_in_free_space(footprint) &&
         !footprint_hits_obstacle(footprint, dynamic_obstacles);
}

bool CollisionChecker::segment_is_free(
    const Point &begin, const Point &end,
    const std::vector<Polygon> &dynamic_obstacles) const {
  const double length = Distance(begin, end);
  if (length <= 1.0e-9) {
    return false;
  }
  const double yaw = std::atan2(end.y - begin.y, end.x - begin.x);
  const int samples = std::max(
      1, static_cast<int>(std::ceil(length / config_.collision_sample_step)));
  for (int index = 0; index <= samples; ++index) {
    const double fraction = static_cast<double>(index) / samples;
    if (!pose_is_free({Interpolate(begin, end, fraction), yaw},
                      dynamic_obstacles)) {
      return false;
    }
  }
  return true;
}

bool CollisionChecker::sweep_is_free(
    const std::vector<Pose> &poses,
    const std::vector<Polygon> &dynamic_obstacles) const {
  if (poses.empty()) {
    return false;
  }
  for (std::size_t index = 0; index < poses.size(); ++index) {
    if (!pose_is_free(poses[index], dynamic_obstacles)) {
      return false;
    }
    if (index == 0) {
      continue;
    }
    const double distance =
        Distance(poses[index - 1].position, poses[index].position);
    const double yaw_change =
        std::abs(NormalizeAngle(poses[index].yaw - poses[index - 1].yaw));
    const int translation_samples =
        static_cast<int>(std::ceil(distance / config_.collision_sample_step));
    const int rotation_samples = static_cast<int>(std::ceil(yaw_change / 0.04));
    const int samples = std::max({1, translation_samples, rotation_samples});
    for (int sample = 1; sample < samples; ++sample) {
      const double fraction = static_cast<double>(sample) / samples;
      Pose interpolated;
      interpolated.position = Interpolate(poses[index - 1].position,
                                          poses[index].position, fraction);
      interpolated.yaw = NormalizeAngle(
          poses[index - 1].yaw +
          fraction * NormalizeAngle(poses[index].yaw - poses[index - 1].yaw));
      if (!pose_is_free(interpolated, dynamic_obstacles)) {
        return false;
      }
    }
  }
  return true;
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
