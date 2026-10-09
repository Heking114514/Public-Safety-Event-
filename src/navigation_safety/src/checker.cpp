#include "navigation_safety/checker.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace navigation_safety {
namespace {

constexpr double kTolerance = 1.0e-9;

bool point_in_rectangle(const Point &point, const Rectangle &rectangle) {
  return point.x >= rectangle.minimum_x - kTolerance &&
         point.x <= rectangle.maximum_x + kTolerance &&
         point.y >= rectangle.minimum_y - kTolerance &&
         point.y <= rectangle.maximum_y + kTolerance;
}

bool corner_in_free_space(const Point &corner,
                          const std::vector<Rectangle> &free_regions) {
  return std::any_of(free_regions.begin(), free_regions.end(),
                     [&corner](const Rectangle &region) {
                       return point_in_rectangle(corner, region);
                     });
}

double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

Pose interpolate(const Pose &begin, const Pose &end, double fraction) {
  return {{begin.position.x +
               fraction * (end.position.x - begin.position.x),
           begin.position.y +
               fraction * (end.position.y - begin.position.y)},
          normalize_angle(begin.yaw +
                          fraction * normalize_angle(end.yaw - begin.yaw))};
}

} // namespace

Polygon rectangle_polygon(const Rectangle &rectangle) {
  return Polygon{{{rectangle.minimum_x, rectangle.minimum_y},
                  {rectangle.maximum_x, rectangle.minimum_y},
                  {rectangle.maximum_x, rectangle.maximum_y},
                  {rectangle.minimum_x, rectangle.maximum_y}}};
}

std::vector<Point> footprint_corners(const Pose &pose,
                                    const Footprint &footprint) {
  const double half_length = 0.5 * (footprint.length + 2.0 * footprint.margin);
  const double half_width = 0.5 * (footprint.width + 2.0 * footprint.margin);
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const Point center{
      pose.position.x + footprint.center_offset_x * cosine -
          footprint.center_offset_y * sine,
      pose.position.y + footprint.center_offset_x * sine +
          footprint.center_offset_y * cosine};
  const std::array<Point, 4> local{{{-half_length, -half_width},
                                    {half_length, -half_width},
                                    {half_length, half_width},
                                    {-half_length, half_width}}};
  std::vector<Point> corners;
  corners.reserve(local.size());
  for (const Point &point : local) {
    corners.push_back({center.x + cosine * point.x - sine * point.y,
                       center.y + sine * point.x + cosine * point.y});
  }
  return corners;
}

bool polygons_intersect(const Polygon &left, const Polygon &right,
                        double *minimum_overlap) {
  if (left.points.size() < 3 || right.points.size() < 3) {
    return false;
  }
  double smallest_overlap = std::numeric_limits<double>::infinity();
  const Polygon *polygons[] = {&left, &right};
  for (const Polygon *polygon : polygons) {
    for (std::size_t index = 0; index < polygon->points.size(); ++index) {
      const Point &begin = polygon->points[index];
      const Point &end = polygon->points[(index + 1) % polygon->points.size()];
      const double axis_x = -(end.y - begin.y);
      const double axis_y = end.x - begin.x;
      double left_min = std::numeric_limits<double>::infinity();
      double left_max = -std::numeric_limits<double>::infinity();
      double right_min = std::numeric_limits<double>::infinity();
      double right_max = -std::numeric_limits<double>::infinity();
      for (const Point &point : left.points) {
        const double projection = point.x * axis_x + point.y * axis_y;
        left_min = std::min(left_min, projection);
        left_max = std::max(left_max, projection);
      }
      for (const Point &point : right.points) {
        const double projection = point.x * axis_x + point.y * axis_y;
        right_min = std::min(right_min, projection);
        right_max = std::max(right_max, projection);
      }
      const double overlap =
          std::min(left_max, right_max) - std::max(left_min, right_min);
      if (overlap <= kTolerance) {
        return false;
      }
      const double axis_length = std::hypot(axis_x, axis_y);
      if (axis_length > kTolerance) {
        smallest_overlap =
            std::min(smallest_overlap, overlap / axis_length);
      }
    }
  }
  if (minimum_overlap != nullptr) {
    *minimum_overlap = smallest_overlap;
  }
  return true;
}

bool pose_is_safe(const Pose &pose, const Footprint &footprint,
                  const std::vector<Rectangle> &free_regions,
                  const std::vector<Polygon> &obstacles,
                  const std::vector<Polygon> &dynamic_obstacles,
                  Report *report) {
  const auto corners = footprint_corners(pose, footprint);
  for (std::size_t index = 0; index < corners.size(); ++index) {
    if (!corner_in_free_space(corners[index], free_regions)) {
      if (report != nullptr) {
        report->failure = Failure::OUTSIDE_FREE_SPACE;
        report->footprint_corner = static_cast<int>(index);
      }
      return false;
    }
  }

  const Polygon body{corners};
  const std::vector<Polygon> *obstacle_sets[] = {&obstacles,
                                                  &dynamic_obstacles};
  for (const auto *obstacle_set : obstacle_sets) {
    for (const Polygon &obstacle : *obstacle_set) {
      double overlap = 0.0;
      if (polygons_intersect(body, obstacle, &overlap)) {
        if (report != nullptr) {
          report->failure = Failure::INTERSECTS_OBSTACLE;
          report->overlap = overlap;
        }
        return false;
      }
    }
  }
  if (report != nullptr) {
    *report = {};
  }
  return true;
}

bool sweep_is_safe(const std::vector<Pose> &poses,
                   const Footprint &footprint,
                   const std::vector<Rectangle> &free_regions,
                   const std::vector<Polygon> &obstacles,
                   const std::vector<Polygon> &dynamic_obstacles,
                   double translation_step, double yaw_step, Report *report) {
  if (poses.empty()) {
    return false;
  }
  for (std::size_t index = 0; index < poses.size(); ++index) {
    Report sample_report;
    if (!pose_is_safe(poses[index], footprint, free_regions, obstacles,
                      dynamic_obstacles, &sample_report)) {
      if (report != nullptr) {
        *report = sample_report;
        report->sample_index = index;
      }
      return false;
    }
    if (index == 0) {
      continue;
    }
    const Pose &previous = poses[index - 1];
    const double distance =
        std::hypot(poses[index].position.x - previous.position.x,
                   poses[index].position.y - previous.position.y);
    const double yaw_change =
        std::abs(normalize_angle(poses[index].yaw - previous.yaw));
    const int translation_samples =
        static_cast<int>(std::ceil(distance / translation_step));
    const int rotation_samples =
        static_cast<int>(std::ceil(yaw_change / yaw_step));
    const int samples = std::max({1, translation_samples, rotation_samples});
    for (int sample = 1; sample < samples; ++sample) {
      sample_report = {};
      if (!pose_is_safe(interpolate(previous, poses[index],
                                    static_cast<double>(sample) / samples),
                        footprint, free_regions, obstacles, dynamic_obstacles,
                        &sample_report)) {
        if (report != nullptr) {
          *report = sample_report;
          report->sample_index = index;
        }
        return false;
      }
    }
  }
  if (report != nullptr) {
    *report = {};
  }
  return true;
}

} // namespace navigation_safety
