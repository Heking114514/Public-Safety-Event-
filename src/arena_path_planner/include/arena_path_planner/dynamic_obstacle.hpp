#ifndef ARENA_PATH_PLANNER__DYNAMIC_OBSTACLE_HPP_
#define ARENA_PATH_PLANNER__DYNAMIC_OBSTACLE_HPP_

#include "arena_path_planner/planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace arena_path_planner
{

// Convert a detector polygon to the conservative axis-aligned rectangle used
// by the grid planner.  A detector can legitimately report a single centre
// point or a line while an object is being acquired; keep those observations
// instead of silently dropping them.  The minimum extent is normally the map
// resolution, so the resulting obstacle reaches at least one grid cell and
// is then covered by the planner's normal inflation radius.
inline bool DynamicObstacleToAabb(
  const std::vector<Point> & points, double minimum_extent,
  Rectangle & rectangle, std::string * rejection_reason = nullptr)
{
  const auto reject = [rejection_reason](const char * reason) {
      if (rejection_reason != nullptr) {
        *rejection_reason = reason;
      }
      return false;
    };

  if (points.empty()) {
    return reject("empty polygon");
  }

  double minimum_x = std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const Point & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      return reject("polygon contains a non-finite point");
    }
    minimum_x = std::min(minimum_x, point.x);
    minimum_y = std::min(minimum_y, point.y);
    maximum_x = std::max(maximum_x, point.x);
    maximum_y = std::max(maximum_y, point.y);
  }

  // A sub-cell observation can fall between every grid-cell centre and vanish
  // before inflation is applied. Expand each thin axis around its measured
  // centre so the obstacle always occupies at least one raster sample.
  const double extent = std::isfinite(minimum_extent) && minimum_extent > 0.0
    ? minimum_extent : 0.01;
  const double target_extent = std::max(extent, 1.0e-3);
  const auto ensure_extent = [target_extent](
      double & minimum, double & maximum) {
      const double width = maximum - minimum;
      if (!std::isfinite(width) || width >= target_extent) {
        return true;
      }
      const double centre = minimum + 0.5 * width;
      const double half_extent = 0.5 * target_extent;
      const double expanded_minimum = centre - half_extent;
      const double expanded_maximum = centre + half_extent;
      if (!std::isfinite(expanded_minimum) ||
        !std::isfinite(expanded_maximum) ||
        !(expanded_maximum > expanded_minimum))
      {
        return false;
      }
      minimum = expanded_minimum;
      maximum = expanded_maximum;
      return true;
    };
  if (!ensure_extent(minimum_x, maximum_x)) {
    return reject("thin x bound cannot be represented");
  }
  if (!ensure_extent(minimum_y, maximum_y)) {
    return reject("thin y bound cannot be represented");
  }

  if (!std::isfinite(minimum_x) || !std::isfinite(minimum_y) ||
    !std::isfinite(maximum_x) || !std::isfinite(maximum_y) ||
    !(maximum_x > minimum_x) || !(maximum_y > minimum_y))
  {
    return reject("invalid polygon bounds");
  }

  rectangle = {minimum_x, minimum_y, maximum_x, maximum_y};
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  return true;
}

}  // namespace arena_path_planner

#endif  // ARENA_PATH_PLANNER__DYNAMIC_OBSTACLE_HPP_
