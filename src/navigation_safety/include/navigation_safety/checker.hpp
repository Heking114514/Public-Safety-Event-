#ifndef NAVIGATION_SAFETY__CHECKER_HPP_
#define NAVIGATION_SAFETY__CHECKER_HPP_

#include <cstddef>
#include <vector>

namespace navigation_safety {

struct Point {
  double x{0.0};
  double y{0.0};
};

struct Pose {
  Point position;
  double yaw{0.0};
};

struct Polygon {
  std::vector<Point> points;
};

struct Rectangle {
  double minimum_x{0.0};
  double minimum_y{0.0};
  double maximum_x{0.0};
  double maximum_y{0.0};
};

struct Footprint {
  double length{0.0};
  double width{0.0};
  double center_offset_x{0.0};
  double center_offset_y{0.0};
  double margin{0.0};
};

enum class Failure {
  NONE,
  OUTSIDE_FREE_SPACE,
  INTERSECTS_OBSTACLE
};

struct Report {
  Failure failure{Failure::NONE};
  int footprint_corner{-1};
  double overlap{0.0};
  std::size_t sample_index{0};
};

Polygon rectangle_polygon(const Rectangle &rectangle);
std::vector<Point> footprint_corners(const Pose &pose,
                                    const Footprint &footprint);
bool polygons_intersect(const Polygon &left, const Polygon &right,
                        double *minimum_overlap = nullptr);
bool pose_is_safe(const Pose &pose, const Footprint &footprint,
                  const std::vector<Rectangle> &free_regions,
                  const std::vector<Polygon> &obstacles,
                  const std::vector<Polygon> &dynamic_obstacles,
                  Report *report = nullptr);
bool sweep_is_safe(const std::vector<Pose> &poses,
                   const Footprint &footprint,
                   const std::vector<Rectangle> &free_regions,
                   const std::vector<Polygon> &obstacles,
                   const std::vector<Polygon> &dynamic_obstacles,
                   double translation_step, double yaw_step,
                   Report *report = nullptr);

} // namespace navigation_safety

#endif // NAVIGATION_SAFETY__CHECKER_HPP_
