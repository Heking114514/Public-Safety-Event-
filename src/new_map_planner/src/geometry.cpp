#include "new_map_planner/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace new_map_planner {
namespace {

double Cross(const Point &a, const Point &b, const Point &c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool OnSegment(const Point &point, const Point &begin, const Point &end) {
  constexpr double tolerance = 1.0e-9;
  return std::abs(Cross(begin, end, point)) <= tolerance &&
         point.x >= std::min(begin.x, end.x) - tolerance &&
         point.x <= std::max(begin.x, end.x) + tolerance &&
         point.y >= std::min(begin.y, end.y) - tolerance &&
         point.y <= std::max(begin.y, end.y) + tolerance;
}

bool SegmentsIntersect(const Point &a, const Point &b, const Point &c,
                       const Point &d) {
  constexpr double tolerance = 1.0e-9;
  const double ab_c = Cross(a, b, c);
  const double ab_d = Cross(a, b, d);
  const double cd_a = Cross(c, d, a);
  const double cd_b = Cross(c, d, b);
  if (((ab_c > tolerance && ab_d < -tolerance) ||
       (ab_c < -tolerance && ab_d > tolerance)) &&
      ((cd_a > tolerance && cd_b < -tolerance) ||
       (cd_a < -tolerance && cd_b > tolerance))) {
    return true;
  }
  return (std::abs(ab_c) <= tolerance && OnSegment(c, a, b)) ||
         (std::abs(ab_d) <= tolerance && OnSegment(d, a, b)) ||
         (std::abs(cd_a) <= tolerance && OnSegment(a, c, d)) ||
         (std::abs(cd_b) <= tolerance && OnSegment(b, c, d));
}

} // namespace

double NormalizeAngle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}

double Distance(const Point &left, const Point &right) {
  return std::hypot(left.x - right.x, left.y - right.y);
}

double SegmentFraction(const Point &point, const Point &begin,
                       const Point &end) {
  const double dx = end.x - begin.x;
  const double dy = end.y - begin.y;
  const double squared_length = dx * dx + dy * dy;
  if (squared_length <= 1.0e-12) {
    return 0.0;
  }
  return std::clamp(((point.x - begin.x) * dx + (point.y - begin.y) * dy) /
                        squared_length,
                    0.0, 1.0);
}

Point Interpolate(const Point &begin, const Point &end, double fraction) {
  return {begin.x + (end.x - begin.x) * fraction,
          begin.y + (end.y - begin.y) * fraction};
}

Point ProjectToSegment(const Point &point, const Point &begin,
                       const Point &end) {
  return Interpolate(begin, end, SegmentFraction(point, begin, end));
}

double DistanceToSegment(const Point &point, const Point &begin,
                         const Point &end) {
  return Distance(point, ProjectToSegment(point, begin, end));
}

int CardinalHeading(double yaw) {
  int heading =
      static_cast<int>(std::llround(NormalizeAngle(yaw) / (0.5 * kPi)));
  heading %= kHeadingCount;
  if (heading < 0) {
    heading += kHeadingCount;
  }
  return heading;
}

double HeadingYaw(int heading) {
  int normalized = heading % kHeadingCount;
  if (normalized < 0) {
    normalized += kHeadingCount;
  }
  return NormalizeAngle(static_cast<double>(normalized) * 0.5 * kPi);
}

int TurnQuarterSteps(int from_heading, int to_heading) {
  int difference = (to_heading - from_heading) % kHeadingCount;
  if (difference < 0) {
    difference += kHeadingCount;
  }
  if (difference == 3) {
    return -1;
  }
  return difference;
}

bool PointInPolygon(const Point &point, const Polygon &polygon) {
  if (polygon.points.size() < 3) {
    return false;
  }
  bool inside = false;
  for (std::size_t index = 0, previous = polygon.points.size() - 1;
       index < polygon.points.size(); previous = index++) {
    const Point &a = polygon.points[previous];
    const Point &b = polygon.points[index];
    if (OnSegment(point, a, b)) {
      return true;
    }
    const bool crosses = (a.y > point.y) != (b.y > point.y);
    if (crosses) {
      const double intersection_x =
          a.x + (point.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (intersection_x >= point.x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

bool PolygonsIntersect(const Polygon &left, const Polygon &right) {
  if (left.points.size() < 3 || right.points.size() < 3) {
    return false;
  }
  for (std::size_t left_index = 0; left_index < left.points.size();
       ++left_index) {
    const Point &left_begin = left.points[left_index];
    const Point &left_end = left.points[(left_index + 1) % left.points.size()];
    for (std::size_t right_index = 0; right_index < right.points.size();
         ++right_index) {
      const Point &right_begin = right.points[right_index];
      const Point &right_end =
          right.points[(right_index + 1) % right.points.size()];
      if (SegmentsIntersect(left_begin, left_end, right_begin, right_end)) {
        return true;
      }
    }
  }
  return PointInPolygon(left.points.front(), right) ||
         PointInPolygon(right.points.front(), left);
}

Polygon RectanglePolygon(const Rectangle &rectangle) {
  return Polygon{{{rectangle.minimum_x, rectangle.minimum_y},
                  {rectangle.maximum_x, rectangle.minimum_y},
                  {rectangle.maximum_x, rectangle.maximum_y},
                  {rectangle.minimum_x, rectangle.maximum_y}}};
}

Polygon FootprintPolygon(const Pose &pose, double length, double width,
                         double base_link_offset_x) {
  const double half_length = 0.5 * length;
  const double half_width = 0.5 * width;
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const Point center{pose.position.x + base_link_offset_x * cosine,
                     pose.position.y + base_link_offset_x * sine};
  const std::array<Point, 4> local{{{-half_length, -half_width},
                                    {half_length, -half_width},
                                    {half_length, half_width},
                                    {-half_length, half_width}}};
  Polygon polygon;
  polygon.points.reserve(local.size());
  for (const Point &point : local) {
    polygon.points.push_back({center.x + cosine * point.x - sine * point.y,
                              center.y + sine * point.x + cosine * point.y});
  }
  return polygon;
}

} // namespace new_map_planner
