#include "new_map_planner/geometry.hpp"

#include <algorithm>
#include <cmath>

namespace new_map_planner {

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

} // namespace new_map_planner
