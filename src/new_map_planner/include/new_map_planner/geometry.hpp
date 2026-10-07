#ifndef NEW_MAP_PLANNER__GEOMETRY_HPP_
#define NEW_MAP_PLANNER__GEOMETRY_HPP_

#include "new_map_planner/types.hpp"

#include <array>
#include <vector>

namespace new_map_planner {

double NormalizeAngle(double angle);
double Distance(const Point &left, const Point &right);
double DistanceToSegment(const Point &point, const Point &begin,
                         const Point &end);
Point ProjectToSegment(const Point &point, const Point &begin,
                       const Point &end);
double SegmentFraction(const Point &point, const Point &begin,
                       const Point &end);
int CardinalHeading(double yaw);
double HeadingYaw(int heading);
int TurnQuarterSteps(int from_heading, int to_heading);
bool PointInPolygon(const Point &point, const Polygon &polygon);
bool PolygonsIntersect(const Polygon &left, const Polygon &right);
Polygon RectanglePolygon(const Rectangle &rectangle);
Polygon FootprintPolygon(const Pose &pose, double length, double width,
                         double base_link_offset_x = 0.0);
Point Interpolate(const Point &begin, const Point &end, double fraction);

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__GEOMETRY_HPP_
