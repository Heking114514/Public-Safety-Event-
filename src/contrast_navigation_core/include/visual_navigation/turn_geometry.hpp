#ifndef VISUAL_NAVIGATION__TURN_GEOMETRY_HPP_
#define VISUAL_NAVIGATION__TURN_GEOMETRY_HPP_

#include "visual_navigation/path_control.hpp"

namespace visual_navigation {

struct TurnEntryGeometry {
  double incoming_heading;
  bool out_and_back;
};

// 暂停恢复可以重建跟踪线，但转弯来路必须取规划路线中的非重复路段。
// 路线首点没有规划来路时，保留原有跟踪线回退；起步转弯另用实测航向。
template <typename WaypointSequence>
inline TurnEntryGeometry BuildTurnEntryGeometry(
    const WaypointSequence &waypoints, std::size_t waypoint_index,
    PlanarPoint tracking_start, double tracking_heading) {
  const auto &junction = waypoints[waypoint_index];
  PlanarPoint incoming_start = tracking_start;
  double incoming_heading = tracking_heading;
  for (std::size_t previous = waypoint_index; previous > 0; --previous) {
    const auto &candidate = waypoints[previous - 1];
    const double dx = junction.x - candidate.x;
    const double dy = junction.y - candidate.y;
    if (std::hypot(dx, dy) <= kPathComparisonEpsilon)
      continue;
    incoming_start = {candidate.x, candidate.y};
    incoming_heading = std::atan2(dy, dx);
    break;
  }
  const auto &next = waypoints[waypoint_index + 1];
  return {incoming_heading,
          IsOutAndBackWaypoint(incoming_start.x, incoming_start.y,
                               junction.x, junction.y, next.x, next.y,
                               0.02, 0.05)};
}

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__TURN_GEOMETRY_HPP_
