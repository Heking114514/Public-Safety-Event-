#include "vehicle_loopback_sim/route_control.hpp"

#include <cmath>

namespace vehicle_loopback_sim
{

double route_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double route_distance(const RoutePose & left, const RoutePose & right)
{
  return std::hypot(right.x - left.x, right.y - left.y);
}

Projection project(
  const RoutePose & start, const RoutePose & target, const RoutePose & pose)
{
  const double dx = target.x - start.x;
  const double dy = target.y - start.y;
  const double length = std::hypot(dx, dy);
  if (length <= 1.0e-6) {
    return {
      std::hypot(target.x - pose.x, target.y - pose.y), 0.0,
      std::atan2(target.y - pose.y, target.x - pose.x), 0.0, false};
  }
  const double ux = dx / length;
  const double uy = dy / length;
  const double from_start_x = pose.x - start.x;
  const double from_start_y = pose.y - start.y;
  const double progress = ux * from_start_x + uy * from_start_y;
  return {
    length - progress,
    ux * from_start_y - uy * from_start_x,
    std::atan2(dy, dx), length, true};
}

std::vector<RoutePose> parse_path(const nav_msgs::msg::Path & message)
{
  std::vector<RoutePose> route;
  for (const auto & stamped : message.poses) {
    const auto & pose = stamped.pose;
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
      !std::isfinite(pose.position.z))
    {
      continue;
    }
    const double yaw = std::atan2(
      2.0 * (pose.orientation.w * pose.orientation.z +
      pose.orientation.x * pose.orientation.y),
      1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
      pose.orientation.z * pose.orientation.z));
    route.push_back({
      pose.position.x, pose.position.y, yaw,
      std::abs(pose.position.z - 1.0e-3) <= 1.0e-6});
  }
  return route;
}

}  // namespace vehicle_loopback_sim
