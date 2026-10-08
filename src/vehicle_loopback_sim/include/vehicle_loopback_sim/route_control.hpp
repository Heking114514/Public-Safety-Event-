#pragma once

#include <vector>

#include <nav_msgs/msg/path.hpp>

namespace vehicle_loopback_sim
{

struct RoutePose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  bool turn_junction{false};
};

struct Projection
{
  double remaining{0.0};
  double cross_track{0.0};
  double heading{0.0};
  double length{0.0};
  bool valid{false};
};

double route_angle(double angle);
double route_distance(const RoutePose & left, const RoutePose & right);
Projection project(
  const RoutePose & start, const RoutePose & target, const RoutePose & pose);
std::vector<RoutePose> parse_path(const nav_msgs::msg::Path & message);

}  // namespace vehicle_loopback_sim
