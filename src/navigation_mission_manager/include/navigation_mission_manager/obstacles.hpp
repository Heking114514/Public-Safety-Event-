#pragma once

#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace navigation_mission_manager
{

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

using Point2d = std::pair<double, double>;
using Polygon2d = std::vector<Point2d>;

bool obstacle_memory_expired(
  const std::optional<double> & observed_at, double now, double retention_s);
Polygon2d front_obstacle_polygon(
  const Pose2d & pose, double measured_range, double sensor_forward_offset,
  double obstacle_width, double obstacle_depth);
Polygon2d valid_polygon(const Polygon2d & points);
void merge_obstacle(
  std::vector<Polygon2d> & obstacles, const Polygon2d & candidate,
  double center_tolerance);
Point2d polygon_center(const Polygon2d & polygon);

namespace detail
{
template<typename T, typename = void>
struct has_blocked_lane_ids : std::false_type {};
template<typename T>
struct has_blocked_lane_ids<T, std::void_t<decltype(std::declval<T &>().blocked_lane_ids)>>
: std::true_type {};

template<typename T, typename = void>
struct has_obstacle_revision : std::false_type {};
template<typename T>
struct has_obstacle_revision<T, std::void_t<decltype(std::declval<T &>().obstacle_revision)>>
: std::true_type {};

template<typename T, typename = void>
struct has_blocked_lane_ids_used : std::false_type {};
template<typename T>
struct has_blocked_lane_ids_used<
  T, std::void_t<decltype(std::declval<T &>().blocked_lane_ids_used)>> : std::true_type {};
}  // namespace detail

template<typename Request>
void fill_blocked_lane_snapshot(
  Request & request, const std::set<std::string> & blocked_lane_ids, uint64_t revision)
{
  if constexpr (detail::has_blocked_lane_ids<Request>::value) {
    request.blocked_lane_ids.assign(blocked_lane_ids.begin(), blocked_lane_ids.end());
  }
  if constexpr (detail::has_obstacle_revision<Request>::value) {
    request.obstacle_revision = revision;
  }
}

template<typename Plan>
bool obstacle_snapshot_matches(
  const Plan & plan, const std::set<std::string> & blocked_lane_ids, uint64_t revision)
{
  bool matches = true;
  if constexpr (detail::has_blocked_lane_ids_used<Plan>::value) {
    const std::vector<std::string> expected(blocked_lane_ids.begin(), blocked_lane_ids.end());
    matches = plan.blocked_lane_ids_used == expected;
  }
  if constexpr (detail::has_obstacle_revision<Plan>::value) {
    matches = matches && plan.obstacle_revision == revision;
  }
  return matches;
}

}  // namespace navigation_mission_manager
