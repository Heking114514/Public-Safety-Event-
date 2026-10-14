#pragma once

#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace navigation_mission_manager
{

// ROS-independent protocol gate. Retired controller IDs survive mission reset:
// a transient-local sample from an aborted mission must never start recovery.
struct RecoverySession
{
  uint64_t generation{0};
  uint64_t route_id{0};
  uint64_t recovery_id{0};
  uint64_t revision{0};
  bool active{false};
  std::set<std::pair<uint64_t, uint64_t>> retired;

  bool begin(uint64_t route, uint64_t recovery, uint64_t obstacle_revision)
  {
    if (active || route == 0 || recovery == 0 ||
      retired.count({route, recovery}) != 0U)
    {
      return false;
    }
    ++generation;
    route_id = route;
    recovery_id = recovery;
    revision = obstacle_revision;
    active = true;
    return true;
  }

  bool matches(uint64_t route, uint64_t recovery, uint64_t obstacle_revision) const
  {
    return active && route == route_id && recovery == recovery_id &&
      obstacle_revision == revision;
  }

  void reset()
  {
    if (active) {
      retired.insert({route_id, recovery_id});
    }
    ++generation;
    active = false;
    route_id = recovery_id = revision = 0;
  }
};

inline bool recovery_borrows_blocked_lane(
  const std::vector<std::string> & borrowed, const std::string & resume_lane,
  const std::set<std::string> & blocked)
{
  if (blocked.count(resume_lane) != 0U) {
    return true;
  }
  for (const auto & lane : borrowed) {
    if (blocked.count(lane) != 0U) {
      return true;
    }
  }
  return false;
}

}  // namespace navigation_mission_manager
