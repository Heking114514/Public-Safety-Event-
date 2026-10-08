#ifndef NEW_MAP_PLANNER__MANEUVER_HPP_
#define NEW_MAP_PLANNER__MANEUVER_HPP_

#include "new_map_planner/collision.hpp"
#include "new_map_planner/types.hpp"

#include <vector>

namespace new_map_planner {

class ManeuverLibrary {
public:
  ManeuverLibrary(const PlannerConfig &config,
                  const CollisionChecker &collision);

  bool transition_is_valid(int node, int incoming_heading,
                           const DirectedLane &outgoing,
                           const std::vector<Polygon> &dynamic_obstacles) const;
  Maneuver make_turn(int node, const DirectedLane &incoming,
                     const DirectedLane &outgoing,
                     const std::vector<Polygon> &dynamic_obstacles) const;
  Maneuver make_start_turn(int node, int incoming_heading,
                           const DirectedLane &outgoing,
                           const std::vector<Polygon> &dynamic_obstacles) const;
  Maneuver make_reverse_arrival_turn(
      int node, int incoming_motion_heading, int vehicle_heading,
      const DirectedLane &outgoing,
      const std::vector<Polygon> &dynamic_obstacles) const;
  std::vector<Pose> turn_sweep(int node, int incoming_heading,
                               int outgoing_heading) const;
  bool turn_pose_is_free(const Pose &pose,
                         const std::vector<Polygon> &dynamic_obstacles) const;

private:
  const PlannerConfig &config_;
  PlannerConfig turn_config_;
  CollisionChecker collision_;
  std::vector<Pose> local_left_turn_profile_;
};

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__MANEUVER_HPP_
