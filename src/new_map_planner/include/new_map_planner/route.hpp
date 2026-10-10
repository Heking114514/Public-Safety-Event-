#ifndef NEW_MAP_PLANNER__ROUTE_HPP_
#define NEW_MAP_PLANNER__ROUTE_HPP_

#include "new_map_planner/collision.hpp"
#include "new_map_planner/graph.hpp"
#include "new_map_planner/maneuver.hpp"
#include "new_map_planner/types.hpp"

#include <vector>

namespace new_map_planner {

class RouteCompiler {
public:
  RouteCompiler(const PlannerConfig &config, const RoadGraph &graph,
                const CollisionChecker &collision,
                const ManeuverLibrary &maneuvers);

  bool compile(const Pose &start_arena,
               const std::vector<Traversal> &traversals,
               const std::vector<Polygon> &dynamic_obstacles,
               PlanResult &result,
               bool allow_initial_recovery_reverse = false) const;

private:
  const PlannerConfig &config_;
  const RoadGraph &graph_;
  const CollisionChecker &collision_;
  const ManeuverLibrary &maneuvers_;
};

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__ROUTE_HPP_
