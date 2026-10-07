#ifndef NEW_MAP_PLANNER__PLANNER_HPP_
#define NEW_MAP_PLANNER__PLANNER_HPP_

#include "new_map_planner/collision.hpp"
#include "new_map_planner/graph.hpp"
#include "new_map_planner/maneuver.hpp"
#include "new_map_planner/route.hpp"
#include "new_map_planner/types.hpp"

#include <vector>

namespace new_map_planner {

class MapPlanner {
public:
  explicit MapPlanner(PlannerConfig config);

  PlanResult plan(const PlanRequest &request) const;
  const PlannerConfig &config() const { return config_; }
  const RoadGraph &road_graph() const { return graph_; }
  const CollisionChecker &collision_checker() const { return collision_; }

private:
  struct StartSeed {
    int node{-1};
    int heading{-1};
    double cost{0.0};
    std::vector<Traversal> prefix;
  };

  std::vector<StartSeed>
  start_seeds(const Pose &start, const std::vector<Polygon> &obstacles) const;
  PlanResult plan_targets(const PlanRequest &request, const Pose &start,
                          const std::vector<Polygon> &obstacles) const;
  PlanResult plan_coverage(const PlanRequest &request, const Pose &start,
                           const std::vector<Polygon> &obstacles,
                           bool allow_tunnel_connectors = true) const;

  PlannerConfig config_;
  RoadGraph graph_;
  CollisionChecker collision_;
  ManeuverLibrary maneuvers_;
  RouteCompiler route_compiler_;
};

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__PLANNER_HPP_
