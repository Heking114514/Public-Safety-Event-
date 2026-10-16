#ifndef NEW_MAP_PLANNER__COVERAGE_HPP_
#define NEW_MAP_PLANNER__COVERAGE_HPP_

#include "new_map_planner/collision.hpp"
#include "new_map_planner/graph.hpp"
#include "new_map_planner/maneuver.hpp"
#include "new_map_planner/route.hpp"
#include "new_map_planner/types.hpp"

#include <vector>

namespace new_map_planner {

struct CoverageStart {
  int node{-1};
  int heading{-1};
  double cost{0.0};
  std::vector<Traversal> prefix;
};

class CoveragePlanner {
public:
  CoveragePlanner(const PlannerConfig &config, const RoadGraph &graph,
                  const CollisionChecker &collision,
                  const ManeuverLibrary &maneuvers,
                  const RouteCompiler &route_compiler);

  PlanResult plan(const PlanRequest &request, const Pose &start,
                  const std::vector<Polygon> &obstacles,
                  const std::vector<CoverageStart> &starts,
                  bool allow_tunnel_connectors) const;

private:
  PlanResult plan_mission(const PlanRequest &request, const Pose &start,
                          const std::vector<Polygon> &obstacles,
                          const std::vector<CoverageStart> &starts) const;

  const PlannerConfig &config_;
  const RoadGraph &graph_;
  const CollisionChecker &collision_;
  const ManeuverLibrary &maneuvers_;
  const RouteCompiler &route_compiler_;
};

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__COVERAGE_HPP_
