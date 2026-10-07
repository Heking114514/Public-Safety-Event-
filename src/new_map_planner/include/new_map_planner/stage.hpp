#ifndef NEW_MAP_PLANNER__STAGE_HPP_
#define NEW_MAP_PLANNER__STAGE_HPP_

#include "new_map_planner/types.hpp"

namespace new_map_planner {

struct ResolvedPlanRequest {
  PlanRequest request;
  bool allow_tunnel_connectors{true};
  bool sequential_targets{false};
};

ResolvedPlanRequest ResolvePlanRequest(const PlanRequest &request,
                                       const PlannerConfig &config);

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__STAGE_HPP_
