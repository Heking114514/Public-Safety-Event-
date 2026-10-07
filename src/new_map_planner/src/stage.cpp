#include "new_map_planner/stage.hpp"

#include <stdexcept>
#include <unordered_map>

namespace new_map_planner {
namespace {

std::vector<std::string> InspectionLanes(const PlannerConfig &config,
                                         bool tunnels) {
  std::unordered_map<std::string, const Lane *> lanes;
  for (const Lane &lane : config.lanes) {
    lanes.emplace(lane.id, &lane);
  }

  std::vector<std::string> selected;
  for (const std::string &id : config.default_inspection_lanes) {
    const auto found = lanes.find(id);
    if (found != lanes.end() && found->second->tunnel == tunnels) {
      selected.push_back(id);
    }
  }
  return selected;
}

void ValidateLaneClass(const PlanRequest &request, const PlannerConfig &config,
                       bool tunnels) {
  std::unordered_map<std::string, bool> lane_classes;
  for (const Lane &lane : config.lanes) {
    lane_classes.emplace(lane.id, lane.tunnel);
  }
  for (const std::string &id : request.required_lane_ids) {
    const auto found = lane_classes.find(id);
    if (found == lane_classes.end()) {
      throw std::invalid_argument("unknown lane: " + id);
    }
    if (found->second != tunnels) {
      throw std::invalid_argument(request.mode + " contains a lane from the " +
                                  (tunnels ? "non-tunnel" : "tunnel") +
                                  " stage: " + id);
    }
  }
}

} // namespace

ResolvedPlanRequest ResolvePlanRequest(const PlanRequest &request,
                                       const PlannerConfig &config) {
  ResolvedPlanRequest resolved{request, true};
  if (request.mode == "targets" || request.mode == "coverage" ||
      request.mode == "mission") {
    return resolved;
  }
  if (request.mode == "layer1") {
    resolved.request.mode = "targets";
    resolved.request.return_home = false;
    resolved.sequential_targets = true;
    return resolved;
  }
  if (request.mode == "layer2") {
    ValidateLaneClass(request, config, true);
    resolved.request.mode = "coverage";
    resolved.request.return_home = false;
    if (resolved.request.required_lane_ids.empty()) {
      resolved.request.required_lane_ids = InspectionLanes(config, true);
    }
    return resolved;
  }
  if (request.mode == "layer3") {
    ValidateLaneClass(request, config, false);
    resolved.request.mode = "coverage";
    resolved.request.return_home = true;
    if (resolved.request.required_lane_ids.empty()) {
      resolved.request.required_lane_ids = InspectionLanes(config, false);
    }
    resolved.allow_tunnel_connectors = false;
    return resolved;
  }
  throw std::invalid_argument(
      "mode must be targets, coverage, mission, layer1, layer2, or layer3");
}

} // namespace new_map_planner
