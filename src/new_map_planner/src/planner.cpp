#include "new_map_planner/planner.hpp"

#include "new_map_planner/geometry.hpp"
#include "new_map_planner/stage.hpp"
#include "new_map_planner/transform.hpp"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace new_map_planner {

const char *PlanFailureName(PlanFailureReason reason) {
  switch (reason) {
  case PlanFailureReason::NONE:
    return "NONE";
  case PlanFailureReason::INVALID_REQUEST:
    return "INVALID_REQUEST";
  case PlanFailureReason::START_POSE_COLLISION:
    return "START_POSE_COLLISION";
  case PlanFailureReason::START_NOT_ON_ROAD:
    return "START_NOT_ON_ROAD";
  case PlanFailureReason::START_HEADING_MISMATCH:
    return "START_HEADING_MISMATCH";
  case PlanFailureReason::START_FORWARD_BLOCKED:
    return "START_FORWARD_BLOCKED";
  case PlanFailureReason::START_REQUIRES_REVERSE:
    return "START_REQUIRES_REVERSE";
  case PlanFailureReason::NO_FEASIBLE_ROUTE:
    return "NO_FEASIBLE_ROUTE";
  case PlanFailureReason::COVERAGE_INCOMPLETE:
    return "COVERAGE_INCOMPLETE";
  case PlanFailureReason::ROUTE_INVALID:
    return "ROUTE_INVALID";
  }
  return "UNKNOWN";
}

namespace {

constexpr uint64_t kNoKey = std::numeric_limits<uint64_t>::max();

struct TargetRecord {
  double cost{std::numeric_limits<double>::infinity()};
  uint64_t previous{kNoKey};
  int via{-1};
  int seed{-1};
};

struct StateRecord {
  double cost{std::numeric_limits<double>::infinity()};
  int previous{-1};
  int via{-1};
};

struct RouteStep {
  int directed_lane{-1};
  int ordinary_count{-1};
};

struct QueueEntry {
  double cost{0.0};
  uint64_t key{0};
  bool operator>(const QueueEntry &other) const { return cost > other.cost; }
};

struct SmallQueueEntry {
  double cost{0.0};
  int state{0};
  bool operator>(const SmallQueueEntry &other) const {
    return cost > other.cost;
  }
};

void AddUnique(std::vector<std::string> &values, const std::string &value) {
  if (std::find(values.begin(), values.end(), value) == values.end()) {
    values.push_back(value);
  }
}

void SetFailure(PlanResult &result, PlanFailureReason reason,
                std::string message) {
  result.failure_reason = reason;
  result.message = std::move(message);
}

const char *StartFailureMessage(PlanFailureReason reason) {
  switch (reason) {
  case PlanFailureReason::START_POSE_COLLISION:
    return "start footprint collides with the map or an obstacle";
  case PlanFailureReason::START_NOT_ON_ROAD:
    return "start pose is outside the road anchor tolerance";
  case PlanFailureReason::START_HEADING_MISMATCH:
    return "start heading is not aligned with a forward road anchor";
  case PlanFailureReason::START_FORWARD_BLOCKED:
    return "start forward connection is blocked";
  case PlanFailureReason::START_REQUIRES_REVERSE:
    return "start connection requires recovery reverse";
  default:
    return "start pose has no valid forward road anchor";
  }
}

} // namespace

MapPlanner::MapPlanner(PlannerConfig config)
    : config_(std::move(config)), graph_(config_), collision_(config_),
      maneuvers_(config_, collision_),
      route_compiler_(config_, graph_, collision_, maneuvers_) {}

MapPlanner::StartSeedResult
MapPlanner::start_seeds(const Pose &start,
                        const std::vector<Polygon> &obstacles) const {
  StartSeedResult result;
  if (!collision_.pose_is_free(start, obstacles)) {
    result.failure_reason = PlanFailureReason::START_POSE_COLLISION;
    return result;
  }

  const int start_heading = CardinalHeading(start.yaw);
  const bool cardinal_aligned =
      std::abs(NormalizeAngle(start.yaw - HeadingYaw(start_heading))) <=
      config_.start_heading_tolerance;
  bool near_road = false;
  bool heading_aligned = false;
  bool forward_candidate = false;
  bool reverse_candidate = false;
  bool forward_blocked = false;

  for (std::size_t node = 0; node < config_.nodes.size(); ++node) {
    const double distance =
        Distance(start.position, config_.nodes[node].position);
    if (distance > config_.lane_snap_distance) {
      continue;
    }
    near_road = true;
    if (!cardinal_aligned) {
      continue;
    }
    heading_aligned = true;
    StartSeed seed;
    seed.node = static_cast<int>(node);
    seed.heading = start_heading;
    seed.cost = distance;
    if (distance <= config_.node_capture_distance) {
      if (distance > 1.0e-6) {
        seed.prefix.push_back(
            {-1, start.position, config_.nodes[node].position, true});
      }
    } else {
      const double snap_heading =
          std::atan2(config_.nodes[node].position.y - start.position.y,
                     config_.nodes[node].position.x - start.position.x);
      const double motion_error =
          std::abs(NormalizeAngle(snap_heading - start.yaw));
      if (std::abs(motion_error - kPi) <= config_.start_heading_tolerance) {
        reverse_candidate = true;
        continue;
      }
      if (motion_error > config_.start_heading_tolerance) {
        continue;
      }
      forward_candidate = true;
      if (!collision_.segment_is_free(start.position,
                                      config_.nodes[node].position, start.yaw,
                                      obstacles)) {
        forward_blocked = true;
        continue;
      }
      seed.prefix.push_back(
          {-1, start.position, config_.nodes[node].position, true});
    }
    result.seeds.push_back(std::move(seed));
  }
  if (!result.seeds.empty()) {
    return result;
  }

  for (std::size_t index = 0; index < graph_.graph().directed_lanes.size();
       ++index) {
    const DirectedLane &directed = graph_.graph().directed_lanes[index];
    const Point &lane_begin = config_.nodes[directed.from].position;
    const Point &lane_end = config_.nodes[directed.to].position;
    if (DistanceToSegment(start.position, lane_begin, lane_end) >
        config_.lane_snap_distance) {
      continue;
    }
    near_road = true;
    if (std::abs(NormalizeAngle(start.yaw - HeadingYaw(directed.heading))) >
        config_.start_heading_tolerance) {
      continue;
    }
    heading_aligned = true;
    const double snap_heading = std::atan2(lane_end.y - start.position.y,
                                           lane_end.x - start.position.x);
    const double motion_error =
        std::abs(NormalizeAngle(snap_heading - start.yaw));
    if (std::abs(motion_error - kPi) <= config_.start_heading_tolerance) {
      reverse_candidate = true;
      continue;
    }
    if (motion_error > config_.start_heading_tolerance) {
      continue;
    }
    forward_candidate = true;
    if (!collision_.segment_is_free(start.position, lane_end,
                                    HeadingYaw(directed.heading), obstacles)) {
      forward_blocked = true;
      continue;
    }
    StartSeed seed;
    seed.node = directed.to;
    seed.heading = directed.heading;
    seed.cost = Distance(start.position, lane_end);
    seed.prefix.push_back(
        {static_cast<int>(index), start.position, lane_end, true});
    result.seeds.push_back(std::move(seed));
  }
  if (!result.seeds.empty()) {
    return result;
  }
  if (!near_road) {
    result.failure_reason = PlanFailureReason::START_NOT_ON_ROAD;
  } else if (!heading_aligned) {
    result.failure_reason = PlanFailureReason::START_HEADING_MISMATCH;
  } else if (forward_candidate && forward_blocked) {
    result.failure_reason = PlanFailureReason::START_FORWARD_BLOCKED;
  } else if (reverse_candidate) {
    result.failure_reason = PlanFailureReason::START_REQUIRES_REVERSE;
  } else {
    result.failure_reason = PlanFailureReason::START_HEADING_MISMATCH;
  }
  return result;
}

PlanResult MapPlanner::plan(const PlanRequest &request) const {
  ResolvedPlanRequest resolved;
  try {
    resolved = ResolvePlanRequest(request, config_);
  } catch (const std::invalid_argument &exception) {
    PlanResult invalid;
    SetFailure(invalid, PlanFailureReason::INVALID_REQUEST, exception.what());
    return invalid;
  }
  const Pose start = MapToArena(resolved.request.start_map, config_.transform);
  std::vector<Polygon> obstacles;
  obstacles.reserve(resolved.request.dynamic_obstacles_map.size());
  for (const Polygon &polygon : request.dynamic_obstacles_map) {
    if (polygon.points.size() < 3) {
      PlanResult invalid;
      SetFailure(invalid, PlanFailureReason::INVALID_REQUEST,
                 "dynamic obstacles must contain at least three points");
      return invalid;
    }
    obstacles.push_back(MapToArena(polygon, config_.transform));
  }
  if (resolved.request.mode == "targets") {
    if (resolved.sequential_targets && resolved.request.allow_partial &&
        resolved.request.target_ids.empty()) {
      resolved.request.target_ids = config_.default_targets;
    }
    if (resolved.sequential_targets && resolved.request.allow_partial &&
        !resolved.request.target_ids.empty()) {
      PlanRequest next_target = resolved.request;
      next_target.target_ids = {resolved.request.target_ids.front()};
      PlanResult result = plan_targets(next_target, start, obstacles);
      if (!result.success && resolved.request.allow_partial) {
        return plan_targets(resolved.request, start, obstacles);
      }
      if (result.success) {
        result.complete =
            result.visit_order.size() == resolved.request.target_ids.size();
        result.message = result.complete
                             ? "complete sequential target route validated"
                             : "next sequential target route validated";
      }
      return result;
    }
    return plan_targets(resolved.request, start, obstacles);
  }
  return plan_coverage(resolved.request, start, obstacles,
                       resolved.allow_tunnel_connectors);
}

PlanResult
MapPlanner::plan_targets(const PlanRequest &request, const Pose &start,
                         const std::vector<Polygon> &obstacles) const {
  PlanResult result;
  const std::vector<std::string> targets =
      request.target_ids.empty() ? config_.default_targets : request.target_ids;
  if (targets.empty()) {
    SetFailure(result, PlanFailureReason::INVALID_REQUEST,
               "target mission is empty");
    return result;
  }
  if (targets.size() > 16) {
    SetFailure(result, PlanFailureReason::INVALID_REQUEST,
               "target mission exceeds the exact planner limit of 16");
    return result;
  }
  std::vector<int> target_nodes;
  target_nodes.reserve(targets.size());
  try {
    for (const std::string &target : targets) {
      target_nodes.push_back(graph_.task_node(target));
    }
  } catch (const std::invalid_argument &exception) {
    SetFailure(result, PlanFailureReason::INVALID_REQUEST, exception.what());
    return result;
  }

  const StartSeedResult start_result = start_seeds(start, obstacles);
  if (start_result.seeds.empty()) {
    SetFailure(result, start_result.failure_reason,
               StartFailureMessage(start_result.failure_reason));
    return result;
  }
  const std::vector<StartSeed> &seeds = start_result.seeds;
  const int count_states = 3;
  const int state_count = static_cast<int>(config_.nodes.size()) *
                          kHeadingCount * count_states;
  const auto is_task_node = [this](int node) {
    return std::any_of(config_.tasks.begin(), config_.tasks.end(),
                       [node](const Task &task) { return task.node == node; });
  };
  const uint64_t full_mask = (uint64_t{1} << targets.size()) - 1U;
  const auto node_mask = [&target_nodes](int node) {
    uint64_t mask = 0;
    for (std::size_t index = 0; index < target_nodes.size(); ++index) {
      if (target_nodes[index] == node) {
        mask |= uint64_t{1} << index;
      }
    }
    return mask;
  };
  const auto key_for = [state_count, count_states](uint64_t mask, int node,
                                                   int heading, int count) {
    return mask * static_cast<uint64_t>(state_count) +
           static_cast<uint64_t>((node * kHeadingCount + heading) *
                                     count_states +
                                 count);
  };
  const auto decode = [state_count, count_states](uint64_t key, uint64_t &mask,
                                                  int &node, int &heading,
                                                  int &count) {
    mask = key / static_cast<uint64_t>(state_count);
    int state = static_cast<int>(key % static_cast<uint64_t>(state_count));
    count = state % count_states;
    state /= count_states;
    node = state / kHeadingCount;
    heading = state % kHeadingCount;
  };

  const std::size_t directed_count = graph_.graph().directed_lanes.size();
  std::vector<bool> lane_free(directed_count, false);
  std::vector<bool> transition_free(directed_count * kHeadingCount, false);
  for (std::size_t index = 0; index < directed_count; ++index) {
    const DirectedLane &directed =
        graph_.directed_lane(static_cast<int>(index));
    lane_free[index] = collision_.segment_is_free(
        config_.nodes[directed.from].position,
        config_.nodes[directed.to].position, obstacles);
    for (int heading = 0; heading < kHeadingCount; ++heading) {
      transition_free[index * kHeadingCount + heading] =
          lane_free[index] && maneuvers_.transition_is_valid(
                                  directed.from, heading, directed, obstacles);
    }
  }

  std::unordered_map<uint64_t, TargetRecord> records;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                      std::greater<QueueEntry>>
      queue;
  for (std::size_t index = 0; index < seeds.size(); ++index) {
    const StartSeed &seed = seeds[index];
    const int count = is_task_node(seed.node)
                          ? 0
                          : std::max(1, request.ordinary_nodes_since_task);
    if (count < 0 || count >= count_states) {
      continue;
    }
    const uint64_t key =
        key_for(node_mask(seed.node), seed.node, seed.heading, count);
    TargetRecord &record = records[key];
    if (seed.cost < record.cost) {
      record.cost = seed.cost;
      record.seed = static_cast<int>(index);
      queue.push({seed.cost, key});
    }
  }

  uint64_t goal_key = kNoKey;
  uint64_t best_partial_key = kNoKey;
  int best_partial_count = -1;
  double best_partial_cost = std::numeric_limits<double>::infinity();
  while (!queue.empty()) {
    const QueueEntry current = queue.top();
    queue.pop();
    const auto current_record = records.find(current.key);
    if (current_record == records.end() ||
        current.cost > current_record->second.cost + 1.0e-9) {
      continue;
    }
    uint64_t mask = 0;
    int node = -1;
    int heading = -1;
    int count = 0;
    decode(current.key, mask, node, heading, count);
    const int completed = static_cast<int>(std::bitset<64>(mask).count());
    if (completed > best_partial_count ||
        (completed == best_partial_count && current.cost < best_partial_cost)) {
      best_partial_count = completed;
      best_partial_cost = current.cost;
      best_partial_key = current.key;
    }
    if (mask == full_mask &&
        (!request.return_home || node == config_.home_node)) {
      goal_key = current.key;
      break;
    }
    for (int directed_index : graph_.outgoing(node)) {
      const DirectedLane &directed = graph_.directed_lane(directed_index);
      if (!transition_free[static_cast<std::size_t>(directed_index) *
                               kHeadingCount +
                           heading]) {
        continue;
      }
      const int next_count = is_task_node(directed.to) ? 0 : count + 1;
      if (next_count >= count_states) {
        continue;
      }
      const uint64_t next_mask = mask | node_mask(directed.to);
      const uint64_t next_key =
          key_for(next_mask, directed.to, directed.heading, next_count);
      const double turn = heading == directed.heading ? 0.0 : config_.turn_cost;
      const double candidate = current.cost + directed.length + turn;
      TargetRecord &next = records[next_key];
      if (candidate + 1.0e-9 >= next.cost) {
        continue;
      }
      next.cost = candidate;
      next.previous = current.key;
      next.via = directed_index;
      next.seed = current_record->second.seed;
      queue.push({candidate, next_key});
    }
  }

  uint64_t selected_key = goal_key;
  if (selected_key == kNoKey && request.allow_partial &&
      best_partial_count > 0) {
    selected_key = best_partial_key;
  }
  if (selected_key == kNoKey) {
    SetFailure(result, PlanFailureReason::NO_FEASIBLE_ROUTE,
               "no forward heading-feasible route reaches the requested "
               "targets");
    result.deferred_target_ids = targets;
    return result;
  }
  std::vector<RouteStep> reversed;
  uint64_t cursor = selected_key;
  while (records.at(cursor).previous != kNoKey) {
    reversed.push_back({records.at(cursor).via});
    cursor = records.at(cursor).previous;
  }
  std::reverse(reversed.begin(), reversed.end());
  const int seed_index = records.at(selected_key).seed;
  result.traversals = seeds[static_cast<std::size_t>(seed_index)].prefix;
  for (const RouteStep &step : reversed) {
    const DirectedLane &directed = graph_.directed_lane(step.directed_lane);
    result.traversals.push_back(
        {step.directed_lane, config_.nodes[directed.from].position,
         config_.nodes[directed.to].position, false, false});
  }
  for (const Traversal &traversal : result.traversals) {
    if (traversal.directed_lane < 0) {
      continue;
    }
    const DirectedLane &directed =
        graph_.directed_lane(traversal.directed_lane);
    if (config_.lanes[directed.lane].inspection) {
      AddUnique(result.covered_lane_ids, config_.lanes[directed.lane].id);
    }
  }

  uint64_t reached_mask = 0;
  int selected_node = -1;
  int selected_heading = -1;
  decode(selected_key, reached_mask, selected_node, selected_heading,
         result.ordinary_nodes_since_task);
  (void)selected_node;
  (void)selected_heading;
  std::unordered_set<std::string> ordered_targets;
  const auto record_targets = [&targets, &target_nodes, &result,
                               &ordered_targets](int node) {
    for (std::size_t index = 0; index < target_nodes.size(); ++index) {
      if (target_nodes[index] == node &&
          ordered_targets.insert(targets[index]).second) {
        result.visit_order.push_back(targets[index]);
      }
    }
  };
  record_targets(seeds[static_cast<std::size_t>(seed_index)].node);
  for (const Traversal &traversal : result.traversals) {
    if (traversal.directed_lane >= 0) {
      record_targets(graph_.directed_lane(traversal.directed_lane).to);
    }
  }
  for (std::size_t index = 0; index < targets.size(); ++index) {
    if ((reached_mask & (uint64_t{1} << index)) == 0U) {
      result.deferred_target_ids.push_back(targets[index]);
    }
  }
  result.complete = goal_key != kNoKey;
  result.success =
      route_compiler_.compile(start, result.traversals, obstacles, result);
  if (result.success) {
    result.failure_reason = PlanFailureReason::NONE;
    result.message = result.complete ? "complete target route validated"
                                     : "partial target route validated";
  } else {
    result.failure_reason = PlanFailureReason::ROUTE_INVALID;
  }
  return result;
}

PlanResult MapPlanner::plan_coverage(const PlanRequest &request,
                                     const Pose &start,
                                     const std::vector<Polygon> &obstacles,
                                     bool allow_tunnel_connectors) const {
  PlanResult best;
  std::vector<std::string> required = request.required_lane_ids.empty()
                                          ? config_.default_inspection_lanes
                                          : request.required_lane_ids;
  std::unordered_set<std::string> completed(request.completed_lane_ids.begin(),
                                            request.completed_lane_ids.end());
  required.erase(std::remove_if(required.begin(), required.end(),
                                [&completed](const std::string &lane) {
                                  return completed.count(lane) != 0;
                                }),
                 required.end());
  try {
    for (const std::string &lane : required) {
      (void)graph_.lane_index(lane);
    }
  } catch (const std::invalid_argument &exception) {
    SetFailure(best, PlanFailureReason::INVALID_REQUEST, exception.what());
    return best;
  }
  const StartSeedResult start_result = start_seeds(start, obstacles);
  if (start_result.seeds.empty()) {
    SetFailure(best, start_result.failure_reason,
               StartFailureMessage(start_result.failure_reason));
    return best;
  }
  const std::vector<StartSeed> &seeds = start_result.seeds;

  const std::size_t directed_count = graph_.graph().directed_lanes.size();
  std::vector<bool> lane_free(directed_count, false);
  std::vector<bool> transition_free(directed_count * kHeadingCount, false);
  for (std::size_t index = 0; index < directed_count; ++index) {
    const DirectedLane &directed =
        graph_.directed_lane(static_cast<int>(index));
    lane_free[index] =
        (allow_tunnel_connectors || !config_.lanes[directed.lane].tunnel) &&
        collision_.segment_is_free(config_.nodes[directed.from].position,
                                   config_.nodes[directed.to].position,
                                   obstacles);
    for (int heading = 0; heading < kHeadingCount; ++heading) {
      transition_free[index * kHeadingCount + heading] =
          lane_free[index] && maneuvers_.transition_is_valid(
                                  directed.from, heading, directed, obstacles);
    }
  }

  constexpr int count_states = 3;
  const int state_count = static_cast<int>(config_.nodes.size()) *
                          kHeadingCount * count_states;
  const auto is_task_node = [this](int node) {
    return std::any_of(config_.tasks.begin(), config_.tasks.end(),
                       [node](const Task &task) { return task.node == node; });
  };
  const auto state_for = [](int node, int heading, int count) {
    return (node * kHeadingCount + heading) * count_states + count;
  };
  const auto decode_state = [](int state, int &node, int &heading, int &count) {
    count = state % count_states;
    state /= count_states;
    node = state / kHeadingCount;
    heading = state % kHeadingCount;
  };
  const auto shortest = [this, state_count, &state_for, &decode_state,
                         &transition_free, &is_task_node](
                            int start_node, int start_heading, int start_count,
                            const std::unordered_set<int> &goal_lanes,
                            int goal_node) {
    std::vector<StateRecord> records(static_cast<std::size_t>(state_count));
    std::priority_queue<SmallQueueEntry, std::vector<SmallQueueEntry>,
                        std::greater<SmallQueueEntry>>
        queue;
    const int start_state = state_for(start_node, start_heading, start_count);
    records[start_state].cost = 0.0;
    queue.push({0.0, start_state});
    int goal_state = -1;
    int goal_previous = -1;
    int goal_via = -1;
    int goal_count = -1;
    double goal_cost = std::numeric_limits<double>::infinity();
    while (!queue.empty()) {
      const SmallQueueEntry current = queue.top();
      queue.pop();
      if (current.cost > records[current.state].cost + 1.0e-9) {
        continue;
      }
      if (!goal_lanes.empty() && current.cost >= goal_cost - 1.0e-9) {
        break;
      }
      int node = -1;
      int heading = -1;
      int count = 0;
      decode_state(current.state, node, heading, count);
      if (goal_lanes.empty() && node == goal_node) {
        goal_state = current.state;
        break;
      }
      for (int directed_index : graph_.outgoing(node)) {
        const DirectedLane &directed = graph_.directed_lane(directed_index);
        if (!transition_free[static_cast<std::size_t>(directed_index) *
                                 kHeadingCount +
                             heading]) {
          continue;
        }
        const int next_count = is_task_node(directed.to) ? 0 : count + 1;
        if (next_count >= count_states) {
          continue;
        }
        const int next_state =
            state_for(directed.to, directed.heading, next_count);
        const double turn =
            heading == directed.heading ? 0.0 : config_.turn_cost;
        const double candidate = current.cost + directed.length + turn;
        if (!goal_lanes.empty() && goal_lanes.count(directed.lane) != 0 &&
            candidate < goal_cost) {
          goal_cost = candidate;
          goal_previous = current.state;
          goal_via = directed_index;
          goal_count = next_count;
        }
        if (candidate + 1.0e-9 >= records[next_state].cost) {
          continue;
        }
        records[next_state].cost = candidate;
        records[next_state].previous = current.state;
        records[next_state].via = directed_index;
        queue.push({candidate, next_state});
      }
    }
    const int path_end = goal_lanes.empty() ? goal_state : goal_previous;
    std::vector<RouteStep> path;
    if (path_end < 0) {
      return path;
    }
    for (int state = path_end; state != start_state;
         state = records[state].previous) {
      if (state < 0 || records[state].via < 0) {
        path.clear();
        return path;
      }
      int node = -1;
      int heading = -1;
      int count = 0;
      decode_state(state, node, heading, count);
      (void)node;
      (void)heading;
      path.push_back({records[state].via, count});
    }
    std::reverse(path.begin(), path.end());
    if (!goal_lanes.empty()) {
      path.push_back({goal_via, goal_count});
    }
    return path;
  };

  for (const StartSeed &seed : seeds) {
    PlanResult candidate;
    candidate.traversals = seed.prefix;
    std::unordered_set<int> remaining;
    for (const std::string &id : required) {
      remaining.insert(graph_.lane_index(id));
    }
    int node = seed.node;
    int heading = seed.heading;
    int count =
        is_task_node(node) ? 0 : std::max(1, request.ordinary_nodes_since_task);
    if (count < 0 || count >= count_states) {
      continue;
    }
    while (!remaining.empty()) {
      const std::vector<RouteStep> path =
          shortest(node, heading, count, remaining, -1);
      if (path.empty()) {
        break;
      }
      for (const RouteStep &step : path) {
        const DirectedLane &directed = graph_.directed_lane(step.directed_lane);
        candidate.traversals.push_back(
            {step.directed_lane, config_.nodes[directed.from].position,
             config_.nodes[directed.to].position, false, false});
        remaining.erase(directed.lane);
        node = directed.to;
        if (step.ordinary_count >= 0) {
          count = step.ordinary_count;
        }
        heading = directed.heading;
      }
    }
    for (int lane : remaining) {
      candidate.deferred_lane_ids.push_back(config_.lanes[lane].id);
    }
    if (request.return_home && remaining.empty() && node != config_.home_node) {
      const std::vector<RouteStep> return_path =
          shortest(node, heading, count, {}, config_.home_node);
      if (return_path.empty()) {
        candidate.deferred_lane_ids.push_back("RETURN_HOME");
      } else {
        for (const RouteStep &step : return_path) {
          const DirectedLane &directed =
              graph_.directed_lane(step.directed_lane);
          candidate.traversals.push_back(
              {step.directed_lane, config_.nodes[directed.from].position,
               config_.nodes[directed.to].position, false, false});
          node = directed.to;
          if (step.ordinary_count >= 0) {
            count = step.ordinary_count;
          }
          heading = directed.heading;
        }
      }
    }
    for (const Traversal &traversal : candidate.traversals) {
      if (traversal.directed_lane < 0) {
        continue;
      }
      const DirectedLane &directed =
          graph_.directed_lane(traversal.directed_lane);
      if (config_.lanes[directed.lane].inspection) {
        AddUnique(candidate.covered_lane_ids, config_.lanes[directed.lane].id);
      }
    }
    const std::unordered_set<std::string> required_set(required.begin(),
                                                       required.end());
    const bool stage_progress =
        required.empty() || std::any_of(candidate.covered_lane_ids.begin(),
                                        candidate.covered_lane_ids.end(),
                                        [&required_set](const std::string &id) {
                                          return required_set.count(id) != 0;
                                        });
    candidate.complete = candidate.deferred_lane_ids.empty();
    candidate.ordinary_nodes_since_task = count;
    if (!candidate.complete && !request.allow_partial) {
      continue;
    }
    if (!candidate.complete && !stage_progress) {
      continue;
    }
    if (candidate.traversals.empty() && required.empty() &&
        (!request.return_home ||
         Distance(start.position, config_.nodes[config_.home_node].position) <=
             1.0e-6)) {
      candidate.success = true;
      candidate.collision_checked = true;
      candidate.failure_reason = PlanFailureReason::NONE;
      candidate.message = "coverage already complete; no movement required";
    } else if (!route_compiler_.compile(start, candidate.traversals, obstacles,
                                        candidate)) {
      continue;
    } else {
      candidate.success = true;
      candidate.failure_reason = PlanFailureReason::NONE;
    }
    std::unordered_set<std::string> visited_tasks;
    const auto record_tasks_at = [&candidate, &visited_tasks,
                                  this](int visited_node) {
      for (const Task &task : config_.tasks) {
        if (task.node == visited_node && visited_tasks.insert(task.id).second) {
          candidate.visit_order.push_back(task.id);
        }
      }
    };
    record_tasks_at(seed.node);
    for (const Traversal &traversal : candidate.traversals) {
      if (traversal.directed_lane >= 0) {
        record_tasks_at(graph_.directed_lane(traversal.directed_lane).to);
      }
    }
    if (request.mode == "mission") {
      for (const Task &task : config_.tasks) {
        if (visited_tasks.count(task.id) == 0) {
          candidate.deferred_target_ids.push_back(task.id);
        }
      }
      if (!candidate.deferred_target_ids.empty()) {
        candidate.complete = false;
        if (!request.allow_partial) {
          continue;
        }
      }
    }
    candidate.message = candidate.complete
                            ? "complete road coverage route validated"
                            : "partial road coverage route validated";
    if (!best.success || candidate.complete > best.complete ||
        (candidate.complete == best.complete &&
         candidate.length < best.length)) {
      best = std::move(candidate);
    }
  }
  if (!best.success) {
    SetFailure(best, PlanFailureReason::COVERAGE_INCOMPLETE,
               "no forward heading-feasible coverage route is executable");
    best.deferred_lane_ids = required;
  }
  return best;
}

} // namespace new_map_planner
