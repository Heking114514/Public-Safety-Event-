#include "new_map_planner/coverage.hpp"

#include "new_map_planner/geometry.hpp"

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <functional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace new_map_planner {
namespace {

constexpr std::size_t kMaximumCoverageStates = 2000000;
constexpr int kCountStates = 3;

struct SearchState {
  uint64_t mask{0};
  int node{-1};
  int heading{-1};
  int ordinary_count{0};
  int arrival_lane{-1};
};

void AddUnique(std::vector<std::string> &values, const std::string &value) {
  if (std::find(values.begin(), values.end(), value) == values.end()) {
    values.push_back(value);
  }
}

} // namespace

CoveragePlanner::CoveragePlanner(const PlannerConfig &config,
                                 const RoadGraph &graph,
                                 const CollisionChecker &collision,
                                 const ManeuverLibrary &maneuvers,
                                 const RouteCompiler &route_compiler)
    : config_(config), graph_(graph), collision_(collision),
      maneuvers_(maneuvers), route_compiler_(route_compiler) {}

PlanResult CoveragePlanner::plan(
    const PlanRequest &request, const Pose &start,
    const std::vector<Polygon> &obstacles,
    const std::vector<CoverageStart> &starts,
    bool allow_tunnel_connectors) const {
  PlanResult result;
  const std::unordered_set<std::string> blocked_lanes(
      request.blocked_lane_ids.begin(), request.blocked_lane_ids.end());
  result.validate_geometry = false;

  std::vector<std::string> required = request.required_lane_ids.empty()
                                          ? config_.default_inspection_lanes
                                          : request.required_lane_ids;
  const std::unordered_set<std::string> completed(
      request.completed_lane_ids.begin(), request.completed_lane_ids.end());
  required.erase(std::remove_if(required.begin(), required.end(),
                                [&completed](const std::string &id) {
                                  return completed.count(id) != 0;
                                }),
                 required.end());
  if (required.size() > 63U) {
    result.message = "coverage request exceeds the exact search limit of 63";
    return result;
  }

  std::unordered_map<int, int> required_bits;
  try {
    for (std::size_t index = 0; index < required.size(); ++index) {
      required_bits.emplace(graph_.lane_index(required[index]),
                            static_cast<int>(index));
    }
  } catch (const std::invalid_argument &exception) {
    result.message = exception.what();
    return result;
  }
  if (starts.empty()) {
    result.message = "start pose is not aligned with a collision-free road";
    result.deferred_lane_ids = required;
    return result;
  }

  const std::size_t directed_count = graph_.graph().directed_lanes.size();
  std::vector<bool> lane_free(directed_count, false);
  std::vector<bool> transition_free(directed_count * kHeadingCount, false);
  for (std::size_t index = 0; index < directed_count; ++index) {
    const DirectedLane &directed = graph_.directed_lane(static_cast<int>(index));
    lane_free[index] =
        (allow_tunnel_connectors || !config_.lanes[directed.lane].tunnel) &&
        blocked_lanes.count(config_.lanes[directed.lane].id) == 0;
    for (int heading = 0; heading < kHeadingCount; ++heading) {
      transition_free[index * kHeadingCount + heading] =
          lane_free[index] && TurnQuarterSteps(heading, directed.heading) != 2;
    }
  }

  const int arrival_states = static_cast<int>(config_.lanes.size()) + 1;
  const int base_state_count = static_cast<int>(config_.nodes.size()) *
                               kHeadingCount * kCountStates * arrival_states;
  const auto base_for = [arrival_states](int node, int heading, int count,
                                         int arrival_lane) {
    return (((node * kHeadingCount + heading) * kCountStates + count) *
                arrival_states +
            arrival_lane + 1);
  };
  const auto key_for = [base_state_count](uint64_t mask, int base) {
    return mask * static_cast<uint64_t>(base_state_count) +
           static_cast<uint64_t>(base);
  };
  const auto is_task_node = [this](int node) {
    return std::any_of(config_.tasks.begin(), config_.tasks.end(),
                       [node](const Task &task) { return task.node == node; });
  };
  const auto lane_mask = [&required_bits](int lane) {
    const auto found = required_bits.find(lane);
    return found == required_bits.end() ? uint64_t{0}
                                        : uint64_t{1} << found->second;
  };
  const auto forward_successors =
      [this, &transition_free, &is_task_node](const SearchState &state) {
        std::vector<int> successors;
        for (int directed_index : graph_.outgoing(state.node)) {
          const DirectedLane &directed = graph_.directed_lane(directed_index);
          if (!transition_free[static_cast<std::size_t>(directed_index) *
                                   kHeadingCount +
                               state.heading]) {
            continue;
          }
          const int next_count =
              is_task_node(directed.to) ? 0 : state.ordinary_count + 1;
          if (next_count < kCountStates) {
            successors.push_back(directed_index);
          }
        }
        return successors;
      };
  const auto can_reach_home =
      [this, &base_for, &forward_successors,
       &is_task_node](const SearchState &start_state) {
        if (start_state.node == config_.home_node) {
          return true;
        }
        std::queue<SearchState> pending;
        std::unordered_set<int> visited;
        pending.push(start_state);
        visited.insert(base_for(start_state.node, start_state.heading,
                                start_state.ordinary_count,
                                start_state.arrival_lane));
        while (!pending.empty()) {
          const SearchState current = pending.front();
          pending.pop();
          for (int directed_index : forward_successors(current)) {
            const DirectedLane &directed = graph_.directed_lane(directed_index);
            SearchState next;
            next.node = directed.to;
            next.heading = directed.heading;
            next.ordinary_count =
                is_task_node(directed.to) ? 0 : current.ordinary_count + 1;
            next.arrival_lane = directed.lane;
            if (next.node == config_.home_node) {
              return true;
            }
            const int base = base_for(next.node, next.heading,
                                      next.ordinary_count, next.arrival_lane);
            if (visited.insert(base).second) {
              pending.push(next);
            }
          }
        }
        return false;
      };

  const uint64_t full_mask = required.empty()
                                 ? uint64_t{0}
                                 : (uint64_t{1} << required.size()) - 1U;
  std::vector<int> selected_path;
  SearchState selected_state;
  int selected_seed = -1;
  bool complete_found = false;
  std::vector<int> current_path;
  int best_seed = -1;
  SearchState best_state;
  std::vector<int> best_path;
  int partial_progress = 0;
  bool search_limit_reached = false;
  std::size_t expanded = 0;
  std::unordered_set<uint64_t> failed;
  std::function<bool(const SearchState &, int)> search =
      [&](const SearchState &state, int seed_index) {
    if (++expanded > kMaximumCoverageStates) {
      search_limit_reached = true;
      return false;
    }
    const uint64_t state_key = key_for(
        state.mask,
        base_for(state.node, state.heading, state.ordinary_count,
                 state.arrival_lane));
    if (!failed.insert(state_key).second) {
      return false;
    }
    if (state.mask == full_mask &&
        (!request.return_home || state.node == config_.home_node)) {
      selected_path = current_path;
      selected_state = state;
      selected_seed = seed_index;
      complete_found = true;
      return true;
    }

    const int progress = static_cast<int>(std::bitset<64>(state.mask).count());
    const bool has_successor = !forward_successors(state).empty();
    const bool return_is_safe =
        !request.return_home || can_reach_home(state);
    if (request.allow_partial && progress > 0 && has_successor &&
        return_is_safe && progress > partial_progress) {
      best_seed = seed_index;
      best_state = state;
      best_path = current_path;
      partial_progress = progress;
    }

    std::vector<int> successors = forward_successors(state);
    std::stable_sort(successors.begin(), successors.end(),
                     [this, &state, &lane_mask](int left, int right) {
      const DirectedLane &left_lane = graph_.directed_lane(left);
      const DirectedLane &right_lane = graph_.directed_lane(right);
      const bool left_new = lane_mask(left_lane.lane) != 0U &&
                            (state.mask & lane_mask(left_lane.lane)) == 0U;
      const bool right_new = lane_mask(right_lane.lane) != 0U &&
                             (state.mask & lane_mask(right_lane.lane)) == 0U;
      if (left_new != right_new) {
        return left_new > right_new;
      }
      return left < right;
    });
    for (int directed_index : successors) {
      const DirectedLane &directed = graph_.directed_lane(directed_index);
      SearchState next;
      next.mask = state.mask | lane_mask(directed.lane);
      next.node = directed.to;
      next.heading = directed.heading;
      next.ordinary_count =
          is_task_node(directed.to) ? 0 : state.ordinary_count + 1;
      next.arrival_lane = directed.lane;
      const double turn =
          state.heading == directed.heading ? 0.0 : config_.turn_cost;
      (void)turn;
      current_path.push_back(directed_index);
      if (search(next, seed_index)) {
        return true;
      }
      current_path.pop_back();
      if (search_limit_reached) {
        return false;
      }
    }
    return false;
  };

  for (std::size_t index = 0; index < starts.size() && !complete_found;
       ++index) {
    const CoverageStart &seed = starts[index];
    SearchState state;
    state.node = seed.node;
    state.heading = seed.heading;
    state.ordinary_count =
        is_task_node(seed.node)
            ? 0
            : (request.ordinary_nodes_since_task_exact
                   ? request.ordinary_nodes_since_task +
                       (seed.cost > config_.node_capture_distance ? 1 : 0)
                   : std::max(1, request.ordinary_nodes_since_task));
    if (state.ordinary_count >= kCountStates) {
      continue;
    }
    for (const Traversal &prefix : seed.prefix) {
      if (prefix.directed_lane >= 0) {
        const DirectedLane &directed = graph_.directed_lane(prefix.directed_lane);
        state.mask |= lane_mask(directed.lane);
        state.arrival_lane = directed.lane;
      }
    }
    current_path.clear();
    failed.clear();
    if (search(state, static_cast<int>(index))) {
      break;
    }
  }

  if (search_limit_reached) {
    result.message = "coverage exact search reached its state limit";
    result.deferred_lane_ids = required;
    return result;
  }
  if (!complete_found && best_seed < 0) {
    result.message = "no forward-only heading-feasible coverage route exists";
    result.deferred_lane_ids = required;
    return result;
  }

  const int seed_index = complete_found ? selected_seed : best_seed;
  const SearchState selected = complete_found ? selected_state : best_state;
  const std::vector<int> &selected_steps =
      complete_found ? selected_path : best_path;
  result.traversals = starts[static_cast<std::size_t>(seed_index)].prefix;
  for (int directed_index : selected_steps) {
    const DirectedLane &directed = graph_.directed_lane(directed_index);
    result.traversals.push_back(
        {directed_index, config_.nodes[directed.from].position,
         config_.nodes[directed.to].position, false, false});
  }

  uint64_t reached_mask = selected.mask;
  for (std::size_t index = 0; index < required.size(); ++index) {
    if ((reached_mask & (uint64_t{1} << index)) == 0U) {
      result.deferred_lane_ids.push_back(required[index]);
    }
  }
  if (request.return_home && selected.node != config_.home_node) {
    result.deferred_lane_ids.push_back("RETURN_HOME");
  }
  for (const Traversal &traversal : result.traversals) {
    if (traversal.reverse) {
      result.message = "internal error: forward coverage produced reverse segment";
      result.deferred_lane_ids = required;
      return result;
    }
    if (traversal.directed_lane < 0) {
      continue;
    }
    const DirectedLane &directed = graph_.directed_lane(traversal.directed_lane);
    if (config_.lanes[directed.lane].inspection) {
      AddUnique(result.covered_lane_ids, config_.lanes[directed.lane].id);
    }
  }

  std::unordered_set<std::string> visited_tasks;
  const auto record_tasks = [this, &visited_tasks, &result](int node) {
    for (const Task &task : config_.tasks) {
      if (task.node == node && visited_tasks.insert(task.id).second) {
        result.visit_order.push_back(task.id);
      }
    }
  };
  record_tasks(starts[static_cast<std::size_t>(seed_index)].node);
  for (const Traversal &traversal : result.traversals) {
    if (traversal.directed_lane >= 0) {
      record_tasks(graph_.directed_lane(traversal.directed_lane).to);
    }
  }
  if (request.mode == "mission") {
    for (const Task &task : config_.tasks) {
      if (visited_tasks.count(task.id) == 0) {
        result.deferred_target_ids.push_back(task.id);
      }
    }
    if (!result.deferred_target_ids.empty() && !request.allow_partial) {
      result.message = "coverage route does not visit every mission target";
      result.deferred_lane_ids = required;
      return result;
    }
  }

  result.complete = complete_found && result.deferred_target_ids.empty();
  result.ordinary_nodes_since_task = selected.ordinary_count;
  if (result.traversals.empty() && result.complete) {
    result.success = true;
    result.collision_checked = true;
    result.message = "coverage already complete; no movement required";
    return result;
  }
  result.success =
      route_compiler_.compile(start, result.traversals, obstacles, result);
  if (result.success) {
    result.message = result.complete
                         ? "complete forward-only coverage route validated"
                         : "safe partial forward-only coverage route validated";
  }
  return result;
}

} // namespace new_map_planner
