#include "new_map_planner/coverage.hpp"

#include "new_map_planner/geometry.hpp"

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace new_map_planner {
namespace {

constexpr std::size_t kMaximumCoverageStates = 2000000;
constexpr int kCountStates = 3;
constexpr std::size_t kMaximumMissionStates = 200000;
constexpr auto kMissionSearchTimeout = std::chrono::seconds(5);

struct MissionBaseState {
  int node{-1};
  int heading{-1};
  int ordinary_count{0};

  bool operator==(const MissionBaseState &other) const {
    return node == other.node && heading == other.heading &&
           ordinary_count == other.ordinary_count;
  }
};

struct MissionBaseStateHash {
  std::size_t operator()(const MissionBaseState &state) const {
    std::size_t value = static_cast<std::size_t>(state.node + 1);
    value = value * 31U + static_cast<std::size_t>(state.heading + 1);
    return value * 7U + static_cast<std::size_t>(state.ordinary_count);
  }
};

struct MissionState {
  int node{-1};
  int heading{-1};
  int ordinary_count{0};
  uint64_t rfid_mask{0};
  uint64_t tunnel_mask{0};
  uint64_t lane_mask{0};

  bool operator==(const MissionState &other) const {
    return node == other.node && heading == other.heading &&
           ordinary_count == other.ordinary_count &&
           rfid_mask == other.rfid_mask && tunnel_mask == other.tunnel_mask &&
           lane_mask == other.lane_mask;
  }
};

struct MissionStateHash {
  std::size_t operator()(const MissionState &state) const {
    std::size_t value = MissionBaseStateHash{}(
        {state.node, state.heading, state.ordinary_count});
    const auto mix = [&value](uint64_t part) {
      value ^= static_cast<std::size_t>(part) + 0x9e3779b97f4a7c15ULL +
               (value << 6U) + (value >> 2U);
    };
    mix(state.rfid_mask);
    mix(state.tunnel_mask);
    mix(state.lane_mask);
    return value;
  }
};

struct MissionBaseTransition {
  MissionBaseState target;
  std::vector<int> directed_lane_indices;
  double length{0.0};
};

struct MissionQueueEntry {
  double priority{0.0};
  int covered_lanes{0};
  uint32_t passes{0};
  double cost{0.0};
  uint64_t serial{0};
  MissionState state;

  bool operator>(const MissionQueueEntry &other) const {
    return std::tie(priority, covered_lanes, passes, cost, serial) >
           std::tie(other.priority, other.covered_lanes, other.passes,
                    other.cost, other.serial);
  }
};

struct MissionLabel {
  uint32_t passes{0};
  double cost{0.0};
};

struct MissionParent {
  bool has_parent{false};
  MissionState previous;
  std::vector<int> directed_lane_indices;
};

struct MissionGoal {
  bool tunnel{false};
  int index{-1};
  std::string id;
  int matrix_index{-1};
  uint64_t bit{0};
};

int Popcount(uint64_t value) {
  return static_cast<int>(std::bitset<64>(value).count());
}

PlanResult MissionFailure(PlanFailureReason reason, std::string message) {
  PlanResult result;
  result.failure_reason = reason;
  result.message = std::move(message);
  return result;
}

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

struct FallbackState {
  MissionBaseState base;
  bool progressed{false};

  bool operator==(const FallbackState &other) const {
    return base == other.base && progressed == other.progressed;
  }
};

struct FallbackStateHash {
  std::size_t operator()(const FallbackState &state) const {
    return MissionBaseStateHash{}(state.base) * 2U +
           static_cast<std::size_t>(state.progressed);
  }
};

struct FallbackParent {
  bool has_parent{false};
  FallbackState previous;
  std::vector<int> directed_lane_indices;
  int seed_index{-1};
};

} // namespace

CoveragePlanner::CoveragePlanner(const PlannerConfig &config,
                                 const RoadGraph &graph,
                                 const CollisionChecker &collision,
                                 const ManeuverLibrary &maneuvers,
                                 const RouteCompiler &route_compiler)
    : config_(config), graph_(graph), collision_(collision),
      maneuvers_(maneuvers), route_compiler_(route_compiler) {}

PlanResult CoveragePlanner::plan_mission(
    const PlanRequest &request, const Pose &start,
    const std::vector<Polygon> &obstacles,
    const std::vector<CoverageStart> &starts) const {
  const auto began = std::chrono::steady_clock::now();
  const auto deadline = began + kMissionSearchTimeout;
  if (starts.empty()) {
    return MissionFailure(PlanFailureReason::START_NOT_ON_ROAD,
                          "start pose is not aligned with a forward road");
  }
  if (config_.lanes.size() > 64U || config_.tasks.size() > 63U) {
    return MissionFailure(PlanFailureReason::INVALID_REQUEST,
                          "mission exceeds the 64-lane or 63-target mask limit");
  }

  const std::unordered_set<std::string> blocked_lanes(
      request.blocked_lane_ids.begin(), request.blocked_lane_ids.end());
  for (const std::string &id : blocked_lanes) {
    try {
      graph_.validate_blocked_lane_id(id);
    } catch (const std::invalid_argument &exception) {
      return MissionFailure(PlanFailureReason::INVALID_REQUEST,
                            exception.what());
    }
  }

  std::vector<int> task_bit_for_node(config_.nodes.size(), -1);
  std::vector<uint64_t> task_mask_for_node(config_.nodes.size(), 0U);
  std::unordered_map<int, std::string> task_id_for_node;
  std::unordered_map<std::string, int> task_index_by_id;
  for (std::size_t index = 0; index < config_.tasks.size(); ++index) {
    const Task &task = config_.tasks[index];
    task_bit_for_node[static_cast<std::size_t>(task.node)] =
        static_cast<int>(index);
    task_mask_for_node[static_cast<std::size_t>(task.node)] =
        uint64_t{1} << index;
    task_id_for_node.emplace(task.node, task.id);
    task_index_by_id.emplace(task.id, static_cast<int>(index));
  }
  const auto is_task_node = [&task_bit_for_node](int node) {
    return node >= 0 && node < static_cast<int>(task_bit_for_node.size()) &&
           task_bit_for_node[static_cast<std::size_t>(node)] >= 0;
  };

  // A corner obstruction retires only its associated RFID goal. Its node and
  // every remaining incident edge stay in the graph.
  const std::vector<std::pair<std::pair<int, int>, std::string>> corner_rules = {
      {{1, 2}, "2"}, {{1, 5}, "2"}, {{3, 4}, "11"}, {{4, 8}, "11"},
      {{12, 16}, "8"}, {{15, 16}, "8"}, {{13, 14}, "5"}, {{9, 13}, "5"}};
  std::unordered_set<int> corner_skipped_task_indices;
  for (const auto &rule : corner_rules) {
    try {
      const std::string lane = graph_.green_pair_to_lane_id(
          rule.first.first, rule.first.second);
      if (blocked_lanes.count(lane) != 0U) {
        const auto found = task_index_by_id.find(rule.second);
        if (found != task_index_by_id.end()) {
          corner_skipped_task_indices.insert(found->second);
        }
      }
    } catch (const std::invalid_argument &) {
      // Custom maps may omit one or more official green-node edges.
    }
  }

  std::vector<int> tunnel_lane_indices;
  std::vector<int> tunnel_bit_for_lane(config_.lanes.size(), -1);
  for (std::size_t index = 0; index < config_.lanes.size(); ++index) {
    if (!config_.lanes[index].tunnel) {
      continue;
    }
    if (tunnel_lane_indices.size() >= 63U) {
      return MissionFailure(PlanFailureReason::INVALID_REQUEST,
                            "mission exceeds the 63-tunnel mask limit");
    }
    tunnel_bit_for_lane[index] = static_cast<int>(tunnel_lane_indices.size());
    tunnel_lane_indices.push_back(static_cast<int>(index));
  }
  if (config_.tasks.size() + tunnel_lane_indices.size() > 64U) {
    return MissionFailure(PlanFailureReason::INVALID_REQUEST,
                          "mission goal masks exceed 64 total goals");
  }
  std::vector<bool> lane_free(config_.lanes.size(), true);
  for (std::size_t index = 0; index < config_.lanes.size(); ++index) {
    lane_free[index] = blocked_lanes.count(config_.lanes[index].id) == 0U;
  }

  const auto build_transitions =
      [this, &lane_free, &is_task_node](const MissionBaseState &state) {
    std::vector<MissionBaseTransition> transitions;
    for (const int directed_index : graph_.outgoing(state.node)) {
      const DirectedLane &directed = graph_.directed_lane(directed_index);
      if (!lane_free[static_cast<std::size_t>(directed.lane)]) {
        continue;
      }
      const int turn = TurnQuarterSteps(state.heading, directed.heading);
      if (turn == 2) {
        continue;
      }
      const int count = is_task_node(directed.to) ? 0 : state.ordinary_count + 1;
      if (count > 2) {
        continue;
      }
      transitions.push_back({{directed.to, directed.heading, count},
                             {directed_index}, directed.length});

      if (state.ordinary_count == 0 && is_task_node(state.node)) {
        const RfidCorridor *corridor = graph_.rfid_corridor(
            state.node, directed.heading);
        if (corridor != nullptr &&
            corridor->directed_lane_indices.front() == directed_index) {
          bool available = true;
          for (const int corridor_directed_index :
               corridor->directed_lane_indices) {
            const DirectedLane &corridor_lane =
                graph_.directed_lane(corridor_directed_index);
            if (!lane_free[static_cast<std::size_t>(corridor_lane.lane)]) {
              available = false;
              break;
            }
          }
          if (available) {
            transitions.push_back(
                {{corridor->end_node, corridor->heading, 0},
                 corridor->directed_lane_indices, corridor->length});
          }
        }
      }
    }
    return transitions;
  };

  // Build the finite forward-only state graph once. Besides identifying goals
  // that can be safely served, this gives the weighted A* search reusable
  // transitions and a return-safe subgraph.
  std::unordered_map<MissionBaseState, std::vector<MissionBaseTransition>,
                     MissionBaseStateHash>
      base_edges;
  std::unordered_map<MissionBaseState, std::vector<MissionBaseState>,
                     MissionBaseStateHash>
      reverse_edges;
  std::unordered_set<MissionBaseState, MissionBaseStateHash> reachable;
  std::queue<MissionBaseState> base_queue;
  std::vector<MissionBaseState> root_states;
  root_states.reserve(starts.size());
  std::vector<int> root_ordinary_counts(starts.size(), -1);
  for (std::size_t seed_index = 0; seed_index < starts.size(); ++seed_index) {
    const CoverageStart &seed = starts[seed_index];
    int count = 0;
    if (seed.node != config_.home_node && !is_task_node(seed.node)) {
      count = request.ordinary_nodes_since_task_exact
                  ? request.ordinary_nodes_since_task +
                        (seed.cost > config_.node_capture_distance ? 1 : 0)
                  : std::max(1, request.ordinary_nodes_since_task);
    }
    if (count > 2) {
      continue;
    }
    MissionBaseState root{seed.node, seed.heading, count};
    root_ordinary_counts[seed_index] = count;
    root_states.push_back(root);
    if (reachable.insert(root).second) {
      base_queue.push(root);
    }
  }
  if (root_states.empty()) {
    return MissionFailure(PlanFailureReason::START_HEADING_MISMATCH,
                          "start exceeds the forward ordinary-node limit");
  }
  while (!base_queue.empty()) {
    const MissionBaseState source = base_queue.front();
    base_queue.pop();
    std::vector<MissionBaseTransition> edges = build_transitions(source);
    base_edges.emplace(source, edges);
    for (const MissionBaseTransition &edge : edges) {
      reverse_edges[edge.target].push_back(source);
      if (reachable.insert(edge.target).second) {
        base_queue.push(edge.target);
      }
    }
  }
  std::unordered_set<MissionBaseState, MissionBaseStateHash> returning;
  std::queue<MissionBaseState> return_queue;
  for (const MissionBaseState &state : reachable) {
    if (state.node == config_.home_node && returning.insert(state).second) {
      return_queue.push(state);
    }
  }
  while (!return_queue.empty()) {
    const MissionBaseState target = return_queue.front();
    return_queue.pop();
    const auto found = reverse_edges.find(target);
    if (found == reverse_edges.end()) {
      continue;
    }
    for (const MissionBaseState &source : found->second) {
      if (returning.insert(source).second) {
        return_queue.push(source);
      }
    }
  }

  std::vector<int> required_task_indices;
  std::vector<int> required_tunnel_indices;
  std::vector<std::string> deferred_corner_targets;
  for (std::size_t index = 0; index < config_.tasks.size(); ++index) {
    const Task &task = config_.tasks[index];
    if (corner_skipped_task_indices.count(static_cast<int>(index)) != 0U) {
      deferred_corner_targets.push_back(task.id);
      continue;
    }
    const bool can_serve_and_return = std::any_of(
        reachable.begin(), reachable.end(), [&](const MissionBaseState &state) {
          return state.node == task.node && returning.count(state) != 0U;
        });
    if (can_serve_and_return) {
      required_task_indices.push_back(static_cast<int>(index));
    } else {
      deferred_corner_targets.push_back(task.id);
    }
  }
  std::unordered_set<int> required_tunnel_set;
  for (const auto &entry : base_edges) {
    if (reachable.count(entry.first) == 0U) {
      continue;
    }
    for (const MissionBaseTransition &transition : entry.second) {
      if (returning.count(transition.target) == 0U) {
        continue;
      }
      for (const int directed_index : transition.directed_lane_indices) {
        const DirectedLane &directed = graph_.directed_lane(directed_index);
        if (config_.lanes[static_cast<std::size_t>(directed.lane)].tunnel) {
          required_tunnel_set.insert(directed.lane);
        }
      }
    }
  }
  for (const int lane : tunnel_lane_indices) {
    if (required_tunnel_set.count(lane) != 0U) {
      required_tunnel_indices.push_back(lane);
    }
  }

  std::vector<uint64_t> rfid_bit_for_task(config_.tasks.size(), 0U);
  std::vector<uint64_t> tunnel_bit_for_lane_mask(config_.lanes.size(), 0U);
  std::vector<MissionGoal> goals;
  for (std::size_t index = 0; index < required_task_indices.size(); ++index) {
    const int task_index = required_task_indices[index];
    const Task &task = config_.tasks[static_cast<std::size_t>(task_index)];
    const uint64_t bit = uint64_t{1} << index;
    rfid_bit_for_task[static_cast<std::size_t>(task_index)] = bit;
    goals.push_back({false, task_index, task.id, task.node, bit});
  }
  const std::size_t tunnel_goal_offset = goals.size();
  for (std::size_t index = 0; index < required_tunnel_indices.size(); ++index) {
    const int lane_index = required_tunnel_indices[index];
    const uint64_t bit = uint64_t{1} << index;
    tunnel_bit_for_lane_mask[static_cast<std::size_t>(lane_index)] = bit;
    goals.push_back({true, lane_index,
                     config_.lanes[static_cast<std::size_t>(lane_index)].id,
                     -1, bit << tunnel_goal_offset});
  }

  const std::size_t matrix_size = config_.nodes.size() +
                                 required_tunnel_indices.size();
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> distance(
      matrix_size, std::vector<double>(matrix_size, infinity));
  for (std::size_t index = 0; index < matrix_size; ++index) {
    distance[index][index] = 0.0;
  }
  std::vector<int> tunnel_midpoint_index(config_.lanes.size(), -1);
  for (std::size_t index = 0; index < required_tunnel_indices.size(); ++index) {
    tunnel_midpoint_index[static_cast<std::size_t>(required_tunnel_indices[index])] =
        static_cast<int>(config_.nodes.size() + index);
  }
  for (std::size_t index = 0; index < config_.lanes.size(); ++index) {
    if (!lane_free[index]) {
      continue;
    }
    const Lane &lane = config_.lanes[index];
    const Point &from = config_.nodes[static_cast<std::size_t>(lane.from)].position;
    const Point &to = config_.nodes[static_cast<std::size_t>(lane.to)].position;
    const double length = Distance(from, to);
    const int midpoint = tunnel_midpoint_index[index];
    if (lane.tunnel && midpoint >= 0) {
      const int mid = midpoint;
      distance[static_cast<std::size_t>(lane.from)][static_cast<std::size_t>(mid)] = length * 0.5;
      distance[static_cast<std::size_t>(mid)][static_cast<std::size_t>(lane.to)] = length * 0.5;
      if (lane.bidirectional) {
        distance[static_cast<std::size_t>(lane.to)][static_cast<std::size_t>(mid)] = length * 0.5;
        distance[static_cast<std::size_t>(mid)][static_cast<std::size_t>(lane.from)] = length * 0.5;
      }
    } else {
      distance[static_cast<std::size_t>(lane.from)][static_cast<std::size_t>(lane.to)] = length;
      if (lane.bidirectional) {
        distance[static_cast<std::size_t>(lane.to)][static_cast<std::size_t>(lane.from)] = length;
      }
    }
  }
  for (std::size_t middle = 0; middle < matrix_size; ++middle) {
    for (std::size_t from = 0; from < matrix_size; ++from) {
      if (!std::isfinite(distance[from][middle])) {
        continue;
      }
      for (std::size_t to = 0; to < matrix_size; ++to) {
        distance[from][to] = std::min(
            distance[from][to], distance[from][middle] + distance[middle][to]);
      }
    }
  }
  for (MissionGoal &goal : goals) {
    if (goal.tunnel) {
      goal.matrix_index = tunnel_midpoint_index[static_cast<std::size_t>(goal.index)];
    }
  }
  std::unordered_map<uint64_t, double> mst_cache;
  const auto heuristic = [&](int node, uint64_t rfid_mask,
                             uint64_t tunnel_mask) {
    const uint64_t combined = rfid_mask |
        (tunnel_mask << tunnel_goal_offset);
    std::vector<int> pending;
    for (const MissionGoal &goal : goals) {
      if ((combined & goal.bit) == 0U) {
        pending.push_back(goal.matrix_index);
      }
    }
    auto cached = mst_cache.find(combined);
    double mst = 0.0;
    if (cached != mst_cache.end()) {
      mst = cached->second;
    } else {
      std::vector<double> connection(matrix_size, infinity);
      std::vector<bool> included(matrix_size, false);
      included[static_cast<std::size_t>(config_.home_node)] = true;
      for (const int pending_node : pending) {
        connection[static_cast<std::size_t>(pending_node)] =
            distance[static_cast<std::size_t>(config_.home_node)]
                    [static_cast<std::size_t>(pending_node)];
      }
      std::size_t remaining = pending.size();
      while (remaining > 0U) {
        int next = -1;
        double edge_cost = infinity;
        for (const int candidate : pending) {
          const std::size_t candidate_index = static_cast<std::size_t>(candidate);
          if (!included[candidate_index] && connection[candidate_index] < edge_cost) {
            edge_cost = connection[candidate_index];
            next = candidate;
          }
        }
        if (next < 0 || !std::isfinite(edge_cost)) {
          mst = 0.0;
          break;
        }
        included[static_cast<std::size_t>(next)] = true;
        mst += edge_cost;
        --remaining;
        for (const int candidate : pending) {
          const std::size_t candidate_index = static_cast<std::size_t>(candidate);
          connection[candidate_index] = std::min(
              connection[candidate_index],
              distance[static_cast<std::size_t>(next)][candidate_index]);
        }
      }
      mst_cache.emplace(combined, mst);
    }
    if (pending.empty()) {
      return distance[static_cast<std::size_t>(node)]
                     [static_cast<std::size_t>(config_.home_node)];
    }
    double nearest = infinity;
    for (const MissionGoal &goal : goals) {
      if ((combined & goal.bit) == 0U) {
        nearest = std::min(
            nearest, distance[static_cast<std::size_t>(node)]
                     [static_cast<std::size_t>(goal.matrix_index)]);
      }
    }
    return nearest + mst;
  };

  const auto initial_state = [&](std::size_t seed_index) {
    const CoverageStart &seed = starts[seed_index];
    MissionState state;
    state.node = seed.node;
    state.heading = seed.heading;
    state.ordinary_count = root_ordinary_counts[seed_index];
    if (seed.node >= 0 && seed.node < static_cast<int>(task_mask_for_node.size())) {
      const int task_index = task_bit_for_node[static_cast<std::size_t>(seed.node)];
      if (task_index >= 0) {
        state.rfid_mask |= rfid_bit_for_task[static_cast<std::size_t>(task_index)];
      }
    }
    for (const Traversal &prefix : seed.prefix) {
      if (prefix.directed_lane < 0) {
        continue;
      }
      const DirectedLane &directed = graph_.directed_lane(prefix.directed_lane);
      state.lane_mask |= uint64_t{1} << directed.lane;
      if (directed.to >= 0 && directed.to < static_cast<int>(task_bit_for_node.size())) {
        const int task_index = task_bit_for_node[static_cast<std::size_t>(directed.to)];
        if (task_index >= 0) {
          state.rfid_mask |= rfid_bit_for_task[static_cast<std::size_t>(task_index)];
        }
      }
      state.tunnel_mask |= tunnel_bit_for_lane_mask[static_cast<std::size_t>(directed.lane)];
    }
    return state;
  };

  // Save one legal partial return before spending the optimization budget.
  // If the 5 s/200k-state bound is reached before A* improves it, the caller
  // still receives an executable route whenever the topology admits one.
  std::vector<int> fallback_route;
  int fallback_seed = -1;
  std::queue<FallbackState> fallback_queue;
  std::unordered_map<FallbackState, FallbackParent, FallbackStateHash> fallback_parent;
  for (std::size_t index = 0; index < starts.size(); ++index) {
    const MissionState root = initial_state(index);
    const bool progressed = root.rfid_mask != 0U || root.tunnel_mask != 0U;
    FallbackState key{{root.node, root.heading, root.ordinary_count}, progressed};
    auto inserted = fallback_parent.emplace(key, FallbackParent{});
    if (inserted.second) {
      inserted.first->second.seed_index = static_cast<int>(index);
      fallback_queue.push(key);
    }
  }
  FallbackState fallback_goal;
  bool fallback_found = false;
  while (!fallback_queue.empty() && !fallback_found) {
    const FallbackState current = fallback_queue.front();
    fallback_queue.pop();
    if (current.base.node == config_.home_node && current.progressed &&
        fallback_parent.at(current).has_parent) {
      fallback_goal = current;
      fallback_found = true;
      break;
    }
    const auto found = base_edges.find(current.base);
    if (found == base_edges.end()) {
      continue;
    }
    for (const MissionBaseTransition &edge : found->second) {
      bool progressed = current.progressed;
      for (const int directed_index : edge.directed_lane_indices) {
        const DirectedLane &directed = graph_.directed_lane(directed_index);
        const int task_index = task_bit_for_node[static_cast<std::size_t>(directed.to)];
        if (task_index >= 0 && rfid_bit_for_task[static_cast<std::size_t>(task_index)] != 0U) {
          progressed = true;
        }
        if (tunnel_bit_for_lane_mask[static_cast<std::size_t>(directed.lane)] != 0U) {
          progressed = true;
        }
      }
      FallbackState next{edge.target, progressed};
      auto inserted = fallback_parent.emplace(next, FallbackParent{});
      if (!inserted.second) {
        continue;
      }
      inserted.first->second.has_parent = true;
      inserted.first->second.previous = current;
      inserted.first->second.directed_lane_indices = edge.directed_lane_indices;
      inserted.first->second.seed_index = fallback_parent.at(current).seed_index;
      fallback_queue.push(next);
    }
  }
  if (fallback_found) {
    fallback_seed = fallback_parent.at(fallback_goal).seed_index;
    std::vector<std::vector<int>> reverse_chunks;
    FallbackState cursor = fallback_goal;
    while (fallback_parent.at(cursor).has_parent) {
      const FallbackParent &parent = fallback_parent.at(cursor);
      reverse_chunks.push_back(parent.directed_lane_indices);
      cursor = parent.previous;
    }
    for (auto chunk = reverse_chunks.rbegin(); chunk != reverse_chunks.rend(); ++chunk) {
      fallback_route.insert(fallback_route.end(), chunk->begin(), chunk->end());
    }
  }

  struct Candidate {
    bool valid{false};
    int seed_index{-1};
    std::vector<int> directed_lane_indices;
    MissionState state;
    MissionLabel label;
    std::tuple<int, int, int, int, double> score;
  } best_candidate;
  const auto evaluate_candidate = [&](int seed_index,
                                      const std::vector<int> &path,
                                      uint32_t passes, double cost) {
    Candidate candidate;
    candidate.valid = true;
    candidate.seed_index = seed_index;
    candidate.directed_lane_indices = path;
    candidate.state = initial_state(static_cast<std::size_t>(seed_index));
    candidate.label = {passes, cost};
    for (const int directed_index : path) {
      const DirectedLane &directed = graph_.directed_lane(directed_index);
      const int turn = TurnQuarterSteps(candidate.state.heading, directed.heading);
      candidate.label.cost += directed.length + (turn == 0 ? 0.0 : config_.turn_cost);
      ++candidate.label.passes;
      candidate.state.node = directed.to;
      candidate.state.heading = directed.heading;
      candidate.state.ordinary_count = is_task_node(directed.to)
                                           ? 0
                                           : candidate.state.ordinary_count + 1;
      candidate.state.lane_mask |= uint64_t{1} << directed.lane;
      if (directed.to >= 0 && directed.to < static_cast<int>(task_bit_for_node.size())) {
        const int task_index = task_bit_for_node[static_cast<std::size_t>(directed.to)];
        if (task_index >= 0) {
          candidate.state.rfid_mask |= rfid_bit_for_task[static_cast<std::size_t>(task_index)];
        }
      }
      candidate.state.tunnel_mask |=
          tunnel_bit_for_lane_mask[static_cast<std::size_t>(directed.lane)];
    }
    candidate.score = {
        -Popcount(candidate.state.rfid_mask),
        -Popcount(candidate.state.tunnel_mask),
        -Popcount(candidate.state.lane_mask),
        static_cast<int>(candidate.label.passes) - Popcount(candidate.state.lane_mask),
        candidate.label.cost};
    return candidate;
  };
  if (fallback_found) {
    const CoverageStart &seed = starts[static_cast<std::size_t>(fallback_seed)];
    uint32_t prefix_passes = 0;
    for (const Traversal &prefix : seed.prefix) {
      if (prefix.directed_lane >= 0) {
        ++prefix_passes;
      }
    }
    best_candidate = evaluate_candidate(fallback_seed, fallback_route,
                                        prefix_passes, seed.cost);
  }

  std::unordered_map<MissionState, MissionLabel, MissionStateHash> best;
  std::unordered_map<MissionState, MissionParent, MissionStateHash> parents;
  std::unordered_map<MissionState, int, MissionStateHash> root_seed_for_state;
  std::priority_queue<MissionQueueEntry, std::vector<MissionQueueEntry>,
                      std::greater<MissionQueueEntry>> queue;
  uint64_t serial = 0;
  for (std::size_t index = 0; index < starts.size(); ++index) {
    MissionState state = initial_state(index);
    uint32_t passes = 0;
    for (const Traversal &prefix : starts[index].prefix) {
      if (prefix.directed_lane >= 0) {
        ++passes;
      }
    }
    const MissionLabel label{passes, starts[index].cost};
    const auto existing = best.find(state);
    if (existing != best.end() &&
        std::tie(existing->second.passes, existing->second.cost) <=
            std::tie(label.passes, label.cost)) {
      continue;
    }
    best[state] = label;
    parents[state] = MissionParent{};
    root_seed_for_state[state] = static_cast<int>(index);
    const double estimate = heuristic(state.node, state.rfid_mask,
                                       state.tunnel_mask);
    queue.push({label.cost + 2.0 * estimate - 0.05 * Popcount(state.lane_mask),
                -Popcount(state.lane_mask), label.passes, label.cost,
                serial++, state});
  }

  bool timed_out = false;
  bool state_limit = false;
  while (!queue.empty()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      timed_out = true;
      break;
    }
    const MissionQueueEntry entry = queue.top();
    queue.pop();
    const auto known = best.find(entry.state);
    if (known == best.end() || known->second.passes != entry.passes ||
        std::abs(known->second.cost - entry.cost) > 1.0e-9) {
      continue;
    }
    const MissionState &state = entry.state;
    if (state.node == config_.home_node && entry.passes > 0U &&
        (state.rfid_mask != 0U || state.tunnel_mask != 0U)) {
      const auto score = std::make_tuple(
          -Popcount(state.rfid_mask), -Popcount(state.tunnel_mask),
          -Popcount(state.lane_mask),
          static_cast<int>(entry.passes) - Popcount(state.lane_mask),
          entry.cost);
      if (!best_candidate.valid || score < best_candidate.score) {
        std::vector<std::vector<int>> reverse_chunks;
        MissionState cursor = state;
        while (parents.at(cursor).has_parent) {
          const MissionParent &parent = parents.at(cursor);
          reverse_chunks.push_back(parent.directed_lane_indices);
          cursor = parent.previous;
        }
        std::vector<int> path;
        for (auto chunk = reverse_chunks.rbegin(); chunk != reverse_chunks.rend(); ++chunk) {
          path.insert(path.end(), chunk->begin(), chunk->end());
        }
        const int seed_index = root_seed_for_state.at(cursor);
        uint32_t prefix_passes = 0;
        for (const Traversal &prefix : starts[static_cast<std::size_t>(seed_index)].prefix) {
          if (prefix.directed_lane >= 0) {
            ++prefix_passes;
          }
        }
        best_candidate = evaluate_candidate(seed_index, path, prefix_passes,
            starts[static_cast<std::size_t>(seed_index)].cost);
      }
    }

    const MissionBaseState base{state.node, state.heading, state.ordinary_count};
    const auto outgoing = base_edges.find(base);
    if (outgoing == base_edges.end()) {
      continue;
    }
    for (const MissionBaseTransition &transition : outgoing->second) {
      MissionState next;
      next.node = transition.target.node;
      next.heading = transition.target.heading;
      next.ordinary_count = transition.target.ordinary_count;
      next.rfid_mask = state.rfid_mask;
      next.tunnel_mask = state.tunnel_mask;
      next.lane_mask = state.lane_mask;
      double transition_cost = 0.0;
      int previous_heading = state.heading;
      for (const int directed_index : transition.directed_lane_indices) {
        const DirectedLane &directed = graph_.directed_lane(directed_index);
        transition_cost += directed.length;
        if (TurnQuarterSteps(previous_heading, directed.heading) != 0) {
          transition_cost += config_.turn_cost;
        }
        previous_heading = directed.heading;
        next.lane_mask |= uint64_t{1} << directed.lane;
        if (directed.to >= 0 && directed.to < static_cast<int>(task_bit_for_node.size())) {
          const int task_index = task_bit_for_node[static_cast<std::size_t>(directed.to)];
          if (task_index >= 0) {
            next.rfid_mask |= rfid_bit_for_task[static_cast<std::size_t>(task_index)];
          }
        }
        next.tunnel_mask |= tunnel_bit_for_lane_mask[static_cast<std::size_t>(directed.lane)];
      }
      MissionLabel label{entry.passes +
                             static_cast<uint32_t>(transition.directed_lane_indices.size()),
                         entry.cost + transition_cost};
      const auto old = best.find(next);
      if (old != best.end() &&
          std::tie(old->second.passes, old->second.cost) <=
              std::tie(label.passes, label.cost)) {
        continue;
      }
      if (old == best.end() && best.size() >= kMaximumMissionStates) {
        state_limit = true;
        queue = decltype(queue){};
        break;
      }
      best[next] = label;
      parents[next] = {true, state, transition.directed_lane_indices};
      root_seed_for_state[next] = root_seed_for_state.at(state);
      const double estimate = heuristic(next.node, next.rfid_mask,
                                        next.tunnel_mask);
      queue.push({label.cost + 2.0 * estimate - 0.05 * Popcount(next.lane_mask),
                  -Popcount(next.lane_mask), label.passes, label.cost,
                  serial++, next});
    }
    if (state_limit || std::chrono::steady_clock::now() >= deadline) {
      if (!state_limit) {
        timed_out = true;
      }
      break;
    }
  }

  if (!best_candidate.valid) {
    return MissionFailure(PlanFailureReason::NO_FEASIBLE_ROUTE,
                          "no forward-only route can visit a mission goal and return home");
  }

  const int selected_seed_index = best_candidate.seed_index;
  PlanResult result;
  result.traversals = starts[static_cast<std::size_t>(selected_seed_index)].prefix;
  for (const int directed_index : best_candidate.directed_lane_indices) {
    const DirectedLane &directed = graph_.directed_lane(directed_index);
    result.traversals.push_back({directed_index,
                                 config_.nodes[static_cast<std::size_t>(directed.from)].position,
                                 config_.nodes[static_cast<std::size_t>(directed.to)].position,
                                 false, false});
  }
  const MissionState &selected = best_candidate.state;
  result.deferred_target_ids = deferred_corner_targets;
  for (const Task &task : config_.tasks) {
    const auto task_index = task_index_by_id.find(task.id);
    if (task_index == task_index_by_id.end()) {
      continue;
    }
    const int index = task_index->second;
    const uint64_t bit = rfid_bit_for_task[static_cast<std::size_t>(index)];
    if (bit != 0U && (selected.rfid_mask & bit) == 0U) {
      AddUnique(result.deferred_target_ids, task.id);
    }
  }
  for (const int lane_index : tunnel_lane_indices) {
    const uint64_t bit = tunnel_bit_for_lane_mask[static_cast<std::size_t>(lane_index)];
    if (bit == 0U || (selected.tunnel_mask & bit) == 0U) {
      AddUnique(result.deferred_lane_ids,
                config_.lanes[static_cast<std::size_t>(lane_index)].id);
    }
  }
  for (const Traversal &traversal : result.traversals) {
    if (traversal.reverse) {
      return MissionFailure(PlanFailureReason::ROUTE_INVALID,
                            "unified mission search produced a reverse segment");
    }
    if (traversal.directed_lane < 0) {
      continue;
    }
    const DirectedLane &directed = graph_.directed_lane(traversal.directed_lane);
    if (config_.lanes[static_cast<std::size_t>(directed.lane)].inspection) {
      AddUnique(result.covered_lane_ids,
                config_.lanes[static_cast<std::size_t>(directed.lane)].id);
    }
    const auto task = task_id_for_node.find(directed.to);
    if (task != task_id_for_node.end()) {
      AddUnique(result.visit_order, task->second);
    }
  }
  const bool returned_home = selected.node == config_.home_node;
  result.complete = returned_home && result.deferred_target_ids.empty() &&
                    result.deferred_lane_ids.empty();
  result.status = result.complete ? "SUCCESS" : "PARTIAL_SUCCESS";
  result.ordinary_nodes_since_task = selected.ordinary_count;
  result.validate_geometry = false;
  result.success = route_compiler_.compile(start, result.traversals,
                                            obstacles, result);
  if (!result.success) {
    result.failure_reason = PlanFailureReason::ROUTE_INVALID;
    result.status.clear();
    return result;
  }
  result.failure_reason = PlanFailureReason::NONE;
  if (result.complete) {
    result.message = "all RFID and tunnel goals completed; forward route returned home";
  } else if (timed_out || state_limit) {
    result.message = "best forward return route returned within the mission search budget";
  } else {
    result.message = "partial forward route returned home; unreachable goals deferred";
  }
  return result;
}

PlanResult CoveragePlanner::plan(
    const PlanRequest &request, const Pose &start,
    const std::vector<Polygon> &obstacles,
    const std::vector<CoverageStart> &starts,
    bool allow_tunnel_connectors) const {
  if (request.mode == "mission") {
    return plan_mission(request, start, obstacles, starts);
  }
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
    result.failure_reason = PlanFailureReason::INVALID_REQUEST;
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
    result.failure_reason = PlanFailureReason::INVALID_REQUEST;
    result.message = exception.what();
    return result;
  }
  if (starts.empty()) {
    result.failure_reason = PlanFailureReason::START_NOT_ON_ROAD;
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
    result.failure_reason = PlanFailureReason::COVERAGE_INCOMPLETE;
    result.message = "coverage exact search reached its state limit";
    result.deferred_lane_ids = required;
    return result;
  }
  if (!complete_found && best_seed < 0) {
    result.failure_reason = PlanFailureReason::NO_FEASIBLE_ROUTE;
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
      result.failure_reason = PlanFailureReason::ROUTE_INVALID;
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
      result.failure_reason = PlanFailureReason::COVERAGE_INCOMPLETE;
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
  } else {
    result.failure_reason = PlanFailureReason::ROUTE_INVALID;
  }
  return result;
}

} // namespace new_map_planner
