#include "new_map_planner/graph.hpp"

#include "new_map_planner/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace new_map_planner {

RoadGraph::RoadGraph(const PlannerConfig &config) : config_(config) {
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      const int number = 4 * row + col + 1;
      green_node_ids_[static_cast<std::size_t>(number - 1)] =
          "x" + std::to_string(col) + "_y" + std::to_string(3 - row);
    }
  }

  graph_.outgoing.resize(config.nodes.size());
  for (std::size_t index = 0; index < config.nodes.size(); ++index) {
    node_indices_.emplace(config.nodes[index].id, static_cast<int>(index));
  }
  for (std::size_t index = 0; index < config.lanes.size(); ++index) {
    const Lane &lane = config.lanes[index];
    lane_indices_.emplace(lane.id, static_cast<int>(index));
    const Point &from = config.nodes[lane.from].position;
    const Point &to = config.nodes[lane.to].position;
    const double forward_yaw = std::atan2(to.y - from.y, to.x - from.x);
    DirectedLane forward{static_cast<int>(index), lane.from, lane.to,
                         CardinalHeading(forward_yaw), Distance(from, to)};
    const int forward_index = static_cast<int>(graph_.directed_lanes.size());
    graph_.directed_lanes.push_back(forward);
    graph_.outgoing[lane.from].push_back(forward_index);
    if (lane.bidirectional) {
      DirectedLane reverse{static_cast<int>(index), lane.to, lane.from,
                           CardinalHeading(forward_yaw + kPi), forward.length};
      const int reverse_index = static_cast<int>(graph_.directed_lanes.size());
      graph_.directed_lanes.push_back(reverse);
      graph_.outgoing[lane.to].push_back(reverse_index);
    }
  }
  for (const Task &task : config.tasks) {
    task_nodes_.emplace(task.id, task.node);
  }
  build_rfid_corridors();
}

const DirectedLane &RoadGraph::directed_lane(int index) const {
  if (index < 0 || index >= static_cast<int>(graph_.directed_lanes.size())) {
    throw std::out_of_range("directed lane index is invalid");
  }
  return graph_.directed_lanes[static_cast<std::size_t>(index)];
}

int RoadGraph::node_index(const std::string &id) const {
  const auto found = node_indices_.find(id);
  if (found == node_indices_.end()) {
    throw std::invalid_argument("unknown node: " + id);
  }
  return found->second;
}

int RoadGraph::lane_index(const std::string &id) const {
  const auto found = lane_indices_.find(id);
  if (found == lane_indices_.end()) {
    throw std::invalid_argument("unknown lane: " + id);
  }
  return found->second;
}

int RoadGraph::task_node(const std::string &id) const {
  const auto found = task_nodes_.find(id);
  if (found == task_nodes_.end()) {
    throw std::invalid_argument("unknown target: " + id);
  }
  return found->second;
}

const std::vector<int> &RoadGraph::outgoing(int node) const {
  if (node < 0 || node >= static_cast<int>(graph_.outgoing.size())) {
    throw std::out_of_range("node index is invalid");
  }
  return graph_.outgoing[static_cast<std::size_t>(node)];
}

const std::string &RoadGraph::green_node_id(int number) const {
  if (number < 1 || number > static_cast<int>(green_node_ids_.size())) {
    throw std::invalid_argument("green node number must be in 1..16");
  }
  return green_node_ids_[static_cast<std::size_t>(number - 1)];
}

void RoadGraph::validate_blocked_lane_id(const std::string &id) const {
  const int index = lane_index(id);
  const Lane &lane = config_.lanes[static_cast<std::size_t>(index)];
  if (id == "LAUNCH" || id == "ROAD_TOP_LEFT" ||
      id == "ROAD_TOP_RIGHT") {
    throw std::invalid_argument("protected lane cannot be blocked: " + id);
  }
  if (lane.tunnel) {
    throw std::invalid_argument("tunnel lane cannot be blocked: " + id);
  }
}

std::string RoadGraph::green_pair_to_lane_id(int a, int b) const {
  if (a < 1 || a > 16 || b < 1 || b > 16 || a == b) {
    throw std::invalid_argument("green node pair must contain distinct numbers in 1..16");
  }
  const int from = node_index(green_node_id(a));
  const int to = node_index(green_node_id(b));

  const Lane *matched = nullptr;
  for (const Lane &lane : config_.lanes) {
    if ((lane.from == from && lane.to == to) ||
        (lane.from == to && lane.to == from)) {
      if (matched != nullptr) {
        throw std::invalid_argument("green node pair maps to multiple physical lanes");
      }
      matched = &lane;
    }
  }
  if (matched == nullptr) {
    throw std::invalid_argument("green nodes " + std::to_string(a) + " and " +
                                std::to_string(b) + " are not directly connected");
  }
  return matched->id;
}

std::vector<std::string> RoadGraph::green_pairs_to_lane_ids(
    const std::vector<std::pair<int, int>> &pairs) const {
  std::vector<std::string> lane_ids;
  lane_ids.reserve(pairs.size());
  std::unordered_set<std::string> seen;
  for (const auto &pair : pairs) {
    // Both (a,b) and (b,a) resolve to the same physical lane ID; this also
    // deduplicates reversed repeated inputs while preserving first-seen order.
    const std::string id = green_pair_to_lane_id(pair.first, pair.second);
    if (seen.insert(id).second) {
      lane_ids.push_back(id);
    }
  }
  return lane_ids;
}

void RoadGraph::build_rfid_corridors() {
  std::unordered_set<int> rfid_nodes;
  for (const auto &entry : task_nodes_) {
    rfid_nodes.insert(entry.second);
  }

  for (const int start_node : rfid_nodes) {
    for (int heading = 0; heading < kHeadingCount; ++heading) {
      int current_node = start_node;
      int ordinary_nodes = 0;
      std::vector<int> path;

      // A straight direction cannot revisit a node in a valid axis-aligned
      // map. The node-count bound also makes this robust for custom graphs.
      for (std::size_t step = 0; step < config_.nodes.size(); ++step) {
        int next_directed_index = -1;
        for (const int directed_index : graph_.outgoing[current_node]) {
          if (directed_lane(directed_index).heading != heading) {
            continue;
          }
          if (next_directed_index >= 0) {
            // A branching or parallel straight path is not a deterministic
            // macro corridor; the ordinary search can still handle it.
            next_directed_index = -2;
            break;
          }
          next_directed_index = directed_index;
        }
        if (next_directed_index < 0) {
          break;
        }

        const DirectedLane &directed = directed_lane(next_directed_index);
        path.push_back(next_directed_index);
        current_node = directed.to;
        if (rfid_nodes.count(current_node) != 0U) {
          if (ordinary_nodes > 2) {
            RfidCorridor corridor;
            corridor.start_node = start_node;
            corridor.end_node = current_node;
            corridor.heading = heading;
            corridor.directed_lane_indices = std::move(path);
            for (const int lane_index : corridor.directed_lane_indices) {
              corridor.length +=
                  directed_lane(lane_index).length;
            }
            const int corridor_index =
                static_cast<int>(rfid_corridors_.size());
            rfid_corridors_.push_back(std::move(corridor));
            auto inserted = rfid_corridor_indices_.emplace(
                start_node, std::array<int, kHeadingCount>{});
            if (inserted.second) {
              inserted.first->second.fill(-1);
            }
            inserted.first->second[static_cast<std::size_t>(heading)] =
                corridor_index;
          }
          break;
        }
        ++ordinary_nodes;
      }
    }
  }
}

const RfidCorridor *RoadGraph::rfid_corridor(int start_node,
                                              int heading) const {
  if (heading < 0 || heading >= kHeadingCount) {
    return nullptr;
  }
  const auto found = rfid_corridor_indices_.find(start_node);
  if (found == rfid_corridor_indices_.end()) {
    return nullptr;
  }
  const int index = found->second[static_cast<std::size_t>(heading)];
  if (index < 0) {
    return nullptr;
  }
  return &rfid_corridors_[static_cast<std::size_t>(index)];
}

} // namespace new_map_planner
