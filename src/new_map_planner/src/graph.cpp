#include "new_map_planner/graph.hpp"

#include "new_map_planner/geometry.hpp"

#include <cmath>
#include <stdexcept>

namespace new_map_planner {

RoadGraph::RoadGraph(const PlannerConfig &config) : config_(config) {
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

} // namespace new_map_planner
