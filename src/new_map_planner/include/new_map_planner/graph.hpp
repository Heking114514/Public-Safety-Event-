#ifndef NEW_MAP_PLANNER__GRAPH_HPP_
#define NEW_MAP_PLANNER__GRAPH_HPP_

#include "new_map_planner/types.hpp"

#include <string>
#include <unordered_map>

namespace new_map_planner {

class RoadGraph {
public:
  explicit RoadGraph(const PlannerConfig &config);

  const Graph &graph() const { return graph_; }
  const DirectedLane &directed_lane(int index) const;
  int node_index(const std::string &id) const;
  int lane_index(const std::string &id) const;
  int task_node(const std::string &id) const;
  const std::vector<int> &outgoing(int node) const;

private:
  const PlannerConfig &config_;
  Graph graph_;
  std::unordered_map<std::string, int> node_indices_;
  std::unordered_map<std::string, int> lane_indices_;
  std::unordered_map<std::string, int> task_nodes_;
};

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__GRAPH_HPP_
