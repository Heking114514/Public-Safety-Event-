#ifndef NEW_MAP_PLANNER__GRAPH_HPP_
#define NEW_MAP_PLANNER__GRAPH_HPP_

#include "new_map_planner/types.hpp"

#include <array>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace new_map_planner {

// A straight, direction-specific route between two RFID nodes. The directed
// lane indices retain the individual physical edges so a search can expand a
// macro transition back into its executable route and update per-lane masks.
struct RfidCorridor {
  int start_node{-1};
  int end_node{-1};
  int heading{-1};
  std::vector<int> directed_lane_indices;
  double length{0.0};
};

class RoadGraph {
public:
  explicit RoadGraph(const PlannerConfig &config);

  const Graph &graph() const { return graph_; }
  const DirectedLane &directed_lane(int index) const;
  int node_index(const std::string &id) const;
  int lane_index(const std::string &id) const;
  int task_node(const std::string &id) const;
  const std::vector<int> &outgoing(int node) const;

  // Green input numbering is row-major from the upper-left corner (1) to the
  // lower-right corner (16), independently of node/lane configuration order.
  const std::string &green_node_id(int number) const;
  std::string green_pair_to_lane_id(int a, int b) const;
  std::vector<std::string> green_pairs_to_lane_ids(
      const std::vector<std::pair<int, int>> &pairs) const;
  void validate_blocked_lane_id(const std::string &lane_id) const;

  // Returns null when there is no qualifying corridor from this RFID node in
  // the requested travel direction.
  const RfidCorridor *rfid_corridor(int start_node, int heading) const;
  const std::vector<RfidCorridor> &rfid_corridors() const {
    return rfid_corridors_;
  }

private:
  void build_rfid_corridors();

  const PlannerConfig &config_;
  Graph graph_;
  std::unordered_map<std::string, int> node_indices_;
  std::unordered_map<std::string, int> lane_indices_;
  std::unordered_map<std::string, int> task_nodes_;
  std::array<std::string, 16> green_node_ids_;
  std::unordered_map<int, std::array<int, kHeadingCount>> rfid_corridor_indices_;
  std::vector<RfidCorridor> rfid_corridors_;
};

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__GRAPH_HPP_
