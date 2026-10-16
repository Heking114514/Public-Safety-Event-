#ifndef NEW_MAP_PLANNER__TYPES_HPP_
#define NEW_MAP_PLANNER__TYPES_HPP_

#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace new_map_planner {

constexpr double kPi = 3.14159265358979323846;
constexpr int kHeadingCount = 4;

struct Point {
  double x{0.0};
  double y{0.0};
};

struct Pose {
  Point position;
  double yaw{0.0};
};

struct Polygon {
  std::vector<Point> points;
};

struct Rectangle {
  double minimum_x{0.0};
  double minimum_y{0.0};
  double maximum_x{0.0};
  double maximum_y{0.0};
};

struct Node {
  std::string id;
  Point position;
};

struct Lane {
  std::string id;
  int from{-1};
  int to{-1};
  double width{0.0};
  bool bidirectional{true};
  bool inspection{true};
  bool tunnel{false};
};

struct Task {
  std::string id;
  int node{-1};
};

struct DirectedLane {
  int lane{-1};
  int from{-1};
  int to{-1};
  int heading{-1};
  double length{0.0};
};

enum class ManeuverType : uint8_t {
  DRIVE = 0,
  TURN_LEFT = 1,
  TURN_RIGHT = 2,
  START_ALIGN = 3,
  FINISH = 4,
  REVERSE = 5,
};

struct Maneuver {
  ManeuverType type{ManeuverType::DRIVE};
  std::string node_id;
  std::string incoming_lane_id;
  std::string outgoing_lane_id;
  Pose start;
  Pose end;
  double distance{0.0};
  bool stop_required{false};
  bool collision_checked{false};
};

struct Traversal {
  int directed_lane{-1};
  Point start;
  Point end;
  bool partial{false};
  bool reverse{false};
};

struct MapTransform {
  Point arena_origin;
  double arena_yaw{0.0};
};

struct PlannerTopics {
  std::string service{"/new_map_planner/plan"};
  std::string plan{"/new_map_planner/route_plan"};
  std::string arena_path{"/new_map_planner/arena_path"};
  std::string navigation_path{"/new_map_planner/navigation_path"};
  std::string occupancy_grid{"/new_map_planner/map"};
};

enum class StartMode : uint8_t {
  NORMAL = 0,
  TURN_RECOVERY = 1,
  OBSTACLE_RECOVERY = 2,
};

struct PlannerConfig {
  std::string map_id{"official_arena_v1"};
  double width{0.0};
  double height{0.0};
  double resolution{0.02};
  double vehicle_length{0.0};
  double vehicle_width{0.0};
  double base_link_offset_x{0.0};
  double footprint_center_offset_x{0.0};
  double safety_margin{0.0};
  double turn_safety_margin{0.0};
  double lane_snap_distance{0.08};
  double node_capture_distance{0.005};
  double start_heading_tolerance{0.45};
  double turn_cost{0.35};
  double reverse_cost{0.20};
  double u_turn_threshold{2.6};
  double collision_sample_step{0.01};
  bool publish_shunt_markers{false};
  bool require_return_home{true};
  int home_node{-1};
  MapTransform transform;
  std::vector<Rectangle> free_regions;
  std::vector<Rectangle> static_obstacles;
  std::vector<Node> nodes;
  std::vector<Lane> lanes;
  std::vector<Task> tasks;
  std::map<int, std::string> rfid_supernodes;
  std::vector<std::string> default_targets;
  std::vector<std::string> default_inspection_lanes;
  PlannerTopics topics;
};

struct PlanRequest {
  Pose start_map;
  StartMode start_mode{StartMode::NORMAL};
  std::string recovery_anchor_node_id;
  std::string recovery_incoming_lane_id;
  bool allow_reverse_departure{false};
  std::string mode{"targets"};
  std::vector<std::string> target_ids;
  std::vector<std::string> required_lane_ids;
  std::vector<std::string> completed_lane_ids;
  std::vector<Polygon> dynamic_obstacles_map;
  std::vector<std::string> blocked_lane_ids;
  uint64_t obstacle_revision{0};
  // Geometry is an explicit validation mode. Normal online business requests
  // leave both switches false so graph planning is not gated by the fixed
  // controller turn sweep. Either switch enables the legacy checks.
  bool validate_geometry{false};
  bool online_geometry_validation{false};
  bool return_home{true};
  bool allow_partial{false};
  int ordinary_nodes_since_task{1};
  bool ordinary_nodes_since_task_exact{false};
  bool sequential_targets{false};
};

inline bool GeometryValidationEnabled(const PlanRequest &request) {
  return request.validate_geometry || request.online_geometry_validation;
}

enum class PlanFailureReason : uint8_t {
  NONE = 0,
  INVALID_REQUEST,
  START_POSE_COLLISION,
  START_NOT_ON_ROAD,
  START_HEADING_MISMATCH,
  START_FORWARD_BLOCKED,
  START_REQUIRES_REVERSE,
  NO_FEASIBLE_ROUTE,
  COVERAGE_INCOMPLETE,
  ROUTE_INVALID,
  RECOVERY_CONTEXT_INVALID,
  RECOVERY_REVERSE_BLOCKED,
  RECOVERY_CONTROLLER_OWNED,
};

const char *PlanFailureName(PlanFailureReason reason);

struct PlanResult {
  bool success{false};
  bool complete{false};
  // Optional public status override used by planners that can safely return a
  // useful route while deferring unreachable mission goals.
  std::string status;
  bool collision_checked{false};
  bool validate_geometry{false};
  PlanFailureReason failure_reason{PlanFailureReason::NONE};
  std::string message;
  std::vector<Traversal> traversals;
  bool recovery_only{false};
  std::vector<Maneuver> maneuvers;
  std::vector<Pose> arena_path;
  std::vector<Pose> navigation_path;
  std::vector<bool> turn_waypoints;
  std::vector<bool> reverse_arrival_turn_waypoints;
  std::vector<std::string> waypoint_supernode_ids;
  std::vector<std::string> visit_order;
  std::vector<std::string> covered_lane_ids;
  std::vector<std::string> deferred_target_ids;
  std::vector<std::string> deferred_lane_ids;
  double length{0.0};
  int ordinary_nodes_since_task{0};
};

struct Graph {
  std::vector<DirectedLane> directed_lanes;
  std::vector<std::vector<int>> outgoing;
};

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__TYPES_HPP_
