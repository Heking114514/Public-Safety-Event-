#ifndef ARENA_PATH_PLANNER__PLANNER_HPP_
#define ARENA_PATH_PLANNER__PLANNER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace arena_path_planner
{

struct Point
{
  double x{0.0};
  double y{0.0};
};

struct Pose
{
  Point position;
  double yaw{0.0};
};

struct Rectangle
{
  double minimum_x{0.0};
  double minimum_y{0.0};
  double maximum_x{0.0};
  double maximum_y{0.0};
};

struct InspectionEdge
{
  int from{0};
  int to{0};
  std::string label;
  bool tunnel{false};
};

struct PlannerTopics
{
  std::string service{"/arena_path_planner/plan"};
  std::string arena_path{"/arena_path_planner/arena_path"};
  std::string navigation_path{"/arena_path_planner/navigation_path"};
  std::string occupancy_grid{"/arena_path_planner/map"};
  std::string route_input{"/waypoint_navigation/route_input"};
};

struct PlannerConfig
{
  double width{0.0};
  double height{0.0};
  double resolution{0.025};
  double inflation_radius{0.05};
  double preferred_clearance{0.15};
  double clearance_cost_weight{6.0};
  // A non-zero footprint makes this a centre-path planner for the complete
  // chassis rather than for a point robot.  Translation is checked with the
  // oriented rectangle; an in-place turn uses its circumscribed radius.
  double vehicle_length{0.0};
  double vehicle_width{0.0};
  double safety_margin{0.0};
  double tracking_margin{0.0};
  bool allow_in_place_turns{false};
  double task_tolerance{0.08};
  double waypoint_spacing{0.35};
  double curve_spacing{0.025};
  double minimum_turning_radius{0.22};
  double maximum_heading_step{0.14};
  double in_place_turn_heading_threshold{0.18};
  std::string default_mode{"coverage"};
  Pose default_start;
  std::vector<Rectangle> free_regions;
  // Known free space immediately outside the drawn arena, for example the
  // physical launch lane. It is never inferred from a map boundary.
  std::vector<Rectangle> staging_regions;
  std::vector<Rectangle> obstacles;
  std::vector<Point> default_targets;
  std::vector<std::string> default_labels;
  std::vector<Point> required_tunnel_points;
  std::vector<std::string> required_tunnel_labels;
  std::vector<std::pair<Point, Point>> tunnel_segments;
  std::vector<std::string> tunnel_segment_labels;
  std::vector<Point> inspection_nodes;
  std::vector<InspectionEdge> inspection_edges;
  PlannerTopics topics;
};

struct PlanResult
{
  bool success{false};
  std::string message;
  std::vector<Point> points;
  std::vector<double> headings;
  std::vector<std::string> visit_order;
  std::vector<std::string> covered_edges;
  std::vector<std::string> deferred_targets;
  bool all_targets_reached{false};
  double length{0.0};
};

// Activation is a separate policy decision from producing a safe preview.
// Partial plans may be inspected and replanned, but must never be handed to
// the navigator as a complete mission.
inline bool ActivationAllowed(bool requested, const PlanResult & result)
{
  return requested && result.success && result.all_targets_reached;
}

class ArenaPlanner
{
public:
  explicit ArenaPlanner(PlannerConfig config);

  static PlannerConfig LoadConfig(const std::string & path);
  PlanResult Plan(
    const Pose & start, const std::vector<Point> & targets,
    const std::vector<std::string> & labels, const std::string & mode,
    const std::vector<std::string> & covered_edges = {}) const;

  const PlannerConfig & config() const {return config_;}
  bool IsFree(const Point & point) const;
  bool PoseIsFree(const Point & point, double yaw) const;
  bool RotationIsFree(const Point & point, double from_yaw, double to_yaw) const;
  bool SegmentIsFree(const Point & start, const Point & end) const;
  double Clearance(const Point & point) const;
  std::vector<int8_t> OccupancyData() const;

private:
  using Cell = std::pair<int, int>;

  struct GridPath
  {
    std::vector<Cell> cells;
    double cost{0.0};
  };

  int Index(int column, int row) const;
  bool InBounds(int column, int row) const;
  bool CellIsFree(int column, int row) const;
  bool BaseSpaceIsFree(const Point & point) const;
  Cell WorldToCell(const Point & point) const;
  Point CellToWorld(const Cell & cell) const;
  GridPath AStar(const Point & start, const Point & goal) const;
  std::vector<Point> Simplify(const std::vector<Point> & points) const;
  std::vector<Point> Smooth(
    const std::vector<Point> & points,
    const std::vector<Point> & required_targets) const;
  std::vector<int> VisitOrder(
    const std::vector<std::vector<double>> & distances,
    const std::string & mode) const;
  PlanResult PlanCoverage(
    const Pose & start, const std::vector<std::string> & covered_edges,
    bool tunnels_only = false, bool non_tunnels_only = false) const;
  void BuildGrid();
  void SnapInspectionNodesToFreeSpace();

  PlannerConfig config_;
  int columns_{0};
  int rows_{0};
  std::vector<bool> base_occupied_;
  std::vector<bool> occupied_;
  std::vector<double> clearance_;
};

double NormalizeAngle(double angle);
double Distance(const Point & left, const Point & right);
double DistanceToSegment(const Point & point, const Point & start, const Point & end);

}  // namespace arena_path_planner

#endif  // ARENA_PATH_PLANNER__PLANNER_HPP_
