#include "arena_path_planner/planner_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace arena_path_planner {
namespace {
constexpr double kPi = 3.14159265358979323846;
}

double NormalizeAngle(double angle) { return std::remainder(angle, 2.0 * kPi); }

bool IsFinite(const Point &point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool IsFinite(const Pose &pose) {
  return IsFinite(pose.position) && std::isfinite(pose.yaw);
}

bool IsFinite(const Rectangle &rectangle) {
  return std::isfinite(rectangle.minimum_x) &&
         std::isfinite(rectangle.minimum_y) &&
         std::isfinite(rectangle.maximum_x) &&
         std::isfinite(rectangle.maximum_y);
}

void ValidatePlannerConfig(const PlannerConfig &config) {
  const auto require_finite = [](double value, const char *name) {
    if (!std::isfinite(value)) {
      throw std::invalid_argument(std::string("planner config ") + name +
                                  " must be finite");
    }
  };
  const auto require_nonnegative = [&](double value, const char *name) {
    require_finite(value, name);
    if (value < 0.0) {
      throw std::invalid_argument(std::string("planner config ") + name +
                                  " must not be negative");
    }
  };
  const auto require_positive = [&](double value, const char *name) {
    require_finite(value, name);
    if (!(value > 0.0)) {
      throw std::invalid_argument(std::string("planner config ") + name +
                                  " must be positive");
    }
  };

  require_positive(config.width, "width");
  require_positive(config.height, "height");
  require_positive(config.resolution, "resolution");
  require_nonnegative(config.inflation_radius, "inflation_radius");
  require_nonnegative(config.preferred_clearance, "preferred_clearance");
  require_nonnegative(config.clearance_cost_weight, "clearance_cost_weight");
  require_nonnegative(config.vehicle_length, "vehicle_length");
  require_nonnegative(config.vehicle_width, "vehicle_width");
  require_nonnegative(config.safety_margin, "safety_margin");
  require_nonnegative(config.tracking_margin, "tracking_margin");
  require_nonnegative(config.task_tolerance, "task_tolerance");
  require_positive(config.waypoint_spacing, "waypoint_spacing");
  require_positive(config.curve_spacing, "curve_spacing");
  require_nonnegative(config.minimum_turning_radius, "minimum_turning_radius");
  require_positive(config.maximum_heading_step, "maximum_heading_step");
  require_nonnegative(config.in_place_turn_heading_threshold,
                      "in_place_turn_heading_threshold");
  if ((config.vehicle_length > 0.0) != (config.vehicle_width > 0.0)) {
    throw std::invalid_argument(
        "planner config vehicle length and width must be configured together");
  }
  const double maximum_physical_scale =
      10.0 * std::max(config.width, config.height);
  if (config.inflation_radius > maximum_physical_scale ||
      config.preferred_clearance > maximum_physical_scale ||
      config.vehicle_length > maximum_physical_scale ||
      config.vehicle_width > maximum_physical_scale ||
      config.safety_margin > maximum_physical_scale ||
      config.tracking_margin > maximum_physical_scale ||
      config.minimum_turning_radius > maximum_physical_scale) {
    throw std::invalid_argument(
        "planner config physical dimensions are unreasonable for the arena");
  }
  if (!IsFinite(config.default_start)) {
    throw std::invalid_argument("planner config default start must be finite");
  }

  // Guard the integer grid dimensions before BuildGrid performs casts and
  // allocations.  The production map is tiny; this also turns a malformed
  // YAML size into a readable startup error instead of an allocation crash.
  const double columns = std::ceil(config.width / config.resolution);
  const double rows = std::ceil(config.height / config.resolution);
  constexpr double kMaxGridCells = 50.0e6;
  if (!std::isfinite(columns) || !std::isfinite(rows) ||
      columns > static_cast<double>(std::numeric_limits<int>::max()) ||
      rows > static_cast<double>(std::numeric_limits<int>::max()) ||
      columns < 1.0 || rows < 1.0 || columns * rows > kMaxGridCells) {
    throw std::invalid_argument("planner config grid dimensions are too large");
  }

  const auto check_rectangle = [&](const Rectangle &rectangle,
                                   const char *name) {
    if (!IsFinite(rectangle) || !(rectangle.minimum_x < rectangle.maximum_x) ||
        !(rectangle.minimum_y < rectangle.maximum_y)) {
      throw std::invalid_argument(std::string("planner config invalid ") +
                                  name + " rectangle");
    }
  };
  for (const Rectangle &rectangle : config.free_regions) {
    check_rectangle(rectangle, "free region");
  }
  for (const Rectangle &rectangle : config.staging_regions) {
    check_rectangle(rectangle, "staging region");
  }
  for (const Rectangle &rectangle : config.obstacles) {
    check_rectangle(rectangle, "obstacle");
  }
  const auto check_points = [&](const std::vector<Point> &points,
                               const char *name) {
    for (const Point &point : points) {
      if (!IsFinite(point)) {
        throw std::invalid_argument(std::string("planner config ") + name +
                                    " contains a non-finite point");
      }
    }
  };
  check_points(config.default_targets, "default targets");
  check_points(config.required_tunnel_points, "tunnel points");
  check_points(config.inspection_nodes, "inspection nodes");
  if (config.default_targets.size() != config.default_labels.size()) {
    throw std::invalid_argument(
        "planner config default targets and labels must have equal length");
  }
  if (config.required_tunnel_points.size() !=
      config.required_tunnel_labels.size()) {
    throw std::invalid_argument(
        "planner config tunnel points and labels must have equal length");
  }
  if (config.tunnel_segments.size() != config.tunnel_segment_labels.size()) {
    throw std::invalid_argument(
        "planner config tunnel segments and labels must have equal length");
  }
  for (const auto &segment : config.tunnel_segments) {
    if (!IsFinite(segment.first) || !IsFinite(segment.second)) {
      throw std::invalid_argument(
          "planner config tunnel segments contain a non-finite point");
    }
  }
  for (const InspectionEdge &edge : config.inspection_edges) {
    if (edge.from < 0 || edge.to < 0 ||
        static_cast<std::size_t>(edge.from) >= config.inspection_nodes.size() ||
        static_cast<std::size_t>(edge.to) >= config.inspection_nodes.size()) {
      throw std::invalid_argument(
          "planner config inspection edge references an invalid node");
    }
    if (edge.from == edge.to || edge.label.empty()) {
      throw std::invalid_argument(
          "planner config inspection edges need distinct nodes and labels");
    }
  }
  std::unordered_set<std::string> labels;
  for (const InspectionEdge &edge : config.inspection_edges) {
    if (!labels.insert(edge.label).second) {
      throw std::invalid_argument(
          "planner config inspection edge labels must be unique");
    }
  }
  const auto require_topic = [](const std::string &topic, const char *name) {
    if (topic.empty()) {
      throw std::invalid_argument(std::string("planner config topic ") + name +
                                  " must not be empty");
    }
  };
  require_topic(config.topics.service, "service");
  require_topic(config.topics.arena_path, "arena_path");
  require_topic(config.topics.navigation_path, "navigation_path");
  require_topic(config.topics.occupancy_grid, "occupancy_grid");
  require_topic(config.topics.route_input, "route_input");
}

double Distance(const Point &left, const Point &right) {
  return std::hypot(right.x - left.x, right.y - left.y);
}

double DistanceToSegment(const Point &point, const Point &start,
                         const Point &end) {
  if (!IsFinite(point) || !IsFinite(start) || !IsFinite(end)) {
    return std::numeric_limits<double>::infinity();
  }
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1.0e-12)
    return Distance(point, start);
  const double ratio = std::clamp(
      ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared,
      0.0, 1.0);
  return Distance(point, {start.x + ratio * dx, start.y + ratio * dy});
}

bool Contains(const Rectangle &rectangle, const Point &point) {
  return point.x >= rectangle.minimum_x && point.x < rectangle.maximum_x &&
         point.y >= rectangle.minimum_y && point.y < rectangle.maximum_y;
}

Point QuadraticBezier(const Point &start, const Point &control,
                      const Point &end, double ratio) {
  const double inverse = 1.0 - ratio;
  return {inverse * inverse * start.x + 2.0 * inverse * ratio * control.x +
              ratio * ratio * end.x,
          inverse * inverse * start.y + 2.0 * inverse * ratio * control.y +
              ratio * ratio * end.y};
}

double PolylineLength(const std::vector<Point> &points) {
  double result = 0.0;
  for (std::size_t index = 1; index < points.size(); ++index) {
    result += Distance(points[index - 1], points[index]);
  }
  return result;
}

} // namespace arena_path_planner
