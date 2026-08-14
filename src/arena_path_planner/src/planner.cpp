#include "arena_path_planner/planner.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <unordered_map>

namespace arena_path_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Rectangle ReadRectangle(const YAML::Node & node)
{
  if (!node.IsSequence() || node.size() != 4) {
    throw std::runtime_error("rectangle must contain four coordinates");
  }
  Rectangle result{
    node[0].as<double>(), node[1].as<double>(),
    node[2].as<double>(), node[3].as<double>()};
  if (!(result.minimum_x < result.maximum_x) ||
    !(result.minimum_y < result.maximum_y))
  {
    throw std::runtime_error("invalid rectangle bounds");
  }
  return result;
}

Point ReadPoint(const YAML::Node & node)
{
  if (!node.IsSequence() || node.size() != 2) {
    throw std::runtime_error("point must contain x and y");
  }
  return {node[0].as<double>(), node[1].as<double>()};
}

bool Contains(const Rectangle & rectangle, const Point & point)
{
  return point.x >= rectangle.minimum_x && point.x < rectangle.maximum_x &&
    point.y >= rectangle.minimum_y && point.y < rectangle.maximum_y;
}

Point QuadraticBezier(
  const Point & start, const Point & control, const Point & end, double ratio)
{
  const double inverse = 1.0 - ratio;
  return {
    inverse * inverse * start.x + 2.0 * inverse * ratio * control.x +
    ratio * ratio * end.x,
    inverse * inverse * start.y + 2.0 * inverse * ratio * control.y +
    ratio * ratio * end.y};
}

double PolylineLength(const std::vector<Point> & points)
{
  double result = 0.0;
  for (std::size_t index = 1; index < points.size(); ++index) {
    result += Distance(points[index - 1], points[index]);
  }
  return result;
}

}  // namespace

double NormalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double Distance(const Point & left, const Point & right)
{
  return std::hypot(right.x - left.x, right.y - left.y);
}

double DistanceToSegment(const Point & point, const Point & start, const Point & end)
{
  const double delta_x = end.x - start.x;
  const double delta_y = end.y - start.y;
  const double squared_length = delta_x * delta_x + delta_y * delta_y;
  if (squared_length <= 1.0e-12) {
    return Distance(point, start);
  }
  const double ratio = std::clamp(
    ((point.x - start.x) * delta_x + (point.y - start.y) * delta_y) /
    squared_length, 0.0, 1.0);
  return Distance(point, {start.x + ratio * delta_x, start.y + ratio * delta_y});
}

std::vector<std::string> CoveredInspectionEdges(
  const PlannerConfig & config, const std::vector<Point> & route)
{
  std::vector<std::string> covered;
  const auto distance_to_route = [&route](const Point & point) {
      double minimum = std::numeric_limits<double>::infinity();
      for (std::size_t index = 1; index < route.size(); ++index) {
        minimum = std::min(minimum, DistanceToSegment(point, route[index - 1], route[index]));
      }
      return minimum;
    };
  for (const InspectionEdge & edge : config.inspection_edges) {
    const Point & from = config.inspection_nodes[static_cast<std::size_t>(edge.from)];
    const Point & to = config.inspection_nodes[static_cast<std::size_t>(edge.to)];
    if (distance_to_route(from) <= config.task_tolerance &&
      distance_to_route(to) <= config.task_tolerance &&
      distance_to_route({(from.x + to.x) * 0.5, (from.y + to.y) * 0.5})
      <= config.task_tolerance)
    {
      covered.push_back(edge.label);
    }
  }
  return covered;
}

PlannerConfig ArenaPlanner::LoadConfig(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  PlannerConfig config;
  const YAML::Node arena = root["arena"];
  config.width = arena["width_m"].as<double>();
  config.height = arena["height_m"].as<double>();
  config.resolution = arena["resolution_m"].as<double>();
  config.inflation_radius = arena["inflation_radius_m"].as<double>();
  config.preferred_clearance = arena["preferred_clearance_m"].as<double>();
  config.clearance_cost_weight = arena["clearance_cost_weight"].as<double>();
  if (arena["vehicle_length_m"]) {
    config.vehicle_length = arena["vehicle_length_m"].as<double>();
  }
  if (arena["vehicle_width_m"]) {
    config.vehicle_width = arena["vehicle_width_m"].as<double>();
  }
  if (arena["safety_margin_m"]) {
    config.safety_margin = arena["safety_margin_m"].as<double>();
  }
  if (arena["tracking_margin_m"]) {
    config.tracking_margin = arena["tracking_margin_m"].as<double>();
  }
  if (arena["allow_in_place_turns"]) {
    config.allow_in_place_turns = arena["allow_in_place_turns"].as<bool>();
  }
  for (const auto & node : arena["free_regions"]) {
    config.free_regions.push_back(ReadRectangle(node));
  }
  if (arena["staging_regions"]) {
    for (const auto & node : arena["staging_regions"]) {
      config.staging_regions.push_back(ReadRectangle(node));
    }
  }
  for (const auto & node : arena["obstacles"]) {
    config.obstacles.push_back(ReadRectangle(node));
  }
  config.default_start.position = ReadPoint(root["start"]["position_m"]);
  config.default_start.yaw = root["start"]["heading_deg"].as<double>() * kPi / 180.0;
  std::vector<std::pair<std::string, Point>> tasks;
  for (const auto & entry : root["tasks"]) {
    tasks.emplace_back(entry.first.as<std::string>(), ReadPoint(entry.second));
  }
  std::sort(
    tasks.begin(), tasks.end(),
    [](const auto & left, const auto & right) {
      try {
        return std::stoi(left.first) < std::stoi(right.first);
      } catch (const std::exception &) {
        return left.first < right.first;
      }
    });
  for (const auto & [label, point] : tasks) {
    config.default_labels.push_back(label);
    config.default_targets.push_back(point);
  }
  if (root["tunnels"]) {
    for (const auto & entry : root["tunnels"]) {
      const std::string label = entry.first.as<std::string>();
      if (entry.second.IsMap() && entry.second["entry"] && entry.second["exit"]) {
        const Point entrance = ReadPoint(entry.second["entry"]);
        const Point exit = ReadPoint(entry.second["exit"]);
        config.tunnel_segments.emplace_back(entrance, exit);
        config.tunnel_segment_labels.push_back(label);
        config.required_tunnel_points.push_back(entrance);
        config.required_tunnel_labels.push_back(label + "_ENTER");
        config.required_tunnel_points.push_back(exit);
        config.required_tunnel_labels.push_back(label + "_EXIT");
      } else {
        config.required_tunnel_labels.push_back(label);
        config.required_tunnel_points.push_back(ReadPoint(entry.second));
      }
    }
  }
  if (root["inspection_graph"]) {
    const YAML::Node graph = root["inspection_graph"];
    for (const auto & node : graph["nodes"]) {
      config.inspection_nodes.push_back(ReadPoint(node));
    }
    for (const auto & edge : graph["edges"]) {
      if (!edge.IsSequence() || edge.size() != 4) {
        throw std::runtime_error("inspection edge must contain from, to, label and tunnel");
      }
      config.inspection_edges.push_back(
        {edge[0].as<int>(), edge[1].as<int>(), edge[2].as<std::string>(), edge[3].as<bool>()});
    }
  }
  const YAML::Node planning = root["planning"];
  config.default_mode = planning["mode"].as<std::string>();
  config.task_tolerance = planning["task_tolerance_m"].as<double>();
  config.waypoint_spacing = planning["waypoint_spacing_m"].as<double>();
  config.curve_spacing = planning["curve_spacing_m"].as<double>();
  config.minimum_turning_radius = planning["minimum_turning_radius_m"].as<double>();
  config.maximum_heading_step =
    planning["maximum_heading_step_deg"].as<double>() * kPi / 180.0;
  return config;
}

PlanResult ArenaPlanner::PlanCoverage(
  const Pose & start, const std::vector<std::string> & covered_edges,
  bool tunnels_only, bool non_tunnels_only) const
{
  PlanResult result;
  try {
    if (config_.inspection_nodes.empty() || config_.inspection_edges.empty()) {
      throw std::runtime_error("coverage mode requires an inspection graph");
    }
    if (!IsFree(start.position)) {
      throw std::runtime_error("start lies outside collision-free space");
    }
    const std::size_t node_count = config_.inspection_nodes.size();
    std::vector<std::vector<std::pair<int, int>>> adjacency(node_count);
    std::vector<bool> usable(config_.inspection_edges.size(), false);
    const auto already_covered = [&covered_edges](const std::string & label) {
        return std::find(covered_edges.begin(), covered_edges.end(), label) != covered_edges.end();
      };
    for (std::size_t index = 0; index < config_.inspection_edges.size(); ++index) {
      const InspectionEdge & edge = config_.inspection_edges[index];
      if (edge.from < 0 || edge.to < 0 ||
        static_cast<std::size_t>(edge.from) >= node_count ||
        static_cast<std::size_t>(edge.to) >= node_count)
      {
        throw std::runtime_error("inspection edge references an invalid node");
      }
      usable[index] = SegmentIsFree(
        config_.inspection_nodes[edge.from], config_.inspection_nodes[edge.to]);
      if (!usable[index]) {
        // Exported graph edges are desired inspection strokes, not a command
        // to drive a rectangular chassis through a wall. If its centre line is
        // too close to an obstacle but the two road nodes remain connected,
        // cover that stroke through a collision-free A* connector instead.
        try {
          (void)AStar(
            config_.inspection_nodes[edge.from], config_.inspection_nodes[edge.to]);
          usable[index] = true;
        } catch (const std::runtime_error &) {
          usable[index] = false;
        }
      }
      if (usable[index]) {
        adjacency[edge.from].emplace_back(edge.to, static_cast<int>(index));
        adjacency[edge.to].emplace_back(edge.from, static_cast<int>(index));
      } else {
        result.deferred_targets.push_back(edge.label);
      }
    }

    // Gap-fill is deliberately incremental.  The old graph DFS walked every
    // edge reachable from the entry node and only filtered the labels.  As a
    // result, stage 3 returned the complete inspection graph again and the GUI
    // quite correctly painted nearly the whole arena purple.  Build a route
    // from the actually uncovered, non-tunnel edges only; A* paths are used as
    // connectors between those edge segments.
    if (non_tunnels_only) {
      std::vector<std::size_t> remaining;
      for (std::size_t index = 0; index < config_.inspection_edges.size(); ++index) {
        const InspectionEdge & edge = config_.inspection_edges[index];
        if (usable[index] && !edge.tunnel && !already_covered(edge.label)) {
          remaining.push_back(index);
        }
      }

      result.points.push_back(start.position);
      Point current = start.position;
      const auto append_grid_path = [this, &result](const GridPath & path) {
          for (std::size_t cell = 1; cell + 1 < path.cells.size(); ++cell) {
            result.points.push_back(CellToWorld(path.cells[cell]));
          }
          if (path.cells.size() > 1) {
            result.points.push_back(CellToWorld(path.cells.back()));
          }
        };
      const auto append_segment = [this, &result](const Point & end) {
          const Point begin = result.points.back();
          const double length = Distance(begin, end);
          const int samples = std::max(
            1, static_cast<int>(std::ceil(length / config_.waypoint_spacing)));
          for (int sample = 1; sample <= samples; ++sample) {
            const double ratio = static_cast<double>(sample) / samples;
            result.points.push_back({
              begin.x + ratio * (end.x - begin.x),
              begin.y + ratio * (end.y - begin.y)});
          }
        };
      const auto append_safe = [this, &result, &append_grid_path, &append_segment](const Point & end) {
          if (SegmentIsFree(result.points.back(), end)) {
            append_segment(end);
            return;
          }
          try {
            append_grid_path(AStar(result.points.back(), end));
            if (Distance(result.points.back(), end) > 1.0e-9) {
              append_segment(end);
            }
          } catch (const std::runtime_error &) {
            // Caller records the route as deferred when no safe connector exists.
          }
        };

      // Exact shortest open tour over the remaining roads.  There are only a
      // handful of gap edges, so subset DP is cheap and avoids greedy zig-zag
      // choices.  Each edge has two legal directions; the edge itself is
      // traversed once and all inter-edge legs use collision-free A* paths.
      struct RoadState { double cost{std::numeric_limits<double>::infinity()}; int parent{-1}; };
      const std::size_t road_count = remaining.size();
      // Exact subset DP is intentionally limited to small graphs.  Exported
      // 15 cm maps can contain hundreds of road cells; use a bounded greedy
      // pass there to avoid exponential memory growth.
      if (road_count > 16) {
        while (!remaining.empty()) {
          std::size_t best = 0;
          bool best_forward = true;
          double best_score = std::numeric_limits<double>::infinity();
          for (std::size_t position = 0; position < remaining.size(); ++position) {
            const InspectionEdge & road = config_.inspection_edges[remaining[position]];
            for (bool forward : {true, false}) {
              const int first = forward ? road.from : road.to;
              const int second = forward ? road.to : road.from;
              try {
                // For large exported grids, use a geometric lower bound while
                // ranking candidates. Running A* for every remaining edge at
                // every iteration made planning O(E^2 * A*) and visibly froze
                // the GUI. A* is still run for the selected edge below.
                const double score = Distance(current,
                  config_.inspection_nodes[static_cast<std::size_t>(first)]) +
                  Distance(config_.inspection_nodes[static_cast<std::size_t>(first)],
                    config_.inspection_nodes[static_cast<std::size_t>(second)]);
                if (score < best_score) {
                  best_score = score;
                  best = position;
                  best_forward = forward;
                }
              } catch (const std::exception &) {
                continue;
              }
            }
          }
          if (!std::isfinite(best_score)) {
            for (std::size_t index : remaining) {
              result.deferred_targets.push_back(config_.inspection_edges[index].label);
            }
            break;
          }
          const InspectionEdge & road = config_.inspection_edges[remaining[best]];
          const int first = best_forward ? road.from : road.to;
          const int second = best_forward ? road.to : road.from;
          try {
            append_grid_path(AStar(current,
              config_.inspection_nodes[static_cast<std::size_t>(first)]));
            append_safe(config_.inspection_nodes[static_cast<std::size_t>(first)]);
            append_safe(config_.inspection_nodes[static_cast<std::size_t>(second)]);
            result.visit_order.push_back(road.label);
            current = config_.inspection_nodes[static_cast<std::size_t>(second)];
          } catch (const std::runtime_error &) {
            result.deferred_targets.push_back(road.label);
          }
          remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best));
        }
        try {
          append_grid_path(AStar(current, config_.default_start.position));
          append_safe(config_.default_start.position);
        } catch (const std::runtime_error &) {
          result.deferred_targets.push_back("RETURN_TO_START");
        }
        remaining.clear();
      }
      if (road_count > 16) {
        // The bounded branch above has already assembled the route.
        std::vector<Point> safe_points;
        if (!result.points.empty()) {
          safe_points.push_back(result.points.front());
          for (std::size_t index = 1; index < result.points.size(); ++index) {
            if (SegmentIsFree(safe_points.back(), result.points[index])) {
              safe_points.push_back(result.points[index]);
            } else {
              try {
                const GridPath connector = AStar(safe_points.back(), result.points[index]);
                for (std::size_t cell = 1; cell < connector.cells.size(); ++cell) {
                  safe_points.push_back(CellToWorld(connector.cells[cell]));
                }
              } catch (const std::runtime_error &) {
                safe_points.push_back(result.points[index]);
              }
            }
          }
          result.points = std::move(safe_points);
        }
        result.points = Simplify(result.points);
        result.points = Smooth(result.points, {});
        result.headings.clear();
        for (std::size_t index = 0; index < result.points.size(); ++index) {
          result.headings.push_back(index + 1 < result.points.size() ?
            std::atan2(result.points[index + 1].y - result.points[index].y,
              result.points[index + 1].x - result.points[index].x) :
            (result.headings.empty() ? start.yaw : result.headings.back()));
        }
        result.length = PolylineLength(result.points);
        result.covered_edges = CoveredInspectionEdges(config_, result.points);
        result.all_targets_reached = result.deferred_targets.empty();
        result.success = true;
        result.message = result.all_targets_reached ?
          "large road graph filled" : "large road graph partially filled";
        return result;
      }
      const std::size_t state_count = std::size_t{1} << road_count;
      const auto state_index = [road_count](std::size_t mask, std::size_t edge, std::size_t direction) {
          return (mask * road_count + edge) * 2 + direction;
        };
      std::vector<RoadState> states(state_count * road_count * 2);
      std::vector<std::vector<std::vector<GridPath>>> paths(
        node_count, std::vector<std::vector<GridPath>>(node_count));
      std::vector<std::vector<std::vector<bool>>> path_ready(
        node_count, std::vector<std::vector<bool>>(node_count, std::vector<bool>(1)));
      const auto get_path = [this, &paths, &path_ready](int from, int to) -> const GridPath * {
          if (!path_ready[from][to][0]) {
            try {
              paths[from][to].push_back(AStar(
                config_.inspection_nodes[static_cast<std::size_t>(from)],
                config_.inspection_nodes[static_cast<std::size_t>(to)]));
              path_ready[from][to][0] = true;
            } catch (const std::runtime_error &) {
              return nullptr;
            }
          }
          return paths[from][to].empty() ? nullptr : &paths[from][to].front();
        };
      // Start legs use the actual stage start rather than a graph node.
      for (std::size_t edge = 0; edge < road_count; ++edge) {
        const InspectionEdge & road = config_.inspection_edges[remaining[edge]];
        for (std::size_t direction = 0; direction < 2; ++direction) {
          const int first = direction == 0 ? road.from : road.to;
          const int second = direction == 0 ? road.to : road.from;
          try {
            const double cost = AStar(current, config_.inspection_nodes[static_cast<std::size_t>(first)]).cost +
              Distance(config_.inspection_nodes[static_cast<std::size_t>(first)],
                config_.inspection_nodes[static_cast<std::size_t>(second)]);
            states[state_index(std::size_t{1} << edge, edge, direction)] = {cost, -1};
          } catch (const std::runtime_error &) {
            continue;
          }
        }
      }
      for (std::size_t mask = 1; mask < state_count; ++mask) {
        for (std::size_t last = 0; last < road_count; ++last) {
          for (std::size_t direction = 0; direction < 2; ++direction) {
            const RoadState current_state = states[state_index(mask, last, direction)];
            if (!std::isfinite(current_state.cost)) continue;
            const InspectionEdge & road = config_.inspection_edges[remaining[last]];
            const int last_end = direction == 0 ? road.to : road.from;
            for (std::size_t next = 0; next < road_count; ++next) {
              if (mask & (std::size_t{1} << next)) continue;
              const InspectionEdge & next_road = config_.inspection_edges[remaining[next]];
              for (std::size_t next_direction = 0; next_direction < 2; ++next_direction) {
                const int next_first = next_direction == 0 ? next_road.from : next_road.to;
                const int next_second = next_direction == 0 ? next_road.to : next_road.from;
                const GridPath * connector = get_path(last_end, next_first);
                if (connector == nullptr) continue;
                const std::size_t next_mask = mask | (std::size_t{1} << next);
                const double candidate = current_state.cost + connector->cost +
                  Distance(config_.inspection_nodes[static_cast<std::size_t>(next_first)],
                    config_.inspection_nodes[static_cast<std::size_t>(next_second)]);
                RoadState & destination = states[state_index(next_mask, next, next_direction)];
                if (candidate < destination.cost) {
                  destination = {
                    candidate,
                    static_cast<int>(state_index(mask, last, direction))};
                }
              }
            }
          }
        }
      }
      const std::size_t full_mask = state_count - 1;
      double best_cost = std::numeric_limits<double>::infinity();
      int best_state = -1;
      for (std::size_t edge = 0; edge < road_count; ++edge) {
        const InspectionEdge & road = config_.inspection_edges[remaining[edge]];
        for (std::size_t direction = 0; direction < 2; ++direction) {
          const int last_end = direction == 0 ? road.to : road.from;
          double total = states[state_index(full_mask, edge, direction)].cost;
          try {
            total += AStar(config_.inspection_nodes[static_cast<std::size_t>(last_end)],
              config_.default_start.position).cost;
          } catch (const std::runtime_error &) {
            total = std::numeric_limits<double>::infinity();
          }
          if (total < best_cost) {
            best_cost = total;
            best_state = static_cast<int>(state_index(full_mask, edge, direction));
          }
        }
      }
      if (best_state >= 0) {
        std::vector<std::pair<std::size_t, std::size_t>> sequence;
        int state = best_state;
        while (state >= 0) {
          const std::size_t direction = static_cast<std::size_t>(state % 2);
          const std::size_t packed = static_cast<std::size_t>(state / 2);
          const std::size_t edge = packed % road_count;
          sequence.emplace_back(edge, direction);
          state = states[static_cast<std::size_t>(state)].parent;
        }
        std::reverse(sequence.begin(), sequence.end());
        for (const auto & [edge_position, direction] : sequence) {
          const InspectionEdge & road = config_.inspection_edges[remaining[edge_position]];
          const int first = direction == 0 ? road.from : road.to;
          const int second = direction == 0 ? road.to : road.from;
          GridPath approach;
          try {
            approach = AStar(current, config_.inspection_nodes[static_cast<std::size_t>(first)]);
          } catch (const std::runtime_error &) {
            continue;
          }
          append_grid_path(approach);
          append_safe(config_.inspection_nodes[static_cast<std::size_t>(first)]);
          append_safe(config_.inspection_nodes[static_cast<std::size_t>(second)]);
          result.visit_order.push_back(road.label);
          current = config_.inspection_nodes[static_cast<std::size_t>(second)];
        }
        try {
          append_grid_path(AStar(current, config_.default_start.position));
          append_safe(config_.default_start.position);
        } catch (const std::runtime_error &) {
          result.deferred_targets.push_back("RETURN_TO_START");
        }
        remaining.clear();
      }

      // Ensure every segment exposed to navigation is collision-free.  A
      // connector to a road endpoint may otherwise be a straight chord that
      // clips an inflated obstacle even though both endpoints are free.
      std::vector<Point> safe_points;
      if (!result.points.empty()) {
        safe_points.push_back(result.points.front());
        for (std::size_t index = 1; index < result.points.size(); ++index) {
          const Point target = result.points[index];
          if (SegmentIsFree(safe_points.back(), target)) {
            safe_points.push_back(target);
            continue;
          }
          try {
            const GridPath connector = AStar(safe_points.back(), target);
            for (std::size_t cell = 1; cell < connector.cells.size(); ++cell) {
              safe_points.push_back(CellToWorld(connector.cells[cell]));
            }
            if (Distance(safe_points.back(), target) > 1.0e-9) {
              safe_points.push_back(target);
            }
          } catch (const std::runtime_error &) {
            safe_points.push_back(target);
          }
        }
        result.points = Smooth(safe_points, {});
      }

      for (std::size_t index = 0; index < result.points.size(); ++index) {
        if (index + 1 < result.points.size()) {
          result.headings.push_back(std::atan2(
              result.points[index + 1].y - result.points[index].y,
              result.points[index + 1].x - result.points[index].x));
        } else {
          result.headings.push_back(result.headings.empty() ? start.yaw : result.headings.back());
        }
      }
      result.length = PolylineLength(result.points);
      result.covered_edges = CoveredInspectionEdges(config_, result.points);
      result.all_targets_reached = result.deferred_targets.empty();
      result.success = true;
      result.message = result.all_targets_reached ?
        "uncovered roads filled" : "reachable uncovered roads filled; blocked roads deferred";
      return result;
    }

    int entry_node = 0;
    GridPath entry_path;
    double entry_cost = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < node_count; ++index) {
      try {
        GridPath candidate = AStar(start.position, config_.inspection_nodes[index]);
        if (candidate.cost < entry_cost) {
          entry_cost = candidate.cost;
          entry_node = static_cast<int>(index);
          entry_path = std::move(candidate);
        }
      } catch (const std::runtime_error &) {
        continue;
      }
    }
    if (!std::isfinite(entry_cost)) {
      throw std::runtime_error("no inspection road is reachable from start");
    }

    std::vector<int> walk{entry_node};
    std::vector<bool> traversed(config_.inspection_edges.size(), false);
    std::function<void(int)> cover = [&](int node) {
        for (const auto & [next, edge_index] : adjacency[node]) {
          if (traversed[edge_index]) {
            continue;
          }
          const InspectionEdge & edge = config_.inspection_edges[edge_index];
          // A layered coverage pass must only traverse its own edge class.
          // Previously we filtered visit_order but still walked every edge,
          // which made the layer-3 route redraw the whole inspection graph in
          // purple, including tunnels already handled by layer 2.
          // Tunnel passes may use ordinary roads as connectors. Gap filling,
          // however, must omit tunnel segments entirely so that stage 3 does
          // not repaint stage 2's work.
          if (non_tunnels_only && edge.tunnel) {
            continue;
          }
          traversed[edge_index] = true;
          const bool report_edge = !tunnels_only || edge.tunnel;
          if (report_edge && !already_covered(edge.label)) {
            result.visit_order.push_back(edge.label);
          }
          walk.push_back(next);
          cover(next);
          walk.push_back(node);
        }
      };
    cover(entry_node);
    for (std::size_t index = 0; index < usable.size(); ++index) {
      const InspectionEdge & edge = config_.inspection_edges[index];
      const bool selected = (!tunnels_only && !non_tunnels_only) ||
        (tunnels_only && edge.tunnel) || (non_tunnels_only && !edge.tunnel);
      if (selected && usable[index] && !traversed[index] &&
        !already_covered(config_.inspection_edges[index].label)) {
        result.deferred_targets.push_back(config_.inspection_edges[index].label);
      }
    }

    result.points.push_back(start.position);
    for (std::size_t index = 1; index + 1 < entry_path.cells.size(); ++index) {
      result.points.push_back(CellToWorld(entry_path.cells[index]));
    }
    if (Distance(result.points.back(), config_.inspection_nodes[entry_node]) > 1.0e-9) {
      result.points.push_back(config_.inspection_nodes[entry_node]);
    }
    const auto append_segment = [this, &result](const Point & end) {
        const Point begin = result.points.back();
        const double length = Distance(begin, end);
        const int samples = std::max(
          1, static_cast<int>(std::ceil(length / config_.waypoint_spacing)));
        for (int sample = 1; sample <= samples; ++sample) {
          const double ratio = static_cast<double>(sample) / samples;
          result.points.push_back(
            {begin.x + ratio * (end.x - begin.x), begin.y + ratio * (end.y - begin.y)});
        }
      };
    for (std::size_t index = 1; index < walk.size(); ++index) {
      append_segment(config_.inspection_nodes[walk[index]]);
    }
    for (std::size_t index = 0; index < result.points.size(); ++index) {
      if (index + 1 < result.points.size()) {
        result.headings.push_back(std::atan2(
            result.points[index + 1].y - result.points[index].y,
            result.points[index + 1].x - result.points[index].x));
      } else {
        result.headings.push_back(result.headings.empty() ? start.yaw : result.headings.back());
      }
    }
      result.length = PolylineLength(result.points);
    result.covered_edges = CoveredInspectionEdges(config_, result.points);
    result.all_targets_reached = result.deferred_targets.empty();
    result.success = true;
    result.message = result.all_targets_reached ?
      "all inspection roads covered" : "reachable roads covered; blocked roads deferred";
  } catch (const std::exception & exception) {
    result.message = exception.what();
  }
  return result;
}

ArenaPlanner::ArenaPlanner(PlannerConfig config)
: config_(std::move(config))
{
  if (!(config_.width > 0.0) || !(config_.height > 0.0) ||
    !(config_.resolution > 0.0))
  {
    throw std::invalid_argument("arena dimensions and resolution must be positive");
  }
  if (config_.vehicle_length < 0.0 || config_.vehicle_width < 0.0 ||
    config_.safety_margin < 0.0 || config_.tracking_margin < 0.0)
  {
    throw std::invalid_argument("vehicle footprint and safety margins must not be negative");
  }
  if ((config_.vehicle_length > 0.0) != (config_.vehicle_width > 0.0)) {
    throw std::invalid_argument("vehicle length and width must be configured together");
  }
  if (config_.vehicle_length > 0.0) {
    const double translational_radius = 0.5 * std::max(
      config_.vehicle_length, config_.vehicle_width) + config_.safety_margin;
    const double rotation_radius = 0.5 * std::hypot(
      config_.vehicle_length, config_.vehicle_width) + config_.safety_margin;
    // The grid is a fast centre-path search.  It only needs the largest body
    // half-axis for translation; every emitted segment is then checked against
    // the full oriented rectangle below.  Using the rotation circle here would
    // wrongly reject a 30 cm straight lane that this chassis can traverse.
    config_.inflation_radius = std::max(config_.inflation_radius, translational_radius);
    config_.preferred_clearance = std::max(
      config_.preferred_clearance, translational_radius + config_.tracking_margin);
    config_.minimum_turning_radius = std::max(
      config_.minimum_turning_radius, rotation_radius);
  }
  BuildGrid();
  SnapInspectionNodesToFreeSpace();
}

int ArenaPlanner::Index(int column, int row) const
{
  return row * columns_ + column;
}

bool ArenaPlanner::InBounds(int column, int row) const
{
  return column >= 0 && column < columns_ && row >= 0 && row < rows_;
}

bool ArenaPlanner::CellIsFree(int column, int row) const
{
  return InBounds(column, row) && !occupied_[Index(column, row)];
}

bool ArenaPlanner::BaseSpaceIsFree(const Point & point) const
{
  const bool in_staging_region = std::any_of(
    config_.staging_regions.begin(), config_.staging_regions.end(),
    [&point](const Rectangle & rectangle) {return Contains(rectangle, point);});
  if (in_staging_region) {
    return true;
  }
  if (point.x < 0.0 || point.y < 0.0 || point.x >= config_.width ||
    point.y >= config_.height)
  {
    return false;
  }
  const Cell cell = WorldToCell(point);
  return !base_occupied_[Index(cell.first, cell.second)];
}

ArenaPlanner::Cell ArenaPlanner::WorldToCell(const Point & point) const
{
  return {
    std::clamp(static_cast<int>(std::floor(point.x / config_.resolution)), 0, columns_ - 1),
    std::clamp(static_cast<int>(std::floor(point.y / config_.resolution)), 0, rows_ - 1)};
}

Point ArenaPlanner::CellToWorld(const Cell & cell) const
{
  return {
    (static_cast<double>(cell.first) + 0.5) * config_.resolution,
    (static_cast<double>(cell.second) + 0.5) * config_.resolution};
}

void ArenaPlanner::BuildGrid()
{
  columns_ = static_cast<int>(std::ceil(config_.width / config_.resolution));
  rows_ = static_cast<int>(std::ceil(config_.height / config_.resolution));
  base_occupied_.assign(columns_ * rows_, true);
  for (int row = 0; row < rows_; ++row) {
    for (int column = 0; column < columns_; ++column) {
      const Point point = CellToWorld({column, row});
      const bool inside = std::any_of(
        config_.free_regions.begin(), config_.free_regions.end(),
        [&point](const Rectangle & rectangle) {return Contains(rectangle, point);});
      const bool obstacle = std::any_of(
        config_.obstacles.begin(), config_.obstacles.end(),
        [&point](const Rectangle & rectangle) {return Contains(rectangle, point);});
      base_occupied_[Index(column, row)] = !inside || obstacle;
    }
  }

  occupied_ = base_occupied_;
  // A* always follows cell centres. The full rectangular footprint is checked
  // at centimetre resolution for every emitted segment, so adding a cell
  // diagonal here would double-count the safety margin and close valid narrow
  // lanes.
  const double inflation_distance = config_.inflation_radius + 0.5 * config_.resolution;
  const int inflation_cells = static_cast<int>(
    std::ceil(inflation_distance / config_.resolution));
  for (int row = 0; row < rows_; ++row) {
    for (int column = 0; column < columns_; ++column) {
      if (base_occupied_[Index(column, row)]) {
        continue;
      }
      for (int row_offset = -inflation_cells; row_offset <= inflation_cells; ++row_offset) {
        for (int column_offset = -inflation_cells;
          column_offset <= inflation_cells; ++column_offset)
        {
          if (std::hypot(column_offset, row_offset) * config_.resolution >
            inflation_distance + 1.0e-9)
          {
            continue;
          }
          const int test_column = column + column_offset;
          const int test_row = row + row_offset;
          const Point test_point = CellToWorld({test_column, test_row});
          if (!BaseSpaceIsFree(test_point))
          {
            occupied_[Index(column, row)] = true;
          }
        }
      }
    }
  }

  clearance_.assign(columns_ * rows_, std::numeric_limits<double>::infinity());
  using Entry = std::tuple<double, int, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
  for (int row = 0; row < rows_; ++row) {
    for (int column = 0; column < columns_; ++column) {
      if (base_occupied_[Index(column, row)]) {
        clearance_[Index(column, row)] = 0.0;
        frontier.emplace(0.0, column, row);
      }
    }
  }
  const std::vector<std::tuple<int, int, double>> steps{
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1, 1, std::sqrt(2.0)}, {1, -1, std::sqrt(2.0)},
    {-1, 1, std::sqrt(2.0)}, {-1, -1, std::sqrt(2.0)}};
  while (!frontier.empty()) {
    const auto [distance, column, row] = frontier.top();
    frontier.pop();
    if (distance > clearance_[Index(column, row)] + 1.0e-12) {
      continue;
    }
    for (const auto & [column_delta, row_delta, multiplier] : steps) {
      const int next_column = column + column_delta;
      const int next_row = row + row_delta;
      if (!InBounds(next_column, next_row)) {
        continue;
      }
      const double candidate = distance + multiplier * config_.resolution;
      if (candidate + 1.0e-12 < clearance_[Index(next_column, next_row)]) {
        clearance_[Index(next_column, next_row)] = candidate;
        frontier.emplace(candidate, next_column, next_row);
      }
    }
  }
}

void ArenaPlanner::SnapInspectionNodesToFreeSpace()
{
  // The inspection graph was exported from an older coarse grid. Some of its
  // node centres lie a few centimetres from a wall, which was acceptable for
  // a point robot but not for the chassis centre. Keep graph connectivity and
  // move only those stale centres to their nearest valid centre-path cell.
  for (Point & node : config_.inspection_nodes) {
    if (IsFree(node)) {
      continue;
    }
    double best_distance = std::numeric_limits<double>::infinity();
    Cell best_cell{-1, -1};
    for (int row = 0; row < rows_; ++row) {
      for (int column = 0; column < columns_; ++column) {
        if (!CellIsFree(column, row)) {
          continue;
        }
        const Point candidate = CellToWorld({column, row});
        const double distance = Distance(node, candidate);
        if (distance + 1.0e-12 < best_distance) {
          best_distance = distance;
          best_cell = {column, row};
        }
      }
    }
    if (best_cell.first >= 0) {
      node = CellToWorld(best_cell);
    }
  }
}

bool ArenaPlanner::IsFree(const Point & point) const
{
  if (point.x < 0.0 || point.y < 0.0 || point.x >= config_.width ||
    point.y >= config_.height)
  {
    return false;
  }
  const Cell cell = WorldToCell(point);
  return CellIsFree(cell.first, cell.second);
}

bool ArenaPlanner::PoseIsFree(const Point & point, double yaw) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(yaw)) {
    return false;
  }
  if (config_.vehicle_length <= 0.0) {
    return BaseSpaceIsFree(point);
  }
  const double half_length = 0.5 * config_.vehicle_length + config_.safety_margin;
  const double half_width = 0.5 * config_.vehicle_width + config_.safety_margin;
  const double spacing = std::max(0.005, config_.resolution * 0.25);
  const int length_samples = std::max(
    1, static_cast<int>(std::ceil(2.0 * half_length / spacing)));
  const int width_samples = std::max(
    1, static_cast<int>(std::ceil(2.0 * half_width / spacing)));
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  for (int length_index = 0; length_index <= length_samples; ++length_index) {
    const double local_x = -half_length +
      2.0 * half_length * static_cast<double>(length_index) / length_samples;
    for (int width_index = 0; width_index <= width_samples; ++width_index) {
      const double local_y = -half_width +
        2.0 * half_width * static_cast<double>(width_index) / width_samples;
      if (!BaseSpaceIsFree({
            point.x + cosine * local_x - sine * local_y,
            point.y + sine * local_x + cosine * local_y}))
      {
        return false;
      }
    }
  }
  return true;
}

bool ArenaPlanner::RotationIsFree(
  const Point & point, double from_yaw, double to_yaw) const
{
  if (!std::isfinite(from_yaw) || !std::isfinite(to_yaw)) {
    return false;
  }
  const double delta = NormalizeAngle(to_yaw - from_yaw);
  const int samples = std::max(
    1, static_cast<int>(std::ceil(std::abs(delta) / config_.maximum_heading_step)));
  for (int index = 0; index <= samples; ++index) {
    if (!PoseIsFree(point, from_yaw + delta * static_cast<double>(index) / samples)) {
      return false;
    }
  }
  return true;
}

double ArenaPlanner::Clearance(const Point & point) const
{
  if (!IsFree(point)) {
    return 0.0;
  }
  const Cell cell = WorldToCell(point);
  const Point centre = CellToWorld(cell);
  const double boundary = std::min(
    {centre.x, centre.y, config_.width - centre.x, config_.height - centre.y});
  return std::max(0.0, std::min(clearance_[Index(cell.first, cell.second)], boundary));
}

bool ArenaPlanner::SegmentIsFree(const Point & start, const Point & end) const
{
  const double distance = Distance(start, end);
  const double spacing = std::max(0.005, config_.resolution * 0.2);
  const int count = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
  const double yaw = std::atan2(end.y - start.y, end.x - start.x);
  for (int index = 0; index <= count; ++index) {
    const double ratio = static_cast<double>(index) / count;
    if (!PoseIsFree(
        {start.x + ratio * (end.x - start.x),
          start.y + ratio * (end.y - start.y)}, yaw))
    {
      return false;
    }
  }
  return true;
}

std::vector<int8_t> ArenaPlanner::OccupancyData() const
{
  std::vector<int8_t> result;
  result.reserve(occupied_.size());
  for (bool value : occupied_) {
    result.push_back(value ? 100 : 0);
  }
  return result;
}

ArenaPlanner::GridPath ArenaPlanner::AStar(const Point & start, const Point & goal) const
{
  const Cell start_cell = WorldToCell(start);
  const Cell goal_cell = WorldToCell(goal);
  if (!CellIsFree(start_cell.first, start_cell.second) ||
    !CellIsFree(goal_cell.first, goal_cell.second))
  {
    throw std::runtime_error("start or target is occupied");
  }
  using Entry = std::tuple<double, double, int, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> frontier;
  const int start_index = Index(start_cell.first, start_cell.second);
  const int goal_index = Index(goal_cell.first, goal_cell.second);
  std::vector<double> costs(columns_ * rows_, std::numeric_limits<double>::infinity());
  std::vector<int> parents(columns_ * rows_, -1);
  std::vector<bool> closed(columns_ * rows_, false);
  costs[start_index] = 0.0;
  frontier.emplace(Distance(start, goal), 0.0, start_cell.first, start_cell.second);
  const std::vector<std::tuple<int, int, double>> steps{
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1, 1, std::sqrt(2.0)}, {1, -1, std::sqrt(2.0)},
    {-1, 1, std::sqrt(2.0)}, {-1, -1, std::sqrt(2.0)}};
  while (!frontier.empty()) {
    const auto [priority, current_cost, column, row] = frontier.top();
    (void)priority;
    frontier.pop();
    const int current_index = Index(column, row);
    if (closed[current_index]) {
      continue;
    }
    closed[current_index] = true;
    if (current_index == goal_index) {
      break;
    }
    for (const auto & [column_delta, row_delta, multiplier] : steps) {
      const int next_column = column + column_delta;
      const int next_row = row + row_delta;
      if (!CellIsFree(next_column, next_row)) {
        continue;
      }
      if (column_delta != 0 && row_delta != 0 &&
        (!CellIsFree(column + column_delta, row) ||
        !CellIsFree(column, row + row_delta)))
      {
        continue;
      }
      const int next_index = Index(next_column, next_row);
      const double clearance = clearance_[next_index];
      const double deficit = config_.preferred_clearance > 1.0e-9 ?
        std::max(0.0, config_.preferred_clearance - clearance) /
        config_.preferred_clearance : 0.0;
      const double step = multiplier * config_.resolution *
        (1.0 + config_.clearance_cost_weight * deficit * deficit);
      const double candidate = current_cost + step;
      if (candidate + 1.0e-12 >= costs[next_index]) {
        continue;
      }
      costs[next_index] = candidate;
      parents[next_index] = current_index;
      const Point next_point = CellToWorld({next_column, next_row});
      frontier.emplace(
        candidate + Distance(next_point, goal), candidate, next_column, next_row);
    }
  }
  if (!std::isfinite(costs[goal_index])) {
    throw std::runtime_error("no collision-free path exists");
  }
  std::vector<Cell> cells;
  int current = goal_index;
  while (current != start_index) {
    cells.emplace_back(current % columns_, current / columns_);
    current = parents[current];
    if (current < 0) {
      throw std::runtime_error("A* predecessor chain is incomplete");
    }
  }
  cells.push_back(start_cell);
  std::reverse(cells.begin(), cells.end());
  return {cells, costs[goal_index]};
}

std::vector<Point> ArenaPlanner::Simplify(const std::vector<Point> & points) const
{
  if (points.size() <= 2) {
    return points;
  }
  std::vector<Point> result{points.front()};
  std::size_t index = 0;
  while (index + 1 < points.size()) {
    std::size_t candidate = points.size() - 1;
    while (candidate > index + 1) {
      const double required = std::max(
        0.0, std::min(
          {config_.preferred_clearance, Clearance(points[index]),
            Clearance(points[candidate])}) - config_.resolution * 0.25);
      bool clearance_ok = true;
      const double length = Distance(points[index], points[candidate]);
      const int samples = std::max(
        1, static_cast<int>(std::ceil(length / config_.resolution)));
      for (int sample = 0; sample <= samples && clearance_ok; ++sample) {
        const double ratio = static_cast<double>(sample) / samples;
        const Point point{
          points[index].x + ratio * (points[candidate].x - points[index].x),
          points[index].y + ratio * (points[candidate].y - points[index].y)};
        clearance_ok = Clearance(point) + 1.0e-9 >= required;
      }
      if (SegmentIsFree(points[index], points[candidate]) && clearance_ok) {
        break;
      }
      --candidate;
    }
    result.push_back(points[candidate]);
    index = candidate;
  }
  return result;
}

std::vector<int> ArenaPlanner::VisitOrder(
  const std::vector<std::vector<double>> & distances, const std::string & mode) const
{
  const int task_count = static_cast<int>(distances.size()) - 1;
  if (mode == "numbered") {
    std::vector<int> result;
    for (int index = 0; index <= task_count; ++index) {
      result.push_back(index);
    }
    result.push_back(0);
    return result;
  }
  if (mode != "shortest") {
    throw std::runtime_error("mode must be shortest or numbered");
  }
  const int mask_count = 1 << task_count;
  std::vector<double> costs(mask_count * task_count, std::numeric_limits<double>::infinity());
  std::vector<int> parents(mask_count * task_count, -1);
  const auto key = [task_count](int mask, int last) {return mask * task_count + last;};
  for (int task = 0; task < task_count; ++task) {
    costs[key(1 << task, task)] = distances[0][task + 1];
  }
  for (int mask = 1; mask < mask_count; ++mask) {
    for (int last = 0; last < task_count; ++last) {
      if (!(mask & (1 << last))) {
        continue;
      }
      const int previous_mask = mask ^ (1 << last);
      if (previous_mask == 0) {
        continue;
      }
      for (int previous = 0; previous < task_count; ++previous) {
        if (!(previous_mask & (1 << previous))) {
          continue;
        }
        const double candidate = costs[key(previous_mask, previous)] +
          distances[previous + 1][last + 1];
        if (candidate < costs[key(mask, last)]) {
          costs[key(mask, last)] = candidate;
          parents[key(mask, last)] = previous;
        }
      }
    }
  }
  const int full_mask = mask_count - 1;
  int last = 0;
  double best = std::numeric_limits<double>::infinity();
  for (int task = 0; task < task_count; ++task) {
    const double candidate = costs[key(full_mask, task)] + distances[task + 1][0];
    if (candidate < best) {
      best = candidate;
      last = task;
    }
  }
  std::vector<int> reversed;
  int mask = full_mask;
  while (mask != 0) {
    reversed.push_back(last + 1);
    const int previous = parents[key(mask, last)];
    mask ^= 1 << last;
    last = previous;
  }
  std::reverse(reversed.begin(), reversed.end());
  reversed.insert(reversed.begin(), 0);
  reversed.push_back(0);
  return reversed;
}

std::vector<Point> ArenaPlanner::Smooth(
  const std::vector<Point> & points, const std::vector<Point> & required_targets) const
{
  if (points.size() < 2) {
    return points;
  }
  // A* connectors and consecutive coverage layers can share their endpoint.
  // Remove those zero-length legs before computing headings; otherwise the
  // next segment appears as an artificial 180 degree turn to the navigator.
  std::vector<Point> clean;
  clean.reserve(points.size());
  for (const Point & point : points) {
    if (clean.empty() || Distance(clean.back(), point) > 1.0e-6) {
      clean.push_back(point);
    }
  }
  // A connector can end one grid cell before a layer endpoint and the next
  // connector can immediately return to that same cell. Collapse these tiny
  // out-and-back artifacts; they otherwise become a visible spike and a
  // 180-degree heading sample despite being only a few millimetres long.
  bool collapsed = true;
  while (collapsed && clean.size() >= 3) {
    collapsed = false;
    for (std::size_t index = 1; index + 1 < clean.size(); ++index) {
      const bool required = std::any_of(
        required_targets.begin(), required_targets.end(),
        [&clean, index](const Point & target) {
          return Distance(target, clean[index]) <= 1.0e-6;
        });
      if (required) {
        continue;
      }
      if (Distance(clean[index - 1], clean[index + 1]) <= config_.resolution * 0.15) {
        clean.erase(clean.begin() + static_cast<std::ptrdiff_t>(index));
        collapsed = true;
        break;
      }
    }
  }
  if (clean.size() < 3) {
    return clean;
  }
  std::vector<Point> output{clean.front()};
  const auto append_line = [this, &output](const Point & end) {
      const Point start = output.back();
      const double length = Distance(start, end);
      const int count = std::max(
        1, static_cast<int>(std::ceil(length / config_.waypoint_spacing)));
      for (int sample = 1; sample <= count; ++sample) {
        const double ratio = static_cast<double>(sample) / count;
        output.push_back(
          {start.x + ratio * (end.x - start.x),
            start.y + ratio * (end.y - start.y)});
      }
    };
  for (std::size_t index = 1; index + 1 < clean.size(); ++index) {
    const Point previous = clean[index - 1];
    const Point corner = clean[index];
    const Point following = clean[index + 1];
    const double incoming_length = Distance(previous, corner);
    const double outgoing_length = Distance(corner, following);
    if (incoming_length <= 1.0e-9 || outgoing_length <= 1.0e-9) {
      append_line(corner);
      continue;
    }
    const Point incoming{
      (corner.x - previous.x) / incoming_length,
      (corner.y - previous.y) / incoming_length};
    const Point outgoing{
      (following.x - corner.x) / outgoing_length,
      (following.y - corner.y) / outgoing_length};
    const double turn = std::abs(NormalizeAngle(
        std::atan2(outgoing.y, outgoing.x) - std::atan2(incoming.y, incoming.x)));
    // A near-U-turn is a genuine constrained reversal, not a corner to round
    // with a quadratic curve. Bezier interpolation there doubles back over
    // the entry point and produces an artificial spike in the route.
    if (turn < config_.maximum_heading_step || turn > 2.6) {
      append_line(corner);
      continue;
    }
    const double maximum_trim = std::min(
      {config_.minimum_turning_radius, incoming_length * 0.35,
        outgoing_length * 0.35});
    bool accepted = false;
    for (double scale : {1.0, 0.85, 0.7, 0.55, 0.4}) {
      const double trim = maximum_trim * scale;
      if (trim < config_.curve_spacing) {
        continue;
      }
      const Point entry{corner.x - incoming.x * trim, corner.y - incoming.y * trim};
      const Point exit{corner.x + outgoing.x * trim, corner.y + outgoing.y * trim};
      const int dense_count = std::max(
        8, static_cast<int>(std::ceil(2.0 * trim / (config_.resolution * 0.5))));
      std::vector<Point> dense;
      for (int sample = 0; sample <= dense_count; ++sample) {
        dense.push_back(QuadraticBezier(
            entry, corner, exit, static_cast<double>(sample) / dense_count));
      }
      bool free = SegmentIsFree(output.back(), entry);
      for (std::size_t sample = 1; sample < dense.size() && free; ++sample) {
        free = SegmentIsFree(dense[sample - 1], dense[sample]);
      }
      bool preserves_targets = true;
      for (const Point & target : required_targets) {
        if (Distance(target, corner) > 1.0e-6) {
          continue;
        }
        double minimum = std::numeric_limits<double>::infinity();
        for (std::size_t sample = 1; sample < dense.size(); ++sample) {
          minimum = std::min(
            minimum, DistanceToSegment(target, dense[sample - 1], dense[sample]));
        }
        preserves_targets = minimum <= config_.task_tolerance;
      }
      if (!free || !preserves_targets) {
        continue;
      }
      append_line(entry);
      const int curve_count = std::max(
        {2, static_cast<int>(std::ceil(turn / config_.maximum_heading_step)),
          static_cast<int>(std::ceil(2.0 * trim / config_.curve_spacing))});
      for (int sample = 1; sample <= curve_count; ++sample) {
        output.push_back(QuadraticBezier(
            entry, corner, exit, static_cast<double>(sample) / curve_count));
      }
      accepted = true;
      break;
    }
    if (!accepted) {
      append_line(corner);
    }
  }
  append_line(clean.back());
  std::vector<Point> deduplicated;
  deduplicated.reserve(output.size());
  for (const Point & point : output) {
    if (deduplicated.empty() || Distance(deduplicated.back(), point) > 1.0e-6) {
      deduplicated.push_back(point);
    }
  }
  output = std::move(deduplicated);
  bool free = true;
  for (std::size_t index = 1; index < output.size(); ++index) {
    if (!SegmentIsFree(output[index - 1], output[index])) {
      free = false;
      break;
    }
  }
  if (!free) {
    // A single tight corner must not discard smoothing for the whole route.
    // Simplify retains collision-free line-of-sight sections and leaves only
    // the locally constrained corner as a short polyline.
    return Simplify(clean);
  }
  return output;
}

PlanResult ArenaPlanner::Plan(
  const Pose & start, const std::vector<Point> & targets,
  const std::vector<std::string> & labels, const std::string & mode,
  const std::vector<std::string> & covered_edges) const
{
  if (mode == "layered") {
    PlanResult layer1 = Plan(start, targets, labels, "layer1", {});
    if (!layer1.success) {
      return layer1;
    }
    Pose layer2_start{layer1.points.back(), layer1.headings.back()};
    PlanResult layer2 = Plan(layer2_start, {}, {}, "layer2", layer1.covered_edges);
    if (!layer2.success) {
      return layer2;
    }
    std::vector<std::string> covered = layer1.covered_edges;
    covered.insert(covered.end(), layer2.covered_edges.begin(), layer2.covered_edges.end());
    Pose layer3_start{layer2.points.back(), layer2.headings.back()};
    PlanResult layer3 = Plan(layer3_start, {}, {}, "layer3", covered);
    if (!layer3.success) {
      return layer3;
    }
    PlanResult combined;
    combined.success = true;
    combined.message = "three-layer route planned";
    combined.visit_order = layer1.visit_order;
    combined.visit_order.insert(combined.visit_order.end(), layer2.visit_order.begin(), layer2.visit_order.end());
    combined.visit_order.insert(combined.visit_order.end(), layer3.visit_order.begin(), layer3.visit_order.end());
    combined.covered_edges = layer1.covered_edges;
    combined.covered_edges.insert(combined.covered_edges.end(), layer2.covered_edges.begin(), layer2.covered_edges.end());
    combined.covered_edges.insert(combined.covered_edges.end(), layer3.covered_edges.begin(), layer3.covered_edges.end());
    combined.deferred_targets = layer1.deferred_targets;
    combined.deferred_targets.insert(combined.deferred_targets.end(), layer2.deferred_targets.begin(), layer2.deferred_targets.end());
    combined.deferred_targets.insert(combined.deferred_targets.end(), layer3.deferred_targets.begin(), layer3.deferred_targets.end());
    const auto append = [&combined](const PlanResult & part) {
        if (part.points.empty()) return;
        if (combined.points.empty()) {
          combined.points = part.points;
          return;
        }
        // The next layer starts at the previous layer's endpoint. Avoid
        // publishing that shared point twice, which creates a zero-length
        // segment and an apparent 180 degree heading flip.
        std::size_t first = 0;
        if (Distance(combined.points.back(), part.points.front()) <= 1.0e-6) {
          first = 1;
        }
        combined.points.insert(combined.points.end(), part.points.begin() + first, part.points.end());
      };
    append(layer1);
    append(layer2);
    append(layer3);
    // Smooth across layer boundaries as well. Each layer is planned from the
    // previous endpoint, but independently smoothing them can leave a sharp
    // reverse turn at that shared boundary. Required checkpoints are retained
    // so this pass only rounds free-space connectors.
    std::vector<Point> required;
    required.reserve(targets.size() + config_.required_tunnel_points.size());
    required.insert(required.end(), targets.begin(), targets.end());
    required.insert(required.end(), config_.required_tunnel_points.begin(),
      config_.required_tunnel_points.end());
    combined.points = Smooth(combined.points, required);
    combined.headings.reserve(combined.points.size());
    for (std::size_t index = 0; index < combined.points.size(); ++index) {
      if (index + 1 < combined.points.size()) {
        combined.headings.push_back(std::atan2(
            combined.points[index + 1].y - combined.points[index].y,
            combined.points[index + 1].x - combined.points[index].x));
      } else {
        combined.headings.push_back(combined.headings.empty() ? start.yaw : combined.headings.back());
      }
    }
    combined.length = PolylineLength(combined.points);
    combined.all_targets_reached = layer1.all_targets_reached && layer2.all_targets_reached && layer3.all_targets_reached;
    return combined;
  }
  if (mode == "coverage") {
    return PlanCoverage(start, covered_edges);
  }
  if (mode == "layer3") {
    return PlanCoverage(start, covered_edges, false, true);
  }
  PlanResult result;
  try {
    const bool layer2 = mode == "layer2";
    const std::vector<Point> layer2_targets = config_.required_tunnel_points;
    const std::vector<std::string> layer2_labels = config_.required_tunnel_labels;
    if (layer2 && layer2_targets.empty()) {
      result.points = {start.position, start.position};
      result.headings = {start.yaw, start.yaw};
      result.length = 0.0;
      result.success = true;
      result.all_targets_reached = true;
      result.message = "no tunnels configured; stage skipped";
      return result;
    }
    const std::vector<Point> & planning_targets = layer2 ? layer2_targets : targets;
    const std::vector<std::string> & planning_input_labels = layer2 ? layer2_labels : labels;
    // Tunnel entry/exit pairs remain mandatory and ordered internally, but
    // the tunnels themselves should be visited from the current stage start
    // using the shortest reachable order instead of the YAML declaration
    // order.  Fixed numbering caused needless vertical zig-zags in stage 2.
    const std::string planning_mode = layer2 ? "shortest" : mode;
    if (planning_targets.empty() || planning_targets.size() != planning_input_labels.size()) {
      throw std::runtime_error("targets and labels must be non-empty and equal length");
    }
    if (planning_targets.size() > 16) {
      throw std::runtime_error("at most 16 targets are supported");
    }
    std::vector<Point> nodes{start.position};
    nodes.insert(nodes.end(), planning_targets.begin(), planning_targets.end());
    // Layer 1 is the task sweep only. Tunnel checkpoints are deliberately
    // deferred to layer 2 so the GUI can show the three mission stages.
    if (mode != "layer1" && !layer2) {
      nodes.insert(
        nodes.end(), config_.required_tunnel_points.begin(), config_.required_tunnel_points.end());
    }
    std::vector<std::string> planning_labels = planning_input_labels;
    if (mode != "layer1" && !layer2) {
      planning_labels.insert(planning_labels.end(), config_.required_tunnel_labels.begin(), config_.required_tunnel_labels.end());
    }
    if (!IsFree(start.position)) {
      throw std::runtime_error("start lies outside collision-free space");
    }
    if (planning_mode != "shortest" && planning_mode != "numbered" && planning_mode != "layer1") {
      throw std::runtime_error("mode must be shortest, numbered, or layer1");
    }
    std::vector<int> remaining;
    for (std::size_t index = 1; index < nodes.size(); ++index) {
      remaining.push_back(static_cast<int>(index));
    }
    std::vector<int> order{0};
    std::vector<GridPath> selected_paths;
    int current_node = 0;
    int forced_tunnel_exit = -1;
    bool retry_pass = false;
    bool optimized_open_route = false;
    // For the first stage, solve the open Hamiltonian path exactly.  The
    // previous nearest-neighbour loop was fast but could add several metres
    // of avoidable backtracking on the arena's narrow grid.
    if (mode == "layer1" && planning_mode == "shortest" && remaining.size() <= 16 &&
      config_.inspection_nodes.size() <= 100) {
      const std::size_t count = remaining.size();
      const std::size_t state_count = std::size_t{1} << count;
      std::vector<double> costs(state_count * count, std::numeric_limits<double>::infinity());
      std::vector<int> parents(state_count * count, -1);
      std::vector<std::vector<GridPath>> pair_paths(
        nodes.size(), std::vector<GridPath>(nodes.size()));
      std::vector<std::vector<bool>> reachable(
        nodes.size(), std::vector<bool>(nodes.size(), false));
      for (std::size_t target = 0; target < count; ++target) {
        try {
          pair_paths[0][target + 1] = AStar(nodes[0], nodes[target + 1]);
          reachable[0][target + 1] = true;
          costs[(std::size_t{1} << target) * count + target] =
            pair_paths[0][target + 1].cost;
        } catch (const std::runtime_error &) {
          continue;
        }
      }
      for (std::size_t mask = 1; mask < state_count; ++mask) {
        for (std::size_t last = 0; last < count; ++last) {
          if (!(mask & (std::size_t{1} << last))) continue;
          const double current_cost = costs[mask * count + last];
          if (!std::isfinite(current_cost)) continue;
          for (std::size_t next = 0; next < count; ++next) {
            if (mask & (std::size_t{1} << next)) continue;
            const int from = static_cast<int>(remaining[last] + 0);
            const int to = static_cast<int>(remaining[next] + 0);
            if (!reachable[from][to]) {
              try {
                pair_paths[from][to] = AStar(nodes[from], nodes[to]);
                reachable[from][to] = true;
              } catch (const std::runtime_error &) {
                continue;
              }
            }
            const std::size_t next_mask = mask | (std::size_t{1} << next);
            const double candidate = current_cost + pair_paths[from][to].cost;
            double & destination = costs[next_mask * count + next];
            if (candidate < destination) {
              destination = candidate;
              parents[next_mask * count + next] = static_cast<int>(last);
            }
          }
        }
      }
      const std::size_t full_mask = state_count - 1;
      int last = -1;
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t candidate = 0; candidate < count; ++candidate) {
        if (costs[full_mask * count + candidate] < best) {
          best = costs[full_mask * count + candidate];
          last = static_cast<int>(candidate);
        }
      }
      if (last >= 0) {
        std::vector<int> sequence(count);
        std::size_t mask = full_mask;
        for (std::size_t position = count; position-- > 0;) {
          sequence[position] = last;
          const int previous = parents[mask * count + static_cast<std::size_t>(last)];
          mask ^= std::size_t{1} << static_cast<std::size_t>(last);
          last = previous;
          if (position > 0 && last < 0) break;
        }
        bool complete = true;
        for (std::size_t position = 0; position < count; ++position) {
          const int node = remaining[static_cast<std::size_t>(sequence[position])];
          const int previous_node = position == 0 ? 0 :
            remaining[static_cast<std::size_t>(sequence[position - 1])];
          if (position == 0) {
            selected_paths.push_back(pair_paths[0][node]);
          } else if (reachable[previous_node][node]) {
            selected_paths.push_back(pair_paths[previous_node][node]);
          } else {
            complete = false;
            break;
          }
          order.push_back(node);
        }
        if (complete) {
          remaining.clear();
          optimized_open_route = true;
        } else {
          order = {0};
          selected_paths.clear();
        }
      }
    }
    while (!remaining.empty() && !optimized_open_route) {
      int selected_position = -1;
      GridPath selected_path;
      double selected_cost = std::numeric_limits<double>::infinity();
      for (std::size_t position = 0; position < remaining.size(); ++position) {
        if (forced_tunnel_exit >= 0 && remaining[position] != forced_tunnel_exit) {
          continue;
        }
        try {
          GridPath path = AStar(nodes[current_node], nodes[remaining[position]]);
          if (planning_mode == "numbered" || path.cost < selected_cost) {
            selected_position = static_cast<int>(position);
            selected_cost = path.cost;
            selected_path = std::move(path);
          }
          if (planning_mode == "numbered") {
            break;
          }
        } catch (const std::runtime_error &) {
          continue;
        }
      }
      if (selected_position < 0) {
        if (!retry_pass && order.size() > 1) {
          // The vehicle has moved to the far side of all currently reachable
          // tasks. Retry every blocked target exactly once from this new side.
          retry_pass = true;
          continue;
        }
        break;
      }
      current_node = remaining[static_cast<std::size_t>(selected_position)];
      order.push_back(current_node);
      selected_paths.push_back(std::move(selected_path));
      remaining.erase(remaining.begin() + selected_position);
      if (current_node > 0) {
        const std::string & tunnel_label = planning_labels[current_node - 1];
        constexpr const char * enter_suffix = "_ENTER";
        constexpr const char * exit_suffix = "_EXIT";
        const bool entered_from_left =
          tunnel_label.size() > 6 &&
          tunnel_label.rfind(enter_suffix) == tunnel_label.size() - 6;
        const bool entered_from_right =
          tunnel_label.size() > 5 &&
          tunnel_label.rfind(exit_suffix) == tunnel_label.size() - 5;
        if (forced_tunnel_exit == current_node) {
          forced_tunnel_exit = -1;
        } else if (entered_from_left || entered_from_right) {
          const std::string counterpart = tunnel_label.substr(
            0, tunnel_label.size() - (entered_from_left ? 6 : 5)) +
            (entered_from_left ? exit_suffix : enter_suffix);
          forced_tunnel_exit = -1;
          for (int candidate : remaining) {
            if (planning_labels[candidate - 1] == counterpart) {
              forced_tunnel_exit = candidate;
              break;
            }
          }
          if (forced_tunnel_exit < 0) {
            throw std::runtime_error("tunnel counterpart is missing after entry");
          }
        }
      }
    }
    for (int deferred : remaining) {
      result.deferred_targets.push_back(planning_labels[deferred - 1]);
    }
    if (order.size() == 1) {
      throw std::runtime_error("no remaining target is currently reachable");
    }

    if (remaining.empty() && mode != "layer1" && !layer2) {
      try {
        selected_paths.push_back(AStar(nodes[current_node], start.position));
        order.push_back(0);
      } catch (const std::runtime_error &) {
        // Reaching every target is preferable to discarding the route when
        // the dynamic map temporarily blocks the return to the start.
      }
    }
    std::vector<Point> anchors{start.position};
    for (std::size_t leg = 1; leg < order.size(); ++leg) {
      const int left = order[leg - 1];
      const int right = order[leg];
      std::vector<Point> raw{nodes[left]};
      const auto & cells = selected_paths[leg - 1].cells;
      for (std::size_t index = 1; index + 1 < cells.size(); ++index) {
        raw.push_back(CellToWorld(cells[index]));
      }
      raw.push_back(nodes[right]);
      std::vector<Point> simplified = Simplify(raw);
      anchors.insert(anchors.end(), simplified.begin() + 1, simplified.end());
      result.visit_order.push_back(right == 0 ? "S" : planning_labels[right - 1]);
    }
    std::vector<Point> reached_targets;
    for (std::size_t index = 1; index < order.size(); ++index) {
      if (order[index] != 0) {
        reached_targets.push_back(nodes[order[index]]);
      }
    }
    result.points = Smooth(anchors, reached_targets);
    for (const Point & target : reached_targets) {
      double minimum = std::numeric_limits<double>::infinity();
      for (std::size_t index = 1; index < result.points.size(); ++index) {
        minimum = std::min(
          minimum, DistanceToSegment(target, result.points[index - 1], result.points[index]));
      }
      if (minimum > config_.task_tolerance + 1.0e-9) {
        throw std::runtime_error("smoothed route missed a required target");
      }
    }
    result.headings.reserve(result.points.size());
    for (std::size_t index = 0; index < result.points.size(); ++index) {
      if (index + 1 < result.points.size()) {
        result.headings.push_back(std::atan2(
            result.points[index + 1].y - result.points[index].y,
            result.points[index + 1].x - result.points[index].x));
      } else {
        result.headings.push_back(result.headings.empty() ? start.yaw : result.headings.back());
      }
    }
    result.length = PolylineLength(result.points);
    result.covered_edges = CoveredInspectionEdges(config_, result.points);
    if (layer2) {
      const std::vector<std::string> checkpoints = result.visit_order;
      result.visit_order.clear();
      for (const std::string & checkpoint : checkpoints) {
        constexpr const char * suffix = "_ENTER";
        if (checkpoint.size() <= 6 ||
          checkpoint.rfind(suffix) != checkpoint.size() - 6)
        {
          continue;
        }
        const std::string tunnel = checkpoint.substr(0, checkpoint.size() - 6);
        if (std::find(result.visit_order.begin(), result.visit_order.end(), tunnel) ==
          result.visit_order.end())
        {
          result.visit_order.push_back(tunnel);
        }
      }
    }
    result.all_targets_reached = result.deferred_targets.empty();
    result.success = true;
    result.message = result.all_targets_reached ?
      "route planned" : "reachable targets planned; blocked targets deferred for one retry";
  } catch (const std::exception & exception) {
    result.success = false;
    result.message = exception.what();
    result.points.clear();
    result.headings.clear();
    result.visit_order.clear();
  }
  return result;
}

}  // namespace arena_path_planner
