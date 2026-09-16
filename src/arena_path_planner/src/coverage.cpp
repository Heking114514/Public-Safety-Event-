#include "arena_path_planner/planner_internal.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace arena_path_planner {
std::vector<std::string>
CoveredInspectionEdges(const PlannerConfig &config,
                       const std::vector<Point> &route) {
  return FullyCoveredInspectionEdges(
    config, CoveredInspectionIntervals(config, route));
}

PlanResult
ArenaPlanner::PlanCoverage(const Pose &start,
                           const std::vector<std::string> &covered_edges,
                           const std::vector<RoadInterval> &covered_intervals,
                           bool tunnels_only, bool non_tunnels_only) const {
  PlanResult result;
  try {
    if (config_.inspection_nodes.empty() || config_.inspection_edges.empty()) {
      throw std::runtime_error("coverage mode requires an inspection graph");
    }
    if (!IsFree(start.position)) {
      throw std::runtime_error("start lies outside collision-free space");
    }
    const std::size_t node_count = config_.inspection_nodes.size();
    const double turn_priority =
      std::max(0.5, 4.0 * config_.minimum_turning_radius);
    const auto path_score = [turn_priority](const GridPath & path) {
        return turn_priority * static_cast<double>(path.turn_count) + path.cost;
      };
    const std::vector<RoadInterval> historical_coverage =
      NormalizeRoadIntervals(config_, covered_edges, covered_intervals);
    const std::vector<std::string> historically_complete =
      FullyCoveredInspectionEdges(config_, historical_coverage);
    std::vector<std::string> known_edges;
    known_edges.reserve(config_.inspection_edges.size());
    for (const InspectionEdge &edge : config_.inspection_edges) {
      known_edges.push_back(edge.label);
    }
    for (const std::string &label : covered_edges) {
      if (std::find(known_edges.begin(), known_edges.end(), label) ==
          known_edges.end()) {
        throw std::runtime_error("covered_edges contains unknown label: " +
                                 label);
      }
    }
    std::vector<std::vector<std::pair<int, int>>> adjacency(node_count);
    std::vector<bool> usable(config_.inspection_edges.size(), false);
    const auto already_covered = [&historically_complete](const std::string &label) {
      return std::find(
        historically_complete.begin(), historically_complete.end(), label) !=
             historically_complete.end();
    };
    for (std::size_t index = 0; index < config_.inspection_edges.size();
         ++index) {
      const InspectionEdge &edge = config_.inspection_edges[index];
      if (edge.from < 0 || edge.to < 0 ||
          static_cast<std::size_t>(edge.from) >= node_count ||
          static_cast<std::size_t>(edge.to) >= node_count) {
        throw std::runtime_error("inspection edge references an invalid node");
      }
      usable[index] = SegmentIsFree(config_.inspection_nodes[edge.from],
                                    config_.inspection_nodes[edge.to]);
      if (!usable[index]) {
        // Exported graph edges are desired inspection strokes, not a command
        // to drive a rectangular chassis through a wall. If its centre line is
        // too close to an obstacle but the two road nodes remain connected,
        // cover that stroke through a collision-free A* connector instead.
        try {
          (void)AStar(config_.inspection_nodes[edge.from],
                      config_.inspection_nodes[edge.to]);
          usable[index] = true;
        } catch (const std::runtime_error &) {
          usable[index] = false;
        }
      }
      if (usable[index]) {
        adjacency[edge.from].emplace_back(edge.to, static_cast<int>(index));
        adjacency[edge.to].emplace_back(edge.from, static_cast<int>(index));
      } else if (((!tunnels_only && !non_tunnels_only) ||
                  (tunnels_only && edge.tunnel) ||
                  (non_tunnels_only && !edge.tunnel)) &&
                 !already_covered(edge.label)) {
        result.deferred_targets.push_back(edge.label);
      }
    }
    const auto defer_once = [&result](const std::string &label) {
        if (std::find(result.deferred_targets.begin(),
                      result.deferred_targets.end(), label) ==
            result.deferred_targets.end()) {
          result.deferred_targets.push_back(label);
        }
      };
    const auto finish_route = [this, &result, &historical_coverage,
                               &defer_once, tunnels_only, non_tunnels_only]() {
        for (const Point &point : result.points) {
          if (!IsFinite(point)) {
            throw std::runtime_error("coverage route contains a non-finite point");
          }
        }
        for (std::size_t index = 1; index < result.points.size(); ++index) {
          if (!SegmentIsFree(result.points[index - 1], result.points[index])) {
            std::ostringstream message;
            message << "coverage route segment " << index - 1 << " -> " << index
                    << " is not collision-free: (" << result.points[index - 1].x
                    << ", " << result.points[index - 1].y << ") -> ("
                    << result.points[index].x << ", " << result.points[index].y << ")";
            throw std::runtime_error(message.str());
          }
        }
        if (!RouteTurnsAreAllowed(result.points)) {
          throw std::runtime_error(
                  "coverage route requires a U-turn or consecutive tight turns; "
                  "recover to a junction before replanning");
        }
        const std::vector<RoadInterval> current_intervals =
          CoveredInspectionIntervals(config_, result.points);
        const std::vector<RoadInterval> prospective_coverage = MergeRoadIntervals(
          config_, historical_coverage, current_intervals);
        result.covered_intervals = historical_coverage;
        result.planned_intervals = current_intervals;
        const std::vector<std::string> current_coverage =
          FullyCoveredInspectionEdges(config_, current_intervals);
        result.visit_order.erase(
          std::remove_if(
            result.visit_order.begin(), result.visit_order.end(),
            [&current_coverage](const std::string & label) {
              return std::find(
                current_coverage.begin(), current_coverage.end(), label) ==
                     current_coverage.end();
            }),
          result.visit_order.end());
        result.covered_edges = FullyCoveredInspectionEdges(
          config_, prospective_coverage);
        result.deferred_intervals = UncoveredRoadIntervals(
          config_, prospective_coverage, tunnels_only, non_tunnels_only);
        for (const RoadInterval & interval : result.deferred_intervals) {
          defer_once(interval.edge_label);
        }
        result.all_targets_reached =
          result.deferred_targets.empty() && result.deferred_intervals.empty();
      };
    const auto set_headings = [&result, &start]() {
        result.headings.clear();
        result.headings.reserve(result.points.size());
        for (std::size_t index = 0; index < result.points.size(); ++index) {
          result.headings.push_back(
              index + 1 < result.points.size()
                  ? std::atan2(result.points[index + 1].y - result.points[index].y,
                               result.points[index + 1].x - result.points[index].x)
                  : (result.headings.empty() ? start.yaw
                                             : result.headings.back()));
        }
      };
    bool ended_after_retreat = false;
    const auto apply_planned_retreat = [this, &result, &ended_after_retreat]() {
        if (!config_.turns_at_junctions_only) {
          return;
        }
        bool inserted = false;
        result.points = StopAfterFirstBlockedReversal(result.points, inserted);
        ended_after_retreat = ended_after_retreat || inserted;
      };

    if (non_tunnels_only) {
      const bool has_partial_history = std::any_of(
        historical_coverage.begin(), historical_coverage.end(),
        [](const RoadInterval & interval) {
          return interval.start_fraction > 1.0e-9 ||
                 interval.end_fraction < 1.0 - 1.0e-9;
        });
      const bool has_split_road = std::any_of(
        config_.inspection_edges.begin(), config_.inspection_edges.end(),
        [this, &already_covered](const InspectionEdge & edge) {
          return !edge.tunnel && !already_covered(edge.label) &&
                 !SegmentIsFree(
            config_.inspection_nodes[edge.from],
            config_.inspection_nodes[edge.to]);
        });
      if (has_partial_history || has_split_road) {
        return PlanRoadIntervals(start, historical_coverage);
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
      for (std::size_t index = 0; index < config_.inspection_edges.size();
           ++index) {
        const InspectionEdge &edge = config_.inspection_edges[index];
        if (usable[index] && !edge.tunnel && !already_covered(edge.label)) {
          remaining.push_back(index);
        }
      }

      result.points.push_back(start.position);
      Point current = start.position;
      std::vector<Point> required_coverage_points = config_.inspection_nodes;
      const auto append_grid_path_safely = [this, &result](
          const GridPath &path, const Point &exact_end) {
        std::vector<Point> grid_points;
        grid_points.reserve(path.cells.size());
        for (const Cell &cell : path.cells) {
          grid_points.push_back(CellToWorld(cell));
        }

        std::size_t first = 0;
        while (first < grid_points.size() &&
               !SegmentIsFree(result.points.back(), grid_points[first])) {
          ++first;
        }
        if (first == grid_points.size()) {
          throw std::runtime_error(
              "A* gap-fill path cannot safely leave the exact start point");
        }

        std::size_t last = grid_points.size() - 1;
        bool reaches_exact_end = false;
        for (std::size_t candidate = grid_points.size(); candidate-- > first;) {
          if (SegmentIsFree(grid_points[candidate], exact_end)) {
            last = candidate;
            reaches_exact_end = true;
            break;
          }
        }
        if (!reaches_exact_end &&
            Distance(grid_points.back(), exact_end) >
                config_.task_tolerance + 1.0e-9) {
          throw std::runtime_error(
              "A* gap-fill path cannot reach the end-point tolerance");
        }
        if (!reaches_exact_end) {
          last = grid_points.size() - 1;
        }
        for (std::size_t index = first; index <= last; ++index) {
          if (Distance(result.points.back(), grid_points[index]) > 1.0e-9) {
            result.points.push_back(grid_points[index]);
          }
        }
        if (reaches_exact_end &&
            Distance(result.points.back(), exact_end) > 1.0e-9) {
          result.points.push_back(exact_end);
        }
      };
      const auto append_segment = [this, &result, &append_grid_path_safely](
          const Point &end) {
        const Point begin = result.points.back();
        if (!SegmentIsFree(begin, end)) {
          append_grid_path_safely(AStar(begin, end), end);
          return;
        }
        const double length = Distance(begin, end);
        const int samples = std::max(
            1, static_cast<int>(std::ceil(length / config_.waypoint_spacing)));
        for (int sample = 1; sample <= samples; ++sample) {
          const double ratio = static_cast<double>(sample) / samples;
          result.points.push_back({begin.x + ratio * (end.x - begin.x),
                                   begin.y + ratio * (end.y - begin.y)});
        }
      };

      // Exact shortest open tour over the remaining roads.  There are only a
      // handful of gap edges, so subset DP is cheap and avoids greedy zig-zag
      // choices.  Each edge has two legal directions; the edge itself is
      // traversed once and all inter-edge legs use collision-free A* paths.
      struct RoadState {
        double cost{std::numeric_limits<double>::infinity()};
        int parent{-1};
      };
      const std::size_t road_count = remaining.size();
      if (road_count == 0) {
        // An empty remaining list means either all selected roads were already
        // covered or every still-uncovered road is currently blocked. Only the
        // first case is completion; treating the second as a successful empty
        // stage makes an automatic frontend replan forever without progress.
        if (!result.deferred_targets.empty()) {
          throw std::runtime_error(
                  "no uncovered road is reachable from the current start");
        }
        if (Distance(start.position, config_.default_start.position) > 1.0e-6) {
          append_segment(config_.default_start.position);
          result.points = Smooth(result.points, {});
          apply_planned_retreat();
          set_headings();
          result.length = PolylineLength(result.points);
          finish_route();
          result.success = true;
          result.message = ended_after_retreat
                               ? "planned retreat before a required U-turn; replan after reversing"
                               : (result.all_targets_reached
                               ? "coverage already complete; return route planned"
                               : "coverage input is incomplete");
          return result;
        }
        // There is genuinely no command to send.  Report completion without
        // manufacturing a one-point route that navigation interprets as a new
        // mission reaching its goal immediately.
        result.points.clear();
        result.headings.clear();
        finish_route();
        result.success = true;
        result.message = result.all_targets_reached
                             ? "coverage already complete; no route required"
                             : "coverage input is incomplete";
        return result;
      }
      // Exact subset DP is intentionally limited to small graphs.  Exported
      // 15 cm maps can contain hundreds of road cells; use a bounded greedy
      // pass there to avoid exponential memory growth.
      if (road_count > 16) {
        while (!remaining.empty()) {
          std::size_t best = 0;
          bool best_forward = true;
          double best_score = std::numeric_limits<double>::infinity();
          for (std::size_t position = 0; position < remaining.size();
               ++position) {
            const InspectionEdge &road =
                config_.inspection_edges[remaining[position]];
            for (bool forward : {true, false}) {
              const int first = forward ? road.from : road.to;
              const int second = forward ? road.to : road.from;
              try {
                // For large exported grids, use a geometric lower bound while
                // ranking candidates. Running A* for every remaining edge at
                // every iteration made planning O(E^2 * A*) and visibly froze
                // the GUI. A* is still run for the selected edge below.
                const double score =
                    Distance(current,
                             config_.inspection_nodes[static_cast<std::size_t>(
                                 first)]) +
                    Distance(
                        config_
                            .inspection_nodes[static_cast<std::size_t>(first)],
                        config_.inspection_nodes[static_cast<std::size_t>(
                            second)]);
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
              result.deferred_targets.push_back(
                  config_.inspection_edges[index].label);
            }
            break;
          }
          const InspectionEdge &road =
              config_.inspection_edges[remaining[best]];
          const int first = best_forward ? road.from : road.to;
          const int second = best_forward ? road.to : road.from;
          try {
            append_grid_path_safely(
                AStar(current,
                      config_.inspection_nodes[static_cast<std::size_t>(first)]),
                config_.inspection_nodes[static_cast<std::size_t>(first)]);
            append_segment(
                config_.inspection_nodes[static_cast<std::size_t>(second)]);
            result.visit_order.push_back(road.label);
            required_coverage_points.push_back(
                config_.inspection_nodes[static_cast<std::size_t>(first)]);
            required_coverage_points.push_back(
                config_.inspection_nodes[static_cast<std::size_t>(second)]);
            current = result.points.back();
          } catch (const std::runtime_error &) {
            result.deferred_targets.push_back(road.label);
          }
          remaining.erase(remaining.begin() +
                          static_cast<std::ptrdiff_t>(best));
        }
        try {
          append_grid_path_safely(
              AStar(current, config_.default_start.position),
              config_.default_start.position);
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
                const GridPath connector =
                    AStar(safe_points.back(), result.points[index]);
                for (std::size_t cell = 1; cell < connector.cells.size();
                     ++cell) {
                  safe_points.push_back(CellToWorld(connector.cells[cell]));
                }
                const Point target = result.points[index];
                if (Distance(safe_points.back(), target) > 1.0e-9) {
                  if (!SegmentIsFree(safe_points.back(), target)) {
                    throw std::runtime_error(
                            "A* repair cannot safely reach the exact route point");
                  }
                  safe_points.push_back(target);
                }
              } catch (const std::runtime_error &) {
                throw std::runtime_error(
                        "failed to repair an unsafe coverage route segment");
              }
            }
          }
          result.points = std::move(safe_points);
        }
        result.points = Simplify(result.points, required_coverage_points);
        result.points = Smooth(result.points, required_coverage_points);
        apply_planned_retreat();
        set_headings();
        result.length = PolylineLength(result.points);
        finish_route();
        if (result.visit_order.empty() && result.length <= 1.0e-6 &&
          !result.all_targets_reached)
        {
          throw std::runtime_error(
                  "no uncovered road is reachable from the current start");
        }
        result.success = true;
        result.message = ended_after_retreat
                             ? "planned retreat before a required U-turn; replan after reversing"
                             : (result.all_targets_reached
                             ? "large road graph filled"
                             : "large road graph partially filled");
        return result;
      }
      const std::size_t state_count = std::size_t{1} << road_count;
      const auto state_index = [road_count](std::size_t mask, std::size_t edge,
                                            std::size_t direction) {
        return (mask * road_count + edge) * 2 + direction;
      };
      std::vector<RoadState> states(state_count * road_count * 2);
      std::vector<std::vector<GridPath>> paths(
          node_count, std::vector<GridPath>(node_count));
      std::vector<std::vector<bool>> path_attempted(
          node_count, std::vector<bool>(node_count, false));
      std::vector<std::vector<bool>> path_reachable(
          node_count, std::vector<bool>(node_count, false));
      const auto get_path =
          [this, &paths, &path_attempted,
           &path_reachable](int from, int to) -> const GridPath * {
        if (!path_attempted[from][to]) {
          path_attempted[from][to] = true;
          try {
            paths[from][to] = AStar(
                config_.inspection_nodes[static_cast<std::size_t>(from)],
                config_.inspection_nodes[static_cast<std::size_t>(to)]);
            path_reachable[from][to] = true;
          } catch (const std::runtime_error &) {
            path_reachable[from][to] = false;
          }
        }
        return path_reachable[from][to] ? &paths[from][to] : nullptr;
      };
      // Start legs use the actual stage start rather than a graph node.
      for (std::size_t edge = 0; edge < road_count; ++edge) {
        const InspectionEdge &road = config_.inspection_edges[remaining[edge]];
        for (std::size_t direction = 0; direction < 2; ++direction) {
          const int first = direction == 0 ? road.from : road.to;
          const int second = direction == 0 ? road.to : road.from;
          try {
            const double cost =
                path_score(AStar(
                  current,
                  config_.inspection_nodes[static_cast<std::size_t>(first)])) +
                Distance(
                    config_.inspection_nodes[static_cast<std::size_t>(first)],
                    config_.inspection_nodes[static_cast<std::size_t>(second)]);
            states[state_index(std::size_t{1} << edge, edge, direction)] = {
                cost, -1};
          } catch (const std::runtime_error &) {
            continue;
          }
        }
      }
      for (std::size_t mask = 1; mask < state_count; ++mask) {
        for (std::size_t last = 0; last < road_count; ++last) {
          for (std::size_t direction = 0; direction < 2; ++direction) {
            const RoadState current_state =
                states[state_index(mask, last, direction)];
            if (!std::isfinite(current_state.cost))
              continue;
            const InspectionEdge &road =
                config_.inspection_edges[remaining[last]];
            const int last_end = direction == 0 ? road.to : road.from;
            for (std::size_t next = 0; next < road_count; ++next) {
              if (mask & (std::size_t{1} << next))
                continue;
              const InspectionEdge &next_road =
                  config_.inspection_edges[remaining[next]];
              for (std::size_t next_direction = 0; next_direction < 2;
                   ++next_direction) {
                const int next_first =
                    next_direction == 0 ? next_road.from : next_road.to;
                const int next_second =
                    next_direction == 0 ? next_road.to : next_road.from;
                const GridPath *connector = get_path(last_end, next_first);
                if (connector == nullptr)
                  continue;
                const std::size_t next_mask = mask | (std::size_t{1} << next);
                const double candidate =
                    current_state.cost + path_score(*connector) +
                    Distance(config_.inspection_nodes[static_cast<std::size_t>(
                                 next_first)],
                             config_.inspection_nodes[static_cast<std::size_t>(
                                 next_second)]);
                RoadState &destination =
                    states[state_index(next_mask, next, next_direction)];
                if (candidate < destination.cost) {
                  destination = {candidate, static_cast<int>(state_index(
                                                mask, last, direction))};
                }
              }
            }
          }
        }
      }
      std::vector<std::vector<double>> return_costs(
          road_count,
          std::vector<double>(2, std::numeric_limits<double>::infinity()));
      for (std::size_t edge = 0; edge < road_count; ++edge) {
        const InspectionEdge &road = config_.inspection_edges[remaining[edge]];
        for (std::size_t direction = 0; direction < 2; ++direction) {
          const int last_end = direction == 0 ? road.to : road.from;
          try {
            return_costs[edge][direction] =
                path_score(AStar(
                  config_.inspection_nodes[static_cast<std::size_t>(last_end)],
                  config_.default_start.position));
          } catch (const std::runtime_error &) {
            // An open partial route is still useful if this entire connected
            // component cannot return home. It is selected only when no road
            // subset has a valid return and is marked accordingly below.
          }
        }
      }
      const auto covered_count = [](std::size_t mask) {
          std::size_t count = 0;
          while (mask != 0) {
            count += mask & std::size_t{1};
            mask >>= 1;
          }
          return count;
        };
      int best_return_state = -1;
      std::size_t best_return_mask = 0;
      std::size_t best_return_count = 0;
      double best_return_cost = std::numeric_limits<double>::infinity();
      int best_open_state = -1;
      std::size_t best_open_mask = 0;
      std::size_t best_open_count = 0;
      double best_open_cost = std::numeric_limits<double>::infinity();
      for (std::size_t mask = 1; mask < state_count; ++mask) {
        const std::size_t count = covered_count(mask);
        for (std::size_t edge = 0; edge < road_count; ++edge) {
          if (!(mask & (std::size_t{1} << edge))) {
            continue;
          }
          for (std::size_t direction = 0; direction < 2; ++direction) {
            const std::size_t candidate = state_index(mask, edge, direction);
            const double route_cost = states[candidate].cost;
            if (!std::isfinite(route_cost)) {
              continue;
            }
            if (best_open_state < 0 || count > best_open_count ||
              (count == best_open_count && route_cost < best_open_cost))
            {
              best_open_state = static_cast<int>(candidate);
              best_open_mask = mask;
              best_open_count = count;
              best_open_cost = route_cost;
            }
            if (!std::isfinite(return_costs[edge][direction])) {
              continue;
            }
            const double total_cost = route_cost + return_costs[edge][direction];
            if (best_return_state < 0 || count > best_return_count ||
              (count == best_return_count && total_cost < best_return_cost))
            {
              best_return_state = static_cast<int>(candidate);
              best_return_mask = mask;
              best_return_count = count;
              best_return_cost = total_cost;
            }
          }
        }
      }
      const bool return_path_available = best_return_state >= 0;
      const int best_state = return_path_available ? best_return_state : best_open_state;
      const std::size_t best_mask =
          return_path_available ? best_return_mask : best_open_mask;
      if (best_state < 0) {
        throw std::runtime_error(
                "no uncovered road is reachable from the current start");
      }
      for (std::size_t edge = 0; edge < road_count; ++edge) {
        if (!(best_mask & (std::size_t{1} << edge))) {
          defer_once(config_.inspection_edges[remaining[edge]].label);
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
        for (const auto &[edge_position, direction] : sequence) {
          const InspectionEdge &road =
              config_.inspection_edges[remaining[edge_position]];
          const int first = direction == 0 ? road.from : road.to;
          const int second = direction == 0 ? road.to : road.from;
          GridPath approach;
          try {
            approach = AStar(
                current,
                config_.inspection_nodes[static_cast<std::size_t>(first)]);
          } catch (const std::runtime_error &) {
            throw std::runtime_error(
                    "coverage tour became unreachable during reconstruction");
          }
          append_grid_path_safely(
              approach,
              config_.inspection_nodes[static_cast<std::size_t>(first)]);
          append_segment(
              config_.inspection_nodes[static_cast<std::size_t>(second)]);
          result.visit_order.push_back(road.label);
          required_coverage_points.push_back(
              config_.inspection_nodes[static_cast<std::size_t>(first)]);
          required_coverage_points.push_back(
              config_.inspection_nodes[static_cast<std::size_t>(second)]);
          current = result.points.back();
        }
        if (return_path_available) {
          try {
            append_grid_path_safely(
                AStar(current, config_.default_start.position),
                config_.default_start.position);
          } catch (const std::runtime_error &) {
            defer_once("RETURN_TO_START");
          }
        } else {
          defer_once("RETURN_TO_START");
        }
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
              if (!SegmentIsFree(safe_points.back(), target)) {
                throw std::runtime_error(
                        "A* repair cannot safely reach the exact route point");
              }
              safe_points.push_back(target);
            }
          } catch (const std::runtime_error &) {
            throw std::runtime_error(
                    "failed to repair an unsafe coverage route segment");
          }
        }
        result.points = Smooth(safe_points, required_coverage_points);
        apply_planned_retreat();
      }

      set_headings();
      result.length = PolylineLength(result.points);
      finish_route();
      result.success = true;
      result.message =
          ended_after_retreat
              ? "planned retreat before a required U-turn; replan after reversing"
              : (result.all_targets_reached
              ? "uncovered roads filled"
              : "reachable uncovered roads filled; blocked roads deferred");
      return result;
    }

    int entry_node = 0;
    GridPath entry_path;
    double entry_cost = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < node_count; ++index) {
      try {
        GridPath candidate =
            AStar(start.position, config_.inspection_nodes[index]);
        if (path_score(candidate) < entry_cost) {
          entry_cost = path_score(candidate);
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
      for (const auto &[next, edge_index] : adjacency[node]) {
        if (traversed[edge_index]) {
          continue;
        }
        const InspectionEdge &edge = config_.inspection_edges[edge_index];
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
      const InspectionEdge &edge = config_.inspection_edges[index];
      const bool selected = (!tunnels_only && !non_tunnels_only) ||
                            (tunnels_only && edge.tunnel) ||
                            (non_tunnels_only && !edge.tunnel);
      if (selected && usable[index] && !traversed[index] &&
          !already_covered(config_.inspection_edges[index].label)) {
        result.deferred_targets.push_back(
            config_.inspection_edges[index].label);
      }
    }

    result.points.push_back(start.position);
    const auto append_grid_path_safely = [this, &result](
        const GridPath & path, const Point & exact_end) {
        std::vector<Point> grid_points;
        grid_points.reserve(path.cells.size());
        for (const Cell & cell : path.cells) {
          grid_points.push_back(CellToWorld(cell));
        }

        std::size_t first = 0;
        while (first < grid_points.size() &&
          !SegmentIsFree(result.points.back(), grid_points[first]))
        {
          ++first;
        }
        if (first == grid_points.size()) {
          throw std::runtime_error(
                  "A* coverage path cannot safely leave the exact start point");
        }

        std::size_t last = grid_points.size() - 1;
        bool reaches_exact_end = false;
        for (std::size_t candidate = grid_points.size(); candidate-- > first;) {
          if (SegmentIsFree(grid_points[candidate], exact_end)) {
            last = candidate;
            reaches_exact_end = true;
            break;
          }
        }
        if (!reaches_exact_end) {
          throw std::runtime_error(
                  "A* coverage path cannot safely reach the exact end point");
        }
        for (std::size_t index = first; index <= last; ++index) {
          if (Distance(result.points.back(), grid_points[index]) > 1.0e-9) {
            result.points.push_back(grid_points[index]);
          }
        }
        if (Distance(result.points.back(), exact_end) > 1.0e-9) {
          result.points.push_back(exact_end);
        }
      };
    append_grid_path_safely(entry_path, config_.inspection_nodes[entry_node]);
    const auto append_segment = [this, &result, &append_grid_path_safely](
        const Point &end) {
      const Point begin = result.points.back();
      if (SegmentIsFree(begin, end)) {
        const double length = Distance(begin, end);
        const int samples = std::max(
            1, static_cast<int>(std::ceil(length / config_.waypoint_spacing)));
        for (int sample = 1; sample <= samples; ++sample) {
          const double ratio = static_cast<double>(sample) / samples;
          result.points.push_back({begin.x + ratio * (end.x - begin.x),
                                   begin.y + ratio * (end.y - begin.y)});
        }
        return;
      }

      // An inspection edge may be connected through a nearby free cell even
      // when its exported centre line clips an inflated obstacle. Reuse the
      // same collision-free A* connector used by task planning instead of
      // emitting an unsafe straight chord.
      append_grid_path_safely(AStar(begin, end), end);
    };
    for (std::size_t index = 1; index < walk.size(); ++index) {
      append_segment(config_.inspection_nodes[walk[index]]);
    }
    if (!tunnels_only &&
      Distance(result.points.back(), config_.default_start.position) > 1.0e-6)
    {
      try {
        append_segment(config_.default_start.position);
      } catch (const std::runtime_error &) {
        defer_once("RETURN_TO_START");
      }
    }
    apply_planned_retreat();
    set_headings();
    result.length = PolylineLength(result.points);
    finish_route();
    result.success = true;
    result.message = ended_after_retreat
                         ? "planned retreat before a required U-turn; replan after reversing"
                         : (result.all_targets_reached
                         ? "all inspection roads covered"
                         : "reachable roads covered; blocked roads deferred");
  } catch (const std::exception &exception) {
    result.message = exception.what();
    result.success = false;
    result.all_targets_reached = false;
    result.points.clear();
    result.headings.clear();
    result.visit_order.clear();
    result.covered_edges.clear();
    result.covered_intervals.clear();
    result.planned_intervals.clear();
    result.blocked_intervals.clear();
    result.deferred_intervals.clear();
    result.length = 0.0;
  }
  return result;
}
} // namespace arena_path_planner
