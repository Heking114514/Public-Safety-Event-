#include "arena_path_planner/planner_internal.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace arena_path_planner
{
PlanResult ArenaPlanner::Plan(
  const Pose & start, const std::vector<Point> & targets,
  const std::vector<std::string> & labels, const std::string & mode,
  const std::vector<std::string> & covered_edges) const
{
  const auto failed = [](const std::string & message) {
      PlanResult result;
      result.message = message;
      return result;
    };
  if (!IsFinite(start)) {
    return failed("start pose must contain finite coordinates and yaw");
  }
  for (const Point & target : targets) {
    if (!IsFinite(target)) {
      return failed("target coordinates must be finite");
    }
  }
  for (const std::string & covered : covered_edges) {
    const bool known = std::any_of(
      config_.inspection_edges.begin(), config_.inspection_edges.end(),
      [&covered](const InspectionEdge & edge) {return edge.label == covered;});
    if (!known) {
      return failed("covered_edges contains unknown label: " + covered);
    }
  }
  const auto append_unique = [](std::vector<std::string> & destination,
      const std::vector<std::string> & source) {
      for (const std::string & value : source) {
        if (std::find(destination.begin(), destination.end(), value) ==
          destination.end())
        {
          destination.push_back(value);
        }
      }
    };
  const auto route_is_free = [this](const std::vector<Point> & points) {
      for (std::size_t index = 1; index < points.size(); ++index) {
        if (!SegmentIsFree(points[index - 1], points[index])) {
          return false;
        }
      }
      return true;
  };
  if (mode == "layered") {
    PlanResult combined;
    std::vector<std::string> covered = covered_edges;
    Pose stage_start = start;
    bool all_stages_complete = true;
    std::vector<std::string> stage_failures;
    const auto append_route = [&combined](const PlanResult & part) {
        for (const Point & point : part.points) {
          if (combined.points.empty() ||
            Distance(combined.points.back(), point) > 1.0e-6)
          {
            combined.points.push_back(point);
          }
        }
      };
    const auto keep_success = [&append_unique, &append_route, &combined,
        &covered, &stage_start, &all_stages_complete](const PlanResult & part) {
        append_route(part);
        combined.visit_order.insert(
          combined.visit_order.end(), part.visit_order.begin(), part.visit_order.end());
        append_unique(combined.deferred_targets, part.deferred_targets);
        append_unique(covered, part.covered_edges);
        all_stages_complete = all_stages_complete && part.all_targets_reached;
        if (!part.points.empty() && part.headings.size() == part.points.size()) {
          stage_start = {part.points.back(), part.headings.back()};
        }
      };
    const auto keep_failure = [&append_unique, &combined, &all_stages_complete,
        &stage_failures](const std::string & stage, const PlanResult & part,
        const std::vector<std::string> & expected) {
        all_stages_complete = false;
        append_unique(combined.deferred_targets, part.deferred_targets);
        append_unique(combined.deferred_targets, expected);
        stage_failures.push_back(stage + ": " + part.message);
      };

    const PlanResult layer1 = Plan(
      stage_start, targets, labels, "layer1", covered);
    if (layer1.success) {
      keep_success(layer1);
    } else {
      keep_failure("task stage failed", layer1, labels);
    }

    const PlanResult layer2 = Plan(stage_start, {}, {}, "layer2", covered);
    if (layer2.success) {
      keep_success(layer2);
    } else {
      const std::vector<std::string> & tunnel_labels =
        config_.tunnel_segment_labels.empty() ?
        config_.required_tunnel_labels : config_.tunnel_segment_labels;
      keep_failure("tunnel stage failed", layer2, tunnel_labels);
    }

    const PlanResult layer3 = Plan(stage_start, {}, {}, "layer3", covered);
    if (layer3.success) {
      keep_success(layer3);
    } else {
      std::vector<std::string> road_labels;
      for (const InspectionEdge & edge : config_.inspection_edges) {
        if (!edge.tunnel &&
          std::find(covered.begin(), covered.end(), edge.label) == covered.end())
        {
          road_labels.push_back(edge.label);
        }
      }
      keep_failure("road stage failed", layer3, road_labels);
    }

    // Smooth across layer boundaries as well. Each layer is planned from the
    // previous endpoint, but independently smoothing them can leave a sharp
    // reverse turn at that shared boundary. Required checkpoints are retained
    // so this pass only rounds free-space connectors.
    std::vector<Point> required;
    required.reserve(targets.size() + config_.required_tunnel_points.size() +
      config_.inspection_nodes.size());
    required.insert(required.end(), targets.begin(), targets.end());
    required.insert(required.end(), config_.required_tunnel_points.begin(),
      config_.required_tunnel_points.end());
    required.insert(required.end(), config_.inspection_nodes.begin(),
      config_.inspection_nodes.end());
    const std::vector<Point> unsmoothed = combined.points;
    combined.points = Smooth(unsmoothed, required);
    if (!route_is_free(combined.points)) {
      combined.points = unsmoothed;
    }
    if (!route_is_free(combined.points)) {
      combined.points.clear();
      combined.message =
        "successful stages could not be combined into a collision-free route";
      return combined;
    }
    // Recompute coverage from the route that will actually be published.  A
    // layer reporting success is not enough if smoothing or a connector left
    // one of the configured inspection strokes untouched.
    const std::vector<std::string> current_coverage =
      CoveredInspectionEdges(config_, combined.points);
    combined.covered_edges = covered_edges;
    append_unique(combined.covered_edges, current_coverage);
    for (const InspectionEdge & edge : config_.inspection_edges) {
      if (std::find(combined.covered_edges.begin(), combined.covered_edges.end(),
          edge.label) == combined.covered_edges.end() &&
        std::find(combined.deferred_targets.begin(), combined.deferred_targets.end(),
          edge.label) == combined.deferred_targets.end())
      {
        combined.deferred_targets.push_back(edge.label);
      }
    }
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
    const bool has_visit_progress = std::any_of(
      combined.visit_order.begin(), combined.visit_order.end(),
      [](const std::string & label) {
        return !label.empty() && label != "S" && label != "RETURN_TO_START";
      });
    const bool has_coverage_progress = std::any_of(
      combined.covered_edges.begin(), combined.covered_edges.end(),
      [&covered_edges](const std::string & label) {
        return std::find(covered_edges.begin(), covered_edges.end(), label) ==
               covered_edges.end();
      });
    if (combined.length <= 1.0e-6 && !has_visit_progress &&
      !has_coverage_progress)
    {
      combined.points.clear();
      combined.headings.clear();
      combined.success = false;
      combined.all_targets_reached = false;
      combined.message = "no layered stage produced executable work";
    } else {
      combined.success = true;
      combined.all_targets_reached =
        all_stages_complete && combined.deferred_targets.empty();
      combined.message = combined.all_targets_reached ?
        "three-layer route planned" :
        "partial three-layer route planned; unavailable work deferred";
    }
    for (const std::string & failure : stage_failures) {
      combined.message += "; " + failure;
    }
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
    std::vector<int> tunnel_counterparts(nodes.size(), -1);
    std::vector<bool> tunnel_pair_is_free(nodes.size(), true);
    if (layer2) {
      constexpr const char * enter_suffix = "_ENTER";
      constexpr const char * exit_suffix = "_EXIT";
      for (std::size_t index = 1; index < nodes.size(); ++index) {
        const std::string & label = planning_labels[index - 1];
        std::string counterpart;
        if (label.size() > 6 &&
          label.rfind(enter_suffix) == label.size() - 6)
        {
          counterpart = label.substr(0, label.size() - 6) + exit_suffix;
        } else if (label.size() > 5 &&
          label.rfind(exit_suffix) == label.size() - 5)
        {
          counterpart = label.substr(0, label.size() - 5) + enter_suffix;
        } else {
          continue;
        }
        const auto found = std::find(
          planning_labels.begin(), planning_labels.end(), counterpart);
        if (found == planning_labels.end()) {
          throw std::runtime_error("tunnel counterpart is missing: " + counterpart);
        }
        const int counterpart_node = static_cast<int>(
          std::distance(planning_labels.begin(), found) + 1);
        tunnel_counterparts[index] = counterpart_node;
        // Reaching both portals by another road is not tunnel completion. The
        // configured portal-to-portal stroke itself must remain traversable.
        tunnel_pair_is_free[index] = SegmentIsFree(
          nodes[index], nodes[static_cast<std::size_t>(counterpart_node)]);
      }
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
    if (mode == "layer1" && remaining.size() <= 16 &&
      config_.inspection_nodes.size() <= 100) {
      const std::size_t count = remaining.size();
      const std::size_t state_count = std::size_t{1} << count;
      std::vector<double> costs(state_count * count, std::numeric_limits<double>::infinity());
      std::vector<int> parents(state_count * count, -1);
      std::vector<std::vector<GridPath>> pair_paths(
        nodes.size(), std::vector<GridPath>(nodes.size()));
      std::vector<std::vector<bool>> attempted(
        nodes.size(), std::vector<bool>(nodes.size(), false));
      std::vector<std::vector<bool>> reachable(
        nodes.size(), std::vector<bool>(nodes.size(), false));
      for (std::size_t target = 0; target < count; ++target) {
        attempted[0][target + 1] = true;
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
            if (!attempted[from][to]) {
              attempted[from][to] = true;
              try {
                pair_paths[from][to] = AStar(nodes[from], nodes[to]);
                reachable[from][to] = true;
              } catch (const std::runtime_error &) {
                reachable[from][to] = false;
              }
            }
            if (!reachable[from][to]) continue;
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
        const int candidate_node = remaining[position];
        if (forced_tunnel_exit >= 0 && candidate_node != forced_tunnel_exit) {
          continue;
        }
        if (layer2 && forced_tunnel_exit < 0 &&
          tunnel_counterparts[static_cast<std::size_t>(candidate_node)] >= 0 &&
          !tunnel_pair_is_free[static_cast<std::size_t>(candidate_node)])
        {
          continue;
        }
        try {
          GridPath path;
          if (layer2 && candidate_node == forced_tunnel_exit) {
            if (!SegmentIsFree(nodes[current_node], nodes[candidate_node])) {
              continue;
            }
            path.cost = Distance(nodes[current_node], nodes[candidate_node]);
          } else {
            path = AStar(nodes[current_node], nodes[candidate_node]);
          }
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
        result.deferred_targets.push_back("RETURN_TO_START");
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
    if (!route_is_free(result.points)) {
      throw std::runtime_error("planned route contains a segment that is not collision-free");
    }
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
    const std::vector<std::string> current_coverage =
      CoveredInspectionEdges(config_, result.points);
    result.covered_edges = covered_edges;
    append_unique(result.covered_edges, current_coverage);
    if (layer2) {
      const std::vector<std::string> checkpoints = result.visit_order;
      result.visit_order.clear();
      for (const std::string & checkpoint : checkpoints) {
        constexpr const char * enter_suffix = "_ENTER";
        constexpr const char * exit_suffix = "_EXIT";
        if (checkpoint.size() <= 6 ||
          checkpoint.rfind(enter_suffix) != checkpoint.size() - 6)
        {
          continue;
        }
        const std::string tunnel = checkpoint.substr(0, checkpoint.size() - 6);
        const std::string exit = tunnel + exit_suffix;
        if (std::find(checkpoints.begin(), checkpoints.end(), exit) ==
          checkpoints.end())
        {
          continue;
        }
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
