#include "arena_path_planner/planner_internal.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace arena_path_planner
{
PlanResult ArenaPlanner::Plan(
  const Pose & start, const std::vector<Point> & targets,
  const std::vector<std::string> & labels, const std::string & mode,
  const std::vector<std::string> & covered_edges,
  const std::vector<RoadInterval> & covered_intervals) const
{
  const auto failed = [](const std::string & message) {
      PlanResult result;
      result.message = message;
      return result;
    };
  if (mode == "layered") {
    return failed(
      "layered mode is disabled because concatenating stages can create an "
      "unexecutable reversal; execute layer1, layer2, and layer3 sequentially");
  }
  if (!IsFinite(start)) {
    return failed("start pose must contain finite coordinates and yaw");
  }
  for (const Point & target : targets) {
    if (!IsFinite(target)) {
      return failed("target coordinates must be finite");
    }
  }
  std::vector<RoadInterval> historical_coverage;
  try {
    historical_coverage = NormalizeRoadIntervals(
      config_, covered_edges, covered_intervals);
  } catch (const std::exception & exception) {
    return failed(exception.what());
  }
  const auto route_is_free = [this](const std::vector<Point> & points) {
      for (std::size_t index = 1; index < points.size(); ++index) {
        if (!SegmentIsFree(points[index - 1], points[index])) {
          return false;
        }
      }
      return true;
    };
  const double turn_priority =
    std::max(0.5, 4.0 * config_.minimum_turning_radius);
  const auto path_score = [turn_priority](const GridPath & path) {
      return turn_priority * static_cast<double>(path.turn_count) + path.cost;
    };
  const auto first_blocked_reversal = [this](const std::vector<Point> & points) {
      constexpr double near_reversal_threshold = 2.6;
      for (std::size_t index = 1; index + 1 < points.size(); ++index) {
        const double incoming = std::atan2(
          points[index].y - points[index - 1].y,
          points[index].x - points[index - 1].x);
        const double outgoing = std::atan2(
          points[index + 1].y - points[index].y,
          points[index + 1].x - points[index].x);
        if (std::abs(NormalizeAngle(outgoing - incoming)) >
          near_reversal_threshold)
        {
          return index;
        }
      }
      return points.size();
    };
  if (mode == "coverage") {
    return PlanCoverage(start, covered_edges, historical_coverage);
  }
  if (mode == "layer3") {
    return PlanCoverage(
      start, covered_edges, historical_coverage, false, true);
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
      result.covered_intervals = historical_coverage;
      result.covered_edges = FullyCoveredInspectionEdges(
        config_, result.covered_intervals);
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
    double current_heading = start.yaw;
    int forced_tunnel_exit = -1;
    bool retry_pass = false;
    bool optimized_open_route = false;
    // The first-stage distance optimum can contain a physically impossible
    // reversal at a task point. Retain the incoming target in the DP state and
    // reject transitions whose actual A* endpoint directions require a
    // near-U-turn. Larger requests use the bounded greedy fallback after each
    // candidate has been checked against the current approach heading.
    if (mode == "layer1" && remaining.size() <= 10 &&
      config_.inspection_nodes.size() <= 100) {
      const std::size_t count = remaining.size();
      const std::size_t state_count = std::size_t{1} << count;
      const std::size_t first_followup_node = nodes.size();
      nodes.insert(nodes.end(), config_.required_tunnel_points.begin(),
        config_.required_tunnel_points.end());
      std::vector<std::vector<GridPath>> pair_paths(
        nodes.size(), std::vector<GridPath>(nodes.size()));
      std::vector<std::vector<bool>> attempted(
        nodes.size(), std::vector<bool>(nodes.size(), false));
      std::vector<std::vector<bool>> reachable(
        nodes.size(), std::vector<bool>(nodes.size(), false));
      const auto ensure_path = [this, &nodes, &pair_paths, &attempted, &reachable](
          int from, int to) {
          if (!attempted[from][to]) {
            attempted[from][to] = true;
            try {
              pair_paths[from][to] = AStar(nodes[from], nodes[to]);
              reachable[from][to] = true;
            } catch (const std::runtime_error &) {
              reachable[from][to] = false;
            }
          }
          return reachable[from][to];
        };
      const std::size_t pair_count = nodes.size();
      std::vector<int8_t> endpoint_status(pair_count * pair_count, -1);
      std::vector<double> departure_yaws(pair_count * pair_count, 0.0);
      std::vector<double> arrival_yaws(pair_count * pair_count, 0.0);
      const auto endpoint_yaws = [this, &nodes, &pair_paths, &ensure_path,
          pair_count, &endpoint_status, &departure_yaws, &arrival_yaws](
          int from, int to, double & departure, double & arrival) {
          const std::size_t pair_index =
            static_cast<std::size_t>(from) * pair_count + static_cast<std::size_t>(to);
          if (endpoint_status[pair_index] >= 0) {
            departure = departure_yaws[pair_index];
            arrival = arrival_yaws[pair_index];
            return endpoint_status[pair_index] != 0;
          }
          if (!ensure_path(from, to)) {
            endpoint_status[pair_index] = 0;
            return false;
          }
          std::vector<Point> raw;
          try {
            raw = MaterializeGridPath(
              pair_paths[from][to], nodes[from], nodes[to]);
          } catch (const std::runtime_error &) {
            endpoint_status[pair_index] = 0;
            return false;
          }
          if (raw.size() < 2) {
            endpoint_status[pair_index] = 0;
            return false;
          }
          // Exact task points often sit on a cell boundary, so the first grid
          // centre can create a meaningless 1 cm diagonal. Use a short sample
          // of the actual A* leg to recover its local direction at each end.
          const double probe_distance = std::max(0.12, 6.0 * config_.resolution);
          std::size_t departure_index = 1;
          while (departure_index + 1 < raw.size() &&
            Distance(raw.front(), raw[departure_index]) < probe_distance)
          {
            ++departure_index;
          }
          std::size_t arrival_index = raw.size() - 2;
          while (arrival_index > 0 &&
            Distance(raw.back(), raw[arrival_index]) < probe_distance)
          {
            --arrival_index;
          }
          departure = std::atan2(
            raw[departure_index].y - raw.front().y,
            raw[departure_index].x - raw.front().x);
          arrival = std::atan2(
            raw.back().y - raw[arrival_index].y,
            raw.back().x - raw[arrival_index].x);
          departure_yaws[pair_index] = departure;
          arrival_yaws[pair_index] = arrival;
          endpoint_status[pair_index] = 1;
          return true;
        };
      constexpr double near_reversal_threshold = 2.6;
      const auto blocked_turn = [this, &nodes, &endpoint_yaws](
          int before, int pivot, int after) {
          double unused_departure = 0.0;
          double incoming = 0.0;
          double outgoing = 0.0;
          double unused_arrival = 0.0;
          return endpoint_yaws(before, pivot, unused_departure, incoming) &&
                 endpoint_yaws(pivot, after, outgoing, unused_arrival) &&
                 std::abs(NormalizeAngle(outgoing - incoming)) >
                   near_reversal_threshold;
        };
      const auto followup_node = [&ensure_path, &pair_paths, &path_score,
          first_followup_node, &nodes](int from) {
          int selected = -1;
          double best = std::numeric_limits<double>::infinity();
          for (std::size_t candidate = first_followup_node;
            candidate < nodes.size(); ++candidate)
          {
            if (ensure_path(from, static_cast<int>(candidate)) &&
              path_score(pair_paths[from][candidate]) < best)
            {
              selected = static_cast<int>(candidate);
              best = path_score(pair_paths[from][candidate]);
            }
          }
          return selected;
        };

      const std::size_t start_slot = count;
      const std::size_t previous_count = count + 1;
      const auto state_index = [count, previous_count](
          std::size_t mask, std::size_t last, std::size_t previous) {
          return (mask * count + last) * previous_count + previous;
        };
      const std::size_t table_size = state_count * count * previous_count;
      std::vector<double> costs(table_size, std::numeric_limits<double>::infinity());
      std::vector<int> parent_previous(table_size, -1);
      std::vector<int8_t> transition_allowed(previous_count * count * count, -1);
      const auto transition_index = [count](
          std::size_t previous, std::size_t last, std::size_t next) {
          return (previous * count + last) * count + next;
        };
      const auto turn_is_allowed = [&remaining, start_slot, &transition_allowed,
          &transition_index, &blocked_turn](
          std::size_t previous, std::size_t last, std::size_t next) {
          int8_t & cached = transition_allowed[transition_index(previous, last, next)];
          if (cached < 0) {
            const int before = previous == start_slot ? 0 : remaining[previous];
            cached = blocked_turn(before, remaining[last], remaining[next]) ? 0 : 1;
          }
          return cached != 0;
        };

      for (std::size_t target = 0; target < count; ++target) {
        const int node = remaining[target];
        if (ensure_path(0, node)) {
          costs[state_index(std::size_t{1} << target, target, start_slot)] =
            path_score(pair_paths[0][node]);
        }
      }
      for (std::size_t mask = 1; mask < state_count; ++mask) {
        for (std::size_t last = 0; last < count; ++last) {
          if (!(mask & (std::size_t{1} << last))) continue;
          for (std::size_t previous = 0; previous < previous_count; ++previous) {
            const double current_cost = costs[state_index(mask, last, previous)];
            if (!std::isfinite(current_cost)) continue;
            for (std::size_t next = 0; next < count; ++next) {
              if (mask & (std::size_t{1} << next)) continue;
              const int from = remaining[last];
              const int to = remaining[next];
              if (!ensure_path(from, to) || !turn_is_allowed(previous, last, next)) continue;
              const std::size_t next_mask = mask | (std::size_t{1} << next);
              const double candidate = current_cost + path_score(pair_paths[from][to]);
              const std::size_t destination = state_index(next_mask, next, last);
              if (candidate < costs[destination]) {
                costs[destination] = candidate;
                parent_previous[destination] = static_cast<int>(previous);
              }
            }
          }
        }
      }

      std::vector<int> accepted_sequence;
      const std::size_t full_mask = state_count - 1;
      int last = -1;
      int previous = -1;
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t candidate = 0; candidate < count; ++candidate) {
        const int pivot = remaining[candidate];
        const int after = followup_node(pivot);
        for (std::size_t candidate_previous = 0;
          candidate_previous < previous_count; ++candidate_previous)
        {
          const double candidate_cost = costs[state_index(
              full_mask, candidate, candidate_previous)];
          if (!std::isfinite(candidate_cost)) continue;
          const int before = candidate_previous == start_slot ? 0 :
            remaining[candidate_previous];
          if (after >= 0 && blocked_turn(before, pivot, after)) continue;
          if (candidate_cost < best) {
            best = candidate_cost;
            last = static_cast<int>(candidate);
            previous = static_cast<int>(candidate_previous);
          }
        }
      }
      if (last >= 0) {
        accepted_sequence.resize(count);
        std::size_t mask = full_mask;
        for (std::size_t position = count; position-- > 0;) {
          accepted_sequence[position] = remaining[static_cast<std::size_t>(last)];
          if (position == 0) break;
          const int before_previous = parent_previous[state_index(
              mask, static_cast<std::size_t>(last), static_cast<std::size_t>(previous))];
          mask ^= std::size_t{1} << static_cast<std::size_t>(last);
          last = previous;
          previous = before_previous;
        }
      }
      if (!accepted_sequence.empty()) {
        bool complete = true;
        for (std::size_t position = 0; position < accepted_sequence.size(); ++position) {
          const int node = accepted_sequence[position];
          const int previous_node = position == 0 ? 0 : accepted_sequence[position - 1];
          if (!reachable[previous_node][node]) {
            complete = false;
            break;
          }
          selected_paths.push_back(pair_paths[previous_node][node]);
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
            path = config_.turns_at_junctions_only ?
              AStar(nodes[current_node], nodes[candidate_node], current_heading) :
              AStar(nodes[current_node], nodes[candidate_node]);
          }
          if (config_.turns_at_junctions_only && !path.cells.empty()) {
            const std::vector<Point> preview = MaterializeGridPath(
              path, nodes[current_node], nodes[candidate_node]);
            if (preview.size() >= 2) {
              std::size_t departure_index = 1;
              while (departure_index + 1 < preview.size() &&
                Distance(preview.front(), preview[departure_index]) < 0.08)
              {
                ++departure_index;
              }
              const double departure = std::atan2(
                preview[departure_index].y - preview[0].y,
                preview[departure_index].x - preview[0].x);
              if (std::abs(NormalizeAngle(departure - current_heading)) > 2.6 ||
                first_blocked_reversal(preview) < preview.size())
              {
                continue;
              }
            }
          }
          if (planning_mode == "numbered" || path_score(path) < selected_cost) {
            selected_position = static_cast<int>(position);
            selected_cost = path_score(path);
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
      const int previous_node = current_node;
      const int selected_node = remaining[static_cast<std::size_t>(selected_position)];
      if (selected_path.cells.empty()) {
        current_heading = std::atan2(
          nodes[selected_node].y - nodes[previous_node].y,
          nodes[selected_node].x - nodes[previous_node].x);
      } else {
        const std::vector<Point> selected_points = MaterializeGridPath(
          selected_path, nodes[previous_node], nodes[selected_node]);
        if (selected_points.size() >= 2) {
          std::size_t arrival_index = selected_points.size() - 2;
          while (arrival_index > 0 &&
            Distance(selected_points.back(), selected_points[arrival_index]) < 0.08)
          {
            --arrival_index;
          }
          current_heading = std::atan2(
            selected_points.back().y -
            selected_points[arrival_index].y,
            selected_points.back().x -
            selected_points[arrival_index].x);
        }
      }
      current_node = selected_node;
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
    double arrival_heading = start.yaw;
    for (std::size_t leg = 1; leg < order.size(); ++leg) {
      const int left = order[leg - 1];
      const int right = order[leg];
      const auto & cells = selected_paths[leg - 1].cells;
      std::vector<Point> raw{anchors.back()};
      if (cells.empty()) {
        // A forced tunnel traversal is represented by its exact two portals.
        // The preceding A* leg can end at a nearby grid centre, so reconnect
        // to the entrance explicitly before crossing the tunnel centreline.
        if (Distance(raw.back(), nodes[left]) > 1.0e-9) {
          if (!SegmentIsFree(raw.back(), nodes[left])) {
            throw std::runtime_error("cannot safely reach the tunnel entrance");
          }
          raw.push_back(nodes[left]);
        }
        if (!SegmentIsFree(raw.back(), nodes[right])) {
          throw std::runtime_error("configured tunnel traversal is not collision-free");
        }
        raw.push_back(nodes[right]);
      } else {
        const GridPath heading_aware = AStar(
          raw.back(), nodes[right], arrival_heading);
        raw = MaterializeGridPath(
          heading_aware, anchors.back(), nodes[right]);
      }
      std::vector<Point> simplified = Simplify(raw);
      if (simplified.size() >= 2) {
        std::size_t arrival_index = simplified.size() - 2;
        while (arrival_index > 0 &&
          Distance(simplified.back(), simplified[arrival_index]) < 0.08)
        {
          --arrival_index;
        }
        arrival_heading = std::atan2(
          simplified.back().y - simplified[arrival_index].y,
          simplified.back().x - simplified[arrival_index].x);
      }
      anchors.insert(anchors.end(), simplified.begin() + 1, simplified.end());
      result.visit_order.push_back(right == 0 ? "S" : planning_labels[right - 1]);
    }
    std::vector<Point> reached_targets;
    for (std::size_t index = 1; index < order.size(); ++index) {
      if (order[index] != 0) {
        reached_targets.push_back(nodes[order[index]]);
      }
    }
    std::vector<Point> semantic_points = reached_targets;
    semantic_points.insert(
      semantic_points.end(), config_.inspection_nodes.begin(),
      config_.inspection_nodes.end());
    semantic_points.insert(
      semantic_points.end(), config_.turn_junctions.begin(),
      config_.turn_junctions.end());
    semantic_points.insert(semantic_points.end(), anchors.begin(), anchors.end());
    result.points = Smooth(anchors, semantic_points);
    if (!route_is_free(result.points)) {
      for (std::size_t index = 1; index < result.points.size(); ++index) {
        if (SegmentIsFree(result.points[index - 1], result.points[index])) {
          continue;
        }
        std::ostringstream message;
        message << "planned route segment " << index - 1 << " -> " << index
                << " is not collision-free: (" << result.points[index - 1].x
                << ", " << result.points[index - 1].y << ") -> ("
                << result.points[index].x << ", " << result.points[index].y << ")";
        throw std::runtime_error(message.str());
      }
    }
    if (!RouteTurnsAreAllowed(result.points)) {
      constexpr double near_reversal_threshold = 2.6;
      constexpr double major_turn_threshold = 1.0;
      const double consecutive_turn_distance =
        std::max(0.35, config_.minimum_turning_radius + 2.0 * config_.resolution);
      std::size_t previous_major_turn = result.points.size();
      for (std::size_t index = 1; index + 1 < result.points.size(); ++index) {
        const double incoming = std::atan2(
          result.points[index].y - result.points[index - 1].y,
          result.points[index].x - result.points[index - 1].x);
        const double outgoing = std::atan2(
          result.points[index + 1].y - result.points[index].y,
          result.points[index + 1].x - result.points[index].x);
        const double heading_change = std::abs(NormalizeAngle(outgoing - incoming));
        if (heading_change >= near_reversal_threshold) {
          std::ostringstream message;
          message << "task route requires a 180 degree U-turn at route index "
                  << index << " (" << result.points[index].x << ", "
                  << result.points[index].y << "), previous ("
                  << result.points[index - 1].x << ", "
                  << result.points[index - 1].y << "), next ("
                  << result.points[index + 1].x << ", "
                  << result.points[index + 1].y
                  << "); recover to a junction before replanning";
          throw std::runtime_error(message.str());
        }
        if (heading_change >= config_.in_place_turn_heading_threshold &&
          !IsTurnJunction(
            result.points[index], kTurnJunctionOperatingTolerance))
        {
          std::ostringstream message;
          message << "task route turns outside the road junctions at route index "
                  << index << " (" << result.points[index].x << ", "
                  << result.points[index].y << "), previous ("
                  << result.points[index - 1].x << ", "
                  << result.points[index - 1].y << "), next ("
                  << result.points[index + 1].x << ", "
                  << result.points[index + 1].y << "), turn "
                  << heading_change * 180.0 / std::acos(-1.0) << " deg";
          throw std::runtime_error(message.str());
        }
        if (heading_change >= major_turn_threshold) {
          if (previous_major_turn + 1 == index &&
            Distance(result.points[previous_major_turn], result.points[index]) <=
            consecutive_turn_distance)
          {
            std::ostringstream message;
            message << "task route requires consecutive tight 90 degree turns "
                    << "at route indices " << previous_major_turn << " and "
                    << index << ", middle distance "
                    << Distance(result.points[previous_major_turn], result.points[index])
                    << " m; recover to a junction before replanning";
            throw std::runtime_error(message.str());
          }
          previous_major_turn = index;
        }
      }
      throw std::runtime_error(
              "task route requires a U-turn or consecutive tight turns; "
              "recover to a junction before replanning");
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
    if (mode == "layer1" || layer2) {
      const std::size_t blocked = first_blocked_reversal(result.points);
      if (blocked < result.points.size()) {
        std::ostringstream message;
        message << (layer2 ? "tunnel route" : "task route")
                << " contains a blocked near-U-turn at ("
                << result.points[blocked].x << ", " << result.points[blocked].y
                << "), route index " << blocked;
        throw std::runtime_error(message.str());
      }
    }
    result.length = PolylineLength(result.points);
    result.covered_intervals = historical_coverage;
    result.planned_intervals = CoveredInspectionIntervals(config_, result.points);
    const std::vector<RoadInterval> prospective_coverage = MergeRoadIntervals(
      config_, historical_coverage, result.planned_intervals);
    result.covered_edges = FullyCoveredInspectionEdges(
      config_, prospective_coverage);
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
    result.covered_intervals.clear();
    result.planned_intervals.clear();
    result.blocked_intervals.clear();
    result.deferred_intervals.clear();
  }
  return result;
}

}  // namespace arena_path_planner
