#include "arena_path_planner/planner_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace arena_path_planner
{
namespace
{

Point Interpolate(const Point & start, const Point & end, double fraction)
{
  return {
    start.x + fraction * (end.x - start.x),
    start.y + fraction * (end.y - start.y)};
}

int HeadingTurnCount(double from, double to)
{
  const double change = std::abs(NormalizeAngle(to - from));
  if (change < 1.0e-3) {
    return 0;
  }
  return change > 0.75 * std::acos(-1.0) ? 2 : 1;
}

double PathLength(const std::vector<Point> & points)
{
  double result = 0.0;
  for (std::size_t index = 1; index < points.size(); ++index) {
    result += Distance(points[index - 1], points[index]);
  }
  return result;
}

int PathTurnCount(const std::vector<Point> & points, double initial_heading)
{
  int turns = 0;
  double heading = initial_heading;
  for (std::size_t index = 1; index < points.size(); ++index) {
    if (Distance(points[index - 1], points[index]) <= 1.0e-9) {
      continue;
    }
    const double next = std::atan2(
      points[index].y - points[index - 1].y,
      points[index].x - points[index - 1].x);
    turns += HeadingTurnCount(heading, next);
    heading = next;
  }
  return turns;
}

bool BetterCandidate(
  int turns, double distance, double stroke_length,
  int best_turns, double best_distance, double best_stroke_length,
  double turn_penalty)
{
  const auto score = [turn_penalty](int turn_count, double travel, double stroke) {
      return turn_penalty * static_cast<double>(turn_count) + travel -
             std::min(stroke, 0.5 * turn_penalty);
    };
  return score(turns, distance, stroke_length) + 1.0e-9 <
         score(best_turns, best_distance, best_stroke_length);
}

}  // namespace

PlanResult ArenaPlanner::PlanRoadIntervals(
  const Pose & start,
  const std::vector<RoadInterval> & historical_coverage) const
{
  PlanResult result;
  try {
    struct Stroke
    {
      RoadInterval interval;
      Point start;
      Point end;
    };

    // Determine geometry on complete roads before subtracting confirmed work.
    // Otherwise yesterday's coverage boundary looks like a new obstacle edge
    // and each replan can creep farther into the same reserved stop buffer.
    std::vector<RoadInterval> traversable_intervals;
    for (const InspectionEdge & edge : config_.inspection_edges) {
      if (edge.tunnel) {
        continue;
      }
      const Point & edge_start = config_.inspection_nodes[edge.from];
      const Point & edge_end = config_.inspection_nodes[edge.to];
      const double length = Distance(edge_start, edge_end);
      if (length <= 1.0e-9) {
        continue;
      }
      const double yaw = std::atan2(
        edge_end.y - edge_start.y, edge_end.x - edge_start.x);
      const int samples = std::max(
        1, static_cast<int>(std::ceil(length / (0.5 * config_.resolution))));
      int run_start = -1;
      for (int sample = 0; sample < samples; ++sample) {
        const double from_fraction = static_cast<double>(sample) / samples;
        const double to_fraction = static_cast<double>(sample + 1) / samples;
        const Point from = Interpolate(edge_start, edge_end, from_fraction);
        const Point to = Interpolate(edge_start, edge_end, to_fraction);
        const bool leg_is_free =
          PoseIsFree(from, yaw) && PoseIsFree(to, yaw) && SegmentIsFree(from, to);
        if (leg_is_free && run_start < 0) {
          run_start = sample;
        }
        if (run_start >= 0 && (!leg_is_free || sample + 1 == samples)) {
          const int run_end = leg_is_free ? sample + 1 : sample;
          const double free_start = static_cast<double>(run_start) / samples;
          const double free_end = static_cast<double>(run_end) / samples;
          double clipped_start = free_start;
          double clipped_end = free_end;
          const double stop_buffer_fraction = std::min(
            1.0, config_.obstacle_stop_buffer / length);
          if (free_start > 1.0e-9) {
            clipped_start += stop_buffer_fraction;
          }
          if (free_end < 1.0 - 1.0e-9) {
            clipped_end -= stop_buffer_fraction;
          }
          if (clipped_end - clipped_start > 1.0e-6) {
            const Point clipped_from = Interpolate(
              edge_start, edge_end, clipped_start);
            const Point clipped_to = Interpolate(
              edge_start, edge_end, clipped_end);
            if (SegmentIsFree(clipped_from, clipped_to)) {
              traversable_intervals.push_back(
                {edge.label, clipped_start, clipped_end});
            }
          }
          run_start = -1;
        }
      }
    }
    traversable_intervals = NormalizeRoadIntervals(
      config_, {}, traversable_intervals);
    const std::vector<RoadInterval> blocked_intervals = UncoveredRoadIntervals(
      config_, traversable_intervals, false, true);
    const std::vector<RoadInterval> debt = UncoveredRoadIntervals(
      config_, historical_coverage, false, true);
    const std::vector<RoadInterval> work = IntersectRoadIntervals(
      config_, debt, traversable_intervals);
    std::vector<Stroke> strokes;
    strokes.reserve(work.size());
    for (const RoadInterval & interval : work) {
      const auto edge_position = std::find_if(
        config_.inspection_edges.begin(), config_.inspection_edges.end(),
        [&interval](const InspectionEdge & edge) {
          return edge.label == interval.edge_label;
        });
      if (edge_position == config_.inspection_edges.end()) {
        throw std::runtime_error("road work references an unknown inspection edge");
      }
      const Point & edge_start = config_.inspection_nodes[edge_position->from];
      const Point & edge_end = config_.inspection_nodes[edge_position->to];
      strokes.push_back({
        interval, Interpolate(edge_start, edge_end, interval.start_fraction),
        Interpolate(edge_start, edge_end, interval.end_fraction)});
    }

    result.points.push_back(start.position);
    Point current = start.position;
    double current_heading = start.yaw;
    std::vector<RoadInterval> planned_intervals;
    std::vector<Point> required_points = config_.inspection_nodes;
    const double turn_penalty =
      std::max(0.5, 4.0 * config_.minimum_turning_radius);
    while (!strokes.empty()) {
      std::size_t selected = strokes.size();
      bool selected_forward = true;
      int best_turns = std::numeric_limits<int>::max();
      double best_distance = std::numeric_limits<double>::infinity();
      double best_stroke_length = -1.0;
      std::vector<Point> best_approach;
      for (std::size_t index = 0; index < strokes.size(); ++index) {
        const Stroke & stroke = strokes[index];
        for (bool forward : {true, false}) {
          const Point entry = forward ? stroke.start : stroke.end;
          const Point exit = forward ? stroke.end : stroke.start;
          try {
            const GridPath grid_path = AStar(current, entry, current_heading);
            const std::vector<Point> approach = MaterializeGridPath(
              grid_path, current, entry);
            const double stroke_heading = std::atan2(
              exit.y - entry.y, exit.x - entry.x);
            double arrival_heading = current_heading;
            if (approach.size() >= 2) {
              arrival_heading = std::atan2(
                approach.back().y - approach[approach.size() - 2].y,
                approach.back().x - approach[approach.size() - 2].x);
            }
            const double entry_turn = std::abs(NormalizeAngle(
              stroke_heading - arrival_heading));
            if ((entry_turn > 2.6 || !config_.allow_in_place_turns) &&
              !(config_.turns_at_junctions_only && IsTurnJunction(entry)) &&
              !RotationIsFree(entry, arrival_heading, stroke_heading))
            {
              continue;
            }
            const int turns = PathTurnCount(approach, current_heading) +
              HeadingTurnCount(arrival_heading, stroke_heading);
            const double distance = PathLength(approach);
            const double stroke_length = Distance(entry, exit);
            if (selected == strokes.size() || BetterCandidate(
                turns, distance, stroke_length,
                best_turns, best_distance, best_stroke_length, turn_penalty))
            {
              selected = index;
              selected_forward = forward;
              best_turns = turns;
              best_distance = distance;
              best_stroke_length = stroke_length;
              best_approach = approach;
            }
          } catch (const std::runtime_error &) {
            continue;
          }
        }
      }
      if (selected == strokes.size()) {
        break;
      }

      const Stroke stroke = strokes[selected];
      const Point entry = selected_forward ? stroke.start : stroke.end;
      const Point exit = selected_forward ? stroke.end : stroke.start;
      if (result.points.size() == 1 && best_approach.size() >= 2) {
        // Preserve the heading-constrained departure through the final global
        // simplification. Without this anchor the isolated stage has no prior
        // segment, so a valid loop can collapse back into a boundary U-turn.
        required_points.push_back(best_approach[1]);
      }
      for (std::size_t index = 1; index < best_approach.size(); ++index) {
        if (Distance(result.points.back(), best_approach[index]) > 1.0e-9) {
          result.points.push_back(best_approach[index]);
        }
      }
      if (Distance(result.points.back(), entry) > 1.0e-9) {
        result.points.push_back(entry);
      }
      if (Distance(result.points.back(), exit) > 1.0e-9) {
        result.points.push_back(exit);
      }
      planned_intervals.push_back(stroke.interval);
      required_points.push_back(stroke.start);
      required_points.push_back(stroke.end);
      if (std::find(result.visit_order.begin(), result.visit_order.end(),
          stroke.interval.edge_label) == result.visit_order.end())
      {
        result.visit_order.push_back(stroke.interval.edge_label);
      }
      current = exit;
      current_heading = std::atan2(exit.y - entry.y, exit.x - entry.x);
      strokes.erase(strokes.begin() + static_cast<std::ptrdiff_t>(selected));
    }

    const std::vector<RoadInterval> normalized_planned = NormalizeRoadIntervals(
      config_, {}, planned_intervals);
    const std::vector<RoadInterval> prospective_coverage = MergeRoadIntervals(
      config_, historical_coverage, normalized_planned);
    const std::vector<RoadInterval> resolved_intervals = MergeRoadIntervals(
      config_, prospective_coverage, blocked_intervals);
    const std::vector<RoadInterval> remaining_intervals = UncoveredRoadIntervals(
      config_, resolved_intervals, false, true);
    if (normalized_planned.empty() && !remaining_intervals.empty()) {
      throw std::runtime_error(
              "no uncovered road interval is reachable from the current start");
    }
    if (Distance(current, config_.default_start.position) > 1.0e-6) {
      try {
        const GridPath return_grid = AStar(
          current, config_.default_start.position, current_heading);
        const std::vector<Point> return_path = MaterializeGridPath(
          return_grid, current, config_.default_start.position);
        for (std::size_t index = 1; index < return_path.size(); ++index) {
          if (Distance(result.points.back(), return_path[index]) > 1.0e-9) {
            result.points.push_back(return_path[index]);
          }
        }
      } catch (const std::runtime_error &) {
        result.deferred_targets.push_back("RETURN_TO_START");
      }
    }

    result.points = Smooth(result.points, required_points);
    for (std::size_t index = 1; index < result.points.size(); ++index) {
      if (!SegmentIsFree(result.points[index - 1], result.points[index])) {
        throw std::runtime_error(
                "road interval route contains a non-collision-free segment " +
                std::to_string(index - 1) + " -> " + std::to_string(index) +
                " (" + std::to_string(result.points[index - 1].x) + ", " +
                std::to_string(result.points[index - 1].y) + ") -> (" +
                std::to_string(result.points[index].x) + ", " +
                std::to_string(result.points[index].y) + ")");
      }
    }
    if (!RouteTurnsAreAllowed(result.points)) {
      throw std::runtime_error(
              "road interval route requires a U-turn or consecutive tight turns; "
              "recover to a junction before replanning");
    }
    result.headings.reserve(result.points.size());
    for (std::size_t index = 0; index < result.points.size(); ++index) {
      result.headings.push_back(
        index + 1 < result.points.size() ?
        std::atan2(
          result.points[index + 1].y - result.points[index].y,
          result.points[index + 1].x - result.points[index].x) :
        (result.headings.empty() ? start.yaw : result.headings.back()));
    }
    result.covered_intervals = historical_coverage;
    result.planned_intervals = normalized_planned;
    result.blocked_intervals = blocked_intervals;
    result.covered_edges = FullyCoveredInspectionEdges(
      config_, prospective_coverage);
    result.deferred_intervals = remaining_intervals;
    for (const RoadInterval & interval : result.deferred_intervals) {
      if (std::find(result.deferred_targets.begin(), result.deferred_targets.end(),
          interval.edge_label) == result.deferred_targets.end())
      {
        result.deferred_targets.push_back(interval.edge_label);
      }
    }
    result.length = PathLength(result.points);
    result.all_targets_reached =
      result.deferred_intervals.empty() && result.deferred_targets.empty();
    result.success = true;
    result.message = result.all_targets_reached ?
      "uncovered road intervals filled" :
      "reachable road intervals filled; remaining intervals deferred";
  } catch (const std::exception & exception) {
    result.success = false;
    result.all_targets_reached = false;
    result.message = exception.what();
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

}  // namespace arena_path_planner
