#include "arena_path_planner/planner_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace arena_path_planner
{
namespace
{

constexpr double kFractionTolerance = 1.0e-9;

std::unordered_map<std::string, std::size_t> EdgeOrder(const PlannerConfig & config)
{
  std::unordered_map<std::string, std::size_t> order;
  order.reserve(config.inspection_edges.size());
  for (std::size_t index = 0; index < config.inspection_edges.size(); ++index) {
    const std::string & label = config.inspection_edges[index].label;
    if (!order.emplace(label, index).second) {
      throw std::runtime_error("inspection edge labels must be unique: " + label);
    }
  }
  return order;
}

bool EdgeSelected(
  const InspectionEdge & edge, bool tunnels_only, bool non_tunnels_only)
{
  return (!tunnels_only && !non_tunnels_only) ||
         (tunnels_only && edge.tunnel) ||
         (non_tunnels_only && !edge.tunnel);
}

}  // namespace

std::vector<RoadInterval> NormalizeRoadIntervals(
  const PlannerConfig & config, const std::vector<std::string> & covered_edges,
  const std::vector<RoadInterval> & intervals)
{
  const auto edge_order = EdgeOrder(config);
  std::vector<RoadInterval> normalized;
  normalized.reserve(covered_edges.size() + intervals.size());
  for (const std::string & label : covered_edges) {
    if (edge_order.find(label) == edge_order.end()) {
      throw std::runtime_error("covered_edges contains unknown label: " + label);
    }
    normalized.push_back({label, 0.0, 1.0});
  }
  for (RoadInterval interval : intervals) {
    if (edge_order.find(interval.edge_label) == edge_order.end()) {
      throw std::runtime_error(
              "covered_intervals contains unknown label: " + interval.edge_label);
    }
    if (!std::isfinite(interval.start_fraction) ||
      !std::isfinite(interval.end_fraction))
    {
      throw std::runtime_error("road interval fractions must be finite");
    }
    if (interval.start_fraction > interval.end_fraction) {
      std::swap(interval.start_fraction, interval.end_fraction);
    }
    if (interval.start_fraction < -kFractionTolerance ||
      interval.end_fraction > 1.0 + kFractionTolerance)
    {
      throw std::runtime_error("road interval fractions must be within [0, 1]");
    }
    interval.start_fraction = std::clamp(interval.start_fraction, 0.0, 1.0);
    interval.end_fraction = std::clamp(interval.end_fraction, 0.0, 1.0);
    if (interval.end_fraction - interval.start_fraction > kFractionTolerance) {
      normalized.push_back(std::move(interval));
    }
  }
  std::sort(
    normalized.begin(), normalized.end(), [&edge_order](
      const RoadInterval & left, const RoadInterval & right) {
      const std::size_t left_order = edge_order.at(left.edge_label);
      const std::size_t right_order = edge_order.at(right.edge_label);
      if (left_order != right_order) {
        return left_order < right_order;
      }
      if (left.start_fraction != right.start_fraction) {
        return left.start_fraction < right.start_fraction;
      }
      return left.end_fraction < right.end_fraction;
    });

  std::vector<RoadInterval> merged;
  for (const RoadInterval & interval : normalized) {
    if (!merged.empty() && merged.back().edge_label == interval.edge_label &&
      interval.start_fraction <= merged.back().end_fraction + kFractionTolerance)
    {
      merged.back().end_fraction = std::max(
        merged.back().end_fraction, interval.end_fraction);
    } else {
      merged.push_back(interval);
    }
  }
  return merged;
}

std::vector<RoadInterval> MergeRoadIntervals(
  const PlannerConfig & config, const std::vector<RoadInterval> & left,
  const std::vector<RoadInterval> & right)
{
  std::vector<RoadInterval> combined = left;
  combined.insert(combined.end(), right.begin(), right.end());
  return NormalizeRoadIntervals(config, {}, combined);
}

std::vector<RoadInterval> IntersectRoadIntervals(
  const PlannerConfig & config, const std::vector<RoadInterval> & left,
  const std::vector<RoadInterval> & right)
{
  const std::vector<RoadInterval> normalized_left =
    NormalizeRoadIntervals(config, {}, left);
  const std::vector<RoadInterval> normalized_right =
    NormalizeRoadIntervals(config, {}, right);
  std::vector<RoadInterval> result;
  for (const RoadInterval & lhs : normalized_left) {
    for (const RoadInterval & rhs : normalized_right) {
      if (lhs.edge_label != rhs.edge_label) {
        continue;
      }
      const double begin = std::max(lhs.start_fraction, rhs.start_fraction);
      const double end = std::min(lhs.end_fraction, rhs.end_fraction);
      if (end - begin > kFractionTolerance) {
        result.push_back({lhs.edge_label, begin, end});
      }
    }
  }
  return NormalizeRoadIntervals(config, {}, result);
}

std::vector<RoadInterval> UncoveredRoadIntervals(
  const PlannerConfig & config, const std::vector<RoadInterval> & covered,
  bool tunnels_only, bool non_tunnels_only)
{
  const std::vector<RoadInterval> normalized =
    NormalizeRoadIntervals(config, {}, covered);
  std::vector<RoadInterval> result;
  for (const InspectionEdge & edge : config.inspection_edges) {
    if (!EdgeSelected(edge, tunnels_only, non_tunnels_only)) {
      continue;
    }
    double cursor = 0.0;
    for (const RoadInterval & interval : normalized) {
      if (interval.edge_label != edge.label) {
        continue;
      }
      if (interval.start_fraction > cursor + kFractionTolerance) {
        result.push_back({edge.label, cursor, interval.start_fraction});
      }
      cursor = std::max(cursor, interval.end_fraction);
    }
    if (cursor < 1.0 - kFractionTolerance) {
      result.push_back({edge.label, cursor, 1.0});
    }
  }
  return result;
}

std::vector<RoadInterval> CoveredInspectionIntervals(
  const PlannerConfig & config, const std::vector<Point> & route)
{
  if (route.size() < 2) {
    return {};
  }
  std::vector<RoadInterval> covered;
  const auto distance_to_route = [&route](const Point & point) {
      double minimum = std::numeric_limits<double>::infinity();
      for (std::size_t index = 1; index < route.size(); ++index) {
        minimum = std::min(
          minimum, DistanceToSegment(point, route[index - 1], route[index]));
      }
      return minimum;
    };
  for (const InspectionEdge & edge : config.inspection_edges) {
    const Point & from = config.inspection_nodes[static_cast<std::size_t>(edge.from)];
    const Point & to = config.inspection_nodes[static_cast<std::size_t>(edge.to)];
    const double sample_spacing = std::max(
      0.005, std::min(config.resolution, config.task_tolerance * 0.5));
    const int samples = std::max(
      1, static_cast<int>(std::ceil(Distance(from, to) / sample_spacing)));
    int run_start = -1;
    for (int sample = 0; sample <= samples; ++sample) {
      const double fraction = static_cast<double>(sample) / samples;
      const Point point{
        from.x + fraction * (to.x - from.x),
        from.y + fraction * (to.y - from.y)};
      const bool is_covered =
        distance_to_route(point) <= config.task_tolerance + 1.0e-9;
      if (is_covered && run_start < 0) {
        run_start = sample;
      }
      if (run_start >= 0 && (!is_covered || sample == samples)) {
        const int run_end = is_covered ? sample : sample - 1;
        const double half_sample = 0.5 / samples;
        covered.push_back({
          edge.label,
          std::max(0.0, static_cast<double>(run_start) / samples - half_sample),
          std::min(1.0, static_cast<double>(run_end) / samples + half_sample)});
        run_start = -1;
      }
    }
  }
  return NormalizeRoadIntervals(config, {}, covered);
}

std::vector<std::string> FullyCoveredInspectionEdges(
  const PlannerConfig & config, const std::vector<RoadInterval> & intervals)
{
  const std::vector<RoadInterval> normalized =
    NormalizeRoadIntervals(config, {}, intervals);
  std::vector<std::string> result;
  for (const InspectionEdge & edge : config.inspection_edges) {
    const auto complete = std::find_if(
      normalized.begin(), normalized.end(), [&edge](const RoadInterval & interval) {
        return interval.edge_label == edge.label &&
               interval.start_fraction <= kFractionTolerance &&
               interval.end_fraction >= 1.0 - kFractionTolerance;
      });
    if (complete != normalized.end()) {
      result.push_back(edge.label);
    }
  }
  return result;
}

}  // namespace arena_path_planner
