#ifndef ARENA_PATH_PLANNER__PLANNER_INTERNAL_HPP_
#define ARENA_PATH_PLANNER__PLANNER_INTERNAL_HPP_

#include "arena_path_planner/planner.hpp"

#include <string>
#include <vector>

namespace arena_path_planner {

// Shared implementation helpers. They are intentionally kept out of the
// public header so the planning API remains stable while source files stay
// independently testable.
bool Contains(const Rectangle &rectangle, const Point &point);
Point QuadraticBezier(const Point &start, const Point &control,
                      const Point &end, double ratio);
double PolylineLength(const std::vector<Point> &points);
bool IsFinite(const Point &point);
bool IsFinite(const Pose &pose);
bool IsFinite(const Rectangle &rectangle);
// Validate values that otherwise reach grid-size calculations, floor/casts,
// or path-cost arithmetic.  Both YAML loading and direct test/API
// construction use this same guard.
void ValidatePlannerConfig(const PlannerConfig &config);
std::vector<std::string>
CoveredInspectionEdges(const PlannerConfig &config,
                       const std::vector<Point> &route);
std::vector<RoadInterval> NormalizeRoadIntervals(
  const PlannerConfig & config, const std::vector<std::string> & covered_edges,
  const std::vector<RoadInterval> & intervals);
std::vector<RoadInterval> MergeRoadIntervals(
  const PlannerConfig & config, const std::vector<RoadInterval> & left,
  const std::vector<RoadInterval> & right);
std::vector<RoadInterval> IntersectRoadIntervals(
  const PlannerConfig & config, const std::vector<RoadInterval> & left,
  const std::vector<RoadInterval> & right);
std::vector<RoadInterval> UncoveredRoadIntervals(
  const PlannerConfig & config, const std::vector<RoadInterval> & covered,
  bool tunnels_only = false, bool non_tunnels_only = false);
std::vector<RoadInterval> CoveredInspectionIntervals(
  const PlannerConfig & config, const std::vector<Point> & route);
std::vector<std::string> FullyCoveredInspectionEdges(
  const PlannerConfig & config, const std::vector<RoadInterval> & intervals);

} // namespace arena_path_planner

#endif // ARENA_PATH_PLANNER__PLANNER_INTERNAL_HPP_
