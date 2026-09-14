#include "arena_path_planner/planner_internal.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace arena_path_planner
{
namespace
{

PlannerConfig IntervalConfig()
{
  PlannerConfig config;
  config.width = 2.0;
  config.height = 1.0;
  config.resolution = 0.05;
  config.inflation_radius = 0.0;
  config.preferred_clearance = 0.0;
  config.clearance_cost_weight = 0.0;
  config.task_tolerance = 0.02;
  config.free_regions = {{0.0, 0.0, 2.0, 1.0}};
  config.inspection_nodes = {{0.2, 0.5}, {1.8, 0.5}};
  config.inspection_edges = {{0, 1, "ROAD", false}};
  return config;
}

TEST(RoadIntervals, MergesDirectionalCoverageAndPreservesDebt)
{
  const PlannerConfig config = IntervalConfig();
  const std::vector<RoadInterval> merged = NormalizeRoadIntervals(
    config, {}, {{"ROAD", 0.0, 0.35}, {"ROAD", 0.70, 1.0},
      {"ROAD", 0.35, 0.40}});
  ASSERT_EQ(merged.size(), 2U);
  EXPECT_DOUBLE_EQ(merged[0].start_fraction, 0.0);
  EXPECT_DOUBLE_EQ(merged[0].end_fraction, 0.40);
  EXPECT_DOUBLE_EQ(merged[1].start_fraction, 0.70);
  EXPECT_DOUBLE_EQ(merged[1].end_fraction, 1.0);

  const std::vector<RoadInterval> debt = UncoveredRoadIntervals(config, merged);
  ASSERT_EQ(debt.size(), 1U);
  EXPECT_DOUBLE_EQ(debt[0].start_fraction, 0.40);
  EXPECT_DOUBLE_EQ(debt[0].end_fraction, 0.70);
  EXPECT_TRUE(FullyCoveredInspectionEdges(config, merged).empty());
}

TEST(RoadIntervals, LegacyCoveredEdgeMeansTheWholeInterval)
{
  const PlannerConfig config = IntervalConfig();
  const std::vector<RoadInterval> merged = NormalizeRoadIntervals(
    config, {"ROAD"}, {{"ROAD", 0.25, 0.50}});
  ASSERT_EQ(merged.size(), 1U);
  EXPECT_DOUBLE_EQ(merged[0].start_fraction, 0.0);
  EXPECT_DOUBLE_EQ(merged[0].end_fraction, 1.0);
  EXPECT_EQ(FullyCoveredInspectionEdges(config, merged),
    std::vector<std::string>({"ROAD"}));
  EXPECT_TRUE(UncoveredRoadIntervals(config, merged).empty());
}

TEST(RoadIntervals, IntersectsDebtWithStableTraversableRanges)
{
  const PlannerConfig config = IntervalConfig();
  const std::vector<RoadInterval> intersection = IntersectRoadIntervals(
    config,
    {{"ROAD", 0.0, 0.25}, {"ROAD", 0.40, 0.80}},
    {{"ROAD", 0.20, 0.50}, {"ROAD", 0.70, 1.0}});
  ASSERT_EQ(intersection.size(), 3U);
  EXPECT_DOUBLE_EQ(intersection[0].start_fraction, 0.20);
  EXPECT_DOUBLE_EQ(intersection[0].end_fraction, 0.25);
  EXPECT_DOUBLE_EQ(intersection[1].start_fraction, 0.40);
  EXPECT_DOUBLE_EQ(intersection[1].end_fraction, 0.50);
  EXPECT_DOUBLE_EQ(intersection[2].start_fraction, 0.70);
  EXPECT_DOUBLE_EQ(intersection[2].end_fraction, 0.80);
}

TEST(RoadIntervals, RouteReportsPartialCoverageWithoutClaimingWholeEdge)
{
  const PlannerConfig config = IntervalConfig();
  const std::vector<RoadInterval> intervals = CoveredInspectionIntervals(
    config, {{0.2, 0.5}, {0.8, 0.5}});
  ASSERT_EQ(intervals.size(), 1U);
  EXPECT_DOUBLE_EQ(intervals[0].start_fraction, 0.0);
  EXPECT_GT(intervals[0].end_fraction, 0.35);
  EXPECT_LT(intervals[0].end_fraction, 0.45);
  EXPECT_TRUE(FullyCoveredInspectionEdges(config, intervals).empty());
}

TEST(RoadIntervals, RejectsUnknownLabelsAndOutOfRangeFractions)
{
  const PlannerConfig config = IntervalConfig();
  EXPECT_THROW(
    NormalizeRoadIntervals(config, {}, {{"UNKNOWN", 0.0, 1.0}}),
    std::runtime_error);
  EXPECT_THROW(
    NormalizeRoadIntervals(config, {}, {{"ROAD", -0.1, 0.5}}),
    std::runtime_error);
}

}  // namespace
}  // namespace arena_path_planner
