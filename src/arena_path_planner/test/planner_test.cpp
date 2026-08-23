#include "arena_path_planner/planner.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

namespace arena_path_planner
{
namespace
{

std::string ConfigPath()
{
  return ament_index_cpp::get_package_share_directory("arena_path_planner") +
         "/config/arena_map.yaml";
}

std::string SyntheticConfigPath()
{
  return ament_index_cpp::get_package_share_directory("arena_path_planner") +
         "/config/arena_map_synthetic.yaml";
}

double RouteDistance(const Point & point, const std::vector<Point> & route)
{
  double distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 1; index < route.size(); ++index) {
    distance = std::min(distance, DistanceToSegment(point, route[index - 1], route[index]));
  }
  return distance;
}

TEST(ArenaPlanner, LoadsDefaultConfiguration)
{
  const PlannerConfig config = ArenaPlanner::LoadConfig(ConfigPath());
  EXPECT_DOUBLE_EQ(config.width, 6.0);
  EXPECT_DOUBLE_EQ(config.height, 6.0);
  EXPECT_DOUBLE_EQ(config.resolution, 0.05);
  EXPECT_DOUBLE_EQ(config.vehicle_length, 0.217);
  EXPECT_DOUBLE_EQ(config.vehicle_width, 0.210);
  EXPECT_DOUBLE_EQ(config.safety_margin, 0.015);
  ASSERT_EQ(config.default_targets.size(), 8U);
  EXPECT_EQ(config.default_labels.front(), "1");
  EXPECT_EQ(config.default_labels.back(), "8");
  ASSERT_FALSE(config.staging_regions.empty());
  EXPECT_EQ(config.topics.service, "/arena_path_planner/plan");
  EXPECT_EQ(config.topics.arena_path, "/arena_path_planner/arena_path");
  EXPECT_EQ(config.topics.navigation_path, "/arena_path_planner/navigation_path");
  EXPECT_EQ(config.topics.occupancy_grid, "/arena_path_planner/map");
  EXPECT_EQ(config.topics.route_input, "/waypoint_navigation/route_input");
}

TEST(ArenaPlanner, LoadsConfiguredRosInterfaces)
{
  std::ifstream source(ConfigPath());
  ASSERT_TRUE(source.is_open());
  std::ostringstream contents;
  contents << source.rdbuf();
  std::string yaml = contents.str();
  const auto replace = [&yaml](const std::string & current, const std::string & replacement) {
      const std::size_t position = yaml.find(current);
      ASSERT_NE(position, std::string::npos);
      yaml.replace(position, current.size(), replacement);
    };
  replace("/arena_path_planner/plan", "/test/plan");
  replace("/arena_path_planner/arena_path", "/test/arena_path");
  replace("/arena_path_planner/navigation_path", "/test/navigation_path");
  replace("/arena_path_planner/map", "/test/map");
  replace("/waypoint_navigation/route_input", "/test/route_input");

  const std::string test_path = ::testing::TempDir() + "arena_map_topics_test.yaml";
  {
    std::ofstream output(test_path);
    ASSERT_TRUE(output.is_open());
    output << yaml;
  }
  const PlannerConfig config = ArenaPlanner::LoadConfig(test_path);
  std::remove(test_path.c_str());

  EXPECT_EQ(config.topics.service, "/test/plan");
  EXPECT_EQ(config.topics.arena_path, "/test/arena_path");
  EXPECT_EQ(config.topics.navigation_path, "/test/navigation_path");
  EXPECT_EQ(config.topics.occupancy_grid, "/test/map");
  EXPECT_EQ(config.topics.route_input, "/test/route_input");
}

TEST(ArenaPlanner, PlansClosedCollisionFreeDefaultRoute)
{
  const PlannerConfig config = ArenaPlanner::LoadConfig(ConfigPath());
  const ArenaPlanner planner(config);
  const PlanResult result = planner.Plan(
    config.default_start, config.default_targets, config.default_labels, "shortest");
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_GT(result.points.size(), 20U);
  EXPECT_NEAR(result.points.front().x, config.default_start.position.x, 1.0e-9);
  EXPECT_NEAR(result.points.front().y, config.default_start.position.y, 1.0e-9);
  EXPECT_NEAR(result.points.back().x, config.default_start.position.x, 1.0e-9);
  EXPECT_NEAR(result.points.back().y, config.default_start.position.y, 1.0e-9);
  for (std::size_t index = 1; index < result.points.size(); ++index) {
    EXPECT_TRUE(planner.SegmentIsFree(result.points[index - 1], result.points[index]));
  }
  for (const Point & target : config.default_targets) {
    EXPECT_LE(RouteDistance(target, result.points), config.task_tolerance + 1.0e-9);
  }
  for (const Point & tunnel : config.required_tunnel_points) {
    EXPECT_LE(RouteDistance(tunnel, result.points), config.task_tolerance + 1.0e-9);
  }
  for (const std::string & label : config.required_tunnel_labels) {
    EXPECT_NE(
      std::find(result.visit_order.begin(), result.visit_order.end(), label),
      result.visit_order.end());
  }
  for (const std::string & label : config.required_tunnel_labels) {
    EXPECT_NE(
      std::find(result.visit_order.begin(), result.visit_order.end(), label),
      result.visit_order.end());
  }
}

TEST(ArenaPlanner, SupportsNumberedAndDynamicStartPlans)
{
  const PlannerConfig config = ArenaPlanner::LoadConfig(ConfigPath());
  const ArenaPlanner planner(config);
  Pose start{{2.1, 0.75}, 0.0};
  const PlanResult result = planner.Plan(
    start, config.default_targets, config.default_labels, "numbered");
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_GE(result.visit_order.size(), 8U);
  EXPECT_EQ(result.visit_order.front(), "1");
  EXPECT_EQ(result.visit_order.back(), "S");
  EXPECT_NEAR(result.points.front().x, start.position.x, 1.0e-9);
  EXPECT_NEAR(result.points.back().x, start.position.x, 1.0e-9);
}

TEST(ArenaPlanner, RejectsOccupiedStart)
{
  const PlannerConfig config = ArenaPlanner::LoadConfig(ConfigPath());
  const ArenaPlanner planner(config);
  const Pose start{{0.5, 0.5}, 0.0};
  const PlanResult result = planner.Plan(
    start, config.default_targets, config.default_labels, "shortest");
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("start"), std::string::npos);
}

TEST(ArenaPlanner, DefersBlockedTargetsAndKeepsPlanningReachableOnes)
{
  PlannerConfig config;
  config.width = 1.0;
  config.height = 1.0;
  config.resolution = 0.025;
  config.free_regions.push_back({0.0, 0.0, 1.0, 1.0});
  config.inflation_radius = 0.0;
  config.preferred_clearance = 0.0;
  config.vehicle_length = 0.0;
  config.vehicle_width = 0.0;
  config.obstacles.push_back({0.45, 0.0, 0.55, 1.0});
  const ArenaPlanner planner(config);
  const PlanResult result = planner.Plan(
    {{0.20, 0.50}, 0.0}, {{0.30, 0.50}, {0.80, 0.50}}, {"near", "right"}, "shortest");
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.all_targets_reached);
  EXPECT_FALSE(result.deferred_targets.empty());
  EXPECT_NE(
    std::find(result.visit_order.begin(), result.visit_order.end(), "near"),
    result.visit_order.end());
  EXPECT_NE(
    std::find(result.deferred_targets.begin(), result.deferred_targets.end(), "right"),
    result.deferred_targets.end());
}

TEST(ArenaPlanner, RejectsSyntheticMapThatCannotFitVehicle)
{
  const PlannerConfig config = ArenaPlanner::LoadConfig(SyntheticConfigPath());
  const ArenaPlanner planner(config);
  const PlanResult result = planner.Plan(
    config.default_start, config.default_targets, config.default_labels, "shortest");
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("start"), std::string::npos);
}

TEST(ArenaPlanner, PartialPlanCannotBeActivated)
{
  PlanResult partial;
  partial.success = true;
  partial.all_targets_reached = false;
  partial.deferred_targets = {"blocked-target"};
  EXPECT_FALSE(ActivationAllowed(true, partial));
  EXPECT_FALSE(ActivationAllowed(false, partial));

  PlanResult complete;
  complete.success = true;
  complete.all_targets_reached = true;
  EXPECT_TRUE(ActivationAllowed(true, complete));
  complete.success = false;
  EXPECT_FALSE(ActivationAllowed(true, complete));
}

TEST(ArenaPlanner, OccupancyDimensionsMatchConfiguration)
{
  const PlannerConfig config = ArenaPlanner::LoadConfig(ConfigPath());
  const ArenaPlanner planner(config);
  const auto data = planner.OccupancyData();
  EXPECT_EQ(data.size(), static_cast<std::size_t>(
    std::ceil(config.width / config.resolution) *
    std::ceil(config.height / config.resolution)));
  EXPECT_GT(std::count(data.begin(), data.end(), int8_t{100}), 0);
  EXPECT_GT(std::count(data.begin(), data.end(), int8_t{0}), 0);
}

TEST(ArenaPlanner, FootprintInflationAndPoseValidationReserveTheCompleteChassis)
{
  PlannerConfig config;
  config.width = 1.0;
  config.height = 1.0;
  config.resolution = 0.025;
  config.free_regions.push_back({0.0, 0.0, 1.0, 1.0});
  config.vehicle_length = 0.217;
  config.vehicle_width = 0.210;
  config.safety_margin = 0.040;
  config.tracking_margin = 0.050;
  const ArenaPlanner planner(config);

  EXPECT_TRUE(planner.IsFree({0.50, 0.50}));
  // The centre is too close to the boundary for the 217 x 210 mm body to
  // rotate without collision, even though a point robot would fit there.
  EXPECT_FALSE(planner.IsFree({0.14, 0.50}));
  EXPECT_FALSE(planner.PoseIsFree({0.147, 0.50}, 0.0));
  EXPECT_GE(planner.config().inflation_radius, 0.148);
  EXPECT_GE(planner.config().preferred_clearance, 0.198);
}

TEST(ArenaPlanner, NarrowLaneAllowsAlignedChassisButRejectsAnInPlaceTurn)
{
  PlannerConfig config;
  config.width = 0.30;
  config.height = 1.0;
  config.resolution = 0.01;
  config.free_regions.push_back({0.0, 0.0, 0.30, 1.0});
  config.vehicle_length = 0.217;
  config.vehicle_width = 0.210;
  config.safety_margin = 0.015;
  const ArenaPlanner planner(config);

  EXPECT_TRUE(planner.IsFree({0.15, 0.50}));
  EXPECT_TRUE(planner.PoseIsFree({0.15, 0.50}, 1.5707963267948966));
  EXPECT_FALSE(planner.RotationIsFree(
      {0.15, 0.50}, 0.0, 1.5707963267948966));
}

TEST(ArenaPlanner, CoverageVisitsEveryConfiguredRoad)
{
  const PlannerConfig config = ArenaPlanner::LoadConfig(ConfigPath());
  const ArenaPlanner planner(config);
  const PlanResult result = planner.Plan(
    config.default_start, config.default_targets, config.default_labels, "coverage");
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(result.all_targets_reached);
  EXPECT_TRUE(result.deferred_targets.empty());
  EXPECT_EQ(result.visit_order.size(), config.inspection_edges.size());
  for (const InspectionEdge & edge : config.inspection_edges) {
    EXPECT_NE(
      std::find(result.visit_order.begin(), result.visit_order.end(), edge.label),
      result.visit_order.end()) << edge.label;
    EXPECT_LE(RouteDistance(config.inspection_nodes[edge.from], result.points),
      config.resolution * 1.5);
    EXPECT_LE(RouteDistance(config.inspection_nodes[edge.to], result.points),
      config.resolution * 1.5);
  }
  for (std::size_t index = 1; index < result.points.size(); ++index) {
    EXPECT_TRUE(planner.SegmentIsFree(result.points[index - 1], result.points[index]));
  }
}

TEST(ArenaPlanner, PlansIncrementalTaskTunnelAndGapFillLayers)
{
  const PlannerConfig config = ArenaPlanner::LoadConfig(ConfigPath());
  const ArenaPlanner planner(config);
  const PlanResult tasks = planner.Plan(
    config.default_start, config.default_targets, config.default_labels, "layer1");
  ASSERT_TRUE(tasks.success) << tasks.message;
  ASSERT_FALSE(tasks.points.empty());
  for (const std::string & label : tasks.visit_order) {
    EXPECT_TRUE(label == "S" || label.find("TUNNEL_") != 0);
  }

  const Pose tunnel_start{tasks.points.back(), tasks.headings.back()};
  const PlanResult tunnels = planner.Plan(
    tunnel_start, {}, {}, "layer2", tasks.covered_edges);
  ASSERT_TRUE(tunnels.success) << tunnels.message;
  ASSERT_FALSE(tunnels.points.empty());
  EXPECT_NEAR(tunnels.points.front().x, tasks.points.back().x, 1.0e-9);
  EXPECT_NEAR(tunnels.points.front().y, tasks.points.back().y, 1.0e-9);
  EXPECT_EQ(tunnels.visit_order.size(), config.tunnel_segment_labels.size());
  for (const std::string & label : config.tunnel_segment_labels) {
    EXPECT_NE(
      std::find(tunnels.visit_order.begin(), tunnels.visit_order.end(), label),
      tunnels.visit_order.end());
  }

  std::vector<std::string> covered = tasks.covered_edges;
  covered.insert(covered.end(), tunnels.covered_edges.begin(), tunnels.covered_edges.end());
  const Pose gap_start{tunnels.points.back(), tunnels.headings.back()};
  const PlanResult gaps = planner.Plan(gap_start, {}, {}, "layer3", covered);
  ASSERT_TRUE(gaps.success) << gaps.message;
  ASSERT_FALSE(gaps.points.empty());
  EXPECT_NEAR(gaps.points.front().x, tunnels.points.back().x, 1.0e-9);
  EXPECT_NEAR(gaps.points.front().y, tunnels.points.back().y, 1.0e-9);
  for (const std::string & label : gaps.visit_order) {
    EXPECT_EQ(label.find("TUNNEL_"), std::string::npos);
  }
  EXPECT_NEAR(gaps.points.back().x, config.default_start.position.x, 1.0e-9);
  EXPECT_NEAR(gaps.points.back().y, config.default_start.position.y, 1.0e-9);
  for (std::size_t index = 1; index < gaps.points.size(); ++index) {
    EXPECT_TRUE(planner.SegmentIsFree(gaps.points[index - 1], gaps.points[index]));
  }
}

}  // namespace
}  // namespace arena_path_planner
