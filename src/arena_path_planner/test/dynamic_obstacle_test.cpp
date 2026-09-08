#include "arena_path_planner/dynamic_obstacle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace arena_path_planner
{
namespace
{

TEST(DynamicObstacle, ConvertsFinitePolygonToConservativeAabb)
{
  Rectangle rectangle;
  std::string reason;
  EXPECT_TRUE(DynamicObstacleToAabb(
    {{1.2, 2.4}, {0.8, 2.1}, {1.0, 2.8}}, 0.05, rectangle, &reason));
  EXPECT_TRUE(reason.empty());
  EXPECT_DOUBLE_EQ(rectangle.minimum_x, 0.8);
  EXPECT_DOUBLE_EQ(rectangle.minimum_y, 2.1);
  EXPECT_DOUBLE_EQ(rectangle.maximum_x, 1.2);
  EXPECT_DOUBLE_EQ(rectangle.maximum_y, 2.8);
}

TEST(DynamicObstacle, RetainsPointAndLineObservations)
{
  Rectangle point;
  Rectangle line;
  EXPECT_TRUE(DynamicObstacleToAabb({{1.0, 2.0}}, 0.05, point));
  EXPECT_TRUE(DynamicObstacleToAabb({{1.0, 1.0}, {1.0, 2.0}}, 0.05, line));

  EXPECT_NEAR(point.maximum_x - point.minimum_x, 0.05, 1.0e-12);
  EXPECT_NEAR(point.maximum_y - point.minimum_y, 0.05, 1.0e-12);
  EXPECT_NEAR(line.maximum_x - line.minimum_x, 0.05, 1.0e-12);
  EXPECT_DOUBLE_EQ(line.minimum_y, 1.0);
  EXPECT_DOUBLE_EQ(line.maximum_y, 2.0);
}

TEST(DynamicObstacle, ThinNonzeroPolygonOccupiesThePlanningGrid)
{
  Rectangle rectangle;
  ASSERT_TRUE(DynamicObstacleToAabb(
    {{0.500, 0.500}, {0.501, 0.501}}, 0.05, rectangle));
  EXPECT_NEAR(rectangle.maximum_x - rectangle.minimum_x, 0.05, 1.0e-12);
  EXPECT_NEAR(rectangle.maximum_y - rectangle.minimum_y, 0.05, 1.0e-12);

  PlannerConfig config;
  config.width = 1.0;
  config.height = 1.0;
  config.resolution = 0.05;
  config.inflation_radius = 0.0;
  config.preferred_clearance = 0.0;
  config.clearance_cost_weight = 0.0;
  config.free_regions = {{0.0, 0.0, 1.0, 1.0}};
  config.obstacles = {rectangle};
  config.default_start = {{0.20, 0.50}, 0.0};

  const ArenaPlanner planner(config, false);
  EXPECT_FALSE(planner.IsFree({0.50, 0.50}));
  EXPECT_FALSE(planner.SegmentIsFree({0.20, 0.50}, {0.80, 0.50}));
}

TEST(DynamicObstacle, RejectsEmptyAndNonFinitePolygons)
{
  Rectangle rectangle;
  std::string reason;
  EXPECT_FALSE(DynamicObstacleToAabb({}, 0.05, rectangle, &reason));
  EXPECT_NE(reason.find("empty"), std::string::npos);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(DynamicObstacleToAabb({{0.0, 0.0}, {nan, 1.0}}, 0.05,
                                     rectangle, &reason));
  EXPECT_NE(reason.find("non-finite"), std::string::npos);

  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(DynamicObstacleToAabb({{0.0, 0.0}, {infinity, 1.0}}, 0.05,
                                     rectangle));
}

}  // namespace
}  // namespace arena_path_planner
