#include "navigation_safety/checker.hpp"

#include <gtest/gtest.h>

namespace navigation_safety {
namespace {

const std::vector<Rectangle> kRoad{{-0.5, -0.1, 0.5, 0.1}};
const Footprint kVehicle{0.132, 0.132, 0.030, 0.0, 0.002};

TEST(NavigationSafety, ChecksOffsetFootprintInsideRoad) {
  EXPECT_TRUE(pose_is_safe({{0.0, 0.0}, 0.0}, kVehicle, kRoad, {}, {}));
  Report report;
  EXPECT_FALSE(
      pose_is_safe({{0.0, 0.04}, 0.0}, kVehicle, kRoad, {}, {}, &report));
  EXPECT_EQ(report.failure, Failure::OUTSIDE_FREE_SPACE);
}

TEST(NavigationSafety, RejectsFootprintIntersectingObstacle) {
  const std::vector<Polygon> obstacles{
      rectangle_polygon({0.095, -0.05, 0.20, 0.05})};
  Report report;
  EXPECT_FALSE(pose_is_safe({{0.0, 0.0}, 0.0}, kVehicle, kRoad, obstacles,
                            {}, &report));
  EXPECT_EQ(report.failure, Failure::INTERSECTS_OBSTACLE);
  EXPECT_GT(report.overlap, 0.0);
}

TEST(NavigationSafety, ChecksTheWholeSweptPath) {
  const std::vector<Polygon> obstacles{
      rectangle_polygon({-0.015, -0.02, 0.015, 0.02})};
  const std::vector<Pose> poses{{{-0.3, 0.0}, 0.0}, {{0.3, 0.0}, 0.0}};
  Report report;
  EXPECT_FALSE(sweep_is_safe(poses, {0.05, 0.05, 0.0, 0.0, 0.0},
                             {{-0.5, -0.2, 0.5, 0.2}}, obstacles, {},
                             0.01, 0.04, &report));
  EXPECT_EQ(report.failure, Failure::INTERSECTS_OBSTACLE);
}

TEST(NavigationSafety, RejectsBoundaryTouchConsistently) {
  Report report;
  EXPECT_FALSE(pose_is_safe({{0.0, 0.0}, 0.0}, {0.10, 0.10, 0.0, 0.0, 0.0},
                            {{-0.049, -0.5, 0.5, 0.5}}, {}, {}, &report));
  EXPECT_EQ(report.failure, Failure::OUTSIDE_FREE_SPACE);
}

TEST(NavigationSafety, TreatsObstacleTangencyAsClearanceBoundary) {
  const std::vector<Polygon> obstacles{
      rectangle_polygon({0.05, -0.05, 0.20, 0.05})};
  Report report;
  EXPECT_TRUE(pose_is_safe({{0.0, 0.0}, 0.0}, {0.10, 0.10, 0.0, 0.0, 0.0},
                           {{-0.5, -0.2, 0.5, 0.2}}, obstacles, {}, &report));
  EXPECT_EQ(report.failure, Failure::NONE);
}

} // namespace
} // namespace navigation_safety
