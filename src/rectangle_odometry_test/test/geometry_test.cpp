#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "rectangle_odometry_test/geometry.hpp"

namespace rectangle = rectangle_odometry_test;

TEST(RectangleGeometry, BuildsClockwiseClosedRectangle)
{
  const auto route = rectangle::build_clockwise_rectangle({0.0, 0.0, 0.0}, 1.8);
  ASSERT_EQ(route.size(), 4U);
  EXPECT_NEAR(route[0].pose.x, 1.8, 1.0e-12);
  EXPECT_NEAR(route[0].pose.y, 0.0, 1.0e-12);
  EXPECT_TRUE(route[0].turn_junction);
  EXPECT_NEAR(route[1].pose.y, -1.8, 1.0e-12);
  EXPECT_NEAR(route[3].pose.x, 0.0, 1.0e-12);
  EXPECT_NEAR(route[3].pose.y, 0.0, 1.0e-12);
  EXPECT_FALSE(route[3].turn_junction);
}

TEST(RectangleGeometry, OriginHeadingRotatesRectangle)
{
  const auto route = rectangle::build_clockwise_rectangle(
    {2.0, 3.0, 0.5 * 3.14159265358979323846}, 1.8);
  EXPECT_NEAR(route[0].pose.x, 2.0, 1.0e-12);
  EXPECT_NEAR(route[0].pose.y, 4.8, 1.0e-12);
}

TEST(RectangleGeometry, RoundedRouteHasNoStopMarkers)
{
  const auto route = rectangle::build_clockwise_rounded_rectangle(
    {0.0, 0.0, 0.0}, 1.8, 0.25, 16, 0.05);
  EXPECT_EQ(route.size(), 168U);
  EXPECT_FALSE(std::any_of(route.begin(), route.end(), [](const auto & waypoint) {
    return waypoint.turn_junction;
  }));
  EXPECT_NEAR(route.back().pose.x, 0.0, 1.0e-12);
  EXPECT_NEAR(route.back().pose.y, 0.0, 1.0e-12);
}
