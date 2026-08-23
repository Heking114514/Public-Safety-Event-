#include <fstream>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "visual_navigation/route_manager.hpp"

TEST(RouteManager, LoadsCommentsHeadersAndDefaults)
{
  const std::string file = "/tmp/route_manager_test.csv";
  {
    std::ofstream output(file);
    output << "x,y,yaw,speed\n# comment\n1.0, 2.0\n";
  }
  std::vector<visual_navigation::RouteWaypoint> route;
  std::string error;
  ASSERT_TRUE(visual_navigation::RouteManager::LoadCsv(file, 0.4, 0.05, route, error)) << error;
  ASSERT_EQ(route.size(), 1u);
  EXPECT_DOUBLE_EQ(route[0].x, 1.0);
  EXPECT_DOUBLE_EQ(route[0].y, 2.0);
  EXPECT_DOUBLE_EQ(route[0].speed, 0.4);
  EXPECT_DOUBLE_EQ(route[0].tolerance, 0.05);
}

TEST(RouteManager, ComparesOnlyRouteGeometryAndYaw)
{
  std::vector<visual_navigation::RouteWaypoint> left(1), right(1);
  left[0].x = right[0].x = 1.0;
  left[0].y = right[0].y = 2.0;
  left[0].yaw = 3.14;
  right[0].yaw = -3.143185307179586;
  left[0].speed = 0.1;
  right[0].speed = 0.5;
  EXPECT_TRUE(visual_navigation::RouteManager::Equivalent(left, right));
  right[0].x += 0.01;
  EXPECT_FALSE(visual_navigation::RouteManager::Equivalent(left, right));
}
