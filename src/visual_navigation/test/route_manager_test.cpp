#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "visual_navigation/route_manager.hpp"
#include "gtest/gtest.h"

TEST(RouteManager, LoadsCommentsHeadersAndDefaults) {
  const std::string file = "/tmp/route_manager_test.csv";
  {
    std::ofstream output(file);
    output << "x,y,yaw,speed\n# comment\n1.0, 2.0\n";
  }
  std::vector<visual_navigation::RouteWaypoint> route;
  std::string error;
  ASSERT_TRUE(
      visual_navigation::RouteManager::LoadCsv(file, 0.4, 0.05, route, error))
      << error;
  ASSERT_EQ(route.size(), 1u);
  EXPECT_DOUBLE_EQ(route[0].x, 1.0);
  EXPECT_DOUBLE_EQ(route[0].y, 2.0);
  EXPECT_DOUBLE_EQ(route[0].speed, 0.4);
  EXPECT_DOUBLE_EQ(route[0].tolerance, 0.05);
}

TEST(RouteManager, ComparesOnlyRouteGeometryAndYaw) {
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

TEST(RouteManager, LoadsDynamicPathWithSharedValidation) {
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 1.0;
  pose.pose.position.y = 2.0;
  pose.pose.position.z = 0.001;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  std::vector<visual_navigation::RouteWaypoint> route;
  std::size_t invalid_index = 99;
  std::string error;
  ASSERT_TRUE(visual_navigation::RouteManager::LoadPath(path, 0.4, 0.05, route,
                                                        invalid_index, error))
      << error;
  ASSERT_EQ(route.size(), 1u);
  EXPECT_DOUBLE_EQ(route[0].yaw, 0.0);
  EXPECT_TRUE(route[0].turn_junction);
  EXPECT_EQ(invalid_index, 0u);
}

TEST(RouteManager, TurnJunctionMetadataChangesRouteIdentity) {
  std::vector<visual_navigation::RouteWaypoint> left(1), right(1);
  left[0].turn_junction = true;
  EXPECT_FALSE(visual_navigation::RouteManager::Equivalent(left, right));

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);
  std::size_t invalid_index = 99;
  std::string error;
  EXPECT_FALSE(visual_navigation::RouteManager::LoadPath(path, 0.4, 0.05, right,
                                                         invalid_index, error));
  EXPECT_EQ(invalid_index, 0u);

  pose.pose.position.z = 0.01;
  path.poses[0] = pose;
  EXPECT_FALSE(visual_navigation::RouteManager::LoadPath(path, 0.4, 0.05, right,
                                                         invalid_index, error));
}

TEST(RouteManager, TurnRecoveryRequiresANewRouteId) {
  constexpr uint64_t failed_route = 101U;

  EXPECT_FALSE(visual_navigation::RouteReplacementAllowsStart(failed_route));
  EXPECT_FALSE(visual_navigation::RouteReplacementAcceptsRoute(failed_route,
                                                               failed_route));
  EXPECT_TRUE(
      visual_navigation::RouteReplacementAcceptsRoute(failed_route, 102U));
  EXPECT_TRUE(visual_navigation::RouteReplacementAllowsStart(0U));
  EXPECT_TRUE(visual_navigation::RouteReplacementAcceptsRoute(0U, 101U));
  EXPECT_FALSE(visual_navigation::RouteReplacementAcceptsRoute(0U, 0U));
}

TEST(RouteManager, LoadsReverseArrivalTurnMarker) {
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.z = 0.002;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);
  std::vector<visual_navigation::RouteWaypoint> route;
  std::size_t invalid_index = 99;
  std::string error;

  ASSERT_TRUE(visual_navigation::RouteManager::LoadPath(path, 0.4, 0.05, route,
                                                        invalid_index, error))
      << error;
  ASSERT_EQ(route.size(), 1u);
  EXPECT_TRUE(route[0].turn_junction);
  EXPECT_TRUE(route[0].reverse_arrival_turn);
}
