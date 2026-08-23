#include "gtest/gtest.h"

#include "visual_navigation/navigation_input_cache.hpp"

TEST(NavigationInputCache, BuildsFreshSnapshotFromSharedReceiptState)
{
  visual_navigation::NavigationInputCache cache;
  const rclcpp::Time received(10, 0, RCL_ROS_TIME);
  cache.update_odometry(true, true, true, 0.25, received);
  cache.update_imu(received);
  cache.update_tracking(2);
  cache.update_fusion_status("FULL", received);
  cache.update_actuator(true, received);

  const auto snapshot = cache.snapshot(
    rclcpp::Time(10, 100000000, RCL_ROS_TIME), 0.5, 0.5, 0.5,
    true, true, true);
  EXPECT_TRUE(snapshot.odometry_fresh);
  EXPECT_TRUE(snapshot.fusion_status_fresh);
  EXPECT_TRUE(snapshot.actuator_status_fresh);
  EXPECT_EQ(snapshot.fusion_status, "FULL");
  EXPECT_DOUBLE_EQ(cache.odometry_velocity(), 0.25);
}

TEST(NavigationInputCache, StaleInputsAreReportedWithoutChangingPolicy)
{
  visual_navigation::NavigationInputCache cache;
  const rclcpp::Time received(10, 0, RCL_ROS_TIME);
  cache.update_odometry(true, true, false, 0.0, received);
  const auto snapshot = cache.snapshot(
    rclcpp::Time(11, 0, RCL_ROS_TIME), 0.5, 0.5, 0.5,
    true, false, false);
  EXPECT_FALSE(snapshot.odometry_fresh);
  EXPECT_FALSE(snapshot.fusion_status_received);
  EXPECT_FALSE(snapshot.actuator_status_received);
}
