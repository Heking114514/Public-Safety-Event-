#include "visual_navigation/odometry_input_validation.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace visual_navigation
{
namespace
{

TEST(OdometryInputValidation, RequiresBothConfiguredFrameNames)
{
  EXPECT_TRUE(OdometryFramePairMatches("map", "base_link", "map", "base_link"));
  EXPECT_FALSE(OdometryFramePairMatches("", "base_link", "map", "base_link"));
  EXPECT_FALSE(OdometryFramePairMatches("map", "", "map", "base_link"));
  EXPECT_FALSE(OdometryFramePairMatches("odom", "base_link", "map", "base_link"));
  EXPECT_FALSE(OdometryFramePairMatches("map", "base_link", "map", "camera_link"));
}

TEST(OdometryInputValidation, RejectsNonFinitePoseAndInvalidQuaternion)
{
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(OdometryPoseIsFinite(odometry));

  odometry.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(OdometryPoseIsFinite(odometry));
  odometry.pose.pose.position.x = 0.0;
  odometry.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(OdometryPoseIsFinite(odometry));

  // Z is not consumed by this planar controller, so an upstream source may
  // leave it unknown without stopping otherwise valid XY/yaw navigation.
  odometry.pose.pose.orientation.w = 1.0;
  odometry.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(OdometryPoseIsFinite(odometry));
}

TEST(OdometryInputValidation, OnlyLongitudinalVelocityIsRequired)
{
  nav_msgs::msg::Odometry odometry;
  odometry.twist.twist.linear.x = 0.1;
  odometry.twist.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(OdometryVelocityIsFinite(odometry));
  odometry.twist.twist.linear.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(OdometryVelocityIsFinite(odometry));
}

}  // namespace
}  // namespace visual_navigation
