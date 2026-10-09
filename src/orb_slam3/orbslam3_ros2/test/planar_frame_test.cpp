#include "planar_frame.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>

namespace
{

Sophus::SE3f Pose(float x, float y, float yaw)
{
    return Sophus::SE3f(
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix(),
        Eigen::Vector3f(x, y, 0.0f));
}

float Yaw(const Sophus::SE3f & pose)
{
    const Eigen::Matrix3f rotation = pose.rotationMatrix();
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

TEST(PlanarFrameTest, ConvertsOrbLateralAndYawSignsToRos)
{
    const Sophus::SE3f converted =
        orbslam3_ros2::ToRosPlanarFrame(Pose(0.20f, 0.50f, 1.5707963f));

    EXPECT_NEAR(converted.translation().x(), 0.20f, 1.0e-5f);
    EXPECT_NEAR(converted.translation().y(), -0.50f, 1.0e-5f);
    EXPECT_NEAR(converted.translation().z(), 0.0f, 1.0e-5f);
    EXPECT_NEAR(Yaw(converted), -1.5707963f, 1.0e-5f);
}

TEST(PlanarFrameTest, KeepsForwardMotionAndIdentityUnchanged)
{
    const Sophus::SE3f identity =
        orbslam3_ros2::ToRosPlanarFrame(Sophus::SE3f());
    const Sophus::SE3f forward =
        orbslam3_ros2::ToRosPlanarFrame(Pose(0.75f, 0.0f, 0.0f));

    EXPECT_TRUE(identity.matrix().isApprox(Sophus::SE3f().matrix(), 1.0e-5f));
    EXPECT_NEAR(forward.translation().x(), 0.75f, 1.0e-5f);
    EXPECT_NEAR(forward.translation().y(), 0.0f, 1.0e-5f);
    EXPECT_NEAR(Yaw(forward), 0.0f, 1.0e-5f);
}

}  // namespace
