#include "pose_velocity_estimator.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

namespace
{

Sophus::SE3f Pose(float x, float y, float yaw)
{
    return Sophus::SE3f(
        Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix(),
        Eigen::Vector3f(x, y, 0.0f));
}

TEST(PoseVelocityEstimatorTest, FirstObservationHasNoVelocity)
{
    orbslam3_ros2::PoseVelocityEstimator estimator;

    const orbslam3_ros2::PoseVelocity velocity = estimator.Observe(Pose(2.0f, 3.0f, 0.4f), 10.0);

    EXPECT_FALSE(velocity.valid);
    EXPECT_TRUE(velocity.linear.isZero());
    EXPECT_TRUE(velocity.angular.isZero());
}

TEST(PoseVelocityEstimatorTest, ComputesVelocityWithinContinuousTrackingSegment)
{
    orbslam3_ros2::PoseVelocityEstimator estimator;
    estimator.Observe(Pose(0.0f, 0.0f, 0.0f), 10.0);

    const orbslam3_ros2::PoseVelocity velocity = estimator.Observe(Pose(1.0f, 0.4f, 0.2f), 10.5);

    ASSERT_TRUE(velocity.valid);
    EXPECT_TRUE(velocity.linear.isApprox(Eigen::Vector3f(2.0f, 0.8f, 0.0f), 1e-5f));
    EXPECT_TRUE(velocity.angular.isApprox(Eigen::Vector3f(0.0f, 0.0f, 0.4f), 1e-5f));
}

TEST(PoseVelocityEstimatorTest, DoesNotDifferentiateAcrossTrackingInterruption)
{
    orbslam3_ros2::PoseVelocityEstimator estimator;
    estimator.Observe(Pose(1.0f, 2.0f, 0.2f), 10.0);
    estimator.Observe(Pose(1.1f, 2.0f, 0.2f), 10.1);

    estimator.Invalidate();
    const orbslam3_ros2::PoseVelocity recovery = estimator.Observe(Pose(-50.0f, 80.0f, -2.0f), 12.0);

    EXPECT_FALSE(recovery.valid);
    EXPECT_TRUE(recovery.linear.isZero());
    EXPECT_TRUE(recovery.angular.isZero());

    const orbslam3_ros2::PoseVelocity next = estimator.Observe(Pose(-49.9f, 80.0f, -2.0f), 12.1);
    EXPECT_TRUE(next.valid);
}

TEST(PoseVelocityEstimatorTest, RejectsNonMonotonicTimestamp)
{
    orbslam3_ros2::PoseVelocityEstimator estimator;
    estimator.Observe(Pose(0.0f, 0.0f, 0.0f), 10.0);

    EXPECT_FALSE(estimator.Observe(Pose(1.0f, 0.0f, 0.0f), 10.0).valid);
}

}  // namespace
