#include "pose_continuity.hpp"

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

void ExpectPoseNear(const Sophus::SE3f &actual, const Sophus::SE3f &expected)
{
    EXPECT_TRUE(actual.translation().isApprox(expected.translation(), 1e-4f));
    EXPECT_TRUE(actual.rotationMatrix().isApprox(expected.rotationMatrix(), 1e-5f));
}

TEST(PoseContinuityTest, FirstPoseDefinesPublishedOrigin)
{
    orbslam3_ros2::PoseContinuity continuity;

    ExpectPoseNear(continuity.Align(Pose(4.0f, -2.0f, 0.7f)), Sophus::SE3f());
    EXPECT_FALSE(continuity.RecoveryPending());
}

TEST(FixedPoseOriginTest, RemovesOpticalWorldOriginAndKeepsBodyMotion)
{
    const Eigen::Matrix3f opticalWorldRotation =
        (Eigen::AngleAxisf(1.5707963f, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(0.8f, Eigen::Vector3f::UnitZ())).toRotationMatrix();
    const Sophus::SE3f sourceOrigin(
        opticalWorldRotation, Eigen::Vector3f(4.0f, -2.0f, 1.5f));
    const Sophus::SE3f bodyMotion = Pose(1.2f, -0.04f, 0.1f);

    orbslam3_ros2::FixedPoseOrigin origin;
    ExpectPoseNear(origin.Align(sourceOrigin), Sophus::SE3f());
    ExpectPoseNear(origin.Align(sourceOrigin * bodyMotion), bodyMotion);
}

TEST(FixedPoseOriginTest, DoesNotHideSourceFrameJump)
{
    orbslam3_ros2::FixedPoseOrigin origin;
    const Sophus::SE3f sourceOrigin = Pose(4.0f, -2.0f, 0.7f);
    origin.Align(sourceOrigin);

    const Sophus::SE3f jumpedSource = Pose(-20.0f, 30.0f, -1.2f);
    const Sophus::SE3f output = origin.Align(jumpedSource);
    EXPECT_FALSE(output.translation().isApprox(Eigen::Vector3f::Zero(), 1e-4f));
    EXPECT_FALSE(output.rotationMatrix().isApprox(Eigen::Matrix3f::Identity(), 1e-5f));
}

TEST(FixedPoseOriginTest, MatchesContinuousMapUntilTrackingIsInterrupted)
{
    orbslam3_ros2::FixedPoseOrigin rawOrigin;
    orbslam3_ros2::PoseContinuity continuity;
    const Sophus::SE3f sourceOrigin = Pose(4.0f, -2.0f, 0.7f);
    const Sophus::SE3f sourceMotion = sourceOrigin * Pose(1.2f, -0.04f, 0.1f);

    ExpectPoseNear(rawOrigin.Align(sourceOrigin), continuity.Align(sourceOrigin));
    ExpectPoseNear(rawOrigin.Align(sourceMotion), continuity.Align(sourceMotion));

    continuity.MarkTrackingInterrupted();
    const Sophus::SE3f jumpedSource = Pose(-20.0f, 30.0f, -1.2f);
    const Sophus::SE3f rawJump = rawOrigin.Align(jumpedSource);
    const Sophus::SE3f continuousRecovery = continuity.Align(jumpedSource);
    EXPECT_FALSE(rawJump.translation().isApprox(continuousRecovery.translation(), 1e-4f));
}

TEST(PoseContinuityTest, RecoveryStartsAtLastPublishedPose)
{
    orbslam3_ros2::PoseContinuity continuity;
    continuity.Align(Pose(4.0f, -2.0f, 0.7f));
    const Sophus::SE3f lastPublished = continuity.Align(Pose(5.0f, -1.5f, 0.9f));

    continuity.MarkTrackingInterrupted();
    EXPECT_TRUE(continuity.RecoveryPending());

    const Sophus::SE3f recoveredSource = Pose(-20.0f, 30.0f, -1.2f);
    ExpectPoseNear(continuity.Align(recoveredSource), lastPublished);
    EXPECT_FALSE(continuity.RecoveryPending());

    const Sophus::SE3f sourceMotion = Pose(0.4f, -0.1f, 0.2f);
    ExpectPoseNear(
        continuity.Align(recoveredSource * sourceMotion),
        lastPublished * sourceMotion);
}

TEST(PoseContinuityTest, RepeatedLossNotificationsKeepSameAnchor)
{
    orbslam3_ros2::PoseContinuity continuity;
    continuity.Align(Pose(1.0f, 2.0f, 0.3f));
    const Sophus::SE3f lastPublished = continuity.Align(Pose(1.5f, 2.2f, 0.4f));

    continuity.MarkTrackingInterrupted();
    continuity.MarkTrackingInterrupted();
    continuity.MarkTrackingInterrupted();

    ExpectPoseNear(continuity.Align(Pose(100.0f, -80.0f, 2.4f)), lastPublished);
}

TEST(PoseContinuityTest, InterruptionBeforeInitializationDoesNotCreateFalseRecovery)
{
    orbslam3_ros2::PoseContinuity continuity;
    continuity.MarkTrackingInterrupted();

    EXPECT_FALSE(continuity.RecoveryPending());
    ExpectPoseNear(continuity.Align(Pose(8.0f, 9.0f, -0.5f)), Sophus::SE3f());
}

}  // namespace
