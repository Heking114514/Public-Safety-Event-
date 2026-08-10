#include <cmath>

#include "gtest/gtest.h"
#include "fused_odometry/fusion_logic.hpp"

using fused_odometry::FusionMode;
using fused_odometry::Pose2d;

TEST(PoseAligner, RecoveryPoseMatchesPredictionAndKeepsIncrements)
{
  fused_odometry::PoseAligner aligner;
  aligner.align_to(Pose2d{1.0, 2.0, 0.0}, Pose2d{4.0, 5.0, fused_odometry::kPi / 2.0});
  const Pose2d recovery = aligner.apply(Pose2d{1.0, 2.0, 0.0});
  EXPECT_NEAR(recovery.x, 4.0, 1e-9);
  EXPECT_NEAR(recovery.y, 5.0, 1e-9);
  EXPECT_NEAR(recovery.yaw, fused_odometry::kPi / 2.0, 1e-9);

  const Pose2d next = aligner.apply(Pose2d{2.0, 2.0, 0.0});
  EXPECT_NEAR(next.x, 4.0, 1e-9);
  EXPECT_NEAR(next.y, 6.0, 1e-9);
}

TEST(PoseResidual, GatesPositionAndWrappedYawWithoutChangingCoordinates)
{
  const Pose2d reference{1.0, 2.0, fused_odometry::kPi - 0.05};
  EXPECT_TRUE(fused_odometry::pose_residual_within(
    Pose2d{1.2, 2.1, -fused_odometry::kPi + 0.05}, reference, 0.3, 0.2));
  EXPECT_FALSE(fused_odometry::pose_residual_within(
    Pose2d{1.5, 2.0, reference.yaw}, reference, 0.3, 0.2));
  EXPECT_FALSE(fused_odometry::pose_residual_within(
    Pose2d{1.0, 2.0, reference.yaw - 0.3}, reference, 0.3, 0.2));
}

TEST(DisagreementCovariance, IsBoundedAndNeverRejectsThePrimaryRateSource)
{
  EXPECT_DOUBLE_EQ(fused_odometry::disagreement_covariance_scale(0.0, 0.2, 0.8), 1.0);
  EXPECT_NEAR(fused_odometry::disagreement_covariance_scale(0.4, 0.2, 0.8), 5.0, 1e-9);
  EXPECT_NEAR(fused_odometry::disagreement_covariance_scale(2.0, 0.2, 0.8), 17.0, 1e-9);
  EXPECT_DOUBLE_EQ(fused_odometry::disagreement_covariance_scale(NAN, 0.2, 0.8), 100.0);
}

TEST(ResidualGate, RejectsAndRecoversWithHysteresis)
{
  fused_odometry::ResidualGate gate(1.0, 3, 2);
  EXPECT_TRUE(gate.update(2.0));
  EXPECT_TRUE(gate.update(2.0));
  EXPECT_FALSE(gate.update(2.0));
  EXPECT_FALSE(gate.update(0.1));
  EXPECT_TRUE(gate.update(0.1));
  EXPECT_GT(gate.covariance_scale(0.5, 0.25), 1.0);
}

TEST(FusionMode, ConservativeDegradation)
{
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, true, true, true, false, false, false, 0.0, 0.0, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kFull);
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, false, true, true, false, false, false, 1.0, 0.2, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kNoVision);
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, false, true, true, false, false, false, 2.1, 0.2, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kNoVision);
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, false, true, false, true, false, false, 0.1, 0.01, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kWheelOnly);
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, false, true, false, true, false, false, 2.1, 0.2, 2.0, 0.3, 2.0, 0.3),
    FusionMode::kFault);
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, true, false, true, false, false, false, 0.0, 0.0, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kNoWheel);
  EXPECT_EQ(
    fused_odometry::select_mode(
      false, false, true, true, true, false, false, 0.0, 0.0, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kFault);
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, true, true, true, false, true, false, 0.0, 0.0, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kVisualRealigned);
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, true, true, true, false, false, true, 0.0, 0.0, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kFaultStalled);
}

TEST(FusionMode, PublicNamesMatchNavigationPolicy)
{
  EXPECT_STREQ(
    fused_odometry::mode_name(FusionMode::kVisualRealigned),
    "DEGRADED_VISUAL_REALIGNED");
  EXPECT_STREQ(fused_odometry::mode_name(FusionMode::kFaultStalled), "FAULT_STALLED");
}

TEST(RobustWindow, MedianRejectsOutlierAndExpiresOldSamples)
{
  fused_odometry::RobustWindow window(0.75);
  window.add(0.0, 1.0);
  window.add(0.1, 1.1);
  window.add(0.2, 100.0);
  window.add(0.3, 0.9);
  window.add(0.4, 1.0);
  EXPECT_NEAR(window.median(), 1.0, 1e-9);
  EXPECT_NEAR(window.mad(), 0.1, 1e-9);
  window.add(1.0, 2.0);
  EXPECT_EQ(window.size(), 3U);
}

TEST(MotionClassifier, DetectsPersistentFaultsAndUsesRecoveryHysteresis)
{
  fused_odometry::MotionClassifier classifier({0.08, 0.04, 0.02, 0.5, 1.0});
  EXPECT_EQ(
    classifier.update(0.0, 0.2, 0.2, 0.0, true, true, true),
    fused_odometry::MotionFault::kNone);
  EXPECT_EQ(
    classifier.update(0.6, 0.2, 0.2, 0.0, true, true, true),
    fused_odometry::MotionFault::kSlip);
  EXPECT_EQ(
    classifier.update(1.0, 0.2, 0.2, 0.2, true, true, true),
    fused_odometry::MotionFault::kSlip);
  EXPECT_EQ(
    classifier.update(2.1, 0.2, 0.2, 0.2, true, true, true),
    fused_odometry::MotionFault::kNone);

  fused_odometry::MotionClassifier stalled({0.08, 0.04, 0.02, 0.5, 1.0});
  stalled.update(0.0, 0.2, 0.0, 0.0, true, true, true);
  EXPECT_EQ(
    stalled.update(0.6, 0.2, 0.0, 0.0, true, true, true),
    fused_odometry::MotionFault::kStalled);

  fused_odometry::MotionClassifier encoder({0.08, 0.04, 0.02, 0.5, 1.0});
  encoder.update(0.0, 0.2, 0.0, 0.2, true, true, true);
  EXPECT_EQ(
    encoder.update(0.6, 0.2, 0.0, 0.2, true, true, true),
    fused_odometry::MotionFault::kEncoderFailure);
}

TEST(AngularStallDetector, DetectsCommandedTurnWithoutMeasuredRotation)
{
  fused_odometry::AngularStallDetector detector({0.3, 0.1, 0.8, 1.0});
  EXPECT_FALSE(detector.update(0.0, 0.6, 0.0, true, true));
  EXPECT_FALSE(detector.update(0.7, 0.6, 0.0, true, true));
  EXPECT_TRUE(detector.update(0.9, 0.6, 0.0, true, true));
  EXPECT_TRUE(detector.update(1.2, 0.0, 0.0, true, true));
  EXPECT_FALSE(detector.update(2.3, 0.0, 0.0, true, true));

  fused_odometry::AngularStallDetector moving({0.3, 0.1, 0.8, 1.0});
  EXPECT_FALSE(moving.update(0.0, 0.6, 0.4, true, true));
  EXPECT_FALSE(moving.update(1.0, 0.6, 0.4, true, true));
}
