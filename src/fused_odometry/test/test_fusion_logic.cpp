#include <cmath>

#include "gtest/gtest.h"
#include "fused_odometry/fusion_logic.hpp"

using fused_odometry::FusionMode;
using fused_odometry::Pose2d;

TEST(Pose2dTransform, MapOdomCompositionRecoversGlobalBodyPose)
{
  const Pose2d map_from_body{5.0, 2.0, fused_odometry::kPi / 2.0};
  const Pose2d odom_from_body{1.0, 0.0, 0.0};
  const Pose2d map_from_odom = fused_odometry::compose_pose(
    map_from_body, fused_odometry::inverse_pose(odom_from_body));
  const Pose2d recovered = fused_odometry::compose_pose(map_from_odom, odom_from_body);
  EXPECT_NEAR(recovered.x, map_from_body.x, 1e-9);
  EXPECT_NEAR(recovered.y, map_from_body.y, 1e-9);
  EXPECT_NEAR(recovered.yaw, map_from_body.yaw, 1e-9);
}

TEST(Pose2dTransform, InterpolationUsesShortestWrappedYaw)
{
  const Pose2d halfway = fused_odometry::interpolate_pose(
    Pose2d{0.0, 0.0, 3.0}, Pose2d{2.0, 4.0, -3.0}, 0.5);
  EXPECT_NEAR(halfway.x, 1.0, 1e-9);
  EXPECT_NEAR(halfway.y, 2.0, 1e-9);
  EXPECT_NEAR(std::abs(halfway.yaw), fused_odometry::kPi, 1e-9);
}

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

TEST(TranslationAligner, RemovesTurnTranslationButKeepsYawAndFutureIncrements)
{
  fused_odometry::TranslationAligner aligner;
  const Pose2d turn_end_raw{2.08, 0.03, fused_odometry::kPi / 2.0};
  const Pose2d turn_anchor{2.0, 0.0, fused_odometry::kPi / 2.0};
  aligner.align_to(turn_end_raw, turn_anchor);

  const Pose2d aligned_end = aligner.apply(turn_end_raw);
  EXPECT_NEAR(aligned_end.x, 2.0, 1e-9);
  EXPECT_NEAR(aligned_end.y, 0.0, 1e-9);
  EXPECT_NEAR(aligned_end.yaw, fused_odometry::kPi / 2.0, 1e-9);

  const Pose2d next = aligner.apply(Pose2d{2.08, 0.53, fused_odometry::kPi / 2.0});
  EXPECT_NEAR(next.x, 2.0, 1e-9);
  EXPECT_NEAR(next.y, 0.50, 1e-9);
  EXPECT_NEAR(next.yaw, fused_odometry::kPi / 2.0, 1e-9);
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

TEST(WheelVisualConsistency, RejectsWheelMotionWhenRawVisionIsStationary)
{
  EXPECT_TRUE(std::isinf(fused_odometry::wheel_visual_rejection_residual(
      0.20, 0.01, 0.025, 0.03)));
  EXPECT_TRUE(std::isinf(fused_odometry::wheel_visual_rejection_residual(
      0.04, 0.00, 0.025, 0.03)));
  EXPECT_NEAR(fused_odometry::wheel_visual_rejection_residual(
      0.20, 0.18, 0.025, 0.03), 0.02, 1e-9);
  EXPECT_NEAR(fused_odometry::wheel_visual_rejection_residual(
      0.02, 0.00, 0.025, 0.03), 0.02, 1e-9);
}

TEST(GlobalCorrection, VisualPoseCorrectsIndependentLocalDrift)
{
  const Pose2d visual_global{0.0, 0.0, 0.0};
  const Pose2d drifted_local{0.25, -0.10, 0.20};
  const Pose2d map_from_odom = fused_odometry::compose_pose(
    visual_global, fused_odometry::inverse_pose(drifted_local));
  const Pose2d corrected = fused_odometry::compose_pose(map_from_odom, drifted_local);

  EXPECT_NEAR(corrected.x, visual_global.x, 1e-9);
  EXPECT_NEAR(corrected.y, visual_global.y, 1e-9);
  EXPECT_NEAR(corrected.yaw, visual_global.yaw, 1e-9);
}

TEST(GlobalCorrection, StationaryCommandRequiresFreshBoundedLinearAndAngularSpeed)
{
  EXPECT_TRUE(fused_odometry::motion_command_is_stationary(
      0.0, 0.0, true, 0.03, 0.12));
  EXPECT_FALSE(fused_odometry::motion_command_is_stationary(
      0.04, 0.0, true, 0.03, 0.12));
  EXPECT_FALSE(fused_odometry::motion_command_is_stationary(
      0.0, 0.13, true, 0.03, 0.12));
  EXPECT_FALSE(fused_odometry::motion_command_is_stationary(
      0.0, 0.0, false, 0.03, 0.12));
}

TEST(DisagreementCovariance, IsBoundedAndNeverRejectsThePrimaryRateSource)
{
  EXPECT_DOUBLE_EQ(fused_odometry::disagreement_covariance_scale(0.0, 0.2, 0.8), 1.0);
  EXPECT_NEAR(fused_odometry::disagreement_covariance_scale(0.4, 0.2, 0.8), 5.0, 1e-9);
  EXPECT_NEAR(fused_odometry::disagreement_covariance_scale(2.0, 0.2, 0.8), 17.0, 1e-9);
  EXPECT_DOUBLE_EQ(fused_odometry::disagreement_covariance_scale(NAN, 0.2, 0.8), 100.0);
}

TEST(WheelYawValidation, RequiresMotionAndInstantaneousAgreement)
{
  EXPECT_FALSE(fused_odometry::yaw_rates_excited(0.0, 0.0, 0.08));
  EXPECT_FALSE(fused_odometry::yaw_rates_excited(0.20, 0.01, 0.08));
  EXPECT_TRUE(fused_odometry::yaw_rates_excited(0.20, 0.18, 0.08));
  EXPECT_TRUE(fused_odometry::yaw_rates_excited(0.20, -0.18, 0.08));
  EXPECT_TRUE(fused_odometry::yaw_rates_consistent(0.20, 0.18, 0.25));
  EXPECT_FALSE(fused_odometry::yaw_rates_consistent(0.20, -0.18, 0.25));
  EXPECT_FALSE(fused_odometry::yaw_rates_consistent(0.60, 0.18, 0.25));
  EXPECT_FALSE(fused_odometry::yaw_rates_consistent(NAN, 0.18, 0.25));
}

TEST(WheelVelocityTurnWeight, PrefersFreshImuAndFallsBackToCommand)
{
  EXPECT_DOUBLE_EQ(
    fused_odometry::wheel_vx_turn_covariance_scale(
      0.10, 0.10, true, true, 0.15, 0.60, 100.0),
    1.0);
  EXPECT_NEAR(
    fused_odometry::wheel_vx_turn_covariance_scale(
      0.375, 0.20, true, true, 0.15, 0.60, 100.0),
    25.75, 1e-9);
  EXPECT_NEAR(
    fused_odometry::wheel_vx_turn_covariance_scale(
      0.20, 0.90, true, true, 0.15, 0.60, 100.0),
    1.0 + (0.05 / 0.45) * (0.05 / 0.45) * 99.0, 1e-9);
  EXPECT_DOUBLE_EQ(
    fused_odometry::wheel_vx_turn_covariance_scale(
      0.20, 0.90, false, true, 0.15, 0.60, 100.0),
    100.0);
  EXPECT_DOUBLE_EQ(
    fused_odometry::wheel_vx_turn_covariance_scale(
      0.90, 0.90, false, false, 0.15, 0.60, 100.0),
    1.0);
}

TEST(WheelVelocityTurnWeight, ZeroesOnlyFreshInPlaceTurnCommands)
{
  EXPECT_TRUE(fused_odometry::zero_wheel_vx_during_in_place_turn(
      0.0, 0.90, true, 0.10, 0.30));
  EXPECT_TRUE(fused_odometry::zero_wheel_vx_during_in_place_turn(
      -0.08, -0.30, true, 0.10, 0.30));
  EXPECT_FALSE(fused_odometry::zero_wheel_vx_during_in_place_turn(
      0.20, 0.90, true, 0.10, 0.30));
  EXPECT_FALSE(fused_odometry::zero_wheel_vx_during_in_place_turn(
      0.0, 0.90, false, 0.10, 0.30));
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
    FusionMode::kFull);
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, true, true, false, false, false, false, 0.0, 0.0, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kNoImu);
  EXPECT_EQ(
    fused_odometry::select_mode(
      true, true, false, false, false, false, false, 0.0, 0.0, 2.0, 0.3, 0.75, 0.1),
    FusionMode::kNoImu);
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

TEST(FusionHealthSeverity, KeepsWheelDiagnosticsVisibleWithoutDegradingFusionMode)
{
  using fused_odometry::HealthSeverity;
  using fused_odometry::MotionFault;

  EXPECT_EQ(
    fused_odometry::health_severity(FusionMode::kFull, true, MotionFault::kNone),
    HealthSeverity::kOk);
  EXPECT_EQ(
    fused_odometry::health_severity(FusionMode::kFull, false, MotionFault::kNone),
    HealthSeverity::kWarning);
  EXPECT_EQ(
    fused_odometry::health_severity(FusionMode::kFull, false, MotionFault::kSlip),
    HealthSeverity::kWarning);
  EXPECT_EQ(
    fused_odometry::health_severity(
      FusionMode::kFull, false, MotionFault::kEncoderFailure),
    HealthSeverity::kWarning);
  EXPECT_EQ(
    fused_odometry::health_severity(FusionMode::kNoImu, true, MotionFault::kNone),
    HealthSeverity::kWarning);
  EXPECT_EQ(
    fused_odometry::health_severity(
      FusionMode::kFaultStalled, false, MotionFault::kStalled),
    HealthSeverity::kError);
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

TEST(YawBiasEstimator, LearnsOnlyWhenEnabledAndClampsBias)
{
  fused_odometry::YawBiasEstimator estimator(1.0, 0.05);
  EXPECT_NEAR(estimator.correct(0.0, 0.10, 0.0, true, false), 0.10, 1e-9);
  for (int index = 1; index <= 20; ++index) {
    estimator.correct(0.1 * index, 0.10, 0.0, true, true);
  }
  EXPECT_NEAR(estimator.bias(), 0.05, 1e-6);
  EXPECT_NEAR(estimator.correct(2.1, 0.10, 0.0, true, false), 0.05, 1e-6);
}

TEST(YawBiasEstimator, InvalidReferenceAndTimeDoNotCreateJumps)
{
  fused_odometry::YawBiasEstimator estimator(2.0, 0.08);
  estimator.correct(1.0, 0.02, 0.0, true, true);
  const double before = estimator.bias();
  EXPECT_NEAR(estimator.correct(0.5, 0.02, NAN, true, true), 0.02 - before, 1e-9);
  EXPECT_DOUBLE_EQ(estimator.bias(), before);
  EXPECT_NEAR(estimator.correct(1.6, 0.02, 0.0, true, true), 0.02 - before, 1e-9);
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
