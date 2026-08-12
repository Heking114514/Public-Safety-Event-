#include <gtest/gtest.h>

#include <limits>

#include "visual_navigation/path_control.hpp"

TEST(PathControl, TrackingPointRotationDoesNotMoveCorrectedBase)
{
  constexpr double offset_x = 0.087;
  constexpr double offset_y = 0.040;
  const auto initial = visual_navigation::BasePositionFromTrackingPoint(
    offset_x, offset_y, 0.0, 0.0, offset_x, offset_y);
  const auto rotated = visual_navigation::BasePositionFromTrackingPoint(
    -offset_y, offset_x, 3.14159265358979323846 / 2.0, 0.0,
    offset_x, offset_y);

  EXPECT_NEAR(initial.x, offset_x, 1e-12);
  EXPECT_NEAR(initial.y, offset_y, 1e-12);
  EXPECT_NEAR(rotated.x, offset_x, 1e-12);
  EXPECT_NEAR(rotated.y, offset_y, 1e-12);
}

TEST(PathControl, StanleyCorrectionIsBoundedAndSpeedAware)
{
  const double slow = visual_navigation::StanleyPathError(0.1, 0.5, 2.0, 0.1, 0.25, 0.7);
  const double fast = visual_navigation::StanleyPathError(0.1, 0.5, 2.0, 1.0, 0.25, 0.7);
  EXPECT_NEAR(slow, -0.6, 1e-9);
  EXPECT_GT(fast, slow);
  EXPECT_LT(fast, 0.1);
  EXPECT_NEAR(
    visual_navigation::StanleyPathError(0.1, -0.5, 2.0, 0.1, 0.25, 0.7),
    0.8, 1e-9);
}

TEST(PathControl, FinalPositionCaptureIsLatchedAcrossRotationDrift)
{
  EXPECT_FALSE(visual_navigation::FinalPositionCaptured(false, true, 0.081, 0.08));
  EXPECT_TRUE(visual_navigation::FinalPositionCaptured(false, true, 0.079, 0.08));
  EXPECT_TRUE(visual_navigation::FinalPositionCaptured(true, true, 0.12, 0.08));
  EXPECT_FALSE(visual_navigation::FinalPositionCaptured(false, false, 0.01, 0.08));
}

TEST(PathControl, FinalCompletionRechecksPositionAfterYawAlignment)
{
  EXPECT_TRUE(visual_navigation::FinalPositionCanComplete(true, 0.039, 0.04));
  EXPECT_FALSE(visual_navigation::FinalPositionCanComplete(true, 0.11, 0.04));
  EXPECT_FALSE(visual_navigation::FinalPositionCanComplete(false, 0.01, 0.04));
}

TEST(PathControl, CompletedWaypointBrakeDoesNotRestartAtFinalPoint)
{
  EXPECT_TRUE(visual_navigation::ShouldBeginWaypointBrake(false, false, true, true));
  EXPECT_FALSE(visual_navigation::ShouldBeginWaypointBrake(false, true, true, true));
  EXPECT_FALSE(visual_navigation::ShouldBeginWaypointBrake(true, false, true, true));
  EXPECT_FALSE(visual_navigation::ShouldBeginWaypointBrake(false, false, true, false));
}

TEST(PathControl, SegmentProjectionKeepsSignedRemainingDistance)
{
  const auto before = visual_navigation::ProjectOntoPathSegment(
    0.01, 0.61, 0.0, -0.02, -0.0401, -0.0143);
  EXPECT_TRUE(before.valid);
  EXPECT_NEAR(before.remaining, 0.00508, 1e-4);
  EXPECT_NEAR(before.cross_track, -0.04016, 1e-4);

  const auto passed = visual_navigation::ProjectOntoPathSegment(
    0.01, 0.61, 0.0, -0.02, -0.0420, -0.0254);
  EXPECT_TRUE(passed.valid);
  EXPECT_LT(passed.remaining, 0.0);
}

TEST(PathControl, PassCorridorCatchesTheRecordedEndpointNearMiss)
{
  const auto projection = visual_navigation::ProjectOntoPathSegment(
    0.01, 0.61, 0.0, -0.02, -0.0401, -0.0143);
  EXPECT_TRUE(visual_navigation::WaypointReached(
      0.04048, 0.04, projection, 0.01, 0.06));
  EXPECT_FALSE(visual_navigation::WaypointNeedsRecovery(
      projection, 0.01, 0.06));
}

TEST(PathControl, LargeLateralMissEntersRecoveryInsteadOfEscaping)
{
  const auto projection = visual_navigation::ProjectOntoPathSegment(
    0.0, 0.6, 0.0, 0.0, -0.09, -0.01);
  EXPECT_FALSE(visual_navigation::WaypointReached(
      std::hypot(0.09, 0.01), 0.04, projection, 0.01, 0.06));
  EXPECT_TRUE(visual_navigation::WaypointNeedsRecovery(
      projection, 0.01, 0.06));
}

TEST(PathControl, SignedApproachDistanceCannotGrowAfterEndpoint)
{
  visual_navigation::PathProjection before{0.03, 0.05, true};
  visual_navigation::PathProjection passed{-0.20, 0.05, true};
  EXPECT_DOUBLE_EQ(
    visual_navigation::EndpointApproachDistance(0.40, before), 0.03);
  EXPECT_DOUBLE_EQ(
    visual_navigation::EndpointApproachDistance(0.40, passed), 0.0);
}

TEST(PathControl, TurnPositionHoldProjectsAnchorErrorOntoBodyForwardAxis)
{
  EXPECT_NEAR(
    visual_navigation::TurnPositionHoldSpeed(
      0.0, 0.0, 0.0, 0.10, 0.0, 0.8, 0.015, 0.08),
    0.068, 1e-12);
  EXPECT_NEAR(
    visual_navigation::TurnPositionHoldSpeed(
      0.0, 0.0, 3.14159265358979323846 / 2.0,
      0.0, -0.10, 0.8, 0.015, 0.08),
    -0.068, 1e-12);
  EXPECT_DOUBLE_EQ(
    visual_navigation::TurnPositionHoldSpeed(
      0.0, 0.0, 0.0, 0.01, 1.0, 0.8, 0.015, 0.08),
    0.0);
}

TEST(PathControl, TurnPositionHoldIsBoundedAndRejectsInvalidInput)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::TurnPositionHoldSpeed(
      0.0, 0.0, 0.0, 1.0, 0.0, 2.0, 0.0, 0.08),
    0.08);
  EXPECT_DOUBLE_EQ(
    visual_navigation::TurnPositionHoldSpeed(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0,
      1.0, 0.0, 2.0, 0.0, 0.08),
    0.0);
}

TEST(PathControl, TurnRequiresContinuousLowRateBeforeSettling)
{
  EXPECT_FALSE(visual_navigation::TurnHasSettled(0.20, 1.0, 0.12, 0.30));
  EXPECT_FALSE(visual_navigation::TurnHasSettled(0.10, 0.29, 0.12, 0.30));
  EXPECT_TRUE(visual_navigation::TurnHasSettled(0.10, 0.30, 0.12, 0.30));
}

TEST(PathControl, CrossTrackErrorProgressivelyLimitsForwardSpeed)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::CrossTrackSpeedLimit(0.2, 0.02, 0.03, 0.06, 0.08), 0.2);
  EXPECT_NEAR(
    visual_navigation::CrossTrackSpeedLimit(0.2, 0.045, 0.03, 0.06, 0.08), 0.14, 1e-9);
  EXPECT_DOUBLE_EQ(
    visual_navigation::CrossTrackSpeedLimit(0.2, 0.08, 0.03, 0.06, 0.08), 0.08);
  EXPECT_DOUBLE_EQ(
    visual_navigation::CrossTrackSpeedLimit(0.05, 0.08, 0.03, 0.06, 0.08), 0.05);
}

TEST(PathControl, WaypointApproachDoesNotOverrideCrossTrackSpeedLimit)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::WaypointApproachSpeedLimit(0.08, 0.20, 0.8, 1.0), 0.08);
  EXPECT_DOUBLE_EQ(
    visual_navigation::WaypointApproachSpeedLimit(0.20, 0.20, 0.8, 0.05), 0.04);
}

TEST(PathControl, BrakingProfileUsesPhysicalSpeedAndControlDelay)
{
  const double stopping_distance = visual_navigation::BrakingDistance(
    0.5, 0.35, 0.12, 0.015);
  EXPECT_NEAR(stopping_distance, 0.43214, 1e-5);
  EXPECT_NEAR(
    visual_navigation::BrakingSpeedLimit(0.5, stopping_distance, 0.35, 0.12, 0.015),
    0.5, 1e-12);
  EXPECT_LT(
    visual_navigation::BrakingSpeedLimit(0.5, 0.20, 0.35, 0.12, 0.015), 0.33);
  EXPECT_DOUBLE_EQ(
    visual_navigation::BrakingSpeedLimit(0.5, 0.015, 0.35, 0.12, 0.015), 0.0);
}

TEST(PathControl, BrakingProfileRejectsInvalidInputs)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::BrakingDistance(
      std::numeric_limits<double>::quiet_NaN(), 0.35, 0.12, 0.015),
    0.0);
  EXPECT_DOUBLE_EQ(
    visual_navigation::BrakingSpeedLimit(0.5, 1.0, 0.0, 0.12, 0.015), 0.0);
  EXPECT_DOUBLE_EQ(
    visual_navigation::BrakingSpeedLimit(
      0.5, std::numeric_limits<double>::infinity(), 0.35, 0.12, 0.015),
    0.0);
}

TEST(PathControl, BrakingFeedbackRespondsToRecordedSpeedLag)
{
  // Latest bag: 0.175m remained while fused motion was still 0.570m/s.
  EXPECT_NEAR(
    visual_navigation::BrakingFeedbackSpeedLimit(
      0.5, 0.570, 0.175, 0.35, 0.20, 0.015, 1.0),
    0.15186, 1e-5);
  EXPECT_DOUBLE_EQ(
    visual_navigation::BrakingFeedbackSpeedLimit(
      0.4, 0.20, 1.0, 0.35, 0.20, 0.015, 1.0),
    0.4);
}

TEST(PathControl, BrakingFeedbackFailsClosedForInvalidSpeed)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::BrakingFeedbackSpeedLimit(
      0.4, std::numeric_limits<double>::quiet_NaN(),
      1.0, 0.35, 0.20, 0.015, 1.0),
    0.0);
}

TEST(PathControl, WaypointStopRequiresContinuousLowSpeedAndMinimumWait)
{
  EXPECT_FALSE(visual_navigation::WaypointStopSatisfied(
      0.02, 0.10, 0.19, 0.03, 0.10, 0.20, 0.60));
  EXPECT_FALSE(visual_navigation::WaypointStopSatisfied(
      0.04, 0.30, 0.30, 0.03, 0.10, 0.20, 0.60));
  EXPECT_TRUE(visual_navigation::WaypointStopSatisfied(
      0.02, 0.10, 0.20, 0.03, 0.10, 0.20, 0.60));
}

TEST(PathControl, WaypointStopTimeoutCannotDeadlockNavigation)
{
  EXPECT_FALSE(visual_navigation::WaypointStopSatisfied(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.59,
      0.03, 0.10, 0.20, 0.60));
  EXPECT_TRUE(visual_navigation::WaypointStopSatisfied(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.60,
      0.03, 0.10, 0.20, 0.60));
}

TEST(PathControl, PathTurnDeadbandBoostsOnlyAnUnexecutedCorrection)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::CompensatePathTurnDeadband(
      0.08, 0.05, 0.01, 0.025, 0.04, 0.14, 0.45),
    0.14);
  EXPECT_DOUBLE_EQ(
    visual_navigation::CompensatePathTurnDeadband(
      -0.08, -0.05, -0.01, 0.025, 0.04, 0.14, 0.45),
    -0.14);
  EXPECT_DOUBLE_EQ(
    visual_navigation::CompensatePathTurnDeadband(
      0.08, 0.05, 0.06, 0.025, 0.04, 0.14, 0.45),
    0.08);
  EXPECT_DOUBLE_EQ(
    visual_navigation::CompensatePathTurnDeadband(
      -0.03, 0.05, 0.08, 0.025, 0.04, 0.14, 0.45),
    -0.03);
  EXPECT_DOUBLE_EQ(
    visual_navigation::CompensatePathTurnDeadband(
      0.02, 0.01, 0.0, 0.025, 0.04, 0.14, 0.45),
    0.02);
}

TEST(PathControl, CompetitionSpeedScheduleAvoidsTheLowSpeedDrivetrainDeadZone)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::CrossTrackSpeedLimit(0.5, 0.015, 0.015, 0.06, 0.20), 0.5);
  EXPECT_NEAR(
    visual_navigation::CrossTrackSpeedLimit(0.5, 0.0375, 0.015, 0.06, 0.20),
    0.35, 1e-12);
  EXPECT_DOUBLE_EQ(
    visual_navigation::CrossTrackSpeedLimit(0.5, 0.08, 0.015, 0.06, 0.20), 0.20);
}

TEST(PathControl, RotateInPlaceUsesHysteresis)
{
  EXPECT_FALSE(visual_navigation::ShouldRotateInPlace(false, 0.29, 0.0, 0.30, 0.04, 0.12));
  EXPECT_TRUE(visual_navigation::ShouldRotateInPlace(false, 0.30, 0.0, 0.30, 0.04, 0.12));
  EXPECT_TRUE(visual_navigation::ShouldRotateInPlace(true, 0.05, 0.0, 0.30, 0.04, 0.12));
  EXPECT_FALSE(visual_navigation::ShouldRotateInPlace(true, 0.04, 0.12, 0.30, 0.04, 0.12));
}

TEST(PathControl, CompletedAlignmentRequiresLargeErrorToReenterRotation)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::RotationEntryThreshold(false, 0.18, 0.44), 0.18);
  EXPECT_DOUBLE_EQ(
    visual_navigation::RotationEntryThreshold(true, 0.18, 0.44), 0.44);
  EXPECT_DOUBLE_EQ(
    visual_navigation::RotationEntryThreshold(true, 0.18, 0.10), 0.18);
}

TEST(PathControl, TurnDoesNotFinishWhileBodyIsStillRotating)
{
  EXPECT_TRUE(visual_navigation::ShouldRotateInPlace(
      true, 0.02, 0.8, 0.18, 0.035, 0.12));
  EXPECT_FALSE(visual_navigation::ShouldRotateInPlace(
      true, 0.02, 0.1, 0.18, 0.035, 0.12));
}

TEST(PathControl, CrossTrackCancellationCannotFinishHeadingAlignment)
{
  const double heading_error = 0.467;
  const double cross_track_error = 0.113;
  const double path_error = visual_navigation::StanleyPathError(
    heading_error, cross_track_error, 2.0, 0.2, 0.25, 0.7);

  EXPECT_NEAR(path_error, 0.002, 0.01);
  EXPECT_TRUE(visual_navigation::ShouldRotateInPlace(
      true, std::abs(heading_error), 0.0, 0.18, 0.035, 0.12));
}

TEST(PathControl, TurnSpeedDropsInsidePrecisionZone)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::SelectMinimumTurnSpeed(0.13, 0.12, 0.45, 0.85), 0.85);
  EXPECT_DOUBLE_EQ(
    visual_navigation::SelectMinimumTurnSpeed(0.12, 0.12, 0.45, 0.85), 0.45);
  EXPECT_DOUBLE_EQ(
    visual_navigation::SelectMinimumTurnSpeed(0.04, 0.12, 1.0, 0.85), 0.85);
  EXPECT_DOUBLE_EQ(
    visual_navigation::SelectMinimumTurnSpeed(
      std::numeric_limits<double>::quiet_NaN(), 0.12, 0.45, 0.85), 0.85);
}

TEST(PathControl, InvalidErrorKeepsRobotInRotateOnlyMode)
{
  EXPECT_TRUE(visual_navigation::ShouldRotateInPlace(
      false, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.30, 0.10, 0.12));
}

TEST(PathControl, MinimumTurnSpeedOvercomesDrivetrainDeadZone)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::EnforceMinimumTurnSpeed(0.2, 0.3, 0.85, 1.2), 0.85);
  EXPECT_DOUBLE_EQ(
    visual_navigation::EnforceMinimumTurnSpeed(-0.2, -0.3, 0.85, 1.2), -0.85);
  EXPECT_DOUBLE_EQ(
    visual_navigation::EnforceMinimumTurnSpeed(1.5, 0.3, 0.85, 1.2), 1.2);
  EXPECT_DOUBLE_EQ(
    visual_navigation::EnforceMinimumTurnSpeed(0.2, 0.0, 0.85, 1.2), 0.2);
}
