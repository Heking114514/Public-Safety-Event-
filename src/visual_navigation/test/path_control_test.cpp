#include <gtest/gtest.h>

#include <limits>
#include <vector>

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

TEST(PathControl, PolylineLookaheadBlendsAcrossCorner)
{
  const std::vector<visual_navigation::PlanarPoint> path = {
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  const auto target = visual_navigation::ComputePolylineLookahead(
    path, 0.80, -0.02, 0.40);

  ASSERT_TRUE(target.valid);
  EXPECT_NEAR(target.projection.x, 0.80, 1e-12);
  EXPECT_NEAR(target.projection.y, 0.00, 1e-12);
  EXPECT_NEAR(target.point.x, 1.00, 1e-12);
  EXPECT_NEAR(target.point.y, 0.20, 1e-12);
  EXPECT_NEAR(target.cross_track_error, -0.02, 1e-12);
  EXPECT_NEAR(target.distance, 0.40, 1e-12);
}

TEST(PathControl, PolylineLookaheadClampsAtPathEnd)
{
  const std::vector<visual_navigation::PlanarPoint> path = {
    {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto target = visual_navigation::ComputePolylineLookahead(
    path, 0.20, 0.10, 5.0);

  ASSERT_TRUE(target.valid);
  EXPECT_NEAR(target.point.x, 2.0, 1e-12);
  EXPECT_NEAR(target.point.y, 0.0, 1e-12);
  EXPECT_NEAR(target.distance, 1.8, 1e-12);
}

TEST(PathControl, PolylineLookaheadRejectsInvalidPath)
{
  const std::vector<visual_navigation::PlanarPoint> path = {
    {0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 1.0}};
  EXPECT_FALSE(visual_navigation::ComputePolylineLookahead(
      path, 0.0, 0.0, 0.30).valid);
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

TEST(PathControl, PassingTargetPlaneCountsAsReached)
{
  const double progress = visual_navigation::PathSegmentProgress(
    0.0, 0.0, 1.0, 0.0, 1.05, 0.0);

  EXPECT_NEAR(progress, 1.05, 1e-12);
  EXPECT_TRUE(visual_navigation::WaypointReached(0.05, 0.01, progress, 0.0, 0.20));
  EXPECT_TRUE(visual_navigation::FinalPositionCaptured(
      false, true, 0.05, 0.01, progress, 0.0, 0.20));
}

TEST(PathControl, PassingTargetPlaneRequiresPathCorridor)
{
  const double progress = visual_navigation::PathSegmentProgress(
    0.0, 0.0, 1.0, 0.0, 1.05, 0.0);

  EXPECT_TRUE(visual_navigation::WaypointReached(0.05, 0.01, progress, 0.05, 0.20));
  EXPECT_FALSE(visual_navigation::WaypointReached(0.30, 0.01, progress, 0.30, 0.20));
  EXPECT_FALSE(visual_navigation::WaypointReached(
      0.30, 0.01, progress, std::numeric_limits<double>::quiet_NaN(), 0.20));
}

TEST(PathControl, ToleranceCircleStillCountsRegardlessOfCorridor)
{
  EXPECT_TRUE(visual_navigation::WaypointReached(0.05, 0.08, 0.50, 1.0, 0.20));
}

TEST(PathControl, WaypointIsNotReachedBeforeToleranceOrTargetPlane)
{
  const double progress = visual_navigation::PathSegmentProgress(
    0.0, 0.0, 1.0, 0.0, 0.90, 0.20);

  EXPECT_NEAR(progress, 0.90, 1e-12);
  EXPECT_FALSE(visual_navigation::WaypointReached(
      std::hypot(0.10, 0.20), 0.05, progress, 0.20, 0.20));
}

TEST(PathControl, ApproachDistanceUsesAlongTrackDistanceNearTargetPlane)
{
  const double progress = visual_navigation::PathSegmentProgress(
    0.0, 0.0, 1.0, 0.0, 0.99, 0.20);
  const double euclidean_distance = std::hypot(0.01, 0.20);

  EXPECT_NEAR(
    visual_navigation::WaypointApproachDistance(euclidean_distance, progress, 1.0),
    0.01, 1e-12);
  EXPECT_NEAR(
    visual_navigation::WaypointApproachSpeedLimit(
      0.20, 0.20, 0.8,
      visual_navigation::WaypointApproachDistance(euclidean_distance, progress, 1.0)),
    0.008, 1e-12);
}

TEST(PathControl, InvalidOrZeroLengthSegmentUsesEuclideanFallback)
{
  const double zero_length_progress = visual_navigation::PathSegmentProgress(
    1.0, 1.0, 1.0, 1.0, 2.0, 1.0);

  EXPECT_FALSE(std::isfinite(zero_length_progress));
  EXPECT_FALSE(visual_navigation::WaypointReached(
      1.0, 0.05, zero_length_progress, 0.0, 0.20));
  EXPECT_DOUBLE_EQ(
    visual_navigation::WaypointApproachDistance(1.0, zero_length_progress, 0.0), 1.0);
}

TEST(PathControl, TurnSpeedTapersTowardTarget)
{
  EXPECT_DOUBLE_EQ(
    visual_navigation::TurnSpeedForError(0.02, 0.035, 0.45, 0.25, 0.65), 0.0);
  EXPECT_DOUBLE_EQ(
    visual_navigation::TurnSpeedForError(0.50, 0.035, 0.45, 0.25, 0.65), 0.65);
  const double middle = visual_navigation::TurnSpeedForError(
    0.20, 0.035, 0.45, 0.25, 0.65);
  EXPECT_GT(middle, 0.25);
  EXPECT_LT(middle, 0.65);
}

TEST(PathControl, TurnBrakesBeforeCrossingTarget)
{
  EXPECT_TRUE(visual_navigation::ShouldBrakeTurn(0.20, 0.60, 0.035, 0.35));
  EXPECT_FALSE(visual_navigation::ShouldBrakeTurn(0.30, 0.60, 0.035, 0.35));
  EXPECT_TRUE(visual_navigation::ShouldBrakeTurn(-0.01, 0.10, 0.035, 0.35));
}

TEST(PathControl, FinalYawBrakesUnderRecordedApproachRate)
{
  EXPECT_TRUE(visual_navigation::ShouldBrakeTurn(0.11, 0.43, 0.06, 0.35));
  EXPECT_FALSE(visual_navigation::ShouldBrakeTurn(0.30, 0.43, 0.06, 0.35));
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
