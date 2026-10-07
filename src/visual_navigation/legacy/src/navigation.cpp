#include "visual_navigation/waypoint_navigator.hpp"

void WaypointNavigator::CompleteNavigation() {
  navigationActive_ = false;
  pathSegmentInitialized_ = false;
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  ResetRunControl();
  PublishStop();
  SetState("GOAL_REACHED");
}

void WaypointNavigator::BeginPathSegment() {
  if (resumePathFromCurrentPose_ || currentWaypointIndex_ == 0) {
    pathSegmentStartX_ = currentX_;
    pathSegmentStartY_ = currentY_;
  } else {
    pathSegmentStartX_ = waypoints_[currentWaypointIndex_ - 1].x;
    pathSegmentStartY_ = waypoints_[currentWaypointIndex_ - 1].y;
  }
  resumePathFromCurrentPose_ = false;
  pathSegmentInitialized_ = true;
  if (continuePathControlOnNextSegment_) {
    continuePathControlOnNextSegment_ = false;
  } else {
    controlState_.reset_segment();
  }
}

void WaypointNavigator::AdvanceWaypoint(bool preservePathFeedback) {
  ++currentWaypointIndex_;
  pathSegmentInitialized_ = false;
  continuePathControlOnNextSegment_ = preservePathFeedback;
  if (!preservePathFeedback) {
    ResetManeuver();
    controlState_.reset_segment();
  }
  PublishCurrentWaypoint();
}

void WaypointNavigator::BeginFinalPositionRecovery() {
  turnProgressSupervisor_.Reset();
  ResetManeuver();
  controlState_.begin_goal_recovery();
  PublishStop();
  SetState("RECOVERING_FINAL_POSITION");
}

bool WaypointNavigator::WaypointRequiresStop(double currentPathHeading,
                                             bool finalWaypoint) const {
  if (finalWaypoint)
    return true;
  const Waypoint &currentTarget = waypoints_[currentWaypointIndex_];
  const Waypoint &nextTarget = waypoints_[currentWaypointIndex_ + 1];
  const double nextDeltaX = nextTarget.x - currentTarget.x;
  const double nextDeltaY = nextTarget.y - currentTarget.y;
  if (std::hypot(nextDeltaX, nextDeltaY) <= 1.0e-9)
    return true;
  const double nextHeading = std::atan2(nextDeltaY, nextDeltaX);
  return std::abs(NormalizeAngle(nextHeading - currentPathHeading)) >=
         preTurnStopHeadingThreshold_;
}

void WaypointNavigator::BeginWaypointBraking() {
  ResetManeuver();
  controlState_.begin_braking();
  waypointBrakeTelemetryBaseline_ = controlTelemetrySampleCount_;
  waypointBrakeRecoveryAttempts_ = 0;
  waypointBrakeStoppedTelemetrySamples_ = 0;
  waypointBrakeRecovering_ = false;
  waypointBrakeController_.begin(std::chrono::steady_clock::now());
}

void WaypointNavigator::UpdateObservedLinearSpeed() {
  pathTrackingController_.update_observed_speed(
      currentX_, currentY_, std::chrono::steady_clock::now(),
      stopMotionWindow_);
}

bool WaypointNavigator::WaypointBrakeHasCompleted() {
  const auto currentTime = std::chrono::steady_clock::now();
  const double telemetryAge = controlTelemetryReceived_
                                  ? std::chrono::duration<double>(
                                        currentTime - lastControlTelemetryArrival_)
                                        .count()
                                  : std::numeric_limits<double>::infinity();
  const double wheelEvidence = visual_navigation::IndependentWheelStopEvidence(
      measuredLeftWheelSpeed_, measuredRightWheelSpeed_, targetLeftWheelSpeed_,
      targetRightWheelSpeed_, preTurnStopSpeed_,
      waypointBrakeStoppedTelemetrySamples_, preTurnStopTelemetrySamples_);
  auto speed = visual_navigation::SelectWaypointBrakeSpeed(
      controlTelemetrySampleCount_ > waypointBrakeTelemetryBaseline_,
      telemetryAge, controlTelemetryTimeout_, wheelEvidence, wheelEvidence,
      pathTrackingController_.observed_speed_valid(),
      pathTrackingController_.observed_speed());
  const bool telemetryFresh = speed.from_wheel_telemetry;
  const bool targetStopped =
      !telemetryFresh ||
      std::max(std::abs(targetLeftWheelSpeed_),
               std::abs(targetRightWheelSpeed_)) <= preTurnStopSpeed_;
  if (targetStopped && visual_navigation::BrakeFallbackMayConfirmStop(
                           waypointBrakeRecoveryAttempts_,
                           preTurnFallbackTimeouts_,
                           pathTrackingController_.observed_speed_valid(),
                           pathTrackingController_.observed_speed(),
                           preTurnFallbackPoseSpeed_, ImuYawRateIsFresh(),
                           imuYawRate_, turnSettleYawRate_)) {
    speed.valid = true;
    speed.absolute_speed = 0.0;
    speed.from_wheel_telemetry = false;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Accepting brake stop from stable fused pose and IMU after %zu "
        "telemetry confirmation timeouts",
        waypointBrakeRecoveryAttempts_);
  }
  const bool complete = waypointBrakeController_.check(
      currentTime, speed.valid, speed.absolute_speed);
  if (waypointBrakeController_.timed_out()) {
    RCLCPP_WARN(get_logger(),
                "Waypoint brake confirmation timed out on retry %zu "
                "(speed=%.3fm/s, source=%s); keeping zero command and retrying",
                waypointBrakeRecoveryAttempts_ + 1, speed.absolute_speed,
                speed.from_wheel_telemetry ? "MCU wheels" : "fused pose");
  }
  return complete;
}

void WaypointNavigator::ResetPathFeedback() {
  pathTrackingController_.reset_pid();
}

void WaypointNavigator::ResetManeuver() {
  continuePathControlOnNextSegment_ = false;
  ResetPathFeedback();
  controlState_.interrupt_maneuver();
  turnSettleController_.reset();
  waypointBrakeController_.reset();
  waypointBrakeRecovering_ = false;
  waypointBrakeRecoveryAttempts_ = 0;
  waypointBrakeStoppedTelemetrySamples_ = 0;
}

void WaypointNavigator::ResetRunControl() {
  stoppedAtPlannedTurn_ = false;
  firstHalfTurnPending_ = false;
  reverseSegmentActive_ = false;
  lastStoppedTurnIndex_ = std::numeric_limits<std::size_t>::max();
  ResetManeuver();
  controlState_.reset_run();
}

double WaypointNavigator::UpdatePathPid(double error, double crossTrackError,
                                        bool allowIntegral, double yawRate) {
  return pathTrackingController_.update_pid(error, crossTrackError,
                                            allowIntegral, yawRate,
                                            std::chrono::steady_clock::now());
}
