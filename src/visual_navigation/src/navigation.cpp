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
  ResetManeuver();
  controlState_.reset_segment();
}

void WaypointNavigator::AdvanceWaypoint() {
  ++currentWaypointIndex_;
  pathSegmentInitialized_ = false;
  ResetManeuver();
  controlState_.reset_segment();
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
  waypointBrakeController_.begin(std::chrono::steady_clock::now());
}

void WaypointNavigator::UpdateObservedLinearSpeed() {
  pathTrackingController_.update_observed_speed(
      currentX_, currentY_, std::chrono::steady_clock::now(),
      stopMotionWindow_);
}

bool WaypointNavigator::WaypointBrakeHasCompleted() {
  const bool complete = waypointBrakeController_.check(
      std::chrono::steady_clock::now(),
      pathTrackingController_.observed_speed_valid(),
      pathTrackingController_.observed_speed());
  if (waypointBrakeController_.timed_out()) {
    RCLCPP_ERROR(get_logger(),
                 "Waypoint brake timed out (observed speed %.3fm/s)",
                 pathTrackingController_.observed_speed_valid()
                     ? pathTrackingController_.observed_speed()
                     : std::numeric_limits<double>::quiet_NaN());
  }
  return complete;
}

void WaypointNavigator::ResetPathFeedback() {
  pathTrackingController_.reset_pid();
}

void WaypointNavigator::ResetManeuver() {
  ResetPathFeedback();
  controlState_.interrupt_maneuver();
  turnSettleController_.reset();
  waypointBrakeController_.reset();
}

void WaypointNavigator::ResetRunControl() {
  ResetManeuver();
  controlState_.reset_run();
}

double WaypointNavigator::UpdatePathPid(double error, double crossTrackError,
                                        bool allowIntegral, double yawRate) {
  return pathTrackingController_.update_pid(error, crossTrackError,
                                            allowIntegral, yawRate,
                                            std::chrono::steady_clock::now());
}
