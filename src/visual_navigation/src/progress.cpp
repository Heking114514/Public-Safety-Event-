#include "visual_navigation/waypoint_navigator.hpp"

bool WaypointNavigator::SupervisePathProgress(
    double measuredProgress, bool eligible, double commandedLinearVelocity,
    double measuredLinearVelocity, bool measuredLinearVelocityValid,
    bool linearMotionExpected) {
  const auto decision = pathProgressSupervisor_.Evaluate(
      currentWaypointIndex_, measuredProgress, currentX_, currentY_, eligible,
      std::chrono::steady_clock::now(), commandedLinearVelocity,
      measuredLinearVelocity, measuredLinearVelocityValid,
      linearMotionExpected);
  if (decision.action == visual_navigation::PathProgressAction::RECOVER) {
    RCLCPP_WARN(get_logger(),
                "No path progress; resetting control for waypoint %zu "
                "(recovery %d)",
                currentWaypointIndex_, decision.recovery_attempt);
    PublishStop();
    ResetManeuver();
    SetState("RECOVERING_NO_PATH_PROGRESS");
    return true;
  }
  if (decision.action == visual_navigation::PathProgressAction::FAULT) {
    RCLCPP_ERROR(get_logger(),
                 "Navigation stopped: waypoint %zu made no path progress after "
                 "%d recoveries",
                 currentWaypointIndex_, decision.recovery_attempt);
    navigationActive_ = false;
    PublishStop();
    ResetManeuver();
    SetState("FAULT_NO_PATH_PROGRESS");
    return true;
  }
  return false;
}

bool WaypointNavigator::SuperviseTurnProgress(std::size_t phaseOffset,
                                              double absoluteYawError,
                                              double expectedYawRate,
                                              bool eligible) {
  const std::size_t phaseId = currentWaypointIndex_ * 3 + phaseOffset;
  const auto decision = turnProgressSupervisor_.Evaluate(
      phaseId, absoluteYawError, expectedYawRate, eligible,
      std::chrono::steady_clock::now());
  if (decision.action == visual_navigation::TurnProgressAction::RECOVER) {
    RCLCPP_WARN(get_logger(), "No turn progress at waypoint %zu (recovery %d)",
                currentWaypointIndex_, decision.recovery_attempt);
    PublishStop();
    ResetManeuver();
    SetState("RECOVERING_NO_TURN_PROGRESS");
    return true;
  }
  if (decision.action == visual_navigation::TurnProgressAction::FAULT) {
    RCLCPP_ERROR(get_logger(),
                 "Navigation stopped: waypoint %zu made no turn progress after "
                 "%d recoveries",
                 currentWaypointIndex_, decision.recovery_attempt);
    navigationActive_ = false;
    PublishStop();
    ResetManeuver();
    SetState("FAULT_NO_TURN_PROGRESS");
    return true;
  }
  return false;
}

double WaypointNavigator::CompletedRouteLengthBeforeCurrentSegment() const {
  double completedLength = 0.0;
  const std::size_t end = std::min(currentWaypointIndex_, waypoints_.size());
  for (std::size_t index = 1; index < end; ++index) {
    completedLength +=
        std::hypot(waypoints_[index].x - waypoints_[index - 1].x,
                   waypoints_[index].y - waypoints_[index - 1].y);
  }
  return completedLength;
}
