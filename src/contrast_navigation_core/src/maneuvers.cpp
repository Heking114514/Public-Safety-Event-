#include "visual_navigation/waypoint_navigator.hpp"
#include "visual_navigation/turn_geometry.hpp"

WaypointNavigator::ControlStep WaypointNavigator::RunWait() {
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  PublishStop();
  if (now() < waitUntil_) {
    SetState(visual_navigation::phase_status(controlState_.phase()));
    return ControlStep::DONE;
  }

  const auto completedAction = controlState_.finish_wait();
  if (completedAction == visual_navigation::WaitAction::ADVANCE) {
    AdvanceWaypoint();
    return ControlStep::CONTINUE;
  }
  CompleteNavigation();
  return ControlStep::DONE;
}

WaypointNavigator::ControlStep
WaypointNavigator::RunBrake(const ControlFrame &frame) {
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  PublishStop();
  if (!WaypointBrakeHasCompleted()) {
    if (waypointBrakeController_.timed_out()) {
      ++waypointBrakeRecoveryAttempts_;
      waypointBrakeRecovering_ = true;
      waypointBrakeController_.begin(ControlNow());
      SetState("RECOVERING_WAYPOINT_BRAKE");
    } else {
      SetState(waypointBrakeRecovering_
                   ? "RECOVERING_WAYPOINT_BRAKE"
                   : visual_navigation::phase_status(controlState_.phase()));
    }
    return ControlStep::DONE;
  }

  waypointBrakeController_.reset();
  waypointBrakeRecovering_ = false;
  waypointBrakeRecoveryAttempts_ = 0;
  controlState_.finish_braking(frame.final_waypoint);
  if (frame.final_waypoint) {
    reverseSegmentActive_ = false;
    return ControlStep::CONTINUE;
  }

  const Waypoint &nextTarget = waypoints_[currentWaypointIndex_ + 1];
  const double nextHeading =
      std::atan2(nextTarget.y - frame.target.y, nextTarget.x - frame.target.x);
  const bool routeStartTurn = routeStartTurnAuthorized_ &&
                              frame.target.turn_junction;
  const auto entryGeometry = visual_navigation::BuildTurnEntryGeometry(
      waypoints_, currentWaypointIndex_,
      {pathSegmentStartX_, pathSegmentStartY_}, frame.path_heading);
  const bool plannedReverse = entryGeometry.out_and_back;
  const bool reverseArrivalTurn =
      frame.target.reverse_arrival_turn && reverseSegmentActive_;
  const double geometryHeading =
      routeStartTurn
          ? currentYaw_
          : NormalizeAngle(
                entryGeometry.incoming_heading +
                (reverseSegmentActive_ && !reverseArrivalTurn ? kPi : 0.0));
  const double plannedTurnError = NormalizeAngle(nextHeading - geometryHeading);
  stoppedTurnGeometryHeading_ = geometryHeading;
  stoppedTurnArrivalHeading_ = currentYaw_;
  stoppedTurnTargetHeading_ = nextHeading;
  stoppedTurnReverseArrival_ = reverseArrivalTurn;
  stoppedAtPlannedTurn_ = frame.target.turn_junction && !plannedReverse &&
                          std::abs(plannedTurnError) >= 0.70;
  if (stoppedAtPlannedTurn_) {
    RCLCPP_INFO(get_logger(),
                "Fixed turn geometry: waypoint=%zu incoming=%.2fdeg "
                "arrival=%.2fdeg target=%.2fdeg route_start=%s reverse_arrival=%s",
                currentWaypointIndex_, geometryHeading * 180.0 / kPi,
                currentYaw_ * 180.0 / kPi, nextHeading * 180.0 / kPi,
                routeStartTurn ? "true" : "false",
                reverseArrivalTurn ? "true" : "false");
    lastStoppedTurnIndex_ = currentWaypointIndex_;
    firstHalfTurnPending_ = std::abs(plannedTurnError) > 2.35;
    if (firstHalfTurnPending_)
      firstHalfTurnYaw_ =
          visual_navigation::FirstHalfTurnYaw(geometryHeading, nextHeading);
  } else {
    firstHalfTurnPending_ = false;
    stoppedTurnReverseArrival_ = false;
  }
  routeStartTurnAuthorized_ = false;
  reverseSegmentActive_ = plannedReverse;

  if (frame.target.stop_time > 0.0) {
    waitUntil_ = now() + rclcpp::Duration::from_seconds(frame.target.stop_time);
    controlState_.begin_wait(visual_navigation::WaitAction::ADVANCE);
    SetState(visual_navigation::phase_status(controlState_.phase()));
    return ControlStep::DONE;
  }

  AdvanceWaypoint();
  return ControlStep::DONE;
}

void WaypointNavigator::RunGoal(const ControlFrame &frame) {
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  if (!visual_navigation::FinalPositionCanComplete(
          controlState_.goal_captured(), frame.distance,
          std::max(frame.target.tolerance, finalPositionReleaseTolerance_))) {
    BeginFinalPositionRecovery();
    return;
  }

  if (frame.target.stop_time > 0.0) {
    waitUntil_ = now() + rclcpp::Duration::from_seconds(frame.target.stop_time);
    controlState_.begin_wait(visual_navigation::WaitAction::COMPLETE);
    PublishStop();
    SetState(visual_navigation::phase_status(controlState_.phase()));
    return;
  }

  CompleteNavigation();
}

void WaypointNavigator::RunRecovery(const ControlFrame &frame,
                                    bool plannedReverse) {
  const double recoveryHeading = std::atan2(frame.delta_y, frame.delta_x);
  const double forwardError = NormalizeAngle(recoveryHeading - currentYaw_);
  const bool reverse = plannedReverse || std::abs(forwardError) > kPi / 2.0;
  const double recoveryHeadingError =
      NormalizeAngle(recoveryHeading - (reverse ? kPi : 0.0) - currentYaw_);
  const double requestedAngularSpeed = Clamp(
      angularGain_ * recoveryHeadingError -
          turnYawRateDamping_ * frame.control_yaw_rate,
      -waypointRecoveryMaxAngularSpeed_, waypointRecoveryMaxAngularSpeed_);
  geometry_msgs::msg::Twist command;
  command.angular.z = requestedAngularSpeed;
  command.linear.x =
      (reverse ? -1.0 : 1.0) *
      std::min(waypointRecoverySpeed_, linearGain_ * frame.distance) *
      std::max(0.25, std::cos(recoveryHeadingError));

  const double recoveryProgress =
      CompletedRouteLengthBeforeCurrentSegment() +
      Clamp(frame.path_length - frame.distance, 0.0, frame.path_length);
  if (SupervisePathProgress(
          recoveryProgress, frame.progress_supervision_allowed,
          lastMotionCommand_.linear.x, inputCache_.odometry_velocity(),
          inputCache_.odometry_velocity_valid(), true)) {
    return;
  }
  turnProgressSupervisor_.Reset();
  PublishMotionCommand(command);
  SetState(plannedReverse
               ? "REVERSING_PLANNED_RETREAT"
               : visual_navigation::phase_status(controlState_.phase()));
}
