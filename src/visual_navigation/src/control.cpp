#include "visual_navigation/waypoint_navigator.hpp"

visual_navigation::NavigationInputStatus
WaypointNavigator::CurrentNavigationInputs() const {
  auto input = inputCache_.snapshot(
      now(), odomTimeout_, fusionStatusTimeout_, actuatorHealthTimeout_,
      requireFusionStatus_, requireTrackingState_, requireActuatorHealth_);
  input.fusion_health =
      fusionHealthPolicy_.Evaluate(inputCache_.fusion_status());
  return input;
}

WaypointNavigator::ControlFrame WaypointNavigator::BuildControlFrame(
    const visual_navigation::FusionHealthDecision &fusionHealth,
    bool progressSupervisionAllowed) const {
  ControlFrame frame;
  frame.target = waypoints_[currentWaypointIndex_];
  frame.fusion_health = fusionHealth;
  frame.progress_supervision_allowed = progressSupervisionAllowed;
  frame.delta_x = frame.target.x - currentX_;
  frame.delta_y = frame.target.y - currentY_;
  frame.distance = std::hypot(frame.delta_x, frame.delta_y);
  frame.control_yaw_rate = ControlYawRate();
  frame.final_waypoint = currentWaypointIndex_ + 1 == waypoints_.size();

  const double pathDeltaX = frame.target.x - pathSegmentStartX_;
  const double pathDeltaY = frame.target.y - pathSegmentStartY_;
  frame.path_length = std::hypot(pathDeltaX, pathDeltaY);
  // Duplicate waypoints have no segment heading. Point toward the target until
  // the zero-length segment is consumed instead of starting a false turn.
  frame.path_heading = frame.path_length > 1.0e-6
                           ? std::atan2(pathDeltaY, pathDeltaX)
                           : std::atan2(frame.delta_y, frame.delta_x);
  frame.path_projection = visual_navigation::ProjectOntoPathSegment(
      pathSegmentStartX_, pathSegmentStartY_, frame.target.x, frame.target.y,
      currentX_, currentY_);
  frame.waypoint_requires_stop =
      WaypointRequiresStop(frame.path_heading, frame.final_waypoint) ||
      frame.target.stop_time > 0.0 || reverseSegmentActive_;
  const bool continuousWaypointPassed =
      !controlState_.recovering_goal() &&
      visual_navigation::ContinuousPathWaypointPassed(
          frame.final_waypoint, frame.waypoint_requires_stop,
          frame.path_projection, waypointPassLongitudinalTolerance_);
  frame.waypoint_reached =
      continuousWaypointPassed ||
      (controlState_.recovering_goal()
           ? frame.distance <= frame.target.tolerance
           : visual_navigation::WaypointReached(
                 frame.distance, frame.target.tolerance, frame.path_projection,
                 waypointPassLongitudinalTolerance_,
                 waypointPassLateralTolerance_));
  return frame;
}

void WaypointNavigator::RunControl() {
  if (MotionHeld()) {
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    PublishStop();
    SetState("HELD_FOR_MISSION");
    return;
  }

  if (!navigationActive_) {
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    PublishStop();
    return;
  }

  const auto supervision = navigationSupervisor_.Evaluate(
      CurrentNavigationInputs(), std::chrono::steady_clock::now());
  if (supervision.action != visual_navigation::NavigationRuntimeAction::DRIVE) {
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    PublishStop();
    if (supervision.reset_path_control)
      ResetManeuver();
    if (supervision.action ==
        visual_navigation::NavigationRuntimeAction::STOP_AND_LATCH) {
      navigationActive_ = false;
      if (pendingFrontObstacle_ || !obstacleRecoveryController_.idle()) {
        pendingFrontObstacle_ = false;
        obstacleBrakeController_.reset();
        obstacleRecoveryController_.require_route_takeover();
      }
    }
    SetState(supervision.state);
    return;
  }

  const auto fusionHealth = supervision.fusion_health;
  const bool progressSupervisionAllowed = true;
  if (!supervision.state.empty())
    SetState(supervision.state);

  const auto currentTime = std::chrono::steady_clock::now();
  const bool obstacleInputFresh =
      frontObstacleReceived_ &&
      std::chrono::duration<double>(currentTime - lastFrontObstacleArrival_)
              .count() <= frontObstacleTimeout_;
  if (frontObstacleReceived_ && !obstacleInputFresh) {
    // No publisher is a normal configuration. A producer that disappears is
    // also treated as "not blocked" until a new true sample arrives.
    frontObstacleReported_ = false;
    frontObstacleReceived_ = false;
    frontObstacleEventHandled_ = false;
    frontObstacleRangeReceived_ = false;
    ClearPendingObstacleSignal("front obstacle heartbeat expired");
  }
  if (pendingFrontObstacle_) {
    const double classificationWait =
        std::chrono::duration<double>(currentTime - pendingObstacleStartedAt_)
            .count();
    if (classificationWait >= frontObstacleClassificationWait_)
      ClassifyPendingObstacle();
    if (pendingFrontObstacle_) {
      PublishStop();
      SetState("OBSTACLE_BRAKING");
      return;
    }
  }
  if (!obstacleRecoveryController_.idle()) {
    RunObstacleRecovery();
    return;
  }

  RecordExecutedPose();

  if (controlState_.waiting() && RunWait() == ControlStep::DONE)
    return;

  if (currentWaypointIndex_ >= waypoints_.size()) {
    CompleteNavigation();
    return;
  }

  if (!pathSegmentInitialized_)
    BeginPathSegment();
  const ControlFrame frame =
      BuildControlFrame(fusionHealth, progressSupervisionAllowed);

  if (frame.final_waypoint && frame.waypoint_reached)
    controlState_.capture_goal();

  // A planned route starts at the request pose. By activation time its first
  // point can be a few millimetres away and imply an arbitrary short heading.
  if (visual_navigation::SatisfiedRouteStartMayAdvance(
          currentWaypointIndex_, frame.final_waypoint, frame.target.stop_time,
          frame.distance, frame.target.tolerance)) {
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    if (routeStartTurnAuthorized_ && frame.target.turn_junction) {
      if (!controlState_.in(visual_navigation::ControlPhase::BRAKE))
        BeginWaypointBraking();
      RunBrake(frame);
      return;
    }
    AdvanceWaypoint();
    routeStartTurnAuthorized_ = false;
    return;
  }

  const bool stopAlreadyCompleted =
      frame.final_waypoint && controlState_.goal_stopped();
  if (!controlState_.in(visual_navigation::ControlPhase::BRAKE) &&
      !stopAlreadyCompleted && frame.waypoint_reached &&
      frame.waypoint_requires_stop) {
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    BeginWaypointBraking();
    PublishStop();
    SetState(visual_navigation::phase_status(controlState_.phase()));
    return;
  }

  if (controlState_.in(visual_navigation::ControlPhase::BRAKE) &&
      RunBrake(frame) == ControlStep::DONE) {
    return;
  }

  if (controlState_.goal_captured()) {
    controlState_.begin_goal_alignment();
    RunGoal(frame);
    return;
  }

  if (frame.waypoint_reached) {
    pathProgressSupervisor_.Reset();
    turnProgressSupervisor_.Reset();
    // This is a pass-through point on the same continuous path. Keeping the
    // path PID history avoids injecting a new steering transient at every
    // planner sample. Stop/turn/recovery paths still use the resetting form.
    AdvanceWaypoint(controlState_.in(visual_navigation::ControlPhase::FOLLOW));
    return;
  }

  const bool recovering =
      controlState_.in(visual_navigation::ControlPhase::RECOVER_WAYPOINT) ||
      controlState_.in(visual_navigation::ControlPhase::RECOVER_GOAL);
  if (frame.waypoint_requires_stop && !recovering &&
      visual_navigation::WaypointNeedsRecovery(
          frame.path_projection, waypointPassLongitudinalTolerance_,
          waypointPassLateralTolerance_)) {
    ResetManeuver();
    if (controlState_.recovering_goal())
      controlState_.begin_goal_recovery();
    else
      controlState_.begin_waypoint_recovery();
  }

  if (controlState_.in(visual_navigation::ControlPhase::RECOVER_WAYPOINT) ||
      controlState_.in(visual_navigation::ControlPhase::RECOVER_GOAL)) {
    RunRecovery(frame);
    return;
  }

  if (reverseSegmentActive_) {
    RunRecovery(frame, true);
    return;
  }

  RunPath(frame);
}
