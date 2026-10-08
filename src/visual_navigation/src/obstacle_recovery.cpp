#include "visual_navigation/waypoint_navigator.hpp"

void WaypointNavigator::HandleFrontObstacle(
    const std_msgs::msg::Bool::SharedPtr message) {
  if (!message)
    return;
  const auto currentTime = std::chrono::steady_clock::now();
  if (!message->data) {
    frontObstacleReported_ = false;
    const bool keepTurnObstacleLatched =
        expectedTurnObstacleSuppressed_ && EarlyPlannedTurnActive();
    if (!keepTurnObstacleLatched) {
      frontObstacleEventHandled_ = false;
      expectedTurnObstacleSuppressed_ = false;
    }
    pendingObstacleAtActiveTurn_ = false;
    if (!keepTurnObstacleLatched)
      pendingObstacleTurnDistance_ = std::numeric_limits<double>::infinity();
    // A Range message has no event id. Dropping the cached sample on the clear
    // edge prevents the preceding obstacle's distance from being reused.
    frontObstacleRangeReceived_ = false;
    frontObstacleReceived_ = true;
    lastFrontObstacleArrival_ = currentTime;
    ClearPendingObstacleSignal("front obstacle signal cleared");
    return;
  }
  if (!frontObstacleReported_ &&
      !(expectedTurnObstacleSuppressed_ && EarlyPlannedTurnActive())) {
    frontObstacleEventHandled_ = false;
    expectedTurnObstacleSuppressed_ = false;
  }
  frontObstacleReported_ = true;
  frontObstacleReceived_ = true;
  lastFrontObstacleArrival_ = currentTime;

  if (expectedTurnObstacleSuppressed_ && !EarlyPlannedTurnActive()) {
    expectedTurnObstacleSuppressed_ = false;
    frontObstacleEventHandled_ = false;
  }

  const bool forwardMotionActive = lastMotionCommand_.linear.x > 1.0e-4;
  const bool forwardFollowerActive =
      state_ == "FOLLOWING" || state_ == "BRAKING_APPROACH";
  if (frontObstacleEventHandled_ || !navigationActive_ || MotionHeld() ||
      (!forwardMotionActive && !forwardFollowerActive) ||
      pendingFrontObstacle_ || !obstacleRecoveryController_.idle()) {
    return;
  }

  // Stop in the sensor callback rather than waiting for the next control tick.
  // Classification may subsequently identify a normal wall beyond a turn.
  pendingFrontObstacle_ = true;
  frontObstacleEventHandled_ = true;
  pendingObstacleAtActiveTurn_ = EarlyPlannedTurnActive();
  pendingObstacleTurnDistance_ = pendingObstacleAtActiveTurn_
                                     ? DistanceToStoppedTurn() +
                                           vehicleFrontOffset_ +
                                           expectedObstacleTolerance_
                                     : std::numeric_limits<double>::infinity();
  pendingObstacleStartedAt_ = currentTime;
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  if (!pendingObstacleAtActiveTurn_) {
    ResetManeuver();
    PublishStop();
    SetState("OBSTACLE_BRAKING");
  } else {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Checking obstacle range while continuing the low-speed corner turn");
  }
}

void WaypointNavigator::HandleFrontObstacleRange(
    const sensor_msgs::msg::Range::SharedPtr message) {
  if (!message)
    return;
  const double measuredRange = static_cast<double>(message->range);
  const bool rangeWithinLimits =
      std::isfinite(measuredRange) && measuredRange > 0.0;
  if (!rangeWithinLimits) {
    frontObstacleRangeReceived_ = false;
    return;
  }
  frontObstacleRange_ = measuredRange;
  frontObstacleRangeReceived_ = true;
  lastFrontObstacleRangeArrival_ = std::chrono::steady_clock::now();
  if (pendingFrontObstacle_)
    ClassifyPendingObstacle();
}

void WaypointNavigator::HandleContinueRetreat(
    const mission_control_interfaces::srv::ContinueRetreat::Request::SharedPtr
        request,
    mission_control_interfaces::srv::ContinueRetreat::Response::SharedPtr
        response) {
  HandleRetreatRequest(request, response, false);
}

void WaypointNavigator::HandleContinueRouteRetreat(
    const mission_control_interfaces::srv::ContinueRetreat::Request::SharedPtr
        request,
    mission_control_interfaces::srv::ContinueRetreat::Response::SharedPtr
        response) {
  HandleRetreatRequest(request, response, true);
}

void WaypointNavigator::HandleRetreatRequest(
    const mission_control_interfaces::srv::ContinueRetreat::Request::SharedPtr
        request,
    mission_control_interfaces::srv::ContinueRetreat::Response::SharedPtr
        response,
    bool routeTransition) {
  response->accepted = false;
  response->trace_exhausted = false;
  response->remaining_breadcrumbs = static_cast<std::uint32_t>(
      obstacleRecoveryController_.remaining_breadcrumbs_before_anchor());
  if (!request || !std::isfinite(request->minimum_distance_m) ||
      request->minimum_distance_m <= 0.0) {
    response->message = "minimum retreat distance must be positive and finite";
    return;
  }
  if (MotionHeld()) {
    response->message = "motion is held by another mission source";
    return;
  }

  auto result = obstacleRecoveryController_.extend_retreat(
      request->minimum_distance_m, currentX_, currentY_, currentYaw_);
  if (result == visual_navigation::RetreatExtensionResult::INVALID_STATE &&
      state_ == "GOAL_REACHED") {
    result = obstacleRecoveryController_.begin_retreat(
        request->minimum_distance_m, currentX_, currentY_, currentYaw_);
  }
  if (result == visual_navigation::RetreatExtensionResult::INVALID_STATE) {
    response->message = routeTransition ?
      "route transition is not waiting for a new route" :
      "obstacle recovery is not waiting for a new route";
    return;
  }
  if (result == visual_navigation::RetreatExtensionResult::EXHAUSTED) {
    response->trace_exhausted = true;
    response->message = "executed breadcrumb trace is exhausted";
    return;
  }

  navigationActive_ = true;
  routeTransitionRetreat_ = routeTransition;
  pathSegmentInitialized_ = false;
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  ResetManeuver();
  BeginObstacleBrakeConfirmation();
  PublishStop();
  SetState(routeTransitionRetreat_ ? "ROUTE_TRANSITION_BRAKING"
                                   : "OBSTACLE_BRAKING");
  response->accepted = true;
  response->remaining_breadcrumbs = static_cast<std::uint32_t>(
      obstacleRecoveryController_.remaining_breadcrumbs_before_anchor());
  response->message = "continuing retreat along the executed breadcrumb trace";
  const auto replayStart = obstacleRecoveryController_.replay_start();
  const auto replayGoal = obstacleRecoveryController_.replay_goal();
  RCLCPP_WARN(get_logger(),
              "%s requested another %.3fm retreat; %u earlier "
              "samples remain; current=(%.3f, %.3f, %.1fdeg) "
              "trace_start=(%.3f, %.3f, %.1fdeg) "
              "trace_goal=(%.3f, %.3f, %.1fdeg)",
              routeTransitionRetreat_ ? "Route transition" : "Obstacle planner",
              request->minimum_distance_m, response->remaining_breadcrumbs,
              currentX_, currentY_, currentYaw_ * 180.0 / kPi,
              replayStart.point.x, replayStart.point.y,
              replayStart.yaw * 180.0 / kPi, replayGoal.point.x,
              replayGoal.point.y, replayGoal.yaw * 180.0 / kPi);
}

void WaypointNavigator::ClearPendingObstacleSignal(const char *reason) {
  if (!pendingFrontObstacle_ || !obstacleRecoveryController_.idle())
    return;
  pendingFrontObstacle_ = false;
  pendingObstacleAtActiveTurn_ = false;
  pendingObstacleTurnDistance_ = std::numeric_limits<double>::infinity();
  RCLCPP_INFO(get_logger(), "%s before recovery began; resuming the route",
              reason);
}

bool WaypointNavigator::CurrentWaypointIsPlannedTurn() const {
  if (!pathSegmentInitialized_ || currentWaypointIndex_ >= waypoints_.size() ||
      currentWaypointIndex_ + 1 >= waypoints_.size()) {
    return false;
  }
  const double incomingX =
      waypoints_[currentWaypointIndex_].x - pathSegmentStartX_;
  const double incomingY =
      waypoints_[currentWaypointIndex_].y - pathSegmentStartY_;
  const double outgoingX = waypoints_[currentWaypointIndex_ + 1].x -
                           waypoints_[currentWaypointIndex_].x;
  const double outgoingY = waypoints_[currentWaypointIndex_ + 1].y -
                           waypoints_[currentWaypointIndex_].y;
  if (std::hypot(incomingX, incomingY) <= 1.0e-6 ||
      std::hypot(outgoingX, outgoingY) <= 1.0e-6) {
    return false;
  }
  if (!waypoints_[currentWaypointIndex_].turn_junction)
    return false;
  const double incomingHeading = std::atan2(incomingY, incomingX);
  const double outgoingHeading = std::atan2(outgoingY, outgoingX);
  return std::abs(NormalizeAngle(outgoingHeading - incomingHeading)) >=
         preTurnStopHeadingThreshold_;
}

bool WaypointNavigator::EarlyPlannedTurnActive() const {
  if (!stoppedAtPlannedTurn_ || stoppedTurnReverseArrival_)
    return false;
  const double turn =
      NormalizeAngle(stoppedTurnTargetHeading_ - stoppedTurnArrivalHeading_);
  if (std::abs(turn) < preTurnStopHeadingThreshold_)
    return false;
  const double progress =
      NormalizeAngle(currentYaw_ - stoppedTurnArrivalHeading_) *
      (turn >= 0.0 ? 1.0 : -1.0);
  const double releaseAngle = std::min(0.55, 0.5 * std::abs(turn));
  return progress < releaseAngle;
}

double WaypointNavigator::DistanceToStoppedTurn() const {
  if (lastStoppedTurnIndex_ >= waypoints_.size())
    return std::numeric_limits<double>::infinity();
  return std::hypot(waypoints_[lastStoppedTurnIndex_].x - currentX_,
                    waypoints_[lastStoppedTurnIndex_].y - currentY_);
}

void WaypointNavigator::ClassifyPendingObstacle() {
  if (!pendingFrontObstacle_)
    return;
  const auto currentTime = std::chrono::steady_clock::now();
  const bool blockedFresh =
      frontObstacleReported_ && frontObstacleReceived_ &&
      std::chrono::duration<double>(currentTime - lastFrontObstacleArrival_)
              .count() <= frontObstacleTimeout_;
  if (!blockedFresh) {
    ClearPendingObstacleSignal("front obstacle heartbeat expired");
    return;
  }
  const double rangeAge = std::chrono::duration<double>(
                              currentTime - lastFrontObstacleRangeArrival_)
                              .count();
  const double rangeBeforeEvent =
      std::chrono::duration<double>(pendingObstacleStartedAt_ -
                                    lastFrontObstacleRangeArrival_)
          .count();
  const bool rangeFresh =
      frontObstacleRangeReceived_ && rangeAge >= 0.0 &&
      rangeAge <= frontObstacleTimeout_ &&
      rangeBeforeEvent <= frontObstaclePairingReorderTolerance_;
  const bool activePlannedTurn =
      stoppedAtPlannedTurn_ && !stoppedTurnReverseArrival_;
  const bool plannedTurn = pendingObstacleAtActiveTurn_ ||
                           (activePlannedTurn && EarlyPlannedTurnActive()) ||
                           (!activePlannedTurn &&
                            CurrentWaypointIsPlannedTurn());
  const double distanceToTurn =
      pendingObstacleAtActiveTurn_ ||
              (activePlannedTurn && EarlyPlannedTurnActive())
          ? (pendingObstacleAtActiveTurn_ ? pendingObstacleTurnDistance_
                                          : DistanceToStoppedTurn())
          : (currentWaypointIndex_ < waypoints_.size()
                 ? std::hypot(waypoints_[currentWaypointIndex_].x - currentX_,
                              waypoints_[currentWaypointIndex_].y - currentY_)
                 : std::numeric_limits<double>::infinity());
  const bool unexpected = visual_navigation::UnexpectedObstacle(
      true, rangeFresh, frontObstacleRange_, plannedTurn, distanceToTurn,
      obstacleSensorForwardOffset_, vehicleFrontOffset_,
      expectedObstacleTolerance_);
  if (!unexpected) {
    pendingFrontObstacle_ = false;
    pendingObstacleAtActiveTurn_ = false;
    pendingObstacleTurnDistance_ = std::numeric_limits<double>::infinity();
    RCLCPP_INFO(get_logger(),
                "Front range %.3fm is beyond the planned turn %.3fm away; "
                "continuing normal turn braking",
                frontObstacleRange_, distanceToTurn);
    SetState("EXPECTED_OBSTACLE_AT_TURN");
    return;
  }
  pendingObstacleAtActiveTurn_ = false;
  pendingObstacleTurnDistance_ = std::numeric_limits<double>::infinity();
  BeginObstacleRecovery();
}

visual_navigation::RecoveryPoint
WaypointNavigator::SelectObstacleRecoveryAnchor() const {
  if (lastStoppedTurnIndex_ < currentWaypointIndex_ &&
      lastStoppedTurnIndex_ < waypoints_.size()) {
    return {waypoints_[lastStoppedTurnIndex_].x,
            waypoints_[lastStoppedTurnIndex_].y};
  }
  // When a route begins inside a lane there is no confirmed junction yet.
  // The trace start is at least a point the robot actually traversed.
  if (obstacleRecoveryController_.breadcrumb_count() > 0)
    return obstacleRecoveryController_.anchor();
  return {currentX_, currentY_};
}

bool WaypointNavigator::BeginJunctionRetreat() {
  routeTransitionRetreat_ = false;
  const bool confirmedJunction =
      lastStoppedTurnIndex_ < currentWaypointIndex_ &&
      lastStoppedTurnIndex_ < waypoints_.size();
  visual_navigation::RecoveryPoint anchor =
      confirmedJunction
          ? visual_navigation::RecoveryPoint{waypoints_[lastStoppedTurnIndex_]
                                                 .x,
                                             waypoints_[lastStoppedTurnIndex_]
                                                 .y}
          : obstacleRecoveryController_.anchor();
  if (obstacleRecoveryController_.breadcrumb_count() < 2) {
    navigationActive_ = false;
    obstacleRecoveryController_.require_route_takeover();
    PublishStop();
    SetState("OBSTACLE_REPLAN_REQUIRED");
    RCLCPP_WARN(get_logger(),
                "Large initial heading error; requesting a replacement route "
                "instead of pivoting outside a confirmed junction");
    return false;
  }
  if (!obstacleRecoveryController_.trigger(
          currentX_, currentY_, currentYaw_, lastMotionCommand_.linear.x,
          lastMotionCommand_.angular.z, anchor.x, anchor.y)) {
    navigationActive_ = false;
    obstacleRecoveryController_.require_route_takeover();
    PublishStop();
    SetState("FAULT_JUNCTION_RETREAT_REJECTED");
    RCLCPP_ERROR(get_logger(),
                 "Junction retreat rejected at (%.3f, %.3f, %.1fdeg)",
                 currentX_, currentY_, currentYaw_ * 180.0 / kPi);
    return false;
  }
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  ResetManeuver();
  BeginObstacleBrakeConfirmation();
  PublishStop();
  SetState(confirmedJunction ? "RETREATING_TO_JUNCTION"
                             : "RETREATING_TO_ROUTE_START");
  RCLCPP_WARN(get_logger(),
              "Large heading error in lane; reversing along executed trace to "
              "%s (%.3f, %.3f) before replanning",
              confirmedJunction ? "last stopped junction" : "route start",
              anchor.x, anchor.y);
  return true;
}

void WaypointNavigator::RecordExecutedPose() {
  obstacleRecoveryController_.record_sample(currentX_, currentY_, currentYaw_,
                                            lastMotionCommand_.linear.x,
                                            lastMotionCommand_.angular.z);
}

void WaypointNavigator::BeginObstacleRecovery() {
  if (!pendingFrontObstacle_ || !navigationActive_)
    return;
  pendingFrontObstacle_ = false;
  routeTransitionRetreat_ = false;
  const auto anchor = SelectObstacleRecoveryAnchor();
  if (!obstacleRecoveryController_.trigger(
          currentX_, currentY_, currentYaw_, lastMotionCommand_.linear.x,
          lastMotionCommand_.angular.z, anchor.x, anchor.y)) {
    return;
  }
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  ResetManeuver();
  BeginObstacleBrakeConfirmation();
  PublishStop();
  SetState("OBSTACLE_BRAKING");
  RCLCPP_WARN(get_logger(),
              "Unexpected front obstacle: stopping at (%.3f, %.3f), then "
              "reversing to junction anchor (%.3f, %.3f)",
              currentX_, currentY_, anchor.x, anchor.y);
}

void WaypointNavigator::BeginObstacleBrakeConfirmation() {
  obstacleBrakeTelemetryBaseline_ = controlTelemetrySampleCount_;
  obstacleBrakeStoppedTelemetrySamples_ = 0;
  obstacleBrakeConfirmationTimeouts_ = 0;
  obstacleBrakeController_.begin(std::chrono::steady_clock::now());
}

bool WaypointNavigator::ObstacleBrakeHasCompleted() {
  const auto currentTime = std::chrono::steady_clock::now();
  const double telemetryAge =
      controlTelemetryReceived_
          ? std::chrono::duration<double>(currentTime -
                                          lastControlTelemetryArrival_)
                .count()
          : std::numeric_limits<double>::infinity();
  const double wheelEvidence = visual_navigation::IndependentWheelStopEvidence(
      measuredLeftWheelSpeed_, measuredRightWheelSpeed_, targetLeftWheelSpeed_,
      targetRightWheelSpeed_, preTurnStopSpeed_,
      obstacleBrakeStoppedTelemetrySamples_, preTurnStopTelemetrySamples_);
  auto speed = visual_navigation::SelectWaypointBrakeSpeed(
      controlTelemetrySampleCount_ > obstacleBrakeTelemetryBaseline_,
      telemetryAge, controlTelemetryTimeout_, wheelEvidence, wheelEvidence,
      pathTrackingController_.observed_speed_valid(),
      pathTrackingController_.observed_speed());
  const bool targetsStopped =
      !speed.from_wheel_telemetry ||
      std::max(std::abs(targetLeftWheelSpeed_),
               std::abs(targetRightWheelSpeed_)) <= preTurnStopSpeed_;
  if (targetsStopped &&
      visual_navigation::BrakeFallbackMayConfirmStop(
          obstacleBrakeConfirmationTimeouts_, preTurnFallbackTimeouts_,
          pathTrackingController_.observed_speed_valid(),
          pathTrackingController_.observed_speed(), preTurnFallbackPoseSpeed_,
          ImuYawRateIsFresh(), imuYawRate_, turnSettleYawRate_)) {
    speed.valid = true;
    speed.absolute_speed = 0.0;
    speed.from_wheel_telemetry = false;
  }
  const bool complete = obstacleBrakeController_.check(currentTime, speed.valid,
                                                       speed.absolute_speed);
  if (!complete && obstacleBrakeController_.timed_out()) {
    ++obstacleBrakeConfirmationTimeouts_;
    obstacleBrakeController_.begin(currentTime);
    RCLCPP_WARN(get_logger(),
                "Obstacle stop confirmation timed out; retaining zero command "
                "before reverse (retry %zu)",
                obstacleBrakeConfirmationTimeouts_);
  }
  return complete;
}

void WaypointNavigator::RunObstacleRecovery() {
  const auto recoveryState = [this](const char * obstacle,
                                    const char * transition) {
      return routeTransitionRetreat_ ? transition : obstacle;
    };
  if (obstacleRecoveryController_.braking()) {
    PublishStop();
    if (!ObstacleBrakeHasCompleted()) {
      SetState(recoveryState("OBSTACLE_BRAKING", "ROUTE_TRANSITION_BRAKING"));
      return;
    }
    obstacleBrakeController_.reset();
    obstacleRecoveryController_.confirm_stopped(
        currentX_, currentY_, std::chrono::steady_clock::now());
    ResetPathFeedback();
    SetState(recoveryState("OBSTACLE_REVERSING", "ROUTE_TRANSITION_REVERSING"));
    return;
  }

  if (!obstacleRecoveryController_.reversing()) {
    PublishStop();
    return;
  }

  if (obstacleRecoveryController_.replaying()) {
    const auto replay = obstacleRecoveryController_.replay_step(
        currentX_, currentY_, currentYaw_, lastMotionCommand_.linear.x,
        lastMotionCommand_.angular.z, std::chrono::steady_clock::now());
    if (replay.action ==
        visual_navigation::ObstacleRecoveryAction::REPLAN_REQUIRED) {
      navigationActive_ = false;
      pathSegmentInitialized_ = false;
      PublishStop();
      SetState(recoveryState(
          "OBSTACLE_REPLAN_REQUIRED", "ROUTE_TRANSITION_REPLAN_REQUIRED"));
      return;
    }
    if (replay.action ==
            visual_navigation::ObstacleRecoveryAction::FAULT_NO_PROGRESS ||
        replay.action ==
            visual_navigation::ObstacleRecoveryAction::FAULT_TIMEOUT) {
      RCLCPP_ERROR(get_logger(),
                   "%s trace replay failed: position_error=%.3fm "
                   "yaw_error=%.1fdeg",
                   routeTransitionRetreat_ ? "Route transition" : "Obstacle",
                   replay.position_error, replay.yaw_error * 180.0 / kPi);
      navigationActive_ = false;
      PublishStop();
      SetState(replay.action ==
                       visual_navigation::ObstacleRecoveryAction::FAULT_TIMEOUT
                   ? recoveryState("FAULT_OBSTACLE_REVERSE_TIMEOUT",
                                   "FAULT_ROUTE_TRANSITION_TIMEOUT")
                   : recoveryState("FAULT_OBSTACLE_REVERSE_NO_PROGRESS",
                                   "FAULT_ROUTE_TRANSITION_NO_PROGRESS"));
      return;
    }
    if (!replay.command_valid) {
      navigationActive_ = false;
      PublishStop();
      SetState(recoveryState(
          "FAULT_OBSTACLE_REVERSE_PATH", "FAULT_ROUTE_TRANSITION_PATH"));
      return;
    }
    geometry_msgs::msg::Twist command;
    command.linear.x = replay.linear_velocity;
    command.angular.z = replay.angular_velocity;
    if (replay.publish_recorded)
      PublishRecordedCommand(command);
    else
      PublishMotionCommand(command);
    SetState(recoveryState("OBSTACLE_REPLAYING", "ROUTE_TRANSITION_REPLAYING"));
    return;
  }

  const auto decision = obstacleRecoveryController_.update_reverse(
      currentX_, currentY_, std::chrono::steady_clock::now());
  if (decision.action ==
      visual_navigation::ObstacleRecoveryAction::REPLAN_REQUIRED) {
    navigationActive_ = false;
    pathSegmentInitialized_ = false;
    PublishStop();
    SetState(recoveryState(
        "OBSTACLE_REPLAN_REQUIRED", "ROUTE_TRANSITION_REPLAN_REQUIRED"));
    return;
  }
  if (decision.action ==
      visual_navigation::ObstacleRecoveryAction::RETRY_BRAKING) {
    BeginObstacleBrakeConfirmation();
    PublishStop();
    SetState(recoveryState("OBSTACLE_BRAKING", "ROUTE_TRANSITION_BRAKING"));
    RCLCPP_WARN(
        get_logger(), "%s reverse made no progress; stopping before retry %d",
        routeTransitionRetreat_ ? "Route transition" : "Obstacle",
        decision.recovery_attempt);
    return;
  }
  if (decision.action ==
          visual_navigation::ObstacleRecoveryAction::FAULT_NO_PROGRESS ||
      decision.action ==
          visual_navigation::ObstacleRecoveryAction::FAULT_TIMEOUT) {
    navigationActive_ = false;
    PublishStop();
    SetState(decision.action ==
                     visual_navigation::ObstacleRecoveryAction::FAULT_TIMEOUT
                 ? recoveryState("FAULT_OBSTACLE_REVERSE_TIMEOUT",
                                 "FAULT_ROUTE_TRANSITION_TIMEOUT")
                 : recoveryState("FAULT_OBSTACLE_REVERSE_NO_PROGRESS",
                                 "FAULT_ROUTE_TRANSITION_NO_PROGRESS"));
    return;
  }

  visual_navigation::RecoveryPoint target;
  if (!obstacleRecoveryController_.reverse_target(target)) {
    navigationActive_ = false;
    PublishStop();
    SetState(recoveryState(
        "FAULT_OBSTACLE_REVERSE_PATH", "FAULT_ROUTE_TRANSITION_PATH"));
    return;
  }
  const auto reverseCommand = visual_navigation::ComputeReverseMotionCommand(
      currentX_, currentY_, currentYaw_, target, obstacleReverseSpeed_,
      obstacleReverseAngularGain_, ControlYawRate(),
      obstacleReverseYawRateDamping_, obstacleReverseMaxAngularSpeed_,
      obstacleReverseMaxHeadingError_);
  if (!reverseCommand.valid) {
    navigationActive_ = false;
    PublishStop();
    SetState(recoveryState(
        "FAULT_OBSTACLE_REVERSE_PATH", "FAULT_ROUTE_TRANSITION_PATH"));
    return;
  }
  geometry_msgs::msg::Twist command;
  command.linear.x = reverseCommand.linear_velocity;
  command.angular.z = reverseCommand.angular_velocity;
  PublishMotionCommand(command);
  SetState(recoveryState("OBSTACLE_REVERSING", "ROUTE_TRANSITION_REVERSING"));
}

void WaypointNavigator::ResetObstacleRecovery(bool preserveExecutedTrace) {
  const auto now = std::chrono::steady_clock::now();
  const bool keepBlockedEvent =
      preserveExecutedTrace && frontObstacleReported_ &&
      frontObstacleReceived_ && frontObstacleEventHandled_ &&
      std::chrono::duration<double>(now - lastFrontObstacleArrival_).count() <=
          frontObstacleTimeout_;
  pendingFrontObstacle_ = false;
  if (!keepBlockedEvent) {
    frontObstacleReported_ = false;
    frontObstacleReceived_ = false;
    frontObstacleEventHandled_ = false;
  }
  pendingObstacleAtActiveTurn_ = false;
  pendingObstacleTurnDistance_ = std::numeric_limits<double>::infinity();
  // Bool and Range have no shared event id. A replacement route starts a new
  // obstacle event generation, so its first true heartbeat must not reuse the
  // distance captured before the reset.
  frontObstacleRangeReceived_ = false;
  obstacleRecoveryController_.prepare_for_route(preserveExecutedTrace);
  obstacleBrakeController_.reset();
  obstacleBrakeStoppedTelemetrySamples_ = 0;
  obstacleBrakeConfirmationTimeouts_ = 0;
  routeTransitionRetreat_ = false;
}
