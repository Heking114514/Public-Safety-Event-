#include "visual_navigation/waypoint_navigator.hpp"

void WaypointNavigator::HandleFrontObstacle(
    const std_msgs::msg::Bool::SharedPtr message) {
  if (!message)
    return;
  const auto currentTime = std::chrono::steady_clock::now();
  if (!message->data) {
    frontObstacleReported_ = false;
    frontObstacleEventHandled_ = false;
    // A Range message has no event id. Dropping the cached sample on the clear
    // edge prevents the preceding obstacle's distance from being reused.
    frontObstacleRangeReceived_ = false;
    frontObstacleReceived_ = true;
    lastFrontObstacleArrival_ = currentTime;
    ClearPendingObstacleSignal("front obstacle signal cleared");
    return;
  }
  if (!frontObstacleReported_)
    frontObstacleEventHandled_ = false;
  frontObstacleReported_ = true;
  frontObstacleReceived_ = true;
  lastFrontObstacleArrival_ = currentTime;

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
  pendingObstacleStartedAt_ = currentTime;
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  ResetManeuver();
  PublishStop();
  SetState("OBSTACLE_BRAKING");
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

void WaypointNavigator::ClearPendingObstacleSignal(const char *reason) {
  if (!pendingFrontObstacle_ || !obstacleRecoveryController_.idle())
    return;
  pendingFrontObstacle_ = false;
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
  const double distanceToTurn =
      currentWaypointIndex_ < waypoints_.size()
          ? std::hypot(waypoints_[currentWaypointIndex_].x - currentX_,
                       waypoints_[currentWaypointIndex_].y - currentY_)
          : std::numeric_limits<double>::infinity();
  const bool unexpected = visual_navigation::UnexpectedObstacle(
      true, rangeFresh, frontObstacleRange_, CurrentWaypointIsPlannedTurn(),
      distanceToTurn, obstacleSensorForwardOffset_, vehicleFrontOffset_,
      expectedObstacleTolerance_);
  if (!unexpected) {
    pendingFrontObstacle_ = false;
    RCLCPP_INFO(get_logger(),
                "Front range %.3fm is beyond the planned turn %.3fm away; "
                "continuing normal turn braking",
                frontObstacleRange_, distanceToTurn);
    SetState("EXPECTED_OBSTACLE_AT_TURN");
    return;
  }
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
  const bool confirmedJunction =
      lastStoppedTurnIndex_ < currentWaypointIndex_ &&
      lastStoppedTurnIndex_ < waypoints_.size();
  visual_navigation::RecoveryPoint anchor =
      confirmedJunction
          ? visual_navigation::RecoveryPoint{
                waypoints_[lastStoppedTurnIndex_].x,
                waypoints_[lastStoppedTurnIndex_].y}
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
          currentX_, currentY_, anchor.x, anchor.y)) {
    PublishStop();
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
  obstacleRecoveryController_.record_pose(currentX_, currentY_);
}

void WaypointNavigator::BeginObstacleRecovery() {
  if (!pendingFrontObstacle_ || !navigationActive_)
    return;
  pendingFrontObstacle_ = false;
  const auto anchor = SelectObstacleRecoveryAnchor();
  if (!obstacleRecoveryController_.trigger(currentX_, currentY_, anchor.x,
                                           anchor.y)) {
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

void WaypointNavigator::RunObstacleRecovery(
    const visual_navigation::FusionHealthDecision &fusionHealth) {
  if (obstacleRecoveryController_.braking()) {
    PublishStop();
    if (!ObstacleBrakeHasCompleted()) {
      SetState("OBSTACLE_BRAKING");
      return;
    }
    obstacleBrakeController_.reset();
    obstacleRecoveryController_.confirm_stopped(
        currentX_, currentY_, std::chrono::steady_clock::now());
    ResetPathFeedback();
    SetState("OBSTACLE_REVERSING");
    return;
  }

  if (!obstacleRecoveryController_.reversing()) {
    PublishStop();
    return;
  }

  const auto decision = obstacleRecoveryController_.update_reverse(
      currentX_, currentY_, std::chrono::steady_clock::now());
  if (decision.action ==
      visual_navigation::ObstacleRecoveryAction::REPLAN_REQUIRED) {
    navigationActive_ = false;
    pathSegmentInitialized_ = false;
    PublishStop();
    SetState("OBSTACLE_REPLAN_REQUIRED");
    return;
  }
  if (decision.action ==
      visual_navigation::ObstacleRecoveryAction::RETRY_BRAKING) {
    BeginObstacleBrakeConfirmation();
    PublishStop();
    SetState("OBSTACLE_BRAKING");
    RCLCPP_WARN(get_logger(),
                "Obstacle reverse made no progress; stopping before retry %d",
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
                 ? "FAULT_OBSTACLE_REVERSE_TIMEOUT"
                 : "FAULT_OBSTACLE_REVERSE_NO_PROGRESS");
    return;
  }

  visual_navigation::RecoveryPoint target;
  if (!obstacleRecoveryController_.reverse_target(target)) {
    navigationActive_ = false;
    PublishStop();
    SetState("FAULT_OBSTACLE_REVERSE_PATH");
    return;
  }
  const auto reverseCommand = visual_navigation::ComputeReverseMotionCommand(
      currentX_, currentY_, currentYaw_, target,
      obstacleReverseSpeed_ * fusionHealth.speed_scale,
      obstacleReverseAngularGain_, ControlYawRate(),
      obstacleReverseYawRateDamping_,
      obstacleReverseMaxAngularSpeed_ * fusionHealth.speed_scale,
      obstacleReverseMaxHeadingError_);
  if (!reverseCommand.valid) {
    navigationActive_ = false;
    PublishStop();
    SetState("FAULT_OBSTACLE_REVERSE_PATH");
    return;
  }
  geometry_msgs::msg::Twist command;
  command.linear.x = reverseCommand.linear_velocity;
  command.angular.z = reverseCommand.angular_velocity;
  PublishMotionCommand(command);
  SetState("OBSTACLE_REVERSING");
}

void WaypointNavigator::ResetObstacleRecovery() {
  pendingFrontObstacle_ = false;
  frontObstacleReported_ = false;
  frontObstacleReceived_ = false;
  frontObstacleEventHandled_ = false;
  // Bool and Range have no shared event id. A replacement route starts a new
  // obstacle event generation, so its first true heartbeat must not reuse the
  // distance captured before the reset.
  frontObstacleRangeReceived_ = false;
  obstacleRecoveryController_.reset();
  obstacleBrakeController_.reset();
  obstacleBrakeStoppedTelemetrySamples_ = 0;
  obstacleBrakeConfirmationTimeouts_ = 0;
}
