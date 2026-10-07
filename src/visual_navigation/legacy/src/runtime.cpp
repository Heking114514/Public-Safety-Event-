#include "visual_navigation/waypoint_navigator.hpp"

void WaypointNavigator::HandleOdometry(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  if (!message) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejecting null odometry message");
    return;
  }
  const bool odometryFrameValid = visual_navigation::OdometryFramePairMatches(
      message->header.frame_id, message->child_frame_id, routeFrame_,
      odomChildFrame_);
  const bool odometryPoseValid =
      visual_navigation::OdometryPoseIsFinite(*message) &&
      QuaternionIsValid(message->pose.pose.orientation);
  if (!odometryFrameValid) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Rejecting odometry frames '%s' -> '%s'; expected '%s' "
                          " -> '%s' (TF conversion is disabled)",
                          message->header.frame_id.c_str(),
                          message->child_frame_id.c_str(), routeFrame_.c_str(),
                          odomChildFrame_.c_str());
    return;
  }
  if (!odometryPoseValid) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Rejecting invalid odometry pose");
    return;
  }

  // Validate structure before timestamp bookkeeping. A malformed sample must
  // not replace the last good pose or advance the monotonic stamp baseline.
  if (!AcceptMeasurementStamp(message->header.stamp, lastOdomStamp_,
                              odomStampValid_, odomTimeout_, "odometry"))
    return;
  const auto odometryReceivedAt = now();

  currentYaw_ = YawFromValidQuaternion(message->pose.pose.orientation);
  if (!trackingReferenceInitialized_) {
    trackingReferenceYaw_ = currentYaw_;
    trackingReferenceInitialized_ = true;
  }
  const auto basePosition = visual_navigation::BasePositionFromTrackingPoint(
      message->pose.pose.position.x, message->pose.pose.position.y, currentYaw_,
      trackingReferenceYaw_, trackingPointOffsetX_, trackingPointOffsetY_);
  currentX_ = basePosition.x;
  currentY_ = basePosition.y;
  UpdateObservedLinearSpeed();
  const bool odometryVelocityValid =
      visual_navigation::OdometryVelocityIsFinite(*message);
  const double odometryVelocity =
      odometryVelocityValid ? message->twist.twist.linear.x : 0.0;
  inputCache_.update_odometry(odometryFrameValid, odometryPoseValid,
                              odometryVelocityValid, odometryVelocity,
                              odometryReceivedAt);
}

void WaypointNavigator::HandleImu(
    const sensor_msgs::msg::Imu::SharedPtr message) {
  if (!AcceptMeasurementStamp(message->header.stamp, lastImuStamp_,
                              imuStampValid_, imuTimeout_, "IMU"))
    return;
  if (!std::isfinite(message->angular_velocity.z))
    return;
  imuYawRate_ = message->angular_velocity.z;
  inputCache_.update_imu(now());
}

void WaypointNavigator::HandleTrackingState(
    const std_msgs::msg::Int32::SharedPtr message) {
  inputCache_.update_tracking(message->data);
}

void WaypointNavigator::HandleFusionStatus(
    const std_msgs::msg::String::SharedPtr message) {
  inputCache_.update_fusion_status(Trim(message->data), now());
}

void WaypointNavigator::HandleActuatorHealth(
    const std_msgs::msg::Bool::SharedPtr message) {
  inputCache_.update_actuator(message->data, now());
}

void WaypointNavigator::HandleControlTelemetry(
    const mission_control_interfaces::msg::ControlTelemetry::SharedPtr message) {
  if (!message || !std::isfinite(message->measured_left_velocity_mps) ||
      !std::isfinite(message->measured_right_velocity_mps) ||
      !std::isfinite(message->target_left_velocity_mps) ||
      !std::isfinite(message->target_right_velocity_mps)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Ignoring invalid wheel control telemetry");
    return;
  }
  if (controlTelemetryReceived_) {
    if (!visual_navigation::ControlTelemetrySampleIsFresh(
            lastControlTelemetrySequence_, lastControlTelemetryMcuTime_,
            message->sample_sequence, message->mcu_time_ms)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Ignoring duplicate or out-of-order wheel telemetry");
      return;
    }
  }
  measuredLeftWheelSpeed_ = message->measured_left_velocity_mps;
  measuredRightWheelSpeed_ = message->measured_right_velocity_mps;
  targetLeftWheelSpeed_ = message->target_left_velocity_mps;
  targetRightWheelSpeed_ = message->target_right_velocity_mps;
  lastControlTelemetrySequence_ = message->sample_sequence;
  lastControlTelemetryMcuTime_ = message->mcu_time_ms;
  lastControlTelemetryArrival_ = std::chrono::steady_clock::now();
  controlTelemetryReceived_ = true;
  ++controlTelemetrySampleCount_;
  if (controlState_.in(visual_navigation::ControlPhase::BRAKE)) {
    const double wheelEvidence = visual_navigation::IndependentWheelStopEvidence(
        measuredLeftWheelSpeed_, measuredRightWheelSpeed_,
        targetLeftWheelSpeed_, targetRightWheelSpeed_, preTurnStopSpeed_, 1, 1);
    if (std::isfinite(wheelEvidence) && wheelEvidence <= preTurnStopSpeed_)
      ++waypointBrakeStoppedTelemetrySamples_;
    else
      waypointBrakeStoppedTelemetrySamples_ = 0;
  }
  if (obstacleRecoveryController_.braking()) {
    const double wheelEvidence = visual_navigation::IndependentWheelStopEvidence(
        measuredLeftWheelSpeed_, measuredRightWheelSpeed_,
        targetLeftWheelSpeed_, targetRightWheelSpeed_, preTurnStopSpeed_, 1, 1);
    if (std::isfinite(wheelEvidence) && wheelEvidence <= preTurnStopSpeed_)
      ++obstacleBrakeStoppedTelemetrySamples_;
    else
      obstacleBrakeStoppedTelemetrySamples_ = 0;
  }
}

void WaypointNavigator::HandleStart(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  response->success = ActivateNavigation(response->message);
}

void WaypointNavigator::HandleStartTopic(
    const std_msgs::msg::Empty::SharedPtr) {
  std::string result;
  if (ActivateNavigation(result))
    RCLCPP_INFO(get_logger(), "Navigation start topic accepted: %s",
                result.c_str());
  else
    RCLCPP_WARN(get_logger(), "Navigation start topic rejected: %s",
                result.c_str());
}

bool WaypointNavigator::ActivateNavigation(std::string &result) {
  if (!routeLoaded_) {
    result = "Waypoint route is not loaded";
    return false;
  }
  if (pendingFrontObstacle_ || !obstacleRecoveryController_.idle()) {
    result = "A new route is required after obstacle recovery";
    return false;
  }
  obstacleRecoveryController_.reset_trace();

  const auto inputs = CurrentNavigationInputs();
  const auto readiness = visual_navigation::EvaluateNavigationStart(inputs);
  if (!readiness.ready) {
    // During startup, missing/stale prerequisites are a recoverable wait. Keep
    // the route active so the first fresh fusion/odometry sample can start it
    // without asking the operator to repeat the command. Structural bad data
    // and an already-confirmed fusion fault still reject the start.
    const bool structural_odometry_error =
        inputs.odometry_received &&
        (!inputs.odometry_frame_valid || !inputs.odometry_pose_valid);
    const bool confirmed_fusion_fault =
        inputs.fusion_status_received && inputs.fusion_status_fresh &&
        inputs.fusion_health.fault;
    if (!structural_odometry_error && !confirmed_fusion_fault) {
      navigationActive_ = true;
      if (currentWaypointIndex_ >= waypoints_.size())
        currentWaypointIndex_ = 0;
      navigationSupervisor_.ResetLocalizationHistory();
      pathProgressSupervisor_.Reset();
      turnProgressSupervisor_.Reset();
      pathSegmentInitialized_ = false;
      ResetRunControl();
      const bool actuator_wait =
          inputs.actuator_required && !ActuatorHealthIsValid(inputs);
      const std::string deferred_state =
          MotionHeld()
              ? "HELD_FOR_MISSION"
              : (actuator_wait
                     ? "WAITING_FOR_ACTUATOR_RECOVERY"
                     : (inputs.fusion_status == "WAITING_FOR_INITIALIZATION"
                            ? "WAITING_FOR_INITIALIZATION"
                            : "WAITING_FOR_LOCALIZATION"));
      SetState(deferred_state);
      result = "Navigation queued until startup inputs are ready";
      return true;
    }
    navigationActive_ = false;
    PublishStop();
    SetState(readiness.failure_state);
    result = "Navigation start rejected: " + readiness.failure_state;
    return false;
  }

  if (currentWaypointIndex_ >= waypoints_.size())
    currentWaypointIndex_ = 0;
  navigationActive_ = true;
  navigationSupervisor_.ResetLocalizationHistory();
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  pathSegmentInitialized_ = false;
  ResetRunControl();
  SetState(MotionHeld() ? "HELD_FOR_MISSION" : "WAITING_FOR_LOCALIZATION");
  result = "Waypoint navigation started";
  return true;
}

void WaypointNavigator::HandleStop(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  navigationActive_ = false;
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  pathSegmentInitialized_ = false;
  ResetRunControl();
  ResetObstacleRecovery();
  PublishStop();
  SetState("IDLE");
  response->success = true;
  response->message = "Waypoint navigation stopped";
}

void WaypointNavigator::HandleSetMotionHold(
    const mission_control_interfaces::srv::SetMotionHold::Request::SharedPtr
        request,
    mission_control_interfaces::srv::SetMotionHold::Response::SharedPtr
        response) {
  const std::string source = Trim(request->source);
  const std::string reason = Trim(request->reason);
  if (source.empty() || source.size() > 128) {
    response->success = false;
    response->motion_held = MotionHeld();
    response->active_sources = ActiveHoldSources();
    response->message =
        "source must contain 1 to 128 non-whitespace characters";
    return;
  }
  if (reason.size() > 256) {
    response->success = false;
    response->motion_held = MotionHeld();
    response->active_sources = ActiveHoldSources();
    response->message = "reason must not exceed 256 characters";
    return;
  }

  const bool wasHeld = MotionHeld();
  if (request->hold) {
    motionHolds_[source] = reason;
    if (!wasHeld) {
      motionHoldStartedAt_ = now();
      motionHoldStartInitialized_ = true;
      pathProgressSupervisor_.Reset();
      turnProgressSupervisor_.Reset();
      pathSegmentInitialized_ = false;
      controlState_.clear_goal();
      ResetManeuver();
    }
    PublishStop();
    SetState("HELD_FOR_MISSION");
    response->message = "Motion hold acquired for " + source;
  } else {
    const bool sourceWasActive = motionHolds_.erase(source) > 0;
    if (wasHeld && !MotionHeld()) {
      if (controlState_.waiting() && motionHoldStartInitialized_) {
        const rclcpp::Duration holdDuration = now() - motionHoldStartedAt_;
        if (holdDuration.nanoseconds() > 0)
          waitUntil_ = waitUntil_ + holdDuration;
      }
      motionHoldStartInitialized_ = false;
      navigationSupervisor_.ResetLocalizationHistory();
      pathSegmentInitialized_ = false;
      resumePathFromCurrentPose_ = true;
      controlState_.clear_goal();
      ResetManeuver();
      SetState(navigationActive_ ? "RESUME_PENDING_LOCALIZATION" : "IDLE");
    }
    response->message = sourceWasActive
                            ? "Motion hold released for " + source
                            : "No motion hold was active for " + source;
  }

  PublishMotionHoldState();
  response->success = true;
  response->motion_held = MotionHeld();
  response->active_sources = ActiveHoldSources();
  RCLCPP_INFO(
      get_logger(), "Motion hold request: source=%s hold=%s active_sources=%zu",
      source.c_str(), request->hold ? "true" : "false", motionHolds_.size());
}

void WaypointNavigator::HandleReset(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  navigationActive_ = false;
  currentWaypointIndex_ = 0;
  navigationSupervisor_.ResetLocalizationHistory();
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  pathSegmentInitialized_ = false;
  ResetRunControl();
  routeStartTurnAuthorized_ =
      !waypoints_.empty() && waypoints_.front().turn_junction;
  ResetObstacleRecovery();
  PublishStop();
  PublishCurrentWaypoint();
  SetState("IDLE");
  response->success = true;
  response->message = "Waypoint navigation reset";
}

bool WaypointNavigator::ImuYawRateIsFresh() const {
  return inputCache_.imu_fresh(now(), imuTimeout_);
}

bool WaypointNavigator::AcceptMeasurementStamp(
    const builtin_interfaces::msg::Time &stamp_message,
    rclcpp::Time &last_stamp, bool &stamp_valid, double max_age,
    const char *source) {
  // rclcpp::Time throws for an invalid ROS time representation. Reject it
  // before construction so one bad sensor packet cannot terminate navigation.
  if (stamp_message.sec < 0 || stamp_message.nanosec >= 1000000000U) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejecting %s measurement with an invalid timestamp",
                         source);
    return false;
  }
  const rclcpp::Time stamp(stamp_message);
  if (stamp.nanoseconds() <= 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejecting %s measurement with an unset timestamp",
                         source);
    return false;
  }
  if (stamp_valid && stamp <= last_stamp) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejecting non-monotonic %s timestamp", source);
    return false;
  }
  const rclcpp::Time current = now();
  if (current.nanoseconds() > 0) {
    const double age = (current - stamp).seconds();
    if (age > max_age || age < -stampFutureTolerance_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Rejecting %s timestamp with age %.3fs", source,
                           age);
      return false;
    }
  }
  last_stamp = stamp;
  stamp_valid = true;
  return true;
}

double WaypointNavigator::ControlYawRate() const {
  return ImuYawRateIsFresh() ? imuYawRate_ : 0.0;
}
