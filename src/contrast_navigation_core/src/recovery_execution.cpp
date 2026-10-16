#include "visual_navigation/waypoint_navigator.hpp"

visual_navigation::PoseRecovery::Pose WaypointNavigator::RecoveryPose() const {
  const double c = std::cos(currentYaw_), s = std::sin(currentYaw_);
  return {{currentX_ - c * trackingPointOffsetX_ + s * trackingPointOffsetY_,
           currentY_ - s * trackingPointOffsetX_ - c * trackingPointOffsetY_}, currentYaw_};
}

void WaypointNavigator::FailPoseRecovery(const std::string &reason) {
  PublishStop();
  recoveryTurnController_.reset();
  cornerReturnController_.reset();
  recoveryStopPending_ = false;
  poseRecoveryPhase_ = "failed";
  SetState(reason);
  PublishPoseRecovery("failed");
}

void WaypointNavigator::BeginRecoveryStop(double t) {
  PublishStop();
  recoveryStopGuard_.begin(t, controlTelemetrySampleCount_);
  recoveryStopPending_ = true;
}

bool WaypointNavigator::RecoveryStopCompleted(double t) {
  PublishStop();
  const double age = std::chrono::duration<double>(
    ControlNow() - lastControlTelemetryArrival_).count();
  const bool fresh = controlTelemetryReceived_ && age <= controlTelemetryTimeout_ &&
    ImuYawRateIsFresh();
  const auto result = recoveryStopGuard_.update(t, controlTelemetrySampleCount_, fresh,
    measuredLeftWheelSpeed_, measuredRightWheelSpeed_, targetLeftWheelSpeed_,
    targetRightWheelSpeed_, ControlYawRate());
  if (result == visual_navigation::RecoveryStopGuard::Result::TIMED_OUT) {
    FailPoseRecovery("FAULT_POSE_RECOVERY_STOP_TIMEOUT");
    return false;
  }
  if (result != visual_navigation::RecoveryStopGuard::Result::STOPPED) return false;
  recoveryStopPending_ = false;
  return true;
}

bool WaypointNavigator::RecoveryCommandSafe(const geometry_msgs::msg::Twist &command) const {
  const auto wall = ControlNow();
  double dt = 1.0 / controlFrequency_;
  if (motionCommandInitialized_) {
    const double measured = std::chrono::duration<double>(wall - lastMotionCommandTime_).count();
    if (measured > 0.0 && measured <= 0.2) dt = measured;
  }
  const double target = std::clamp(command.linear.x, -maxLinearSpeed_, maxLinearSpeed_);
  double v = visual_navigation::LimitRate(target, lastMotionCommand_.linear.x,
    maxLinearAcceleration_, maxLinearDeceleration_, dt);
  if (std::abs(target) > 1e-9 && std::abs(v) < minLinearCommandSpeed_)
    v = std::copysign(minLinearCommandSpeed_, target);
  else if (std::abs(target) <= 1e-9 && std::abs(v) < minLinearCommandSpeed_) v = 0.0;
  double w = visual_navigation::LimitRate(command.angular.z, lastMotionCommand_.angular.z,
    maxAngularAcceleration_, maxAngularDeceleration_, dt);
  auto pose = RecoveryPose();
  const auto &steps=poseRecovery_.steps();
  const bool adjustment=poseRecoveryPhase_=="motion" && recoveryStep_<steps.size() &&
    steps[recoveryStep_].phase=="FORWARD_ADJUSTMENT";
  const auto safe=[&](const auto &sweep) {
    return adjustment ? poseRecovery_.adjustment_safe(sweep) : poseRecovery_.safe(sweep);
  };
  // 检查本周期真实限速输出及紧急减速到停的车身扫掠。
  auto sweep = poseRecovery_.motion_samples(pose, v, w, w, dt);
  if (!safe(sweep)) return false;
  pose = sweep.back();
  const double brake_dt = 1.0 / controlFrequency_;
  while (std::abs(v) > 1e-9 || std::abs(w) > 1e-9) {
    v = visual_navigation::LimitRate(0.0, v, maxLinearAcceleration_, maxLinearDeceleration_, brake_dt);
    w = visual_navigation::LimitRate(0.0, w, maxAngularAcceleration_, maxAngularDeceleration_, brake_dt);
    sweep = poseRecovery_.motion_samples(pose, v, w, w, brake_dt);
    if (!safe(sweep)) return false;
    pose = sweep.back();
  }
  return true;
}

bool WaypointNavigator::RunPoseRecovery() {
  using Recovery = visual_navigation::PoseRecovery;
  if (poseRecoveryPhase_.empty()) return false;
  if (poseRecoveryPhase_ == "failed" || poseRecoveryPhase_ == "complete") {
    PublishStop(); return true;
  }
  if (RunTurnObstacleRecovery()) return true;
  const double t = poseRecovery_.active_seconds(ControlSeconds());
  if (poseRecoveryPhase_ == "braking") {
    if (!RecoveryStopCompleted(t)) return true;
    // 普通避障停车期间更新初始障碍；转弯特例保留原障碍和原道路。
    if (!turnObstacleContextValid_ && !recoveryKnownRoad_) {
      const double age = std::chrono::duration<double>(
        ControlNow() - recoveryPolygonTime_).count();
      if (age <= frontObstacleTimeout_ && latestRecoveryPolygon_.polygon.points.size() >= 3) {
        poseRecoveryObstacle_.points.clear();
        for (const auto &p : latestRecoveryPolygon_.polygon.points)
          poseRecoveryObstacle_.points.push_back({p.x, p.y});
        recoveryObservedObstacles_ = {poseRecoveryObstacle_};
        poseRecovery_.set_dynamic_obstacles(recoveryObservedObstacles_);
      }
      poseRecovery_.set_incoming_lane("", {});
    } else if (turnObstacleContextValid_) {
      const auto &saved = turnObstacleContext_;
      const double entry = turnManeuverConfig_.entry_target_x;
      const Recovery::Pose original{{saved.junction_x + entry * std::cos(saved.geometry_heading),
                                    saved.junction_y + entry * std::sin(saved.geometry_heading)},
                                    saved.geometry_heading};
      poseRecovery_.set_incoming_lane(poseRecovery_.lane_at_pose(original),
        {saved.junction_x, saved.junction_y});
    }
    recoveryCandidates_ = poseRecovery_.candidates(RecoveryPose(), poseRecoveryObstacle_);
    if (recoveryCandidates_.empty()) {
      FailPoseRecovery(poseRecovery_.unsafe_entry_rejected() ?
        "FAULT_POSE_RECOVERY_FIXED_ENTRY_UNSAFE" : "FAULT_POSE_RECOVERY_NO_ROAD_EXIT");
      return true;
    }
    poseRecoveryPhase_ = "candidates";
    recoveryStepStarted_ = t;
    SetState("POSE_RECOVERY_CANDIDATES");
    PublishPoseRecovery("candidates");
    return true;
  }
  if (poseRecoveryPhase_ == "candidates") {
    PublishStop();
    if (t - recoveryStepStarted_ > get_parameter("recovery_selection_timeout_s").as_double())
      FailPoseRecovery("FAULT_POSE_RECOVERY_SELECTION_TIMEOUT");
    return true;
  }
  if (poseRecoveryPhase_ != "motion") {
    FailPoseRecovery("FAULT_POSE_RECOVERY_PHASE"); return true;
  }
  if (recoveryStopPending_) {
    if (!RecoveryStopCompleted(t)) return true;
    if (!recoveryDirectionStop_) recoveryStepArmed_ = false;
    recoveryDirectionStop_ = false;
  }
  const auto &steps = poseRecovery_.steps();
  if (recoveryStep_ >= steps.size()) {
    PublishStop();
    if (!poseRecovery_.exit_matches(RecoveryPose(), recoverySelected_) ||
        poseRecovery_.lane_at_pose(RecoveryPose()) != recoverySelected_.resume_lane) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_EXIT_POSE"); return true;
    }
    poseRecoveryPhase_ = "complete";
    SetState("POSE_RECOVERY_COMPLETE"); PublishPoseRecovery("complete"); return true;
  }
  const auto &step = steps[recoveryStep_];
  if (!recoveryStepArmed_) {
    recoveryStepStarted_ = recoveryLastProgress_ = t;
    recoveryBestError_ = std::numeric_limits<double>::infinity();
    recoveryStepArmed_ = true;
    recoveryTurnManeuverStarted_ = false;
    recoveryMotionDirection_ = 0;
  }
  const auto next = [&]() {
    ++recoveryStep_;
    recoveryStepArmed_ = false;
    recoveryTurnController_.reset();
    recoveryTurnManeuverStarted_ = false;
    BeginRecoveryStop(t);
    SetState("POSE_RECOVERY_DIRECTION_STOP");
  };
  if (step.kind == Recovery::Step::Kind::STOP) {
    next(); return true;
  }
  if (t - recoveryStepStarted_ > step.duration + recoveryTurnConfig_.phase_timeout) {
    FailPoseRecovery("FAULT_POSE_RECOVERY_ACTION_TIMEOUT"); return true;
  }
  const auto actual = RecoveryPose();
  geometry_msgs::msg::Twist command;
  if (step.kind == Recovery::Step::Kind::FIXED_TURN) {
    if (!recoveryTurnManeuverStarted_) {
      const double c = std::cos(step.geometry_heading), s = std::sin(step.geometry_heading);
      const double sign = NormalizeAngle(step.target_heading - step.geometry_heading) > 0 ? 1.0 : -1.0;
      const double dx = actual.position.x - step.junction.x;
      const double dy = actual.position.y - step.junction.y;
      const double along = dx * c + dy * s - recoveryTurnConfig_.entry_target_x;
      const double cross = -dx * s + dy * c - sign * recoveryTurnConfig_.entry_target_y;
      if (std::abs(along) > recoveryTurnConfig_.entry_position_tolerance ||
          std::abs(NormalizeAngle(actual.yaw - step.geometry_heading)) >
            recoveryTurnConfig_.entry_yaw_tolerance) {
        FailPoseRecovery("FAULT_POSE_RECOVERY_FIXED_ENTRY"); return true;
      }
      RCLCPP_INFO(get_logger(), "Recovery fixed entry stopped: junction=(%.4f,%.4f) actual=(%.4f,%.4f) heading=%.4f error=(%.4f,%.4f)",
        step.junction.x, step.junction.y, actual.position.x, actual.position.y,
        actual.yaw, along, cross);
      recoveryTurnController_.begin(t);
      recoveryTurnManeuverStarted_ = true;
    }
    visual_navigation::TurnManeuverInput input;
    auto &context = input.context;
    context.pose = {actual.position.x, actual.position.y, actual.yaw};
    context.junction_x = step.junction.x; context.junction_y = step.junction.y;
    context.geometry_heading = context.arrival_heading = step.geometry_heading;
    context.target_heading = step.target_heading;
    context.reverse_arrival = step.reverse_arrival;
    context.applied_twist = {lastMotionCommand_.linear.x, lastMotionCommand_.angular.z};
    context.measured_twist = {
      inputCache_.odometry_velocity_valid() ? inputCache_.odometry_velocity() : lastMotionCommand_.linear.x,
      ControlYawRate()};
    context.measured_yaw_rate_valid = ImuYawRateIsFresh();
    const auto decision = recoveryTurnController_.update(input, t);
    if (decision.result == visual_navigation::TurnManeuverResult::COMPLETED) {
      if (!poseRecovery_.safe({actual}) ||
          std::abs(NormalizeAngle(actual.yaw - step.target_heading)) >
            recoveryTurnConfig_.entry_yaw_tolerance + 0.02) {
        FailPoseRecovery("FAULT_POSE_RECOVERY_TURN_POSE"); return true;
      }
      next(); return true;
    }
    if (decision.result != visual_navigation::TurnManeuverResult::RUNNING) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_FIXED_TURN"); return true;
    }
    command.linear.x = decision.command.linear_x;
    command.angular.z = decision.command.angular_z;
    SetState(std::string("POSE_RECOVERY_") + visual_navigation::TurnManeuverStatus(decision.phase));
  } else {
    const double c = std::cos(step.geometry_heading), s = std::sin(step.geometry_heading);
    const double dx = step.target.position.x - actual.position.x;
    const double dy = step.target.position.y - actual.position.y;
    const double along = dx * c + dy * s;
    const double yaw_error = NormalizeAngle(step.target.yaw - actual.yaw);
    const double error = std::abs(along);
    if (recoveryBestError_ - error > recoveryTurnConfig_.min_position_progress) {
      recoveryBestError_ = error; recoveryLastProgress_ = t;
    }
    if (std::abs(along) <= recoveryTurnConfig_.entry_position_tolerance &&
        std::abs(yaw_error) <= recoveryTurnConfig_.entry_yaw_tolerance) {
      next(); return true;
    }
    if (t - recoveryLastProgress_ > get_parameter("recovery_progress_timeout_s").as_double()) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_NO_PROGRESS"); return true;
    }
    if (along * step.linear < -recoveryTurnConfig_.entry_position_tolerance * std::abs(step.linear)) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_STRAIGHT_OVERSHOOT"); return true;
    }
    command.linear.x = step.linear;
    command.angular.z = std::clamp(recoveryTurnConfig_.entry_heading_gain * yaw_error,
      -recoveryTurnConfig_.entry_max_angular_speed, recoveryTurnConfig_.entry_max_angular_speed);
    SetState("POSE_RECOVERY_" + step.phase);
  }
  if (std::abs(command.linear.x) <= 1e-9 && std::abs(command.angular.z) <= 1e-9) {
    PublishStop(); return true;
  }
  const int direction = command.linear.x > 1e-9 ? 1 : command.linear.x < -1e-9 ? -1 : 0;
  if (direction && recoveryMotionDirection_ && direction != recoveryMotionDirection_) {
    recoveryMotionDirection_ = direction;
    recoveryDirectionStop_ = true;
    BeginRecoveryStop(t);
    SetState("POSE_RECOVERY_DIRECTION_STOP");
    return true;
  }
  if (direction) recoveryMotionDirection_ = direction;
  if (!RecoveryCommandSafe(command)) {
    FailPoseRecovery("FAULT_POSE_RECOVERY_STRICT_TURN_SWEEP"); return true;
  }
  PublishMotionCommand(command);
  return true;
}
