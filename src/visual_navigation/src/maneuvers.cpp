#include "visual_navigation/waypoint_navigator.hpp"

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
      navigationActive_ = false;
      ResetManeuver();
      SetState("FAULT_WAYPOINT_BRAKE_TIMEOUT");
    } else {
      SetState(visual_navigation::phase_status(controlState_.phase()));
    }
    return ControlStep::DONE;
  }

  waypointBrakeController_.reset();
  controlState_.finish_braking(frame.final_waypoint);
  if (frame.final_waypoint)
    return ControlStep::CONTINUE;

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
  if (std::isfinite(frame.target.yaw)) {
    const double finalYawError = NormalizeAngle(frame.target.yaw - currentYaw_);
    const bool finalYawNeedsControl =
        std::abs(finalYawError) > finalYawTolerance_ ||
        std::abs(frame.control_yaw_rate) > turnSettleYawRate_ ||
        controlState_.anchor_valid();
    if (finalYawNeedsControl) {
      if (SuperviseTurnProgress(2, std::abs(finalYawError),
                                finalYawMaxAngularSpeed_ *
                                    frame.fusion_health.speed_scale,
                                frame.progress_supervision_allowed)) {
        return;
      }
      if (!controlState_.anchor_valid()) {
        controlState_.ensure_anchor(currentX_, currentY_);
        turnSettleController_.reset();
      }
      controlState_.ensure_anchor(currentX_, currentY_);

      geometry_msgs::msg::Twist command;
      if (std::abs(finalYawError) <= finalYawTolerance_)
        turnSettleController_.begin_settling();
      if (turnSettleController_.settling()) {
        PublishMotionCommand(command);
        const double turnActivity =
            std::max(std::abs(frame.control_yaw_rate),
                     std::abs(lastMotionCommand_.angular.z));
        if (turnActivity > turnSettleYawRate_) {
          turnSettleController_.reset_timer();
          SetState(visual_navigation::phase_status(controlState_.phase()));
          return;
        }

        if (!turnSettleController_.update(std::chrono::steady_clock::now(),
                                          turnActivity, turnSettleYawRate_,
                                          turnSettleDwell_)) {
          SetState(visual_navigation::phase_status(controlState_.phase()));
          return;
        }

        if (std::abs(finalYawError) <= finalYawTolerance_) {
          if (!visual_navigation::FinalPositionCanComplete(
                  controlState_.goal_captured(), frame.distance,
                  frame.target.tolerance)) {
            BeginFinalPositionRecovery();
            return;
          }
          CompleteNavigation();
          return;
        }
        turnSettleController_.reset();
        SetState(visual_navigation::phase_status(controlState_.phase()));
        return;
      }

      turnSettleController_.reset_timer();
      const double scaledMaxAngularSpeed =
          finalYawMaxAngularSpeed_ * frame.fusion_health.speed_scale;
      const double requestedAngularSpeed =
          Clamp(angularGain_ * finalYawError -
                    turnYawRateDamping_ * frame.control_yaw_rate,
                -scaledMaxAngularSpeed, scaledMaxAngularSpeed);
      const double minimumTurnSpeed = visual_navigation::SelectMinimumTurnSpeed(
          std::abs(finalYawError), precisionTurnThreshold_,
          finalYawMinPrecisionSpeed_, finalYawMinTurnSpeed_);
      command.angular.z = visual_navigation::EnforceMinimumTurnSpeed(
          requestedAngularSpeed, finalYawError,
          minimumTurnSpeed * frame.fusion_health.speed_scale,
          scaledMaxAngularSpeed);
      command.linear.x =
          visual_navigation::TurnPositionHoldSpeed(
              currentX_, currentY_, currentYaw_, controlState_.anchor_x(),
              controlState_.anchor_y(), turnPositionHoldGain_,
              turnPositionHoldDeadband_, turnPositionHoldMaxSpeed_) *
          frame.fusion_health.speed_scale;
      PublishMotionCommand(command);
      SetState(visual_navigation::phase_status(controlState_.phase()));
      return;
    }
  }

  turnProgressSupervisor_.Reset();
  if (!visual_navigation::FinalPositionCanComplete(
          controlState_.goal_captured(), frame.distance,
          frame.target.tolerance)) {
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

void WaypointNavigator::RunRecovery(const ControlFrame &frame) {
  const double recoveryHeading = std::atan2(frame.delta_y, frame.delta_x);
  const double recoveryHeadingError =
      NormalizeAngle(recoveryHeading - currentYaw_);
  const double scaledMaxAngularSpeed =
      waypointRecoveryMaxAngularSpeed_ * frame.fusion_health.speed_scale;
  const double requestedAngularSpeed =
      Clamp(angularGain_ * recoveryHeadingError -
                turnYawRateDamping_ * frame.control_yaw_rate,
            -scaledMaxAngularSpeed, scaledMaxAngularSpeed);
  geometry_msgs::msg::Twist command;
  if (std::abs(recoveryHeadingError) > waypointRecoveryHeadingTolerance_) {
    command.angular.z = visual_navigation::EnforceMinimumTurnSpeed(
        requestedAngularSpeed, recoveryHeadingError,
        minPrecisionTurnSpeed_ * frame.fusion_health.speed_scale,
        scaledMaxAngularSpeed);
  } else {
    command.angular.z = requestedAngularSpeed;
    command.linear.x =
        std::min(waypointRecoverySpeed_, linearGain_ * frame.distance) *
        std::max(0.0, std::cos(recoveryHeadingError)) *
        frame.fusion_health.speed_scale;
  }

  const double recoveryProgress =
      CompletedRouteLengthBeforeCurrentSegment() +
      Clamp(frame.path_length - frame.distance, 0.0, frame.path_length);
  if (SupervisePathProgress(
          recoveryProgress, frame.progress_supervision_allowed,
          lastMotionCommand_.linear.x,
          inputCache_.odometry_velocity(),
          inputCache_.odometry_velocity_valid(),
          std::abs(recoveryHeadingError) <= waypointRecoveryHeadingTolerance_)) {
    return;
  }
  if (std::abs(recoveryHeadingError) > waypointRecoveryHeadingTolerance_) {
    if (SuperviseTurnProgress(1, std::abs(recoveryHeadingError),
                              scaledMaxAngularSpeed,
                              frame.progress_supervision_allowed)) {
      return;
    }
  } else {
    turnProgressSupervisor_.Reset();
  }
  PublishMotionCommand(command);
  SetState(visual_navigation::phase_status(controlState_.phase()));
}
