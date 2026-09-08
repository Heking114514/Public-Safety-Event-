#include "visual_navigation/waypoint_navigator.hpp"

void WaypointNavigator::RunPath(const ControlFrame &frame) {
  const double headingError = NormalizeAngle(frame.path_heading - currentYaw_);
  // Positive cross-track error means the car is to the left of the path.
  const double crossTrackError =
      std::cos(frame.path_heading) * (currentY_ - pathSegmentStartY_) -
      std::sin(frame.path_heading) * (currentX_ - pathSegmentStartX_);
  const double requestedSpeed = std::min(frame.target.speed, maxLinearSpeed_);
  double signedPathCurvature = 0.0;
  if (currentWaypointIndex_ + 1 < waypoints_.size()) {
    const Waypoint &next = waypoints_[currentWaypointIndex_ + 1];
    const double nextLength =
        std::hypot(next.x - frame.target.x, next.y - frame.target.y);
    if (nextLength > 1.0e-4) {
      const double nextHeading =
          std::atan2(next.y - frame.target.y, next.x - frame.target.x);
      signedPathCurvature = NormalizeAngle(nextHeading - frame.path_heading) /
                            std::max(0.02, nextLength);
    }
  }
  const double pathCurvature = std::abs(signedPathCurvature);
  const double pathError = visual_navigation::StanleyPathError(
      headingError, crossTrackError, crossTrackGain_, requestedSpeed,
      stanleySofteningSpeed_, maxCrossTrackCorrection_);
  const double rotationEntryThreshold =
      visual_navigation::RotationEntryThreshold(controlState_.path_aligned(),
                                                rotateInPlaceThreshold_,
                                                rotateInPlaceReentryThreshold_);
  const bool shouldRotateInPlace = visual_navigation::ShouldRotateInPlace(
      false, std::abs(headingError), std::abs(frame.control_yaw_rate),
      rotationEntryThreshold, rotateInPlaceExitThreshold_, turnSettleYawRate_);

  if (!controlState_.in(visual_navigation::ControlPhase::ALIGN_PATH) &&
      shouldRotateInPlace) {
    ResetManeuver();
    controlState_.begin_path_alignment(currentX_, currentY_);
    turnSettleController_.reset();
  } else if (!controlState_.in(visual_navigation::ControlPhase::ALIGN_PATH) &&
             !controlState_.path_aligned()) {
    controlState_.mark_path_aligned();
  }

  const bool aligningPath =
      controlState_.in(visual_navigation::ControlPhase::ALIGN_PATH);
  const bool allowPathIntegral = !aligningPath &&
                                 std::abs(headingError) < 0.30 &&
                                 std::abs(crossTrackError) < 0.08;
  const double pathFeedbackAngularSpeed =
      UpdatePathPid(pathError, crossTrackError, allowPathIntegral,
                    frame.control_yaw_rate) *
      frame.fusion_health.speed_scale;
  geometry_msgs::msg::Twist command;
  bool brakingApproach = false;
  const double segmentProgress =
      frame.path_projection.valid
          ? CompletedRouteLengthBeforeCurrentSegment() +
                Clamp(frame.path_length - frame.path_projection.remaining, 0.0,
                      frame.path_length)
          : std::numeric_limits<double>::quiet_NaN();

  if (aligningPath) {
    if (SupervisePathProgress(segmentProgress,
                              frame.progress_supervision_allowed,
                              0.0, 0.0, false, false)) {
      return;
    }
    if (SuperviseTurnProgress(0, std::abs(headingError),
                              turnCruiseSpeed_ *
                                  frame.fusion_health.speed_scale,
                              frame.progress_supervision_allowed)) {
      return;
    }
    controlState_.ensure_anchor(currentX_, currentY_);
    if (std::abs(headingError) <= rotateInPlaceExitThreshold_)
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
      if (std::abs(headingError) <= rotateInPlaceExitThreshold_) {
        ResetManeuver();
        turnProgressSupervisor_.Reset();
        controlState_.finish_path_alignment();
        SetState("PATH_ALIGNED");
        return;
      }
      turnSettleController_.reset();
      SetState(visual_navigation::phase_status(controlState_.phase()));
      return;
    }

    turnSettleController_.reset_timer();
    const double scaledMaxAngularSpeed =
        turnCruiseSpeed_ * frame.fusion_health.speed_scale;
    const double requestedAngularSpeed =
        Clamp(angularGain_ * headingError -
                  turnYawRateDamping_ * frame.control_yaw_rate,
              -scaledMaxAngularSpeed, scaledMaxAngularSpeed);
    command.angular.z = visual_navigation::EnforceMinimumTurnSpeed(
        requestedAngularSpeed, headingError,
        minPrecisionTurnSpeed_ * frame.fusion_health.speed_scale,
        scaledMaxAngularSpeed);
    command.linear.x =
        visual_navigation::TurnPositionHoldSpeed(
            currentX_, currentY_, currentYaw_, controlState_.anchor_x(),
            controlState_.anchor_y(), turnPositionHoldGain_,
            turnPositionHoldDeadband_, turnPositionHoldMaxSpeed_) *
        frame.fusion_health.speed_scale;
  } else {
    turnProgressSupervisor_.Reset();
    const double headingScale = std::max(0.0, std::cos(pathError));
    double linearSpeed = visual_navigation::CrossTrackSpeedLimit(
        requestedSpeed, std::abs(crossTrackError), crossTrackSlowdownStart_,
        crossTrackSlowdownFull_, crossTrackMinimumSpeed_);
    linearSpeed = visual_navigation::CurvatureSpeedLimit(
        linearSpeed, pathCurvature, maxLateralAcceleration_);
    const double approachDistance = visual_navigation::EndpointApproachDistance(
        frame.distance, frame.path_projection);
    if (frame.waypoint_requires_stop) {
      linearSpeed = visual_navigation::BrakingSpeedLimit(
          linearSpeed, approachDistance, effectiveBrakingDeceleration_,
          brakingControlDelay_, brakingSafetyMargin_);
      const double feedbackSpeed =
          inputCache_.odometry_velocity_valid()
              ? std::abs(inputCache_.odometry_velocity())
              : std::abs(lastMotionCommand_.linear.x);
      linearSpeed = visual_navigation::BrakingFeedbackSpeedLimit(
          linearSpeed, feedbackSpeed, approachDistance,
          effectiveBrakingDeceleration_, brakingControlDelay_,
          brakingSafetyMargin_, brakingDistanceFeedbackGain_);
    }
    command.linear.x =
        linearSpeed * headingScale * frame.fusion_health.speed_scale;
    const double feedforwardAngularSpeed =
        visual_navigation::CurvatureFeedforwardAngularSpeed(
            command.linear.x, signedPathCurvature,
            pathCurvatureFeedforwardGain_);
    const double scaledMaxPathAngularSpeed =
        maxPathAngularSpeed_ * frame.fusion_health.speed_scale;
    const double lateralAngularLimit =
        visual_navigation::LateralAccelerationAngularLimit(
            scaledMaxPathAngularSpeed, command.linear.x,
            maxLateralAcceleration_);
    command.angular.z = visual_navigation::CompensatePathTurnDeadband(
        feedforwardAngularSpeed + pathFeedbackAngularSpeed, pathError,
        frame.control_yaw_rate, pathAngularActivationError_,
        pathYawResponseThreshold_,
        minPathAngularSpeed_ * frame.fusion_health.speed_scale,
        lateralAngularLimit);

    const double measuredSpeed = inputCache_.odometry_velocity_valid()
                                     ? std::abs(inputCache_.odometry_velocity())
                                     : std::abs(lastMotionCommand_.linear.x);
    const double stoppingDistance = visual_navigation::BrakingDistance(
        measuredSpeed, effectiveBrakingDeceleration_, brakingControlDelay_,
        brakingSafetyMargin_);
    brakingApproach = frame.waypoint_requires_stop &&
                      approachDistance <= stoppingDistance &&
                      command.linear.x > 0.0;
  }

  if (!aligningPath &&
      SupervisePathProgress(
          segmentProgress,
          !brakingApproach && frame.progress_supervision_allowed,
          lastMotionCommand_.linear.x,
          inputCache_.odometry_velocity(),
          inputCache_.odometry_velocity_valid(),
          !brakingApproach)) {
    return;
  }
  PublishMotionCommand(command);
  if (aligningPath)
    SetState(visual_navigation::phase_status(controlState_.phase()));
  else
    SetState(brakingApproach ? "BRAKING_APPROACH" : "FOLLOWING");
}
