#include "visual_navigation/waypoint_navigator.hpp"

void WaypointNavigator::RunRfidScan(const ControlFrame &frame) {
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  rfidScanActive_ = true;
  const double heading = frame.path_heading;
  const double passed =
      std::cos(heading) * (currentX_ - frame.target.x) +
      std::sin(heading) * (currentY_ - frame.target.y);
  if (passed + 1.0e-6 >= rfidScanMaxDistance_) {
    PublishStop();
    SetState("WAITING_FOR_RFID:" + std::to_string(currentWaypointIndex_));
    return;
  }

  geometry_msgs::msg::Twist command;
  const double scanDirection = reverseSegmentActive_ ? -1.0 : 1.0;
  command.linear.x = scanDirection * rfidScanSpeed_;
  const double vehicleHeading =
      NormalizeAngle(heading + (reverseSegmentActive_ ? kPi : 0.0));
  const double headingError = NormalizeAngle(vehicleHeading - currentYaw_);
  command.angular.z = Clamp(
      angularGain_ * headingError - turnYawRateDamping_ * frame.control_yaw_rate,
      -std::min(maxPathAngularSpeed_, 0.25),
      std::min(maxPathAngularSpeed_, 0.25));
  PublishMotionCommand(command);
  SetState("RFID_SCANNING:" + std::to_string(currentWaypointIndex_));
}

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
  // A discontinuous corner is handled by braking and ALIGN_PATH at the
  // waypoint. Applying its curvature over the whole incoming segment makes a
  // straight approach turn toward the following segment from the start.
  signedPathCurvature = visual_navigation::ContinuousTrackingCurvature(
      signedPathCurvature, frame.waypoint_requires_stop);
  const double pathCurvature = std::abs(signedPathCurvature);
  const double unscaledTrackingSpeed = visual_navigation::CurvatureSpeedLimit(
      visual_navigation::CrossTrackSpeedLimit(
          requestedSpeed, std::abs(crossTrackError), crossTrackSlowdownStart_,
          crossTrackSlowdownFull_, crossTrackMinimumSpeed_),
      pathCurvature, maxLateralAcceleration_);
  const double approachDistance = visual_navigation::EndpointApproachDistance(
      frame.distance, frame.path_projection);
  double limitedLinearSpeed = unscaledTrackingSpeed;
  if (frame.waypoint_requires_stop) {
    limitedLinearSpeed = visual_navigation::BrakingSpeedLimit(
        limitedLinearSpeed, approachDistance, effectiveBrakingDeceleration_,
        brakingControlDelay_, brakingSafetyMargin_);
    const double feedbackSpeed = inputCache_.odometry_velocity_valid()
                                     ? std::abs(inputCache_.odometry_velocity())
                                     : std::abs(lastMotionCommand_.linear.x);
    limitedLinearSpeed = visual_navigation::BrakingFeedbackSpeedLimit(
        limitedLinearSpeed, feedbackSpeed, approachDistance,
        effectiveBrakingDeceleration_, brakingControlDelay_,
        brakingSafetyMargin_, brakingDistanceFeedbackGain_);
  }
  double controlDt = 1.0 / controlFrequency_;
  if (motionCommandInitialized_) {
    const double measuredDt = std::chrono::duration<double>(
        ControlNow() - lastMotionCommandTime_).count();
    if (measuredDt > 0.0 && measuredDt <= 0.2)
      controlDt = measuredDt;
  }
  const double trackingSpeed = std::abs(visual_navigation::LimitRate(
      limitedLinearSpeed, lastMotionCommand_.linear.x, maxLinearAcceleration_,
      maxLinearDeceleration_, controlDt));
  const double pathError = visual_navigation::StanleyPathError(
      headingError, crossTrackError, crossTrackGain_, trackingSpeed,
      stanleySofteningSpeed_, maxCrossTrackCorrection_);
  // Only a completed stop at a planned corner authorizes a pivot. Once the
  // car starts down a lane, lateral error must not trigger another pivot.
  const double distanceFromTurn = currentWaypointIndex_ > 0
      ? std::hypot(currentX_ - waypoints_[currentWaypointIndex_ - 1].x,
                   currentY_ - waypoints_[currentWaypointIndex_ - 1].y)
      : std::numeric_limits<double>::infinity();
  const bool shouldRotateInPlace = visual_navigation::PlannedTurnMayPivot(
      stoppedAtPlannedTurn_, controlState_.path_aligned(), distanceFromTurn,
      headingError, std::max(0.70, rotateInPlaceThreshold_));
  const bool turningManeuverActive =
      controlState_.in(visual_navigation::ControlPhase::SHUNT_TURN) ||
      controlState_.in(visual_navigation::ControlPhase::ALIGN_PATH);

  if (!turningManeuverActive && shouldRotateInPlace) {
    ResetManeuver();
    const double plannedTurnError =
        NormalizeAngle(stoppedTurnTargetHeading_ - stoppedTurnArrivalHeading_);
    if (turnManeuverController_.enabled_for(plannedTurnError)) {
      controlState_.begin_corner_shunt(currentX_, currentY_);
      turnManeuverController_.begin(
          ControlSeconds());
    } else {
      controlState_.begin_path_alignment(currentX_, currentY_);
    }
    turnSettleController_.reset();
  } else if (!turningManeuverActive && !controlState_.path_aligned()) {
    controlState_.mark_path_aligned();
    stoppedAtPlannedTurn_ = false;
    firstHalfTurnPending_ = false;
  }

  const bool shunting =
      controlState_.in(visual_navigation::ControlPhase::SHUNT_TURN);
  const bool aligningPath =
      controlState_.in(visual_navigation::ControlPhase::ALIGN_PATH);
  const double alignmentError = NormalizeAngle(
      (firstHalfTurnPending_ ? firstHalfTurnYaw_ : frame.path_heading) -
      currentYaw_);
  const bool allowPathIntegral = !aligningPath &&
                                 std::abs(headingError) < 0.30 &&
                                 std::abs(crossTrackError) < 0.08;
  const double pathFeedbackAngularSpeed =
      UpdatePathPid(pathError, crossTrackError, allowPathIntegral,
                    frame.control_yaw_rate);
  geometry_msgs::msg::Twist command;
  bool brakingApproach = false;
  const double segmentProgress =
      frame.path_projection.valid
          ? CompletedRouteLengthBeforeCurrentSegment() +
                Clamp(frame.path_length - frame.path_projection.remaining, 0.0,
                      frame.path_length)
          : std::numeric_limits<double>::quiet_NaN();

  if (shunting) {
    RunCornerShunt(frame, segmentProgress);
    return;
  }

  if (!aligningPath && std::abs(pathError) > 0.70) {
    BeginJunctionRetreat();
    return;
  }

  if (aligningPath) {
    if (SupervisePathProgress(segmentProgress,
                              frame.progress_supervision_allowed,
                              0.0, 0.0, false, false)) {
      return;
    }
    if (SuperviseTurnProgress(0, std::abs(alignmentError),
                              turnProgressExpectedYawRate_,
                              frame.progress_supervision_allowed)) {
      return;
    }
    controlState_.ensure_anchor(currentX_, currentY_);
    if (std::abs(alignmentError) <= rotateInPlaceExitThreshold_)
      turnSettleController_.begin_settling();
    if (turnSettleController_.settling()) {
      PublishStop();
      const double turnActivity =
          std::max(std::abs(frame.control_yaw_rate),
                   std::abs(lastMotionCommand_.angular.z));
      const double telemetryAge = controlTelemetryReceived_
          ? std::chrono::duration<double>(ControlNow() -
                                          lastControlTelemetryArrival_).count()
          : std::numeric_limits<double>::infinity();
      const bool translationStopped = visual_navigation::TurnTranslationHasStopped(
          controlTelemetryReceived_, telemetryAge, controlTelemetryTimeout_,
          measuredLeftWheelSpeed_, measuredRightWheelSpeed_,
          pathTrackingController_.observed_speed(),
          pathTrackingController_.observed_speed_valid(), preTurnStopSpeed_);
      if (turnActivity > turnSettleYawRate_ || !translationStopped) {
        turnSettleController_.reset_timer();
        SetState(visual_navigation::phase_status(controlState_.phase()));
        return;
      }
      if (!turnSettleController_.update(ControlNow(),
                                        turnActivity, turnSettleYawRate_,
                                        turnSettleDwell_)) {
        SetState(visual_navigation::phase_status(controlState_.phase()));
        return;
      }
      if (std::abs(alignmentError) <= rotateInPlaceExitThreshold_) {
        if (firstHalfTurnPending_) {
          firstHalfTurnPending_ = false;
          turnSettleController_.reset();
          turnProgressSupervisor_.Reset();
          SetState("TURN_HALF_SETTLED");
          return;
        }
        ResetManeuver();
        turnProgressSupervisor_.Reset();
        controlState_.finish_path_alignment();
        stoppedAtPlannedTurn_ = false;
        SetState("PATH_ALIGNED");
        return;
      }
      turnSettleController_.reset();
      SetState(visual_navigation::phase_status(controlState_.phase()));
      return;
    }

    turnSettleController_.reset_timer();
    const double requestedAngularSpeed =
        Clamp(angularGain_ * alignmentError -
                  turnYawRateDamping_ * frame.control_yaw_rate,
              -turnCruiseSpeed_, turnCruiseSpeed_);
    const double minimumTurnSpeed = visual_navigation::SelectMinimumTurnSpeed(
        std::abs(alignmentError), precisionTurnThreshold_,
        minPrecisionTurnSpeed_, turnCruiseSpeed_);
    command.angular.z = visual_navigation::EnforceMinimumTurnSpeed(
        requestedAngularSpeed, alignmentError,
        minimumTurnSpeed, turnCruiseSpeed_);
  } else {
    turnProgressSupervisor_.Reset();
    const double headingScale = std::max(0.0, std::cos(pathError));
    command.linear.x = limitedLinearSpeed * headingScale;
    const double feedforwardAngularSpeed =
        visual_navigation::CurvatureFeedforwardAngularSpeed(
            command.linear.x, signedPathCurvature,
            pathCurvatureFeedforwardGain_);
    const double lateralAngularLimit =
        visual_navigation::LateralAccelerationAngularLimit(
            maxPathAngularSpeed_, command.linear.x,
            maxLateralAcceleration_);
    command.angular.z = visual_navigation::CompensatePathTurnDeadband(
        feedforwardAngularSpeed + pathFeedbackAngularSpeed, pathError,
        frame.control_yaw_rate, pathAngularActivationError_,
        pathYawResponseThreshold_,
        minPathAngularSpeed_,
        lateralAngularLimit);
    if (frame.waypoint_requires_stop) {
      // On a straight approach, a near-zero braking speed must not leave an
      // angular-only command that pivots the car before the junction.
      command.angular.z = visual_navigation::StopRequiredPathAngularSpeed(
          command.angular.z, command.linear.x, preTurnPathAngularRatio_);
    }

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
  PublishMotionCommand(command, aligningPath);
  if (aligningPath)
    SetState(visual_navigation::phase_status(controlState_.phase()));
  else
    SetState(brakingApproach ? "BRAKING_APPROACH" : "FOLLOWING");
}
