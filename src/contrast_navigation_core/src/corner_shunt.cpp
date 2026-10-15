#include "visual_navigation/waypoint_navigator.hpp"

#include <cmath>

visual_navigation::CornerShuntContext
WaypointNavigator::BuildCornerShuntContext(const ControlFrame &frame) const {
  std::size_t junction_index = currentWaypointIndex_;
  if (lastStoppedTurnIndex_ < waypoints_.size())
    junction_index = lastStoppedTurnIndex_;
  else if (currentWaypointIndex_ > 0)
    junction_index = currentWaypointIndex_ - 1;
  const Waypoint &junction = waypoints_[junction_index];
  visual_navigation::CornerShuntContext context;
  const double c = std::cos(currentYaw_), s = std::sin(currentYaw_);
  context.pose = {
      currentX_ - c * trackingPointOffsetX_ + s * trackingPointOffsetY_,
      currentY_ - s * trackingPointOffsetX_ - c * trackingPointOffsetY_,
      currentYaw_};
  context.applied_twist = {lastMotionCommand_.linear.x,
                           lastMotionCommand_.angular.z};
  context.measured_twist = {inputCache_.odometry_velocity_valid()
                                ? inputCache_.odometry_velocity()
                                : lastMotionCommand_.linear.x,
                            ControlYawRate()};
  context.measured_yaw_rate_valid = ImuYawRateIsFresh();
  context.junction_x = junction.x;
  context.junction_y = junction.y;
  context.geometry_heading =
      stoppedAtPlannedTurn_ ? stoppedTurnGeometryHeading_ : frame.path_heading;
  context.arrival_heading =
      stoppedAtPlannedTurn_ ? stoppedTurnArrivalHeading_ : frame.path_heading;
  context.target_heading =
      stoppedAtPlannedTurn_ ? stoppedTurnTargetHeading_ : frame.path_heading;
  context.reverse_arrival = stoppedTurnReverseArrival_;
  return context;
}

void WaypointNavigator::RunCornerShunt(const ControlFrame &frame,
                                       double segmentProgress) {
  (void)segmentProgress;
  pathProgressSupervisor_.Pause();
  turnProgressSupervisor_.Pause();
  visual_navigation::TurnManeuverInput input;
  input.context = BuildCornerShuntContext(frame);
  if (normalTurnEntryStop_) {
    if (!RecoveryStopCompleted(ControlSeconds())) {
      input.motion_allowed = false;
      turnManeuverController_.update(input, ControlSeconds());
      return;
    }
    normalTurnEntryStop_ = false;
    if (!visual_navigation::FixedTurnEntryMatches(input.context, turnManeuverConfig_)) {
      navigationActive_ = false;
      PublishStop();
      turnManeuverController_.reset();
      SetState("FAULT_TURN_ENTRY_STOP_POSE");
      return;
    }
    RCLCPP_INFO(get_logger(), "Normal fixed entry stopped: actual=(%.4f,%.4f,%.4f)",
      input.context.pose.x,input.context.pose.y,input.context.pose.yaw);
  }
  if (poseRecoveryEnabled_ && !input.context.reverse_arrival &&
      poseRecovery_.junction_at({input.context.junction_x,input.context.junction_y}) ==
        visual_navigation::PoseRecovery::RoadJunction::CORNER) {
    lastCornerContext_ = input.context;
    lastCornerContextValid_ = true;
    const double h = input.context.target_heading;
    const double d = poseRecovery_.config().exit_distance;
    lastCornerExitLane_ = poseRecovery_.lane_at_pose({
      {input.context.junction_x+d*std::cos(h),input.context.junction_y+d*std::sin(h)},h});
  } else {
    lastCornerContextValid_ = false;
  }
  const auto decision = turnManeuverController_.update(input, ControlSeconds());
  if (decision.phase == visual_navigation::TurnManeuverPhase::CAPTURE)
    normalTurnEntryConfirmed_ = false;
  if (poseRecoveryEnabled_ && !normalTurnEntryConfirmed_ &&
      decision.phase == visual_navigation::TurnManeuverPhase::TURN_60 &&
      std::abs(decision.command.linear_x) <= 1e-9 &&
      std::abs(decision.command.angular_z) <= 1e-9) {
    normalTurnEntryStop_ = true;
    normalTurnEntryConfirmed_ = true;
    BeginRecoveryStop(ControlSeconds());
    SetState("TURN_ENTRY_STOP");
    return;
  }
  if (decision.result == visual_navigation::TurnManeuverResult::COMPLETED) {
    ResetPathFeedback();
    turnManeuverController_.reset();
    controlState_.finish_path_alignment();
    stoppedAtPlannedTurn_ = false;
    stoppedTurnReverseArrival_ = false;
    firstHalfTurnPending_ = false;
    SetState(visual_navigation::TurnManeuverStatus(decision.phase));
    return;
  }
  if (decision.result == visual_navigation::TurnManeuverResult::NEED_REPLAN) {
    PublishStop();
    navigationActive_ = false;
    routeReplacementRequiredForId_ = activeRouteId_;
    turnManeuverController_.reset();
    controlState_.interrupt_maneuver();
    stoppedAtPlannedTurn_ = false;
    stoppedTurnReverseArrival_ = false;
    firstHalfTurnPending_ = false;
    SetState("TURN_NEED_REPLAN");
    return;
  }
  if (decision.result == visual_navigation::TurnManeuverResult::FAULT ||
      decision.result == visual_navigation::TurnManeuverResult::CANCELED) {
    navigationActive_ = false;
    PublishStop();
    SetState(visual_navigation::TurnManeuverStatus(decision.phase));
    return;
  }
  if (std::abs(decision.command.linear_x) <= 1.0e-9 &&
      std::abs(decision.command.angular_z) <= 1.0e-9)
    PublishStop();
  else {
    geometry_msgs::msg::Twist command;
    command.linear.x = decision.command.linear_x;
    command.angular.z = decision.command.angular_z;
    PublishMotionCommand(command);
  }
  SetState(visual_navigation::TurnManeuverStatus(decision.phase));
}
