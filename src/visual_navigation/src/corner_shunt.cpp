#include "visual_navigation/waypoint_navigator.hpp"

#include <cmath>

namespace {

double SteadySeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

} // namespace

visual_navigation::CornerShuntContext
WaypointNavigator::BuildCornerShuntContext(const ControlFrame &frame) const {
  std::size_t junctionIndex = currentWaypointIndex_;
  if (lastStoppedTurnIndex_ < waypoints_.size()) {
    junctionIndex = lastStoppedTurnIndex_;
  } else if (currentWaypointIndex_ > 0) {
    junctionIndex = currentWaypointIndex_ - 1;
  }
  const Waypoint &junction = waypoints_[junctionIndex];

  visual_navigation::CornerShuntContext context;
  const double cos_yaw = std::cos(currentYaw_);
  const double sin_yaw = std::sin(currentYaw_);
  context.pose = {currentX_ - cos_yaw * cornerShuntConfig_.base_offset_x +
                      sin_yaw * cornerShuntConfig_.base_offset_y,
                  currentY_ - sin_yaw * cornerShuntConfig_.base_offset_x -
                      cos_yaw * cornerShuntConfig_.base_offset_y,
                  currentYaw_};
  context.applied_twist = {lastMotionCommand_.linear.x,
                           lastMotionCommand_.angular.z};
  context.junction_x = junction.x;
  context.junction_y = junction.y;
  context.incoming_heading =
      stoppedAtPlannedTurn_ ? stoppedTurnArrivalHeading_ : frame.path_heading;
  context.target_heading =
      stoppedAtPlannedTurn_ ? stoppedTurnTargetHeading_ : frame.path_heading;
  context.reverse_arrival = stoppedTurnReverseArrival_;
  return context;
}

void WaypointNavigator::RunCornerShunt(const ControlFrame &frame,
                                       double segmentProgress) {
  (void)segmentProgress;
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();

  const auto decision = cornerShuntController_.update(
      BuildCornerShuntContext(frame), SteadySeconds());
  if (state_ == "TURN_CHECK" &&
      decision.phase != visual_navigation::CornerShuntPhase::CHECK &&
      !decision.blocked) {
    RCLCPP_INFO(get_logger(),
                "Corner shunt plan ready: commands=%zu plans=%d "
                "base=(%.3f, %.3f, %.1fdeg)",
                cornerShuntController_.planned_command_count(),
                cornerShuntController_.plan_count(), currentX_, currentY_,
                currentYaw_ * 180.0 / kPi);
  }
  if (decision.complete) {
    ResetPathFeedback();
    cornerShuntController_.reset();
    controlState_.finish_path_alignment();
    stoppedAtPlannedTurn_ = false;
    stoppedTurnReverseArrival_ = false;
    firstHalfTurnPending_ = false;
    SetState(visual_navigation::CornerShuntStatus(decision.phase));
    return;
  }
  if (decision.blocked) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Corner shunt blocked after %d plan(s) at base=(%.3f, %.3f, %.1fdeg)",
        cornerShuntController_.plan_count(), currentX_, currentY_,
        currentYaw_ * 180.0 / kPi);
    PublishStop();
    SetState(visual_navigation::CornerShuntStatus(decision.phase));
    return;
  }
  if (decision.phase == visual_navigation::CornerShuntPhase::STOP ||
      decision.phase == visual_navigation::CornerShuntPhase::CHECK) {
    PublishStop();
    SetState(visual_navigation::CornerShuntStatus(decision.phase));
    return;
  }

  geometry_msgs::msg::Twist command;
  command.linear.x = decision.command.linear_x;
  command.angular.z = decision.command.angular_z;
  PublishMotionCommand(command);
  SetState(visual_navigation::CornerShuntStatus(decision.phase));
}
