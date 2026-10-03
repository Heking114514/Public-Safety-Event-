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
  context.junction_x = junction.x;
  context.junction_y = junction.y;
  context.incoming_heading =
      stoppedAtPlannedTurn_ ? stoppedTurnArrivalHeading_ : frame.path_heading;
  context.target_heading =
      stoppedAtPlannedTurn_ ? stoppedTurnTargetHeading_ : frame.path_heading;
  return context;
}

void WaypointNavigator::RunCornerShunt(const ControlFrame &frame,
                                       double segmentProgress) {
  (void)segmentProgress;
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();

  const auto decision = cornerShuntController_.update(
      BuildCornerShuntContext(frame), SteadySeconds());
  if (decision.complete) {
    ResetPathFeedback();
    cornerShuntController_.reset();
    controlState_.finish_path_alignment();
    stoppedAtPlannedTurn_ = false;
    firstHalfTurnPending_ = false;
    SetState(visual_navigation::CornerShuntStatus(decision.phase));
    return;
  }
  if (decision.blocked) {
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
