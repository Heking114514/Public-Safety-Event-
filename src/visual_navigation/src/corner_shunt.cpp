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
  context.measured_twist = {
    inputCache_.odometry_velocity_valid() ? inputCache_.odometry_velocity() :
    lastMotionCommand_.linear.x,
    ControlYawRate()};
  context.measured_yaw_rate_valid = ImuYawRateIsFresh();
  context.junction_x = junction.x;
  context.junction_y = junction.y;
  context.geometry_heading = stoppedAtPlannedTurn_
                                 ? stoppedTurnGeometryHeading_
                                 : frame.path_heading;
  context.arrival_heading = stoppedAtPlannedTurn_
                                ? stoppedTurnArrivalHeading_
                                : frame.path_heading;
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

  const auto context = BuildCornerShuntContext(frame);
  visual_navigation::TurnManeuverInput input;
  input.context = context;
  const auto decision = turnManeuverController_.update(input, SteadySeconds());
  if (decision.trim_target_selected) {
    RCLCPP_INFO(get_logger(),
                "Turn trim target selected: attempt=%zu/%d target_local_x=%.3f "
                "candidates=%zu footprint_rejected=%zu turn_rejected=%zu",
                decision.trim_attempts,
                turnManeuverConfig_.max_trim_attempts,
                decision.trim_target_local_x,
                decision.trim_candidates_checked,
                decision.trim_footprint_rejections,
                decision.trim_turn_rejections);
  }
  if (decision.phase == visual_navigation::TurnManeuverPhase::REVERSE_TRIM ||
      decision.phase == visual_navigation::TurnManeuverPhase::FORWARD_TRIM) {
    const double cosine = std::cos(context.geometry_heading);
    const double sine = std::sin(context.geometry_heading);
    const double local_x = cosine * (context.pose.x - context.junction_x) +
                           sine * (context.pose.y - context.junction_y);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Turn trim control: phase=%s direction=%d switches=%zu vx=%.3f "
        "target_local_x=%.3f local_x=%.3f longitudinal_error=%.3f",
        visual_navigation::TurnManeuverStatus(decision.phase),
        decision.trim_direction, decision.trim_direction_switches,
        decision.command.linear_x, decision.trim_target_local_x, local_x,
        decision.trim_target_local_x - local_x);
  }
  if (decision.has_safety_report) {
    const auto &report = decision.safety_report;
    RCLCPP_WARN(
        get_logger(),
        "Turn footprint rejected: source=%s reason=%s prediction_sample=%d "
        "corner=%d corner_local=(%.3f, %.3f) overlap=%.3f "
        "checked_pose=(%.3f, %.3f, %.1fdeg) attempted=(%.3f, %.3f) "
        "geometry=%.1fdeg junction=(%.3f, %.3f)",
        report.source,
        visual_navigation::CornerShuntSafetyFailureName(report.failure),
        report.prediction_sample, report.footprint_corner,
        report.corner_local.x, report.corner_local.y, report.overlap_m,
        report.checked_pose.x, report.checked_pose.y,
        report.checked_pose.yaw * 180.0 / kPi,
        report.attempted_command.linear_x,
        report.attempted_command.angular_z,
        context.geometry_heading * 180.0 / kPi, context.junction_x,
        context.junction_y);
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
    navigationActive_ = false;
    RCLCPP_WARN(
        get_logger(),
        "Junction maneuver search ended: reason=%s attempts=%zu/%d "
        "candidates=%zu footprint_rejected=%zu turn_rejected=%zu "
        "base=(%.3f, %.3f, %.1fdeg); requesting route-level recovery",
        decision.trim_limit_reached ? "trim_limit" : "no_trim_candidate",
        decision.trim_attempts,
        turnManeuverConfig_.max_trim_attempts,
        decision.trim_candidates_checked,
        decision.trim_footprint_rejections,
        decision.trim_turn_rejections,
        currentX_, currentY_, currentYaw_ * 180.0 / kPi);
    PublishStop();
    SetState("TURN_NEED_REPLAN");
    return;
  }
  if (decision.result == visual_navigation::TurnManeuverResult::FAULT ||
      decision.result == visual_navigation::TurnManeuverResult::CANCELED) {
    navigationActive_ = false;
    PublishStop();
    SetState(decision.result == visual_navigation::TurnManeuverResult::FAULT
                 ? "FAULT_TURN_MANEUVER"
                 : "TURN_CANCELED");
    return;
  }
  if (std::abs(decision.command.linear_x) <= 1.0e-9 &&
      std::abs(decision.command.angular_z) <= 1.0e-9) {
    PublishStop();
    SetState(visual_navigation::TurnManeuverStatus(decision.phase));
    return;
  }

  geometry_msgs::msg::Twist command;
  command.linear.x = decision.command.linear_x;
  command.angular.z = decision.command.angular_z;
  PublishMotionCommand(command);
  SetState(visual_navigation::TurnManeuverStatus(decision.phase));
}
