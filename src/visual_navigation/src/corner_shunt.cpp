#include "visual_navigation/waypoint_navigator.hpp"

#include <cmath>
#include <cstring>
#include <limits>

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
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Turn context: waypoint=%zu pose=(%.3f, %.3f, %.1fdeg) junction=(%.3f, %.3f) "
      "geometry=%.1fdeg arrival=%.1fdeg target=%.1fdeg reverse_arrival=%s "
      "measured=(%.3f, %.3f)",
      currentWaypointIndex_, context.pose.x, context.pose.y,
      context.pose.yaw * 180.0 / kPi, context.junction_x, context.junction_y,
      context.geometry_heading * 180.0 / kPi,
      context.arrival_heading * 180.0 / kPi,
      context.target_heading * 180.0 / kPi,
      context.reverse_arrival ? "true" : "false", context.measured_twist.linear_x,
      context.measured_twist.angular_z);
  visual_navigation::TurnManeuverInput input;
  input.context = context;
  const auto decision = turnManeuverController_.update(input, SteadySeconds());
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Turn local trajectory: points=%zu length=%.3f m index=%zu planned_commands=%zu",
      turnManeuverController_.local_trajectory_size(),
      turnManeuverController_.local_trajectory_length(),
      turnManeuverController_.local_trajectory_index(),
      turnManeuverController_.planned_command_count());
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Turn control: source=%s phase=%s command=(%.3f, %.3f) local_rejected=%s",
      visual_navigation::CornerShuntCommandSourceName(
          turnManeuverController_.last_command_source()),
      visual_navigation::TurnManeuverStatus(decision.phase),
      decision.command.linear_x, decision.command.angular_z,
      turnManeuverController_.local_trajectory_rejected() ? "true" : "false");
  if (turnManeuverController_.local_trajectory_rejected()) {
    const auto report = turnManeuverController_.local_trajectory_report();
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Turn local trajectory rejected: reason=%s prediction_sample=%d "
        "corner=%d checked_pose=(%.3f, %.3f, %.1fdeg) attempted=(%.3f, %.3f)",
        visual_navigation::CornerShuntSafetyFailureName(report.failure),
        report.prediction_sample, report.footprint_corner,
        report.checked_pose.x, report.checked_pose.y,
        report.checked_pose.yaw * 180.0 / kPi,
        report.attempted_command.linear_x, report.attempted_command.angular_z);
  }
  if (decision.trim_target_selected) {
    RCLCPP_INFO(get_logger(),
                "Turn trim target selected: attempt=%zu/%d target_local_x=%.3f "
                "candidates=%zu footprint_rejected=%zu turn_rejected=%zu "
                "safety_recovery=%zu/%zu",
                decision.trim_attempts,
                turnManeuverConfig_.max_trim_attempts,
                decision.trim_target_local_x,
                decision.trim_candidates_checked,
                decision.trim_footprint_rejections,
                decision.trim_turn_rejections,
                decision.safety_recovery_attempts,
                decision.safety_recovery_limit);
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
    // A safety report is carried through several control cycles while the
    // maneuver transitions to recovery. Log the first occurrence of a given
    // failure signature; pose samples may change slightly in the meantime.
    const bool same_report =
        lastTurnSafetyReportValid_ &&
        std::strcmp(report.source, lastTurnSafetyReport_.source) == 0 &&
        report.failure == lastTurnSafetyReport_.failure &&
        report.prediction_sample == lastTurnSafetyReport_.prediction_sample &&
        report.footprint_corner == lastTurnSafetyReport_.footprint_corner;
    if (!same_report) {
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
      lastTurnSafetyReport_ = report;
      lastTurnSafetyReportValid_ = true;
    }
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
    RCLCPP_WARN(
        get_logger(),
        "Junction maneuver search ended: reason=%s attempts=%zu/%d "
        "candidates=%zu footprint_rejected=%zu turn_rejected=%zu "
        "safety_recovery=%zu/%zu base=(%.3f, %.3f, %.1fdeg); "
        "requesting route-level recovery",
        decision.safety_recovery_limit_reached
            ? "safety_recovery_limit"
            : (decision.trim_limit_reached ? "trim_limit" : "no_trim_candidate"),
        decision.trim_attempts,
        turnManeuverConfig_.max_trim_attempts,
        decision.trim_candidates_checked,
        decision.trim_footprint_rejections,
        decision.trim_turn_rejections,
        decision.safety_recovery_attempts,
        decision.safety_recovery_limit,
        currentX_, currentY_, currentYaw_ * 180.0 / kPi);
    RCLCPP_WARN(get_logger(),
                "Turn recovery snapshot: junction=(%.3f, %.3f) geometry=%.1fdeg "
                "arrival=%.1fdeg target=%.1fdeg last_safety=%s checked_pose=(%.3f, %.3f, %.1fdeg)",
                context.junction_x, context.junction_y,
                context.geometry_heading * 180.0 / kPi,
                context.arrival_heading * 180.0 / kPi,
                context.target_heading * 180.0 / kPi,
                decision.has_safety_report
                    ? visual_navigation::CornerShuntSafetyFailureName(
                          decision.safety_report.failure)
                    : "none",
                decision.has_safety_report ? decision.safety_report.checked_pose.x
                                           : context.pose.x,
                decision.has_safety_report ? decision.safety_report.checked_pose.y
                                           : context.pose.y,
                decision.has_safety_report
                    ? decision.safety_report.checked_pose.yaw * 180.0 / kPi
                    : context.pose.yaw * 180.0 / kPi);
    // A failed turn leaves the vehicle at an intermediate heading. Replaying
    // a straight lane or sending this pose to the planner is unsafe. Reuse
    // the obstacle recovery trace: it is built from measured poses and the
    // reverse command is recomputed from the measured pose on every tick.
    const bool safe_anchor_valid = turnManeuverController_.last_safe_pose_valid();
    const auto safe_anchor = safe_anchor_valid
                                 ? turnManeuverController_.last_safe_pose()
                                 : visual_navigation::CornerShuntPose{currentX_, currentY_, currentYaw_};
    if (BeginJunctionRetreat()) {
      PublishNavigationRecoveryState(
          decision.safety_recovery_limit_reached ? "safety_recovery_limit"
                                                 : "turn_no_safe_action",
          safe_anchor, safe_anchor_valid);
      RCLCPP_WARN(
          get_logger(),
          "Turn recovery switched to closed-loop executed-trace replay; "
          "planner will be called only after the junction anchor is reached");
    }
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
  if (decision.phase == visual_navigation::TurnManeuverPhase::REVERSE_TRIM ||
      decision.phase == visual_navigation::TurnManeuverPhase::FORWARD_TRIM) {
    const double telemetry_age =
        controlTelemetryReceived_
            ? std::chrono::duration<double>(
                  std::chrono::steady_clock::now() -
                  lastControlTelemetryArrival_)
                  .count()
            : std::numeric_limits<double>::infinity();
    const double commanded_left =
        lastMotionCommand_.linear.x -
        0.5 * turnManeuverConfig_.wheel_track * lastMotionCommand_.angular.z;
    const double commanded_right =
        lastMotionCommand_.linear.x +
        0.5 * turnManeuverConfig_.wheel_track * lastMotionCommand_.angular.z;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Turn actuator: requested=(%.3f, %.3f) published=(%.3f, %.3f) "
        "wheel_command=(%.3f, %.3f) wheel_target=(%.3f, %.3f) "
        "wheel_measured=(%.3f, %.3f) "
        "telemetry_age=%.3f valid=%s",
        command.linear.x, command.angular.z, lastMotionCommand_.linear.x,
        lastMotionCommand_.angular.z, commanded_left, commanded_right,
        targetLeftWheelSpeed_,
        targetRightWheelSpeed_, measuredLeftWheelSpeed_, measuredRightWheelSpeed_,
        telemetry_age,
        (controlTelemetryReceived_ && telemetry_age <= controlTelemetryTimeout_)
            ? "true"
            : "false");
  }
  SetState(visual_navigation::TurnManeuverStatus(decision.phase));
}
