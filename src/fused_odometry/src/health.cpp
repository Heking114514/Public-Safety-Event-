#include "fused_odometry/fusion_gate_node.hpp"

namespace fused_odometry {

void FusionGateNode::publish_status() {
  if (visual_.accepted && !vision_healthy()) {
    mark_visual_interrupted();
  }
  const bool vision = vision_healthy();
  const bool command_fresh = command_.input.fresh(command_timeout_);
  const bool wheel_measurement_fresh = wheel_.input.fresh(wheel_timeout_);
  const bool raw_visual_motion_valid = raw_visual_increment_healthy();
  const bool accepted_visual_motion_valid =
      vision && visual_vx_window_.ready(robust_min_samples_);
  const bool visual_motion_valid =
      raw_visual_motion_valid || accepted_visual_motion_valid;
  const double visual_motion_velocity =
      raw_visual_motion_valid
          ? raw_visual_vx_window_.median()
          : (accepted_visual_motion_valid ? visual_vx_window_.median()
                                          : visual_.forward_velocity);
  const double current_time = steady_seconds();
  const double wheel_motion_velocity =
      wheel_vx_window_.ready(robust_min_samples_) ? wheel_vx_window_.median()
                                                  : wheel_.velocity;
  const bool imu = imu_healthy();
  const bool wheel_yaw_backup =
      fuse_wheel_yaw_ && wheel_.yaw.validated &&
      current_time - wheel_.yaw.last_validated_at <= wheel_yaw_backup_time_ &&
      std::abs(wheel_.velocity) <= max_wheel_only_speed_;
  FusionHealthInput health_input;
  health_input.now_seconds = current_time;
  health_input.initialized = visual_.ever_accepted;
  health_input.initialization_timed_out =
      !health_input.initialized &&
      current_time - initialization_started_at_ >= initialization_timeout_;
  health_input.vision = vision;
  health_input.wheel_measurement_fresh = wheel_measurement_fresh;
  health_input.wheel_available =
      wheel_measurement_fresh && !wheel_gate_.rejected();
  health_input.imu = imu;
  health_input.wheel_yaw_backup = wheel_yaw_backup;
  health_input.visual_realigned = current_time <= visual_.realigned_until;
  health_input.command_velocity = command_.velocity;
  health_input.command_yaw_rate = command_.yaw_rate;
  health_input.command_fresh = command_fresh;
  health_input.wheel_velocity = wheel_motion_velocity;
  health_input.visual_velocity = visual_motion_velocity;
  health_input.visual_motion_valid = visual_motion_valid;
  health_input.visual_yaw_rate = visual_.yaw_rate;
  health_input.visual_yaw_valid = vision && visual_.velocity_valid;
  health_input.imu_yaw_rate = imu_.yaw_rate;
  health_input.wheel_yaw_rate = wheel_.yaw.rate;
  health_input.wheel_yaw_valid =
      fuse_wheel_yaw_ && wheel_.yaw.rate_finite && wheel_.yaw.validated;
  const FusionHealthSnapshot health = health_monitor_.evaluate(
      health_input,
      FusionHealthLimits{max_dead_reckoning_time_, max_dead_reckoning_distance_,
                         max_wheel_only_time_, max_wheel_only_distance_});
  const bool wheel = health.wheel_healthy;
  const FusionMode mode = health.mode;

  std_msgs::msg::String status;
  status.data = mode_name(mode);
  status_publisher_->publish(status);

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = now();
  diagnostic_msgs::msg::DiagnosticStatus item;
  item.name = "fused_odometry/health";
  item.hardware_id = "odometry_fusion";
  switch (health_severity(mode, wheel, health.motion_fault)) {
  case HealthSeverity::kOk:
    item.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    break;
  case HealthSeverity::kWarning:
    item.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    break;
  case HealthSeverity::kError:
    item.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    break;
  }
  item.message = mode_name(mode);
  if (health.motion_fault != MotionFault::kNone) {
    item.message +=
        std::string(" (") + motion_fault_name(health.motion_fault) + ")";
  } else if (!wheel) {
    item.message += " (WHEEL_UNAVAILABLE)";
  }
  item.values.push_back(value("vision", vision ? "healthy" : "unavailable"));
  item.values.push_back(value(
      "initialization", health_input.initialized
          ? "complete"
          : (health_input.initialization_timed_out ? "timed_out" : "waiting")));
  item.values.push_back(value("wheel", wheel ? "healthy" : "unavailable"));
  item.values.push_back(value("imu", imu ? "healthy" : "unavailable"));
  item.values.push_back(
      value("tracking_state", std::to_string(tracking_.code)));
  item.values.push_back(value("visual_recovery_samples",
                              std::to_string(visual_.recovery_samples)));
  item.values.push_back(
      value("wheel_visual_residual_mps", std::to_string(wheel_.residual)));
  item.values.push_back(
      value("imu_visual_residual_radps", std::to_string(imu_.visual_residual)));
  item.values.push_back(
      value("imu_yaw_bias_radps", std::to_string(imu_.bias_estimator.bias())));
  item.values.push_back(value("visual_yaw_disagreement_scale",
                              std::to_string(visual_.yaw_disagreement_scale)));
  item.values.push_back(
      value("wheel_imu_yaw_residual_radps",
            std::to_string(wheel_imu_yaw_residual_window_.median())));
  item.values.push_back(
      value("wheel_visual_residual_mad_mps",
            std::to_string(wheel_visual_residual_window_.mad())));
  item.values.push_back(
      value("motion_fault", motion_fault_name(health.motion_fault)));
  item.values.push_back(
      value("command_vx_mps", std::to_string(command_.velocity)));
  item.values.push_back(
      value("command_wz_radps", std::to_string(command_.yaw_rate)));
  item.values.push_back(
      value("measured_wz_radps", std::to_string(health.measured_yaw_rate)));
  item.values.push_back(
      value("angular_sources", std::to_string(health.angular_source_count)));
  item.values.push_back(
      value("angular_stalled", health.angular_stalled ? "true" : "false"));
  item.values.push_back(
      value("wheel_yaw_validated", wheel_.yaw.validated ? "true" : "false"));
  item.values.push_back(
      value("wheel_yaw_excited", wheel_.yaw.excited ? "true" : "false"));
  item.values.push_back(
      value("wheel_yaw_instantly_consistent",
            wheel_.yaw.instantly_consistent ? "true" : "false"));
  item.values.push_back(
      value("wheel_yaw_fused", fuse_wheel_yaw_ ? "true" : "false"));
  item.values.push_back(
      value("wheel_yaw_backup", wheel_yaw_backup ? "true" : "false"));
  item.values.push_back(value("wheel_vx_turn_covariance_scale",
                              std::to_string(wheel_.vx_turn_scale)));
  item.values.push_back(
      value("wheel_vx_zeroed", wheel_.vx_zeroed ? "true" : "false"));
  item.values.push_back(
      value("raw_visual_increment",
            raw_visual_increment_healthy() ? "healthy" : "unavailable"));
  item.values.push_back(
      value("raw_visual_vx_mps", std::to_string(raw_visual_.forward_velocity)));
  item.values.push_back(value("raw_visual_used", "true"));
  item.values.push_back(
      value("visual_realign_count", std::to_string(visual_.realign_count)));
  item.values.push_back(value("visual_recovery_rejected",
                              std::to_string(visual_.rejected_recoveries)));
  item.values.push_back(
      value("map_change_sequence", std::to_string(map_change_.sequence)));
  item.values.push_back(
      value("map_change_grace",
            steady_seconds() <= map_change_.grace_until ? "true" : "false"));
  item.values.push_back(
      value("vision_outage_s", std::to_string(health.vision_outage_seconds)));
  item.values.push_back(value("dead_reckoning_distance_m",
                              std::to_string(health.dead_reckoning_distance)));
  item.values.push_back(
      value("wheel_frame_approximation",
            wheel_expected_child_frame_ == base_frame_ ? "false" : "true"));
  item.values.push_back(
      value("invalid_raw_visual", std::to_string(raw_visual_.invalid_samples)));
  item.values.push_back(
      value("invalid_wheel", std::to_string(wheel_.invalid_samples)));
  item.values.push_back(
      value("invalid_imu", std::to_string(imu_.invalid_samples)));
  diagnostics.status.push_back(item);
  diagnostics_publisher_->publish(diagnostics);

  if (mode != last_mode_) {
    RCLCPP_WARN(get_logger(), "Fusion mode: %s", mode_name(mode));
    last_mode_ = mode;
  }
}

} // namespace fused_odometry
