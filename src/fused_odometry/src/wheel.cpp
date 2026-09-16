#include "fused_odometry/fusion_gate_node.hpp"

namespace fused_odometry {

void FusionGateNode::wheel_callback(
    const nav_msgs::msg::Odometry::SharedPtr message) {
  const double velocity = message->twist.twist.linear.x;
  const double wheel_yaw_rate = message->twist.twist.angular.z;
  if (!FramePairMatches(message->header.frame_id, message->child_frame_id,
                        wheel_expected_frame_, wheel_expected_child_frame_) ||
      !FiniteAndWithin(velocity, max_wheel_speed_)) {
    ++wheel_.invalid_samples;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting wheel odometry with invalid vx/frame (%s -> %s; expected %s "
        "-> %s)",
        message->header.frame_id.c_str(), message->child_frame_id.c_str(),
        wheel_expected_frame_.c_str(), wheel_expected_child_frame_.c_str());
    return;
  }
  if (!accept_measurement_stamp(message->header.stamp, wheel_.stamp,
                                wheel_timeout_, "wheel")) {
    ++wheel_.invalid_samples;
    return;
  }

  const double sample_time = steady_seconds();
  wheel_.yaw.rate = wheel_yaw_rate;
  wheel_.yaw.rate_finite =
      finite(wheel_yaw_rate) && std::abs(wheel_yaw_rate) <= max_wheel_yaw_rate_;
  const bool imu_fresh = imu_.input.fresh(imu_timeout_);
  wheel_.vx_zeroed = zero_wheel_vx_during_in_place_turn(
      velocity, imu_.yaw_rate, true, imu_fresh,
      wheel_in_place_max_linear_speed_, wheel_in_place_min_yaw_rate_);
  const double effective_velocity = wheel_.vx_zeroed ? 0.0 : velocity;
  wheel_vx_window_.add(sample_time, effective_velocity);
  wheel_.velocity = effective_velocity;

  double residual = 0.0;
  bool compared = false;
  const bool raw_visual_reference = raw_visual_increment_healthy();
  const bool accepted_visual_reference =
      visual_vx_window_.ready(robust_min_samples_) && vision_healthy();
  if (raw_visual_reference || accepted_visual_reference) {
    const double visual_velocity = raw_visual_reference
                                       ? raw_visual_vx_window_.median()
                                       : visual_vx_window_.median();
    wheel_visual_residual_window_.add(
        sample_time, std::abs(effective_velocity - visual_velocity));
    residual = wheel_visual_residual_window_.median();
    const double rejection_residual = wheel_visual_rejection_residual(
        effective_velocity, visual_velocity, visual_stationary_speed_,
        stationary_wheel_reject_speed_);
    wheel_gate_.update(
        std::max(rejection_residual,
                 residual + 1.4826 * wheel_visual_residual_window_.mad()));
    compared = true;
  }
  wheel_.yaw.excited = false;
  wheel_.yaw.instantly_consistent = false;
  if (wheel_.yaw.rate_finite && imu_.input.fresh(imu_timeout_)) {
    wheel_.yaw.excited = yaw_rates_excited(wheel_yaw_rate, imu_.yaw_rate,
                                           wheel_yaw_validation_min_rate_);
    wheel_.yaw.instantly_consistent = yaw_rates_consistent(
        wheel_yaw_rate, imu_.yaw_rate, wheel_imu_yaw_reject_);
    const bool either_yaw_excited =
        std::abs(wheel_yaw_rate) >= wheel_yaw_validation_min_rate_ ||
        std::abs(imu_.yaw_rate) >= wheel_yaw_validation_min_rate_;
    if (wheel_.yaw.excited) {
      wheel_imu_yaw_residual_window_.add(
          sample_time, std::abs(wheel_yaw_rate - imu_.yaw_rate));
    }
    if (wheel_.yaw.excited && wheel_.yaw.instantly_consistent &&
        wheel_imu_yaw_residual_window_.ready(robust_min_samples_)) {
      const double robust_residual =
          wheel_imu_yaw_residual_window_.median() +
          1.4826 * wheel_imu_yaw_residual_window_.mad();
      if (robust_residual <= wheel_imu_yaw_reject_) {
        if (!wheel_.yaw.validation_active) {
          wheel_.yaw.validation_active = true;
          wheel_.yaw.validation_started_at = sample_time;
        }
        if (sample_time - wheel_.yaw.validation_started_at >=
            wheel_yaw_validation_time_) {
          wheel_.yaw.validated = true;
          wheel_.yaw.last_validated_at = sample_time;
        }
      } else {
        wheel_.yaw.validation_active = false;
        wheel_.yaw.validated = false;
      }
    } else if (either_yaw_excited) {
      wheel_.yaw.validation_active = false;
      wheel_.yaw.validated = false;
    } else if (!wheel_.yaw.validated) {
      // Zero-rate agreement is not evidence that the encoder yaw scale and
      // sign are valid. Require an actual turn before starting validation.
      wheel_.yaw.validation_active = false;
    }
  }
  wheel_.input.update();
  wheel_.residual = compared ? residual : 0.0;
  const bool stationary_visual_conflict =
      raw_visual_reference &&
      std::abs(raw_visual_vx_window_.median()) <= visual_stationary_speed_ &&
      std::abs(effective_velocity) > stationary_wheel_reject_speed_;
  if (wheel_gate_.rejected() || stationary_visual_conflict ||
      !visual_.ever_accepted ||
      health_monitor_.motion_fault() != MotionFault::kNone) {
    return;
  }

  nav_msgs::msg::Odometry output;
  output.header = message->header;
  output.header.frame_id = wheel_frame_;
  output.child_frame_id = base_frame_;
  output.pose.pose.orientation.w = 1.0;
  output.twist.twist.linear.x = effective_velocity;
  output.pose.covariance.fill(0.0);
  output.twist.covariance.fill(0.0);
  for (std::size_t index = 0; index < 6; ++index) {
    output.pose.covariance[index * 6 + index] = 1.0e6;
    output.twist.covariance[index * 6 + index] = 1.0e6;
  }
  const double residual_scale =
      compared ? wheel_gate_.covariance_scale(residual, wheel_soft_residual_)
               : 2.0;
  const double turn_scale = wheel_vx_turn_covariance_scale(
      imu_.yaw_rate, imu_fresh,
      wheel_turn_downweight_start_, wheel_turn_full_downweight_,
      wheel_turn_covariance_scale_);
  wheel_.vx_turn_scale = turn_scale;
  const double input_vx_variance = message->twist.covariance[0];
  const double base_vx_variance =
      finite(input_vx_variance) && input_vx_variance > 0.0
          ? std::max(wheel_vx_variance_, input_vx_variance)
          : wheel_vx_variance_;
  output.twist.covariance[0] = base_vx_variance * residual_scale * turn_scale;
  const bool wheel_yaw_backup =
      fuse_wheel_yaw_ && wheel_.yaw.validated &&
      sample_time - wheel_.yaw.last_validated_at <= wheel_yaw_backup_time_ &&
      std::abs(effective_velocity) <= max_wheel_only_speed_;
  const bool wheel_yaw_matches_live_imu =
      imu_healthy() && wheel_.yaw.instantly_consistent;
  if (fuse_wheel_yaw_ && wheel_.yaw.rate_finite && wheel_.yaw.validated &&
      (wheel_yaw_matches_live_imu || wheel_yaw_backup)) {
    output.twist.twist.angular.z = wheel_yaw_rate;
    const double input_wz_variance = message->twist.covariance[35];
    const double base_wz_variance =
        finite(input_wz_variance) && input_wz_variance > 0.0
            ? std::max(wheel_wz_variance_, input_wz_variance)
            : wheel_wz_variance_;
    const double yaw_residual =
        wheel_imu_yaw_residual_window_.ready(robust_min_samples_)
            ? wheel_imu_yaw_residual_window_.median()
            : wheel_imu_yaw_reject_;
    output.twist.covariance[35] =
        base_wz_variance *
        std::clamp(1.0 + std::pow(yaw_residual / wheel_imu_yaw_soft_, 2), 1.0,
                   100.0);
  }
  if (wheel_.vx_zeroed) {
    output.twist.twist.linear.x = 0.0;
    output.twist.covariance[0] = base_vx_variance * residual_scale;
  }
  wheel_publisher_->publish(output);
}

} // namespace fused_odometry
