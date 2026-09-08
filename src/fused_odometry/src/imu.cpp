#include "fused_odometry/fusion_gate_node.hpp"

namespace fused_odometry {

void FusionGateNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr message) {
  const double yaw_rate = message->angular_velocity.z;
  if (message->header.frame_id != imu_expected_frame_ ||
      !FiniteAndWithin(yaw_rate, max_imu_yaw_rate_)) {
    ++imu_.invalid_samples;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting IMU with invalid wz/frame (wz=%.3f, %s; expected %s)",
        yaw_rate, message->header.frame_id.c_str(),
        imu_expected_frame_.c_str());
    return;
  }
  if (!accept_measurement_stamp(message->header.stamp, imu_.stamp, imu_timeout_,
                                "IMU")) {
    ++imu_.invalid_samples;
    return;
  }
  const bool command_fresh = command_.input.fresh(command_timeout_);
  const bool raw_visual_rate_ready =
      raw_visual_increment_healthy() &&
      raw_visual_wz_window_.ready(robust_min_samples_);
  const bool accepted_visual_rate_ready =
      visual_.velocity_valid && vision_healthy() &&
      visual_wz_window_.ready(robust_min_samples_);
  const bool visual_rate_ready =
      raw_visual_rate_ready || accepted_visual_rate_ready;
  const double visual_rate_reference =
      visual_rate_ready
          ? (raw_visual_rate_ready ? raw_visual_wz_window_.median()
                                   : visual_wz_window_.median())
          : visual_.yaw_rate;
  const bool learn_bias =
      visual_rate_ready && command_fresh &&
      std::abs(command_.yaw_rate) <= imu_bias_learning_command_rate_ &&
      std::abs(visual_rate_reference) <= imu_bias_learning_visual_rate_;
  imu_.yaw_rate = imu_.bias_estimator.correct(steady_seconds(), yaw_rate,
                                              visual_rate_reference,
                                              visual_rate_ready, learn_bias);
  double residual = 0.0;
  bool compared = false;
  if (visual_.velocity_valid && vision_healthy()) {
    imu_visual_residual_window_.add(steady_seconds(),
                                    std::abs(imu_.yaw_rate - visual_.yaw_rate));
    residual = imu_visual_residual_window_.median();
    imu_.robust_visual_residual =
        residual + 1.4826 * imu_visual_residual_window_.mad();
    compared = true;
  }
  imu_.input.update();
  imu_.visual_residual = compared ? residual : 0.0;
  if (!visual_.ever_accepted) {
    return;
  }

  sensor_msgs::msg::Imu output;
  output.header = message->header;
  output.header.frame_id = base_frame_;
  output.orientation_covariance[0] = -1.0;
  output.linear_acceleration_covariance[0] = -1.0;
  output.angular_velocity.z = imu_.yaw_rate;
  output.angular_velocity_covariance.fill(0.0);
  output.angular_velocity_covariance[0] = 1.0e6;
  output.angular_velocity_covariance[4] = 1.0e6;
  const double source_variance = message->angular_velocity_covariance[8];
  const double base_variance = finite(source_variance) && source_variance > 0.0
                                   ? std::max(imu_wz_variance_, source_variance)
                                   : imu_wz_variance_;
  output.angular_velocity_covariance[8] = base_variance;
  imu_publisher_->publish(output);
}

} // namespace fused_odometry
