#include "fused_odometry/fusion_gate_node.hpp"

namespace fused_odometry {

void FusionGateNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr message) {
  const double yaw_rate = message->angular_velocity.z;
  if (message->header.frame_id != imu_expected_frame_ ||
      !ValidQuaternion(message->orientation) ||
      !FiniteAndWithin(yaw_rate, max_imu_yaw_rate_)) {
    ++imu_.invalid_samples;
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting ATT IMU with invalid yaw/frame/value (wz=%.3f, %s; expected %s)",
        yaw_rate, message->header.frame_id.c_str(),
        imu_expected_frame_.c_str());
    return;
  }
  if (!accept_measurement_stamp(message->header.stamp, imu_.stamp, imu_timeout_,
                                "IMU")) {
    ++imu_.invalid_samples;
    return;
  }
  imu_.yaw_rate = yaw_rate;
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

  sensor_msgs::msg::Imu output = *message;
  output.header = message->header;
  output.header.frame_id = base_frame_;
  output.orientation_covariance[0] = 1.0e6;
  output.orientation_covariance[4] = 1.0e6;
  const double source_orientation_variance =
      message->orientation_covariance[8];
  output.orientation_covariance[8] =
      finite(source_orientation_variance) && source_orientation_variance > 0.0
          ? source_orientation_variance
          : 0.0025;
  output.linear_acceleration_covariance[0] = -1.0;
  output.angular_velocity.x = 0.0;
  output.angular_velocity.y = 0.0;
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
