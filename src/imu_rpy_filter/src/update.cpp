#include "imu_rpy_filter/imu_rpy_filter_node.hpp"

namespace imu_rpy_filter {

void ImuRpyFilterNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr message) {
  if (!valid_ros_time_representation(message->header.stamp.sec,
                                     message->header.stamp.nanosec)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejecting IMU with invalid timestamp fields");
    return;
  }
  const double stamp = rclcpp::Time(message->header.stamp).seconds();
  Vector3 gyro = {message->angular_velocity.x, message->angular_velocity.y,
                  message->angular_velocity.z};
  Vector3 accel = {message->linear_acceleration.x,
                   message->linear_acceleration.y,
                   message->linear_acceleration.z};
  Matrix3 gyro_covariance = message->angular_velocity_covariance;
  Matrix3 accel_covariance = message->linear_acceleration_covariance;

  if (!std::isfinite(stamp) || stamp <= 0.0 ||
      !accepted_imu_frame(message->header.frame_id, input_frame_) ||
      !finite_vector(gyro) || !finite_vector(accel) ||
      !finite_matrix(gyro_covariance) || !finite_matrix(accel_covariance)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting IMU with invalid timestamp, frame, value or covariance");
    return;
  }

  double dt = 0.0;
  if (initialized_) {
    if (!update_imu_time_baseline(stamp, last_stamp_, dt)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting IMU dt %.6f seconds and resetting timestamp baseline", dt);
      return;
    }
  }

  const bool input_is_optical =
      transform_optical_frame_ && message->header.frame_id.size() >= 14 &&
      message->header.frame_id.compare(message->header.frame_id.size() - 14, 14,
                                       "_optical_frame") == 0;
  const bool optical_frame =
      message->header.frame_id.size() >= 14 &&
      message->header.frame_id.compare(message->header.frame_id.size() - 14, 14,
                                       "_optical_frame") == 0;
  if (optical_frame && !transform_optical_frame_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Rejecting optical-frame IMU because "
                         "transform_optical_frame is disabled");
    return;
  }
  if (input_is_optical) {
    gyro = optical_to_body(gyro);
    accel = optical_to_body(accel);
    gyro_covariance = optical_covariance_to_body(gyro_covariance);
    accel_covariance = optical_covariance_to_body(accel_covariance);
  }

  gyro = mean_gyro_.update(median_gyro_.update(gyro));
  accel = mean_accel_.update(median_accel_.update(accel));

  if (!initialized_) {
    gyro = low_pass_gyro_.update(gyro, 0.005);
    accel = low_pass_accel_.update(accel, 0.005);
    roll_ = std::atan2(accel[1], accel[2]);
    pitch_ = std::atan2(-accel[0], std::hypot(accel[1], accel[2]));
    last_stamp_ = stamp;
    initialized_ = true;
    publish_outputs(*message, gyro, accel, gyro_covariance, accel_covariance,
                    false, false, euler_yaw_rate(gyro, roll_, pitch_));
    return;
  }

  gyro = low_pass_gyro_.update(gyro, dt);
  accel = low_pass_accel_.update(accel, dt);
  const double accel_norm = vector_norm(accel);
  const double roll_accel = std::atan2(accel[1], accel[2]);
  const double pitch_accel =
      std::atan2(-accel[0], std::hypot(accel[1], accel[2]));

  double cos_pitch = std::cos(pitch_);
  if (std::abs(cos_pitch) < 1e-3) {
    cos_pitch = std::copysign(1e-3, cos_pitch);
  }
  const double tan_pitch = std::sin(pitch_) / cos_pitch;
  const double roll_rate = gyro[0] + std::sin(roll_) * tan_pitch * gyro[1] +
                           std::cos(roll_) * tan_pitch * gyro[2];
  const double pitch_rate =
      std::cos(roll_) * gyro[1] - std::sin(roll_) * gyro[2];
  // Convert the camera-frame angular velocity to Euler yaw rate. The D455 is
  // mounted with a fixed pitch, so using gyro.z directly undercounts turns.
  const double yaw_rate = euler_yaw_rate(gyro, roll_, pitch_);

  roll_ = wrap_angle(roll_ + roll_rate * dt);
  pitch_ = wrap_angle(pitch_ + pitch_rate * dt);
  yaw_filter_.predict(yaw_rate, dt);

  const double acceleration_error = std::abs(accel_norm - kGravity);
  const double acceleration_confidence =
      std::clamp((0.75 - acceleration_error) / 0.5, 0.0, 1.0);
  if (acceleration_confidence > 0.0) {
    const double gyro_weight = correction_time_ / (correction_time_ + dt);
    const double accel_weight = (1.0 - gyro_weight) * acceleration_confidence;
    roll_ = wrap_angle(roll_ + accel_weight * wrap_angle(roll_accel - roll_));
    pitch_ =
        wrap_angle(pitch_ + accel_weight * wrap_angle(pitch_accel - pitch_));
  }

  const double corrected_yaw_rate =
      bias_corrected_yaw_rate(yaw_rate, yaw_filter_.bias());
  const double angular_rate_norm =
      std::sqrt(gyro[0] * gyro[0] + gyro[1] * gyro[1] +
                corrected_yaw_rate * corrected_yaw_rate);
  update_external_motion_diagnostics();
  const bool stationary =
      stationary_detector_.update(stamp, acceleration_error, angular_rate_norm);
  const bool stationary_like =
      stationary || stationary_detector_.has_candidate();
  const bool imu_quiet =
      acceleration_error <= 0.35 && angular_rate_norm <= 0.05;
  if (stationary_like && !stationary_detector_.exit_pending() && imu_quiet) {
    stationary_bias_window_.add(stamp, yaw_rate);
  } else if (!stationary_like) {
    stationary_bias_window_.clear();
  }
  apply_stationary_update(stamp, stationary);
  apply_visual_update();
  const double output_yaw_rate =
      bias_corrected_yaw_rate(yaw_rate, yaw_filter_.bias());
  publish_outputs(*message, gyro, accel, gyro_covariance, accel_covariance,
                  stationary_like, stationary_detector_.exit_pending(),
                  output_yaw_rate);

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                       "RPY deg [%.3f %.3f %.3f], bias %.3f deg/s, source=%s",
                       degrees(roll_), degrees(pitch_),
                       degrees(yaw_filter_.yaw()), degrees(yaw_filter_.bias()),
                       correction_source_.c_str());
}

} // namespace imu_rpy_filter
