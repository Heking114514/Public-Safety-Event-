#include "imu_rpy_filter/imu_rpy_filter_node.hpp"

namespace imu_rpy_filter {

void ImuRpyFilterNode::publish_outputs(
    const sensor_msgs::msg::Imu &input, const Vector3 &gyro,
    const Vector3 &accel, const Matrix3 &gyro_covariance,
    const Matrix3 &accel_covariance, bool stationary_like,
    bool stationary_exit_pending, double corrected_yaw_rate) {
  std_msgs::msg::Header header = input.header;
  header.frame_id = output_frame_;

  geometry_msgs::msg::Vector3Stamped rpy;
  rpy.header = header;
  rpy.vector.x = roll_;
  rpy.vector.y = pitch_;
  rpy.vector.z = yaw_filter_.yaw();
  rpy_publisher_->publish(rpy);

  geometry_msgs::msg::Vector3Stamped rpy_degrees = rpy;
  rpy_degrees.vector.x = degrees(roll_);
  rpy_degrees.vector.y = degrees(pitch_);
  rpy_degrees.vector.z = degrees(yaw_filter_.yaw());
  rpy_degrees_publisher_->publish(rpy_degrees);

  sensor_msgs::msg::Imu filtered;
  filtered.header = header;
  filtered.orientation = quaternion_from_rpy(roll_, pitch_, yaw_filter_.yaw());
  filtered.orientation_covariance = {
      roll_pitch_variance_,
      0.0,
      0.0,
      0.0,
      roll_pitch_variance_,
      0.0,
      0.0,
      0.0,
      std::max(yaw_filter_.yaw_variance(),
               stationary_yaw_std_ * stationary_yaw_std_)};
  filtered.angular_velocity.x = gyro[0];
  filtered.angular_velocity.y = gyro[1];
  // Stationarity only changes uncertainty and bias learning. Publishing the
  // measured rate avoids hiding a real turn during stationary debounce.
  filtered.angular_velocity.z = corrected_yaw_rate;
  filtered.angular_velocity_covariance = gyro_covariance;
  filtered.angular_velocity_covariance[8] = yaw_rate_variance(
      stationary_like, stationary_exit_pending, stationary_yaw_rate_variance_,
      moving_yaw_rate_variance_);
  filtered.linear_acceleration.x = accel[0];
  filtered.linear_acceleration.y = accel[1];
  filtered.linear_acceleration.z = accel[2];
  filtered.linear_acceleration_covariance = accel_covariance;
  filtered_imu_publisher_->publish(filtered);
}

diagnostic_msgs::msg::KeyValue
ImuRpyFilterNode::diagnostic_value(const std::string &key,
                                   const std::string &value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

} // namespace imu_rpy_filter
