#include "imu_rpy_filter/imu_rpy_filter_node.hpp"

namespace imu_rpy_filter {

void ImuRpyFilterNode::publish_diagnostics() {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string(": yaw filter");
  status.hardware_id = "d455_imu";
  status.level = initialized_ ? diagnostic_msgs::msg::DiagnosticStatus::OK
                              : diagnostic_msgs::msg::DiagnosticStatus::WARN;
  status.message =
      !initialized_
          ? "waiting for IMU"
          : (stationary_detector_.exit_pending()
                 ? "stationary exit debounce"
                 : (stationary_detector_.active() ? "stationary bias tracking"
                                                  : "integrating yaw rate"));
  status.values.push_back(diagnostic_value(
      "stationary", stationary_detector_.active() ? "true" : "false"));
  status.values.push_back(diagnostic_value(
      "stationary_candidate",
      stationary_detector_.has_candidate() ? "true" : "false"));
  status.values.push_back(
      diagnostic_value("stationary_exit_pending",
                       stationary_detector_.exit_pending() ? "true" : "false"));
  status.values.push_back(diagnostic_value(
      "external_stationary", external_stationary_ ? "true" : "false"));
  status.values.push_back(
      diagnostic_value("external_moving", external_moving_ ? "true" : "false"));
  status.values.push_back(diagnostic_value(
      "gyro_bias_z_deg_s", std::to_string(degrees(yaw_filter_.bias()))));
  status.values.push_back(diagnostic_value(
      "gyro_bias_std_deg_s",
      std::to_string(degrees(std::sqrt(yaw_filter_.bias_variance())))));
  status.values.push_back(
      diagnostic_value("yaw_deg", std::to_string(degrees(yaw_filter_.yaw()))));
  status.values.push_back(diagnostic_value(
      "yaw_std_deg", std::to_string(degrees(std::sqrt(std::max(
                         yaw_filter_.yaw_variance(),
                         stationary_yaw_std_ * stationary_yaw_std_))))));
  status.values.push_back(diagnostic_value(
      "rejected_stationary_transients",
      std::to_string(stationary_detector_.rejected_transient_count())));
  status.values.push_back(
      diagnostic_value("correction_source", correction_source_));

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = now();
  array.status.push_back(std::move(status));
  diagnostic_publisher_->publish(array);
}

} // namespace imu_rpy_filter
