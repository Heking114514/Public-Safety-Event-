#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "wheel_odometry/odometry_integrator.hpp"
#include "wheel_odometry/mcu_clock_mapper.hpp"

namespace wheel_odometry
{

class WheelOdometryNode : public rclcpp::Node
{
public:
  WheelOdometryNode()
  : Node("wheel_odometry_node"), integrator_(read_config())
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/cup_car_serial/encoder_ticks");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/wheel/odom");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    diagnostic_stale_timeout_s_ = positive_parameter("diagnostic_stale_timeout_s", 0.5);
    const double diagnostic_period_s = positive_parameter("diagnostic_period_s", 0.5);

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 20);
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10);
    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    encoder_subscription_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](const std_msgs::msg::Int32MultiArray::SharedPtr message) {
        encoder_callback(*message);
      });
    diagnostic_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(diagnostic_period_s),
      [this]() {publish_diagnostics();});

    RCLCPP_INFO(
      get_logger(), "Wheel odometry: %s -> %s (%s -> %s, TF %s)",
      input_topic_.c_str(), odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
      publish_tf_ ? "enabled" : "disabled");
  }

private:
  IntegratorConfig read_config()
  {
    IntegratorConfig config;
    config.wheel_radius_m = positive_parameter("wheel_radius_m", config.wheel_radius_m);
    config.ticks_per_revolution = positive_parameter(
      "ticks_per_revolution", config.ticks_per_revolution);
    config.wheel_track_m = positive_parameter("wheel_track_m", config.wheel_track_m);
    config.left_distance_scale = positive_parameter(
      "left_distance_scale", config.left_distance_scale);
    config.right_distance_scale = positive_parameter(
      "right_distance_scale", config.right_distance_scale);
    config.yaw_slip_scale = positive_parameter("yaw_slip_scale", config.yaw_slip_scale);
    config.max_wheel_speed_mps = positive_parameter(
      "max_wheel_speed_mps", config.max_wheel_speed_mps);
    config.min_dt_s = positive_parameter("min_dt_s", config.min_dt_s);
    config.nominal_sample_period_s = positive_parameter(
      "nominal_sample_period_s", config.nominal_sample_period_s);
    config.nominal_sequence_increment = positive_integer_parameter(
      "nominal_sequence_increment", config.nominal_sequence_increment);
    config.pose_xy_variance = nonnegative_parameter(
      "pose_xy_variance", config.pose_xy_variance);
    config.pose_yaw_variance = nonnegative_parameter(
      "pose_yaw_variance", config.pose_yaw_variance);
    config.twist_linear_variance = nonnegative_parameter(
      "twist_linear_variance", config.twist_linear_variance);
    config.twist_yaw_variance = nonnegative_parameter(
      "twist_yaw_variance", config.twist_yaw_variance);
    config.pose_xy_variance_per_meter = nonnegative_parameter(
      "pose_xy_variance_per_meter", config.pose_xy_variance_per_meter);
    config.pose_xy_variance_per_radian = nonnegative_parameter(
      "pose_xy_variance_per_radian", config.pose_xy_variance_per_radian);
    config.pose_yaw_variance_per_meter = nonnegative_parameter(
      "pose_yaw_variance_per_meter", config.pose_yaw_variance_per_meter);
    config.pose_yaw_variance_per_radian = nonnegative_parameter(
      "pose_yaw_variance_per_radian", config.pose_yaw_variance_per_radian);
    config.pose_xy_variance_per_missing_sample = nonnegative_parameter(
      "pose_xy_variance_per_missing_sample", config.pose_xy_variance_per_missing_sample);
    config.pose_yaw_variance_per_missing_sample = nonnegative_parameter(
      "pose_yaw_variance_per_missing_sample", config.pose_yaw_variance_per_missing_sample);
    config.pose_xy_variance_per_rebase = nonnegative_parameter(
      "pose_xy_variance_per_rebase", config.pose_xy_variance_per_rebase);
    config.pose_yaw_variance_per_rebase = nonnegative_parameter(
      "pose_yaw_variance_per_rebase", config.pose_yaw_variance_per_rebase);
    config.twist_linear_variance_per_missing_sample = nonnegative_parameter(
      "twist_linear_variance_per_missing_sample",
      config.twist_linear_variance_per_missing_sample);
    config.twist_yaw_variance_per_missing_sample = nonnegative_parameter(
      "twist_yaw_variance_per_missing_sample", config.twist_yaw_variance_per_missing_sample);
    config.twist_linear_variance_per_interval_ratio = nonnegative_parameter(
      "twist_linear_variance_per_interval_ratio",
      config.twist_linear_variance_per_interval_ratio);
    config.twist_yaw_variance_per_interval_ratio = nonnegative_parameter(
      "twist_yaw_variance_per_interval_ratio", config.twist_yaw_variance_per_interval_ratio);
    config.twist_linear_variance_per_wheel_difference_mps = nonnegative_parameter(
      "twist_linear_variance_per_wheel_difference_mps",
      config.twist_linear_variance_per_wheel_difference_mps);
    config.twist_yaw_variance_per_wheel_difference_mps = nonnegative_parameter(
      "twist_yaw_variance_per_wheel_difference_mps",
      config.twist_yaw_variance_per_wheel_difference_mps);
    return config;
  }

  double positive_parameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(name + " must be finite and positive");
    }
    return value;
  }

  double nonnegative_parameter(const std::string & name, double default_value)
  {
    const double value = declare_parameter<double>(name, default_value);
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(name + " must be finite and non-negative");
    }
    return value;
  }

  uint32_t positive_integer_parameter(const std::string & name, uint32_t default_value)
  {
    const std::int64_t value = declare_parameter<std::int64_t>(name, default_value);
    if (value < 1 || value > static_cast<std::int64_t>(UINT32_MAX)) {
      throw std::invalid_argument(name + " must be a positive uint32 value");
    }
    return static_cast<uint32_t>(value);
  }

  void encoder_callback(const std_msgs::msg::Int32MultiArray & message)
  {
    if (message.data.size() != 4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring encoder message with %zu fields; expected 4", message.data.size());
      return;
    }

    const rclcpp::Time reception_time = now();

    EncoderSample sample;
    sample.mcu_time_ms = static_cast<uint32_t>(message.data[0]);
    sample.sequence = static_cast<uint32_t>(message.data[1]);
    sample.left_total_ticks = message.data[2];
    sample.right_total_ticks = message.data[3];
    const UpdateResult result = integrator_.update(sample);
    last_update_status_ = result.status;
    last_sequence_delta_ = result.sequence_delta;

    // A duplicate frame proves only that the serial stream is repeating old
    // data. It must not keep the wheel source marked fresh while its pose is
    // frozen. Valid integrated and baseline samples do refresh freshness;
    // rebases remain visible as a diagnostic warning.
    if (result.status != UpdateStatus::kDuplicate) {
      last_encoder_reception_ = reception_time;
      has_received_encoder_ = true;
    }

    if (!result.publish) {
      if (result.status == UpdateStatus::kInitialized) {
        clock_mapper_.reset();
      }
      if (result.status == UpdateStatus::kRebasedSequenceRegression ||
        result.status == UpdateStatus::kRebasedTimeRegression ||
        result.status == UpdateStatus::kRebasedInvalidDt ||
        result.status == UpdateStatus::kRebasedTickJump)
      {
        // A reset/rebase invalidates the MCU-to-ROS epoch. Establish a new
        // anchor on the next accepted interval instead of manufacturing a
        // timestamp jump that could look like stale or future data upstream.
        clock_mapper_.reset();
      }
      if (result.status != UpdateStatus::kInitialized &&
        result.status != UpdateStatus::kDuplicate)
      {
        RCLCPP_WARN(
          get_logger(), "Encoder baseline reset: %s (seq=%u, dt=%.3f s, dL=%ld, dR=%ld)",
          OdometryIntegrator::status_string(result.status), sample.sequence, result.dt_s,
          static_cast<long>(result.left_delta_ticks),
          static_cast<long>(result.right_delta_ticks));
      }
      return;
    }

    if (result.sequence_delta > 1U) {
      RCLCPP_DEBUG(
        get_logger(), "Integrated across %u missing encoder samples", result.sequence_delta - 1U);
    }
    rclcpp::Time measurement_stamp(
      clock_mapper_.map(sample.mcu_time_ms, reception_time.nanoseconds()),
      RCL_ROS_TIME);
    // The initial serial delay is unknowable. Never publish a future-dated
    // sample; health freshness continues to use reception_time below.
    if (measurement_stamp > reception_time) {
      measurement_stamp = reception_time;
    }
    if (last_measurement_stamp_.nanoseconds() != 0 &&
      measurement_stamp <= last_measurement_stamp_)
    {
      ++nonmonotonic_stamp_count_;
      return;
    }
    last_measurement_stamp_ = measurement_stamp;
    publish_odometry(measurement_stamp);
  }

  static diagnostic_msgs::msg::KeyValue diagnostic_value(
    const std::string & key, const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    return item;
  }

  void publish_diagnostics()
  {
    const rclcpp::Time stamp = now();
    const double sample_age_s = has_received_encoder_ ?
      (stamp - last_encoder_reception_).seconds() :
      std::numeric_limits<double>::infinity();
    const bool fresh = has_received_encoder_ && sample_age_s <= diagnostic_stale_timeout_s_;
    const auto & uncertainty = integrator_.uncertainty();
    const auto & statistics = integrator_.statistics();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name() + std::string(": encoder stream");
    status.hardware_id = "differential_drive_encoders";
    if (!fresh) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = has_received_encoder_ ? "encoder sample stale" : "no encoder sample";
    } else if (last_update_status_ != UpdateStatus::kIntegrated &&
      last_update_status_ != UpdateStatus::kInitialized &&
      last_update_status_ != UpdateStatus::kDuplicate)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = OdometryIntegrator::status_string(last_update_status_);
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "encoder samples fresh";
    }

    status.values.push_back(diagnostic_value("fresh_sample", fresh ? "true" : "false"));
    status.values.push_back(diagnostic_value("sample_age_s", std::to_string(sample_age_s)));
    status.values.push_back(diagnostic_value(
      "last_update_status", OdometryIntegrator::status_string(last_update_status_)));
    status.values.push_back(diagnostic_value(
      "last_sequence_delta", std::to_string(last_sequence_delta_)));
    status.values.push_back(diagnostic_value(
      "integrated_samples", std::to_string(statistics.integrated_samples)));
    status.values.push_back(diagnostic_value(
      "sequence_gap_events", std::to_string(statistics.sequence_gap_events)));
    status.values.push_back(diagnostic_value(
      "missing_samples", std::to_string(statistics.missing_samples)));
    status.values.push_back(diagnostic_value(
      "rebase_count", std::to_string(statistics.rebase_count)));
    status.values.push_back(diagnostic_value(
      "tick_jump_count", std::to_string(statistics.tick_jump_count)));
    status.values.push_back(diagnostic_value(
      "pose_xy_variance", std::to_string(uncertainty.pose_xy_variance)));
    status.values.push_back(diagnostic_value(
      "pose_yaw_variance", std::to_string(uncertainty.pose_yaw_variance)));
    status.values.push_back(diagnostic_value(
      "twist_linear_variance", std::to_string(uncertainty.twist_linear_variance)));
    status.values.push_back(diagnostic_value(
      "twist_yaw_variance", std::to_string(uncertainty.twist_yaw_variance)));
    status.values.push_back(diagnostic_value(
      "accumulated_wheel_travel_m",
      std::to_string(statistics.accumulated_wheel_travel_m)));
    status.values.push_back(diagnostic_value(
      "accumulated_abs_turn_rad", std::to_string(statistics.accumulated_abs_turn_rad)));
    status.values.push_back(diagnostic_value(
      "nonmonotonic_measurement_stamps", std::to_string(nonmonotonic_stamp_count_)));

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = stamp;
    array.status.push_back(std::move(status));
    diagnostic_publisher_->publish(array);
  }

  void publish_odometry(const rclcpp::Time & stamp)
  {
    const OdometryState & state = integrator_.state();
    const OdometryUncertainty & uncertainty = integrator_.uncertainty();
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, state.yaw_rad);

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = stamp;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = base_frame_;
    odometry.pose.pose.position.x = state.x_m;
    odometry.pose.pose.position.y = state.y_m;
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.pose.pose.orientation.w = orientation.w();
    odometry.twist.twist.linear.x = state.linear_velocity_mps;
    odometry.twist.twist.angular.z = state.angular_velocity_radps;
    odometry.pose.covariance[0] = uncertainty.pose_xy_variance;
    odometry.pose.covariance[7] = uncertainty.pose_xy_variance;
    odometry.pose.covariance[14] = 1.0e6;
    odometry.pose.covariance[21] = 1.0e6;
    odometry.pose.covariance[28] = 1.0e6;
    odometry.pose.covariance[35] = uncertainty.pose_yaw_variance;
    odometry.twist.covariance[0] = uncertainty.twist_linear_variance;
    odometry.twist.covariance[7] = 1.0e6;
    odometry.twist.covariance[14] = 1.0e6;
    odometry.twist.covariance[21] = 1.0e6;
    odometry.twist.covariance[28] = 1.0e6;
    odometry.twist.covariance[35] = uncertainty.twist_yaw_variance;
    odom_publisher_->publish(odometry);

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = odometry.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = state.x_m;
      transform.transform.translation.y = state.y_m;
      transform.transform.rotation = odometry.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  OdometryIntegrator integrator_;
  std::string input_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  double diagnostic_stale_timeout_s_{0.5};
  bool has_received_encoder_{false};
  rclcpp::Time last_encoder_reception_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_measurement_stamp_{0, 0, RCL_ROS_TIME};
  McuClockMapper clock_mapper_;
  uint64_t nonmonotonic_stamp_count_{0};
  UpdateStatus last_update_status_{UpdateStatus::kInitialized};
  uint32_t last_sequence_delta_{0};
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_subscription_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace wheel_odometry

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wheel_odometry::WheelOdometryNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("wheel_odometry_node"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
