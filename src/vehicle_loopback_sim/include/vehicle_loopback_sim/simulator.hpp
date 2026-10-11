#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mission_control_interfaces/msg/control_telemetry.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <navigation_safety/checker.hpp>

#include "vehicle_loopback_sim/motion.hpp"
#include "vehicle_loopback_sim/obstacle.hpp"

namespace vehicle_loopback_sim
{

class VehicleLoopbackSim : public rclcpp::Node
{
public:
  VehicleLoopbackSim();

private:
  void declare_parameters();
  void create_interfaces();
  ObstacleConfig obstacle_config(std::optional<double> activation_floor = std::nullopt) const;
  void rebuild_obstacle_detector();
  void reset_obstacle_reporting();

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message);
  void on_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
  void on_manual_obstacle(const geometry_msgs::msg::PolygonStamped::SharedPtr message);
  void on_clear_obstacles(const std_msgs::msg::Empty::SharedPtr message);
  void update();
  void publish_odometry(const rclcpp::Time & stamp);
  void publish_imu(const rclcpp::Time & stamp);
  void publish_transform(const rclcpp::Time & stamp);
  void publish_telemetry(const rclcpp::Time & stamp, double command_age_s);
  void publish_obstacle_report(const rclcpp::Time & stamp);
  void publish_dynamic_obstacle(
    const ObstacleDetection & detection, const rclcpp::Time & stamp);
  void publish_status_bundle();
  void load_collision_model();
  bool pose_is_safe(const Pose2d & pose, navigation_safety::Report * report = nullptr) const;

  std::string map_frame_;
  std::string base_frame_;
  std::string obstacle_frame_;
  std::string cmd_vel_topic_;
  std::string odom_topic_;
  std::string simulation_mode_;
  double wheel_track_m_{0.1466};
  double base_offset_x_m_{0.03};
  double base_offset_y_m_{0.0};
  double command_timeout_s_{0.5};
  double playback_speed_{1.0};
  double max_linear_speed_mps_{0.2};
  double max_angular_speed_rps_{0.9};
  double max_linear_accel_mps2_{0.0};
  double max_angular_accel_rps2_{0.0};
  double obstacle_fov_rad_{0.4};
  double obstacle_min_range_m_{0.03};
  bool publish_tf_{true};
  bool publish_actuator_health_{true};
  bool publish_control_telemetry_{true};
  bool publish_obstacle_detection_{true};
  bool enforce_static_collision_{true};
  navigation_safety::Footprint footprint_{};
  std::vector<navigation_safety::Rectangle> free_regions_;
  std::vector<navigation_safety::Polygon> static_obstacles_;

  Pose2d pose_;
  Twist2d desired_twist_;
  Twist2d applied_twist_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time started_time_{0, 0, RCL_ROS_TIME};
  std::string last_status_text_;
  uint32_t telemetry_sequence_{0U};

  std::vector<Polygon2d> manual_obstacles_;
  std::unique_ptr<SimulatedObstacleDetector> obstacle_detector_;
  std::unique_ptr<StableObstacleGate> obstacle_gate_;
  std::optional<std::size_t> last_obstacle_index_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
    manual_obstacle_subscription_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_obstacles_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fusion_status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr actuator_health_publisher_;
  rclcpp::Publisher<mission_control_interfaces::msg::ControlTelemetry>::SharedPtr
    telemetry_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr front_obstacle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr front_range_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
    dynamic_obstacle_publisher_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace vehicle_loopback_sim
