#include "vehicle_loopback_sim/simulator.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/create_timer.hpp>
#include <yaml-cpp/yaml.h>

namespace vehicle_loopback_sim
{
namespace
{

struct PhysicalDefaults
{
  double wheel_track_m;
  double base_x_m;
  double base_y_m;
  double camera_x_m;
  double camera_y_m;
};

PhysicalDefaults physical_defaults()
{
  const std::string path =
    ament_index_cpp::get_package_share_directory("vehicle_config") +
    "/config/vehicle_physical.yaml";
  const YAML::Node root = YAML::LoadFile(path);
  return {
    root["differential_drive"]["wheel_track_m"].as<double>(),
    root["differential_drive"]["base_link_offset_x_m"].as<double>(),
    root["differential_drive"]["base_link_offset_y_m"].as<double>(),
    root["camera"]["x_m"].as<double>(),
    root["camera"]["y_m"].as<double>()};
}

double finite(double value, const std::string & name)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(name + " must be finite");
  }
  return value;
}

}  // namespace

VehicleLoopbackSim::VehicleLoopbackSim()
: Node("vehicle_loopback_sim")
{
  declare_parameters();
  map_frame_ = get_parameter("map_frame_id").as_string();
  base_frame_ = get_parameter("base_frame_id").as_string();
  obstacle_frame_ = get_parameter("obstacle_detection_frame_id").as_string();
  cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  wheel_track_m_ = finite(get_parameter("wheel_track_m").as_double(), "wheel_track_m");
  base_offset_x_m_ = finite(get_parameter("base_offset_x_m").as_double(), "base_offset_x_m");
  base_offset_y_m_ = finite(get_parameter("base_offset_y_m").as_double(), "base_offset_y_m");
  command_timeout_s_ = std::max(
    0.0, finite(get_parameter("command_timeout_s").as_double(), "command_timeout_s"));
  playback_speed_ = std::clamp(
    finite(get_parameter("playback_speed").as_double(), "playback_speed"), 0.1, 10.0);
  max_linear_speed_mps_ = finite(
    get_parameter("max_linear_speed_mps").as_double(), "max_linear_speed_mps");
  max_angular_speed_rps_ = finite(
    get_parameter("max_angular_speed_rps").as_double(), "max_angular_speed_rps");
  max_linear_accel_mps2_ = finite(
    get_parameter("max_linear_accel_mps2").as_double(), "max_linear_accel_mps2");
  max_angular_accel_rps2_ = finite(
    get_parameter("max_angular_accel_rps2").as_double(), "max_angular_accel_rps2");
  obstacle_fov_rad_ = std::max(
    0.01, finite(get_parameter("obstacle_detection_fov_rad").as_double(),
    "obstacle_detection_fov_rad"));
  obstacle_min_range_m_ = finite(
    get_parameter("obstacle_detection_min_range_m").as_double(),
    "obstacle_detection_min_range_m");
  publish_tf_ = get_parameter("publish_tf").as_bool();
  publish_actuator_health_ = get_parameter("publish_actuator_health").as_bool();
  publish_control_telemetry_ = get_parameter("publish_control_telemetry").as_bool();
  publish_obstacle_detection_ = get_parameter("publish_obstacle_detection").as_bool();
  obstacle_gate_ = std::make_unique<StableObstacleGate>(std::max(
      0.0, finite(get_parameter("obstacle_detection_stable_time_s").as_double(),
      "obstacle_detection_stable_time_s")));
  pose_ = {
    finite(get_parameter("initial_x_m").as_double(), "initial_x_m"),
    finite(get_parameter("initial_y_m").as_double(), "initial_y_m"),
    finite(get_parameter("initial_yaw_rad").as_double(), "initial_yaw_rad")};
  const auto stamp = now();
  last_command_time_ = stamp;
  last_update_time_ = stamp;
  started_time_ = stamp;
  create_interfaces();
  RCLCPP_INFO(
    get_logger(), "vehicle loopback C++ ready: cmd=%s odom=%s frame=%s->%s playback=%.1fx",
    cmd_vel_topic_.c_str(), odom_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str(),
    playback_speed_);
}

void VehicleLoopbackSim::declare_parameters()
{
  const auto physical = physical_defaults();
  declare_parameter("cmd_vel_topic", "/cmd_vel_nav");
  declare_parameter("initial_pose_topic", "/initialpose");
  declare_parameter("odom_topic", "/odometry/fused");
  declare_parameter("imu_topic", "/imu/control");
  declare_parameter("fusion_status_topic", "/odometry/fusion_status");
  declare_parameter("actuator_health_topic", "/cup_car_serial/actuator_healthy");
  declare_parameter("control_telemetry_topic", "/cup_car_serial/control_telemetry");
  declare_parameter("front_obstacle_topic", "/obstacle/front_blocked");
  declare_parameter("front_obstacle_range_topic", "/obstacle/front_range");
  declare_parameter("dynamic_obstacle_topic", "/perception/dynamic_obstacle");
  declare_parameter("manual_obstacle_topic", "/vehicle_loopback_sim/manual_obstacle");
  declare_parameter("clear_manual_obstacles_topic", "/vehicle_loopback_sim/clear_obstacles");
  declare_parameter("status_topic", "/vehicle_loopback_sim/status");
  declare_parameter(
    "arena_config",
    ament_index_cpp::get_package_share_directory("new_map_planner") + "/config/arena.yaml");
  declare_parameter("map_frame_id", "map");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("initial_x_m", 0.0);
  declare_parameter("initial_y_m", 0.0);
  declare_parameter("initial_yaw_rad", 0.0);
  declare_parameter("update_rate_hz", 50.0);
  declare_parameter("status_rate_hz", 10.0);
  declare_parameter("command_timeout_s", 0.5);
  declare_parameter("playback_speed", 1.0);
  declare_parameter("max_linear_speed_mps", 0.2);
  declare_parameter("max_angular_speed_rps", 0.9);
  declare_parameter("max_linear_accel_mps2", 0.0);
  declare_parameter("max_angular_accel_rps2", 0.0);
  declare_parameter("wheel_track_m", physical.wheel_track_m);
  declare_parameter("base_offset_x_m", physical.base_x_m);
  declare_parameter("base_offset_y_m", physical.base_y_m);
  declare_parameter("publish_tf", true);
  declare_parameter("publish_actuator_health", true);
  declare_parameter("publish_control_telemetry", true);
  declare_parameter("publish_obstacle_detection", true);
  declare_parameter("obstacle_detection_frame_id", "camera_link");
  declare_parameter("obstacle_sensor_x_m", physical.camera_x_m);
  declare_parameter("obstacle_sensor_y_m", physical.camera_y_m);
  declare_parameter("obstacle_detection_trigger_range_m", 0.25);
  declare_parameter("obstacle_detection_trigger_jitter_m", 0.0);
  declare_parameter("obstacle_detection_lateral_tolerance_m", 0.08);
  declare_parameter("obstacle_detection_fov_rad", 0.40);
  declare_parameter("obstacle_detection_min_range_m", 0.03);
  declare_parameter("obstacle_detection_stable_time_s", 0.0);
  declare_parameter("obstacle_detection_random_seed", 0);
}

void VehicleLoopbackSim::create_interfaces()
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 10,
    std::bind(&VehicleLoopbackSim::on_cmd_vel, this, std::placeholders::_1));
  initial_pose_subscription_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    get_parameter("initial_pose_topic").as_string(), 10,
    std::bind(&VehicleLoopbackSim::on_initial_pose, this, std::placeholders::_1));
  manual_obstacle_subscription_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
    get_parameter("manual_obstacle_topic").as_string(), 10,
    std::bind(&VehicleLoopbackSim::on_manual_obstacle, this, std::placeholders::_1));
  clear_obstacles_subscription_ = create_subscription<std_msgs::msg::Empty>(
    get_parameter("clear_manual_obstacles_topic").as_string(), 10,
    std::bind(&VehicleLoopbackSim::on_clear_obstacles, this, std::placeholders::_1));

  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
    get_parameter("imu_topic").as_string(), 10);
  fusion_status_publisher_ = create_publisher<std_msgs::msg::String>(
    get_parameter("fusion_status_topic").as_string(), 10);
  status_publisher_ = create_publisher<std_msgs::msg::String>(
    get_parameter("status_topic").as_string(), 10);
  const auto durable = rclcpp::QoS(1).reliable().transient_local();
  actuator_health_publisher_ = create_publisher<std_msgs::msg::Bool>(
    get_parameter("actuator_health_topic").as_string(), durable);
  telemetry_publisher_ =
    create_publisher<mission_control_interfaces::msg::ControlTelemetry>(
    get_parameter("control_telemetry_topic").as_string(), 10);
  front_obstacle_publisher_ = create_publisher<std_msgs::msg::Bool>(
    get_parameter("front_obstacle_topic").as_string(), rclcpp::SensorDataQoS());
  front_range_publisher_ = create_publisher<sensor_msgs::msg::Range>(
    get_parameter("front_obstacle_range_topic").as_string(), rclcpp::SensorDataQoS());
  dynamic_obstacle_publisher_ = create_publisher<geometry_msgs::msg::PolygonStamped>(
    get_parameter("dynamic_obstacle_topic").as_string(), 10);

  const double update_rate = std::max(
    1.0, finite(get_parameter("update_rate_hz").as_double(), "update_rate_hz"));
  const double status_rate = std::max(
    1.0, finite(get_parameter("status_rate_hz").as_double(), "status_rate_hz"));
  update_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / (update_rate * playback_speed_)),
    std::bind(&VehicleLoopbackSim::update, this));
  status_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / status_rate),
    std::bind(&VehicleLoopbackSim::publish_status_bundle, this));
}

ObstacleConfig VehicleLoopbackSim::obstacle_config(
  std::optional<double> activation_floor) const
{
  return {
    get_parameter("obstacle_sensor_x_m").as_double(),
    get_parameter("obstacle_sensor_y_m").as_double(),
    std::max(0.01, get_parameter("obstacle_detection_trigger_range_m").as_double()),
    std::max(0.0, get_parameter("obstacle_detection_trigger_jitter_m").as_double()),
    std::max(0.0, get_parameter("obstacle_detection_lateral_tolerance_m").as_double()),
    std::max(0.0, obstacle_min_range_m_),
    static_cast<int>(get_parameter("obstacle_detection_random_seed").as_int()),
    activation_floor};
}

}  // namespace vehicle_loopback_sim
