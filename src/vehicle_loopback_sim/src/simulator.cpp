#include "vehicle_loopback_sim/simulator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/create_timer.hpp>
#include <yaml-cpp/yaml.h>
#include <limits>

namespace vehicle_loopback_sim
{
namespace
{

struct MapTransform
{
  double origin_x;
  double origin_y;
  double yaw;
};

struct Point2d
{
  double x;
  double y;
};

Point2d transform_point(const MapTransform & transform, double x, double y)
{
  const double dx = x - transform.origin_x;
  const double dy = y - transform.origin_y;
  const double cosine = std::cos(transform.yaw);
  const double sine = std::sin(transform.yaw);
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

navigation_safety::Rectangle transform_rectangle(
  const MapTransform & transform, const YAML::Node & node)
{
  const double minimum_x = node[0].as<double>();
  const double minimum_y = node[1].as<double>();
  const double maximum_x = node[2].as<double>();
  const double maximum_y = node[3].as<double>();
  const std::array<Point2d, 4> corners{
    transform_point(transform, minimum_x, minimum_y),
    transform_point(transform, maximum_x, minimum_y),
    transform_point(transform, maximum_x, maximum_y),
    transform_point(transform, minimum_x, maximum_y)};
  navigation_safety::Rectangle result{
    std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity(),
    -std::numeric_limits<double>::infinity()};
  for (const Point2d & point : corners) {
    result.minimum_x = std::min(result.minimum_x, point.x);
    result.minimum_y = std::min(result.minimum_y, point.y);
    result.maximum_x = std::max(result.maximum_x, point.x);
    result.maximum_y = std::max(result.maximum_y, point.y);
  }
  return result;
}

navigation_safety::Polygon transform_obstacle(
  const MapTransform & transform, const YAML::Node & node)
{
  const double minimum_x = node[0].as<double>();
  const double minimum_y = node[1].as<double>();
  const double maximum_x = node[2].as<double>();
  const double maximum_y = node[3].as<double>();
  navigation_safety::Polygon result;
  for (const auto & corner : std::array<std::pair<double, double>, 4>{
      std::pair{minimum_x, minimum_y}, std::pair{maximum_x, minimum_y},
      std::pair{maximum_x, maximum_y}, std::pair{minimum_x, maximum_y}}) {
    const Point2d point = transform_point(transform, corner.first, corner.second);
    result.points.push_back({point.x, point.y});
  }
  return result;
}

struct PhysicalDefaults
{
  double length_m;
  double width_m;
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
    root["chassis"]["length_m"].as<double>(),
    root["chassis"]["width_m"].as<double>(),
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
  simulation_mode_ = get_parameter("simulation_mode").as_string();
  if (simulation_mode_ != "closed_loop" && simulation_mode_ != "offline") {
    throw std::invalid_argument(
            "simulation_mode must be closed_loop or offline");
  }
  playback_speed_ = std::clamp(
    finite(get_parameter("playback_speed").as_double(), "playback_speed"), 0.1, 10.0);
  if (simulation_mode_ == "closed_loop" && std::abs(playback_speed_ - 1.0) > 1.0e-9) {
    throw std::invalid_argument(
            "closed_loop requires playback_speed=1.0; use simulation_mode=offline "
            "for non-acceptance accelerated playback");
  }
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
  enforce_static_collision_ = get_parameter("enforce_static_collision").as_bool();
  footprint_ = {
    physical_defaults().length_m, physical_defaults().width_m, 0.0, 0.0,
    std::max(0.0, get_parameter("static_collision_margin_m").as_double())};
  load_collision_model();
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
    get_logger(),
    "vehicle loopback C++ ready: cmd=%s odom=%s frame=%s->%s mode=%s playback=%.1fx",
    cmd_vel_topic_.c_str(), odom_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str(),
    simulation_mode_.c_str(), playback_speed_);
  if (simulation_mode_ == "offline" && std::abs(playback_speed_ - 1.0) > 1.0e-9) {
    RCLCPP_WARN(
      get_logger(),
      "accelerated offline playback (%.1fx) is NON-ACCEPTANCE and must not be used "
      "to validate the production closed loop",
      playback_speed_);
  }
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
  declare_parameter("simulation_mode", "closed_loop");
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
  declare_parameter("enforce_static_collision", true);
  declare_parameter("static_collision_margin_m", 0.002);
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

void VehicleLoopbackSim::load_collision_model()
{
  const YAML::Node root = YAML::LoadFile(get_parameter("arena_config").as_string());
  const YAML::Node origin = root["frame"]["map_origin_m"];
  const MapTransform transform{
    origin[0].as<double>(), origin[1].as<double>(),
    root["frame"]["map_x_heading_deg"].as<double>() *
    3.14159265358979323846 / 180.0};
  for (const YAML::Node & region : root["arena"]["free_regions"]) {
    free_regions_.push_back(transform_rectangle(transform, region));
  }
  for (const YAML::Node & obstacle : root["arena"]["obstacles"]) {
    static_obstacles_.push_back(transform_obstacle(transform, obstacle));
  }
  if (free_regions_.empty()) {
    throw std::invalid_argument("arena_config must define free_regions");
  }
  RCLCPP_INFO(
    get_logger(), "static collision model ready: free_regions=%zu obstacles=%zu footprint=%.3fx%.3fm margin=%.3fm",
    free_regions_.size(), static_obstacles_.size(), footprint_.length,
    footprint_.width, footprint_.margin);
}

bool VehicleLoopbackSim::pose_is_safe(
  const Pose2d & pose, navigation_safety::Report * report) const
{
  if (!enforce_static_collision_) {
    return true;
  }
  std::vector<navigation_safety::Polygon> dynamic;
  dynamic.reserve(manual_obstacles_.size());
  for (const auto &polygon : manual_obstacles_) {
    navigation_safety::Polygon converted;
    for (const auto &point : polygon) converted.points.push_back({point.first, point.second});
    dynamic.push_back(std::move(converted));
  }
  return navigation_safety::pose_is_safe(
    {{pose.x, pose.y}, pose.yaw}, footprint_, free_regions_, static_obstacles_, dynamic, report);
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
