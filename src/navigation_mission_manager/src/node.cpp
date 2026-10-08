#include "navigation_mission_manager/manager.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace navigation_mission_manager
{
namespace
{

std::string trim(const std::string & value)
{
  const auto first = std::find_if_not(
    value.begin(), value.end(), [](unsigned char c) {return std::isspace(c) != 0;});
  const auto last = std::find_if_not(
    value.rbegin(), value.rend(), [](unsigned char c) {return std::isspace(c) != 0;}).base();
  return first < last ? std::string(first, last) : std::string{};
}

std::string state_name(uint8_t state)
{
  using State = mission_control_interfaces::msg::MissionState;
  switch (state) {
    case State::IDLE: return "IDLE";
    case State::WAIT_READY: return "WAIT_READY";
    case State::PLANNING: return "PLANNING";
    case State::WAIT_ROUTE_ACK: return "WAIT_ROUTE_ACK";
    case State::EXECUTING: return "EXECUTING";
    case State::RECOVERING_OBSTACLE: return "RECOVERING_OBSTACLE";
    case State::REPLANNING: return "REPLANNING";
    case State::WAIT_REPORTER: return "WAIT_REPORTER";
    case State::REPORTING: return "REPORTING";
    case State::PAUSED: return "PAUSED";
    case State::FAULT: return "FAULT";
    case State::COMPLETE: return "COMPLETE";
    default: return "UNKNOWN";
  }
}

double camera_x_default()
{
  const std::string path =
    ament_index_cpp::get_package_share_directory("vehicle_config") +
    "/config/vehicle_physical.yaml";
  return YAML::LoadFile(path)["camera"]["x_m"].as<double>();
}

}  // namespace

double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::vector<std::string> parse_ids(const std::string & text)
{
  std::vector<std::string> result;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ',')) {
    item = trim(item);
    if (!item.empty() && std::find(result.begin(), result.end(), item) == result.end()) {
      result.push_back(item);
    }
  }
  return result;
}

std::optional<double> quaternion_yaw(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    return std::nullopt;
  }
  const double x = quaternion.x / std::sqrt(norm);
  const double y = quaternion.y / std::sqrt(norm);
  const double z = quaternion.z / std::sqrt(norm);
  const double w = quaternion.w / std::sqrt(norm);
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

bool VehiclePose::fresh(double timeout_s) const
{
  return steady_seconds() - received_at <= timeout_s;
}

MissionManager::MissionManager()
: Node("navigation_mission_manager")
{
  declare_parameters();
  load_configuration();
  create_interfaces();
  reset_rfid();
  auto_start_pending_ = get_parameter("auto_start").as_bool();
  timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {tick();});
  publish_state();
  RCLCPP_INFO(
    get_logger(), "navigation mission manager C++ ready: map_file=%s auto_start=%s targets=%zu",
    get_parameter("map_file").as_string().c_str(), auto_start_pending_ ? "true" : "false",
    default_targets_.size());
}

void MissionManager::declare_parameters()
{
  const std::string planner_share =
    ament_index_cpp::get_package_share_directory("new_map_planner");
  declare_parameter("map_file", planner_share + "/config/arena.yaml");
  declare_parameter("planner_service", "/new_map_planner/plan");
  declare_parameter("odom_topic", "/odometry/fused");
  declare_parameter("navigator_status_topic", "/waypoint_navigation/status");
  declare_parameter("current_waypoint_topic", "/waypoint_navigation/current_waypoint");
  declare_parameter("route_ack_topic", "/waypoint_navigation/route_ack");
  declare_parameter("route_input_topic", "/waypoint_navigation/route_input");
  declare_parameter("motion_hold_service", "/waypoint_navigator/set_motion_hold");
  declare_parameter("continue_retreat_service", "/waypoint_navigator/continue_obstacle_retreat");
  declare_parameter(
    "continue_route_retreat_service", "/waypoint_navigator/continue_route_retreat");
  declare_parameter("motion_hold_state_topic", "/waypoint_navigation/motion_hold_state");
  declare_parameter("front_obstacle_topic", "/obstacle/front_blocked");
  declare_parameter("front_range_topic", "/obstacle/front_range");
  declare_parameter("dynamic_obstacle_topic", "/perception/dynamic_obstacle");
  declare_parameter("event_candidate_topic", "/perception/event_candidate");
  declare_parameter("reported_event_topic", "/navigation_mission/reported_event");
  declare_parameter("mission_state_topic", "/navigation_mission/state");
  declare_parameter("status_text_topic", "/navigation_mission/status_text");
  declare_parameter("control_service", "/navigation_mission/control");
  declare_parameter("report_action", "/event_reporter/report");
  declare_parameter("rfid_number_topic", "/cup_car_serial/rfid_number");
  declare_parameter("task_confirmation_mode", "rfid");
  declare_parameter("speak_text_service", "/cup_car_serial/speak_text");
  declare_parameter("correct_map_position_service", "/fused_odometry/correct_map_position");
  declare_parameter(
    "confirm_rfid_checkpoint_service", "/waypoint_navigator/confirm_rfid_checkpoint");
  declare_parameter("speech_text_topic", "/navigation_mission/speech_text");
  declare_parameter("public_motion_hold_service", "/navigation_mission/set_motion_hold");
  declare_parameter(
    "public_continue_retreat_service", "/navigation_mission/continue_obstacle_retreat");
  public_service_timeout_s_ = declare_parameter("public_service_timeout_s", 2.0);
  speech_hold_duration_s_ = declare_parameter("speech_hold_duration_s", 3.0);
  rfid_event_cooldown_s_ = declare_parameter("rfid_event_cooldown_s", 3.0);
  rfid_hold_duration_s_ = declare_parameter("rfid_hold_duration_s", 3.0);
  rfid_stop_duration_s_ = declare_parameter("rfid_stop_duration_s", 1.0);
  rfid_detection_radius_m_ = declare_parameter("rfid_detection_radius_m", 0.30);
  rfid_correction_tolerance_m_ = declare_parameter("rfid_correction_tolerance_m", 0.025);
  rfid_correction_settle_s_ = declare_parameter("rfid_correction_settle_s", 0.20);
  rfid_correction_timeout_s_ = declare_parameter("rfid_correction_timeout_s", 3.0);
  rfid_scan_max_distance_m_ = declare_parameter("rfid_scan_max_distance_m", 0.20);
  rfid_stop_speed_mps_ = declare_parameter("rfid_stop_speed_mps", 0.02);
  rfid_stationary_position_tolerance_m_ = declare_parameter(
    "rfid_stationary_position_tolerance_m", 0.005);
  rfid_stationary_settle_s_ = declare_parameter("rfid_stationary_settle_s", 0.20);
  rfid_announcement_template_ = declare_parameter(
    "rfid_announcement_template", std::string("到达{number}号巡逻点"));
  rfid_max_speech_retries_ = declare_parameter("rfid_max_speech_retries", 2);
  declare_parameter("auto_start", false);
  declare_parameter("target_ids", "1,2,3,4,5,6,7,8,9,10,11,12");
  odom_timeout_s_ = declare_parameter("odom_timeout_s", 0.6);
  route_ack_timeout_s_ = declare_parameter("route_ack_timeout_s", 3.0);
  replan_delay_s_ = declare_parameter("replan_delay_s", 0.15);
  plan_retry_delay_s_ = declare_parameter("plan_retry_delay_s", 1.0);
  max_plan_retries_ = declare_parameter("max_plan_retries", 3);
  retreat_step_m_ = declare_parameter("retreat_step_m", 0.4);
  max_retreat_extensions_ = declare_parameter("max_retreat_extensions", 4);
  max_no_progress_routes_ = declare_parameter("max_no_progress_routes", 2);
  obstacle_sensor_forward_offset_m_ = declare_parameter(
    "obstacle_sensor_forward_offset_m", camera_x_default());
  obstacle_observation_timeout_s_ = declare_parameter("obstacle_observation_timeout_s", 5.0);
  obstacle_range_sync_s_ = declare_parameter("obstacle_range_sync_s", 0.5);
  obstacle_width_m_ = declare_parameter("obstacle_width_m", 0.24);
  obstacle_depth_m_ = declare_parameter("obstacle_depth_m", 0.12);
  obstacle_merge_distance_m_ = declare_parameter("obstacle_merge_distance_m", 0.18);
  max_dynamic_obstacles_ = declare_parameter("max_dynamic_obstacles", 20);
  dynamic_obstacle_retention_s_ = declare_parameter("dynamic_obstacle_retention_s", 120.0);
  declare_parameter("event_confidence_threshold", 0.70);
  declare_parameter("event_merge_distance_m", 0.35);
  report_requires_hold_ = declare_parameter("report_requires_hold", true);
  report_server_timeout_s_ = declare_parameter("report_server_timeout_s", 5.0);
  max_report_retries_ = declare_parameter("max_report_retries", 2);
}

void MissionManager::load_configuration()
{
  mission_ = std::make_unique<StagedMission>(
    MissionLayout::from_file(get_parameter("map_file").as_string()));
  task_confirmation_mode_ = trim(get_parameter("task_confirmation_mode").as_string());
  if (task_confirmation_mode_ != "rfid" && task_confirmation_mode_ != "route") {
    throw std::invalid_argument("task_confirmation_mode must be 'rfid' or 'route'");
  }
  default_targets_ = parse_ids(get_parameter("target_ids").as_string());
  events_ = std::make_unique<EventRegistry>(
    get_parameter("event_confidence_threshold").as_double(),
    get_parameter("event_merge_distance_m").as_double());
}

void MissionManager::create_interfaces()
{
  const auto durable = rclcpp::QoS(1).reliable().transient_local();
  planner_client_ = create_client<new_map_planner::srv::PlanRoute>(
    get_parameter("planner_service").as_string());
  hold_client_ = create_client<mission_control_interfaces::srv::SetMotionHold>(
    get_parameter("motion_hold_service").as_string());
  retreat_client_ = create_client<mission_control_interfaces::srv::ContinueRetreat>(
    get_parameter("continue_retreat_service").as_string());
  route_retreat_client_ = create_client<mission_control_interfaces::srv::ContinueRetreat>(
    get_parameter("continue_route_retreat_service").as_string());
  reporter_client_ = rclcpp_action::create_client<ReportEvent>(
    this, get_parameter("report_action").as_string());
  speaker_client_ = create_client<mission_control_interfaces::srv::SpeakText>(
    get_parameter("speak_text_service").as_string());
  position_corrector_client_ =
    create_client<mission_control_interfaces::srv::CorrectMapPosition>(
    get_parameter("correct_map_position_service").as_string());
  rfid_checkpoint_client_ = create_client<
    mission_control_interfaces::srv::ConfirmRfidCheckpoint>(
    get_parameter("confirm_rfid_checkpoint_service").as_string());

  state_publisher_ = create_publisher<mission_control_interfaces::msg::MissionState>(
    get_parameter("mission_state_topic").as_string(), durable);
  text_publisher_ = create_publisher<std_msgs::msg::String>(
    get_parameter("status_text_topic").as_string(), durable);
  reported_publisher_ = create_publisher<mission_control_interfaces::msg::EventCandidate>(
    get_parameter("reported_event_topic").as_string(), durable);
  route_publisher_ = create_publisher<nav_msgs::msg::Path>(
    get_parameter("route_input_topic").as_string(), rclcpp::QoS(1).reliable());

  odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
    get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
    std::bind(&MissionManager::on_odometry, this, std::placeholders::_1));
  navigator_subscription_ = create_subscription<std_msgs::msg::String>(
    get_parameter("navigator_status_topic").as_string(), durable,
    std::bind(&MissionManager::on_navigator_status, this, std::placeholders::_1));
  ack_subscription_ = create_subscription<std_msgs::msg::UInt64>(
    get_parameter("route_ack_topic").as_string(), durable,
    std::bind(&MissionManager::on_route_ack, this, std::placeholders::_1));
  hold_state_subscription_ =
    create_subscription<mission_control_interfaces::msg::MotionHoldState>(
    get_parameter("motion_hold_state_topic").as_string(), durable,
    std::bind(&MissionManager::on_hold_state, this, std::placeholders::_1));
  obstacle_subscription_ = create_subscription<std_msgs::msg::Bool>(
    get_parameter("front_obstacle_topic").as_string(), rclcpp::SensorDataQoS(),
    std::bind(&MissionManager::on_front_blocked, this, std::placeholders::_1));
  range_subscription_ = create_subscription<sensor_msgs::msg::Range>(
    get_parameter("front_range_topic").as_string(), rclcpp::SensorDataQoS(),
    std::bind(&MissionManager::on_front_range, this, std::placeholders::_1));
  obstacle_polygon_subscription_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
    get_parameter("dynamic_obstacle_topic").as_string(), 10,
    std::bind(&MissionManager::on_obstacle_polygon, this, std::placeholders::_1));
  event_subscription_ =
    create_subscription<mission_control_interfaces::msg::EventCandidate>(
    get_parameter("event_candidate_topic").as_string(), 10,
    std::bind(&MissionManager::on_event_candidate, this, std::placeholders::_1));
  rfid_subscription_ = create_subscription<std_msgs::msg::Int32>(
    get_parameter("rfid_number_topic").as_string(), 10,
    std::bind(&MissionManager::on_rfid_number, this, std::placeholders::_1));
  current_waypoint_subscription_ = create_subscription<std_msgs::msg::Int32>(
    get_parameter("current_waypoint_topic").as_string(), durable,
    std::bind(&MissionManager::on_current_waypoint, this, std::placeholders::_1));
  speech_subscription_ = create_subscription<std_msgs::msg::String>(
    get_parameter("speech_text_topic").as_string(), 10,
    std::bind(&MissionManager::on_speech_text, this, std::placeholders::_1));

  control_service_ = create_service<mission_control_interfaces::srv::ControlMission>(
    get_parameter("control_service").as_string(),
    std::bind(
      &MissionManager::on_control, this,
      std::placeholders::_1, std::placeholders::_2));
  public_service_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  public_hold_service_ = create_service<mission_control_interfaces::srv::SetMotionHold>(
    get_parameter("public_motion_hold_service").as_string(),
    std::bind(
      &MissionManager::on_public_motion_hold, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, public_service_group_);
  public_retreat_service_ =
    create_service<mission_control_interfaces::srv::ContinueRetreat>(
    get_parameter("public_continue_retreat_service").as_string(),
    std::bind(
      &MissionManager::on_public_continue_retreat, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, public_service_group_);
}

void MissionManager::tick()
{
  const double now = steady_seconds();
  expire_dynamic_obstacles(now);
  if (auto_start_pending_ && ready()) {
    auto_start_pending_ = false;
    start_mission(default_targets_);
  }
  if (plan_due_at_.has_value() && now >= *plan_due_at_) {
    plan_due_at_.reset();
    request_plan();
  }
  if (state_ == mission_control_interfaces::msg::MissionState::WAIT_ROUTE_ACK &&
    ack_deadline_ > 0.0 && now >= ack_deadline_)
  {
    fault("路线 ACK 超时");
  }
  service_event_reporting(now);
  service_rfid(now);
  if (state_ == mission_control_interfaces::msg::MissionState::WAIT_READY && ready()) {
    set_state(mission_control_interfaces::msg::MissionState::IDLE, "系统就绪");
  }
  publish_state();
}

bool MissionManager::ready() const
{
  return planner_client_->service_is_ready() && hold_client_->service_is_ready() &&
         retreat_client_->service_is_ready() && route_retreat_client_->service_is_ready() &&
         rfid_checkpoint_client_->service_is_ready() &&
         pose_.has_value() &&
         pose_->fresh(odom_timeout_s_);
}

void MissionManager::set_state(uint8_t state, const std::string & detail)
{
  const bool changed = state_ != state || detail_ != detail;
  const uint8_t previous = state_;
  state_ = state;
  detail_ = detail;
  if (changed) {
    RCLCPP_INFO(
      get_logger(), "mission state: %s -> %s; detail=%s",
      state_name(previous).c_str(), state_name(state).c_str(), detail.c_str());
    publish_state();
  }
}

mission_control_interfaces::msg::MissionState MissionManager::state_message() const
{
  mission_control_interfaces::msg::MissionState message;
  message.header.stamp = now();
  message.state = state_;
  message.state_name = state_name(state_);
  message.stage = mission_->current_stage();
  message.message = detail_;
  message.active_route_id = active_route_.has_value() ? active_route_->route_id : 0U;
  message.autonomous = mission_->running() && !mission_->paused();
  {
    std::lock_guard<std::mutex> lock(hold_state_mutex_);
    message.motion_held = motion_held_;
  }
  message.remaining_tasks = mission_->remaining_tasks().size();
  message.remaining_tunnels = mission_->remaining_tunnels().size();
  message.remaining_roads = mission_->remaining_roads().size();
  message.dynamic_obstacles = dynamic_obstacles_.size();
  message.confirmed_events = events_->reported_count();
  message.pending_reports = events_->pending_count();
  message.completed_task_ids.assign(
    mission_->completed_tasks().begin(), mission_->completed_tasks().end());
  message.completed_tunnel_ids.assign(
    mission_->completed_tunnels().begin(), mission_->completed_tunnels().end());
  message.completed_lane_ids.assign(
    mission_->confirmed_lanes().begin(), mission_->confirmed_lanes().end());
  return message;
}

void MissionManager::publish_state()
{
  state_publisher_->publish(state_message());
  publish_text(state_name(state_) + ":" + detail_);
}

void MissionManager::publish_text(const std::string & text)
{
  std_msgs::msg::String message;
  message.data = text;
  text_publisher_->publish(message);
}

void MissionManager::on_hold_state(
  const mission_control_interfaces::msg::MotionHoldState::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(hold_state_mutex_);
  motion_held_ = message->held;
  active_hold_sources_ = {message->active_sources.begin(), message->active_sources.end()};
}

void MissionManager::request_hold(
  const std::string & suffix, bool hold, const std::string & reason)
{
  if (!hold_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "motion hold service is not ready");
    return;
  }
  auto request = std::make_shared<mission_control_interfaces::srv::SetMotionHold::Request>();
  request->source = "navigation_mission_manager/" + suffix;
  request->hold = hold;
  request->reason = reason;
  hold_client_->async_send_request(
    request,
    [this](rclcpp::Client<mission_control_interfaces::srv::SetMotionHold>::SharedFuture future) {
      on_hold_response(future);
    });
}

void MissionManager::on_hold_response(
  rclcpp::Client<mission_control_interfaces::srv::SetMotionHold>::SharedFuture future)
{
  try {
    const auto response = future.get();
    std::lock_guard<std::mutex> lock(hold_state_mutex_);
    motion_held_ = response->motion_held;
    if (!response->success) {
      RCLCPP_ERROR(get_logger(), "motion hold rejected: %s", response->message.c_str());
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "motion hold request failed: %s", error.what());
  }
}

void MissionManager::request_shutdown_hold()
{
  request_hold("shutdown", true, "mission manager is shutting down");
}

geometry_msgs::msg::Pose MissionManager::pose_message() const
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose_->x;
  message.position.y = pose_->y;
  message.orientation.z = std::sin(0.5 * pose_->yaw);
  message.orientation.w = std::cos(0.5 * pose_->yaw);
  return message;
}

geometry_msgs::msg::Polygon MissionManager::polygon_message(const Polygon2d & polygon) const
{
  geometry_msgs::msg::Polygon message;
  for (const auto & point : polygon) {
    geometry_msgs::msg::Point32 converted;
    converted.x = static_cast<float>(point.first);
    converted.y = static_cast<float>(point.second);
    message.points.push_back(converted);
  }
  return message;
}

}  // namespace navigation_mission_manager
