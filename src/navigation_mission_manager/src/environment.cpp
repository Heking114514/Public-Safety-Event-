#include "navigation_mission_manager/manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <regex>

namespace navigation_mission_manager
{
namespace
{

bool json_bool(const std::string & json, const std::string & key, bool fallback)
{
  std::smatch match;
  if (std::regex_search(
      json, match, std::regex("\\\"" + key + "\\\"\\s*:\\s*(true|false)")))
  {
    return match[1].str() == "true";
  }
  return fallback;
}

double json_number(const std::string & json, const std::string & key, double fallback)
{
  std::smatch match;
  if (std::regex_search(
      json, match, std::regex(
        "\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)")))
  {
    return std::stod(match[1].str());
  }
  return fallback;
}

std::string json_string(const std::string & json, const std::string & key)
{
  std::smatch match;
  if (std::regex_search(
      json, match, std::regex("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"")))
  {
    return match[1].str();
  }
  return {};
}

std::vector<std::string> json_targets(const std::string & json)
{
  const auto start = json.find("\"targets\"");
  if (start == std::string::npos) {
    return {};
  }
  const auto open = json.find('[', start);
  const auto close = json.find(']', open);
  if (open == std::string::npos || close == std::string::npos) {
    return {};
  }
  const std::string values = json.substr(open + 1U, close - open - 1U);
  std::vector<std::string> targets;
  const std::regex item("\\\"([^\\\"]+)\\\"");
  for (std::sregex_iterator iterator(values.begin(), values.end(), item), end;
    iterator != end; ++iterator)
  {
    const auto value = (*iterator)[1].str();
    if (std::find(targets.begin(), targets.end(), value) == targets.end()) {
      targets.push_back(value);
    }
  }
  return targets;
}

std::string target_announcement(const std::string & target)
{
  if (target.size() < 2U || (target.front() != 'f' && target.front() != 'k')) {
    return {};
  }
  const auto number = target.substr(1U);
  if (number.empty() || number.find_first_not_of("0123456789") != std::string::npos) {
    return {};
  }
  return target.front() == 'k' ? "发现" + number + "号刀具" : "发现" + number + "号嫌疑人";
}

}  // namespace

void MissionManager::on_insight(const std_msgs::msg::String::SharedPtr message)
{
  const auto targets = json_targets(message->data);
  const bool detected = json_bool(message->data, "target_detected", !targets.empty());
  if (!detected || targets.empty()) {
    visual_announced_targets_.clear();
    return;
  }
  for (const auto & target : targets) {
    const auto text = target_announcement(target);
    if (text.empty() || !visual_announced_targets_.insert(target).second) {
      continue;
    }
    enqueue_speech_text(text, false);
    RCLCPP_INFO(get_logger(), "Insight target queued: %s", text.c_str());
  }
}

void MissionManager::on_safe(const std_msgs::msg::String::SharedPtr message)
{
  const auto status = json_string(message->data, "status");
  const bool obstacle = json_bool(
    message->data, "obstacle",
    json_bool(message->data, "obstacle_detected", status == "obstacle"));
  double distance = json_number(message->data, "distance_m", -1.0);
  if (distance < 0.0) {
    const double centimeters = json_number(message->data, "distance_cm", -1.0);
    const double legacy_centimeters = json_number(
      message->data, "obstacle_distance_cm", -1.0);
    const double selected_centimeters = centimeters > 0.0 ? centimeters : legacy_centimeters;
    distance = selected_centimeters > 0.0 ? selected_centimeters * 0.01 : -1.0;
  }

  std_msgs::msg::Bool blocked;
  blocked.data = obstacle;
  // Update the manager before publishing the controller-facing message. The
  // manager also listens to the legacy obstacle topic for physical sensors;
  // setting this state first makes its own adapter output idempotent and
  // prevents a self-published rising edge from starting a second recovery.
  front_blocked_ = obstacle;
  if (!obstacle || distance <= 0.0 || !std::isfinite(distance)) {
    latest_range_.reset();
    visual_blocked_publisher_->publish(blocked);
    return;
  }
  latest_range_ = std::make_pair(distance, steady_seconds());
  record_visual_obstacle(distance);

  visual_blocked_publisher_->publish(blocked);
  sensor_msgs::msg::Range range;
  range.header.stamp = now();
  range.header.frame_id = "base_link";
  range.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  range.min_range = 0.05F;
  range.max_range = 10.0F;
  range.range = static_cast<float>(distance);
  visual_range_publisher_->publish(range);
}

void MissionManager::record_visual_obstacle(double distance)
{
  if (!pose_.has_value() || !pose_->fresh(odom_timeout_s_)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Visual obstacle received without a fresh map/base_link pose");
    return;
  }
  try {
    const auto polygon = front_obstacle_polygon(
      Pose2d{pose_->x, pose_->y, pose_->yaw}, distance,
      obstacle_sensor_forward_offset_m_, obstacle_width_m_, obstacle_depth_m_);
    pending_obstacle_ = std::make_pair(polygon, steady_seconds());
  } catch (const std::invalid_argument &) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Ignoring invalid visual obstacle distance %.3f m", distance);
  }
}

void MissionManager::on_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
{
  if (message->header.frame_id != "map" || message->child_frame_id != "base_link") {
    return;
  }
  const int64_t stamp_ns = rclcpp::Time(message->header.stamp).nanoseconds();
  if (stamp_ns <= 0 || stamp_ns <= last_odom_stamp_ns_) {
    return;
  }
  const auto yaw = quaternion_yaw(message->pose.pose.orientation);
  const double x = message->pose.pose.position.x;
  const double y = message->pose.pose.position.y;
  const double speed = std::abs(message->twist.twist.linear.x);
  if (!yaw.has_value() || !std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(speed))
  {
    return;
  }
  last_odom_stamp_ns_ = stamp_ns;
  pose_ = VehiclePose{x, y, *yaw, speed, steady_seconds()};
}

void MissionManager::on_front_range(const sensor_msgs::msg::Range::SharedPtr message)
{
  const double value = message->range;
  if (std::isfinite(value) && value > 0.0) {
    latest_range_ = std::make_pair(value, steady_seconds());
  }
}

void MissionManager::on_front_blocked(const std_msgs::msg::Bool::SharedPtr message)
{
  const bool rising = message->data && !front_blocked_;
  front_blocked_ = message->data;
  if (!rising || !pose_.has_value() || !latest_range_.has_value()) {
    return;
  }
  const double now = steady_seconds();
  if (now - latest_range_->second > obstacle_range_sync_s_) {
    return;
  }
  try {
    pending_obstacle_ = std::make_pair(
      front_obstacle_polygon(
        Pose2d{pose_->x, pose_->y, pose_->yaw}, latest_range_->first,
        obstacle_sensor_forward_offset_m_, obstacle_width_m_, obstacle_depth_m_),
      now);
  } catch (const std::invalid_argument &) {
  }
}

void MissionManager::on_obstacle_polygon(
  const geometry_msgs::msg::PolygonStamped::SharedPtr message)
{
  if (!message->header.frame_id.empty() && message->header.frame_id != "map") {
    return;
  }
  Polygon2d polygon;
  for (const auto & point : message->polygon.points) {
    polygon.emplace_back(point.x, point.y);
  }
  try {
    pending_obstacle_ = std::make_pair(valid_polygon(polygon), steady_seconds());
  } catch (const std::invalid_argument &) {
  }
}

bool MissionManager::promote_pending_obstacle()
{
  if (!pending_obstacle_.has_value()) {
    return false;
  }
  const double now = steady_seconds();
  if (now - pending_obstacle_->second > obstacle_observation_timeout_s_) {
    pending_obstacle_.reset();
    return false;
  }
  merge_obstacle(
    dynamic_obstacles_, pending_obstacle_->first, obstacle_merge_distance_m_);
  const auto blocked_lane = infer_blocked_lane_id(pending_obstacle_->first);
  if (blocked_lane.has_value() && blocked_lane_ids_.insert(*blocked_lane).second) {
    ++obstacle_revision_;
    publish_text("blocked_lane:" + *blocked_lane + ":revision=" +
      std::to_string(obstacle_revision_));
  }
  if (max_dynamic_obstacles_ > 0 &&
    dynamic_obstacles_.size() > static_cast<std::size_t>(max_dynamic_obstacles_))
  {
    dynamic_obstacles_.erase(
      dynamic_obstacles_.begin(),
      dynamic_obstacles_.end() - max_dynamic_obstacles_);
  }
  dynamic_obstacles_updated_at_ = now;
  pending_obstacle_.reset();
  return true;
}

std::optional<std::string> MissionManager::infer_blocked_lane_id(const Polygon2d & polygon) const
{
  if (!pose_.has_value() || !active_route_.has_value() || polygon.empty()) {
    return std::nullopt;
  }
  const auto center = polygon_center(polygon);
  double best_distance = std::numeric_limits<double>::max();
  std::optional<std::string> result;
  for (const auto & maneuver : active_route_->plan.maneuvers) {
    const auto distance_to = [&center](const geometry_msgs::msg::Pose & pose) {
        return std::hypot(center.first - pose.position.x, center.second - pose.position.y);
      };
    const double distance = std::min(distance_to(maneuver.start_pose), distance_to(maneuver.end_pose));
    const std::string lane_id = !maneuver.outgoing_lane_id.empty() ?
      maneuver.outgoing_lane_id : maneuver.incoming_lane_id;
    if (!lane_id.empty() && distance < best_distance) {
      best_distance = distance;
      result = lane_id;
    }
  }
  return result;
}
void MissionManager::expire_dynamic_obstacles(double now)
{
  if (pose_obstacles_latched_) {
    return;
  }
  if (!dynamic_obstacles_.empty() && obstacle_memory_expired(
      dynamic_obstacles_updated_at_, now, dynamic_obstacle_retention_s_))
  {
    dynamic_obstacles_.clear();
    dynamic_obstacles_updated_at_.reset();
    if (!blocked_lane_ids_.empty()) {
      blocked_lane_ids_.clear();
      ++obstacle_revision_;
    }
    publish_text("dynamic_obstacles_expired");
  }
}


std::optional<EventData> MissionManager::event_data(
  const mission_control_interfaces::msg::EventCandidate & message) const
{
  const auto yaw = quaternion_yaw(message.pose.orientation);
  if (!yaw.has_value()) {
    return std::nullopt;
  }
  return EventData{
    message.candidate_id, message.source, message.event_type,
    message.confidence, message.pose.position.x, message.pose.position.y, *yaw,
    message.evidence_uri, message.announcement, 0};
}

mission_control_interfaces::msg::EventCandidate MissionManager::event_message(
  const EventData & event) const
{
  mission_control_interfaces::msg::EventCandidate message;
  message.header.stamp = now();
  message.header.frame_id = "map";
  message.candidate_id = event.candidate_id;
  message.source = event.source;
  message.event_type = event.event_type;
  message.confidence = static_cast<float>(event.confidence);
  message.pose.position.x = event.x;
  message.pose.position.y = event.y;
  message.pose.orientation.z = std::sin(0.5 * event.yaw);
  message.pose.orientation.w = std::cos(0.5 * event.yaw);
  message.evidence_uri = event.evidence_uri;
  message.announcement = event.announcement;
  return message;
}

void MissionManager::on_event_candidate(
  const mission_control_interfaces::msg::EventCandidate::SharedPtr message)
{
  if (message->header.frame_id != "map" || !mission_->running()) {
    return;
  }
  const auto candidate = event_data(*message);
  if (candidate.has_value() && events_->offer(*candidate)) {
    detail_ = "发现待播报事件：" + candidate->event_type;
    publish_state();
  }
}

void MissionManager::service_event_reporting(double now_seconds)
{
  const EventData * event = events_->current();
  if (event == nullptr || report_goal_pending_ || report_result_pending_) {
    return;
  }
  using State = mission_control_interfaces::msg::MissionState;
  if (state_ == State::PLANNING || state_ == State::REPLANNING ||
    state_ == State::RECOVERING_OBSTACLE || state_ == State::WAIT_ROUTE_ACK ||
    state_ == State::PAUSED || state_ == State::FAULT)
  {
    return;
  }
  bool motion_held = false;
  {
    std::lock_guard<std::mutex> lock(hold_state_mutex_);
    motion_held = motion_held_;
  }
  if (report_requires_hold_ && !motion_held) {
    if (!report_hold_requested_) {
      report_hold_requested_ = true;
      report_wait_started_ = now_seconds;
      request_hold("report", true, "event report in progress");
    }
    if (report_wait_started_.has_value() &&
      now_seconds - *report_wait_started_ > report_server_timeout_s_)
    {
      report_failed("车辆未能为播报驻停");
    } else {
      set_state(State::WAIT_REPORTER, "等待车辆驻停后播报");
    }
    return;
  }
  if (!reporter_client_->action_server_is_ready()) {
    if (!report_wait_started_.has_value()) {
      report_wait_started_ = now_seconds;
    }
    if (now_seconds - *report_wait_started_ > report_server_timeout_s_) {
      report_failed("播报服务未就绪");
    } else {
      set_state(State::WAIT_REPORTER, "等待播报服务");
    }
    return;
  }

  ReportEvent::Goal goal;
  goal.event = event_message(*event);
  const uint64_t generation = ++report_generation_;
  report_goal_pending_ = true;
  rclcpp_action::Client<ReportEvent>::SendGoalOptions options;
  options.goal_response_callback =
    [this, generation](ReportGoalHandle::SharedPtr handle) {
      on_report_goal(generation, handle);
    };
  options.result_callback =
    [this, generation](const ReportGoalHandle::WrappedResult & result) {
      on_report_result(generation, result);
    };
  reporter_client_->async_send_goal(goal, options);
  set_state(State::REPORTING, "正在播报：" + event->event_type);
}

void MissionManager::on_report_goal(uint64_t generation, ReportGoalHandle::SharedPtr handle)
{
  if (generation != report_generation_) {
    if (handle) {
      reporter_client_->async_cancel_goal(handle);
    }
    return;
  }
  report_goal_pending_ = false;
  if (!handle) {
    report_failed("播报请求被拒绝");
    return;
  }
  report_goal_handle_ = handle;
  report_result_pending_ = true;
}

void MissionManager::on_report_result(
  uint64_t generation, const ReportGoalHandle::WrappedResult & wrapped)
{
  if (generation != report_generation_) {
    return;
  }
  report_result_pending_ = false;
  report_goal_handle_.reset();
  if (!wrapped.result || !wrapped.result->success) {
    report_failed(
      wrapped.result && !wrapped.result->message.empty() ?
      wrapped.result->message : "播报失败");
    return;
  }
  const EventData event = events_->mark_reported();
  reported_publisher_->publish(event_message(event));
  const bool held_for_report = report_hold_requested_;
  report_wait_started_.reset();
  report_hold_requested_ = false;
  if (held_for_report) {
    request_hold("report", false, "event report completed");
  }
  if (mission_->finished()) {
    set_state(mission_control_interfaces::msg::MissionState::COMPLETE, "事件播报完成，任务已结束");
  } else if (mission_->paused()) {
    set_state(mission_control_interfaces::msg::MissionState::PAUSED, "事件播报完成，任务保持暂停");
  } else if (active_route_.has_value()) {
    set_state(mission_control_interfaces::msg::MissionState::EXECUTING, "事件播报完成，继续当前路线");
  } else if (mission_->running()) {
    schedule_plan(false);
  } else {
    set_state(mission_control_interfaces::msg::MissionState::IDLE, "事件播报完成");
  }
}

void MissionManager::report_failed(const std::string & message)
{
  const EventData event = events_->mark_attempt();
  report_goal_pending_ = false;
  report_result_pending_ = false;
  report_goal_handle_.reset();
  report_wait_started_ = steady_seconds();
  if (event.attempts > max_report_retries_) {
    fault(message);
  } else {
    set_state(
      mission_control_interfaces::msg::MissionState::WAIT_REPORTER,
      message + "；准备重试");
  }
}

void MissionManager::cancel_report()
{
  ++report_generation_;
  if (report_goal_handle_) {
    reporter_client_->async_cancel_goal(report_goal_handle_);
  }
  report_goal_pending_ = false;
  report_result_pending_ = false;
  report_goal_handle_.reset();
  report_wait_started_.reset();
  if (report_hold_requested_) {
    request_hold("report", false, "event report canceled");
  }
  report_hold_requested_ = false;
}

}  // namespace navigation_mission_manager
