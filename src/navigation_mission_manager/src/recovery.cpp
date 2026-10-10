#include "navigation_mission_manager/manager.hpp"

#include <cmath>
#include <set>

namespace navigation_mission_manager
{

void MissionManager::on_navigation_recovery_state(
  const mission_control_interfaces::msg::NavigationRecoveryState::SharedPtr message)
{
  latest_navigation_recovery_ = *message;
  RCLCPP_INFO(
    get_logger(),
    "收到控制器恢复锚点: valid=%s reason=%s route=%lu pose=(%.3f, %.3f)",
    message->valid ? "true" : "false", message->reason.c_str(),
    static_cast<unsigned long>(message->route_id),
    message->safe_pose.pose.position.x, message->safe_pose.pose.position.y);
  if (turn_replan_waiting_for_recovery_ && message->valid) {
    handle_turn_replan();
  }
}

void MissionManager::on_navigator_status(const std_msgs::msg::String::SharedPtr message)
{
  const std::string status = message->data;
  navigator_status_ = status;
  if (task_confirmation_mode_ == "route" && !rfid_active_.has_value() &&
    status.rfind("RFID_SCANNING:", 0U) == 0U)
  {
    const auto checkpoint = expected_rfid_checkpoint();
    if (checkpoint.has_value() && active_route_.has_value()) {
      const auto task = mission_->layout().supernode_task_ids.find(checkpoint->second);
      if (task != mission_->layout().supernode_task_ids.end()) {
        rfid_active_ = std::stoi(task->second);
        rfid_active_supernode_ = checkpoint->second;
        rfid_active_waypoint_index_ = checkpoint->first;
        rfid_active_route_id_ = active_route_->route_id;
        rfid_skip_correction_ = true;
        rfid_speech_required_ = false;
        rfid_phase_ = "waiting_hold";
        request_hold("rfid", true, "loopback RFID checkpoint " + rfid_active_supernode_);
      }
    }
  }
  if (status == "EXPECTED_OBSTACLE_AT_TURN") {
    pending_obstacle_.reset();
    return;
  }
  if (status == "ROUTE_TRANSITION_BRAKING" ||
    status == "ROUTE_TRANSITION_REVERSING" ||
    status == "ROUTE_TRANSITION_REPLAYING")
  {
    if (route_transition_recovery_active_) {
      set_state(
        mission_control_interfaces::msg::MissionState::REPLANNING,
        status);
    }
    return;
  }
  if (status == "ROUTE_TRANSITION_REPLAN_REQUIRED") {
    if (route_transition_recovery_active_) {
      awaiting_retreat_ = false;
      schedule_plan(true);
    }
    return;
  }
  if (status == "OBSTACLE_BRAKING" || status == "OBSTACLE_REPLAYING" ||
    status == "OBSTACLE_REVERSING" || status == "RETREATING_TO_JUNCTION" ||
    status == "RETREATING_TO_ROUTE_START")
  {
    if (status == "RETREATING_TO_JUNCTION" || status == "RETREATING_TO_ROUTE_START") {
      awaiting_heading_retreat_ = true;
      obstacle_recovery_active_ = true;
      retreat_extensions_ = 0;
    }
    if (active_route_.has_value() || awaiting_retreat_) {
      set_state(mission_control_interfaces::msg::MissionState::RECOVERING_OBSTACLE, status);
    }
    return;
  }
  if (status == "OBSTACLE_REPLAN_REQUIRED") {
    if (awaiting_heading_retreat_) {
      awaiting_heading_retreat_ = false;
      clear_active_route();
      obstacle_recovery_active_ = false;
      completed_route_retreat_available_ = false;
      schedule_plan(true);
      return;
    }
    handle_obstacle_replan();
    return;
  }
  if (status == "TURN_NEED_REPLAN") {
    // A turn failure is recovered by the controller's executed-trace replay.
    // Do not feed the in-turn pose into the planner or start a second recovery
    // request while the replay is already active.
    if (route_transition_recovery_active_ || awaiting_retreat_ || retreat_pending_) {
      return;
    }
    turn_replan_waiting_for_recovery_ = true;
    handle_turn_replan();
    return;
  }
  if (status == "GOAL_REACHED") {
    handle_goal_reached();
    return;
  }
  const bool navigation_owned = active_route_.has_value() || obstacle_recovery_active_ ||
    route_transition_recovery_active_;
  if (navigation_owned &&
    (status.rfind("FAULT_", 0) == 0U || status == "TURN_BLOCKED"))
  {
    fault("导航故障：" + status);
    return;
  }
  if (active_route_.has_value() && active_route_->acknowledged) {
    const std::set<std::string> inactive{
      "IDLE", "WAITING_FOR_ROUTE", "WAITING_FOR_LOCALIZATION"};
    if (inactive.count(status) == 0U) {
      active_route_->motion_seen = true;
    }
  }
}

void MissionManager::handle_turn_replan()
{
  if (!mission_->running() || !active_route_.has_value() ||
    !active_route_->acknowledged || planning_pending_ || plan_due_at_.has_value())
  {
    return;
  }
  const uint64_t route_id = active_route_->route_id;
  if (!latest_navigation_recovery_.has_value() ||
    !latest_navigation_recovery_->valid ||
    latest_navigation_recovery_->route_id != route_id)
  {
    RCLCPP_WARN(
      get_logger(),
      "路口恢复使用控制器执行轨迹回放；当前路线尚无有效安全锚点，继续等待");
    return;
  }

  // The safe pose is evidence that the controller stopped after a validated
  // turn command. It is deliberately not used as a planner start pose: the
  // controller owns the reverse replay and closes the loop against its trace.
  clear_active_route();
  turn_replan_waiting_for_recovery_ = false;
  route_transition_recovery_active_ = true;
  retreat_extensions_ = 0;
  // The controller starts replay before publishing TURN_NEED_REPLAN. Do not
  // issue a second retreat request; wait for ROUTE_TRANSITION_REPLAN_REQUIRED.
  RCLCPP_INFO(get_logger(),
    "路口恢复已由控制器接管历史轨迹回放: route=%lu",
    static_cast<unsigned long>(route_id));
}

std::optional<std::pair<std::string, std::string>>
MissionManager::turn_recovery_context() const
{
  if (!pose_.has_value() || !pose_->fresh(odom_timeout_s_) ||
    !active_route_.has_value())
  {
    return std::nullopt;
  }
  constexpr double kMaximumRecoveryContextDistance = 0.35;
  double best_distance = kMaximumRecoveryContextDistance;
  std::optional<std::pair<std::string, std::string>> result;
  for (const auto & maneuver : active_route_->plan.maneuvers) {
    if (maneuver.type != new_map_planner::msg::Maneuver::TURN_LEFT &&
      maneuver.type != new_map_planner::msg::Maneuver::TURN_RIGHT &&
      maneuver.type != new_map_planner::msg::Maneuver::START_ALIGN)
    {
      continue;
    }
    if (maneuver.node_id.empty() || maneuver.incoming_lane_id.empty()) {
      continue;
    }
    const auto distance_to = [this](const geometry_msgs::msg::Pose & pose) {
        return std::hypot(pose_->x - pose.position.x, pose_->y - pose.position.y);
      };
    const double distance = std::min(
      distance_to(maneuver.start_pose), distance_to(maneuver.end_pose));
    if (distance < best_distance) {
      best_distance = distance;
      result = std::make_pair(maneuver.node_id, maneuver.incoming_lane_id);
    }
  }
  return result;
}

void MissionManager::handle_obstacle_replan()
{
  if (awaiting_retreat_) {
    awaiting_retreat_ = false;
    schedule_plan(true);
    return;
  }
  if (!active_route_.has_value() || !active_route_->acknowledged) {
    return;
  }
  if (!active_route_->motion_seen) {
    fault("控制器拒绝了尚未开始执行的规划路线");
    return;
  }
  clear_active_route();
  if (!promote_pending_obstacle()) {
    fault("控制器请求避障重规划，但没有可用障碍位置");
    return;
  }
  obstacle_recovery_active_ = true;
  retreat_extensions_ = 0;
  schedule_plan(true);
}

void MissionManager::handle_plan_rejection(const std::string & message)
{
  if (obstacle_recovery_active_) {
    request_additional_retreat(message);
  } else if (route_transition_recovery_active_) {
    request_additional_retreat(message);
  } else if (completed_route_retreat_available_) {
    completed_route_retreat_available_ = false;
    route_transition_recovery_active_ = true;
    retreat_extensions_ = 0;
    request_additional_retreat(message);
  } else {
    plan_failed(message);
  }
}

void MissionManager::request_additional_retreat(const std::string & planner_message)
{
  if (retreat_pending_) {
    return;
  }
  if (retreat_extensions_ >= max_retreat_extensions_) {
    fault(
      (route_transition_recovery_active_ ?
      "路线衔接仍无路，已达到最大倒退次数：" :
      "绕障规划仍无路，已达到最大继续倒退次数：") + planner_message);
    return;
  }
  const auto client = route_transition_recovery_active_ ?
    route_retreat_client_ : retreat_client_;
  if (!client->service_is_ready()) {
    fault(route_transition_recovery_active_ ?
      "路线衔接倒退服务不可用" : "控制器继续倒退服务不可用");
    return;
  }
  auto request =
    std::make_shared<mission_control_interfaces::srv::ContinueRetreat::Request>();
  request->minimum_distance_m = retreat_step_m_;
  const uint64_t generation = ++retreat_generation_;
  retreat_pending_ = true;
  client->async_send_request(
    request,
    [this, generation, planner_message](
      rclcpp::Client<mission_control_interfaces::srv::ContinueRetreat>::SharedFuture future) {
      on_retreat_response(generation, planner_message, future);
    });
  set_state(
    route_transition_recovery_active_ ?
    mission_control_interfaces::msg::MissionState::REPLANNING :
    mission_control_interfaces::msg::MissionState::RECOVERING_OBSTACLE,
    (route_transition_recovery_active_ ?
    "路线衔接无路，请求沿历史轨迹倒退：" :
    "当前恢复姿态无路，请求继续倒退：") + planner_message);
}

void MissionManager::on_retreat_response(
  uint64_t generation, const std::string & planner_message,
  rclcpp::Client<mission_control_interfaces::srv::ContinueRetreat>::SharedFuture future)
{
  (void)planner_message;
  if (generation != retreat_generation_) {
    return;
  }
  retreat_pending_ = false;
  mission_control_interfaces::srv::ContinueRetreat::Response::SharedPtr response;
  try {
    response = future.get();
  } catch (const std::exception & error) {
    fault(std::string("继续倒退服务失败：") + error.what());
    return;
  }
  if (!response->accepted) {
    const std::string reason = response->trace_exhausted ?
      "已执行的安全轨迹耗尽" : response->message;
    fault(
      (route_transition_recovery_active_ ?
      "路线衔接仍无路且无法继续倒退：" :
      "绕障规划仍无路且无法继续倒退：") + reason);
    return;
  }
  ++retreat_extensions_;
  awaiting_retreat_ = true;
  set_state(
    route_transition_recovery_active_ ?
    mission_control_interfaces::msg::MissionState::REPLANNING :
    mission_control_interfaces::msg::MissionState::RECOVERING_OBSTACLE,
    (route_transition_recovery_active_ ?
    "路线衔接沿历史轨迹倒退 (" :
    "控制器继续沿历史轨迹倒退 (") +
    std::to_string(retreat_extensions_) + "/" +
    std::to_string(max_retreat_extensions_) + ")");
}

void MissionManager::clear_obstacle_recovery()
{
  ++retreat_generation_;
  retreat_pending_ = false;
  awaiting_retreat_ = false;
  awaiting_heading_retreat_ = false;
  obstacle_recovery_active_ = false;
  route_transition_recovery_active_ = false;
  retreat_extensions_ = 0;
}

}  // namespace navigation_mission_manager
