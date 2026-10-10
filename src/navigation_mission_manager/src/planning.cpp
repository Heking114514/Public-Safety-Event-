#include "navigation_mission_manager/manager.hpp"

#include <algorithm>
#include <stdexcept>

namespace navigation_mission_manager
{

void MissionManager::start_mission(const std::vector<std::string> & targets)
{
  if (mission_->running() || planning_pending_) {
    throw std::runtime_error("已有任务正在运行");
  }
  if (report_goal_pending_ || report_result_pending_) {
    throw std::runtime_error("事件播报尚未完成");
  }
  if (!ready()) {
    throw std::runtime_error("规划器、控制器或融合里程计尚未就绪");
  }
  mission_->start(targets);
  mission_running_public_.store(true);
  events_->reset();
  reset_rfid();
  dynamic_obstacles_.clear();
  dynamic_obstacles_updated_at_.reset();
  pending_obstacle_.reset();
  front_blocked_ = false;
  latest_range_.reset();
  clear_active_route();
  clear_obstacle_recovery();
  completed_route_retreat_available_ = false;
  next_planning_context_ = PlanningContext{};
  completed_stages_ = 0;
  ordinary_nodes_since_task_ = 1;
  plan_retries_ = 0;
  no_progress_routes_ = 0;
  request_hold("operator", false, "new mission started");
  request_hold("fault", false, "new mission started");
  request_hold("report", false, "new mission started");
  schedule_plan(false);
}

void MissionManager::abort_mission()
{
  mission_->abort();
  mission_running_public_.store(false);
  cancel_report();
  events_->reset();
  reset_rfid();
  ++plan_generation_;
  planning_pending_ = false;
  pending_plan_generation_ = 0U;
  plan_due_at_.reset();
  pending_obstacle_.reset();
  clear_active_route();
  clear_obstacle_recovery();
  completed_route_retreat_available_ = false;
  next_planning_context_ = PlanningContext{};
  request_hold("operator", true, "mission aborted");
  set_state(mission_control_interfaces::msg::MissionState::IDLE, "任务已终止，车辆保持驻停");
}

void MissionManager::reset_mission()
{
  cancel_report();
  mission_->reset();
  mission_running_public_.store(false);
  events_->reset();
  reset_rfid();
  dynamic_obstacles_.clear();
  dynamic_obstacles_updated_at_.reset();
  pending_obstacle_.reset();
  ++plan_generation_;
  planning_pending_ = false;
  pending_plan_generation_ = 0U;
  plan_due_at_.reset();
  clear_active_route();
  clear_obstacle_recovery();
  completed_route_retreat_available_ = false;
  next_planning_context_ = PlanningContext{};
  request_hold("operator", true, "mission reset");
  set_state(mission_control_interfaces::msg::MissionState::IDLE, "任务状态已复位，车辆保持驻停");
}

void MissionManager::schedule_plan(bool replanning)
{
  replanning_ = replanning;
  plan_due_at_ = steady_seconds() + (replanning ? std::max(0.0, replan_delay_s_) : 0.0);
  set_state(
    replanning ? mission_control_interfaces::msg::MissionState::REPLANNING :
    mission_control_interfaces::msg::MissionState::PLANNING,
    replanning ? "等待最新位姿重新规划" : "正在规划");
}

void MissionManager::request_plan()
{
  if (planning_pending_) {
    plan_due_at_ = steady_seconds() + 0.10;
    return;
  }
  if (!mission_->running()) {
    return;
  }
  if (mission_->paused()) {
    set_state(mission_control_interfaces::msg::MissionState::PAUSED, "任务已暂停");
    return;
  }
  if (!ready()) {
    plan_due_at_ = steady_seconds() + 0.25;
    set_state(
      mission_control_interfaces::msg::MissionState::WAIT_READY,
      "等待新鲜融合位姿、规划和控制服务");
    return;
  }

  PlanSpec spec;
  try {
    spec = mission_->plan_spec();
  } catch (const std::exception & error) {
    fault(error.what());
    return;
  }
  auto request = std::make_shared<new_map_planner::srv::PlanRoute::Request>();
  request->start = pose_message();
  const PlanningContext context = next_planning_context_;
  request->start_mode = context.start_mode;
  request->recovery_anchor_node_id = context.recovery_anchor_node_id;
  request->incoming_lane_id = context.recovery_incoming_lane_id;
  request->allow_reverse_departure = context.allow_reverse_departure;
  next_planning_context_ = PlanningContext{};
  request->mode = spec.mode;
  request->target_ids = spec.target_ids;
  request->required_lane_ids = spec.required_lane_ids;
  request->completed_lane_ids = spec.completed_lane_ids;
  for (const auto & obstacle : dynamic_obstacles_) {
    request->dynamic_obstacles.push_back(polygon_message(obstacle));
  }
  request->return_home = spec.return_home;
  request->allow_partial = spec.allow_partial;
  request->activate = false;
  request->ordinary_nodes_since_task = ordinary_nodes_since_task_;
  const uint64_t generation = ++plan_generation_;
  planning_pending_ = true;
  pending_plan_generation_ = generation;
  planner_client_->async_send_request(
    request,
    [this, generation, spec, context](
      rclcpp::Client<new_map_planner::srv::PlanRoute>::SharedFuture future) {
      on_plan_response(generation, spec, context, future);
    });
  set_state(
    replanning_ ? mission_control_interfaces::msg::MissionState::REPLANNING :
    mission_control_interfaces::msg::MissionState::PLANNING,
    "规划 " + spec.mode);
}

void MissionManager::on_plan_response(
  uint64_t generation, const PlanSpec & spec,
  const PlanningContext & context,
  rclcpp::Client<new_map_planner::srv::PlanRoute>::SharedFuture future)
{
  if (generation != plan_generation_) {
    if (pending_plan_generation_ == generation) {
      planning_pending_ = false;
      pending_plan_generation_ = 0U;
    }
    return;
  }
  planning_pending_ = false;
  pending_plan_generation_ = 0U;
  new_map_planner::srv::PlanRoute::Response::SharedPtr response;
  try {
    response = future.get();
  } catch (const std::exception & error) {
    if (context.start_mode ==
      new_map_planner::srv::PlanRoute::Request::START_MODE_TURN_RECOVERY)
    {
      fault(std::string("转弯恢复规划服务调用失败：") + error.what());
      return;
    }
    plan_failed(std::string("规划服务调用失败：") + error.what());
    return;
  }
  if (!response->success) {
    if (context.start_mode ==
      new_map_planner::srv::PlanRoute::Request::START_MODE_TURN_RECOVERY)
    {
      fault("转弯恢复规划失败：" + response->message);
      return;
    }
    handle_plan_rejection("规划失败：" + response->message);
    return;
  }
  const auto & plan = response->plan;
  const bool recovery_request = context.start_mode ==
    new_map_planner::srv::PlanRoute::Request::START_MODE_TURN_RECOVERY;
  if (recovery_request != plan.recovery_only) {
    handle_plan_rejection(recovery_request ?
      "恢复请求返回了普通路线" : "普通请求返回了恢复路线");
    return;
  }
  if (plan.navigation_path.poses.empty()) {
    if (recovery_request) {
      fault("转弯恢复规划没有可执行倒车路线");
      return;
    }
    if (plan.complete && spec.mode == "layer3") {
      try {
        mission_->commit_empty(spec, true);
      } catch (const std::exception & error) {
        fault(error.what());
        return;
      }
      ++completed_stages_;
      finish_mission();
    } else {
      handle_plan_rejection("规划结果没有可执行路线");
    }
    return;
  }
  if (!plan.collision_checked || plan.plan_id == 0U) {
    handle_plan_rejection("规划结果未通过碰撞验证");
    return;
  }
  if (plan.waypoint_supernode_ids.size() != plan.navigation_path.poses.size()) {
    handle_plan_rejection("规划结果中的超节点元数据与路径长度不一致");
    return;
  }
  for (const auto & supernode_id : plan.waypoint_supernode_ids) {
    if (!supernode_id.empty() &&
      mission_->layout().supernode_task_ids.count(supernode_id) == 0U)
    {
      handle_plan_rejection("规划结果包含未配置的 RFID 超节点：" + supernode_id);
      return;
    }
  }
  if (!plan.recovery_only && !spec.has_progress(plan.visit_order, plan.covered_lane_ids)) {
    handle_plan_rejection("规划结果没有新增当前阶段进度");
    return;
  }
  plan_retries_ = 0;
  rfid_confirmed_waypoints_.clear();
  route_checkpoint_authorizations_pending_.clear();
  current_waypoint_index_ = 0;
  clear_obstacle_recovery();
  completed_route_retreat_available_ = false;
  active_route_ = ActiveRoute{plan.plan_id, spec, plan, false, false};
  ack_deadline_ = steady_seconds() + route_ack_timeout_s_;
  set_state(
    mission_control_interfaces::msg::MissionState::WAIT_ROUTE_ACK,
    "等待路线 " + std::to_string(plan.plan_id) + " ACK");
  route_publisher_->publish(plan.navigation_path);
  if (ack_cache_.count(plan.plan_id) != 0U) {
    accept_ack(plan.plan_id);
  }
}

void MissionManager::plan_failed(const std::string & message)
{
  ++plan_retries_;
  if (plan_retries_ > max_plan_retries_) {
    fault(message);
    return;
  }
  plan_due_at_ = steady_seconds() + std::max(0.1, plan_retry_delay_s_);
  set_state(
    mission_control_interfaces::msg::MissionState::REPLANNING,
    message + "；准备重试");
}

void MissionManager::on_route_ack(const std_msgs::msg::UInt64::SharedPtr message)
{
  ack_cache_.insert(message->data);
  while (ack_cache_.size() > 64U) {
    ack_cache_.erase(ack_cache_.begin());
  }
  accept_ack(message->data);
}

void MissionManager::accept_ack(uint64_t route_id)
{
  if (!active_route_.has_value() || route_id != active_route_->route_id) {
    return;
  }
  active_route_->acknowledged = true;
  ack_deadline_ = 0.0;
  set_state(
    mission_control_interfaces::msg::MissionState::EXECUTING,
    "路线 " + std::to_string(route_id) + " 执行中");
}

void MissionManager::handle_goal_reached()
{
  if (!active_route_.has_value() || !active_route_->acknowledged ||
    !active_route_->motion_seen)
  {
    return;
  }
  const ActiveRoute route = *active_route_;
  if (route.plan.recovery_only) {
    clear_active_route();
    next_planning_context_ = PlanningContext{};
    schedule_plan(true);
    return;
  }
  const std::string previous_stage = route.spec.mode;
  const std::vector<std::string> confirmed = task_confirmation_mode_ == "route" ?
    route.plan.visit_order :
    std::vector<std::string>(rfid_confirmed_tasks_.begin(), rfid_confirmed_tasks_.end());
  std::size_t progress = 0U;
  try {
    progress = mission_->commit(
      route.spec, confirmed, route.plan.covered_lane_ids, route.plan.complete);
  } catch (const std::exception & error) {
    fault(error.what());
    return;
  }
  ordinary_nodes_since_task_ = route.plan.ordinary_nodes_since_task;
  clear_active_route();
  completed_route_retreat_available_ = true;
  const std::string current_stage = mission_->current_stage();
  if (mission_->finished()) {
    ++completed_stages_;
    finish_mission();
    return;
  }
  if (current_stage != previous_stage) {
    ++completed_stages_;
    publish_text("stage_complete:" + previous_stage);
  }
  no_progress_routes_ = progress == 0U ? no_progress_routes_ + 1 : 0;
  if (no_progress_routes_ > max_no_progress_routes_) {
    fault("连续路线没有新增任务进度");
    return;
  }
  schedule_plan(false);
}

void MissionManager::finish_mission()
{
  mission_running_public_.store(false);
  if (events_->pending_count() > 0U) {
    set_state(
      mission_control_interfaces::msg::MissionState::WAIT_REPORTER,
      "导航完成，等待事件播报");
  } else {
    set_state(
      mission_control_interfaces::msg::MissionState::COMPLETE,
      "三阶段自主任务完成");
  }
  publish_text("mission_complete:stages=" + std::to_string(completed_stages_));
}

void MissionManager::clear_active_route()
{
  active_route_.reset();
  ack_deadline_ = 0.0;
}

void MissionManager::fault(const std::string & message)
{
  if (mission_->running()) {
    mission_->pause();
  }
  ++plan_generation_;
  planning_pending_ = false;
  pending_plan_generation_ = 0U;
  plan_due_at_.reset();
  clear_active_route();
  clear_obstacle_recovery();
  completed_route_retreat_available_ = false;
  next_planning_context_ = PlanningContext{};
  cancel_report();
  request_hold("fault", true, message);
  set_state(mission_control_interfaces::msg::MissionState::FAULT, message);
}

}  // namespace navigation_mission_manager
