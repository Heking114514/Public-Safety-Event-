#include "navigation_mission_manager/manager.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace navigation_mission_manager
{
namespace
{
bool valid_exit_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
    quaternion_yaw(pose.orientation).has_value();
}
}  // namespace

void MissionManager::clear_pose_recovery()
{
  if (latest_navigation_recovery_.has_value() &&
    !latest_navigation_recovery_->phase.empty() &&
    latest_navigation_recovery_->recovery_id != 0U)
  {
    pose_recovery_.retired.insert({latest_navigation_recovery_->route_id,
      latest_navigation_recovery_->recovery_id});
  }
  pose_recovery_.reset();
  recovery_candidates_.clear();
  recovery_candidate_index_ = 0;
  recovery_evaluation_pending_ = false;
  recovery_selection_pending_ = false;
  recovery_selection_accepted_ = false;
  recovery_handed_off_ = false;
}

void MissionManager::preserve_interrupted_progress()
{
  if (!active_route_.has_value() || !active_route_->acknowledged) {
    return;
  }
  const auto & route = *active_route_;
  const auto & path = route.plan.navigation_path.poses;
  std::vector<std::string> lanes;
  // Route waypoints are ordered. Only a DRIVE whose end waypoint has already
  // been passed can be committed; the currently targeted edge is unfinished.
  std::size_t cursor = 0;
  for (const auto & maneuver : route.plan.maneuvers) {
    if (maneuver.type != new_map_planner::msg::Maneuver::DRIVE) {
      continue;
    }
    std::size_t end = cursor;
    while (end < path.size() && std::hypot(
        path[end].pose.position.x - maneuver.end_pose.position.x,
        path[end].pose.position.y - maneuver.end_pose.position.y) > 1.0e-5)
    {
      ++end;
    }
    if (end == path.size()) {
      break;  // Missing metadata cannot be turned into claimed progress.
    }
    cursor = end + 1;
    if (end >= static_cast<std::size_t>(std::max(0, current_waypoint_index_))) {
      break;
    }
    const auto & lane = maneuver.incoming_lane_id;
    if (std::find(route.plan.covered_lane_ids.begin(), route.plan.covered_lane_ids.end(),
      lane) != route.plan.covered_lane_ids.end())
    {
      lanes.push_back(lane);
    }
  }
  std::vector<std::string> tasks(rfid_confirmed_tasks_.begin(), rfid_confirmed_tasks_.end());
  // In route-confirmation mode, successful checkpoint authorizations are the
  // evidence; the future planned visit_order is never committed on interruption.
  if (task_confirmation_mode_ == "route") {
    for (const auto index : rfid_confirmed_waypoints_) {
      if (index >= route.plan.waypoint_supernode_ids.size()) {
        continue;
      }
      const auto task = mission_->layout().supernode_task_ids.find(
        route.plan.waypoint_supernode_ids[index]);
      if (task != mission_->layout().supernode_task_ids.end()) {
        tasks.push_back(task->second);
      }
    }
  }
  mission_->commit(route.spec, tasks, lanes, false);
  publish_text("recovery_progress_preserved:lanes=" + std::to_string(lanes.size()) +
    ":tasks=" + std::to_string(tasks.size()));
}

void MissionManager::on_pose_recovery(
  const mission_control_interfaces::msg::NavigationRecoveryState & state)
{
  if (!mission_->running() || state.recovery_id == 0U) {
    return;
  }
  if (mission_->paused()) {
    if (pose_recovery_.matches(state.route_id, state.recovery_id, state.obstacle_revision)) {
      latest_navigation_recovery_ = state;
    }
    return;
  }
  if (!pose_recovery_.active && state.phase == "braking") {
    if (!active_route_.has_value() || state.route_id != active_route_->route_id ||
      pose_recovery_.retired.count({state.route_id, state.recovery_id}) != 0U)
    {
      return;
    }
    if (!latest_navigation_recovery_.has_value() ||
      latest_navigation_recovery_->recovery_id != state.recovery_id ||
      latest_navigation_recovery_->route_id != state.route_id)
    {
      ++plan_generation_;
      planning_pending_ = false;
      pending_plan_generation_ = 0;
      plan_due_at_.reset();
    }
    latest_navigation_recovery_ = state;
    set_state(mission_control_interfaces::msg::MissionState::RECOVERING_OBSTACLE,
      "pose recovery braking; waiting for measured obstacle candidates");
    return;
  }
  if (!pose_recovery_.active) {
    // A durable completed/failed sample cannot start a new transaction. A route
    // match excludes stale samples after abort/reset and after normal replanning.
    if (state.phase != "candidates" ||
      !active_route_.has_value() || state.route_id != active_route_->route_id ||
      !pose_recovery_.begin(state.route_id, state.recovery_id, state.obstacle_revision))
    {
      return;
    }
    Polygon2d polygon;
    for (const auto & point : state.obstacle.points) {
      polygon.emplace_back(point.x, point.y);
    }
    try {
      polygon = valid_polygon(polygon);
      if (state.blocked_lane_id.empty()) {
        throw std::runtime_error("pose recovery missing blocked physical lane");
      }
      preserve_interrupted_progress();
    } catch (const std::exception & error) {
      fault(std::string("恢复锁存失败：") + error.what());
      return;
    }
    // The controller's real polygon replaces any estimated pending observation.
    // This snapshot is latched once for this recovery, not refreshed by phases.
    merge_obstacle(dynamic_obstacles_, polygon, obstacle_merge_distance_m_);
    blocked_lane_ids_.insert(state.blocked_lane_id);
    ++obstacle_revision_;
    recovery_snapshot_revision_ = obstacle_revision_;
    pose_obstacles_latched_ = true;
    pending_obstacle_.reset();
    dynamic_obstacles_updated_at_ = steady_seconds();
    mission_->skip_blocked_roads(blocked_lane_ids_);
    publish_text("pose_recovery_latched:id=" + std::to_string(state.recovery_id) +
      ":controller_revision=" + std::to_string(state.obstacle_revision) +
      ":snapshot_revision=" + std::to_string(obstacle_revision_) +
      ":blocked_lane=" + state.blocked_lane_id);
    // Cancel all ordinary requests before releasing the interrupted route.
    ++plan_generation_;
    planning_pending_ = false;
    pending_plan_generation_ = 0;
    plan_due_at_.reset();
    ++retreat_generation_;
    retreat_pending_ = awaiting_retreat_ = awaiting_heading_retreat_ = false;
    obstacle_recovery_active_ = route_transition_recovery_active_ = false;
    turn_replan_waiting_for_recovery_ = false;
    completed_route_retreat_available_ = false;
    next_planning_context_ = PlanningContext{};
    clear_active_route();
    // A partially completed RFID workflow cannot authorize the interrupted
    // route during recovery. Confirmed task evidence was committed above.
    if (rfid_active_.has_value()) {
      request_hold("rfid", false, "pose recovery superseded RFID workflow");
    }
    rfid_active_.reset();
    rfid_active_route_id_ = 0;
    rfid_active_supernode_.clear();
    rfid_phase_ = "idle";
    rfid_authorization_pending_ = false;
    rfid_correction_pending_ = false;
    rfid_speech_pending_ = false;
    route_checkpoint_authorizations_pending_.clear();
  }
  if (!pose_recovery_.matches(state.route_id, state.recovery_id, state.obstacle_revision) ||
    recovery_handed_off_)
  {
    return;
  }
  latest_navigation_recovery_ = state;
  if (state.phase == "complete") {
    if ((!recovery_selection_pending_ && !recovery_selection_accepted_) || !state.valid ||
      !valid_exit_pose(state.safe_pose.pose) || state.resume_node.empty() ||
      state.resume_lane.empty() || blocked_lane_ids_.count(state.resume_lane) != 0U ||
      !std::isfinite(state.resume_heading) ||
      (!state.safe_pose.header.frame_id.empty() && state.safe_pose.header.frame_id != "map"))
    {
      fault("恢复完成消息缺少有效实际出口或选择交接");
      return;
    }
    const double yaw = *quaternion_yaw(state.safe_pose.pose.orientation);
    if (std::abs(std::atan2(std::sin(yaw - state.resume_heading),
      std::cos(yaw - state.resume_heading))) > 0.20)
    {
      fault("恢复实际出口航向与安全位姿不一致");
      return;
    }
    recovery_handed_off_ = true;
    recovery_evaluation_pending_ = recovery_selection_pending_ = false;
    next_planning_context_ = PlanningContext{};
    next_planning_context_.has_safe_pose = true;
    next_planning_context_.safe_pose = state.safe_pose.pose;
    ordinary_nodes_since_task_ = 0;
    plan_retries_ = 0;
    completed_route_retreat_available_ = false;
    publish_text("pose_recovery_complete:id=" + std::to_string(state.recovery_id) +
      ":resume_node=" + state.resume_node + ":resume_lane=" + state.resume_lane +
      ":ordinary_nodes_since_task=0");
    schedule_plan(true);  // NORMAL, measured exit, no replay/extra retreat.
    return;
  }
  if (state.phase == "failed") {
    // A synchronous local planning failure is reported by the select response;
    // it is safe to try another candidate after that response, never concurrently.
    if (!recovery_selection_pending_) {
      fault("恢复运动失败并安全停车：" + state.reason);
    }
    return;
  }
  set_state(mission_control_interfaces::msg::MissionState::RECOVERING_OBSTACLE,
    "pose recovery " + state.phase);
  if (state.phase == "candidates" && recovery_candidates_.empty() &&
    !recovery_selection_accepted_)
  {
    std::set<uint32_t> seen;
    for (const auto & candidate : state.candidates) {
      if (!seen.insert(candidate.candidate_id).second || candidate.resume_node.empty() ||
        candidate.resume_lane.empty() || !valid_exit_pose(candidate.exit_pose) ||
        !std::isfinite(candidate.resume_heading) || !std::isfinite(candidate.motion_cost) ||
        recovery_borrows_blocked_lane(
          candidate.borrowed_lane_ids, candidate.resume_lane, blocked_lane_ids_))
      {
        continue;
      }
      recovery_candidates_.push_back(candidate);
    }
    std::stable_sort(recovery_candidates_.begin(), recovery_candidates_.end(),
      [](const auto & a, const auto & b) {return a.motion_cost < b.motion_cost;});
    if (recovery_candidates_.empty()) {
      fault("没有不借用阻塞道路的有效恢复出口");
      return;
    }
  }
  service_pose_recovery();
}

void MissionManager::service_pose_recovery()
{
  if (pose_recovery_.active && !recovery_handed_off_ && !mission_->paused() &&
    latest_navigation_recovery_.has_value() && latest_navigation_recovery_->phase == "complete")
  {
    on_pose_recovery(*latest_navigation_recovery_);
    return;
  }
  if (!pose_recovery_.active || recovery_handed_off_ || recovery_evaluation_pending_ ||
    recovery_selection_pending_ || recovery_selection_accepted_ ||
    recovery_candidates_.empty() || !mission_->running() || mission_->paused())
  {
    return;
  }
  if (obstacle_revision_ != recovery_snapshot_revision_) {
    fault("恢复期间障碍快照发生变化，保持安全停车");
    return;
  }
  if (recovery_candidate_index_ >= recovery_candidates_.size()) {
    fault("恢复候选均不可达或局部运动规划失败，保持安全停车");
    return;
  }
  if (!planner_client_->service_is_ready() || !select_recovery_client_->service_is_ready()) {
    return;
  }
  evaluate_recovery_candidate();
}

void MissionManager::evaluate_recovery_candidate()
{
  const auto & candidate = recovery_candidates_[recovery_candidate_index_];
  if (recovery_borrows_blocked_lane(
      candidate.borrowed_lane_ids, candidate.resume_lane, blocked_lane_ids_))
  {
    ++recovery_candidate_index_;
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
  request->start = candidate.exit_pose;
  request->start_mode = new_map_planner::srv::PlanRoute::Request::START_MODE_NORMAL;
  request->mode = spec.mode;
  request->target_ids = spec.target_ids;
  request->required_lane_ids = spec.required_lane_ids;
  request->completed_lane_ids = spec.completed_lane_ids;
  request->return_home = spec.return_home;
  request->allow_partial = spec.allow_partial;
  request->activate = false;
  request->validate_geometry = true;
  request->online_geometry_validation = true;
  request->ordinary_nodes_since_task = 0;
  request->ordinary_nodes_since_task_exact = true;
  for (const auto & obstacle : dynamic_obstacles_) {
    request->dynamic_obstacles.push_back(polygon_message(obstacle));
  }
  fill_blocked_lane_snapshot(*request, blocked_lane_ids_, obstacle_revision_);
  recovery_evaluation_pending_ = true;
  const uint64_t generation = pose_recovery_.generation;
  const uint64_t revision = obstacle_revision_;
  const uint32_t candidate_id = candidate.candidate_id;
  publish_text("pose_recovery_evaluate:candidate=" + std::to_string(candidate_id));
  planner_client_->async_send_request(request,
    [this, generation, revision, candidate_id, spec](
      rclcpp::Client<new_map_planner::srv::PlanRoute>::SharedFuture future)
    {
      on_recovery_evaluation(generation, revision, candidate_id, spec, future);
    });
}

void MissionManager::on_recovery_evaluation(
  uint64_t generation, uint64_t revision, uint32_t candidate_id, const PlanSpec & spec,
  rclcpp::Client<new_map_planner::srv::PlanRoute>::SharedFuture future)
{
  if (!pose_recovery_.active || generation != pose_recovery_.generation ||
    recovery_handed_off_ || revision != obstacle_revision_ ||
    !mission_->running() ||
    recovery_candidate_index_ >= recovery_candidates_.size() ||
    candidate_id != recovery_candidates_[recovery_candidate_index_].candidate_id)
  {
    return;
  }
  recovery_evaluation_pending_ = false;
  if (mission_->paused()) {
    return;
  }
  try {
    const auto response = future.get();
    const auto & plan = response->plan;
    const bool empty_home = spec.mode == "layer3" && spec.required_lane_ids.empty() &&
      plan.complete;
    if (response->success && !response->activated && !plan.recovery_only &&
      plan.collision_checked && plan.plan_id != 0U &&
      obstacle_snapshot_matches(plan, blocked_lane_ids_, obstacle_revision_) &&
      (empty_home || (!plan.navigation_path.poses.empty() &&
      spec.has_progress(plan.visit_order, plan.covered_lane_ids))))
    {
      // Evaluation never publishes/activates a Path. Only the controller service
      // receives the finite candidate selection and the current blocked lanes.
      select_recovery_candidate(candidate_id);
      return;
    }
    RCLCPP_WARN(get_logger(), "恢复候选 %u 下游不可达: %s", candidate_id,
      response->message.c_str());
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "恢复候选 %u 评估失败: %s", candidate_id, error.what());
  }
  ++recovery_candidate_index_;
  service_pose_recovery();
}

void MissionManager::select_recovery_candidate(uint32_t candidate_id)
{
  auto request = std::make_shared<mission_control_interfaces::srv::SelectRecovery::Request>();
  request->recovery_id = pose_recovery_.recovery_id;
  request->obstacle_revision = pose_recovery_.revision;
  request->candidate_id = candidate_id;
  request->blocked_lane_ids.assign(blocked_lane_ids_.begin(), blocked_lane_ids_.end());
  recovery_selection_pending_ = true;
  const uint64_t generation = pose_recovery_.generation;
  const uint64_t revision = obstacle_revision_;
  publish_text("pose_recovery_select:candidate=" + std::to_string(candidate_id));
  select_recovery_client_->async_send_request(request,
    [this, generation, revision, candidate_id](
      rclcpp::Client<mission_control_interfaces::srv::SelectRecovery>::SharedFuture future)
    {
      on_recovery_selection(generation, revision, candidate_id, future);
    });
}

void MissionManager::on_recovery_selection(
  uint64_t generation, uint64_t revision, uint32_t candidate_id,
  rclcpp::Client<mission_control_interfaces::srv::SelectRecovery>::SharedFuture future)
{
  if (!pose_recovery_.active || generation != pose_recovery_.generation ||
    recovery_handed_off_ || revision != obstacle_revision_ || !mission_->running() ||
    recovery_candidate_index_ >= recovery_candidates_.size() ||
    candidate_id != recovery_candidates_[recovery_candidate_index_].candidate_id)
  {
    return;
  }
  recovery_selection_pending_ = false;
  try {
    const auto response = future.get();
    if (response->success) {
      recovery_selection_accepted_ = true;
      return;
    }
    RCLCPP_WARN(get_logger(), "恢复候选 %u 局部规划失败: %s", candidate_id,
      response->message.c_str());
  } catch (const std::exception & error) {
    RCLCPP_WARN(get_logger(), "恢复选择服务失败: %s", error.what());
  }
  ++recovery_candidate_index_;
  service_pose_recovery();
}

}  // namespace navigation_mission_manager
