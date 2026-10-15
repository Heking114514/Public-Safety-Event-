#include "navigation_mission_manager/manager.hpp"

namespace navigation_mission_manager
{
bool MissionManager::request_road_recovery()
{
  if (!pose_recovery_enabled_ || !mission_->running() || mission_->paused() ||
    active_route_.has_value() || !completed_route_retreat_available_ ||
    completed_route_id_ == 0U || blocked_lane_ids_.empty() || dynamic_obstacles_.empty() ||
    road_recovery_route_id_ != 0U || !pose_.has_value() || !pose_->fresh(odom_timeout_s_))
  {
    return false;
  }
  if (!road_recovery_client_->service_is_ready()) {
    return false;
  }
  clear_pose_recovery();
  latest_navigation_recovery_.reset();
  auto request = std::make_shared<mission_control_interfaces::srv::StartRoadRecovery::Request>();
  request->route_id = completed_route_id_;
  request->obstacle_revision = obstacle_revision_;
  request->blocked_lane_ids.assign(blocked_lane_ids_.begin(), blocked_lane_ids_.end());
  for (const auto & polygon : dynamic_obstacles_) {
    request->obstacles.push_back(polygon_message(polygon));
  }
  // 在发服务前登记，braking/candidates 消息可以先于服务响应到达。
  road_recovery_route_id_ = request->route_id;
  road_recovery_revision_ = obstacle_revision_;
  completed_route_retreat_available_ = false;
  plan_due_at_.reset();
  const auto generation = ++retreat_generation_;
  const auto revision = obstacle_revision_;
  road_recovery_client_->async_send_request(request,
    [this, generation, revision](
      rclcpp::Client<mission_control_interfaces::srv::StartRoadRecovery>::SharedFuture future)
    {
      if (generation != retreat_generation_ || revision != obstacle_revision_ ||
        road_recovery_route_id_ == 0U || !mission_->running() || mission_->paused())
      {
        return;
      }
      try {
        const auto response = future.get();
        if (!response->accepted) {
          fault("已知封路恢复被拒绝：" + response->message);
        }
      } catch (const std::exception & error) {
        fault(std::string("已知封路恢复请求失败：") + error.what());
      }
    });
  set_state(mission_control_interfaces::msg::MissionState::RECOVERING_OBSTACLE,
    "已知障碍导致前进无路，停车后沿原道路退至 T/X 固定入口");
  return true;
}
}  // namespace navigation_mission_manager
