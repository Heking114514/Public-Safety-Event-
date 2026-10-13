#include "navigation_mission_manager/manager.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <future>
#include <stdexcept>

namespace navigation_mission_manager
{
namespace
{

std::string trimmed(const std::string & value)
{
  const auto first = std::find_if_not(
    value.begin(), value.end(), [](unsigned char c) {return std::isspace(c) != 0;});
  const auto last = std::find_if_not(
    value.rbegin(), value.rend(), [](unsigned char c) {return std::isspace(c) != 0;}).base();
  return first < last ? std::string(first, last) : std::string{};
}

}  // namespace

void MissionManager::on_control(
  const mission_control_interfaces::srv::ControlMission::Request::SharedPtr request,
  mission_control_interfaces::srv::ControlMission::Response::SharedPtr response)
{
  try {
    switch (request->command) {
      case mission_control_interfaces::srv::ControlMission::Request::START:
        start_mission(request->target_ids.empty() ? default_targets_ : request->target_ids);
        response->message = "自主任务已启动";
        break;
      case mission_control_interfaces::srv::ControlMission::Request::PAUSE:
        if (!mission_->running()) {
          throw std::runtime_error("没有正在运行的任务");
        }
        mission_->pause();
        ++plan_generation_;
        planning_pending_ = false;
        pending_plan_generation_ = 0U;
        plan_due_at_.reset();
        request_hold("operator", true, "operator paused mission");
        set_state(mission_control_interfaces::msg::MissionState::PAUSED, "任务已暂停");
        response->message = "暂停请求已接受";
        break;
      case mission_control_interfaces::srv::ControlMission::Request::RESUME:
        if (!mission_->running() || !mission_->paused()) {
          throw std::runtime_error("任务不在暂停状态");
        }
        mission_->resume();
        request_hold("operator", false, "operator resumed mission");
        request_hold("fault", false, "operator resumed mission");
        if (active_route_.has_value()) {
          set_state(mission_control_interfaces::msg::MissionState::EXECUTING, "继续当前路线");
        } else {
          schedule_plan(false);
        }
        response->message = "继续请求已接受";
        break;
      case mission_control_interfaces::srv::ControlMission::Request::ABORT:
        abort_mission();
        response->message = "任务已终止并驻停";
        break;
      case mission_control_interfaces::srv::ControlMission::Request::RESET:
        reset_mission();
        response->message = "任务状态已复位";
        break;
      default:
        throw std::runtime_error("未知任务命令");
    }
    response->accepted = true;
  } catch (const std::exception & error) {
    response->accepted = false;
    response->message = error.what();
  }
  response->state = state_message();
}

void MissionManager::on_public_motion_hold(
  const mission_control_interfaces::srv::SetMotionHold::Request::SharedPtr request,
  mission_control_interfaces::srv::SetMotionHold::Response::SharedPtr response)
{
  const auto snapshot = [this, &response]() {
      std::lock_guard<std::mutex> lock(hold_state_mutex_);
      response->motion_held = motion_held_;
      response->active_sources.assign(active_hold_sources_.begin(), active_hold_sources_.end());
    };
  const std::string source = trimmed(request->source);
  if (source.empty() || source.rfind("navigation_mission_manager/", 0) == 0U) {
    response->success = false;
    response->message = "external source must be non-empty and cannot use an internal prefix";
    snapshot();
    return;
  }
  if (!hold_client_->service_is_ready()) {
    response->success = false;
    response->message = "navigation hold service is not ready";
    snapshot();
    return;
  }

  auto forwarded = std::make_shared<mission_control_interfaces::srv::SetMotionHold::Request>();
  forwarded->source = "navigation_mission_manager/external/" + source;
  forwarded->hold = request->hold;
  forwarded->reason = request->reason;
  auto future = hold_client_->async_send_request(forwarded);
  if (future.wait_for(std::chrono::duration<double>(public_service_timeout_s_)) !=
    std::future_status::ready)
  {
    response->success = false;
    response->message = "navigation hold service timed out";
    snapshot();
    return;
  }
  try {
    const auto result = future.get();
    response->success = result->success;
    response->motion_held = result->motion_held;
    response->active_sources = result->active_sources;
    response->message = result->message;
  } catch (const std::exception & error) {
    response->success = false;
    response->message = std::string("navigation hold service failed: ") + error.what();
    snapshot();
  }
}

void MissionManager::on_public_continue_retreat(
  const mission_control_interfaces::srv::ContinueRetreat::Request::SharedPtr request,
  mission_control_interfaces::srv::ContinueRetreat::Response::SharedPtr response)
{
  if (pose_recovery_enabled_) {
    response->message = "pose recovery uses candidate selection; legacy retreat is disabled";
    return;
  }
  if (!mission_running_public_.load()) {
    response->message = "no active mission";
    return;
  }
  if (!retreat_client_->service_is_ready()) {
    response->message = "obstacle retreat service is not ready";
    return;
  }
  auto forwarded =
    std::make_shared<mission_control_interfaces::srv::ContinueRetreat::Request>();
  forwarded->minimum_distance_m = request->minimum_distance_m;
  auto future = retreat_client_->async_send_request(forwarded);
  if (future.wait_for(std::chrono::duration<double>(public_service_timeout_s_)) !=
    std::future_status::ready)
  {
    response->message = "obstacle retreat service timed out";
    return;
  }
  try {
    const auto result = future.get();
    response->accepted = result->accepted;
    response->trace_exhausted = result->trace_exhausted;
    response->remaining_breadcrumbs = result->remaining_breadcrumbs;
    response->message = result->message;
  } catch (const std::exception & error) {
    response->message = std::string("obstacle retreat service failed: ") + error.what();
  }
}

}  // namespace navigation_mission_manager
