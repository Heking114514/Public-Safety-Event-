#include "navigation_mission_manager/manager.hpp"

#include <algorithm>

namespace navigation_mission_manager
{
namespace
{

constexpr const char * kRfidHoldSource = "navigation_mission_manager/rfid";
constexpr const char * kSpeechHoldSource = "navigation_mission_manager/speech";

const std::map<int, std::string> kNumberNames{
  {1, "一"}, {2, "二"}, {3, "三"}, {4, "四"}, {5, "五"}, {6, "六"},
  {7, "七"}, {8, "八"}, {9, "九"}, {10, "十"}, {11, "十一"}, {12, "十二"}};

std::string render_announcement(std::string value, const std::string & number)
{
  const std::string token = "{number}";
  const auto position = value.find(token);
  if (position != std::string::npos) {
    value.replace(position, token.size(), number);
  }
  return value;
}

}  // namespace

void MissionManager::reset_rfid()
{
  bool release_rfid = rfid_active_.has_value();
  bool release_speech = speech_active_.has_value() && speech_active_->request_hold;
  {
    std::lock_guard<std::mutex> lock(hold_state_mutex_);
    release_rfid = release_rfid || active_hold_sources_.count(kRfidHoldSource) != 0U;
    release_speech = release_speech || active_hold_sources_.count(kSpeechHoldSource) != 0U;
  }
  if (release_rfid && hold_client_) {
    request_hold("rfid", false, "RFID workflow reset");
  }
  if (release_speech && hold_client_) {
    request_hold("speech", false, "speech workflow reset");
  }
  rfid_recent_.clear();
  rfid_confirmed_waypoints_.clear();
  route_checkpoint_authorizations_pending_.clear();
  rfid_confirmed_tasks_.clear();
  rfid_announced_.clear();
  rfid_active_.reset();
  rfid_active_card_ = 0;
  rfid_active_supernode_.clear();
  rfid_active_waypoint_index_ = 0U;
  rfid_active_route_id_ = 0U;
  rfid_phase_ = "idle";
  rfid_release_at_.reset();
  rfid_correction_stable_since_.reset();
  rfid_stationary_since_.reset();
  rfid_correction_deadline_ = 0.0;
  rfid_authorization_pending_ = false;
  rfid_skip_correction_ = false;
  rfid_speech_pending_ = false;
  rfid_correction_pending_ = false;
  rfid_speech_attempts_ = 0;
  rfid_next_speech_at_ = 0.0;
  speech_queue_.clear();
  speech_active_.reset();
  speech_release_at_.reset();
  speech_pending_ = false;
}

void MissionManager::on_speech_text(const std_msgs::msg::String::SharedPtr message)
{
  enqueue_speech_text(message->data, true);
}

void MissionManager::enqueue_speech_text(const std::string & input, bool request_hold)
{
  std::string text = input;
  const auto first = text.find_first_not_of(" \t");
  const auto last = text.find_last_not_of(" \t");
  text = first == std::string::npos ? std::string{} : text.substr(first, last - first + 1U);
  if (text.empty() || text.size() > 128U || text.find('\n') != std::string::npos ||
    text.find('\r') != std::string::npos)
  {
    return;
  }
  // A single insight event may contain several people and knives. Keep the
  // queue large enough to preserve that order while retaining a bounded
  // safety limit for malformed or overly chatty publishers.
  if (speech_queue_.size() < 64U) {
    speech_queue_.push_back(SpeechItem{text, request_hold});
  }
}

void MissionManager::service_speech_text(double now_seconds)
{
  if (!speech_active_.has_value() && !speech_queue_.empty()) {
    speech_active_ = speech_queue_.front();
    speech_queue_.pop_front();
    speech_release_at_.reset();
    if (speech_active_->request_hold) {
      request_hold("speech", true, "mission speech request");
    }
  }
  if (!speech_active_.has_value()) {
    return;
  }
  if (speech_active_->request_hold) {
    bool held = false;
    {
      std::lock_guard<std::mutex> lock(hold_state_mutex_);
      held = active_hold_sources_.count(kSpeechHoldSource) != 0U;
    }
    if (!held) {
      return;
    }
  }
  if (!speech_pending_ && !speech_release_at_.has_value() &&
    speaker_client_->service_is_ready())
  {
    auto request = std::make_shared<mission_control_interfaces::srv::SpeakText::Request>();
    request->text = speech_active_->text;
    speech_pending_ = true;
    if (speech_active_->request_hold) {
      speech_release_at_ = now_seconds + speech_hold_duration_s_;
    }
    const bool request_hold = speech_active_->request_hold;
    speaker_client_->async_send_request(
      request,
      [this, request_hold](
        rclcpp::Client<mission_control_interfaces::srv::SpeakText>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_ERROR(
              get_logger(), "mission speech failed: %s", response->message.c_str());
          }
          if (!request_hold) {
            speech_active_.reset();
            speech_release_at_.reset();
            speech_pending_ = false;
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "mission speech service failed: %s", error.what());
          if (!request_hold) {
            speech_active_.reset();
            speech_release_at_.reset();
            speech_pending_ = false;
          }
        }
      });
  } else if (speech_release_at_.has_value() && now_seconds >= *speech_release_at_) {
    request_hold("speech", false, "mission speech completed");
    speech_active_.reset();
    speech_release_at_.reset();
    speech_pending_ = false;
  }
}

void MissionManager::on_rfid_number(const std_msgs::msg::Int32::SharedPtr message)
{
  const int number = message->data;
  if (number < 1 || number > 12) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "RFID card number %d is outside the reader's supported range", number);
    return;
  }
  if (!mission_->running() || mission_->paused() || !active_route_.has_value() ||
    !active_route_->acknowledged || rfid_active_.has_value())
  {
    return;
  }
  const auto checkpoint = expected_rfid_checkpoint();
  if (!checkpoint.has_value()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Ignoring RFID card %d outside the expected route supernode window", number);
    return;
  }
  const std::string & supernode_id = checkpoint->second;
  const auto task = mission_->layout().supernode_task_ids.find(supernode_id);
  if (task == mission_->layout().supernode_task_ids.end()) {
    fault("RFID 超节点没有对应的地图任务点：" + supernode_id);
    return;
  }
  const double now_seconds = steady_seconds();
  const auto recent = rfid_recent_.find(checkpoint->first);
  if (recent != rfid_recent_.end() && now_seconds - recent->second < rfid_event_cooldown_s_) {
    return;
  }
  rfid_recent_[checkpoint->first] = now_seconds;
  rfid_active_ = std::stoi(task->second);
  rfid_active_card_ = number;
  rfid_active_supernode_ = supernode_id;
  rfid_active_waypoint_index_ = checkpoint->first;
  rfid_active_route_id_ = active_route_->route_id;
  rfid_phase_ = "waiting_hold";
  rfid_release_at_.reset();
  rfid_correction_stable_since_.reset();
  rfid_stationary_since_.reset();
  rfid_correction_deadline_ = now_seconds + rfid_correction_timeout_s_;
  rfid_speech_required_ = mission_->current_stage() == "layer1" &&
    std::find(mission_->requested_tasks().begin(), mission_->requested_tasks().end(), task->second) !=
    mission_->requested_tasks().end() && !rfid_announcement_template_.empty();
  rfid_speech_attempts_ = 0;
  rfid_next_speech_at_ = now_seconds;
  request_hold(
    "rfid", true,
    "RFID " + std::to_string(number) + " confirmed at " + supernode_id);
  publish_text("rfid_detected:card=" + std::to_string(number) + ",supernode=" + supernode_id);
  RCLCPP_INFO(
    get_logger(), "RFID checkpoint captured: card=%d supernode=%s task=%s waypoint=%zu",
    number, supernode_id.c_str(), task->second.c_str(), checkpoint->first);
}

std::optional<std::pair<std::size_t, std::string>>
MissionManager::expected_rfid_checkpoint() const
{
  if (!pose_.has_value() || !pose_->fresh(odom_timeout_s_) || !active_route_.has_value()) {
    return std::nullopt;
  }
  const auto & plan = active_route_->plan;
  for (std::size_t index = 0; index < plan.waypoint_supernode_ids.size(); ++index) {
    const std::string & supernode_id = plan.waypoint_supernode_ids[index];
    if (supernode_id.empty() || rfid_confirmed_waypoints_.count(index) != 0U) {
      continue;
    }
    if (index < static_cast<std::size_t>(std::max<int32_t>(0, current_waypoint_index_))) {
      return std::nullopt;
    }
    const auto & point = plan.navigation_path.poses[index].pose.position;
    const double distance = std::hypot(pose_->x - point.x, pose_->y - point.y);
    const bool scanning = index == static_cast<std::size_t>(current_waypoint_index_) &&
      (navigator_status_.rfind("RFID_SCANNING:", 0U) == 0U ||
       navigator_status_.rfind("WAITING_FOR_RFID:", 0U) == 0U);
    const double radius = rfid_detection_radius_m_ +
      (scanning ? rfid_scan_max_distance_m_ : 0.0);
    if (distance <= radius) {
      return std::make_pair(index, supernode_id);
    }
    return std::nullopt;
  }
  return std::nullopt;
}

void MissionManager::on_current_waypoint(const std_msgs::msg::Int32::SharedPtr message)
{
  current_waypoint_index_ = message->data;
}

void MissionManager::service_rfid(double now_seconds)
{
  service_speech_text(now_seconds);
  if (!rfid_active_.has_value()) {
    return;
  }
  if (rfid_phase_ == "waiting_hold") {
    bool held = false;
    {
      std::lock_guard<std::mutex> lock(hold_state_mutex_);
      held = active_hold_sources_.count(kRfidHoldSource) != 0U;
    }
    if (!held) {
      return;
    }
    if (!pose_.has_value() || !pose_->fresh(odom_timeout_s_)) {
      rfid_stationary_since_.reset();
      return;
    }
    if (pose_->linear_speed > rfid_stop_speed_mps_) {
      rfid_stationary_since_.reset();
      rfid_stationary_reference_x_ = pose_->x;
      rfid_stationary_reference_y_ = pose_->y;
      return;
    }
    if (!rfid_stationary_since_.has_value()) {
      rfid_stationary_since_ = now_seconds;
      rfid_stationary_reference_x_ = pose_->x;
      rfid_stationary_reference_y_ = pose_->y;
      return;
    }
    if (std::hypot(
        pose_->x - rfid_stationary_reference_x_,
        pose_->y - rfid_stationary_reference_y_) >
      rfid_stationary_position_tolerance_m_)
    {
      rfid_stationary_since_ = now_seconds;
      rfid_stationary_reference_x_ = pose_->x;
      rfid_stationary_reference_y_ = pose_->y;
      return;
    }
    if (now_seconds - *rfid_stationary_since_ < rfid_stationary_settle_s_) {
      return;
    }
    rfid_phase_ = "correcting";
    rfid_correction_deadline_ = now_seconds + rfid_correction_timeout_s_;
    if (rfid_skip_correction_) {
      begin_rfid_post_speech(now_seconds, false);
      return;
    }
  }
  if (rfid_phase_ == "correcting") {
    if (now_seconds >= rfid_correction_deadline_) {
      fault("RFID map correction service timed out at " + rfid_active_supernode_);
      return;
    }
    if (!rfid_correction_pending_ && position_corrector_client_->service_is_ready()) {
      const auto found = mission_->layout().task_positions.find(
        mission_->layout().supernode_task_ids.at(rfid_active_supernode_));
      if (found == mission_->layout().task_positions.end()) {
        fault("No map position configured for " + rfid_active_supernode_);
        return;
      }
      auto request =
        std::make_shared<mission_control_interfaces::srv::CorrectMapPosition::Request>();
      request->x = found->second.first;
      request->y = found->second.second;
      request->source = kRfidHoldSource;
      rfid_correction_pending_ = true;
      position_corrector_client_->async_send_request(
        request,
        [this](
          rclcpp::Client<mission_control_interfaces::srv::CorrectMapPosition>::SharedFuture future)
        {
          on_rfid_correction(rfid_active_.value_or(0), future);
        });
    }
    return;
  }
  if (rfid_phase_ == "waiting_correction") {
    if (now_seconds >= rfid_correction_deadline_) {
      fault("RFID map correction did not converge at " + rfid_active_supernode_);
      return;
    }
    const auto & task_id = mission_->layout().supernode_task_ids.at(rfid_active_supernode_);
    const auto target = mission_->layout().task_positions.at(task_id);
    const bool converged = pose_.has_value() && pose_->fresh(odom_timeout_s_) &&
      std::hypot(pose_->x - target.first, pose_->y - target.second) <=
      rfid_correction_tolerance_m_;
    if (!converged) {
      rfid_correction_stable_since_.reset();
      return;
    }
    if (!rfid_correction_stable_since_.has_value()) {
      rfid_correction_stable_since_ = now_seconds;
      return;
    }
    if (now_seconds - *rfid_correction_stable_since_ < rfid_correction_settle_s_) {
      return;
    }
    rfid_phase_ = "announcing";
  }
  if (rfid_phase_ == "announcing") {
    if (rfid_speech_required_ && rfid_announced_.count(rfid_active_card_) == 0U) {
      send_rfid_speech(now_seconds);
    } else if (!rfid_release_at_.has_value()) {
      begin_rfid_post_speech(now_seconds, rfid_speech_required_);
    }
  }
  if (rfid_phase_ == "post_speech" && rfid_release_at_.has_value() &&
    now_seconds >= *rfid_release_at_)
  {
    rfid_phase_ = "authorizing";
  }
  if (rfid_phase_ == "authorizing" && !rfid_authorization_pending_ &&
    rfid_checkpoint_client_->service_is_ready())
  {
    auto request = std::make_shared<
      mission_control_interfaces::srv::ConfirmRfidCheckpoint::Request>();
    request->route_id = rfid_active_route_id_;
    request->waypoint_index = static_cast<uint32_t>(rfid_active_waypoint_index_);
    request->supernode_id = rfid_active_supernode_;
    rfid_authorization_pending_ = true;
    rfid_checkpoint_client_->async_send_request(
      request,
      [this, route_id = rfid_active_route_id_, index = rfid_active_waypoint_index_](
        rclcpp::Client<mission_control_interfaces::srv::ConfirmRfidCheckpoint>::SharedFuture future)
      {
        on_rfid_checkpoint_authorization(route_id, index, false, future);
      });
  }
}

void MissionManager::send_rfid_speech(double now_seconds)
{
  if (!rfid_active_.has_value() || rfid_active_card_ <= 0 || rfid_speech_pending_ ||
    !speaker_client_->service_is_ready() || now_seconds < rfid_next_speech_at_)
  {
    return;
  }
  const int number = rfid_active_card_;
  auto request = std::make_shared<mission_control_interfaces::srv::SpeakText::Request>();
  request->text = render_announcement(rfid_announcement_template_, kNumberNames.at(number));
  ++rfid_speech_attempts_;
  rfid_speech_pending_ = true;
  speaker_client_->async_send_request(
    request,
    [this, number](
      rclcpp::Client<mission_control_interfaces::srv::SpeakText>::SharedFuture future)
    {
      on_rfid_speech(number, future);
    });
}

void MissionManager::on_rfid_speech(
  int number,
  rclcpp::Client<mission_control_interfaces::srv::SpeakText>::SharedFuture future)
{
  if (!rfid_active_.has_value() || number != rfid_active_card_) {
    return;
  }
  rfid_speech_pending_ = false;
  mission_control_interfaces::srv::SpeakText::Response::SharedPtr response;
  try {
    response = future.get();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "RFID speech service failed: %s", error.what());
  }
  if (response && response->success) {
    rfid_announced_.insert(number);
    publish_text("rfid_announced:" + std::to_string(number));
    return;
  }
  if (rfid_speech_attempts_ > rfid_max_speech_retries_) {
    const std::string detail = response ? response->message : "service exception";
    RCLCPP_ERROR(
      get_logger(), "RFID %d announcement failed: %s", number, detail.c_str());
    begin_rfid_post_speech(steady_seconds(), true);
  } else {
    rfid_next_speech_at_ = steady_seconds() + 0.2;
  }
}

void MissionManager::on_rfid_correction(
  int number,
  rclcpp::Client<mission_control_interfaces::srv::CorrectMapPosition>::SharedFuture future)
{
  if (!rfid_active_.has_value() || number != *rfid_active_) {
    return;
  }
  rfid_correction_pending_ = false;
  try {
    const auto response = future.get();
    if (!response->accepted) {
      fault("RFID position correction rejected at " + rfid_active_supernode_ +
        ": " + response->message);
      return;
    }
  } catch (const std::exception & error) {
    fault(std::string("RFID position correction service failed: ") + error.what());
    return;
  }
  rfid_phase_ = "waiting_correction";
  rfid_correction_deadline_ = steady_seconds() + rfid_correction_timeout_s_;
  rfid_correction_stable_since_.reset();
  rfid_stationary_since_.reset();
}

void MissionManager::begin_rfid_post_speech(double now_seconds, bool speech_required)
{
  rfid_phase_ = "post_speech";
  rfid_release_at_ = now_seconds +
    (speech_required ? rfid_hold_duration_s_ : rfid_stop_duration_s_);
}

void MissionManager::on_rfid_checkpoint_authorization(
  uint64_t route_id, std::size_t waypoint_index, bool route_mode,
  rclcpp::Client<mission_control_interfaces::srv::ConfirmRfidCheckpoint>::SharedFuture future)
{
  mission_control_interfaces::srv::ConfirmRfidCheckpoint::Response::SharedPtr response;
  try {
    response = future.get();
  } catch (const std::exception & error) {
    if (route_mode) {
      route_checkpoint_authorizations_pending_.erase(waypoint_index);
    } else {
      rfid_authorization_pending_ = false;
    }
    fault(std::string("RFID checkpoint authorization failed: ") + error.what());
    return;
  }
  if (!response->accepted) {
    if (route_mode) {
      route_checkpoint_authorizations_pending_.erase(waypoint_index);
    } else {
      rfid_authorization_pending_ = false;
    }
    fault("RFID checkpoint authorization rejected: " + response->message);
    return;
  }
  if (route_mode) {
    if (!active_route_.has_value() || active_route_->route_id != route_id ||
      waypoint_index >= active_route_->plan.waypoint_supernode_ids.size())
    {
      route_checkpoint_authorizations_pending_.erase(waypoint_index);
      return;
    }
    route_checkpoint_authorizations_pending_.erase(waypoint_index);
    rfid_confirmed_waypoints_.insert(waypoint_index);
    return;
  }
  if (!rfid_active_.has_value() || rfid_active_route_id_ != route_id ||
    rfid_active_waypoint_index_ != waypoint_index)
  {
    return;
  }
  rfid_authorization_pending_ = false;
  finish_rfid_workflow();
}

void MissionManager::authorize_route_checkpoint(std::size_t waypoint_index)
{
  if (!active_route_.has_value() || waypoint_index >=
    active_route_->plan.waypoint_supernode_ids.size() ||
    active_route_->plan.waypoint_supernode_ids[waypoint_index].empty() ||
    rfid_confirmed_waypoints_.count(waypoint_index) != 0U ||
    !route_checkpoint_authorizations_pending_.insert(waypoint_index).second ||
    !rfid_checkpoint_client_->service_is_ready())
  {
    return;
  }
  auto request = std::make_shared<
    mission_control_interfaces::srv::ConfirmRfidCheckpoint::Request>();
  request->route_id = active_route_->route_id;
  request->waypoint_index = static_cast<uint32_t>(waypoint_index);
  request->supernode_id = active_route_->plan.waypoint_supernode_ids[waypoint_index];
  const uint64_t route_id = active_route_->route_id;
  rfid_checkpoint_client_->async_send_request(
    request,
    [this, route_id, waypoint_index](
      rclcpp::Client<mission_control_interfaces::srv::ConfirmRfidCheckpoint>::SharedFuture future)
    {
      on_rfid_checkpoint_authorization(route_id, waypoint_index, true, future);
    });
}

void MissionManager::finish_rfid_workflow()
{
  const std::string supernode = rfid_active_supernode_.empty() ? "none" : rfid_active_supernode_;
  if (rfid_active_.has_value() && mission_->current_stage() == "layer1" &&
    std::find(mission_->requested_tasks().begin(), mission_->requested_tasks().end(),
      std::to_string(*rfid_active_)) != mission_->requested_tasks().end())
  {
    rfid_confirmed_tasks_.insert(std::to_string(*rfid_active_));
  }
  rfid_confirmed_waypoints_.insert(rfid_active_waypoint_index_);
  request_hold("rfid", false, "RFID patrol stop completed");
  rfid_active_.reset();
  rfid_active_card_ = 0;
  rfid_active_supernode_.clear();
  rfid_active_waypoint_index_ = 0U;
  rfid_active_route_id_ = 0U;
  rfid_phase_ = "idle";
  rfid_release_at_.reset();
  rfid_correction_stable_since_.reset();
  rfid_correction_deadline_ = 0.0;
  rfid_skip_correction_ = false;
  rfid_speech_pending_ = false;
  rfid_correction_pending_ = false;
  rfid_authorization_pending_ = false;
  publish_text("rfid_completed:" + supernode);
}

}  // namespace navigation_mission_manager
