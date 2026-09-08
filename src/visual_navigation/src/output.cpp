#include "visual_navigation/waypoint_navigator.hpp"

void WaypointNavigator::PublishStop() {
  geometry_msgs::msg::Twist command;
  cmdVelPublisher_->publish(command);
  lastMotionCommand_ = command;
  lastMotionCommandTime_ = std::chrono::steady_clock::now();
  motionCommandInitialized_ = true;
}

void WaypointNavigator::PublishMotionCommand(
    const geometry_msgs::msg::Twist &desired) {
  const auto currentTime = std::chrono::steady_clock::now();
  double dt = 1.0 / controlFrequency_;
  if (motionCommandInitialized_) {
    const double measuredDt =
        std::chrono::duration<double>(currentTime - lastMotionCommandTime_)
            .count();
    if (measuredDt > 0.0 && measuredDt <= 0.2)
      dt = measuredDt;
  }

  geometry_msgs::msg::Twist limited = desired;
  limited.linear.x = visual_navigation::LimitRate(
      desired.linear.x, lastMotionCommand_.linear.x, maxLinearAcceleration_,
      maxLinearDeceleration_, dt);
  limited.angular.z = visual_navigation::LimitRate(
      desired.angular.z, lastMotionCommand_.angular.z, maxAngularAcceleration_,
      maxAngularDeceleration_, dt);
  cmdVelPublisher_->publish(limited);
  lastMotionCommand_ = limited;
  lastMotionCommandTime_ = currentTime;
  motionCommandInitialized_ = true;
}

void WaypointNavigator::PublishCurrentWaypoint() {
  std_msgs::msg::Int32 message;
  message.data = static_cast<int32_t>(currentWaypointIndex_);
  currentWaypointPublisher_->publish(message);
}

bool WaypointNavigator::MotionHeld() const { return !motionHolds_.empty(); }

std::vector<std::string> WaypointNavigator::ActiveHoldSources() const {
  std::vector<std::string> sources;
  sources.reserve(motionHolds_.size());
  for (const auto &entry : motionHolds_)
    sources.push_back(entry.first);
  return sources;
}

void WaypointNavigator::PublishMotionHoldState() {
  mission_control_interfaces::msg::MotionHoldState message;
  message.held = MotionHeld();
  message.active_sources.reserve(motionHolds_.size());
  message.reasons.reserve(motionHolds_.size());
  for (const auto &entry : motionHolds_) {
    message.active_sources.push_back(entry.first);
    message.reasons.push_back(entry.second);
  }
  motionHoldStatePublisher_->publish(message);
}

void WaypointNavigator::PublishState() {
  std_msgs::msg::String message;
  message.data = state_;
  statusPublisher_->publish(message);
}

void WaypointNavigator::SetState(const std::string &state) {
  if (state == state_)
    return;
  state_ = state;
  PublishState();
  RCLCPP_INFO(get_logger(), "Navigation state: %s", state.c_str());
}
