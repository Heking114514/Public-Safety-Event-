#include "visual_navigation/waypoint_navigator.hpp"

bool WaypointNavigator::LoadRoute(const std::string &routeFile) {
  std::vector<Waypoint> loadedWaypoints;
  std::string error;
  if (!visual_navigation::RouteManager::LoadCsv(routeFile, defaultSpeed_,
                                                waypointTolerance_,
                                                loadedWaypoints, error)) {
    if (routeFile.empty())
      RCLCPP_INFO(get_logger(), "%s; waiting for a dynamic route",
                  error.c_str());
    else
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    return false;
  }

  waypoints_ = std::move(loadedWaypoints);
  RCLCPP_INFO(get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(),
              routeFile.c_str());
  return true;
}

void WaypointNavigator::PublishRoutePath() {
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = routeFrame_;
  path.poses.reserve(waypoints_.size());

  for (std::size_t index = 0; index < waypoints_.size(); ++index) {
    const Waypoint &waypoint = waypoints_[index];
    double yaw = waypoint.yaw;
    if (!std::isfinite(yaw)) {
      if (index + 1 < waypoints_.size()) {
        yaw = std::atan2(waypoints_[index + 1].y - waypoint.y,
                         waypoints_[index + 1].x - waypoint.x);
      } else if (index > 0) {
        yaw = std::atan2(waypoint.y - waypoints_[index - 1].y,
                         waypoint.x - waypoints_[index - 1].x);
      } else {
        yaw = 0.0;
      }
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    pose.pose.orientation = QuaternionFromYaw(yaw);
    path.poses.push_back(pose);
  }

  pathPublisher_->publish(path);
}

void WaypointNavigator::PublishRouteAck(uint64_t route_id) {
  std_msgs::msg::UInt64 message;
  message.data = route_id;
  routeAckPublisher_->publish(message);
}

void WaypointNavigator::HandleRouteInput(
    const nav_msgs::msg::Path::SharedPtr message) {
  if (message->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Ignoring empty dynamic waypoint route");
    return;
  }

  if (!message->header.frame_id.empty() &&
      message->header.frame_id != routeFrame_) {
    RCLCPP_ERROR(get_logger(),
                 "Ignoring dynamic route in frame '%s'; expected '%s'",
                 message->header.frame_id.c_str(), routeFrame_.c_str());
    return;
  }
  if (message->header.stamp.sec < 0 ||
      message->header.stamp.nanosec >= 1000000000U) {
    RCLCPP_ERROR(get_logger(),
                 "Ignoring dynamic route with an invalid timestamp");
    return;
  }
  const uint64_t route_id =
      static_cast<uint64_t>(rclcpp::Time(message->header.stamp).nanoseconds());
  if (route_id == 0U) {
    RCLCPP_ERROR(get_logger(),
                 "Ignoring dynamic route without a non-zero route id");
    return;
  }

  std::vector<Waypoint> loadedWaypoints;
  std::size_t invalidWaypointIndex = 0;
  std::string routeError;
  if (!visual_navigation::RouteManager::LoadPath(
          *message, defaultSpeed_, waypointTolerance_, loadedWaypoints,
          invalidWaypointIndex, routeError)) {
    RCLCPP_ERROR(get_logger(), "Ignoring dynamic route: waypoint %zu %s",
                 invalidWaypointIndex, routeError.c_str());
    return;
  }

  if (routeLoaded_ && visual_navigation::RouteManager::Equivalent(
                          waypoints_, loadedWaypoints)) {
    // A repeated publish is also the ACK-loss recovery path. Do not reset a
    // route that is still running; re-ACK it and publish the current state so
    // the frontend can arm completion even when the original status was lost.
    if (navigationActive_) {
      PublishRoutePath();
      PublishRouteAck(route_id);
      PublishState();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Re-acknowledging a duplicate route without resetting active progress");
      return;
    }

    std::string activation_result;
    if (!ActivateNavigation(activation_result)) {
      PublishRoutePath();
      PublishState();
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Duplicate route is loaded but not acknowledged as active: %s",
          activation_result.c_str());
      return;
    }
    PublishRoutePath();
    PublishCurrentWaypoint();
    PublishRouteAck(route_id);
    PublishState();
    RCLCPP_INFO(get_logger(), "Reactivated an accepted duplicate route");
    return;
  }

  // Stop the old route before atomically replacing it with the new one.
  navigationActive_ = false;
  currentWaypointIndex_ = 0;
  navigationSupervisor_.ResetLocalizationHistory();
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  pathSegmentInitialized_ = false;
  resumePathFromCurrentPose_ = false;
  ResetRunControl();
  PublishStop();

  waypoints_ = std::move(loadedWaypoints);
  routeLoaded_ = true;
  PublishRoutePath();
  PublishCurrentWaypoint();
  std::string activation_result;
  if (!ActivateNavigation(activation_result)) {
    PublishState();
    RCLCPP_WARN(get_logger(), "Loaded dynamic route but activation was rejected: %s",
                activation_result.c_str());
    return;
  }
  PublishRouteAck(route_id);
  PublishState();
  RCLCPP_INFO(get_logger(), "Loaded and activated %zu dynamic waypoints",
              waypoints_.size());
}
