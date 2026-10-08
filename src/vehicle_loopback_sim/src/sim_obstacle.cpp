#include "vehicle_loopback_sim/simulator.hpp"

#include <cmath>

namespace vehicle_loopback_sim
{

void VehicleLoopbackSim::rebuild_obstacle_detector()
{
  if (!publish_obstacle_detection_ || manual_obstacles_.empty()) {
    obstacle_detector_.reset();
    return;
  }
  obstacle_detector_ = std::make_unique<SimulatedObstacleDetector>(
    manual_obstacles_, obstacle_config(obstacle_min_range_m_));
  RCLCPP_INFO(
    get_logger(), "simulated obstacle recognition ready: source=manual obstacles=%zu",
    manual_obstacles_.size());
}

void VehicleLoopbackSim::reset_obstacle_reporting()
{
  obstacle_gate_->reset();
  last_obstacle_index_.reset();
}

void VehicleLoopbackSim::on_manual_obstacle(
  const geometry_msgs::msg::PolygonStamped::SharedPtr message)
{
  if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
    RCLCPP_WARN(
      get_logger(), "ignoring manual obstacle in frame %s; expected %s",
      message->header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  Polygon2d obstacle;
  for (const auto & point : message->polygon.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      RCLCPP_WARN(get_logger(), "ignoring invalid manual obstacle polygon");
      return;
    }
    obstacle.emplace_back(point.x, point.y);
  }
  if (obstacle.size() < 3U) {
    RCLCPP_WARN(get_logger(), "ignoring invalid manual obstacle polygon");
    return;
  }
  manual_obstacles_.push_back(std::move(obstacle));
  rebuild_obstacle_detector();
  reset_obstacle_reporting();
  RCLCPP_INFO(
    get_logger(), "manual simulated obstacle added: count=%zu", manual_obstacles_.size());
}

void VehicleLoopbackSim::on_clear_obstacles(const std_msgs::msg::Empty::SharedPtr message)
{
  (void)message;
  if (manual_obstacles_.empty()) {
    return;
  }
  manual_obstacles_.clear();
  rebuild_obstacle_detector();
  reset_obstacle_reporting();
  RCLCPP_INFO(get_logger(), "manual simulated obstacles cleared");
}

void VehicleLoopbackSim::publish_obstacle_report(const rclcpp::Time & stamp)
{
  if (!publish_obstacle_detection_) {
    return;
  }
  const std::optional<ObstacleDetection> raw = obstacle_detector_ ?
    obstacle_detector_->detect(pose_) : std::nullopt;
  const auto detection = obstacle_gate_->update(raw, stamp.seconds());
  if (detection.has_value()) {
    sensor_msgs::msg::Range range;
    range.header.stamp = stamp;
    range.header.frame_id = obstacle_frame_;
    range.radiation_type = sensor_msgs::msg::Range::INFRARED;
    range.field_of_view = static_cast<float>(obstacle_fov_rad_);
    range.min_range = static_cast<float>(obstacle_min_range_m_);
    range.max_range = static_cast<float>(detection->trigger_range_m);
    range.range = static_cast<float>(detection->range_m);
    front_range_publisher_->publish(range);
    publish_dynamic_obstacle(*detection, stamp);
  }
  std_msgs::msg::Bool blocked;
  blocked.data = detection.has_value();
  front_obstacle_publisher_->publish(blocked);

  const std::optional<std::size_t> current = detection.has_value() ?
    std::optional<std::size_t>(detection->obstacle_index) : std::nullopt;
  if (current == last_obstacle_index_) {
    return;
  }
  last_obstacle_index_ = current;
  if (detection.has_value()) {
    RCLCPP_WARN(
      get_logger(), "simulated front obstacle detected: index=%zu range=%.2fm trigger=%.2fm",
      detection->obstacle_index, detection->range_m, detection->trigger_range_m);
  } else {
    RCLCPP_INFO(get_logger(), "simulated front obstacle clear");
  }
}

void VehicleLoopbackSim::publish_dynamic_obstacle(
  const ObstacleDetection & detection, const rclcpp::Time & stamp)
{
  if (detection.obstacle_index >= manual_obstacles_.size()) {
    return;
  }
  geometry_msgs::msg::PolygonStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = map_frame_;
  for (const auto & point : manual_obstacles_[detection.obstacle_index]) {
    geometry_msgs::msg::Point32 converted;
    converted.x = static_cast<float>(point.first);
    converted.y = static_cast<float>(point.second);
    message.polygon.points.push_back(converted);
  }
  dynamic_obstacle_publisher_->publish(message);
}

}  // namespace vehicle_loopback_sim
