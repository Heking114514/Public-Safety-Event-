#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include "rectangle_odometry_test/geometry.hpp"

namespace rectangle_odometry_test
{

class RectangleRouteNode : public rclcpp::Node
{
public:
  RectangleRouteNode()
  : Node("rectangle_odometry_test")
  {
    declare_parameter("odom_topic", "/odometry/fused");
    declare_parameter("route_input_topic", "/waypoint_navigation/route_input");
    declare_parameter("route_ack_topic", "/waypoint_navigation/route_ack");
    declare_parameter("status_topic", "/waypoint_navigation/status");
    declare_parameter("route_frame", "map");
    declare_parameter("child_frame", "base_link");
    declare_parameter("side_length_m", 1.8);
    declare_parameter("corner_mode", "pivot");
    declare_parameter("corner_radius_m", 0.25);
    declare_parameter("corner_samples", 16);
    declare_parameter("straight_step_m", 0.05);
    declare_parameter("startup_delay_s", 1.0);
    declare_parameter("publish_retry_period_s", 0.5);
    declare_parameter("anchor_to_current_pose", true);
    declare_parameter("start_x", 0.0);
    declare_parameter("start_y", 0.0);
    declare_parameter("start_yaw", 0.0);

    side_length_m_ = get_parameter("side_length_m").as_double();
    corner_mode_ = get_parameter("corner_mode").as_string();
    corner_radius_m_ = get_parameter("corner_radius_m").as_double();
    corner_samples_ = get_parameter("corner_samples").as_int();
    straight_step_m_ = get_parameter("straight_step_m").as_double();
    startup_delay_s_ = get_parameter("startup_delay_s").as_double();
    publish_retry_period_s_ = get_parameter("publish_retry_period_s").as_double();
    anchor_to_current_pose_ = get_parameter("anchor_to_current_pose").as_bool();
    fixed_start_ = {
      get_parameter("start_x").as_double(), get_parameter("start_y").as_double(),
      wrap_angle(get_parameter("start_yaw").as_double())};
    if (!std::isfinite(side_length_m_) || side_length_m_ <= 0.0 ||
      (corner_mode_ != "pivot" && corner_mode_ != "arc") ||
      !std::isfinite(corner_radius_m_) || corner_radius_m_ <= 0.0 ||
      corner_radius_m_ * 2.0 >= side_length_m_ || corner_samples_ < 2 ||
      !std::isfinite(straight_step_m_) || straight_step_m_ <= 0.0 ||
      startup_delay_s_ < 0.0 || publish_retry_period_s_ <= 0.0)
    {
      throw std::invalid_argument("invalid rectangle route parameters");
    }

    route_publisher_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("route_input_topic").as_string(), rclcpp::QoS(1).reliable());
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      std::bind(&RectangleRouteNode::on_odometry, this, std::placeholders::_1));
    const auto transient = rclcpp::QoS(1).reliable().transient_local();
    ack_subscription_ = create_subscription<std_msgs::msg::UInt64>(
      get_parameter("route_ack_topic").as_string(), transient,
      std::bind(&RectangleRouteNode::on_route_ack, this, std::placeholders::_1));
    status_subscription_ = create_subscription<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), transient,
      std::bind(&RectangleRouteNode::on_status, this, std::placeholders::_1));
    started_at_ = now();
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {tick();});
    RCLCPP_WARN(
      get_logger(), "Launching this node will request a real vehicle motion test: %.2f m clockwise rectangle (corner_mode=%s)",
      side_length_m_, corner_mode_.c_str());
  }

private:
  static double quaternion_yaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static geometry_msgs::msg::Quaternion yaw_quaternion(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
  }

  void on_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if ((!get_parameter("route_frame").as_string().empty() &&
      message->header.frame_id != get_parameter("route_frame").as_string()) ||
      (!get_parameter("child_frame").as_string().empty() &&
      message->child_frame_id != get_parameter("child_frame").as_string()) ||
      (message->header.stamp.sec == 0 && message->header.stamp.nanosec == 0))
    {
      return;
    }
    const Pose2d pose{
      message->pose.pose.position.x, message->pose.pose.position.y,
      quaternion_yaw(message->pose.pose.orientation)};
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw)) {
      return;
    }
    latest_odom_ = pose;
  }

  void on_route_ack(const std_msgs::msg::UInt64::SharedPtr message)
  {
    if (route_id_ != 0U && message->data == route_id_ && !route_accepted_) {
      route_accepted_ = true;
      RCLCPP_INFO(get_logger(), "Rectangle route accepted by waypoint navigator");
    }
  }

  void on_status(const std_msgs::msg::String::SharedPtr message)
  {
    if (message->data != last_status_) {
      last_status_ = message->data;
      RCLCPP_INFO(get_logger(), "Navigator status: %s", message->data.c_str());
    }
    if (message->data == "GOAL_REACHED" && !completed_reported_) {
      completed_reported_ = true;
      report_closure_error();
    }
  }

  void tick()
  {
    if (!route_.has_value()) {
      const double elapsed = (now() - started_at_).seconds();
      if (!latest_odom_.has_value() || elapsed < startup_delay_s_) {
        return;
      }
      origin_ = anchor_to_current_pose_ ? *latest_odom_ : fixed_start_;
      route_ = make_route(*origin_);
      RCLCPP_INFO(
        get_logger(), "Captured rectangle origin: x=%.3f y=%.3f yaw=%.1f deg",
        origin_->x, origin_->y, origin_->yaw * 180.0 / 3.14159265358979323846);
    }
    if (route_accepted_) {
      return;
    }
    const rclcpp::Time timestamp = now();
    const double seconds = timestamp.seconds();
    if (route_id_ == 0U) {
      route_->header.stamp = timestamp;
      route_id_ = static_cast<uint64_t>(timestamp.nanoseconds());
      for (auto & pose : route_->poses) {
        pose.header = route_->header;
      }
    }
    if (seconds - last_publish_at_ >= publish_retry_period_s_) {
      route_publisher_->publish(*route_);
      last_publish_at_ = seconds;
      RCLCPP_INFO(
        get_logger(), "Published clockwise rectangle route with %zu targets",
        route_->poses.size());
    }
  }

  nav_msgs::msg::Path make_route(const Pose2d & origin) const
  {
    const auto waypoints = corner_mode_ == "arc" ?
      build_clockwise_rounded_rectangle(
      origin, side_length_m_, corner_radius_m_, corner_samples_, straight_step_m_) :
      build_clockwise_rectangle(origin, side_length_m_);
    nav_msgs::msg::Path path;
    path.header.frame_id = get_parameter("route_frame").as_string();
    for (const auto & waypoint : waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = waypoint.pose.x;
      pose.pose.position.y = waypoint.pose.y;
      pose.pose.position.z = waypoint.turn_junction ? 1.0e-3 : 0.0;
      pose.pose.orientation = yaw_quaternion(waypoint.pose.yaw);
      path.poses.push_back(pose);
    }
    return path;
  }

  void report_closure_error() const
  {
    if (!origin_.has_value() || !latest_odom_.has_value()) {
      RCLCPP_WARN(get_logger(), "Rectangle completed without a final odometry sample");
      return;
    }
    const double dx = latest_odom_->x - origin_->x;
    const double dy = latest_odom_->y - origin_->y;
    const double dyaw = wrap_angle(latest_odom_->yaw - origin_->yaw);
    RCLCPP_INFO(
      get_logger(), "Rectangle closure: dx=%.3f m dy=%.3f m position_error=%.3f m dyaw=%.2f deg",
      dx, dy, std::hypot(dx, dy), dyaw * 180.0 / 3.14159265358979323846);
  }

  double side_length_m_{1.8};
  std::string corner_mode_;
  double corner_radius_m_{0.25};
  int corner_samples_{16};
  double straight_step_m_{0.05};
  double startup_delay_s_{1.0};
  double publish_retry_period_s_{0.5};
  bool anchor_to_current_pose_{true};
  Pose2d fixed_start_;
  std::optional<Pose2d> latest_odom_;
  std::optional<Pose2d> origin_;
  std::optional<nav_msgs::msg::Path> route_;
  uint64_t route_id_{0U};
  bool route_accepted_{false};
  bool completed_reported_{false};
  std::string last_status_;
  double last_publish_at_{-std::numeric_limits<double>::infinity()};
  rclcpp::Time started_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr ack_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rectangle_odometry_test

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rectangle_odometry_test::RectangleRouteNode>());
  rclcpp::shutdown();
  return 0;
}
