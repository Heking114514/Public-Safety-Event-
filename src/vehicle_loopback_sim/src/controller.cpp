#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

#include "vehicle_loopback_sim/route_control.hpp"

namespace vehicle_loopback_sim
{

class LoopbackRouteController : public rclcpp::Node
{
public:
  LoopbackRouteController()
  : Node("loopback_route_controller")
  {
    declare_parameters();
    const auto durable = rclcpp::QoS(1).reliable().transient_local();
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        const auto & pose = message->pose.pose;
        pose_ = RoutePose{
          pose.position.x, pose.position.y,
          std::atan2(
            2.0 * (pose.orientation.w * pose.orientation.z +
            pose.orientation.x * pose.orientation.y),
            1.0 - 2.0 * (pose.orientation.y * pose.orientation.y +
            pose.orientation.z * pose.orientation.z)),
          false};
      });
    route_subscription_ = create_subscription<nav_msgs::msg::Path>(
      get_parameter("route_input_topic").as_string(), 10,
      [this](const nav_msgs::msg::Path::SharedPtr message) {handle_route(*message);});
    cmd_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(), 10);
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), durable);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), durable);
    waypoint_publisher_ = create_publisher<std_msgs::msg::Int32>(
      get_parameter("current_waypoint_topic").as_string(), durable);
    const double rate = std::max(1.0, parameter("control_rate_hz"));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {control();});
    set_status("LOOPBACK_WAITING_FOR_ROUTE");
    RCLCPP_INFO(get_logger(), "loopback route controller C++ ready");
  }

private:
  void declare_parameters()
  {
    declare_parameter("route_input_topic", "/waypoint_navigation/route_input");
    declare_parameter("odom_topic", "/odometry/fused");
    declare_parameter("cmd_vel_topic", "/cmd_vel_nav");
    declare_parameter("path_topic", "/waypoint_path");
    declare_parameter("status_topic", "/waypoint_navigation/status");
    declare_parameter("current_waypoint_topic", "/waypoint_navigation/current_waypoint");
    declare_parameter("control_rate_hz", 30.0);
    declare_parameter("cruise_speed_mps", 0.12);
    declare_parameter("minimum_drive_speed_mps", 0.035);
    declare_parameter("slowdown_distance_m", 0.10);
    declare_parameter("stop_tolerance_m", 0.018);
    declare_parameter("pass_tolerance_m", 0.012);
    declare_parameter("lateral_tolerance_m", 0.050);
    declare_parameter("turn_heading_threshold_rad", 0.35);
    declare_parameter("turn_yaw_tolerance_rad", 0.055);
    declare_parameter("heading_gain", 1.7);
    declare_parameter("cross_track_gain", 1.8);
    declare_parameter("stanley_softening_speed_mps", 0.12);
    declare_parameter("turn_gain", 1.8);
    declare_parameter("min_turn_speed_rps", 0.24);
    declare_parameter("max_turn_speed_rps", 0.70);
  }

  double parameter(const std::string & name) const
  {
    return get_parameter(name).as_double();
  }

  void handle_route(const nav_msgs::msg::Path & message)
  {
    auto route = parse_path(message);
    if (route.size() < 2U) {
      active_ = false;
      route_.clear();
      publish_stop();
      set_status("LOOPBACK_ROUTE_REJECTED");
      RCLCPP_WARN(get_logger(), "ignoring route with fewer than two valid poses");
      return;
    }
    route_ = std::move(route);
    route_message_ = message;
    index_ = 0U;
    active_ = true;
    phase_ = "FOLLOW";
    path_publisher_->publish(route_message_);
    publish_waypoint();
    set_status("LOOPBACK_ROUTE_LOADED");
  }

  void control()
  {
    if (!pose_.has_value()) {
      publish_stop();
      set_status("LOOPBACK_WAITING_FOR_ODOM");
      return;
    }
    if (!active_ || route_.empty()) {
      publish_stop();
      set_status("LOOPBACK_WAITING_FOR_ROUTE");
      return;
    }
    skip_satisfied_start();
    if (index_ >= route_.size()) {
      complete();
    } else if (phase_ == "ALIGN") {
      run_align();
    } else {
      run_follow();
    }
  }

  void run_follow()
  {
    const RoutePose & target = route_[index_];
    const Projection projection = project(segment_start(), target, *pose_);
    const bool final = index_ + 1U >= route_.size();
    const bool stop_required = final || heading_change_requires_stop(projection.heading);
    const bool lateral_ok =
      std::abs(projection.cross_track) <= parameter("lateral_tolerance_m");
    if (target_reached(projection, stop_required, lateral_ok)) {
      if (final) {
        complete();
      } else if (stop_required) {
        phase_ = "ALIGN";
        publish_stop();
        set_status("LOOPBACK_ALIGNING");
      } else {
        advance();
      }
      return;
    }
    const double heading_error = route_angle(projection.heading - pose_->yaw);
    const double cruise = parameter("cruise_speed_mps");
    double speed = std::min(cruise, std::max(0.0, route_distance(*pose_, target) * 1.2));
    if (stop_required) {
      const double slowdown = std::max(1.0e-6, parameter("slowdown_distance_m"));
      speed = std::min(
        speed, cruise * std::min(1.0, std::max(0.0, projection.remaining) / slowdown));
    }
    speed = std::clamp(speed, 0.0, cruise);
    if (speed > 0.0) {
      speed = std::max(parameter("minimum_drive_speed_mps"), speed);
    }
    const double correction = std::atan2(
      parameter("cross_track_gain") * projection.cross_track,
      std::abs(speed) + parameter("stanley_softening_speed_mps"));
    geometry_msgs::msg::Twist command;
    command.linear.x = speed * std::max(0.25, std::cos(heading_error));
    command.angular.z = std::clamp(
      parameter("heading_gain") * route_angle(heading_error - correction),
      -parameter("max_turn_speed_rps"), parameter("max_turn_speed_rps"));
    cmd_publisher_->publish(command);
    set_status(stop_required ? "LOOPBACK_APPROACHING_TURN" : "LOOPBACK_FOLLOWING");
  }

  void run_align()
  {
    const auto desired = next_heading();
    if (!desired.has_value()) {
      complete();
      return;
    }
    const double error = route_angle(*desired - pose_->yaw);
    if (std::abs(error) <= parameter("turn_yaw_tolerance_rad")) {
      publish_stop();
      advance();
      phase_ = "FOLLOW";
      set_status("LOOPBACK_PATH_ALIGNED");
      return;
    }
    double angular = std::clamp(
      parameter("turn_gain") * error,
      -parameter("max_turn_speed_rps"), parameter("max_turn_speed_rps"));
    if (std::abs(angular) < parameter("min_turn_speed_rps")) {
      angular = std::copysign(parameter("min_turn_speed_rps"), error);
    }
    geometry_msgs::msg::Twist command;
    command.angular.z = angular;
    cmd_publisher_->publish(command);
    set_status("LOOPBACK_ALIGNING");
  }

  bool target_reached(
    const Projection & projection, bool stop_required, bool lateral_ok) const
  {
    if (!projection.valid) {
      return route_distance(*pose_, route_[index_]) <= parameter("stop_tolerance_m");
    }
    return projection.remaining <= parameter(
      stop_required ? "stop_tolerance_m" : "pass_tolerance_m") && lateral_ok;
  }

  bool heading_change_requires_stop(double heading) const
  {
    const auto desired = next_heading();
    return !desired.has_value() ||
      std::abs(route_angle(*desired - heading)) >= parameter("turn_heading_threshold_rad");
  }

  std::optional<double> next_heading() const
  {
    if (index_ + 1U >= route_.size()) {
      return std::nullopt;
    }
    const RoutePose & current = route_[index_];
    const RoutePose & next = route_[index_ + 1U];
    return route_distance(current, next) <= 1.0e-6 ? next.yaw :
      std::optional<double>(std::atan2(next.y - current.y, next.x - current.x));
  }

  RoutePose segment_start() const
  {
    return index_ == 0U ? *pose_ : route_[index_ - 1U];
  }

  void skip_satisfied_start()
  {
    while (index_ < route_.size() &&
      route_distance(*pose_, route_[index_]) <= parameter("stop_tolerance_m"))
    {
      advance();
    }
  }

  void advance()
  {
    ++index_;
    publish_waypoint();
  }

  void complete()
  {
    active_ = false;
    publish_stop();
    set_status("LOOPBACK_GOAL_REACHED");
  }

  void publish_stop()
  {
    cmd_publisher_->publish(geometry_msgs::msg::Twist());
  }

  void publish_waypoint()
  {
    std_msgs::msg::Int32 message;
    message.data = static_cast<int32_t>(index_);
    waypoint_publisher_->publish(message);
  }

  void set_status(const std::string & status)
  {
    if (status == last_status_) {
      return;
    }
    last_status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_publisher_->publish(message);
  }

  std::optional<RoutePose> pose_;
  std::vector<RoutePose> route_;
  nav_msgs::msg::Path route_message_;
  std::size_t index_{0U};
  bool active_{false};
  std::string phase_{"WAITING_FOR_ROUTE"};
  std::string last_status_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr route_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr waypoint_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace vehicle_loopback_sim

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vehicle_loopback_sim::LoopbackRouteController>());
  rclcpp::shutdown();
  return 0;
}
