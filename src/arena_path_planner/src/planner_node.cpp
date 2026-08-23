#include "arena_path_planner/planner.hpp"

#include "arena_path_planner/srv/plan_arena_path.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace arena_path_planner
{
namespace
{

double YawFromQuaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  const double sin_yaw = 2.0 *
    (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  const double cos_yaw = 1.0 - 2.0 *
    (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  return std::atan2(sin_yaw, cos_yaw);
}

geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

bool RouteHasUnsafeInPlaceTurn(const ArenaPlanner & planner, const PlanResult & result)
{
  constexpr double kNavigatorRotationThreshold = 0.18;
  if (result.points.size() < 3 || result.headings.size() != result.points.size()) {
    return false;
  }
  for (std::size_t index = 0; index + 2 < result.points.size(); ++index) {
    const double heading_change = std::abs(NormalizeAngle(
      result.headings[index + 1] - result.headings[index]));
    if (heading_change < kNavigatorRotationThreshold) {
      continue;
    }
    // The navigator brakes at waypoint index + 1 and rotates at that centre.
    if (!planner.RotationIsFree(
        result.points[index + 1], result.headings[index], result.headings[index + 1]))
    {
      return true;
    }
  }
  return false;
}

}  // namespace

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("arena_path_planner")
  {
    const std::string config_file = declare_parameter<std::string>("config_file", "");
    if (config_file.empty()) {
      throw std::runtime_error("config_file parameter is required");
    }
    planner_ = std::make_unique<ArenaPlanner>(ArenaPlanner::LoadConfig(config_file));
    const PlannerTopics & topics = planner_->config().topics;

    const auto durable_qos = rclcpp::QoS(1).reliable().transient_local();
    arena_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      topics.arena_path, durable_qos);
    navigation_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      topics.navigation_path, durable_qos);
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      topics.occupancy_grid, durable_qos);
    route_publisher_ = create_publisher<nav_msgs::msg::Path>(
      topics.route_input, rclcpp::QoS(1).reliable());
    service_ = create_service<srv::PlanArenaPath>(
      topics.service,
      std::bind(&PlannerNode::HandlePlan, this, std::placeholders::_1, std::placeholders::_2));

    PublishMap();
    RCLCPP_INFO(
      get_logger(),
      "arena planner ready: %.2f x %.2f m, %zu default targets, service %s",
      planner_->config().width, planner_->config().height,
      planner_->config().default_targets.size(), topics.service.c_str());
  }

private:
  nav_msgs::msg::Path MakeArenaPath(
    const PlanResult & result, const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "arena";
    path.header.stamp = stamp;
    for (std::size_t index = 0; index < result.points.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = result.points[index].x;
      pose.pose.position.y = result.points[index].y;
      pose.pose.orientation = QuaternionFromYaw(result.headings[index]);
      path.poses.push_back(pose);
    }
    return path;
  }

  nav_msgs::msg::Path MakeNavigationPath(
    const PlanResult & result, const rclcpp::Time & stamp) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = stamp;
    // The fused map frame starts at the configured arena start pose. Always
    // use that fixed transform so a route replanned after the vehicle moves
    // remains in the same global map frame.
    const Pose & map_origin = planner_->config().default_start;
    const double cosine = std::cos(map_origin.yaw);
    const double sine = std::sin(map_origin.yaw);
    for (std::size_t index = 0; index < result.points.size(); ++index) {
      const double delta_x = result.points[index].x - map_origin.position.x;
      const double delta_y = result.points[index].y - map_origin.position.y;
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = cosine * delta_x + sine * delta_y;
      pose.pose.position.y = -sine * delta_x + cosine * delta_y;
      pose.pose.orientation = QuaternionFromYaw(
        NormalizeAngle(result.headings[index] - map_origin.yaw));
      path.poses.push_back(pose);
    }
    return path;
  }

  void HandlePlan(
    const std::shared_ptr<srv::PlanArenaPath::Request> request,
    std::shared_ptr<srv::PlanArenaPath::Response> response)
  {
    const auto begin = std::chrono::steady_clock::now();
    Pose start;
    start.position = {request->start.position.x, request->start.position.y};
    start.yaw = YawFromQuaternion(request->start.orientation);
    std::vector<Point> targets;
    targets.reserve(request->targets.size());
    for (const auto & target : request->targets) {
      targets.push_back({target.x, target.y});
    }
    std::vector<std::string> labels = request->labels;
    if (targets.empty()) {
      targets = planner_->config().default_targets;
      labels = planner_->config().default_labels;
    }
    if (labels.empty() && !targets.empty()) {
      for (std::size_t index = 0; index < targets.size(); ++index) {
        labels.push_back(std::to_string(index + 1));
      }
    }
    const std::string mode = request->mode.empty() ? planner_->config().default_mode : request->mode;
    PlannerConfig request_config = planner_->config();
    for (const auto & polygon : request->dynamic_obstacles) {
      if (polygon.points.empty()) {
        continue;
      }
      Rectangle rectangle{
        polygon.points.front().x, polygon.points.front().y,
        polygon.points.front().x, polygon.points.front().y};
      for (const auto & point : polygon.points) {
        rectangle.minimum_x = std::min(rectangle.minimum_x, static_cast<double>(point.x));
        rectangle.minimum_y = std::min(rectangle.minimum_y, static_cast<double>(point.y));
        rectangle.maximum_x = std::max(rectangle.maximum_x, static_cast<double>(point.x));
        rectangle.maximum_y = std::max(rectangle.maximum_y, static_cast<double>(point.y));
      }
      if (rectangle.maximum_x > rectangle.minimum_x &&
        rectangle.maximum_y > rectangle.minimum_y)
      {
        request_config.obstacles.push_back(rectangle);
      }
    }
    // The static-map case is the normal frontend path. Reusing the planner
    // avoids rebuilding the 5 cm occupancy and clearance grids on every
    // mouse drag or layer request. Dynamic obstacles still get an isolated
    // planner instance below, so their map cannot leak into later requests.
    PlanResult result;
    const ArenaPlanner * active_planner = planner_.get();
    std::unique_ptr<ArenaPlanner> dynamic_planner;
    if (request->dynamic_obstacles.empty()) {
      result = planner_->Plan(start, targets, labels, mode, request->covered_edges);
    } else {
      dynamic_planner = std::make_unique<ArenaPlanner>(std::move(request_config));
      active_planner = dynamic_planner.get();
      result = active_planner->Plan(
        start, targets, labels, mode, request->covered_edges);
    }
    if (result.success && !active_planner->config().allow_in_place_turns &&
      RouteHasUnsafeInPlaceTurn(*active_planner, result)) {
      result.success = false;
      result.message =
        "route requires an in-place turn without clearance for the complete chassis";
    }
    response->success = result.success;
    response->message = result.message;
    response->visit_order = result.visit_order;
    response->deferred_targets = result.deferred_targets;
    response->covered_edges = result.covered_edges;
    response->all_targets_reached = result.all_targets_reached;
    response->length = result.length;
    response->planning_time_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - begin).count();
    if (!result.success) {
      RCLCPP_WARN(get_logger(), "planning failed: %s", result.message.c_str());
      return;
    }

    const rclcpp::Time stamp = now();
    response->arena_path = MakeArenaPath(result, stamp);
    response->navigation_path = MakeNavigationPath(result, stamp);
    arena_path_publisher_->publish(response->arena_path);
    navigation_path_publisher_->publish(response->navigation_path);
    // A route with deferred work is a useful preview, but it is not a
    // complete mission. Never hand it to the navigator under an activation
    // request: the caller must explicitly replan the deferred work first.
    const bool activation_allowed = ActivationAllowed(request->activate_navigation, result);
    if (request->activate_navigation && !activation_allowed) {
      response->message += "; activation refused while targets remain deferred";
      RCLCPP_WARN(
        get_logger(), "route preview is partial (%zu deferred); activation refused",
        response->deferred_targets.size());
    }
    if (activation_allowed) {
      route_publisher_->publish(response->navigation_path);
      RCLCPP_INFO(
        get_logger(), "published %.2f m route with %zu poses to navigator",
        result.length, result.points.size());
    }
  }

  void PublishMap()
  {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "arena";
    map.header.stamp = now();
    map.info.resolution = static_cast<float>(planner_->config().resolution);
    map.info.width = static_cast<uint32_t>(
      std::ceil(planner_->config().width / planner_->config().resolution));
    map.info.height = static_cast<uint32_t>(
      std::ceil(planner_->config().height / planner_->config().resolution));
    map.info.origin.orientation.w = 1.0;
    map.data = planner_->OccupancyData();
    map_publisher_->publish(map);
  }

  std::unique_ptr<ArenaPlanner> planner_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr arena_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr navigation_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr route_publisher_;
  rclcpp::Service<srv::PlanArenaPath>::SharedPtr service_;
};

}  // namespace arena_path_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<arena_path_planner::PlannerNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("arena_path_planner"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
