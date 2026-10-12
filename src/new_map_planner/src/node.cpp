#include "new_map_planner/config.hpp"
#include "new_map_planner/geometry.hpp"
#include "new_map_planner/planner.hpp"
#include "new_map_planner/transform.hpp"

#include "new_map_planner/msg/maneuver.hpp"
#include "new_map_planner/msg/route_plan.hpp"
#include "new_map_planner/srv/plan_route.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace new_map_planner {
namespace {

geometry_msgs::msg::Quaternion QuaternionFromYaw(double yaw) {
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
  return quaternion;
}

double QuaternionYaw(const geometry_msgs::msg::Quaternion &quaternion) {
  const double norm = quaternion.x * quaternion.x +
                      quaternion.y * quaternion.y +
                      quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  if (!std::isfinite(norm) || norm <= 1.0e-12) {
    throw std::invalid_argument("start orientation is not a finite quaternion");
  }
  return std::atan2(
      2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y) / norm,
      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z) /
                norm);
}

Pose FromRosPose(const geometry_msgs::msg::Pose &pose) {
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y)) {
    throw std::invalid_argument("pose coordinates must be finite");
  }
  return {{pose.position.x, pose.position.y}, QuaternionYaw(pose.orientation)};
}

StartMode StartModeFromMessage(uint8_t mode) {
  switch (mode) {
    case srv::PlanRoute::Request::START_MODE_NORMAL:
      return StartMode::NORMAL;
    case srv::PlanRoute::Request::START_MODE_TURN_RECOVERY:
      return StartMode::TURN_RECOVERY;
    case srv::PlanRoute::Request::START_MODE_OBSTACLE_RECOVERY:
      return StartMode::OBSTACLE_RECOVERY;
    default:
      throw std::invalid_argument("unknown planning start mode");
  }
}

geometry_msgs::msg::Pose ToRosPose(const Pose &pose) {
  geometry_msgs::msg::Pose message;
  message.position.x = pose.position.x;
  message.position.y = pose.position.y;
  message.orientation = QuaternionFromYaw(pose.yaw);
  return message;
}

} // namespace

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode() : Node("new_map_planner") {
    const std::string package_share =
        ament_index_cpp::get_package_share_directory("new_map_planner");
    const std::string vehicle_share =
        ament_index_cpp::get_package_share_directory("vehicle_config");
    const std::string map_file = declare_parameter<std::string>(
        "map_file", package_share + "/config/arena.yaml");
    const std::string vehicle_file = declare_parameter<std::string>(
        "vehicle_file", vehicle_share + "/config/vehicle_physical.yaml");
    planner_ = std::make_unique<MapPlanner>(LoadConfig(map_file, vehicle_file));

    const PlannerTopics &topics = planner_->config().topics;
    const auto durable_qos = rclcpp::QoS(1).reliable().transient_local();
    plan_publisher_ =
        create_publisher<msg::RoutePlan>(topics.plan, durable_qos);
    arena_path_publisher_ =
        create_publisher<nav_msgs::msg::Path>(topics.arena_path, durable_qos);
    navigation_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
        topics.navigation_path, durable_qos);
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        topics.occupancy_grid, durable_qos);
    service_ = create_service<srv::PlanRoute>(
        topics.service,
        std::bind(&PlannerNode::HandlePlan, this, std::placeholders::_1,
                  std::placeholders::_2));
    PublishMap();
    RCLCPP_INFO(get_logger(),
                "new map planner ready: map=%s nodes=%zu lanes=%zu service=%s",
                planner_->config().map_id.c_str(),
                planner_->config().nodes.size(),
                planner_->config().lanes.size(), topics.service.c_str());
  }

private:
  nav_msgs::msg::Path MakePath(const std::vector<Pose> &poses,
                               const std::string &frame,
                               const rclcpp::Time &stamp,
                               const PlanResult &result) const {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.header.stamp = stamp;
    path.poses.reserve(poses.size());
    for (std::size_t index = 0; index < poses.size(); ++index) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = path.header;
      stamped.pose = ToRosPose(poses[index]);
      if (planner_->config().publish_shunt_markers && frame == "map" &&
          index < result.turn_waypoints.size() &&
          result.turn_waypoints[index]) {
        const bool reverse_arrival =
            index < result.reverse_arrival_turn_waypoints.size() &&
            result.reverse_arrival_turn_waypoints[index];
        const bool rfid_supernode = index < result.waypoint_supernode_ids.size() &&
            !result.waypoint_supernode_ids[index].empty();
        stamped.pose.position.z = rfid_supernode ?
            (reverse_arrival ? 5.0e-3 : 4.0e-3) :
            (reverse_arrival ? 2.0e-3 : 1.0e-3);
      } else if (frame == "map" && index < result.waypoint_supernode_ids.size() &&
                 !result.waypoint_supernode_ids[index].empty()) {
        stamped.pose.position.z = 3.0e-3;
      }
      path.poses.push_back(std::move(stamped));
    }
    return path;
  }

  msg::Maneuver MakeManeuverMessage(const Maneuver &maneuver) const {
    msg::Maneuver message;
    message.type = static_cast<uint8_t>(maneuver.type);
    message.node_id = maneuver.node_id;
    message.incoming_lane_id = maneuver.incoming_lane_id;
    message.outgoing_lane_id = maneuver.outgoing_lane_id;
    message.start_pose =
        ToRosPose(ArenaToMap(maneuver.start, planner_->config().transform));
    message.end_pose =
        ToRosPose(ArenaToMap(maneuver.end, planner_->config().transform));
    message.distance_m = maneuver.distance;
    message.stop_required = maneuver.stop_required;
    message.collision_checked = maneuver.collision_checked;
    return message;
  }

  msg::RoutePlan MakePlanMessage(const PlanResult &result,
                                 const rclcpp::Time &stamp) const {
    msg::RoutePlan message;
    message.header.frame_id = "map";
    message.header.stamp = stamp;
    message.plan_id = static_cast<uint64_t>(stamp.nanoseconds());
    message.map_id = planner_->config().map_id;
    message.status = result.recovery_only ? "TURN_RECOVERY" :
      (result.complete ? "COMPLETE" : "PARTIAL");
    message.arena_path = MakePath(result.arena_path, "arena", stamp, result);
    message.navigation_path =
        MakePath(result.navigation_path, "map", stamp, result);
    message.waypoint_supernode_ids = result.waypoint_supernode_ids;
    message.maneuvers.reserve(result.maneuvers.size());
    for (const Maneuver &maneuver : result.maneuvers) {
      message.maneuvers.push_back(MakeManeuverMessage(maneuver));
    }
    message.visit_order = result.visit_order;
    message.covered_lane_ids = result.covered_lane_ids;
    message.deferred_target_ids = result.deferred_target_ids;
    message.deferred_lane_ids = result.deferred_lane_ids;
    message.length_m = result.length;
    message.complete = result.complete;
    message.collision_checked = result.collision_checked;
    message.recovery_only = result.recovery_only;
    message.ordinary_nodes_since_task =
        static_cast<uint8_t>(result.ordinary_nodes_since_task);
    return message;
  }

  void HandlePlan(const std::shared_ptr<srv::PlanRoute::Request> request,
                  std::shared_ptr<srv::PlanRoute::Response> response) {
    const auto begin = std::chrono::steady_clock::now();
    RCLCPP_INFO(
        get_logger(),
        "plan request: mode=%s targets=%zu lanes=%zu completed_lanes=%zu "
        "start_map=(%.3f, %.3f, %.1fdeg) dynamic_obstacles=%zu "
        "return_home=%s allow_partial=%s ordinary_since_task=%u",
        request->mode.empty() ? "targets" : request->mode.c_str(),
        request->target_ids.size(), request->required_lane_ids.size(),
        request->completed_lane_ids.size(), request->start.position.x,
        request->start.position.y,
        std::atan2(2.0 * request->start.orientation.w *
                       request->start.orientation.z,
                   1.0 - 2.0 * request->start.orientation.z *
                               request->start.orientation.z) *
            180.0 / 3.141592653589793,
        request->dynamic_obstacles.size(), request->return_home ? "true" : "false",
        request->allow_partial ? "true" : "false",
        static_cast<unsigned int>(request->ordinary_nodes_since_task));
    try {
      PlanRequest planner_request;
      planner_request.start_map = FromRosPose(request->start);
      planner_request.start_mode = StartModeFromMessage(request->start_mode);
      planner_request.recovery_anchor_node_id = request->recovery_anchor_node_id;
      planner_request.recovery_incoming_lane_id = request->incoming_lane_id;
      planner_request.allow_reverse_departure = request->allow_reverse_departure;
      planner_request.mode = request->mode.empty() ? "targets" : request->mode;
      planner_request.target_ids = request->target_ids;
      planner_request.required_lane_ids = request->required_lane_ids;
      planner_request.completed_lane_ids = request->completed_lane_ids;
      planner_request.blocked_lane_ids = request->blocked_lane_ids;
      planner_request.obstacle_revision = request->obstacle_revision;
      planner_request.validate_geometry = request->validate_geometry;
      planner_request.online_geometry_validation =
          request->online_geometry_validation;
      planner_request.return_home = request->return_home;
      planner_request.allow_partial = request->allow_partial;
      planner_request.ordinary_nodes_since_task =
          request->ordinary_nodes_since_task;
       planner_request.ordinary_nodes_since_task_exact =
           request->ordinary_nodes_since_task_exact;
      for (const geometry_msgs::msg::Polygon &polygon :
           request->dynamic_obstacles) {
        Polygon converted;
        for (const geometry_msgs::msg::Point32 &point : polygon.points) {
          if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            throw std::invalid_argument(
                "dynamic obstacle coordinates must be finite");
          }
          converted.points.push_back({point.x, point.y});
        }
        planner_request.dynamic_obstacles_map.push_back(std::move(converted));
      }

      const PlanResult result = planner_->plan(planner_request);
      response->success = result.success;
      response->message = result.message;
      if (result.success) {
        const rclcpp::Time stamp = now();
        response->plan = MakePlanMessage(result, stamp);
        response->plan.blocked_lane_ids_used = request->blocked_lane_ids;
        response->plan.obstacle_revision = request->obstacle_revision;
        plan_publisher_->publish(response->plan);
        arena_path_publisher_->publish(response->plan.arena_path);
        navigation_path_publisher_->publish(response->plan.navigation_path);
        response->activated = false;
      } else {
        response->plan.map_id = planner_->config().map_id;
        response->plan.status = PlanFailureName(result.failure_reason);
      }
    } catch (const std::exception &exception) {
      response->success = false;
      response->activated = false;
      response->plan.map_id = planner_->config().map_id;
      response->plan.status =
          PlanFailureName(PlanFailureReason::INVALID_REQUEST);
      response->message =
          std::string("planning request rejected: ") + exception.what();
    }
    response->planning_time_ms = std::chrono::duration<double, std::milli>(
                                     std::chrono::steady_clock::now() - begin)
                                     .count();
    if (!response->success) {
      RCLCPP_WARN(get_logger(),
                  "plan rejected: status=%s reason=%s start_map=(%.3f, %.3f, %.1fdeg) "
                  "elapsed=%.1f ms",
                  response->plan.status.c_str(), response->message.c_str(),
                  request->start.position.x, request->start.position.y,
                  std::atan2(2.0 * request->start.orientation.w *
                                 request->start.orientation.z,
                             1.0 - 2.0 * request->start.orientation.z *
                                         request->start.orientation.z) *
                      180.0 / 3.141592653589793,
                  response->planning_time_ms);
    } else {
      RCLCPP_INFO(get_logger(), "plan %lu: %s, %.2f m, %.1f ms, activated=%s",
                  response->plan.plan_id, response->plan.status.c_str(),
                  response->plan.length_m, response->planning_time_ms,
                  response->activated ? "true" : "false");
    }
  }

  void PublishMap() {
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "arena";
    map.header.stamp = now();
    map.info.resolution = static_cast<float>(planner_->config().resolution);
    map.info.width = static_cast<uint32_t>(
        std::ceil(planner_->config().width / planner_->config().resolution));
    map.info.height = static_cast<uint32_t>(
        std::ceil(planner_->config().height / planner_->config().resolution));
    map.info.origin.orientation.w = 1.0;
    map.data = planner_->collision_checker().occupancy_data();
    map_publisher_->publish(map);
  }

  std::unique_ptr<MapPlanner> planner_;
  rclcpp::Publisher<msg::RoutePlan>::SharedPtr plan_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr arena_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr navigation_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Service<srv::PlanRoute>::SharedPtr service_;
};

} // namespace new_map_planner

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<new_map_planner::PlannerNode>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("new_map_planner"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
