#include "arena_path_planner/planner_internal.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace arena_path_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
Rectangle ReadRectangle(const YAML::Node & node)
{
  if (!node.IsSequence() || node.size() != 4) {
    throw std::runtime_error("rectangle must contain four coordinates");
  }
  Rectangle result{
    node[0].as<double>(), node[1].as<double>(),
    node[2].as<double>(), node[3].as<double>()};
  if (!IsFinite(result) || !(result.minimum_x < result.maximum_x) ||
    !(result.minimum_y < result.maximum_y))
  {
    throw std::runtime_error("invalid rectangle bounds");
  }
  return result;
}

Point ReadPoint(const YAML::Node & node)
{
  if (!node.IsSequence() || node.size() != 2) {
    throw std::runtime_error("point must contain x and y");
  }
  const Point result{node[0].as<double>(), node[1].as<double>()};
  if (!IsFinite(result)) {
    throw std::runtime_error("point coordinates must be finite");
  }
  return result;
}

void ReadTopic(
  const YAML::Node & topics, const char * key, std::string & destination)
{
  if (!topics || !topics[key]) {
    return;
  }
  destination = topics[key].as<std::string>();
  if (destination.empty()) {
    throw std::runtime_error(std::string("topics.") + key + " must not be empty");
  }
}
}  // namespace
PlannerConfig ArenaPlanner::LoadConfig(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  PlannerConfig config;
  const YAML::Node arena = root["arena"];
  config.width = arena["width_m"].as<double>();
  config.height = arena["height_m"].as<double>();
  config.resolution = arena["resolution_m"].as<double>();
  config.inflation_radius = arena["inflation_radius_m"].as<double>();
  config.preferred_clearance = arena["preferred_clearance_m"].as<double>();
  config.clearance_cost_weight = arena["clearance_cost_weight"].as<double>();
  if (arena["vehicle_length_m"]) {
    config.vehicle_length = arena["vehicle_length_m"].as<double>();
  }
  if (arena["vehicle_width_m"]) {
    config.vehicle_width = arena["vehicle_width_m"].as<double>();
  }
  if (arena["safety_margin_m"]) {
    config.safety_margin = arena["safety_margin_m"].as<double>();
  }
  if (arena["tracking_margin_m"]) {
    config.tracking_margin = arena["tracking_margin_m"].as<double>();
  }
  if (arena["allow_in_place_turns"]) {
    config.allow_in_place_turns = arena["allow_in_place_turns"].as<bool>();
  }
  for (const auto & node : arena["free_regions"]) {
    config.free_regions.push_back(ReadRectangle(node));
  }
  if (arena["staging_regions"]) {
    for (const auto & node : arena["staging_regions"]) {
      config.staging_regions.push_back(ReadRectangle(node));
    }
  }
  for (const auto & node : arena["obstacles"]) {
    config.obstacles.push_back(ReadRectangle(node));
  }
  config.default_start.position = ReadPoint(root["start"]["position_m"]);
  config.default_start.yaw = root["start"]["heading_deg"].as<double>() * kPi / 180.0;
  std::vector<std::pair<std::string, Point>> tasks;
  for (const auto & entry : root["tasks"]) {
    tasks.emplace_back(entry.first.as<std::string>(), ReadPoint(entry.second));
  }
  std::sort(
    tasks.begin(), tasks.end(),
    [](const auto & left, const auto & right) {
      try {
        return std::stoi(left.first) < std::stoi(right.first);
      } catch (const std::exception &) {
        return left.first < right.first;
      }
    });
  for (const auto & [label, point] : tasks) {
    config.default_labels.push_back(label);
    config.default_targets.push_back(point);
  }
  if (root["tunnels"]) {
    for (const auto & entry : root["tunnels"]) {
      const std::string label = entry.first.as<std::string>();
      if (entry.second.IsMap() && entry.second["entry"] && entry.second["exit"]) {
        const Point entrance = ReadPoint(entry.second["entry"]);
        const Point exit = ReadPoint(entry.second["exit"]);
        config.tunnel_segments.emplace_back(entrance, exit);
        config.tunnel_segment_labels.push_back(label);
        config.required_tunnel_points.push_back(entrance);
        config.required_tunnel_labels.push_back(label + "_ENTER");
        config.required_tunnel_points.push_back(exit);
        config.required_tunnel_labels.push_back(label + "_EXIT");
      } else {
        config.required_tunnel_labels.push_back(label);
        config.required_tunnel_points.push_back(ReadPoint(entry.second));
      }
    }
  }
  if (root["inspection_graph"]) {
    const YAML::Node graph = root["inspection_graph"];
    for (const auto & node : graph["nodes"]) {
      config.inspection_nodes.push_back(ReadPoint(node));
    }
    for (const auto & edge : graph["edges"]) {
      if (!edge.IsSequence() || edge.size() != 4) {
        throw std::runtime_error("inspection edge must contain from, to, label and tunnel");
      }
      config.inspection_edges.push_back(
        {edge[0].as<int>(), edge[1].as<int>(), edge[2].as<std::string>(), edge[3].as<bool>()});
    }
  }
  const YAML::Node planning = root["planning"];
  config.default_mode = planning["mode"].as<std::string>();
  config.task_tolerance = planning["task_tolerance_m"].as<double>();
  config.waypoint_spacing = planning["waypoint_spacing_m"].as<double>();
  config.curve_spacing = planning["curve_spacing_m"].as<double>();
  config.minimum_turning_radius = planning["minimum_turning_radius_m"].as<double>();
  config.maximum_heading_step =
    planning["maximum_heading_step_deg"].as<double>() * kPi / 180.0;
  if (planning["in_place_turn_heading_threshold_deg"]) {
    config.in_place_turn_heading_threshold =
      planning["in_place_turn_heading_threshold_deg"].as<double>() * kPi / 180.0;
  }
  const YAML::Node topics = root["topics"];
  ReadTopic(topics, "service", config.topics.service);
  ReadTopic(topics, "arena_path", config.topics.arena_path);
  ReadTopic(topics, "navigation_path", config.topics.navigation_path);
  ReadTopic(topics, "occupancy_grid", config.topics.occupancy_grid);
  ReadTopic(topics, "route_input", config.topics.route_input);
  ValidatePlannerConfig(config);
  return config;
}
}  // namespace arena_path_planner
