#include "new_map_planner/config.hpp"

#include "new_map_planner/geometry.hpp"
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace new_map_planner {
namespace {

Point ReadPoint(const YAML::Node &node) {
  if (!node.IsSequence() || node.size() != 2) {
    throw std::runtime_error("point must contain exactly two coordinates");
  }
  return {node[0].as<double>(), node[1].as<double>()};
}

Rectangle ReadRectangle(const YAML::Node &node) {
  if (!node.IsSequence() || node.size() != 4) {
    throw std::runtime_error(
        "rectangle must contain min_x, min_y, max_x, max_y");
  }
  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>(),
          node[3].as<double>()};
}

std::vector<std::string> ReadStrings(const YAML::Node &node) {
  std::vector<std::string> values;
  if (!node) {
    return values;
  }
  values.reserve(node.size());
  for (const YAML::Node &value : node) {
    values.push_back(value.as<std::string>());
  }
  return values;
}

bool FinitePoint(const Point &point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

} // namespace

PlannerConfig LoadConfig(const std::string &map_path,
                         const std::string &vehicle_path) {
  const YAML::Node root = YAML::LoadFile(map_path);
  const YAML::Node vehicle = YAML::LoadFile(vehicle_path);
  PlannerConfig config;
  config.map_id = root["map_id"].as<std::string>();
  config.width = root["arena"]["width_m"].as<double>();
  config.height = root["arena"]["height_m"].as<double>();
  config.resolution = root["arena"]["display_resolution_m"].as<double>();
  for (const YAML::Node &region : root["arena"]["free_regions"]) {
    config.free_regions.push_back(ReadRectangle(region));
  }
  for (const YAML::Node &obstacle : root["arena"]["obstacles"]) {
    config.static_obstacles.push_back(ReadRectangle(obstacle));
  }

  config.vehicle_length = vehicle["chassis"]["length_m"].as<double>();
  config.vehicle_width = vehicle["chassis"]["width_m"].as<double>();
  config.base_link_offset_x =
      vehicle["differential_drive"]["base_link_offset_x_m"].as<double>();
  config.base_link_offset_y =
      vehicle["differential_drive"]["base_link_offset_y_m"].as<double>();
  config.safety_margin = root["planning"]["safety_margin_m"].as<double>();
  config.turn_safety_margin =
      root["planning"]["turn_safety_margin_m"].as<double>();
  config.lane_snap_distance =
      root["planning"]["lane_snap_distance_m"].as<double>();
  config.node_capture_distance =
      root["planning"]["node_capture_distance_m"].as<double>();
  config.start_heading_tolerance =
      root["planning"]["start_heading_tolerance_deg"].as<double>() * kPi /
      180.0;
  config.turn_cost = root["planning"]["turn_cost_m"].as<double>();
  config.collision_sample_step =
      root["planning"]["collision_sample_step_m"].as<double>();
  config.publish_shunt_markers =
      root["planning"]["publish_shunt_markers"].as<bool>();
  config.require_return_home = root["planning"]["return_home"].as<bool>();

  config.transform.arena_origin = ReadPoint(root["frame"]["map_origin_m"]);
  config.transform.arena_yaw =
      root["frame"]["map_x_heading_deg"].as<double>() * kPi / 180.0;

  std::unordered_map<std::string, int> node_indices;
  for (const auto &item : root["nodes"]) {
    Node node;
    node.id = item.first.as<std::string>();
    node.position = ReadPoint(item.second);
    node_indices.emplace(node.id, static_cast<int>(config.nodes.size()));
    config.nodes.push_back(std::move(node));
  }
  const std::string home_id = root["home_node"].as<std::string>();
  const auto home = node_indices.find(home_id);
  if (home == node_indices.end()) {
    throw std::runtime_error("home_node does not name a configured node");
  }
  config.home_node = home->second;

  for (const YAML::Node &item : root["lanes"]) {
    Lane lane;
    lane.id = item["id"].as<std::string>();
    const std::string from = item["from"].as<std::string>();
    const std::string to = item["to"].as<std::string>();
    if (node_indices.count(from) == 0 || node_indices.count(to) == 0) {
      throw std::runtime_error("lane " + lane.id +
                               " references an unknown node");
    }
    lane.from = node_indices.at(from);
    lane.to = node_indices.at(to);
    lane.width = item["width_m"].as<double>();
    lane.bidirectional = item["bidirectional"].as<bool>();
    lane.inspection = item["inspection"].as<bool>();
    lane.tunnel = item["tunnel"].as<bool>();
    config.lanes.push_back(std::move(lane));
  }

  for (const auto &item : root["tasks"]) {
    Task task;
    task.id = item.first.as<std::string>();
    const std::string node_id = item.second.as<std::string>();
    if (node_indices.count(node_id) == 0) {
      throw std::runtime_error("task " + task.id +
                               " references an unknown node");
    }
    task.node = node_indices.at(node_id);
    config.tasks.push_back(std::move(task));
  }
  if (root["rfid_supernodes"]) {
    std::unordered_map<std::string, int> task_nodes;
    for (const Task &task : config.tasks) {
      task_nodes.emplace(task.id, task.node);
    }
    std::unordered_set<int> supernode_nodes;
    std::unordered_set<std::string> supernode_ids;
    for (const auto &item : root["rfid_supernodes"]) {
      const std::string supernode_id = item.first.as<std::string>();
      const std::string task_id = item.second.as<std::string>();
      const auto task = task_nodes.find(task_id);
      if (supernode_id.empty() || task == task_nodes.end() ||
          !supernode_ids.insert(supernode_id).second ||
          !supernode_nodes.insert(task->second).second) {
        throw std::runtime_error(
            "RFID supernodes must uniquely reference configured task nodes");
      }
      config.rfid_supernodes.emplace(task->second, supernode_id);
    }
    if (config.rfid_supernodes.size() != config.tasks.size()) {
      throw std::runtime_error(
          "every configured task node must have one RFID supernode id");
    }
  }
  config.default_targets = ReadStrings(root["missions"]["default_targets"]);
  config.default_inspection_lanes =
      ReadStrings(root["missions"]["default_inspection_lanes"]);

  const YAML::Node topics = root["topics"];
  config.topics.service = topics["service"].as<std::string>();
  config.topics.plan = topics["plan"].as<std::string>();
  config.topics.arena_path = topics["arena_path"].as<std::string>();
  config.topics.navigation_path = topics["navigation_path"].as<std::string>();
  config.topics.occupancy_grid = topics["occupancy_grid"].as<std::string>();

  ValidateConfig(config);
  return config;
}

void ValidateConfig(const PlannerConfig &config) {
  if (config.map_id.empty() || config.width <= 0.0 || config.height <= 0.0 ||
      config.resolution <= 0.0 || config.vehicle_length <= 0.0 ||
      config.vehicle_width <= 0.0 ||
      config.safety_margin < 0.0 || config.turn_safety_margin < 0.0 ||
      config.node_capture_distance < 0.0 ||
      config.node_capture_distance > config.lane_snap_distance ||
      config.collision_sample_step <= 0.0) {
    throw std::runtime_error(
        "map dimensions and planning dimensions must be positive");
  }
  if (config.nodes.empty() || config.lanes.empty()) {
    throw std::runtime_error("road graph must contain nodes and lanes");
  }
  if (!config.publish_shunt_markers) {
    throw std::runtime_error(
        "publish_shunt_markers must be true for executable navigation paths");
  }
  if (config.home_node < 0 ||
      config.home_node >= static_cast<int>(config.nodes.size())) {
    throw std::runtime_error("home node index is invalid");
  }
  std::unordered_set<std::string> node_ids;
  for (const Node &node : config.nodes) {
    if (node.id.empty() || !FinitePoint(node.position) ||
        !node_ids.insert(node.id).second) {
      throw std::runtime_error(
          "node ids must be unique and coordinates finite");
    }
  }
  std::unordered_set<std::string> lane_ids;
  const double footprint_width =
      config.vehicle_width + 2.0 * config.safety_margin;
  for (const Lane &lane : config.lanes) {
    if (lane.id.empty() || !lane_ids.insert(lane.id).second ||
        lane.from == lane.to || lane.from < 0 || lane.to < 0 ||
        lane.from >= static_cast<int>(config.nodes.size()) ||
        lane.to >= static_cast<int>(config.nodes.size())) {
      throw std::runtime_error(
          "lane ids and endpoints must be valid and unique");
    }
    if (lane.width + 1.0e-9 < footprint_width) {
      throw std::runtime_error("lane " + lane.id +
                               " is narrower than the safety footprint");
    }
    const Point &begin = config.nodes[lane.from].position;
    const Point &end = config.nodes[lane.to].position;
    if (std::abs(begin.x - end.x) > 1.0e-9 &&
        std::abs(begin.y - end.y) > 1.0e-9) {
      throw std::runtime_error("lane " + lane.id + " is not axis aligned");
    }
  }
  std::unordered_set<std::string> task_ids;
  for (const Task &task : config.tasks) {
    if (task.id.empty() || !task_ids.insert(task.id).second || task.node < 0 ||
        task.node >= static_cast<int>(config.nodes.size())) {
      throw std::runtime_error(
          "task ids and task nodes must be valid and unique");
    }
  }
  for (const std::string &task : config.default_targets) {
    if (task_ids.count(task) == 0) {
      throw std::runtime_error("default target " + task + " is not configured");
    }
  }
  for (const std::string &lane : config.default_inspection_lanes) {
    if (lane_ids.count(lane) == 0) {
      throw std::runtime_error("default inspection lane " + lane +
                               " is not configured");
    }
  }
}

} // namespace new_map_planner
