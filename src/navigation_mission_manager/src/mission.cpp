#include "navigation_mission_manager/mission.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace navigation_mission_manager
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

template<typename Container>
bool contains(const Container & values, const std::string & value)
{
  return std::find(values.begin(), values.end(), value) != values.end();
}

std::vector<std::string> set_difference_in_order(
  const std::vector<std::string> & source, const std::set<std::string> & completed)
{
  std::vector<std::string> result;
  for (const auto & value : source) {
    if (completed.count(value) == 0U) {
      result.push_back(value);
    }
  }
  return result;
}

}  // namespace

bool PlanSpec::has_progress(
  const std::vector<std::string> & visit_order,
  const std::vector<std::string> & covered_lane_ids) const
{
  if (mode == "layer1") {
    return std::any_of(
      visit_order.begin(), visit_order.end(),
      [this](const auto & value) {return contains(target_ids, value);});
  }
  if (!required_lane_ids.empty()) {
    return std::any_of(
      covered_lane_ids.begin(), covered_lane_ids.end(),
      [this](const auto & value) {return contains(required_lane_ids, value);});
  }
  return true;
}

MissionLayout MissionLayout::from_file(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  MissionLayout layout;

  std::unordered_map<std::string, bool> tunnel_by_lane;
  for (const auto & lane : root["lanes"]) {
    tunnel_by_lane[lane["id"].as<std::string>()] = lane["tunnel"].as<bool>();
  }
  for (const auto & lane : root["missions"]["default_inspection_lanes"]) {
    const std::string id = lane.as<std::string>();
    const auto found = tunnel_by_lane.find(id);
    if (found == tunnel_by_lane.end()) {
      throw std::runtime_error("inspection mission references unknown lane: " + id);
    }
    (found->second ? layout.tunnel_lane_ids : layout.road_lane_ids).push_back(id);
  }
  for (const auto & task : root["missions"]["default_targets"]) {
    layout.task_ids.push_back(task.as<std::string>());
  }

  const auto origin = root["frame"]["map_origin_m"];
  const double origin_x = origin[0].as<double>();
  const double origin_y = origin[1].as<double>();
  const double heading = root["frame"]["map_x_heading_deg"].as<double>() * kPi / 180.0;
  const double cosine = std::cos(heading);
  const double sine = std::sin(heading);
  const YAML::Node nodes = root["nodes"];
  for (const auto & task : root["tasks"]) {
    const std::string id = task.first.as<std::string>();
    const std::string node_id = task.second.as<std::string>();
    const YAML::Node point = nodes[node_id];
    if (!point || point.size() != 2U) {
      throw std::runtime_error("task references unknown node: " + node_id);
    }
    const double dx = point[0].as<double>() - origin_x;
    const double dy = point[1].as<double>() - origin_y;
    layout.task_positions[id] = {
      cosine * dx + sine * dy,
      -sine * dx + cosine * dy};
  }
  if (root["rfid_supernodes"]) {
    std::set<std::string> mapped_tasks;
    for (const auto & item : root["rfid_supernodes"]) {
      const std::string supernode_id = item.first.as<std::string>();
      const std::string task_id = item.second.as<std::string>();
      if (supernode_id.empty() || layout.task_positions.count(task_id) == 0U ||
        !mapped_tasks.insert(task_id).second ||
        !layout.supernode_task_ids.emplace(supernode_id, task_id).second)
      {
        throw std::runtime_error(
                "RFID supernodes must uniquely reference configured task points");
      }
    }
    if (layout.supernode_task_ids.size() != layout.task_ids.size()) {
      throw std::runtime_error("every task point must have one RFID supernode id");
    }
  } else {
    throw std::runtime_error("map does not define RFID supernodes");
  }
  return layout;
}

StagedMission::StagedMission(MissionLayout layout, bool unified)
: layout_(std::move(layout)), unified_(unified)
{
  reset();
}

void StagedMission::reset()
{
  partial_ = false;
  requested_tasks_ = layout_.task_ids;
  completed_tasks_.clear();
  completed_tunnels_.clear();
  confirmed_lanes_.clear();
  skipped_roads_.clear();
  started_ = false;
  running_ = false;
  paused_ = false;
  finished_ = false;
}

void StagedMission::start(const std::vector<std::string> & task_ids)
{
  reset();
  const auto & selected = (unified_ || task_ids.empty()) ? layout_.task_ids : task_ids;
  requested_tasks_.clear();
  for (const auto & task : selected) {
    if (contains(layout_.task_ids, task) && !contains(requested_tasks_, task)) {
      requested_tasks_.push_back(task);
    }
  }
  if (requested_tasks_.empty()) {
    throw std::invalid_argument("三阶段任务至少需要一个任务点");
  }
  started_ = true;
  running_ = true;
}

void StagedMission::pause()
{
  if (running_) {
    paused_ = true;
  }
}

void StagedMission::resume()
{
  if (running_) {
    paused_ = false;
  }
}

void StagedMission::abort()
{
  running_ = false;
  paused_ = true;
}

std::vector<std::string> StagedMission::remaining_tasks() const
{
  return set_difference_in_order(requested_tasks_, completed_tasks_);
}

std::vector<std::string> StagedMission::remaining_tunnels() const
{
  return set_difference_in_order(layout_.tunnel_lane_ids, completed_tunnels_);
}

std::vector<std::string> StagedMission::remaining_roads() const
{
  auto excluded = confirmed_lanes_;
  excluded.insert(skipped_roads_.begin(), skipped_roads_.end());
  return set_difference_in_order(layout_.road_lane_ids, excluded);
}

std::string StagedMission::current_stage() const
{
  if (!started_ || finished_) {
    return {};
  }
  if (unified_) {
    return "mission";
  }
  if (!remaining_tasks().empty()) {
    return "layer1";
  }
  if (!remaining_tunnels().empty()) {
    return "layer2";
  }
  return "layer3";
}

PlanSpec StagedMission::plan_spec() const
{
  if (!running_ || paused_ || finished_) {
    throw std::runtime_error("三阶段任务当前不可规划");
  }
  PlanSpec spec;
  spec.mode = current_stage();
  spec.allow_partial = true;
  spec.completed_lane_ids.assign(confirmed_lanes_.begin(), confirmed_lanes_.end());
  if (spec.mode == "mission") {
    spec.target_ids = remaining_tasks();
    spec.return_home = true;
    return spec;
  }
  if (spec.mode == "layer1") {
    spec.target_ids = remaining_tasks();
    spec.return_home = false;
    return spec;
  }
  if (spec.mode == "layer2") {
    spec.required_lane_ids = remaining_tunnels();
    spec.return_home = false;
    const std::set<std::string> unfinished(
      spec.required_lane_ids.begin(), spec.required_lane_ids.end());
    spec.completed_lane_ids.clear();
    for (const auto & lane : confirmed_lanes_) {
      if (unfinished.count(lane) == 0U) {
        spec.completed_lane_ids.push_back(lane);
      }
    }
    return spec;
  }
  if (spec.mode == "layer3") {
    spec.required_lane_ids = remaining_roads();
    spec.return_home = true;
    return spec;
  }
  throw std::runtime_error("三阶段任务已经完成");
}

std::size_t StagedMission::commit(
  const PlanSpec & spec,
  const std::vector<std::string> & confirmed_task_ids,
  const std::vector<std::string> & covered_lane_ids,
  bool plan_complete)
{
  if (!running_ || spec.mode != current_stage()) {
    throw std::runtime_error("完成结果与当前任务阶段不匹配");
  }
  const std::size_t before = progress_count();
  confirmed_lanes_.insert(covered_lane_ids.begin(), covered_lane_ids.end());
  if (spec.mode == "mission") {
    for (const auto & task : confirmed_task_ids) {
      if (contains(requested_tasks_, task)) completed_tasks_.insert(task);
    }
    for (const auto & lane : covered_lane_ids) {
      if (contains(layout_.tunnel_lane_ids, lane)) completed_tunnels_.insert(lane);
    }
    // Both SUCCESS and PARTIAL_SUCCESS routes terminate at home. Never retry
    // deferred targets indefinitely after a successfully executed return route.
    partial_ = !plan_complete || !remaining_tasks().empty() || !remaining_tunnels().empty();
    finished_ = true;
    running_ = false;
  } else if (spec.mode == "layer1") {
    for (const auto & task : confirmed_task_ids) {
      if (contains(spec.target_ids, task)) {
        completed_tasks_.insert(task);
      }
    }
  } else if (spec.mode == "layer2") {
    for (const auto & lane : covered_lane_ids) {
      if (contains(spec.required_lane_ids, lane)) {
        completed_tunnels_.insert(lane);
      }
    }
  } else if (spec.mode == "layer3" && plan_complete && remaining_roads().empty()) {
    finished_ = true;
    running_ = false;
  }
  return progress_count() - before;
}

void StagedMission::commit_empty(const PlanSpec & spec, bool plan_complete)
{
  if (spec.mode == "mission" && plan_complete) {
    finished_ = true;
    running_ = false;
    return;
  }
  if (spec.mode != "layer3" || !spec.required_lane_ids.empty() || !plan_complete ||
    !remaining_roads().empty())
  {
    throw std::runtime_error("只有无需移动的最终回程阶段可以直接完成");
  }
  finished_ = true;
  running_ = false;
}

std::size_t StagedMission::progress_count() const
{
  return completed_tasks_.size() + completed_tunnels_.size() + confirmed_lanes_.size();
}

void StagedMission::skip_blocked_roads(const std::set<std::string> & blocked)
{
  for (const auto & lane : layout_.road_lane_ids) {
    if (blocked.count(lane) != 0U && confirmed_lanes_.count(lane) == 0U) {
      skipped_roads_.insert(lane);
    }
  }
}

std::string StagedMission::summary() const
{
  const std::string skipped = skipped_roads_.empty() ? std::string{} :
    " | 跳过阻塞普通道路 " + std::to_string(skipped_roads_.size());
  if (unified_) {
    return std::string(finished_ ? (partial_ ? "PARTIAL_SUCCESS 统一巡航已返航" : "SUCCESS 统一巡航已返航") : "统一巡航") +
      " | 未确认 RFID " + std::to_string(remaining_tasks().size()) +
      " 隧道 " + std::to_string(remaining_tunnels().size()) + skipped;
  }
  if (finished_) {
    return "三阶段任务完成" + skipped;
  }
  const std::string stage = current_stage();
  if (stage.empty()) {
    return "三阶段任务未开始";
  }
  const std::string label = stage == "layer1" ? "1/3 任务点" :
    (stage == "layer2" ? "2/3 隧道" : "3/3 道路补漏与回程");
  return label + " | 任务 " + std::to_string(remaining_tasks().size()) +
         " 隧道 " + std::to_string(remaining_tunnels().size()) +
         " 道路 " + std::to_string(remaining_roads().size()) + skipped;
}

}  // namespace navigation_mission_manager
