#pragma once

#include <cstddef>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace navigation_mission_manager
{

struct PlanSpec
{
  std::string mode;
  std::vector<std::string> target_ids;
  std::vector<std::string> required_lane_ids;
  std::vector<std::string> completed_lane_ids;
  bool return_home{true};
  bool allow_partial{false};

  bool has_progress(
    const std::vector<std::string> & visit_order,
    const std::vector<std::string> & covered_lane_ids) const;
};

struct MissionLayout
{
  std::vector<std::string> task_ids;
  std::vector<std::string> tunnel_lane_ids;
  std::vector<std::string> road_lane_ids;
  std::map<std::string, std::pair<double, double>> task_positions;
  // Physical RFID checkpoints have stable map identities, independent of card IDs.
  std::map<std::string, std::string> supernode_task_ids;

  static MissionLayout from_file(const std::string & path);
};

class StagedMission
{
public:
  explicit StagedMission(MissionLayout layout);

  void reset();
  void start(const std::vector<std::string> & task_ids = {});
  void pause();
  void resume();
  void abort();

  std::vector<std::string> remaining_tasks() const;
  std::vector<std::string> remaining_tunnels() const;
  std::vector<std::string> remaining_roads() const;
  std::string current_stage() const;
  PlanSpec plan_spec() const;
  std::size_t commit(
    const PlanSpec & spec,
    const std::vector<std::string> & confirmed_task_ids,
    const std::vector<std::string> & covered_lane_ids,
    bool plan_complete);
  void commit_empty(const PlanSpec & spec, bool plan_complete);
  std::size_t progress_count() const;
  std::string summary() const;

  const MissionLayout & layout() const {return layout_;}
  const std::vector<std::string> & requested_tasks() const {return requested_tasks_;}
  const std::set<std::string> & completed_tasks() const {return completed_tasks_;}
  const std::set<std::string> & completed_tunnels() const {return completed_tunnels_;}
  const std::set<std::string> & confirmed_lanes() const {return confirmed_lanes_;}
  bool started() const {return started_;}
  bool running() const {return running_;}
  bool paused() const {return paused_;}
  bool finished() const {return finished_;}

private:
  MissionLayout layout_;
  std::vector<std::string> requested_tasks_;
  std::set<std::string> completed_tasks_;
  std::set<std::string> completed_tunnels_;
  std::set<std::string> confirmed_lanes_;
  bool started_{false};
  bool running_{false};
  bool paused_{false};
  bool finished_{false};
};

}  // namespace navigation_mission_manager
