#ifndef VISUAL_NAVIGATION__POSE_RECOVERY_HPP_
#define VISUAL_NAVIGATION__POSE_RECOVERY_HPP_
#include <string>
#include <vector>
#include <map>
#include <navigation_safety/checker.hpp>
#include <mission_control_interfaces/msg/recovery_candidate.hpp>
#include "visual_navigation/turn_maneuver.hpp"

namespace visual_navigation {
// Road actions use mapped junctions and the existing fixed quarter-turn.
class PoseRecovery {
public:
  using Pose = navigation_safety::Pose;
  struct Step {
    enum class Kind { STRAIGHT, FIXED_TURN, STOP };
    Pose start, end, target;
    double linear, angular, duration;
    double initial_angular{0.0};
    std::string phase{"RETREAT_TO_TURN_POINT"};
    Kind kind{Kind::STRAIGHT};
    navigation_safety::Point junction{};
    double geometry_heading{0.0}, target_heading{0.0};
    bool reverse_arrival{false};
    std::string lane_id;
    navigation_safety::Point road_origin{};
  };
  struct Config {
    double speed{0.035}, angular{0.65}, primitive_s{0.45};
    double margin{0.002}, exit_distance{0.18}, max_radius{0.85};
    double base_offset_x{0.030}, base_offset_y{0.0};
    double angular_acceleration{1.8}, angular_deceleration{3.0}, control_dt{1.0/30.0};
    int max_expansions{220000};
    TurnManeuverConfig turn;
    double linear_acceleration{0.2}, linear_deceleration{0.3};
    double minimum_linear_speed{0.0};
  };
  void load(const std::string &map_file, navigation_safety::Footprint footprint,
            const Config &config);
  std::vector<mission_control_interfaces::msg::RecoveryCandidate>
  candidates(const Pose &pose, const navigation_safety::Polygon &obstacle);
  bool plan(const Pose &start,
            const mission_control_interfaces::msg::RecoveryCandidate &candidate,
            const std::vector<std::string> &blocked);
  bool safe(const std::vector<Pose> &poses) const;
  void set_dynamic_obstacles(const std::vector<navigation_safety::Polygon> &obstacles);
  bool exit_matches(const Pose &pose,
    const mission_control_interfaces::msg::RecoveryCandidate &candidate) const;
  std::string lane_at_pose(const Pose &pose) const;
  void set_incoming_lane(const std::string &lane,
                        navigation_safety::Point junction);
  Pose integrate(Pose pose, double v, double w, double dt) const;
  std::vector<Pose> motion_samples(Pose pose, double v, double target_w,
                                   double initial_w, double duration,
                                   double *final_w = nullptr) const;
  Pose reference(const Step &step, double elapsed) const;
  bool safe_motion(Pose pose, double v, double w, double duration) const;
  void reset_clock(double wall_seconds) { active_time_=0.0; last_wall_=wall_seconds; paused_=false; }
  void pause_clock() { paused_=true; }
  double active_seconds(double wall_seconds);
  const std::vector<Step> &steps() const { return steps_; }
  const std::string &blocked_lane() const { return blocked_lane_; }
  const Config &config() const { return config_; }
  int last_expansions() const { return last_expansions_; }
private:
  struct Lane { std::string id, from, to; double width; };
  Config config_;
  navigation_safety::Footprint footprint_;
  std::vector<navigation_safety::Rectangle> free_;
  std::vector<navigation_safety::Polygon> static_, dynamic_;
  std::map<std::string, navigation_safety::Point> nodes_;
  std::vector<Lane> lanes_;
  std::vector<navigation_safety::Polygon> observed_;
  bool live_obstacles_set_{false};
  std::string anchor_, blocked_lane_;
  std::string incoming_override_;
  navigation_safety::Point junction_override_{};
  std::vector<Step> steps_;
  double active_time_{0.0}, last_wall_{0.0};
  bool paused_{true};
  int last_expansions_{0};
  bool append_straight(Pose &pose, Pose target, double direction,
                       const std::string &phase, const std::string &lane);
  bool append_turn(Pose &pose, navigation_safety::Point junction,
                   double incoming, double outgoing, bool reverse_arrival,
                   const std::string &lane);
  bool braking_safe(Pose pose, double linear, double angular) const;
};
} // namespace visual_navigation
#endif
