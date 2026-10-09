#ifndef VISUAL_NAVIGATION__TURN_MANEUVER_HPP_
#define VISUAL_NAVIGATION__TURN_MANEUVER_HPP_

#include <cstddef>

#include "visual_navigation/corner_shunt.hpp"

namespace visual_navigation {

enum class TurnManeuverPhase {
  IDLE,
  CAPTURE,
  ENTRY_CHECK,
  TRIM_SEARCH,
  REVERSE_TRIM,
  FORWARD_TRIM,
  TRIM_SETTLE,
  ARC,
  EXIT_ALIGN,
  COMPLETE,
  PAUSED,
  NEED_REPLAN,
  FAULT
};

enum class TurnManeuverResult {
  RUNNING,
  COMPLETED,
  NEED_REPLAN,
  CANCELED,
  FAULT
};

struct TurnManeuverConfig {
  CornerShuntConfig corner;
  double trim_speed{0.035};
  double trim_search_step{0.020};
  double trim_max_distance{0.20};
  double preferred_entry_x{-0.050};
  double direct_entry_yaw_tolerance{0.20};
  double trim_position_tolerance{0.012};
  double trim_lateral_tolerance{0.018};
  double trim_yaw_tolerance{0.15};
  double trim_heading_gain{1.20};
  double trim_lateral_gain{2.00};
  double trim_yaw_rate_damping{0.35};
  double trim_max_angular_speed{0.25};
  double trim_settle_duration{0.20};
  double trim_no_progress_timeout{3.0};
  double trim_progress_distance{0.005};
  double trim_progress_yaw{0.03};
  int max_trim_attempts{6};
};

struct TurnManeuverInput {
  CornerShuntContext context;
  bool motion_allowed{true};
  bool localization_valid{true};
  bool obstacle_blocked{false};
  bool cancel{false};
};

struct TurnManeuverDecision {
  TurnManeuverPhase phase{TurnManeuverPhase::IDLE};
  CornerShuntTwist command;
  TurnManeuverResult result{TurnManeuverResult::RUNNING};
  bool has_safety_report{false};
  CornerShuntSafetyReport safety_report;
  bool trim_target_selected{false};
  bool trim_limit_reached{false};
  std::size_t trim_attempts{0};
  std::size_t trim_candidates_checked{0};
  std::size_t trim_footprint_rejections{0};
  std::size_t trim_turn_rejections{0};
  double trim_target_local_x{0.0};

  TurnManeuverDecision() = default;
  TurnManeuverDecision(TurnManeuverPhase phase_value,
                       CornerShuntTwist command_value,
                       TurnManeuverResult result_value)
      : phase(phase_value), command(command_value), result(result_value) {}
};

const char *TurnManeuverStatus(TurnManeuverPhase phase);

class TurnManeuverController {
public:
  explicit TurnManeuverController(TurnManeuverConfig config = {});

  void configure(const TurnManeuverConfig &config);
  void reset();
  void begin(double now_s);
  bool active() const;
  bool enabled_for(double turn_error) const;
  std::size_t trim_attempts() const;
  double trim_target_x() const;

  TurnManeuverDecision update(const TurnManeuverInput &input, double now_s);

private:
  TurnManeuverDecision transition(TurnManeuverPhase phase);
  TurnManeuverDecision terminal(TurnManeuverPhase phase,
                                TurnManeuverResult result);
  CornerShuntContext live_context(const TurnManeuverInput &input) const;
  CornerShuntPose local_pose(const CornerShuntPose &pose) const;
  CornerShuntPose world_pose(double local_x, double local_y,
                             double yaw) const;
  bool context_valid(const CornerShuntContext &context) const;
  bool direct_entry_allowed(const CornerShuntContext &context) const;
  bool direct_turn_feasible(const CornerShuntContext &start) const;
  bool find_trim_target(const CornerShuntContext &context);
  bool trim_target_reached(const CornerShuntContext &context) const;
  CornerShuntTwist trim_command(const CornerShuntContext &context) const;
  bool command_is_safe(const CornerShuntContext &context,
                       const CornerShuntTwist &command,
                       CornerShuntSafetyReport *report = nullptr) const;
  void attach_trim_diagnostics(TurnManeuverDecision *decision) const;
  bool trim_stalled(const CornerShuntContext &context, double now_s);
  void start_arc(const CornerShuntContext &context, double now_s);

  TurnManeuverConfig config_;
  CornerShuntController arc_controller_;
  CornerShuntContext captured_;
  TurnManeuverPhase phase_{TurnManeuverPhase::IDLE};
  bool active_{false};
  bool capture_valid_{false};
  bool arc_started_{false};
  bool paused_{false};
  double paused_at_{0.0};
  double started_at_{0.0};
  double phase_started_at_{0.0};
  double trim_search_distance_{0.0};
  double trim_target_x_{0.0};
  std::size_t trim_attempts_{0};
  std::size_t trim_candidates_checked_{0};
  std::size_t trim_footprint_rejections_{0};
  std::size_t trim_turn_rejections_{0};
  bool has_safety_report_{false};
  CornerShuntSafetyReport safety_report_;
  double progress_started_at_{0.0};
  CornerShuntPose progress_pose_;
};

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__TURN_MANEUVER_HPP_
