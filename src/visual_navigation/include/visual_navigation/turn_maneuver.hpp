#ifndef VISUAL_NAVIGATION__TURN_MANEUVER_HPP_
#define VISUAL_NAVIGATION__TURN_MANEUVER_HPP_

#include <cstddef>
#include <limits>
#include <string>

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

enum class TurnManeuverFailureReason {
  NONE,
  INVALID_CONTEXT,
  CURRENT_POSE_UNSAFE,
  ARC_REJECTED_NO_IMPROVEMENT,
  TRIM_REJECTED_NO_IMPROVEMENT,
  NO_TRIM_CANDIDATE,
  TRIM_LIMIT,
  EXIT_ALIGN_REJECTED,
  CANCELED,
};

inline const char *TurnManeuverFailureReasonName(
    TurnManeuverFailureReason reason) {
  switch (reason) {
  case TurnManeuverFailureReason::NONE:
    return "none";
  case TurnManeuverFailureReason::INVALID_CONTEXT:
    return "invalid_context";
  case TurnManeuverFailureReason::CURRENT_POSE_UNSAFE:
    return "current_pose_unsafe";
  case TurnManeuverFailureReason::ARC_REJECTED_NO_IMPROVEMENT:
    return "arc_rejected_no_improvement";
  case TurnManeuverFailureReason::TRIM_REJECTED_NO_IMPROVEMENT:
    return "trim_rejected_no_improvement";
  case TurnManeuverFailureReason::NO_TRIM_CANDIDATE:
    return "no_trim_candidate";
  case TurnManeuverFailureReason::TRIM_LIMIT:
    return "trim_limit";
  case TurnManeuverFailureReason::EXIT_ALIGN_REJECTED:
    return "exit_align_rejected";
  case TurnManeuverFailureReason::CANCELED:
    return "canceled";
  }
  return "unknown";
}

enum class TurnManeuverRecoveryDirection {
  NONE,
  REVERSE_ALONG_ENTRY,
  FORWARD_TO_EXIT,
};

inline const char *TurnManeuverRecoveryDirectionName(
    TurnManeuverRecoveryDirection direction) {
  switch (direction) {
  case TurnManeuverRecoveryDirection::NONE:
    return "none";
  case TurnManeuverRecoveryDirection::REVERSE_ALONG_ENTRY:
    return "reverse_along_entry";
  case TurnManeuverRecoveryDirection::FORWARD_TO_EXIT:
    return "forward_to_exit";
  }
  return "unknown";
}

struct TurnManeuverRejectionSignature {
  bool valid{false};
  CornerShuntSafetyFailure failure{CornerShuntSafetyFailure::NONE};
  int prediction_sample{-1};
  double local_x{0.0};
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
  double trim_reverse_hysteresis{0.015};
  double trim_min_direction_hold{0.30};
  double trim_min_direction_distance{0.010};
  int trim_completion_samples{3};
  int max_trim_direction_switches{2};
  double trim_no_progress_timeout{3.0};
  double trim_progress_distance{0.005};
  double trim_progress_yaw{0.03};
  int max_trim_attempts{6};
  int max_identical_rejections{1};
};

struct TurnManeuverInput {
  CornerShuntContext context;
  std::string junction_id;
  std::size_t junction_index{std::numeric_limits<std::size_t>::max()};
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
  int trim_direction{0};
  std::size_t trim_direction_switches{0};
  double trim_target_local_x{0.0};
  bool has_last_safe_pose{false};
  CornerShuntPose last_safe_pose;
  std::size_t junction_index{std::numeric_limits<std::size_t>::max()};
  std::string junction_id;
  CornerShuntPose junction_pose;
  double geometry_heading{0.0};
  double arrival_heading{0.0};
  double target_heading{0.0};
  TurnManeuverFailureReason failure_reason{TurnManeuverFailureReason::NONE};
  TurnManeuverRecoveryDirection suggested_recovery_direction{
      TurnManeuverRecoveryDirection::NONE};
  TurnManeuverRejectionSignature rejection_signature;

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
  bool trim_direction_change_allowed(const CornerShuntContext &context,
                                     double now_s) const;
  bool command_is_safe(const CornerShuntContext &context,
                       const CornerShuntTwist &command,
                       CornerShuntSafetyReport *report = nullptr) const;
  void attach_trim_diagnostics(TurnManeuverDecision *decision) const;
  void reset_trim_progress(const CornerShuntContext &context,
                           double now_s);
  bool trim_stalled(const CornerShuntContext &context, double now_s);
  void start_arc(const CornerShuntContext &context, double now_s);
  void attach_metadata(TurnManeuverDecision *decision) const;
  bool update_last_safe_pose(const CornerShuntContext &context,
                             CornerShuntSafetyReport *report = nullptr);
  bool record_rejection(const CornerShuntContext &context,
                        const CornerShuntSafetyReport &report);
  TurnManeuverRecoveryDirection recovery_direction() const;
  bool exit_aligned(const CornerShuntContext &context) const;
  CornerShuntTwist exit_align_command(const CornerShuntContext &context) const;

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
  int trim_direction_{0};
  double trim_direction_started_at_{0.0};
  double trim_direction_start_x_{0.0};
  int trim_completion_samples_{0};
  std::size_t trim_direction_switches_{0};
  std::size_t trim_attempts_{0};
  std::size_t trim_candidates_checked_{0};
  std::size_t trim_footprint_rejections_{0};
  std::size_t trim_turn_rejections_{0};
  bool has_safety_report_{false};
  CornerShuntSafetyReport safety_report_;
  double progress_started_at_{0.0};
  CornerShuntPose progress_pose_;
  double progress_longitudinal_error_{0.0};
  double progress_lateral_error_{0.0};
  double progress_yaw_error_{0.0};
  bool last_safe_pose_valid_{false};
  CornerShuntPose last_safe_pose_;
  std::size_t captured_junction_index_{
      std::numeric_limits<std::size_t>::max()};
  std::string captured_junction_id_;
  TurnManeuverFailureReason failure_reason_{TurnManeuverFailureReason::NONE};
  TurnManeuverRecoveryDirection recovery_direction_{
      TurnManeuverRecoveryDirection::NONE};
  TurnManeuverRejectionSignature rejection_signature_;
  std::size_t identical_rejection_count_{0};
};

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__TURN_MANEUVER_HPP_
