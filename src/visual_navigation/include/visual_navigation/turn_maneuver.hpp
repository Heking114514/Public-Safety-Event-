#ifndef VISUAL_NAVIGATION__TURN_MANEUVER_HPP_
#define VISUAL_NAVIGATION__TURN_MANEUVER_HPP_

#include <cmath>
#include <cstddef>

namespace visual_navigation {
struct CornerShuntPose { double x{0.0}; double y{0.0}; double yaw{0.0}; };
struct CornerShuntTwist { double linear_x{0.0}; double angular_z{0.0}; };
struct CornerShuntContext {
  CornerShuntPose pose; CornerShuntTwist applied_twist; CornerShuntTwist measured_twist;
  bool measured_yaw_rate_valid{false}; double junction_x{0.0}; double junction_y{0.0};
  double geometry_heading{0.0}; double arrival_heading{0.0}; double target_heading{0.0};
  bool reverse_arrival{false};
};
inline double NormalizeShuntAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}
enum class TurnManeuverPhase { IDLE, CAPTURE, APPROACH_ENTRY, TURN_60, DIAGONAL_CREEP, TURN_30, COMPLETE, PAUSED, FAULT };
enum class TurnManeuverResult { RUNNING, COMPLETED, NEED_REPLAN, CANCELED, FAULT };
struct TurnManeuverConfig {
  bool enabled{true}; double entry_target_x{-0.070}; double entry_target_y{-0.015};
  double entry_speed{0.035}; double turn_angular_speed{0.35};
  double turn_min_angular_speed{0.15}; double creep_linear_speed{0.030};
  double entry_position_tolerance{0.010};
};
struct TurnManeuverInput { CornerShuntContext context; bool motion_allowed{true}; bool localization_valid{true}; bool obstacle_blocked{false}; bool cancel{false}; };
struct TurnManeuverDecision {
  TurnManeuverPhase phase{TurnManeuverPhase::IDLE}; CornerShuntTwist command; TurnManeuverResult result{TurnManeuverResult::RUNNING};
  TurnManeuverDecision() = default;
  TurnManeuverDecision(TurnManeuverPhase p, CornerShuntTwist c, TurnManeuverResult r) : phase(p), command(c), result(r) {}
};
const char *TurnManeuverStatus(TurnManeuverPhase phase);
class TurnManeuverController {
public:
  explicit TurnManeuverController(TurnManeuverConfig config = {});
  void configure(const TurnManeuverConfig &config); void reset(); void begin(double now_s);
  bool active() const; bool enabled_for(double turn_error) const;
  std::size_t trim_attempts() const { return 0; } double trim_target_x() const { return config_.entry_target_x; }
  TurnManeuverDecision update(const TurnManeuverInput &input, double now_s);
  bool last_safe_pose_valid() const { return false; } CornerShuntPose last_safe_pose() const { return captured_.pose; }
private:
  TurnManeuverDecision terminal(TurnManeuverPhase phase, TurnManeuverResult result);
  CornerShuntContext live_context(const TurnManeuverInput &input) const;
  CornerShuntPose local_pose(const CornerShuntPose &pose) const;
  bool context_valid(const CornerShuntContext &context) const;
  TurnManeuverConfig config_; CornerShuntContext captured_;
  TurnManeuverPhase phase_{TurnManeuverPhase::IDLE}; bool active_{false}; bool capture_valid_{false};
  double phase_started_at_{0.0}; double creep_start_x_{0.0}; double creep_start_y_{0.0};
};
}  // namespace visual_navigation
#endif  // VISUAL_NAVIGATION__TURN_MANEUVER_HPP_
