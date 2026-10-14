#ifndef VISUAL_NAVIGATION__CORNER_RETURN_HPP_
#define VISUAL_NAVIGATION__CORNER_RETURN_HPP_

#include "visual_navigation/turn_maneuver.hpp"

namespace visual_navigation {

enum class CornerReturnPhase {
  IDLE, CAPTURE, APPROACH_ENTRY, TURN_30, DIAGONAL_CREEP, TURN_60, RETURN_ENTRY,
  COMPLETE, PAUSED, FAULT
};

struct CornerReturnConfig {
  TurnManeuverConfig turn;
  // 物理值及输出限制由中央 YAML/导航 YAML 注入，缺失不能构建模板。
  double base_offset_x{NAN}, base_offset_y{NAN};
  double linear_acceleration{NAN}, linear_deceleration{NAN};
  double angular_acceleration{NAN}, angular_deceleration{NAN};
  double minimum_linear_speed{NAN}, control_dt{NAN};
};

struct CornerReturnInput {
  CornerShuntContext context;
  bool motion_allowed{true};
  bool localization_valid{true};
  bool obstacle_blocked{false};
  bool cancel{false};
};

struct CornerReturnDecision {
  CornerReturnPhase phase{CornerReturnPhase::IDLE};
  CornerShuntTwist command;
  TurnManeuverResult result{TurnManeuverResult::RUNNING};
};

const char *CornerReturnStatus(CornerReturnPhase phase);

class CornerReturnController {
public:
  explicit CornerReturnController(CornerReturnConfig config = {});
  void configure(const CornerReturnConfig &config);
  void reset();
  void begin(const CornerShuntContext &context, double now_s);
  CornerReturnDecision update(const CornerReturnInput &input, double now_s);
  bool active() const { return active_; }
  bool geometry_valid() const { return captured_valid_; }
  CornerReturnPhase phase() const { return phase_; }
  CornerShuntPose entrance_pose() const { return entrance_; }
  CornerShuntPose exit_pose() const { return exit_; }

private:
  CornerReturnDecision terminal(CornerReturnPhase phase,
                                TurnManeuverResult result);
  bool valid(const CornerShuntContext &context) const;
  bool build_geometry();
  CornerReturnConfig config_;
  CornerShuntContext captured_;
  CornerShuntPose entrance_, exit_, sixty_end_, creep_end_;
  CornerReturnPhase phase_{CornerReturnPhase::IDLE};
  CornerReturnPhase paused_phase_{CornerReturnPhase::IDLE};
  bool active_{false};
  bool captured_valid_{false};
  double phase_started_at_{0.0};
  double progress_started_at_{0.0};
  CornerShuntPose progress_pose_;
};

}  // namespace visual_navigation

#endif
