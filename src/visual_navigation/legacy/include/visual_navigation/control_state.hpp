#ifndef VISUAL_NAVIGATION__CONTROL_STATE_HPP_
#define VISUAL_NAVIGATION__CONTROL_STATE_HPP_

namespace visual_navigation {

enum class ControlPhase {
  FOLLOW,
  ALIGN_PATH,
  RECOVER_WAYPOINT,
  BRAKE,
  WAIT,
  ALIGN_GOAL,
  RECOVER_GOAL
};

enum class GoalStage { ROUTE, PENDING_STOP, STOPPED, RECOVERY };

enum class WaitAction { ADVANCE, COMPLETE };

inline const char *phase_status(ControlPhase phase) {
  switch (phase) {
  case ControlPhase::FOLLOW:
    return "FOLLOWING";
  case ControlPhase::ALIGN_PATH:
    return "ROTATING_TO_PATH";
  case ControlPhase::RECOVER_WAYPOINT:
  case ControlPhase::RECOVER_GOAL:
    // Keep the legacy public status after the final-recovery entry cycle.
    return "RECOVERING_WAYPOINT";
  case ControlPhase::BRAKE:
    return "BRAKING_AT_WAYPOINT";
  case ControlPhase::WAIT:
    return "WAITING_AT_WAYPOINT";
  case ControlPhase::ALIGN_GOAL:
    return "ALIGNING_FINAL_YAW";
  }
  return "FOLLOWING";
}

class ControlState {
public:
  ControlPhase phase() const { return phase_; }
  GoalStage goal_stage() const { return goal_stage_; }
  WaitAction wait_action() const { return wait_action_; }

  bool in(ControlPhase phase) const { return phase_ == phase; }
  bool waiting() const { return in(ControlPhase::WAIT); }
  bool goal_captured() const {
    return goal_stage_ == GoalStage::PENDING_STOP ||
           goal_stage_ == GoalStage::STOPPED;
  }
  bool goal_stopped() const { return goal_stage_ == GoalStage::STOPPED; }
  bool recovering_goal() const { return goal_stage_ == GoalStage::RECOVERY; }

  void reset_run() {
    phase_ = ControlPhase::FOLLOW;
    goal_stage_ = GoalStage::ROUTE;
    wait_action_ = WaitAction::ADVANCE;
    path_aligned_ = false;
    clear_anchor();
  }

  void reset_segment() {
    phase_ = ControlPhase::FOLLOW;
    goal_stage_ = GoalStage::ROUTE;
    path_aligned_ = false;
    clear_anchor();
  }

  void interrupt_maneuver() {
    clear_anchor();
    // GoalStage survives so final-position reach checks keep their legacy
    // distance-only semantics when control resumes.
    if (!waiting())
      phase_ = ControlPhase::FOLLOW;
  }

  void clear_goal() {
    goal_stage_ = GoalStage::ROUTE;
    if (phase_ == ControlPhase::ALIGN_GOAL ||
        phase_ == ControlPhase::RECOVER_GOAL) {
      phase_ = ControlPhase::FOLLOW;
    }
  }

  void capture_goal() {
    if (goal_stage_ != GoalStage::STOPPED)
      goal_stage_ = GoalStage::PENDING_STOP;
  }

  void begin_path_alignment(double x, double y) {
    phase_ = ControlPhase::ALIGN_PATH;
    set_anchor(x, y);
  }

  void finish_path_alignment() {
    phase_ = ControlPhase::FOLLOW;
    path_aligned_ = true;
    clear_anchor();
  }

  void mark_path_aligned() {
    path_aligned_ = true;
    phase_ = ControlPhase::FOLLOW;
  }

  void begin_waypoint_recovery() {
    phase_ = ControlPhase::RECOVER_WAYPOINT;
    clear_anchor();
  }

  void begin_braking() {
    phase_ = ControlPhase::BRAKE;
    clear_anchor();
  }

  void finish_braking(bool final_waypoint) {
    clear_anchor();
    if (final_waypoint) {
      goal_stage_ = GoalStage::STOPPED;
      phase_ = ControlPhase::ALIGN_GOAL;
    } else {
      goal_stage_ = GoalStage::ROUTE;
      phase_ = ControlPhase::FOLLOW;
    }
  }

  void begin_wait(WaitAction action) {
    phase_ = ControlPhase::WAIT;
    wait_action_ = action;
    clear_anchor();
  }

  WaitAction finish_wait() {
    const WaitAction completed = wait_action_;
    phase_ = ControlPhase::FOLLOW;
    wait_action_ = WaitAction::ADVANCE;
    return completed;
  }

  void begin_goal_alignment() { phase_ = ControlPhase::ALIGN_GOAL; }

  void begin_goal_recovery() {
    goal_stage_ = GoalStage::RECOVERY;
    phase_ = ControlPhase::RECOVER_GOAL;
    clear_anchor();
  }

  bool path_aligned() const { return path_aligned_; }
  bool anchor_valid() const { return anchor_valid_; }
  double anchor_x() const { return anchor_x_; }
  double anchor_y() const { return anchor_y_; }

  void ensure_anchor(double x, double y) {
    if (!anchor_valid_)
      set_anchor(x, y);
  }

private:
  void set_anchor(double x, double y) {
    anchor_x_ = x;
    anchor_y_ = y;
    anchor_valid_ = true;
  }

  void clear_anchor() {
    anchor_valid_ = false;
    anchor_x_ = 0.0;
    anchor_y_ = 0.0;
  }

  ControlPhase phase_{ControlPhase::FOLLOW};
  GoalStage goal_stage_{GoalStage::ROUTE};
  WaitAction wait_action_{WaitAction::ADVANCE};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  bool path_aligned_{false};
  bool anchor_valid_{false};
};

} // namespace visual_navigation

#endif // VISUAL_NAVIGATION__CONTROL_STATE_HPP_
