#include "visual_navigation/waypoint_navigator.hpp"

#include <algorithm>
#include <cmath>

namespace {
double TurnWallSeconds() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace

void WaypointNavigator::PauseActiveTurn() {
  if (!turnManeuverController_.active()) return;
  visual_navigation::TurnManeuverInput input;
  input.motion_allowed = false;
  input.context.pose = {currentX_, currentY_, currentYaw_};
  turnManeuverController_.update(input, TurnWallSeconds());
}

bool WaypointNavigator::BeginTurnObstacleRecovery() {
  if (!turnManeuverController_.active() || !stoppedAtPlannedTurn_ ||
      stoppedTurnReverseArrival_ || currentWaypointIndex_ >= waypoints_.size())
    return false;
  const double turn = NormalizeAngle(stoppedTurnTargetHeading_ -
                                     stoppedTurnGeometryHeading_);
  if (std::abs(std::abs(turn) - kPi / 2.0) >
      turnManeuverConfig_.entry_yaw_tolerance)
    return false;
  // Save once. Subsequent detections may invalidate safety, but cannot move
  // this junction, the original approach heading, or the eventual return lane.
  turnObstacleContext_ = BuildCornerShuntContext(
      BuildControlFrame({}, false));
  turnObstacleContextValid_ = true;
  recoveryDirectionStop_ = false;
  recoveryMotionDirection_ = lastMotionCommand_.linear.x > 1e-9 ? 1 :
                              lastMotionCommand_.linear.x < -1e-9 ? -1 : 0;
  poseRecoveryPhase_ = "finish_turn";
  pathProgressSupervisor_.Pause();
  turnProgressSupervisor_.Pause();
  SetState("POSE_RECOVERY_FINISH_CURRENT_TURN");
  PublishPoseRecovery("finish_turn");
  return true;
}

bool WaypointNavigator::CheckReverseTurnPlan() const {
  using Recovery = visual_navigation::PoseRecovery;
  visual_navigation::TurnManeuverController controller(recoveryTurnConfig_);
  Recovery::Pose actual = RecoveryPose();
  const double dt = 1.0 / controlFrequency_;
  const double maximum = 5.0 * recoveryTurnConfig_.phase_timeout;
  controller.begin(0.0);
  double v = 0.0, w = 0.0;
  for (double t = 0.0; t <= maximum; t += dt) {
    auto context = turnObstacleReturnContext_;
    context.pose = {actual.position.x, actual.position.y,
                    NormalizeAngle(actual.yaw + kPi)};
    context.applied_twist = {-v, w};
    context.measured_twist = context.applied_twist;
    visual_navigation::TurnManeuverInput input;
    input.context = context;
    const auto result = controller.update(input, t);
    if (result.result == visual_navigation::TurnManeuverResult::COMPLETED)
      return poseRecovery_.safe({actual});
    if (result.result != visual_navigation::TurnManeuverResult::RUNNING)
      return false;
    const double target = std::clamp(-result.command.linear_x,
                                    -maxLinearSpeed_, maxLinearSpeed_);
    // A return-to-lane action may only drive backwards. A fixed controller
    // asking for forward entry correction requires a different safe entry.
    if (target > 1.0e-9) return false;
    v = visual_navigation::LimitRate(target, v, maxLinearAcceleration_,
                                     maxLinearDeceleration_, dt);
    if (std::abs(target) > 1.0e-9 && std::abs(v) < minLinearCommandSpeed_)
      v = std::copysign(minLinearCommandSpeed_, target);
    const double target_w = std::clamp(result.command.angular_z,
                                      -maxAngularSpeed_, maxAngularSpeed_);
    w = visual_navigation::LimitRate(target_w, w, maxAngularAcceleration_,
                                     maxAngularDeceleration_, dt);
    if (std::abs(result.command.linear_x) <= 1.0e-9 &&
        std::abs(result.command.angular_z) <= 1.0e-9) {
      v = 0.0;
      w = 0.0;
    }
    const auto sweep = poseRecovery_.motion_samples(actual, v, w, w, dt);
    if (!poseRecovery_.safe(sweep)) return false;
    actual = sweep.back();
  }
  return false;
}

void WaypointNavigator::FinishTurnObstacleRecovery() {
  // This ends only the corner prelude. It does not report recovery complete
  // or mark a route edge, RFID checkpoint or mission layer as traversed.
  recoveryTurnController_.reset();
  recoveryTurnManeuverStarted_ = false;
  turnManeuverController_.reset();
  stoppedAtPlannedTurn_ = false;
  stoppedTurnReverseArrival_ = false;
  controlState_.interrupt_maneuver();
  poseRecoveryPhase_ = "braking";
  BeginRecoveryStop(poseRecovery_.active_seconds(TurnWallSeconds()));
  PublishStop();
  SetState("POSE_RECOVERY_ORIGINAL_LANE_STOP");
  PublishPoseRecovery("braking");
}

bool WaypointNavigator::RunTurnObstacleRecovery() {
  const bool owned = poseRecoveryPhase_ == "finish_turn" ||
      poseRecoveryPhase_ == "turn_stop" ||
      poseRecoveryPhase_ == "reverse_turn" ||
      poseRecoveryPhase_ == "return_straight" ||
      poseRecoveryPhase_ == "return_stop";
  if (!owned) return false;
  const double t = poseRecovery_.active_seconds(TurnWallSeconds());
  if (recoveryDirectionStop_) {
    if (!RecoveryStopCompleted(t)) return true;
    recoveryDirectionStop_ = false;
  }
  const auto fail = [this](const char *reason) {
    FailPoseRecovery(reason);
  };
  const auto live_context = [this](visual_navigation::CornerShuntContext saved,
                                  bool reverse) {
    const double c = std::cos(currentYaw_), s = std::sin(currentYaw_);
    saved.pose = {
        currentX_ - c * trackingPointOffsetX_ + s * trackingPointOffsetY_,
        currentY_ - s * trackingPointOffsetX_ - c * trackingPointOffsetY_,
        NormalizeAngle(currentYaw_ + (reverse ? kPi : 0.0))};
    const double sign = reverse ? -1.0 : 1.0;
    saved.applied_twist = {sign * lastMotionCommand_.linear.x,
                          lastMotionCommand_.angular.z};
    saved.measured_twist = {
        sign * (inputCache_.odometry_velocity_valid() ?
            inputCache_.odometry_velocity() : lastMotionCommand_.linear.x),
        ControlYawRate()};
    saved.measured_yaw_rate_valid = ImuYawRateIsFresh();
    return saved;
  };
  if (poseRecoveryPhase_ == "finish_turn" ||
      poseRecoveryPhase_ == "reverse_turn") {
    const bool reverse = poseRecoveryPhase_ == "reverse_turn";
    visual_navigation::TurnManeuverInput input;
    input.context = live_context(reverse ? turnObstacleReturnContext_ :
                                         turnObstacleContext_, reverse);
    const auto decision = reverse ? recoveryTurnController_.update(input, t) :
        turnManeuverController_.update(input, TurnWallSeconds());
    if (decision.result == visual_navigation::TurnManeuverResult::COMPLETED) {
      const auto actual = RecoveryPose();
      const double heading = reverse ? turnObstacleContext_.geometry_heading :
                                       turnObstacleContext_.target_heading;
      const double tolerance = reverse ? recoveryTurnConfig_.entry_yaw_tolerance :
                                         turnManeuverConfig_.entry_yaw_tolerance;
      const double outgoing_projection =
          (actual.position.x - turnObstacleContext_.junction_x) * std::cos(heading) +
          (actual.position.y - turnObstacleContext_.junction_y) * std::sin(heading);
      if (!poseRecovery_.safe({actual}) ||
          std::abs(NormalizeAngle(actual.yaw - heading)) > tolerance ||
          (!reverse && outgoing_projection <= 0.0)) {
        fail(reverse ? "FAULT_POSE_RECOVERY_RETURN_TURN_POSE" :
                       "FAULT_POSE_RECOVERY_CURRENT_TURN_POSE");
        return true;
      }
      PublishStop();
      poseRecoveryPhase_ = reverse ? "return_straight" : "turn_stop";
      recoveryStepStarted_ = t;
      BeginRecoveryStop(t);
      SetState(reverse ? "POSE_RECOVERY_RETURN_TURN_STOP" :
                         "POSE_RECOVERY_CURRENT_TURN_STOP");
      PublishPoseRecovery(poseRecoveryPhase_);
      return true;
    }
    if (decision.result != visual_navigation::TurnManeuverResult::RUNNING) {
      fail(reverse ? "FAULT_POSE_RECOVERY_RETURN_TURN" :
                     "FAULT_POSE_RECOVERY_CURRENT_TURN");
      return true;
    }
    geometry_msgs::msg::Twist command;
    command.linear.x = (reverse ? -1.0 : 1.0) * decision.command.linear_x;
    command.angular.z = decision.command.angular_z;
    if (reverse && command.linear.x > 1.0e-9) {
      fail("FAULT_POSE_RECOVERY_RETURN_ENTRY_DIRECTION");
      return true;
    }
    const int direction = command.linear.x > 1e-9 ? 1 : command.linear.x < -1e-9 ? -1 : 0;
    if (direction && recoveryMotionDirection_ && direction != recoveryMotionDirection_) {
      recoveryMotionDirection_ = direction;
      recoveryDirectionStop_ = true;
      BeginRecoveryStop(t);
      SetState("POSE_RECOVERY_DIRECTION_STOP");
      return true;
    }
    if (direction) recoveryMotionDirection_ = direction;
    if (!RecoveryCommandSafe(command)) {
      fail(reverse ? "FAULT_POSE_RECOVERY_RETURN_TURN_SWEEP" :
                     "FAULT_POSE_RECOVERY_CURRENT_TURN_SWEEP");
      return true;
    }
    if (std::abs(command.linear.x) <= 1.0e-9 &&
        std::abs(command.angular.z) <= 1.0e-9) PublishStop();
    else PublishMotionCommand(command);
    SetState(std::string(reverse ? "POSE_RECOVERY_REVERSE_" :
                                  "POSE_RECOVERY_FINISH_") +
             visual_navigation::TurnManeuverStatus(decision.phase));
    return true;
  }
  if (poseRecoveryPhase_ == "turn_stop") {
    PublishStop();
    if (!RecoveryStopCompleted(t)) return true;
    turnObstacleReturnContext_ = live_context(turnObstacleContext_, true);
    turnObstacleReturnContext_.geometry_heading =
        NormalizeAngle(turnObstacleContext_.target_heading + kPi);
    turnObstacleReturnContext_.arrival_heading =
        NormalizeAngle(turnObstacleContext_.target_heading + kPi);
    turnObstacleReturnContext_.target_heading =
        NormalizeAngle(turnObstacleContext_.geometry_heading + kPi);
    turnObstacleReturnContext_.reverse_arrival = true;
    if (!CheckReverseTurnPlan()) {
      fail("FAULT_POSE_RECOVERY_RETURN_TURN_PLAN");
      return true;
    }
    recoveryTurnController_.begin(t);
    recoveryMotionDirection_ = 0;
    poseRecoveryPhase_ = "reverse_turn";
    SetState("POSE_RECOVERY_REVERSE_CURRENT_TURN");
    PublishPoseRecovery("reverse_turn");
    return true;
  }
  if (poseRecoveryPhase_ == "return_straight") {
    if (recoveryStopPending_) {
      PublishStop();
      if (!RecoveryStopCompleted(t)) return true;
      recoveryStepStarted_ = t;
    }
    const auto actual = live_context(turnObstacleContext_, false).pose;
    const double heading = turnObstacleContext_.geometry_heading;
    const double dx = actual.x - turnObstacleContext_.junction_x;
    const double dy = actual.y - turnObstacleContext_.junction_y;
    const double longitudinal = std::cos(heading) * dx + std::sin(heading) * dy;
    const double lateral = -std::sin(heading) * dx + std::cos(heading) * dy;
    const double yaw_error = NormalizeAngle(heading - currentYaw_);
    if (std::abs(lateral) > recoveryTurnConfig_.exit_lateral_tolerance ||
        std::abs(yaw_error) > recoveryTurnConfig_.entry_yaw_tolerance) {
      fail("FAULT_POSE_RECOVERY_ORIGINAL_LANE");
      return true;
    }
    const double target = turnManeuverConfig_.entry_target_x;
    if (longitudinal <= target + recoveryTurnConfig_.entry_position_tolerance) {
      poseRecoveryPhase_ = "return_stop";
      BeginRecoveryStop(t);
      return true;
    }
    if (t - recoveryStepStarted_ >
        recoveryTurnConfig_.phase_timeout) {
      fail("FAULT_POSE_RECOVERY_ORIGINAL_LANE_TIMEOUT");
      return true;
    }
    geometry_msgs::msg::Twist command;
    command.linear.x = -poseRecovery_.config().speed;
    command.angular.z = std::clamp(
        recoveryTurnConfig_.entry_heading_gain * yaw_error +
            recoveryTurnConfig_.entry_lateral_gain * lateral,
        -recoveryTurnConfig_.entry_max_angular_speed,
        recoveryTurnConfig_.entry_max_angular_speed);
    if (!RecoveryCommandSafe(command)) {
      fail("FAULT_POSE_RECOVERY_ORIGINAL_LANE_SWEEP");
      return true;
    }
    PublishMotionCommand(command);
    SetState("POSE_RECOVERY_RETREAT_ON_ORIGINAL_LANE");
    return true;
  }
  PublishStop();
  if (RecoveryStopCompleted(t)) {
    const auto actual = RecoveryPose();
    const double heading = turnObstacleContext_.geometry_heading;
    const double dx = actual.position.x - turnObstacleContext_.junction_x;
    const double dy = actual.position.y - turnObstacleContext_.junction_y;
    const double longitudinal = std::cos(heading) * dx + std::sin(heading) * dy;
    const double lateral = -std::sin(heading) * dx + std::cos(heading) * dy;
    if (!poseRecovery_.safe({actual}) ||
        std::abs(NormalizeAngle(actual.yaw - heading)) >
            recoveryTurnConfig_.entry_yaw_tolerance ||
        std::abs(lateral) > recoveryTurnConfig_.exit_lateral_tolerance ||
        longitudinal > turnManeuverConfig_.entry_target_x +
                           recoveryTurnConfig_.entry_position_tolerance) {
      fail("FAULT_POSE_RECOVERY_ORIGINAL_LANE_STOP_POSE");
      return true;
    }
    FinishTurnObstacleRecovery();
  }
  return true;
}
