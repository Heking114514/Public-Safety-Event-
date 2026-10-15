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
  if (!turnManeuverController_.active() && lastCornerContextValid_ &&
      poseRecovery_.lane_at_pose(RecoveryPose()) == lastCornerExitLane_ &&
      std::abs(NormalizeAngle(currentYaw_-lastCornerContext_.target_heading)) <=
        turnManeuverConfig_.entry_yaw_tolerance) {
    normalTurnEntryStop_ = false;
    normalTurnEntryConfirmed_ = false;
    turnObstacleContext_ = lastCornerContext_;
    turnObstacleContextValid_ = true;
    poseRecoveryPhase_ = "turn_stop";
    recoveryMotionDirection_ = 0;
    BeginRecoveryStop(0.0);
    SetState("POSE_RECOVERY_CURRENT_TURN_STOP");
    PublishPoseRecovery("turn_stop");
    return true;
  }
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
  if (normalTurnEntryStop_) BeginRecoveryStop(0.0);
  pathProgressSupervisor_.Pause();
  turnProgressSupervisor_.Pause();
  SetState("POSE_RECOVERY_FINISH_CURRENT_TURN");
  PublishPoseRecovery("finish_turn");
  return true;
}

bool WaypointNavigator::CheckReverseTurnPlan() const {
  visual_navigation::CornerReturnController controller(cornerReturnConfig_);
  auto context = turnObstacleContext_;
  auto actual = RecoveryPose();
  context.pose = {actual.position.x, actual.position.y, actual.yaw};
  controller.begin(context, 0.0);
  double v = 0.0, w = 0.0;
  const double dt = cornerReturnConfig_.control_dt;
  for (double t = 0.0; t < 6 * cornerReturnConfig_.turn.phase_timeout; t += dt) {
    visual_navigation::CornerReturnInput input;
    input.context = context;
    input.context.pose = {actual.position.x, actual.position.y, actual.yaw};
    const auto phase = controller.phase();
    const auto decision = controller.update(input, t);
    if (decision.result == visual_navigation::TurnManeuverResult::COMPLETED)
      return poseRecovery_.safe({actual});
    if (decision.result != visual_navigation::TurnManeuverResult::RUNNING ||
        decision.command.linear_x > 1e-9) {
      const auto entry = controller.entrance_pose();
      const double dx = actual.position.x-entry.x, dy = actual.position.y-entry.y;
      RCLCPP_WARN(get_logger(),
        "L reverse preflight rejected: phase=%s result=%d t=%.3f pose=(%.6f,%.6f,%.6f) entry_error=(%.6f,%.6f,%.6f)",
        visual_navigation::CornerReturnStatus(phase), static_cast<int>(decision.result), t,
        actual.position.x, actual.position.y, actual.yaw,
        dx*std::cos(entry.yaw)+dy*std::sin(entry.yaw),
        -dx*std::sin(entry.yaw)+dy*std::cos(entry.yaw), NormalizeAngle(actual.yaw-entry.yaw));
      return false;
    }
    if (std::abs(decision.command.linear_x) <= 1e-9 &&
        std::abs(decision.command.angular_z) <= 1e-9) v = w = 0.0;
    else {
      v = visual_navigation::LimitRate(decision.command.linear_x, v,
        maxLinearAcceleration_, maxLinearDeceleration_, dt);
      if (std::abs(v) < minLinearCommandSpeed_)
        v = std::copysign(minLinearCommandSpeed_, decision.command.linear_x);
      w = visual_navigation::LimitRate(decision.command.angular_z, w,
        maxAngularAcceleration_, maxAngularDeceleration_, dt);
    }
    const auto sweep = poseRecovery_.motion_samples(actual, v, w, w, dt);
    if (!poseRecovery_.safe(sweep)) {
      RCLCPP_WARN(get_logger(),
        "L reverse preflight sweep unsafe: phase=%s t=%.3f pose=(%.6f,%.6f,%.6f) command=(%.4f,%.4f)",
        visual_navigation::CornerReturnStatus(phase), t, actual.position.x, actual.position.y,
        actual.yaw, v, w);
      return false;
    }
    actual = sweep.back();
  }
  RCLCPP_WARN(get_logger(), "L reverse preflight exceeded complete action timeout");
  return false;
}

void WaypointNavigator::FinishTurnObstacleRecovery() {
  // This ends only the corner prelude. It does not report recovery complete
  // or mark a route edge, RFID checkpoint or mission layer as traversed.
  recoveryTurnController_.reset();
  recoveryTurnManeuverStarted_ = false;
  turnManeuverController_.reset();
  normalTurnEntryStop_ = false;
  normalTurnEntryConfirmed_ = false;
  lastCornerContextValid_ = false;
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
    poseRecoveryPhase_ == "turn_stop" || poseRecoveryPhase_ == "post_turn_point" ||
    poseRecoveryPhase_ == "post_turn_stop" || poseRecoveryPhase_ == "reverse_turn" ||
    poseRecoveryPhase_ == "return_stop";
  if (!owned) return false;
  const double t = poseRecovery_.active_seconds(TurnWallSeconds());
  const auto context = [this]() {
    auto saved = turnObstacleContext_;
    const auto p = RecoveryPose();
    saved.pose = {p.position.x, p.position.y, p.yaw};
    saved.applied_twist = {lastMotionCommand_.linear.x, lastMotionCommand_.angular.z};
    saved.measured_twist = {inputCache_.odometry_velocity(), ControlYawRate()};
    saved.measured_yaw_rate_valid = ImuYawRateIsFresh();
    return saved;
  };
  const auto publish = [this, t](const visual_navigation::CornerShuntTwist &twist,
                                const std::string &state) {
    geometry_msgs::msg::Twist command;
    command.linear.x = twist.linear_x; command.angular.z = twist.angular_z;
    const int direction = command.linear.x > 1e-9 ? 1 : command.linear.x < -1e-9 ? -1 : 0;
    if (direction && recoveryMotionDirection_ && direction != recoveryMotionDirection_) {
      recoveryMotionDirection_ = direction;
      recoveryDirectionStop_ = true;
      BeginRecoveryStop(t);
      SetState("POSE_RECOVERY_DIRECTION_STOP");
      return;
    }
    if (direction) recoveryMotionDirection_ = direction;
    if (!RecoveryCommandSafe(command)) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_CORNER_SWEEP"); return;
    }
    if (std::abs(command.linear.x) <= 1e-9 && std::abs(command.angular.z) <= 1e-9)
      PublishStop();
    else PublishMotionCommand(command);
    SetState(state);
  };
  if (recoveryDirectionStop_) {
    if (!RecoveryStopCompleted(t)) return true;
    recoveryDirectionStop_ = false;
  }
  if (poseRecoveryPhase_ == "finish_turn") {
    if (normalTurnEntryStop_) {
      PauseActiveTurn();
      if (!RecoveryStopCompleted(t)) return true;
      normalTurnEntryStop_ = false;
      if (!visual_navigation::FixedTurnEntryMatches(context(), turnManeuverConfig_)) {
        FailPoseRecovery("FAULT_POSE_RECOVERY_CURRENT_ENTRY_POSE"); return true;
      }
    }
    visual_navigation::TurnManeuverInput input; input.context = context();
    const auto decision = turnManeuverController_.update(input, TurnWallSeconds());
    if (!normalTurnEntryConfirmed_ &&
        decision.phase == visual_navigation::TurnManeuverPhase::TURN_60 &&
        std::abs(decision.command.linear_x) <= 1e-9 &&
        std::abs(decision.command.angular_z) <= 1e-9) {
      normalTurnEntryStop_ = normalTurnEntryConfirmed_ = true;
      BeginRecoveryStop(t);
      SetState("POSE_RECOVERY_CURRENT_ENTRY_STOP");
      return true;
    }
    if (decision.result == visual_navigation::TurnManeuverResult::COMPLETED) {
      PublishStop(); poseRecoveryPhase_ = "turn_stop"; BeginRecoveryStop(t);
      SetState("POSE_RECOVERY_CURRENT_TURN_STOP"); PublishPoseRecovery("turn_stop");
    } else if (decision.result != visual_navigation::TurnManeuverResult::RUNNING)
      FailPoseRecovery("FAULT_POSE_RECOVERY_CURRENT_TURN");
    else publish(decision.command, std::string("POSE_RECOVERY_FINISH_") +
      visual_navigation::TurnManeuverStatus(decision.phase));
    return true;
  }
  if (poseRecoveryPhase_ == "turn_stop") {
    if (!RecoveryStopCompleted(t)) return true;
    cornerReturnController_.begin(context(), t);
    if (!cornerReturnController_.geometry_valid()) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_RETURN_GEOMETRY"); return true;
    }
    poseRecoveryPhase_ = "post_turn_point";
    recoveryStepStarted_ = t;
    const auto target = cornerReturnController_.exit_pose();
    const auto actual = context().pose;
    recoveryBestError_ = std::hypot(target.x-actual.x,target.y-actual.y);
    RCLCPP_INFO(get_logger(), "L return fixed outgoing point: target=(%.4f,%.4f,%.4f)",
      target.x, target.y, target.yaw);
    PublishPoseRecovery("post_turn_point");
  }
  if (poseRecoveryPhase_ == "post_turn_point") {
    const auto target = cornerReturnController_.exit_pose();
    const auto actual = context().pose;
    const double c = std::cos(target.yaw), s = std::sin(target.yaw);
    const double along = (target.x-actual.x)*c + (target.y-actual.y)*s;
    const double cross = -(actual.x-target.x)*s + (actual.y-target.y)*c;
    const double yaw = NormalizeAngle(target.yaw-actual.yaw);
    if (std::abs(along) <= turnManeuverConfig_.entry_position_tolerance &&
        std::abs(cross) <= turnManeuverConfig_.entry_lateral_tolerance &&
        std::abs(yaw) <= turnManeuverConfig_.entry_yaw_tolerance) {
      PublishStop(); poseRecoveryPhase_ = "post_turn_stop"; BeginRecoveryStop(t);
      SetState("POSE_RECOVERY_POST_TURN_POINT_STOP"); PublishPoseRecovery("post_turn_stop");
      return true;
    }
    if (t-recoveryStepStarted_ > recoveryBestError_/poseRecovery_.config().speed +
        turnManeuverConfig_.phase_timeout) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_POST_TURN_POINT_TIMEOUT"); return true;
    }
    const double direction = along >= 0 ? 1.0 : -1.0;
    visual_navigation::CornerShuntTwist command;
    command.linear_x = direction*poseRecovery_.config().speed;
    command.angular_z = std::clamp(turnManeuverConfig_.entry_heading_gain*yaw -
      direction*turnManeuverConfig_.entry_lateral_gain*cross,
      -turnManeuverConfig_.entry_max_angular_speed, turnManeuverConfig_.entry_max_angular_speed);
    publish(command, "POSE_RECOVERY_POST_TURN_POINT"); return true;
  }
  if (poseRecoveryPhase_ == "post_turn_stop") {
    if (!RecoveryStopCompleted(t)) return true;
    if (!CheckReverseTurnPlan()) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_RETURN_TURN_PLAN"); return true;
    }
    const auto actual = context().pose;
    const auto target = cornerReturnController_.exit_pose();
    if (std::hypot(actual.x-target.x,actual.y-target.y) >
        std::hypot(turnManeuverConfig_.entry_position_tolerance,
                   turnManeuverConfig_.entry_lateral_tolerance)) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_POST_TURN_STOP_POSE"); return true;
    }
    RCLCPP_INFO(get_logger(), "L outgoing point stopped: actual=(%.4f,%.4f,%.4f)",
      actual.x,actual.y,actual.yaw);
    cornerReturnController_.begin(context(), t);
    recoveryMotionDirection_ = 0; poseRecoveryPhase_ = "reverse_turn";
    PublishPoseRecovery("reverse_turn");
  }
  if (poseRecoveryPhase_ == "reverse_turn") {
    visual_navigation::CornerReturnInput input; input.context = context();
    const auto decision = cornerReturnController_.update(input, t);
    if (decision.result == visual_navigation::TurnManeuverResult::COMPLETED) {
      PublishStop(); poseRecoveryPhase_ = "return_stop"; BeginRecoveryStop(t);
      SetState("POSE_RECOVERY_RETURN_TURN_STOP"); PublishPoseRecovery("return_stop");
    } else if (decision.result != visual_navigation::TurnManeuverResult::RUNNING ||
               decision.command.linear_x > 1e-9)
      FailPoseRecovery("FAULT_POSE_RECOVERY_RETURN_TURN");
    else publish(decision.command, std::string("POSE_RECOVERY_REVERSE_") +
      visual_navigation::CornerReturnStatus(decision.phase));
    return true;
  }
  if (poseRecoveryPhase_ == "return_stop" && RecoveryStopCompleted(t)) {
    const auto actual = context().pose;
    const auto target = cornerReturnController_.entrance_pose();
    const double dx = actual.x-target.x, dy = actual.y-target.y;
    const double c = std::cos(target.yaw), s = std::sin(target.yaw);
    if (!poseRecovery_.safe({RecoveryPose()}) ||
        std::abs(dx*c+dy*s) > turnManeuverConfig_.entry_position_tolerance ||
        std::abs(-dx*s+dy*c) > turnManeuverConfig_.entry_lateral_tolerance ||
        std::abs(NormalizeAngle(actual.yaw-target.yaw)) > turnManeuverConfig_.entry_yaw_tolerance) {
      FailPoseRecovery("FAULT_POSE_RECOVERY_ORIGINAL_LANE_STOP_POSE"); return true;
    }
    FinishTurnObstacleRecovery();
  }
  return true;
}
