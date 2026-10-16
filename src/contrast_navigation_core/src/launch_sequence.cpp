#include "visual_navigation/waypoint_navigator.hpp"

void WaypointNavigator::InitLaunchSequence() {
  launchSequenceEnabled_ = declare_parameter<bool>("launch_sequence_enabled", false);
  if (!launchSequenceEnabled_) return;
  launchOrigin_ = declare_parameter<std::vector<double>>("launch_origin", std::vector<double>{});
  const auto required = [this](const char *name) {
    return declare_parameter<double>(name, std::numeric_limits<double>::quiet_NaN());
  };
  launchProbeDistance_ = required("launch_probe_distance_m");
  launchPositionTolerance_ = required("launch_position_tolerance_m");
  launchOriginTolerance_ = required("launch_origin_tolerance_m");
  launchPhaseTimeout_ = required("launch_phase_timeout_s");
  launchSpeed_ = required("launch_speed_mps");
  const double limits[] = {launchProbeDistance_, launchPositionTolerance_,
    launchOriginTolerance_, launchPhaseTimeout_, launchSpeed_};
  if (!poseRecoveryEnabled_ || launchOrigin_.size() != 3 ||
      std::any_of(launchOrigin_.begin(), launchOrigin_.end(),
        [](double x) { return !std::isfinite(x); }) ||
      std::any_of(std::begin(limits), std::end(limits),
        [](double x) { return !std::isfinite(x) || x <= 0.0; }) ||
      launchPositionTolerance_ >= launchProbeDistance_ ||
      launchPositionTolerance_ > turnManeuverConfig_.entry_position_tolerance ||
      launchSpeed_ < minLinearCommandSpeed_ || launchSpeed_ > maxLinearSpeed_)
    throw std::invalid_argument("invalid launch sequence parameters");
  launchStopGuard_.configure(get_parameter("recovery_stop_speed_mps").as_double(),
    get_parameter("recovery_stop_yaw_rate_rps").as_double(),
    get_parameter("recovery_switch_stop_s").as_double(),
    get_parameter("recovery_stop_timeout_s").as_double(),
    static_cast<unsigned>(preTurnStopTelemetrySamples_));
}

bool WaypointNavigator::RunLaunchSequence() {
  if (!launchSequenceEnabled_) return false;
  using Phase = visual_navigation::LaunchSequence::Phase;
  const auto pose = RecoveryPose();
  const visual_navigation::CornerShuntPose actual{
    pose.position.x, pose.position.y, pose.yaw};
  if (launchRouteId_ != activeRouteId_) {
    launchRouteId_ = activeRouteId_;
    launchSequence_ = {};
    launchStopArmed_ = false;
    // 只接管从配置原点出发、首个路口前仍是同一直路的正常路线。
    if (currentWaypointIndex_ != 0 || waypoints_.size() < 3 ||
        std::hypot(actual.x-launchOrigin_[0], actual.y-launchOrigin_[1]) > launchOriginTolerance_ ||
        std::hypot(waypoints_.front().x-launchOrigin_[0],
                   waypoints_.front().y-launchOrigin_[1]) > launchOriginTolerance_ ||
        std::abs(NormalizeAngle(actual.yaw-launchOrigin_[2])) > turnManeuverConfig_.entry_yaw_tolerance)
      return false;
    const double c = std::cos(launchOrigin_[2]), s = std::sin(launchOrigin_[2]);
    for (std::size_t i=1; i+1<waypoints_.size(); ++i) {
      const auto &point = waypoints_[i];
      const double dx=point.x-launchOrigin_[0], dy=point.y-launchOrigin_[1];
      if (point.rfid_supernode || point.reverse_arrival_turn ||
          point.stop_time > 0.0 || std::abs(-s*dx+c*dy) > launchPositionTolerance_)
        return false;
      if (!point.turn_junction) continue;
      const double next = std::atan2(waypoints_[i+1].y-point.y,
                                     waypoints_[i+1].x-point.x);
      if (std::abs(std::abs(NormalizeAngle(next-launchOrigin_[2]))-kPi/2) >
            turnManeuverConfig_.entry_yaw_tolerance) return false;
      const double entry=c*dx+s*dy+turnManeuverConfig_.entry_target_x;
      // 首次前出可以越过入弯点，返回原点后再单独前进入位；前出仍不越过路口。
      if (entry <= 0.0 || entry >= c*dx+s*dy || launchProbeDistance_ >= c*dx+s*dy)
        return false;
      launchTurnIndex_=i;
      launchSequence_.begin({launchOrigin_[0],launchOrigin_[1],launchOrigin_[2]},
        launchProbeDistance_,entry,launchPositionTolerance_,launchPhaseTimeout_);
      RCLCPP_INFO(get_logger(), "Launch sequence: probe=%.3fm origin=(%.3f,%.3f) entry=(%.3f,%.3f) junction=%zu",
        launchProbeDistance_,launchOrigin_[0],launchOrigin_[1],
        launchOrigin_[0]+entry*c,launchOrigin_[1]+entry*s,i);
      break;
    }
  }
  if (!launchSequence_.active()) return false;
  const auto fail = [this](const char *state) {
    PublishStop();
    navigationActive_=false;
    routeReplacementRequiredForId_=activeRouteId_;
    SetState(state);
  };
  pathProgressSupervisor_.Reset();
  turnProgressSupervisor_.Reset();
  double dt=std::chrono::duration<double>(ControlNow()-lastMotionCommandTime_).count();
  dt=std::clamp(dt,0.0,0.2);
  bool stopped=false;
  if (launchSequence_.stopping()) {
    PublishStop();
    if (!launchStopArmed_) {
      launchStopGuard_.begin(launchSequence_.time(),controlTelemetrySampleCount_);
      launchStopArmed_=true;
    }
    const double age=std::chrono::duration<double>(ControlNow()-lastControlTelemetryArrival_).count();
    const auto result=launchStopGuard_.update(launchSequence_.time(),controlTelemetrySampleCount_,
      controlTelemetryReceived_ && age<=controlTelemetryTimeout_ && ImuYawRateIsFresh(),
      measuredLeftWheelSpeed_,measuredRightWheelSpeed_,targetLeftWheelSpeed_,
      targetRightWheelSpeed_,ControlYawRate());
    stopped=result==visual_navigation::RecoveryStopGuard::Result::STOPPED;
    if (result==visual_navigation::RecoveryStopGuard::Result::TIMED_OUT) {
      fail("FAULT_LAUNCH_STOP_TIMEOUT");
      return true;
    }
  }
  const auto previous=launchSequence_.phase();
  launchSequence_.update(actual,stopped,dt);
  if (previous!=launchSequence_.phase()) {
    launchStopArmed_=false;
    PublishStop();
    if (stopped)
      RCLCPP_INFO(get_logger(), "Launch stopped: phase=%u actual=(%.4f,%.4f,%.4f)",
        static_cast<unsigned>(previous),actual.x,actual.y,actual.yaw);
  }
  if (launchSequence_.phase()==Phase::FAULT) {
    fail("FAULT_LAUNCH_TIMEOUT"); return true;
  }
  if (launchSequence_.phase()==Phase::COMPLETE) {
    // 停车滑移超出原转弯入口容差时驻停，不能再次倒车补位。
    if (std::abs(launchSequence_.remaining(actual)) > turnManeuverConfig_.entry_position_tolerance) {
      fail("FAULT_LAUNCH_ENTRY_POSITION"); return true;
    }
    // 跳过首路口的中心停车目标，但保留原路口与后继点给固定转弯几何使用。
    ResetManeuver();
    controlState_.reset_segment();
    currentWaypointIndex_=launchTurnIndex_+1;
    lastStoppedTurnIndex_=launchTurnIndex_;
    stoppedAtPlannedTurn_=true;
    firstHalfTurnPending_=false;
    reverseSegmentActive_=false;
    stoppedTurnReverseArrival_=false;
    stoppedTurnGeometryHeading_=launchSequence_.heading();
    stoppedTurnArrivalHeading_=currentYaw_;
    const auto &junction=waypoints_[launchTurnIndex_];
    const auto &next=waypoints_[currentWaypointIndex_];
    stoppedTurnTargetHeading_=std::atan2(next.y-junction.y,next.x-junction.x);
    routeStartTurnAuthorized_=false;
    pathSegmentInitialized_=false;
    PublishCurrentWaypoint();
    SetState("LAUNCH_READY_TO_TURN");
    return true;
  }
  static const char *states[]={"LAUNCH_IDLE","LAUNCH_INITIAL_STOP","LAUNCH_FORWARD",
    "LAUNCH_FORWARD_STOP","LAUNCH_RETURN","LAUNCH_ORIGIN_STOP","LAUNCH_APPROACH_ENTRY",
    "LAUNCH_ENTRY_STOP","LAUNCH_COMPLETE","FAULT_LAUNCH_TIMEOUT"};
  SetState(states[static_cast<unsigned>(launchSequence_.phase())]);
  if (launchSequence_.stopping() || previous!=launchSequence_.phase()) return true;
  const double yawError=NormalizeAngle(launchSequence_.heading()-currentYaw_);
  geometry_msgs::msg::Twist command;
  command.linear.x=launchSequence_.direction()*launchSpeed_;
  command.angular.z=Clamp(turnManeuverConfig_.entry_heading_gain*yawError,
    -turnManeuverConfig_.entry_max_angular_speed,turnManeuverConfig_.entry_max_angular_speed);
  if (!RecoveryCommandSafe(command)) {
    fail("FAULT_LAUNCH_SWEEP"); return true;
  }
  PublishMotionCommand(command);
  return true;
}
