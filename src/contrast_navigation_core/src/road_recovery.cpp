#include "visual_navigation/waypoint_navigator.hpp"

void WaypointNavigator::HandleRoadRecovery(
    const mission_control_interfaces::srv::StartRoadRecovery::Request::SharedPtr request,
    mission_control_interfaces::srv::StartRoadRecovery::Response::SharedPtr response) {
  // 已完成路线的令牌、防止旧请求在新路线或人工驻停期间启动倒车。
  if (!poseRecoveryEnabled_ || request->route_id==0 || request->route_id!=activeRouteId_ ||
      completedRoadRecoveryRoute_!=request->route_id || navigationActive_ || MotionHeld() ||
      !poseRecoveryPhase_.empty() || request->obstacle_revision==0 ||
      turnManeuverController_.active()) {
    response->message="road recovery requires the current completed route and released motion hold";
    return;
  }
  std::vector<navigation_safety::Polygon> obstacles;
  for (const auto &source:request->obstacles) {
    navigation_safety::Polygon polygon;
    if (source.points.size()<3) { response->message="invalid obstacle polygon"; return; }
    for (const auto &p:source.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        response->message="nonfinite obstacle polygon"; return;
      }
      polygon.points.push_back({p.x,p.y});
    }
    obstacles.push_back(polygon);
  }
  const auto selected=poseRecovery_.forward_obstacle(RecoveryPose(),obstacles,
    request->blocked_lane_ids);
  if (selected<0) {
    response->message="no confirmed blocked road on the current forward approach";
    return;
  }
  poseRecovery_.set_incoming_lane("",{});
  poseRecovery_.set_dynamic_obstacles(obstacles);
  if (!poseRecovery_.safe({RecoveryPose()})) {
    response->message="current footprint is unsafe";
    return;
  }
  recoveryObservedObstacles_=obstacles;
  poseRecoveryObstacle_=obstacles[selected];
  recoveryKnownRoad_=true;
  recoveryRangeOnly_=false;
  recoveryNewHazard_=false;
  turnObstacleContextValid_=false;
  pendingFrontObstacle_=pendingObstacleAtActiveTurn_=false;
  ++poseRecoveryId_;
  completedRoadRecoveryRoute_=0;
  poseRecoveryRevision_=request->obstacle_revision;
  const double wall=ControlSeconds();
  poseRecovery_.reset_clock(wall);
  navigationActive_=true;
  if (BeginTurnObstacleRecovery()) {
    PublishPoseRecovery("braking");
    response->accepted=true;
    response->recovery_id=poseRecoveryId_;
    response->message="known blockage: returning from L before road retreat";
    return;
  }
  ResetManeuver();
  recoveryTurnController_.reset();
  recoveryTurnManeuverStarted_=false;
  recoveryCandidates_.clear();
  poseRecoveryPhase_="braking";
  BeginRecoveryStop(0.0);
  SetState("POSE_RECOVERY_BRAKING");
  PublishPoseRecovery("braking");
  response->accepted=true;
  response->recovery_id=poseRecoveryId_;
  response->message="confirmed blocked-road recovery; stop, retreat to T/X and fixed turn";
  RCLCPP_INFO(get_logger(),"Known blocked-road recovery: route=%lu revision=%lu obstacles=%zu",
    static_cast<unsigned long>(activeRouteId_),
    static_cast<unsigned long>(poseRecoveryRevision_),obstacles.size());
}
