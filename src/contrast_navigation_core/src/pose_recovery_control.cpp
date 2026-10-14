#include "visual_navigation/waypoint_navigator.hpp"

#include <stdexcept>

namespace {
double recovery_seconds() {
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
bool same_obstacle(const navigation_safety::Polygon &a,
                   const navigation_safety::Polygon &b) {
  if (a.points.size() != b.points.size()) return false;
  for (const auto &point : a.points) {
    const auto found = std::find_if(b.points.begin(), b.points.end(), [&](const auto &other) {
      return std::hypot(point.x - other.x, point.y - other.y) < 0.001;
    });
    if (found == b.points.end()) return false;
  }
  return true;
}
}

void WaypointNavigator::InitPoseRecovery() {
  poseRecoveryEnabled_ = declare_parameter<bool>("obstacle_pose_recovery_enabled", false);
  if (!poseRecoveryEnabled_) return;
  const auto path = declare_parameter<std::string>("recovery_map_file", "");
  if (path.empty()) throw std::invalid_argument("recovery_map_file is required");
  auto physical = [this](const std::string &name) {
    return declare_parameter<double>(name, std::numeric_limits<double>::quiet_NaN());
  };
  navigation_safety::Footprint footprint;
  footprint.length = physical("vehicle_length_m");
  footprint.width = physical("vehicle_width_m");
  footprint.center_offset_x = footprint.center_offset_y = 0.0;
  visual_navigation::PoseRecovery::Config config;
  config.base_offset_x = physical("base_offset_x_m");
  config.base_offset_y = physical("base_offset_y_m");
  config.angular_acceleration = maxAngularAcceleration_;
  config.angular_deceleration = maxAngularDeceleration_;
  config.linear_acceleration = maxLinearAcceleration_;
  config.linear_deceleration = maxLinearDeceleration_;
  config.minimum_linear_speed = minLinearCommandSpeed_;
  config.control_dt = 1.0 / controlFrequency_;
  if (!std::isfinite(footprint.length) || !std::isfinite(footprint.width) ||
      !std::isfinite(config.base_offset_x) || !std::isfinite(config.base_offset_y) ||
      footprint.length <= 0.0 || footprint.width <= 0.0)
    throw std::invalid_argument("pose recovery requires central vehicle physical parameters");
  config.speed = declare_parameter<double>("recovery_linear_speed", 0.035);
  config.angular = declare_parameter<double>("recovery_angular_speed", 0.65);
  config.primitive_s = declare_parameter<double>("recovery_primitive_s", 0.45);
  config.margin = declare_parameter<double>("recovery_margin_m", 0.002);
  config.exit_distance = declare_parameter<double>("recovery_exit_distance_m", 0.18);
  // Compatibility parameter only: retreat length is selected by road topology.
  config.max_radius = declare_parameter<double>("recovery_max_radius_m", 0.85);
  config.max_expansions = declare_parameter<int>("recovery_max_expansions", 220000);
  declare_parameter<double>("recovery_progress_timeout_s", 2.0);
  declare_parameter<double>("recovery_selection_timeout_s", 30.0);
  declare_parameter<double>("recovery_switch_stop_s", 0.22);
  recoveryTurnConfig_ = turnManeuverConfig_;
  recoveryTurnConfig_.turn_angular_speed =
    declare_parameter<double>("recovery_turn_angular_speed", 0.65);
  recoveryTurnConfig_.turn_min_angular_speed =
    declare_parameter<double>("recovery_turn_min_angular_speed", 0.65);
  recoveryTurnConfig_.entry_target_x =
    declare_parameter<double>("recovery_turn_entry_target_x", -0.030);
  recoveryTurnConfig_.entry_target_y =
    declare_parameter<double>("recovery_turn_entry_target_y", 0.0);
  recoveryTurnConfig_.creep_distance =
    declare_parameter<double>("recovery_turn_creep_distance", 0.005);
  recoveryTurnConfig_.entry_position_tolerance =
    declare_parameter<double>("recovery_turn_entry_tolerance", 0.003);
  config.turn = recoveryTurnConfig_;
  const double stop_speed = declare_parameter<double>("recovery_stop_speed_mps", 0.002);
  const double stop_yaw = declare_parameter<double>("recovery_stop_yaw_rate_rps", 0.03);
  const double stop_timeout = declare_parameter<double>("recovery_stop_timeout_s", 2.0);
  const double stop_dwell = get_parameter("recovery_switch_stop_s").as_double();
  const double limits[] = {config.speed, config.angular, config.margin, config.exit_distance,
    recoveryTurnConfig_.turn_angular_speed, recoveryTurnConfig_.turn_min_angular_speed,
    recoveryTurnConfig_.entry_target_x, recoveryTurnConfig_.entry_target_y,
    recoveryTurnConfig_.creep_distance,
    recoveryTurnConfig_.entry_position_tolerance,
    stop_speed, stop_yaw, stop_timeout, stop_dwell};
  if (std::any_of(std::begin(limits), std::end(limits),
      [](double value) { return !std::isfinite(value); }))
    throw std::invalid_argument("pose recovery limits must be finite");
  if (config.speed < minLinearCommandSpeed_ || config.speed > maxLinearSpeed_ ||
      config.angular <= 0.0 || config.angular > maxAngularSpeed_ ||
      config.margin < 0.0 || config.exit_distance <= 0.0 ||
      recoveryTurnConfig_.turn_angular_speed > maxAngularSpeed_ ||
      recoveryTurnConfig_.turn_min_angular_speed <= 0.0 ||
      recoveryTurnConfig_.turn_min_angular_speed > recoveryTurnConfig_.turn_angular_speed ||
      recoveryTurnConfig_.creep_distance <= 0.0 ||
      recoveryTurnConfig_.entry_position_tolerance <= 0.0 ||
      stop_speed <= 0.0 || stop_yaw <= 0.0 || stop_dwell <= 0.0 || stop_timeout <= stop_dwell)
    throw std::invalid_argument("invalid pose recovery limits");
  recoveryStopGuard_.configure(stop_speed, stop_yaw, stop_dwell, stop_timeout,
    static_cast<unsigned>(preTurnStopTelemetrySamples_));
  recoveryTurnController_.configure(recoveryTurnConfig_);
  poseRecovery_.load(path, footprint, config);
  recoveryPolygonSubscription_ = create_subscription<geometry_msgs::msg::PolygonStamped>(
    declare_parameter<std::string>("recovery_obstacle_topic", "/perception/dynamic_obstacle"), 10,
    [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
      if (msg->header.frame_id != routeFrame_ || msg->polygon.points.size() < 3) return;
      navigation_safety::Polygon polygon;
      for (const auto &p : msg->polygon.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return;
        polygon.points.push_back({p.x, p.y});
      }
      latestRecoveryPolygon_ = *msg;
      recoveryPolygonTime_ = std::chrono::steady_clock::now();
      if (poseRecoveryPhase_.empty() || poseRecoveryPhase_ == "braking" ||
          poseRecoveryPhase_ == "failed" || poseRecoveryPhase_ == "complete") return;
      const bool known = std::any_of(recoveryObservedObstacles_.begin(),
        recoveryObservedObstacles_.end(), [&](const auto &old) { return same_obstacle(old, polygon); });
      if (!known) {
        recoveryObservedObstacles_.push_back(polygon);
        poseRecovery_.set_dynamic_obstacles(recoveryObservedObstacles_);
        recoveryNewHazard_ = true;
        // Keep the original transaction identity so its owner receives failure.
        FailPoseRecovery("FAULT_POSE_RECOVERY_NEW_OBSTACLE");
      }
    });
  selectRecoveryService_ = create_service<mission_control_interfaces::srv::SelectRecovery>(
    "~/select_recovery", [this](
      mission_control_interfaces::srv::SelectRecovery::Request::SharedPtr req,
      mission_control_interfaces::srv::SelectRecovery::Response::SharedPtr res) {
      if (poseRecoveryPhase_ != "candidates" || req->recovery_id != poseRecoveryId_ ||
          req->obstacle_revision != poseRecoveryRevision_ || recoveryNewHazard_) {
        res->message = "stale recovery selection"; return;
      }
      const auto it = std::find_if(recoveryCandidates_.begin(), recoveryCandidates_.end(),
        [&](const auto &candidate) { return candidate.candidate_id == req->candidate_id; });
      if (it == recoveryCandidates_.end()) { res->message = "unknown candidate"; return; }
      PublishStop();
      if (!poseRecovery_.plan(RecoveryPose(), *it, req->blocked_lane_ids)) {
        res->message = "road template has no complete safe execution"; return;
      }
      recoverySelected_ = *it;
      recoveryStep_ = 0;
      recoveryStepArmed_ = false;
      recoveryDirectionStop_ = false;
      recoveryMotionDirection_ = 0;
      recoveryTurnManeuverStarted_ = false;
      recoveryTurnController_.reset();
      poseRecoveryPhase_ = "motion";
      BeginRecoveryStop(poseRecovery_.active_seconds(recovery_seconds()));
      SetState("POSE_RECOVERY_DIRECTION_STOP");
      PublishPoseRecovery("motion");
      res->success = true;
      res->message = "selected complete road recovery template";
      RCLCPP_INFO(get_logger(), "Pose recovery selected: id=%lu template=%s lane=%s actions=%zu",
        static_cast<unsigned long>(poseRecoveryId_), it->template_id.c_str(),
        it->resume_lane.c_str(), poseRecovery_.steps().size());
    });
}

void WaypointNavigator::PublishPoseRecovery(const std::string &phase) {
  mission_control_interfaces::msg::NavigationRecoveryState msg;
  msg.header.stamp = now(); msg.header.frame_id = routeFrame_; msg.route_id = activeRouteId_;
  msg.valid = phase != "failed"; msg.phase = phase; msg.reason = state_;
  msg.recovery_id = poseRecoveryId_; msg.obstacle_revision = poseRecoveryRevision_;
  const auto actual = RecoveryPose();
  msg.safe_pose.header = msg.header;
  msg.safe_pose.pose.position.x = actual.position.x;
  msg.safe_pose.pose.position.y = actual.position.y;
  msg.safe_pose.pose.orientation.z = std::sin(actual.yaw / 2.0);
  msg.safe_pose.pose.orientation.w = std::cos(actual.yaw / 2.0);
  msg.blocked_lane_id = poseRecovery_.blocked_lane();
  for (auto p : poseRecoveryObstacle_.points) {
    geometry_msgs::msg::Point32 q; q.x = p.x; q.y = p.y; msg.obstacle.points.push_back(q);
  }
  if (phase == "candidates") msg.candidates = recoveryCandidates_;
  if (phase == "complete") {
    msg.resume_node = recoverySelected_.resume_node;
    msg.resume_lane = poseRecovery_.lane_at_pose(actual);
    msg.resume_heading = actual.yaw;
  }
  navigationRecoveryStatePublisher_->publish(msg);
}

void WaypointNavigator::BeginPoseRecovery() {
  pendingFrontObstacle_ = false; pendingObstacleAtActiveTurn_ = false;
  if (!poseRecoveryPhase_.empty()) return;
  ++poseRecoveryId_; poseRecoveryRevision_ = 0;
  recoveryNewHazard_ = false;
  poseRecoveryObstacle_.points.clear();
  const auto wall = std::chrono::steady_clock::now();
  const double polygon_age = std::chrono::duration<double>(wall - recoveryPolygonTime_).count();
  const double range_age = std::chrono::duration<double>(wall - lastFrontObstacleRangeArrival_).count();
  if (polygon_age < frontObstacleTimeout_ && latestRecoveryPolygon_.polygon.points.size() >= 3) {
    for (auto p : latestRecoveryPolygon_.polygon.points) poseRecoveryObstacle_.points.push_back({p.x, p.y});
  } else if (frontObstacleRangeReceived_ && range_age < frontObstacleTimeout_) {
    const double near = obstacleSensorForwardOffset_ + frontObstacleRange_;
    const auto base = RecoveryPose();
    for (auto xy : std::vector<std::pair<double, double>>{
        {near,-0.09}, {near+0.08,-0.09}, {near+0.08,0.09}, {near,0.09}})
      poseRecoveryObstacle_.points.push_back({
        base.position.x + xy.first * std::cos(base.yaw) - xy.second * std::sin(base.yaw),
        base.position.y + xy.first * std::sin(base.yaw) + xy.second * std::cos(base.yaw)});
  }
  recoveryObservedObstacles_ = {poseRecoveryObstacle_};
  poseRecovery_.set_dynamic_obstacles(recoveryObservedObstacles_);
  poseRecovery_.reset_clock(recovery_seconds());
  recoveryTurnManeuverStarted_ = false;
  recoveryTurnController_.reset();
  turnObstacleContextValid_ = false;
  if (poseRecoveryObstacle_.points.size() < 3) {
    FailPoseRecovery("FAULT_POSE_RECOVERY_OBSTACLE_GEOMETRY"); return;
  }
  if (BeginTurnObstacleRecovery()) {
    PublishPoseRecovery("braking"); return;
  }
  ResetManeuver(); pathProgressSupervisor_.Reset(); turnProgressSupervisor_.Reset();
  poseRecoveryPhase_ = "braking";
  BeginRecoveryStop(0.0);
  SetState("POSE_RECOVERY_BRAKING"); PublishPoseRecovery("braking");
}
