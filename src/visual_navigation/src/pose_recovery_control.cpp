#include "visual_navigation/waypoint_navigator.hpp"

namespace {
double recovery_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
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
  footprint.length=physical("vehicle_length_m"); footprint.width=physical("vehicle_width_m");
  footprint.center_offset_x=0.0;footprint.center_offset_y=0.0;
  visual_navigation::PoseRecovery::Config config;
  config.base_offset_x=physical("base_offset_x_m");
  config.base_offset_y=physical("base_offset_y_m");
  config.angular_acceleration=maxAngularAcceleration_;
  config.angular_deceleration=maxAngularDeceleration_;
  config.control_dt=1.0/controlFrequency_;
  if (!std::isfinite(footprint.length) || !std::isfinite(footprint.width) ||
      !std::isfinite(config.base_offset_x) || !std::isfinite(config.base_offset_y) ||
      footprint.length<=0 || footprint.width<=0)
    throw std::invalid_argument("pose recovery requires central vehicle physical parameters");
  config.speed=declare_parameter<double>("recovery_linear_speed",0.035);
  config.angular=declare_parameter<double>("recovery_angular_speed",0.65);
  config.primitive_s=declare_parameter<double>("recovery_primitive_s",0.45);
  config.margin=declare_parameter<double>("recovery_margin_m",0.002);
  config.exit_distance=declare_parameter<double>("recovery_exit_distance_m",0.18);
  config.max_radius=declare_parameter<double>("recovery_max_radius_m",0.85);
  config.max_expansions=declare_parameter<int>("recovery_max_expansions",220000);
  declare_parameter<double>("recovery_progress_timeout_s",2.0);
  declare_parameter<double>("recovery_selection_timeout_s",30.0);
  declare_parameter<double>("recovery_switch_stop_s",0.22);
  if (config.speed<minLinearCommandSpeed_ || config.speed>maxLinearSpeed_ ||
      config.angular<=0 || config.angular>maxAngularSpeed_ || config.primitive_s<=0 ||
      config.margin<0 || config.exit_distance<=0 || config.max_radius<=0 || config.max_expansions<=0)
    throw std::invalid_argument("invalid pose recovery limits");
  poseRecovery_.load(path,footprint,config);
  recoveryPolygonSubscription_=create_subscription<geometry_msgs::msg::PolygonStamped>(
    declare_parameter<std::string>("recovery_obstacle_topic","/perception/dynamic_obstacle"),10,
    [this](geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
      if (msg->header.frame_id==routeFrame_ && msg->polygon.points.size()>=3) {
        latestRecoveryPolygon_=*msg; recoveryPolygonTime_=std::chrono::steady_clock::now();
      }
    });
  selectRecoveryService_=create_service<mission_control_interfaces::srv::SelectRecovery>(
    "~/select_recovery",[this](
      mission_control_interfaces::srv::SelectRecovery::Request::SharedPtr req,
      mission_control_interfaces::srv::SelectRecovery::Response::SharedPtr res) {
      if (poseRecoveryPhase_!="candidates" || req->recovery_id!=poseRecoveryId_ ||
          req->obstacle_revision!=poseRecoveryRevision_) {
        res->message="stale recovery selection";return;
      }
      auto it=std::find_if(recoveryCandidates_.begin(),recoveryCandidates_.end(),
        [&](const auto &c){return c.candidate_id==req->candidate_id;});
      if (it==recoveryCandidates_.end()) {res->message="unknown candidate";return;}
      const auto blocked=[&](const std::string &lane) {
        return std::find(req->blocked_lane_ids.begin(),req->blocked_lane_ids.end(),lane)!=req->blocked_lane_ids.end();
      };
      if(blocked(it->resume_lane) || std::any_of(it->borrowed_lane_ids.begin(),it->borrowed_lane_ids.end(),blocked)) {
        res->message="selected exit or borrowed lane is blocked";return;
      }
      PublishStop();
      recoverySelected_=*it;
      PublishPoseRecovery("planning_motion");
      if (!poseRecovery_.plan({{currentX_,currentY_},currentYaw_},*it,req->blocked_lane_ids)) {
        poseRecovery_.pause_clock();
        PublishPoseRecovery("candidates");
        res->message="bounded local pose search found no safe motion";return;
      }
      poseRecovery_.pause_clock();
      const double t=poseRecovery_.active_seconds(recovery_seconds());
      recoveryStep_=0;recoveryStepArmed_=false;
      recoveryPauseUntil_=t+get_parameter("recovery_switch_stop_s").as_double();
      recoveryLastProgress_=t;recoveryProgressPose_={{currentX_,currentY_},currentYaw_};
      poseRecoveryPhase_="motion";PublishPoseRecovery("motion");
      res->success=true;res->message="selected safe pose recovery";
      RCLCPP_INFO(get_logger(),"Pose recovery selected: id=%lu exit=%s lane=%s primitives=%zu",
        static_cast<unsigned long>(poseRecoveryId_),it->resume_node.c_str(),it->resume_lane.c_str(),poseRecovery_.steps().size());
    });
}
void WaypointNavigator::PublishPoseRecovery(const std::string &phase) {
  mission_control_interfaces::msg::NavigationRecoveryState msg;
  msg.header.stamp=now();msg.header.frame_id=routeFrame_;msg.route_id=activeRouteId_;
  msg.valid=phase!="failed";msg.phase=phase;msg.reason=state_;
  msg.recovery_id=poseRecoveryId_;msg.obstacle_revision=poseRecoveryRevision_;
  msg.safe_pose.header=msg.header;msg.safe_pose.pose.position.x=currentX_;msg.safe_pose.pose.position.y=currentY_;
  msg.safe_pose.pose.orientation.z=std::sin(currentYaw_/2);msg.safe_pose.pose.orientation.w=std::cos(currentYaw_/2);
  msg.blocked_lane_id=poseRecovery_.blocked_lane();
  for (auto p : poseRecoveryObstacle_.points) {geometry_msgs::msg::Point32 q;q.x=p.x;q.y=p.y;msg.obstacle.points.push_back(q);}
  if (phase=="candidates") msg.candidates=recoveryCandidates_;
  if (phase=="complete") {msg.resume_node=recoverySelected_.resume_node;msg.resume_lane=recoverySelected_.resume_lane;msg.resume_heading=currentYaw_;}
  navigationRecoveryStatePublisher_->publish(msg);
}
void WaypointNavigator::BeginPoseRecovery() {
  pendingFrontObstacle_=false;pendingObstacleAtActiveTurn_=false;
  if (!poseRecoveryPhase_.empty()) return;
  ++poseRecoveryId_;poseRecoveryRevision_=0;poseRecoveryObstacle_.points.clear();
  const double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-recoveryPolygonTime_).count();
  if (age<frontObstacleTimeout_ && latestRecoveryPolygon_.polygon.points.size()>=3) {
    for (auto p : latestRecoveryPolygon_.polygon.points) poseRecoveryObstacle_.points.push_back({p.x,p.y});
  } else if (frontObstacleRangeReceived_) {
    // Conservative fallback from the same stopped observation, latched once.
    const double near=obstacleSensorForwardOffset_+frontObstacleRange_;
    for (auto xy : std::vector<std::pair<double,double>>{{near,-0.09},{near+0.08,-0.09},{near+0.08,0.09},{near,0.09}})
      poseRecoveryObstacle_.points.push_back({currentX_+xy.first*std::cos(currentYaw_)-xy.second*std::sin(currentYaw_),
        currentY_+xy.first*std::sin(currentYaw_)+xy.second*std::cos(currentYaw_)});
  }
  ResetManeuver();pathProgressSupervisor_.Reset();turnProgressSupervisor_.Reset();PublishStop();
  poseRecovery_.reset_clock(recovery_seconds());
  poseRecoveryPhase_="braking";recoveryPauseUntil_=get_parameter("recovery_switch_stop_s").as_double();
  SetState("POSE_RECOVERY_BRAKING");PublishPoseRecovery("braking");
}
bool WaypointNavigator::RunPoseRecovery() {
  if (poseRecoveryPhase_.empty()) return false;
  const double t=poseRecovery_.active_seconds(recovery_seconds());
  using Recovery=visual_navigation::PoseRecovery;
  const Recovery::Pose actual{{currentX_,currentY_},currentYaw_};
  auto fail=[&](const char *reason) {
    PublishStop();poseRecoveryPhase_="failed";SetState(reason);PublishPoseRecovery("failed");
    RCLCPP_ERROR(get_logger(),"Pose recovery failed: %s at (%.3f,%.3f,%.3f)",reason,currentX_,currentY_,currentYaw_);
  };
  if (poseRecoveryPhase_=="failed" || poseRecoveryPhase_=="complete") {PublishStop();return true;}
  if (poseRecoveryPhase_=="braking") {
    PublishStop();
    if(t<recoveryPauseUntil_ || std::max(std::abs(measuredLeftWheelSpeed_),std::abs(measuredRightWheelSpeed_))>preTurnStopSpeed_) return true;
    // Bool/Range and polygon callbacks may arrive in either order. Latch once
    // after braking; motion/candidate phases never replace this geometry.
    const double age=std::chrono::duration<double>(std::chrono::steady_clock::now()-recoveryPolygonTime_).count();
    if(age<frontObstacleTimeout_ && latestRecoveryPolygon_.polygon.points.size()>=3) {
      poseRecoveryObstacle_.points.clear();
      for(auto p:latestRecoveryPolygon_.polygon.points) poseRecoveryObstacle_.points.push_back({p.x,p.y});
    }
    recoveryCandidates_=poseRecovery_.candidates(actual,poseRecoveryObstacle_);
    if (recoveryCandidates_.empty()) {fail("FAULT_POSE_RECOVERY_NO_EXIT");return true;}
    poseRecoveryPhase_="candidates";recoveryStepStarted_=t;
    SetState("POSE_RECOVERY_SELECTING");PublishPoseRecovery("candidates");return true;
  }
  if (poseRecoveryPhase_=="candidates") {
    PublishStop();
    if(t-recoveryStepStarted_>get_parameter("recovery_selection_timeout_s").as_double()) fail("FAULT_POSE_RECOVERY_SELECTION_TIMEOUT");
    return true;
  }
  const auto &steps=poseRecovery_.steps();
  if(t<recoveryPauseUntil_ || (!recoveryStepArmed_ &&
      std::max(std::abs(measuredLeftWheelSpeed_),std::abs(measuredRightWheelSpeed_))>preTurnStopSpeed_)) {
    PublishStop();recoveryLastProgress_=t;recoveryProgressPose_=actual;return true;
  }
  if(!recoveryStepArmed_) {recoveryStepStarted_=t;recoveryStepArmed_=true;}
  // Carry boundary overshoot into the next same-direction primitive. Returning
  // here used to leave the previous command running for one extra tick/step.
  while(recoveryStep_<steps.size() && t-recoveryStepStarted_>=steps[recoveryStep_].duration) {
    const double old_linear=steps[recoveryStep_].linear;
    recoveryStepStarted_+=steps[recoveryStep_].duration;
    ++recoveryStep_;
    if(recoveryStep_<steps.size() && old_linear*steps[recoveryStep_].linear<0) {
      recoveryStepArmed_=false;
      recoveryPauseUntil_=t+get_parameter("recovery_switch_stop_s").as_double();
      recoveryLastProgress_=t;recoveryProgressPose_=actual;
      PublishStop();SetState("POSE_RECOVERY_DIRECTION_STOP");return true;
    }
  }
  if (recoveryStep_>=steps.size()) {
    PublishStop();
    if(std::max(std::abs(measuredLeftWheelSpeed_),std::abs(measuredRightWheelSpeed_))>preTurnStopSpeed_) return true;
    const double distance=std::hypot(currentX_-recoverySelected_.exit_pose.position.x,currentY_-recoverySelected_.exit_pose.position.y);
    if (distance>0.025 || std::abs(NormalizeAngle(currentYaw_-recoverySelected_.resume_heading))>0.10) {
      fail("FAULT_POSE_RECOVERY_EXIT_ERROR");return true;
    }
    poseRecoveryPhase_="complete";SetState("POSE_RECOVERY_COMPLETE");PublishPoseRecovery("complete");return true;
  }
  const auto &step=steps[recoveryStep_];
  const double elapsed=t-recoveryStepStarted_;
  auto reference=poseRecovery_.reference(step,elapsed);
  const double ex=reference.position.x-currentX_,ey=reference.position.y-currentY_;
  const double lateral=-std::sin(currentYaw_)*ex+std::cos(currentYaw_)*ey;
  geometry_msgs::msg::Twist command;
  command.linear.x=step.linear;
  command.angular.z=std::clamp(step.angular+1.8*NormalizeAngle(reference.yaw-currentYaw_)+
    std::copysign(1.,step.linear)*4.*lateral,-poseRecovery_.config().angular,poseRecovery_.config().angular);
  // Match PublishMotionCommand's measured dt, saturation and minimum speed.
  double command_dt=1.0/controlFrequency_;
  if(motionCommandInitialized_) {
    const double measured=std::chrono::duration<double>(std::chrono::steady_clock::now()-lastMotionCommandTime_).count();
    if(measured>0.0 && measured<=0.2) command_dt=measured;
  }
  const double target_linear=std::clamp(command.linear.x,-maxLinearSpeed_,maxLinearSpeed_);
  double predicted_linear=visual_navigation::LimitRate(target_linear,lastMotionCommand_.linear.x,
    maxLinearAcceleration_,maxLinearDeceleration_,command_dt);
  if(std::abs(predicted_linear)<minLinearCommandSpeed_)
    predicted_linear=std::copysign(minLinearCommandSpeed_,target_linear);
  const double predicted_angular=visual_navigation::LimitRate(command.angular.z,lastMotionCommand_.angular.z,
    maxAngularAcceleration_,maxAngularDeceleration_,command_dt);
  const double horizon=std::max(command_dt,std::min(0.20,step.duration-elapsed));
  auto sweep=poseRecovery_.motion_samples(actual,predicted_linear,predicted_angular,predicted_angular,command_dt);
  if(horizon>command_dt) {
    const auto tail=poseRecovery_.motion_samples(sweep.back(),predicted_linear,command.angular.z,
      predicted_angular,horizon-command_dt);
    sweep.insert(sweep.end(),tail.begin()+1,tail.end());
  }
  if(!poseRecovery_.safe(sweep)) {
    fail("FAULT_POSE_RECOVERY_SWEEP");return true;
  }
  if(std::hypot(currentX_-recoveryProgressPose_.position.x,currentY_-recoveryProgressPose_.position.y)>0.003 ||
    std::abs(NormalizeAngle(currentYaw_-recoveryProgressPose_.yaw))>0.03) {
    recoveryLastProgress_=t;recoveryProgressPose_=actual;
  } else if(t-recoveryLastProgress_>get_parameter("recovery_progress_timeout_s").as_double()) {
    fail("FAULT_POSE_RECOVERY_NO_PROGRESS");return true;
  }
  SetState(step.linear>0 ? "POSE_RECOVERY_FORWARD" : "POSE_RECOVERY_REVERSE");
  PublishMotionCommand(command);return true;
}
