#pragma once
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include "vehicle_loopback_sim/motion.hpp"
#include "visual_navigation/pose_recovery.hpp"
#include "visual_navigation/rate_limiter.hpp"
#include "visual_navigation/recovery_stop.hpp"

// 验收用独立车辆积分。动作之间连续保留实测姿态，绝不写入 Step.start/end。
// 直行采用公开的道路反馈律，转弯调用正式特定转弯控制器；不调用 plan/reference/integrate。
class RecoveryReplay {
public:
  explicit RecoveryReplay(visual_navigation::PoseRecovery &recovery,const std::string &root)
    : recovery_(recovery),config_(recovery.config()) {
    const auto output=YAML::LoadFile(root+"/src/visual_navigation/config/waypoint_navigation.yaml")
      ["waypoint_navigator"]["ros__parameters"];
    const auto params=YAML::LoadFile(root+"/src/contrast_controller/config/contrast_controller.yaml")
      ["contrast_controller"]["ros__parameters"];
    const auto physical=YAML::LoadFile(root+"/src/vehicle_config/config/vehicle_physical.yaml");
    track_=physical["differential_drive"]["wheel_track_m"].as<double>();
    max_linear_=output["max_linear_speed"].as<double>();
    max_angular_=output["max_angular_speed"].as<double>();
    stop_.configure(params["recovery_stop_speed_mps"].as<double>(),
      params["recovery_stop_yaw_rate_rps"].as<double>(),
      params["recovery_switch_stop_s"].as<double>(),
      params["recovery_stop_timeout_s"].as<double>(),
      output["pre_turn_stop_telemetry_samples"].as<unsigned>());
  }

  bool run(const visual_navigation::PoseRecovery::Pose &start,
           const std::vector<visual_navigation::PoseRecovery::Step> &steps) {
    actual_={start.position.x,start.position.y,start.yaw};
    for(const auto &step:steps) {
      bool result=false;
      if(step.kind==visual_navigation::PoseRecovery::Step::Kind::FIXED_TURN) result=turn(step);
      else if(step.kind==visual_navigation::PoseRecovery::Step::Kind::STOP) result=stop();
      else result=straight(step);
      if(!result) { failure_=step.phase+": "+failure_; return false; }
      if(!stop()) return false;
    }
    return true;
  }
  visual_navigation::PoseRecovery::Pose actual() const {
    return {{actual_.x,actual_.y},actual_.yaw};
  }
  const std::string &failure() const {return failure_;}
  std::size_t turns60() const {return turns60_;}
  std::size_t turns30() const {return turns30_;}
  double reverse_distance() const {return reverse_distance_;}

private:
  using Step=visual_navigation::PoseRecovery::Step;
  bool command(vehicle_loopback_sim::Twist2d desired) {
    const double dt=config_.control_dt;
    if(std::abs(desired.linear_x)<1e-9 && std::abs(desired.angular_z)<1e-9) twist_={};
    else {
      const double v=std::clamp(desired.linear_x,-max_linear_,max_linear_);
      twist_.linear_x=visual_navigation::LimitRate(v,twist_.linear_x,
        config_.linear_acceleration,config_.linear_deceleration,dt);
      if(std::abs(v)>1e-9 && std::abs(twist_.linear_x)<config_.minimum_linear_speed)
        twist_.linear_x=std::copysign(config_.minimum_linear_speed,v);
      else if(std::abs(v)<=1e-9 && std::abs(twist_.linear_x)<config_.minimum_linear_speed)
        twist_.linear_x=0;
      twist_.angular_z=visual_navigation::LimitRate(
        std::clamp(desired.angular_z,-max_angular_,max_angular_),twist_.angular_z,
        config_.angular_acceleration,config_.angular_deceleration,dt);
    }
    std::vector<visual_navigation::PoseRecovery::Pose> sweep{actual()};
    for(int sample=0;sample<4;++sample) {
      const auto previous=actual_;
      actual_=vehicle_loopback_sim::integrate_base_link(actual_,twist_,dt/4,
        config_.base_offset_x,config_.base_offset_y);
      if(twist_.linear_x<0)
        reverse_distance_+=std::hypot(actual_.x-previous.x,actual_.y-previous.y);
      sweep.push_back(actual());
    }
    now_+=dt; ++sample_;
    if(!recovery_.safe(sweep)) {failure_="独立车辆积分的实际扫掠不安全";return false;}
    return true;
  }
  bool stop() {
    stop_.begin(now_,sample_);
    for(unsigned attempt=0;attempt<300;++attempt) {
      if(!command({})) return false;
      const auto wheels=vehicle_loopback_sim::wheel_speeds(twist_,track_);
      const auto result=stop_.update(now_,sample_,true,wheels.first,wheels.second,
        wheels.first,wheels.second,twist_.angular_z);
      if(result==visual_navigation::RecoveryStopGuard::Result::STOPPED) return true;
      if(result==visual_navigation::RecoveryStopGuard::Result::TIMED_OUT) break;
    }
    failure_="独立实测停车未确认";return false;
  }
  bool straight(const Step &step) {
    const double started=now_,c=std::cos(step.geometry_heading),s=std::sin(step.geometry_heading);
    while(now_-started<=step.duration+config_.turn.phase_timeout) {
      const double along=(step.target.position.x-actual_.x)*c+(step.target.position.y-actual_.y)*s;
      const double lateral=-(actual_.x-step.road_origin.x)*s+(actual_.y-step.road_origin.y)*c;
      const double yaw=vehicle_loopback_sim::normalize_angle(step.target.yaw-actual_.yaw);
      if(std::abs(along)<=config_.turn.entry_position_tolerance &&
         std::abs(lateral)<=config_.turn.entry_lateral_tolerance &&
         std::abs(yaw)<=config_.turn.entry_yaw_tolerance) return true;
      if(along*step.linear < -config_.turn.entry_position_tolerance*std::abs(step.linear)) {
        failure_="直行实测位置越过动作目标";return false;
      }
      const double direction=std::copysign(1.,step.linear);
      const double angular=std::clamp(config_.turn.entry_heading_gain*yaw-
        direction*config_.turn.entry_lateral_gain*lateral,
        -config_.turn.entry_max_angular_speed,config_.turn.entry_max_angular_speed);
      if(!command({step.linear,angular})) return false;
    }
    failure_="直行闭环实际未收敛";return false;
  }
  bool turn(const Step &step) {
    visual_navigation::TurnManeuverController controller(config_.turn);
    controller.begin(now_);
    auto previous=visual_navigation::TurnManeuverPhase::IDLE;
    int previous_direction=0;
    const double started=now_;
    while(now_-started<=step.duration+config_.turn.phase_timeout) {
      visual_navigation::TurnManeuverInput input;
      auto &context=input.context;
      context.pose={actual_.x,actual_.y,actual_.yaw};
      context.junction_x=step.junction.x;context.junction_y=step.junction.y;
      context.geometry_heading=context.arrival_heading=step.geometry_heading;
      context.target_heading=step.target_heading;context.reverse_arrival=step.reverse_arrival;
      context.applied_twist={twist_.linear_x,twist_.angular_z};
      context.measured_twist=context.applied_twist;context.measured_yaw_rate_valid=true;
      const auto decision=controller.update(input,now_);
      if(decision.phase!=previous) {
        turns60_+=decision.phase==visual_navigation::TurnManeuverPhase::TURN_60;
        turns30_+=decision.phase==visual_navigation::TurnManeuverPhase::TURN_30;
        previous=decision.phase;
      }
      if(decision.result==visual_navigation::TurnManeuverResult::COMPLETED) return true;
      if(decision.result!=visual_navigation::TurnManeuverResult::RUNNING) {
        failure_="连续实测姿态下特定转弯失败";return false;
      }
      const int direction=decision.command.linear_x>1e-9 ? 1 :
                          decision.command.linear_x < -1e-9 ? -1 : 0;
      if(direction && previous_direction && direction!=previous_direction && !stop())
        return false;
      if(direction) previous_direction=direction;
      if(!command({decision.command.linear_x,decision.command.angular_z})) return false;
    }
    failure_="连续实测姿态下特定转弯超时";return false;
  }
  visual_navigation::PoseRecovery &recovery_;
  visual_navigation::PoseRecovery::Config config_;
  visual_navigation::RecoveryStopGuard stop_;
  vehicle_loopback_sim::Pose2d actual_;
  vehicle_loopback_sim::Twist2d twist_;
  double track_{0},max_linear_{0},max_angular_{0},now_{0};
  double reverse_distance_{0.0};
  std::uint64_t sample_{0};
  std::size_t turns60_{0},turns30_{0};
  std::string failure_;
};
