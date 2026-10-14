#include "visual_navigation/corner_return.hpp"
#include "visual_navigation/rate_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace visual_navigation {
namespace {
constexpr double pi = 3.14159265358979323846;
double distance(CornerShuntPose a, CornerShuntPose b) {
  return std::hypot(a.x-b.x,a.y-b.y);
}
double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}
// 仅按车型轴心几何推导固定模板，不读取已执行动作/指令历史。
CornerShuntPose integrate(CornerShuntPose p, CornerShuntTwist twist,
                          const CornerReturnConfig &config) {
  const double ox=config.base_offset_x, oy=config.base_offset_y;
  p.x-=std::cos(p.yaw)*ox-std::sin(p.yaw)*oy;
  p.y-=std::sin(p.yaw)*ox+std::cos(p.yaw)*oy;
  const double angle=p.yaw+twist.angular_z*config.control_dt;
  if(std::abs(twist.angular_z)<1e-9) {
    p.x+=twist.linear_x*config.control_dt*std::cos(p.yaw);
    p.y+=twist.linear_x*config.control_dt*std::sin(p.yaw);
  } else {
    p.x+=twist.linear_x/twist.angular_z*(std::sin(angle)-std::sin(p.yaw));
    p.y+=twist.linear_x/twist.angular_z*(std::cos(p.yaw)-std::cos(angle));
  }
  p.yaw=NormalizeShuntAngle(angle);
  p.x+=std::cos(p.yaw)*ox-std::sin(p.yaw)*oy;
  p.y+=std::sin(p.yaw)*ox+std::cos(p.yaw)*oy;
  return p;
}
}

const char *CornerReturnStatus(CornerReturnPhase phase) {
  switch(phase) {
  case CornerReturnPhase::IDLE:return "RETURN_IDLE";
  case CornerReturnPhase::CAPTURE:return "RETURN_CAPTURE";
  case CornerReturnPhase::APPROACH_ENTRY:return "RETURN_APPROACH_ENTRY";
  case CornerReturnPhase::TURN_30:return "RETURN_TURN_30";
  case CornerReturnPhase::DIAGONAL_CREEP:return "RETURN_DIAGONAL_CREEP";
  case CornerReturnPhase::TURN_60:return "RETURN_TURN_60";
  case CornerReturnPhase::RETURN_ENTRY:return "RETURN_ENTRY_STOP_POINT";
  case CornerReturnPhase::COMPLETE:return "RETURN_COMPLETE";
  case CornerReturnPhase::PAUSED:return "RETURN_PAUSED";
  case CornerReturnPhase::FAULT:return "FAULT_CORNER_RETURN";
  }
  return "FAULT_CORNER_RETURN";
}

CornerReturnController::CornerReturnController(CornerReturnConfig config):config_(config) {}
void CornerReturnController::configure(const CornerReturnConfig &config) {config_=config;reset();}
void CornerReturnController::reset() {*this=CornerReturnController(config_);}
bool CornerReturnController::valid(const CornerShuntContext &context) const {
  const double values[]={config_.base_offset_x,config_.base_offset_y,
    config_.linear_acceleration,config_.linear_deceleration,
    config_.angular_acceleration,config_.angular_deceleration,
    config_.minimum_linear_speed,config_.control_dt,
    context.pose.x,context.pose.y,context.pose.yaw,context.junction_x,
    context.junction_y,context.geometry_heading,context.target_heading};
  for(double value:values) if(!std::isfinite(value)) return false;
  return config_.linear_acceleration>0 && config_.linear_deceleration>0 &&
    config_.angular_acceleration>0 && config_.angular_deceleration>0 &&
    config_.control_dt>0 && config_.minimum_linear_speed>=0 &&
    std::abs(std::abs(NormalizeShuntAngle(context.target_heading-
      context.geometry_heading))-pi/2)<0.01;
}

bool CornerReturnController::build_geometry() {
  const auto &turn=config_.turn;
  const double sign=NormalizeShuntAngle(captured_.target_heading-
                                      captured_.geometry_heading)>0 ? 1.0:-1.0;
  const double c=std::cos(captured_.geometry_heading),s=std::sin(captured_.geometry_heading);
  entrance_={captured_.junction_x+turn.entry_target_x*c-sign*turn.entry_target_y*s,
             captured_.junction_y+turn.entry_target_x*s+sign*turn.entry_target_y*c,
             captured_.geometry_heading};
  auto context=captured_;
  context.pose=entrance_;
  TurnManeuverController controller(turn);
  controller.begin(0.0);
  CornerShuntTwist applied{};
  for(double t=0; t<6*turn.phase_timeout;t+=config_.control_dt) {
    TurnManeuverInput input;input.context=context;
    input.context.applied_twist=applied;input.context.measured_twist=applied;
    const auto result=controller.update(input,t);
    if(result.result==TurnManeuverResult::COMPLETED) {exit_=context.pose;return true;}
    if(result.result!=TurnManeuverResult::RUNNING) return false;
    if(result.phase==TurnManeuverPhase::DIAGONAL_CREEP &&
       std::abs(result.command.linear_x)<1e-9) sixty_end_=context.pose;
    if(result.phase==TurnManeuverPhase::TURN_30 &&
       std::abs(result.command.linear_x)<1e-9) creep_end_=context.pose;
    if(std::abs(result.command.linear_x)<1e-9 && std::abs(result.command.angular_z)<1e-9)
      applied={};
    else {
      applied.linear_x=LimitRate(result.command.linear_x,applied.linear_x,
        config_.linear_acceleration,config_.linear_deceleration,config_.control_dt);
      if(std::abs(applied.linear_x)<config_.minimum_linear_speed)
        applied.linear_x=std::copysign(config_.minimum_linear_speed,result.command.linear_x);
      applied.angular_z=LimitRate(result.command.angular_z,applied.angular_z,
        config_.angular_acceleration,config_.angular_deceleration,config_.control_dt);
    }
    context.pose=integrate(context.pose,applied,config_);
  }
  return false;
}

void CornerReturnController::begin(const CornerShuntContext &context,double now_s) {
  reset();captured_=context;active_=true;phase_=CornerReturnPhase::CAPTURE;
  phase_started_at_=progress_started_at_=now_s;progress_pose_=context.pose;
  captured_valid_=valid(context) && build_geometry();
}
CornerReturnDecision CornerReturnController::terminal(CornerReturnPhase phase,
                                                      TurnManeuverResult result) {
  phase_=phase;active_=false;return {phase,{},result};
}

CornerReturnDecision CornerReturnController::update(const CornerReturnInput &input,double t) {
  using Phase=CornerReturnPhase;
  if(!active_ || !std::isfinite(t) || !captured_valid_)
    return terminal(Phase::FAULT,TurnManeuverResult::FAULT);
  if(input.cancel) return terminal(Phase::IDLE,TurnManeuverResult::CANCELED);
  if(!input.motion_allowed || !input.localization_valid || input.obstacle_blocked) {
    if(phase_!=Phase::PAUSED) {
      paused_phase_=phase_;phase_=Phase::PAUSED;progress_started_at_=t;
    }
    return {phase_,{},TurnManeuverResult::RUNNING};
  }
  if(phase_==Phase::PAUSED) {
    phase_started_at_+=std::max(0.0,t-progress_started_at_);
    phase_=paused_phase_;progress_pose_=input.context.pose;progress_started_at_=t;
    return {phase_,{},TurnManeuverResult::RUNNING};
  }
  const auto p=input.context.pose;
  if(!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.yaw))
    return terminal(Phase::FAULT,TurnManeuverResult::FAULT);
  const auto &cfg=config_.turn;
  const double sign=NormalizeShuntAngle(captured_.target_heading-captured_.geometry_heading)>0 ? 1.0:-1.0;
  const auto enter=[&](Phase next) {
    phase_=next;phase_started_at_=progress_started_at_=t;progress_pose_=p;
    return CornerReturnDecision{phase_,{},TurnManeuverResult::RUNNING};
  };
  if(phase_==Phase::CAPTURE) return enter(Phase::APPROACH_ENTRY);
  const auto target=phase_==Phase::APPROACH_ENTRY ? exit_ :
    phase_==Phase::TURN_30 ? creep_end_ :
    phase_==Phase::DIAGONAL_CREEP ? sixty_end_ : entrance_;
  const double yaw_error=NormalizeShuntAngle(target.yaw-p.yaw);
  if(t-phase_started_at_>=cfg.phase_timeout)
    return terminal(Phase::FAULT,TurnManeuverResult::NEED_REPLAN);
  if(t-progress_started_at_>=cfg.no_progress_timeout) {
    const bool turning=phase_==Phase::TURN_30 || phase_==Phase::TURN_60;
    const double progress=turning ?
      std::abs(NormalizeShuntAngle(target.yaw-progress_pose_.yaw))-std::abs(yaw_error) :
      distance(progress_pose_,target)-distance(p,target);
    if(progress<(turning ? cfg.min_yaw_progress:cfg.min_position_progress))
      return terminal(Phase::FAULT,TurnManeuverResult::NEED_REPLAN);
    progress_pose_=p;progress_started_at_=t;
  }
  CornerShuntTwist command;
  if(phase_==Phase::APPROACH_ENTRY || phase_==Phase::RETURN_ENTRY) {
    const double c=std::cos(target.yaw),s=std::sin(target.yaw);
    const double along=(target.x-p.x)*c+(target.y-p.y)*s;
    const double lateral=-(p.x-target.x)*s+(p.y-target.y)*c;
    if(std::abs(along)<=cfg.entry_position_tolerance &&
       std::abs(lateral)<=cfg.entry_lateral_tolerance &&
       std::abs(yaw_error)<=cfg.entry_yaw_tolerance) {
      if(phase_==Phase::RETURN_ENTRY)
        return terminal(Phase::COMPLETE,TurnManeuverResult::COMPLETED);
      return enter(Phase::TURN_30);
    }
    if(along>cfg.entry_position_tolerance)
      return terminal(Phase::FAULT,TurnManeuverResult::NEED_REPLAN);
    command.linear_x=-cfg.turn_linear_speed;
    command.angular_z=clamp(cfg.entry_heading_gain*yaw_error+
      cfg.entry_lateral_gain*lateral,-cfg.entry_max_angular_speed,cfg.entry_max_angular_speed);
  } else if(phase_==Phase::DIAGONAL_CREEP) {
    const double c=std::cos(target.yaw),s=std::sin(target.yaw);
    const double along=(p.x-target.x)*c+(p.y-target.y)*s;
    if(along<=config_.control_dt*cfg.turn_linear_speed/2) return enter(Phase::TURN_60);
    command.linear_x=-cfg.turn_linear_speed;
    command.angular_z=clamp(cfg.entry_heading_gain*yaw_error,
      -cfg.turn_angular_speed,cfg.turn_angular_speed);
  } else {
    // 逆30/60按照实时角度从原正向角速度曲线反向取值：不是第二次正向动作。
    const double remaining=sign*NormalizeShuntAngle(p.yaw-target.yaw);
    const double tolerance=config_.control_dt*cfg.turn_min_angular_speed/2;
    if(remaining<=tolerance) {
      if(phase_==Phase::TURN_30) return enter(Phase::DIAGONAL_CREEP);
      return enter(Phase::RETURN_ENTRY);
    }
    const double angle=phase_==Phase::TURN_30 ? pi/6:pi/3;
    const double forward_target=captured_.geometry_heading+sign*(phase_==Phase::TURN_30 ? pi/2:pi/3);
    const double fraction=clamp(std::abs(NormalizeShuntAngle(forward_target-p.yaw))/angle,0.0,1.0);
    command.linear_x=-cfg.turn_linear_speed;
    command.angular_z=-sign*(cfg.turn_min_angular_speed+
      (cfg.turn_angular_speed-cfg.turn_min_angular_speed)*fraction);
  }
  return {phase_,command,TurnManeuverResult::RUNNING};
}
} // namespace visual_navigation
