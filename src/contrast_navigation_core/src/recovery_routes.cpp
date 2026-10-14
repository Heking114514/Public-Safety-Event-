#include "visual_navigation/pose_recovery.hpp"
#include "visual_navigation/rate_limiter.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace visual_navigation {
namespace {
constexpr double pi = 3.14159265358979323846;
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double length(navigation_safety::Point a, navigation_safety::Point b) {
  return std::hypot(a.x-b.x, a.y-b.y);
}
}

void PoseRecovery::set_dynamic_obstacles(
    const std::vector<navigation_safety::Polygon> &obstacles) {
  observed_ = obstacles;
  live_obstacles_set_ = true;
  dynamic_ = observed_;
}

void PoseRecovery::set_incoming_lane(const std::string &lane,
                                     navigation_safety::Point junction) {
  incoming_override_ = lane;
  junction_override_ = junction;
}

PoseRecovery::RoadJunction PoseRecovery::junction_type(const std::string &node) const {
  const auto center=nodes_.find(node);
  if (center==nodes_.end()) return RoadJunction::INVALID;
  std::vector<double> headings;
  for (const auto &lane : lanes_) {
    if (lane.from!=node && lane.to!=node) continue;
    const auto other=nodes_.at(lane.from==node ? lane.to : lane.from);
    if (length(center->second,other)<1e-9) return RoadJunction::INVALID;
    const double heading=std::atan2(other.y-center->second.y,other.x-center->second.x);
    // Duplicate logical edges do not create another physical road mouth.
    if (std::none_of(headings.begin(),headings.end(),[&](double h) {
      return std::abs(wrap(h-heading))<0.01;
    })) headings.push_back(heading);
  }
  if (headings.size()==1) return RoadJunction::DEAD_END;
  if (headings.size()<2 || headings.size()>4) return RoadJunction::INVALID;
  int opposite_pairs=0, quarter_pairs=0;
  for (std::size_t i=0; i<headings.size(); ++i) {
    for (std::size_t j=i+1; j<headings.size(); ++j) {
      const double delta=std::abs(wrap(headings[i]-headings[j]));
      if (std::abs(delta-pi)<0.01) ++opposite_pairs;
      else if (std::abs(delta-pi/2)<0.01) ++quarter_pairs;
      else return RoadJunction::INVALID;
    }
  }
  if (headings.size()==2)
    return opposite_pairs==1 ? RoadJunction::STRAIGHT : RoadJunction::CORNER;
  if (headings.size()==3 && opposite_pairs==1 && quarter_pairs==2) return RoadJunction::T;
  if (headings.size()==4 && opposite_pairs==2 && quarter_pairs==4) return RoadJunction::CROSS;
  return RoadJunction::INVALID;
}

PoseRecovery::RoadJunction PoseRecovery::junction_at(navigation_safety::Point point) const {
  // Context junctions are mapped points; this is not a nearest-road guess.
  for (const auto &node : nodes_)
    if (length(node.second,point)<1e-6) return junction_type(node.first);
  return RoadJunction::INVALID;
}

PoseRecovery::Pose PoseRecovery::turn_entry(navigation_safety::Point junction,
                                           double incoming, double outgoing) const {
  const double lateral=std::copysign(1.0,wrap(outgoing-incoming))*config_.turn.entry_target_y;
  const double c=std::cos(incoming), s=std::sin(incoming);
  return {{junction.x+config_.turn.entry_target_x*c-lateral*s,
           junction.y+config_.turn.entry_target_x*s+lateral*c},incoming};
}

std::string PoseRecovery::lane_at_pose(const Pose &pose) const {
  double best = std::numeric_limits<double>::infinity();
  std::string result;
  for (const auto &lane : lanes_) {
    const auto a=nodes_.at(lane.from), b=nodes_.at(lane.to);
    const double d=length(a,b);
    if (d < 1e-9) continue;
    const double h=std::atan2(b.y-a.y,b.x-a.x);
    const double heading_error=std::min(std::abs(wrap(pose.yaw-h)),
                                      std::abs(wrap(pose.yaw-h-pi)));
    if (heading_error > 0.25) continue;
    const double along=((pose.position.x-a.x)*(b.x-a.x)+
                        (pose.position.y-a.y)*(b.y-a.y))/(d*d);
    if (along < -0.01 || along > 1.01) continue;
    const double error=std::abs(-(pose.position.x-a.x)*std::sin(h)+
                                (pose.position.y-a.y)*std::cos(h));
    if (error <= lane.width/2 && error < best) { best=error; result=lane.id; }
  }
  return result;
}

bool PoseRecovery::exit_matches(const Pose &pose,
    const mission_control_interfaces::msg::RecoveryCandidate &candidate) const {
  const auto it=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane &lane) {
    return lane.id == candidate.resume_lane;
  });
  const auto node=nodes_.find(candidate.resume_node);
  if (it == lanes_.end() || node == nodes_.end() ||
      (it->from != candidate.resume_node && it->to != candidate.resume_node)) return false;
  const auto other=nodes_.at(it->from == candidate.resume_node ? it->to : it->from);
  const double heading=std::atan2(other.y-node->second.y,other.x-node->second.x);
  const double dx=pose.position.x-node->second.x, dy=pose.position.y-node->second.y;
  const double along=dx*std::cos(heading)+dy*std::sin(heading);
  const double lateral=-dx*std::sin(heading)+dy*std::cos(heading);
  return std::abs(wrap(candidate.resume_heading-heading)) < 0.01 &&
    std::abs(wrap(pose.yaw-heading)) <= config_.turn.entry_yaw_tolerance+0.02 &&
    along > 0.005 && along < length(node->second,other) &&
    std::abs(lateral) <= std::min(0.04, it->width/2) && safe({pose});
}

bool PoseRecovery::braking_safe(Pose pose, double v, double w, bool adjustment) const {
  while (std::abs(v)>1e-9 || std::abs(w)>1e-9) {
    v=LimitRate(0.0,v,config_.linear_acceleration,config_.linear_deceleration,config_.control_dt);
    w=LimitRate(0.0,w,config_.angular_acceleration,config_.angular_deceleration,config_.control_dt);
    const auto sweep=motion_samples(pose,v,w,w,config_.control_dt);
    if (!(adjustment ? adjustment_safe(sweep) : safe(sweep))) return false;
    pose=sweep.back();
  }
  return true;
}

bool PoseRecovery::append_straight(Pose &pose, Pose target, double direction,
                                  const std::string &phase, const std::string &lane) {
  const Pose start=pose;
  const double distance=length(pose.position,target.position);
  const double duration=distance/config_.speed+config_.turn.phase_timeout;
  const double c=std::cos(target.yaw), s=std::sin(target.yaw);
  const bool adjustment=phase=="FORWARD_ADJUSTMENT";
  if (adjustment && !adjustment_safe({pose})) return false;
  double v=0.0, w=0.0, elapsed=0.0;
  bool complete=false;
  while (elapsed<=duration) {
    const double dx=target.position.x-pose.position.x;
    const double dy=target.position.y-pose.position.y;
    const double along=dx*c+dy*s;
    const double lateral=-(pose.position.x-target.position.x)*s+
                          (pose.position.y-target.position.y)*c;
    const double yaw_error=wrap(target.yaw-pose.yaw);
    if (std::abs(along)<=config_.turn.entry_position_tolerance &&
        std::abs(lateral)<=config_.turn.entry_lateral_tolerance &&
        std::abs(yaw_error)<=config_.turn.entry_yaw_tolerance) { complete=true; break; }
    if (along*direction < -config_.turn.entry_position_tolerance) return false;
    const double target_w=std::clamp(config_.turn.entry_heading_gain*yaw_error-
      direction*config_.turn.entry_lateral_gain*lateral,
      -config_.turn.entry_max_angular_speed,config_.turn.entry_max_angular_speed);
    v=LimitRate(direction*config_.speed,v,config_.linear_acceleration,
                config_.linear_deceleration,config_.control_dt);
    if (std::abs(v)<config_.minimum_linear_speed)
      v=direction*config_.minimum_linear_speed;
    w=LimitRate(target_w,w,config_.angular_acceleration,
                config_.angular_deceleration,config_.control_dt);
    const auto sweep=motion_samples(pose,v,w,w,config_.control_dt);
    if (!(adjustment ? adjustment_safe(sweep) : safe(sweep)) ||
        !braking_safe(sweep.back(),v,w,adjustment)) return false;
    pose=sweep.back(); elapsed+=config_.control_dt;
  }
  if (!complete) return false;
  Step step{};
  step.start=start; step.end=pose; step.target=target;
  step.linear=direction*config_.speed; step.duration=duration; step.phase=phase;
  step.lane_id=lane; step.road_origin=target.position;
  step.geometry_heading=target.yaw; step.target_heading=target.yaw;
  steps_.push_back(step);
  return true;
}

bool PoseRecovery::append_turn(Pose &pose, navigation_safety::Point junction,
                              double incoming, double outgoing, bool reverse_arrival,
                              const std::string &lane) {
  if (std::abs(std::abs(wrap(outgoing-incoming))-pi/2) > 0.01) return false;
  TurnManeuverController controller(config_.turn);
  controller.begin(0.0);
  Pose start=pose;
  double v=0.0, w=0.0, elapsed=0.0;
  bool complete=false;
  const double limit=6*config_.turn.phase_timeout;
  while (elapsed <= limit) {
    TurnManeuverInput input;
    input.context.pose={pose.position.x,pose.position.y,pose.yaw};
    input.context.junction_x=junction.x; input.context.junction_y=junction.y;
    input.context.geometry_heading=incoming; input.context.arrival_heading=incoming;
    input.context.target_heading=outgoing; input.context.reverse_arrival=reverse_arrival;
    input.context.applied_twist={v,w}; input.context.measured_twist={v,w};
    input.context.measured_yaw_rate_valid=true;
    const auto result=controller.update(input,elapsed);
    if (result.result == TurnManeuverResult::COMPLETED) { complete=true; break; }
    if (result.result != TurnManeuverResult::RUNNING) return false;
    // The executor publishes an immediate zero command at phase boundaries.
    if (std::abs(result.command.linear_x)<=1e-9 && std::abs(result.command.angular_z)<=1e-9) {
      v=w=0.0; elapsed+=config_.control_dt; continue;
    }
    v=LimitRate(result.command.linear_x,v,config_.linear_acceleration,
                config_.linear_deceleration,config_.control_dt);
    if (std::abs(result.command.linear_x)>1e-9 && std::abs(v)<config_.minimum_linear_speed)
      v=std::copysign(config_.minimum_linear_speed,result.command.linear_x);
    w=LimitRate(result.command.angular_z,w,config_.angular_acceleration,
                config_.angular_deceleration,config_.control_dt);
    const auto sweep=motion_samples(pose,v,w,w,config_.control_dt);
    if (!safe(sweep) || !braking_safe(sweep.back(),v,w)) return false;
    pose=sweep.back(); elapsed+=config_.control_dt;
  }
  if (!complete) return false;
  Step step{};
  step.start=start; step.end=pose; step.target=pose;
  step.linear=config_.turn.turn_linear_speed; step.duration=elapsed+2.0;
  step.phase="TURN_INTO_EXIT";
  step.kind=Step::Kind::FIXED_TURN;
  step.junction=junction; step.geometry_heading=incoming; step.target_heading=outgoing;
  step.reverse_arrival=reverse_arrival; step.lane_id=lane; step.road_origin=junction;
  steps_.push_back(step);
  return true;
}

std::vector<mission_control_interfaces::msg::RecoveryCandidate>
PoseRecovery::candidates(const Pose &pose, const navigation_safety::Polygon &obstacle) {
  if (!live_obstacles_set_) observed_={obstacle};
  dynamic_=observed_;
  steps_.clear(); anchor_.clear(); blocked_lane_.clear();
  unsafe_entry_rejected_=false;
  if (obstacle.points.empty()) return {};
  navigation_safety::Point center{};
  for (const auto &p : obstacle.points) { center.x+=p.x; center.y+=p.y; }
  center.x/=obstacle.points.size(); center.y/=obstacle.points.size();
  double best=std::numeric_limits<double>::infinity();
  for (const auto &lane : lanes_) {
    const auto a=nodes_.at(lane.from), b=nodes_.at(lane.to);
    const double d=length(a,b);
    const double t=std::clamp(((center.x-a.x)*(b.x-a.x)+(center.y-a.y)*(b.y-a.y))/(d*d),0.0,1.0);
    const double error=length(center,{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)});
    if (error < best) { best=error; blocked_lane_=lane.id; }
  }
  if (best > 0.12) return {};
  const std::string incoming=incoming_override_.empty() ? lane_at_pose(pose) : incoming_override_;
  auto lane=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane &l){return l.id==incoming;});
  if (lane == lanes_.end()) return {};
  // A turn interrupted at a corner returns to its original road first. Its
  // recovery junction is still the endpoint behind the live road heading.
  const auto from=nodes_.at(lane->from), to=nodes_.at(lane->to);
  anchor_=((to.x-from.x)*std::cos(pose.yaw)+(to.y-from.y)*std::sin(pose.yaw)>0) ?
    lane->from : lane->to;
  double incoming_heading=std::atan2(
    nodes_.at(lane->from==anchor_ ? lane->to : lane->from).y-nodes_.at(anchor_).y,
    nodes_.at(lane->from==anchor_ ? lane->to : lane->from).x-nodes_.at(anchor_).x);
  if (std::abs(wrap(pose.yaw-incoming_heading))>pi/2)
    incoming_heading=wrap(incoming_heading+pi);
  std::vector<const Lane *> exits;
  for (const auto &l : lanes_) {
    if (l.from==anchor_ || l.to==anchor_) exits.push_back(&l);
  }
  const auto topology=junction_type(anchor_);
  if (topology!=RoadJunction::T && topology!=RoadJunction::CROSS) return {};
  const std::string junction=topology==RoadJunction::CROSS ? "x_junction" : "t_junction";
  std::vector<mission_control_interfaces::msg::RecoveryCandidate> result;
  bool unsafe_entry=false;
  for (const auto *exit : exits) {
    if (exit->id==blocked_lane_) continue;
    const auto a=nodes_.at(anchor_), b=nodes_.at(exit->from==anchor_ ? exit->to : exit->from);
    const double h=std::atan2(b.y-a.y,b.x-a.x), delta=wrap(h-incoming_heading);
    const bool reverse=std::abs(std::abs(delta)-pi)<0.01;
    if (!reverse && std::abs(std::abs(delta)-pi/2)>0.01) continue;
    mission_control_interfaces::msg::RecoveryCandidate candidate;
    candidate.candidate_id=static_cast<uint32_t>(result.size()+1);
    candidate.template_id=junction+(reverse ? "_reverse" : (delta>0 ? "_left" : "_right"));
    candidate.resume_node=anchor_; candidate.resume_lane=exit->id; candidate.resume_heading=h;
    candidate.borrowed_lane_ids={exit->id};
    candidate.exit_pose.position.x=a.x+config_.exit_distance*std::cos(h);
    candidate.exit_pose.position.y=a.y+config_.exit_distance*std::sin(h);
    candidate.exit_pose.orientation.z=std::sin(h/2); candidate.exit_pose.orientation.w=std::cos(h/2);
    std::vector<const Lane *> branches;
    if (reverse) {
      for (const auto *branch : exits) {
        if (branch->id==blocked_lane_ || branch->id==incoming || branch==exit) continue;
        const auto p=nodes_.at(branch->from==anchor_ ? branch->to : branch->from);
        const double bh=std::atan2(p.y-a.y,p.x-a.x);
        if (std::abs(std::abs(wrap(bh-incoming_heading))-pi/2)<0.01)
          branches.push_back(branch);
      }
    } else branches.push_back(exit);
    for (const auto *branch : branches) {
      auto option=candidate;
      option.candidate_id=static_cast<uint32_t>(result.size()+1);
      option.borrowed_lane_ids=reverse ? std::vector<std::string>{branch->id,exit->id} :
                                        std::vector<std::string>{exit->id};
      if (!plan(pose,option,{})) {
        unsafe_entry=unsafe_entry || unsafe_entry_rejected_;
        continue;
      }
      const auto end=steps_.back().end;
      option.exit_pose.position.x=end.position.x; option.exit_pose.position.y=end.position.y;
      double cost=0;
      for (const auto &step : steps_) cost+=std::abs(step.linear)*step.duration;
      option.motion_cost=cost;
      if (!reverse) {
        // The first retreat can use the through road on the other side of the junction.
        for (const auto *opposite : exits) {
          const auto p=nodes_.at(opposite->from==anchor_ ? opposite->to : opposite->from);
          const double oh=std::atan2(p.y-a.y,p.x-a.x);
          if (std::abs(std::abs(wrap(oh-incoming_heading))-pi)<0.01 &&
              opposite->id!=exit->id && opposite->id!=blocked_lane_)
            option.borrowed_lane_ids.push_back(opposite->id);
        }
      }
      result.push_back(option);
    }
  }
  steps_.clear();
  unsafe_entry_rejected_=unsafe_entry;
  return result;
}

bool PoseRecovery::plan_direct(const Pose &start,
    const mission_control_interfaces::msg::RecoveryCandidate &candidate,
    const std::vector<std::string> &blocked) {
  steps_.clear(); last_expansions_=0;
  const auto forbidden=[&](const std::string &id) {
    return std::find(blocked.begin(),blocked.end(),id)!=blocked.end();
  };
  if (candidate.resume_node!=anchor_ || nodes_.find(anchor_)==nodes_.end() ||
      forbidden(candidate.resume_lane) ||
      std::any_of(candidate.borrowed_lane_ids.begin(),candidate.borrowed_lane_ids.end(),forbidden)) return false;
  const auto topology=junction_type(anchor_);
  if (topology!=RoadJunction::T && topology!=RoadJunction::CROSS) return false;
  dynamic_=observed_;
  for (const auto &lane : lanes_) {
    if (lane.id==blocked_lane_ || !forbidden(lane.id)) continue;
    auto a=nodes_.at(lane.from), b=nodes_.at(lane.to);
    const double h=std::atan2(b.y-a.y,b.x-a.x);
    const double d=length(a,b);
    if (d <= lane.width) continue;
    // A blocked edge does not make the shared junction itself an obstacle.
    a.x+=lane.width/2*std::cos(h); a.y+=lane.width/2*std::sin(h);
    b.x-=lane.width/2*std::cos(h); b.y-=lane.width/2*std::sin(h);
    const double nx=-std::sin(h)*lane.width/2, ny=std::cos(h)*lane.width/2;
    dynamic_.push_back({{{a.x+nx,a.y+ny},{b.x+nx,b.y+ny},
                         {b.x-nx,b.y-ny},{a.x-nx,a.y-ny}}});
  }
  if (!safe({start})) return false;
  const auto junction=nodes_.at(anchor_);
  const std::string incoming=incoming_override_.empty() ? lane_at_pose(start) : incoming_override_;
  const auto lane=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane &l){return l.id==incoming;});
  if (lane==lanes_.end()) return false;
  const auto other=nodes_.at(lane->from==anchor_ ? lane->to : lane->from);
  double heading=std::atan2(other.y-junction.y,other.x-junction.x);
  if (std::abs(wrap(start.yaw-heading))>pi/2) heading=wrap(heading+pi);
  if (std::abs(wrap(start.yaw-heading))>0.25) return false;
  const auto exit=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane &l) {
    return l.id==candidate.resume_lane && (l.from==anchor_ || l.to==anchor_);
  });
  if (exit==lanes_.end() || candidate.resume_lane==blocked_lane_) return false;
  const auto exit_other=nodes_.at(exit->from==anchor_ ? exit->to : exit->from);
  const double outgoing=std::atan2(exit_other.y-junction.y,exit_other.x-junction.x);
  if (std::abs(wrap(outgoing-candidate.resume_heading))>0.01) return false;
  const bool reverse=std::abs(std::abs(wrap(outgoing-heading))-pi)<0.01;
  double first_heading=outgoing;
  std::string first_lane=candidate.resume_lane;
  if (reverse) {
    if (candidate.borrowed_lane_ids.size()!=2 ||
        candidate.borrowed_lane_ids.back()!=candidate.resume_lane ||
        candidate.borrowed_lane_ids.front()==candidate.resume_lane) return false;
    first_lane=candidate.borrowed_lane_ids.front();
    const auto branch=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane &l) {
      return l.id==first_lane && (l.from==anchor_ || l.to==anchor_);
    });
    if (branch==lanes_.end() || first_lane==blocked_lane_) return false;
    const auto p=nodes_.at(branch->from==anchor_ ? branch->to : branch->from);
    first_heading=std::atan2(p.y-junction.y,p.x-junction.x);
    if (std::abs(std::abs(wrap(first_heading-heading))-pi/2)>0.01) return false;
  } else if (std::abs(std::abs(wrap(outgoing-heading))-pi/2)>0.01) return false;
  Pose pose=start;
  const Pose turn_point=turn_entry(junction,heading,first_heading);
  if (!safe({turn_point})) {
    unsafe_entry_rejected_=true;
    steps_.clear(); return false;
  }
  const double backward=(start.position.x-turn_point.position.x)*std::cos(heading)+
                        (start.position.y-turn_point.position.y)*std::sin(heading);
  if (backward < -config_.turn.entry_position_tolerance ||
      !append_straight(pose,turn_point,-1,"RETREAT_TO_TURN_POINT",incoming) ||
      !append_turn(pose,junction,heading,first_heading,true,first_lane)) {
    steps_.clear(); return false;
  }
  if (reverse) {
    const double current=(pose.position.x-junction.x)*std::cos(first_heading)+
                         (pose.position.y-junction.y)*std::sin(first_heading);
    const double advance=std::max(0.0,config_.exit_distance-current);
    Pose branch_point{{junction.x+(current+advance)*std::cos(first_heading),
                        junction.y+(current+advance)*std::sin(first_heading)},first_heading};
    if (!append_straight(pose,branch_point,1,"ADVANCE_TO_SECOND_TURN_POINT",first_lane)) {
      steps_.clear(); return false;
    }
    const auto stop=[&]() {
      Step step{};
      step.start=step.end=step.target=pose; step.duration=0.5;
      step.phase="DIRECTION_STOP";
      step.kind=Step::Kind::STOP; step.lane_id=first_lane;
      steps_.push_back(step);
    };
    stop();
    // The same mapped junction is reused; no displaced junction is invented.
    // The short retreat is checked by the complete swept vehicle footprint below.
    const auto branch=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane &l) {
      return l.id==first_lane;
    });
    if (branch==lanes_.end()) {
      steps_.clear(); return false;
    }
    const Pose second_point=turn_entry(junction,first_heading,outgoing);
    if (!append_straight(pose,second_point,-1,"RETREAT_TO_SECOND_TURN_POINT",first_lane)) {
      steps_.clear(); return false;
    }
    stop();
    if (!append_turn(pose,junction,first_heading,outgoing,true,candidate.resume_lane)) {
      steps_.clear(); return false;
    }
  }
  const double along=(pose.position.x-junction.x)*std::cos(candidate.resume_heading)+
                     (pose.position.y-junction.y)*std::sin(candidate.resume_heading);
  if (along < config_.exit_distance) {
    const double extension=config_.exit_distance-along;
    Pose goal{{junction.x+(along+extension)*std::cos(candidate.resume_heading),
               junction.y+(along+extension)*std::sin(candidate.resume_heading)},candidate.resume_heading};
    if (!append_straight(pose,goal,1,"FORWARD_TO_EXIT",candidate.resume_lane)) {
      steps_.clear(); return false;
    }
    steps_.back().road_origin=junction;
  }
  if (!exit_matches(pose,candidate)) { steps_.clear(); return false; }
  last_expansions_=static_cast<int>(steps_.size());
  return true;
}
} // namespace visual_navigation
