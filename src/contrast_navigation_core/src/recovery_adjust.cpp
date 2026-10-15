#include "visual_navigation/pose_recovery.hpp"
#include <algorithm>
#include <cmath>

namespace visual_navigation {
namespace {
double segment_distance(navigation_safety::Point p, navigation_safety::Point a,
                        navigation_safety::Point b) {
  const double dx=b.x-a.x,dy=b.y-a.y,length=dx*dx+dy*dy;
  const double t=length>0 ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/length,0.,1.) : 0.;
  return std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
}
}

double PoseRecovery::obstacle_clearance(const Pose &pose) const {
  auto physical=footprint_; physical.margin=0;
  const navigation_safety::Polygon body{navigation_safety::footprint_corners(pose,physical)};
  double clearance=std::numeric_limits<double>::infinity();
  for (const auto &obstacle:dynamic_) {
    if (obstacle.points.size()<3) continue;
    if (navigation_safety::polygons_intersect(body,obstacle)) return 0;
    for (std::size_t i=0;i<body.points.size();++i)
      for (std::size_t j=0;j<obstacle.points.size();++j) {
        const auto a=body.points[i],b=body.points[(i+1)%body.points.size()];
        const auto c=obstacle.points[j],d=obstacle.points[(j+1)%obstacle.points.size()];
        clearance=std::min({clearance,segment_distance(a,c,d),segment_distance(b,c,d),
          segment_distance(c,a,b),segment_distance(d,a,b)});
      }
  }
  return clearance;
}

bool PoseRecovery::adjustment_safe(const std::vector<Pose> &poses) const {
  return safe(poses) && std::all_of(poses.begin(),poses.end(),[&](const Pose &pose) {
    return obstacle_clearance(pose)>=config_.forward_obstacle_clearance_m;
  });
}

bool PoseRecovery::plan(const Pose &start,
    const mission_control_interfaces::msg::RecoveryCandidate &candidate,
    const std::vector<std::string> &blocked) {
  unsafe_entry_rejected_=false;
  if (plan_direct(start,candidate,blocked)) return true;
  steps_.clear();
  // 固定入口本身放不下车时，前移并不能使这个入口安全。
  if (unsafe_entry_rejected_ || !adjustment_safe({start})) return false;
  const auto incoming=incoming_override_.empty() ? lane_at_pose(start) : incoming_override_;
  const auto road=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane &lane) {
    return lane.id==incoming;
  });
  if (road==lanes_.end() ||
      (incoming!=blocked_lane_ &&
       std::find(blocked.begin(),blocked.end(),incoming)!=blocked.end())) return false;
  const auto a=nodes_.at(road->from), b=nodes_.at(road->to);
  double heading=std::atan2(b.y-a.y,b.x-a.x);
  if (std::cos(heading-start.yaw)<0) heading=std::atan2(a.y-b.y,a.x-b.x);
  if (std::abs(std::atan2(std::sin(heading-start.yaw),std::cos(heading-start.yaw)))>.25)
    return false;
  const double c=std::cos(heading),s=std::sin(heading);
  const double along=(start.position.x-a.x)*c+(start.position.y-a.y)*s;
  double limit=std::max((a.x-start.position.x)*c+(a.y-start.position.y)*s,
                        (b.x-start.position.x)*c+(b.y-start.position.y)*s);
  // 上界由本段道路和障碍决定；额外预留控制周期、入口容差与制动位移。
  const double reserve=config_.turn.entry_position_tolerance+config_.speed*config_.control_dt+
    config_.speed*config_.speed/(2*config_.linear_deceleration);
  const double front=footprint_.center_offset_x+footprint_.length/2;
  const double half_width=footprint_.width/2+config_.forward_obstacle_clearance_m;
  for (const auto &obstacle:dynamic_) {
    double near=std::numeric_limits<double>::infinity(),far=-near,lo=near,hi=far;
    for (const auto &p:obstacle.points) {
      const double x=(p.x-start.position.x)*c+(p.y-start.position.y)*s;
      const double y=-(p.x-a.x)*s+(p.y-a.y)*c-footprint_.center_offset_y;
      near=std::min(near,x); far=std::max(far,x); lo=std::min(lo,y); hi=std::max(hi,y);
    }
    if (far>=0 && lo<=half_width && hi>=-half_width)
      limit=std::min(limit,near-front-config_.forward_obstacle_clearance_m-reserve);
  }
  // 从短到长尝试，选足以恢复的最小前移量，无须每次贴近障碍。
  const double increment=2*config_.turn.entry_position_tolerance;
  for (double distance=std::min(increment,limit);distance>0;) {
    const Pose target{{a.x+(along+distance)*c,a.y+(along+distance)*s},heading};
    Pose adjusted=start;
    steps_.clear();
    if (append_straight(adjusted,target,1,"FORWARD_ADJUSTMENT",incoming)) {
      const auto adjustment=steps_.back();
      if (plan_direct(adjusted,candidate,blocked)) {
        Step stop{};
        stop.start=stop.end=stop.target=adjusted;
        stop.kind=Step::Kind::STOP; stop.phase="DIRECTION_STOP";
        stop.duration=.5; stop.lane_id=incoming;
        steps_.insert(steps_.begin(),stop);
        steps_.insert(steps_.begin(),adjustment);
        last_expansions_=static_cast<int>(steps_.size());
        return true;
      }
    }
    if (unsafe_entry_rejected_ || distance>=limit) break;
    distance=std::min(distance+increment,limit);
  }
  steps_.clear();
  return false;
}
} // namespace visual_navigation
