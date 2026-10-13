#include "visual_navigation/pose_recovery.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <stdexcept>
#include "visual_navigation/rate_limiter.hpp"

namespace visual_navigation {
namespace {
constexpr double pi = 3.14159265358979323846;
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double distance(navigation_safety::Point a, navigation_safety::Point b) {
  return std::hypot(a.x-b.x, a.y-b.y);
}
}
void PoseRecovery::load(const std::string &path, navigation_safety::Footprint footprint,
                        const Config &config) {
  config_ = config; footprint_ = footprint; footprint_.margin = config.margin;
  const auto root = YAML::LoadFile(path);
  const double ox = root["frame"]["map_origin_m"][0].as<double>();
  const double oy = root["frame"]["map_origin_m"][1].as<double>();
  const double yaw = root["frame"]["map_x_heading_deg"].as<double>() * pi / 180.;
  const auto transform = [=](double x, double y) {
    return navigation_safety::Point{std::cos(yaw)*(x-ox)+std::sin(yaw)*(y-oy),
      -std::sin(yaw)*(x-ox)+std::cos(yaw)*(y-oy)};
  };
  free_.clear(); static_.clear(); lanes_.clear(); nodes_.clear();
  for (const auto &r : root["arena"]["free_regions"]) {
    auto a = transform(r[0].as<double>(),r[1].as<double>());
    auto b = transform(r[2].as<double>(),r[3].as<double>());
    free_.push_back({std::min(a.x,b.x),std::min(a.y,b.y),std::max(a.x,b.x),std::max(a.y,b.y)});
  }
  for (const auto &r : root["arena"]["obstacles"]) {
    navigation_safety::Polygon p;
    for (auto xy : std::array<std::pair<int,int>,4>{{{0,1},{2,1},{2,3},{0,3}}})
      p.points.push_back(transform(r[xy.first].as<double>(),r[xy.second].as<double>()));
    static_.push_back(p);
  }
  for (const auto &n : root["nodes"])
    nodes_[n.first.as<std::string>()] = transform(n.second[0].as<double>(),n.second[1].as<double>());
  for (const auto &l : root["lanes"])
    lanes_.push_back({l["id"].as<std::string>(),l["from"].as<std::string>(),
      l["to"].as<std::string>(),l["width_m"].as<double>()});
}
double PoseRecovery::active_seconds(double wall_seconds) {
  if (!paused_) active_time_ += std::max(0.0, wall_seconds-last_wall_);
  last_wall_=wall_seconds; paused_=false;
  return active_time_;
}
PoseRecovery::Pose PoseRecovery::integrate(Pose p, double v, double w, double dt) const {
  if (dt<=0.0) return p;
  // Odometry is base_link, while cmd_vel linear.x is the wheel axle velocity.
  const double ox=config_.base_offset_x, oy=config_.base_offset_y;
  p.position.x -= std::cos(p.yaw)*ox-std::sin(p.yaw)*oy;
  p.position.y -= std::sin(p.yaw)*ox+std::cos(p.yaw)*oy;
  if (std::abs(w) < 1e-9) {
    p.position.x += v*dt*std::cos(p.yaw); p.position.y += v*dt*std::sin(p.yaw);
  } else {
    p.position.x += v/w*(std::sin(p.yaw+w*dt)-std::sin(p.yaw));
    p.position.y += v/w*(std::cos(p.yaw)-std::cos(p.yaw+w*dt));
  }
  p.yaw = wrap(p.yaw+w*dt);
  p.position.x += std::cos(p.yaw)*ox-std::sin(p.yaw)*oy;
  p.position.y += std::sin(p.yaw)*ox+std::cos(p.yaw)*oy;
  return p;
}
std::vector<PoseRecovery::Pose> PoseRecovery::motion_samples(
    Pose p,double v,double target_w,double initial_w,double duration,double *final_w) const {
  std::vector<Pose> poses{p};
  double w=initial_w;
  for (double elapsed=0.0; elapsed<duration-1e-10;) {
    const double dt=std::min(config_.control_dt,duration-elapsed);
    w=LimitRate(target_w,w,config_.angular_acceleration,config_.angular_deceleration,dt);
    // Sample the actual base_link arc, not just a chord between primitive ends.
    const double speed=std::abs(v)+std::hypot(config_.base_offset_x,config_.base_offset_y)*std::abs(w);
    const int samples=std::max({1,static_cast<int>(std::ceil(speed*dt/0.003)),
      static_cast<int>(std::ceil(std::abs(w)*dt/0.035))});
    const Pose begin=p;
    for (int i=1;i<=samples;++i) poses.push_back(integrate(begin,v,w,dt*i/samples));
    p=poses.back();elapsed+=dt;
  }
  if (final_w) *final_w=w;
  return poses;
}
PoseRecovery::Pose PoseRecovery::reference(const Step &step,double elapsed) const {
  return motion_samples(step.start,step.linear,step.angular,step.initial_angular,
    std::clamp(elapsed,0.0,step.duration)).back();
}
bool PoseRecovery::safe_motion(Pose p,double v,double w,double duration) const {
  // The already limited command remains constant until the next control tick.
  return safe(motion_samples(p,v,w,w,duration));
}
bool PoseRecovery::safe(const std::vector<Pose> &poses) const {
  return navigation_safety::sweep_is_safe(poses,footprint_,free_,static_,dynamic_,0.003,0.035);
}
std::vector<mission_control_interfaces::msg::RecoveryCandidate>
PoseRecovery::candidates(const Pose &pose, const navigation_safety::Polygon &obstacle) {
  dynamic_ = {obstacle}; steps_.clear(); anchor_.clear(); blocked_lane_.clear();
  navigation_safety::Point center{};
  for (auto p : obstacle.points) { center.x+=p.x; center.y+=p.y; }
  if (obstacle.points.empty()) return {};
  center.x/=obstacle.points.size(); center.y/=obstacle.points.size();
  double best = std::numeric_limits<double>::infinity();
  for (const auto &l : lanes_) {
    auto a=nodes_.at(l.from), b=nodes_.at(l.to);
    const double dx=b.x-a.x,dy=b.y-a.y,len2=dx*dx+dy*dy;
    double t=std::clamp(((center.x-a.x)*dx+(center.y-a.y)*dy)/len2,0.,1.);
    double error=distance(center,{a.x+t*dx,a.y+t*dy});
    if (error < best) {
      best=error; blocked_lane_=l.id;
      // Select the endpoint behind the current body heading, never a past sample.
      anchor_ = (dx*std::cos(pose.yaw)+dy*std::sin(pose.yaw) > 0) ? l.from : l.to;
    }
  }
  if (best > 0.12 || distance(nodes_.at(anchor_),pose.position)>config_.max_radius) return {};
  auto a=nodes_.at(anchor_);
  std::vector<mission_control_interfaces::msg::RecoveryCandidate> result;
  for (const auto &l : lanes_) {
    if (l.id==blocked_lane_ || (l.from!=anchor_ && l.to!=anchor_)) continue;
    auto b=nodes_.at(l.from==anchor_ ? l.to : l.from);
    double yaw=std::atan2(b.y-a.y,b.x-a.x);
    mission_control_interfaces::msg::RecoveryCandidate c;
    c.candidate_id=static_cast<uint32_t>(result.size()+1);
    c.template_id="junction_pose_forward_reverse";
    c.resume_node=anchor_; c.resume_lane=l.id; c.resume_heading=yaw;
    c.borrowed_lane_ids={l.id};
    c.exit_pose.position.x=a.x+config_.exit_distance*std::cos(yaw);
    c.exit_pose.position.y=a.y+config_.exit_distance*std::sin(yaw);
    c.exit_pose.orientation.z=std::sin(yaw/2); c.exit_pose.orientation.w=std::cos(yaw/2);
    c.motion_cost=distance(pose.position,{c.exit_pose.position.x,c.exit_pose.position.y});
    if (safe({{{c.exit_pose.position.x,c.exit_pose.position.y},yaw}})) result.push_back(c);
  }
  return result;
}
bool PoseRecovery::plan(const Pose &start,
    const mission_control_interfaces::msg::RecoveryCandidate &candidate,
    const std::vector<std::string> &blocked) {
  steps_.clear();last_expansions_=0;
  const auto forbidden=[&](const std::string &lane) {
    return std::find(blocked.begin(),blocked.end(),lane)!=blocked.end();
  };
  if(forbidden(candidate.resume_lane) ||
      std::any_of(candidate.borrowed_lane_ids.begin(),candidate.borrowed_lane_ids.end(),forbidden)) return false;
  // Other blocked roads are forbidden even when they would shorten a shunt.
  dynamic_.resize(std::min<std::size_t>(1,dynamic_.size()));
  for (const auto &l : lanes_) {
    if (l.id==blocked_lane_ || std::find(blocked.begin(),blocked.end(),l.id)==blocked.end()) continue;
    auto a=nodes_.at(l.from),b=nodes_.at(l.to);
    double yaw=std::atan2(b.y-a.y,b.x-a.x),nx=-std::sin(yaw)*l.width/2,ny=std::cos(yaw)*l.width/2;
    dynamic_.push_back({{{a.x+nx,a.y+ny},{b.x+nx,b.y+ny},{b.x-nx,b.y-ny},{a.x-nx,a.y-ny}}});
  }
  const Pose goal{{candidate.exit_pose.position.x,candidate.exit_pose.position.y},candidate.resume_heading};
  const auto anchor=nodes_.at(anchor_);
  if (!safe({start}) || !safe({goal})) return false;
  struct Node { Pose p; double g; int parent; double v,w; int direction; double final_w; };
  std::vector<Node> nodes{{start,0.,-1,0.,0.,0,0.}};
  using QueueEntry=std::pair<double,int>;
  std::priority_queue<QueueEntry,std::vector<QueueEntry>,std::greater<QueueEntry>> queue;
  auto heuristic=[&](Pose p) {return 1.6*distance(p.position,goal.position)+0.055*std::abs(wrap(p.yaw-goal.yaw));};
  auto key=[&](Pose p,int dir,double angular) {
    const auto x=static_cast<int64_t>(std::llround((p.position.x-anchor.x)/0.004))+512;
    const auto y=static_cast<int64_t>(std::llround((p.position.y-anchor.y)/0.004))+512;
    const auto a=static_cast<int64_t>(std::llround((wrap(p.yaw)+pi)/0.045));
    const auto rate=static_cast<int64_t>(std::llround(angular/0.15))+8;
    return ((((x*1024+y)*160+a)*2+(dir>0))*32+rate);
  };
  std::unordered_map<int64_t,double> costs;
  queue.push({heuristic(start),0}); int found=-1;
  const double dt=config_.primitive_s;
  for (int expanded=0;!queue.empty() && expanded<config_.max_expansions;++expanded) {
    last_expansions_=expanded+1;
    int index=queue.top().second;queue.pop(); const Node n=nodes[index];
    if (distance(n.p.position,goal.position)<0.009 && std::abs(wrap(n.p.yaw-goal.yaw))<0.065 && n.direction>0) { found=index;break; }
    for (int dir : {1,-1}) for (double fraction : {0.,-1.,-.5,.5,1.}) {
      double v=dir*config_.speed,w=fraction*config_.angular;
      const double initial_w=(n.direction!=dir ? 0.0 : n.final_w);
      double final_w=initial_w;
      const auto samples=motion_samples(n.p,v,w,initial_w,dt,&final_w);
      const Pose end=samples.back();
      if (distance(end.position,anchor)>config_.max_radius) continue;
      const double g=n.g+config_.speed*dt*(dir<0 ? 1.08:1.)+
        (n.direction!=0 && dir!=n.direction ? 0.028:0.)+std::abs(w)*dt*0.003;
      const auto k=key(end,dir,final_w); auto it=costs.find(k);
      if (it!=costs.end() && it->second<=g) continue;
      if (!safe(samples)) continue;
      costs[k]=g; nodes.push_back({end,g,index,v,w,dir,final_w});
      queue.push({g+heuristic(end),static_cast<int>(nodes.size()-1)});
    }
  }
  if (found<0) return false;
  for (int i=found;nodes[i].parent>=0;i=nodes[i].parent) {
    const auto &n=nodes[i];const auto &parent=nodes[n.parent];
    steps_.push_back({parent.p,n.p,n.v,n.w,dt,parent.direction!=n.direction ? 0.0 : parent.final_w});
  }
  std::reverse(steps_.begin(),steps_.end());
  return !steps_.empty();
}
} // namespace visual_navigation
