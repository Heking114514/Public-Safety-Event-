// Standalone feasibility experiment. Does not modify or invoke a navigation controller.
// Compile with the repository's unmodified motion.cpp and checker.cpp; see run_validation.py.
#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "vehicle_loopback_sim/motion.hpp"
#include "navigation_safety/checker.hpp"
#include "visual_navigation/rate_limiter.hpp"
using vehicle_loopback_sim::Pose2d;
using vehicle_loopback_sim::Twist2d;
constexpr double pi=3.14159265358979323846;
struct Config {
  double length,width,offset,track,margin,speed,omega,accel,lin_up,lin_down,ang_up,ang_down,dt;
  double half_corridor=.1,window=.2,tolerance=3*pi/180,turn_reserve=.001;
  static Config load(const std::filesystem::path &root) {
    const auto p=YAML::LoadFile((root/"src/vehicle_config/config/vehicle_physical.yaml").string());
    const auto c=YAML::LoadFile((root/"src/contrast_controller/config/contrast_controller.yaml").string())["contrast_controller"]["ros__parameters"];
    const auto n=YAML::LoadFile((root/"src/visual_navigation/config/waypoint_navigation.yaml").string())["waypoint_navigator"]["ros__parameters"];
    Config out;
    out.length=p["chassis"]["length_m"].as<double>();out.width=p["chassis"]["width_m"].as<double>();
    out.offset=p["differential_drive"]["base_link_offset_x_m"].as<double>();
    if(std::abs(p["differential_drive"]["base_link_offset_y_m"].as<double>())>1e-12) throw std::runtime_error("This experiment assumes offset_y=0");
    out.track=p["differential_drive"]["wheel_track_m"].as<double>();out.margin=c["recovery_margin_m"].as<double>();
    out.speed=c["recovery_linear_speed"].as<double>();out.omega=c["recovery_angular_speed"].as<double>();
    out.lin_up=n["max_linear_acceleration"].as<double>();out.lin_down=n["max_linear_deceleration"].as<double>();
    out.ang_up=n["max_angular_acceleration"].as<double>();out.ang_down=n["max_angular_deceleration"].as<double>();
    out.accel=std::min(out.ang_up,out.ang_down);out.dt=1/n["control_frequency"].as<double>();
    if(std::abs(out.speed-n["min_linear_command_speed"].as<double>())>1e-12) throw std::runtime_error("Expected the actual deadband driving speed");
    return out;
  }
  navigation_safety::Footprint footprint() const{return {length,width,0,0,margin};}
  double initial_limit(double theta) const{return half_corridor-(length/2+margin)*std::abs(std::sin(theta))-(width/2+margin)*std::abs(std::cos(theta));}
};
struct Bounds {
  double xmin=std::numeric_limits<double>::infinity(),xmax=-xmin,ymin=xmin,ymax=-xmin;
  void add(double x,double y){xmin=std::min(xmin,x);xmax=std::max(xmax,x);ymin=std::min(ymin,y);ymax=std::max(ymax,y);}
  void merge(const Bounds &b){add(b.xmin,b.ymin);add(b.xmax,b.ymax);}
};
Bounds point_bounds(const Pose2d &p,const Config &c){
  Bounds out;for(auto q:navigation_safety::footprint_corners({{p.x,p.y},p.yaw},c.footprint()))out.add(q.x,q.y);return out;
}
// Exact extrema of EVERY inflated corner during each constant-command interval.
// Coordinate is A*cos(theta)+B*sin(theta)+C: check endpoints and all derivative roots.
Bounds interval_bounds(const Pose2d &p,const Twist2d &u,const Config &c,double duration){
  Bounds out=point_bounds(p,c);
  const auto end=vehicle_loopback_sim::integrate_base_link(p,u,duration,c.offset,0);out.merge(point_bounds(end,c));
  if(std::abs(u.angular_z)<1e-12)return out;
  const double r=u.linear_x/u.angular_z;
  const double axle_x=p.x-c.offset*std::cos(p.yaw),axle_y=p.y-c.offset*std::sin(p.yaw);
  const double lo=std::min(p.yaw,end.yaw),hi=std::max(p.yaw,end.yaw);
  const double a=c.length/2+c.margin,b=c.width/2+c.margin;
  for(int sx:{-1,1})for(int sy:{-1,1}){
    const double cx=c.offset+sx*a,cy=sy*b;
    auto add_at=[&](double th){out.add(axle_x-r*std::sin(p.yaw)+cx*std::cos(th)+(r-cy)*std::sin(th),axle_y+r*std::cos(p.yaw)+(cy-r)*std::cos(th)+cx*std::sin(th));};
    const double roots[]={std::atan2(r-cy,cx),std::atan2(cx,cy-r)};
    for(double root:roots)for(int k=-3;k<=3;++k){const double th=root+k*pi;if(th>lo && th<hi)add_at(th);}
  }
  return out;
}
Twist2d output_limit(const Twist2d &target,const Twist2d &last,const Config &c){
  Twist2d out;
  out.linear_x=visual_navigation::LimitRate(target.linear_x,last.linear_x,c.lin_up,c.lin_down,c.dt);
  // Exact output.cpp minimum command floor: current loopback plant has its own accel limits disabled.
  if(std::abs(target.linear_x)>1e-9 && std::abs(out.linear_x)<c.speed)out.linear_x=std::copysign(c.speed,target.linear_x);
  else if(std::abs(target.linear_x)<=1e-9 && std::abs(out.linear_x)<c.speed)out.linear_x=0;
  out.angular_z=visual_navigation::LimitRate(target.angular_z,last.angular_z,c.ang_up,c.ang_down,c.dt);
  return out;
}
struct Step{Twist2d u;std::string phase;};
std::vector<Step> turn_template(double theta,const Config &c,int forced_n=0){
  if(std::abs(theta)<=c.tolerance+1e-12 && forced_n==0)return {};
  int n=forced_n>0?forced_n:2;std::vector<double>w;
  double area=0;
  for(;n<10000;++n){w.clear();area=0;for(int k=0;k<n;++k){const double value=std::min({c.accel*(k+1)*c.dt,c.omega,c.accel*(n-1-k)*c.dt});w.push_back(value);area+=value*c.dt;}if(area>=std::abs(theta)-1e-15)break;}
  if(n==10000)throw std::runtime_error("angular pulse exceeds budget");
  const double scale=std::abs(theta)/area;std::vector<Step> out;
  for(double value:w){const double angular=-std::copysign(value*scale,theta);out.push_back({{value>1e-12?c.speed:0,angular},value>1e-12?"FORWARD_ARC":"STOP"});}
  return out;
}
struct Result{
  bool pass=true,baseline_pass=true;std::string reason="ok";
  int prep_ticks=0,switches=0;double duration=0,axle_distance=0,base_distance=0,max_base_x=0,min_clear=1;
  double y_goal=0,turn_lo=0,turn_hi=0,baseline_clear=1;Pose2d final;Bounds bounds;
};
Result simulate(double theta,double initial_y,const Config &c,std::ostream *trace=nullptr,bool preadjust=true){
  Result result;Pose2d p{0,initial_y,theta};
  auto pulse=turn_template(theta,c);Bounds relative=point_bounds({0,0,theta},c);Pose2d template_pose{0,0,theta};
  for(const auto&s:pulse){relative.merge(interval_bounds(template_pose,s.u,c,c.dt));template_pose=vehicle_loopback_sim::integrate_base_link(template_pose,s.u,c.dt,c.offset,0);}
  result.baseline_clear=std::min(c.half_corridor+initial_y+relative.ymin,c.half_corridor-initial_y-relative.ymax);
  result.baseline_pass=result.baseline_clear>=-1e-10;
  std::vector<Step> program;
  if(!pulse.empty() && preadjust){
    const double q=c.initial_limit(theta);
    result.turn_lo=std::max(-q,-c.half_corridor-relative.ymin+c.turn_reserve);
    result.turn_hi=std::min(q,c.half_corridor-relative.ymax-c.turn_reserve);
    if(result.turn_lo>result.turn_hi){result.pass=false;result.reason="empty_safe_turn_interval";return result;}
    const double desired=std::clamp(initial_y,result.turn_lo,result.turn_hi);result.y_goal=desired;
    if(std::abs(desired-initial_y)>1e-12){
      const double direction=std::copysign(1,(desired-initial_y)/std::sin(theta));
      const double per_tick=direction*c.speed*std::sin(theta)*c.dt;
      result.prep_ticks=static_cast<int>(std::ceil(std::abs((desired-initial_y)/per_tick)-1e-10));
      const double actual=initial_y+result.prep_ticks*per_tick;
      if(actual<result.turn_lo-1e-10 || actual>result.turn_hi+1e-10){result.pass=false;result.reason="quantized_prep_misses_interval";return result;}
      for(int k=0;k<result.prep_ticks;++k)program.push_back({{direction*c.speed,0},direction>0?"FORWARD_PREP":"REVERSE_PREP"});
      for(int k=0;k<static_cast<int>(std::ceil(.22/c.dt));++k)program.push_back({{0,0},"CONFIRMED_STOP"});
      result.switches=direction<0?1:0;
    }
  }
  program.insert(program.end(),pulse.begin(),pulse.end());
  for(int k=0;k<static_cast<int>(std::ceil(.22/c.dt));++k)program.push_back({{0,0},"FINAL_STOP"});
  const std::vector<navigation_safety::Rectangle> free{{-c.window,-c.half_corridor,c.window,c.half_corridor}};
  const std::vector<navigation_safety::Polygon> none;
  auto observe=[&](const Pose2d &pose,const Bounds &bounds){
    result.bounds.merge(bounds);result.max_base_x=std::max(result.max_base_x,std::abs(pose.x));
    result.min_clear=std::min(result.min_clear,std::min(c.half_corridor+bounds.ymin,c.half_corridor-bounds.ymax));
    if(bounds.xmin < -c.window-1e-10 || bounds.xmax>c.window+1e-10 || result.min_clear < -1e-10){result.pass=false;result.reason="continuous_footprint_outside";}
    if(!navigation_safety::pose_is_safe({{pose.x,pose.y},pose.yaw},c.footprint(),free,none,none)){result.pass=false;result.reason="shared_checker_rejection";}
  };
  observe(p,point_bounds(p,c));
  if(trace)*trace<<std::setprecision(17)<<"time_s,phase,linear_mps,angular_rps,x_m,y_m,yaw_rad,interval_clearance_m\n"<<0<<",INITIAL,0,0,"<<p.x<<","<<p.y<<","<<p.yaw<<","<<result.min_clear<<"\n";
  Twist2d last{};
  for(const auto&s:program){
    const auto applied=output_limit(s.u,last,c);
    if(std::abs(applied.linear_x-s.u.linear_x)>1e-10 || std::abs(applied.angular_z-s.u.angular_z)>1e-10){result.pass=false;result.reason="command_not_publishable";}
    if(std::abs(applied.angular_z)>c.omega+1e-12 || (std::abs(applied.angular_z)>1e-12 && std::abs(applied.linear_x)<c.speed-1e-12)){result.pass=false;result.reason="violated_drive_arc_constraint";}
    const auto wheels=vehicle_loopback_sim::wheel_speeds(applied,c.track);
    if(std::max(std::abs(wheels.first),std::abs(wheels.second))>1.0+1e-12){result.pass=false;result.reason="wheel_speed_limit";}
    const auto bounds=interval_bounds(p,applied,c,c.dt);
    p=vehicle_loopback_sim::integrate_base_link(p,applied,c.dt,c.offset,0);
    observe(p,bounds);result.duration+=c.dt;result.axle_distance+=std::abs(applied.linear_x)*c.dt;
    result.base_distance+=std::hypot(applied.linear_x,c.offset*applied.angular_z)*c.dt;
    const double clear=std::min(c.half_corridor+bounds.ymin,c.half_corridor-bounds.ymax);
    if(trace)*trace<<result.duration<<","<<s.phase<<","<<applied.linear_x<<","<<applied.angular_z<<","<<p.x<<","<<p.y<<","<<p.yaw<<","<<clear<<"\n";
    last=applied;
  }
  result.final=p;
  if(std::abs(p.yaw)>c.tolerance+1e-10){result.pass=false;result.reason="heading_tolerance";}
  if(std::abs(last.linear_x)+std::abs(last.angular_z)>1e-12){result.pass=false;result.reason="not_stopped";}
  return result;
}
// Interval certificate, including angles BETWEEN grid samples. Within one pulse
// length N, theta(t)=f(t)*theta0. A corner coordinate has theta0-derivative bounded
// by speed*T + 2*offset + hypot(inflated_half_length,inflated_half_width).
// Extremum over time preserves that Lipschitz bound. Split at EVERY pulse-length
// transition; use endpoint values plus the bound to enclose each entire cell.
void certify_continuous_angles(const Config &c,const std::filesystem::path &out){
  const double maxangle=40*pi/180,a=c.length/2+c.margin,b=c.width/2+c.margin;
  const double radius=std::hypot(a,b),q_lipschitz=radius,pad=1e-9;
  double prev_area=0,min_width=1,max_prep=0,max_path=0,max_body_x=0;long cells=0;
  for(int n=2;n<1000;++n){
    double area=0;for(int k=0;k<n;++k)area+=std::min({c.accel*(k+1)*c.dt,c.omega,c.accel*(n-1-k)*c.dt})*c.dt;
    const double left=std::max(c.tolerance,prev_area),right=std::min(maxangle,area);
    if(right>left+1e-14){
      const double lipschitz=c.speed*n*c.dt+2*c.offset+radius;
      auto evaluate=[&](double theta){
        Pose2d p{0,0,theta};Bounds bounds=point_bounds(p,c);
        for(const auto&s:turn_template(theta,c,n)){bounds.merge(interval_bounds(p,s.u,c,c.dt));p=vehicle_loopback_sim::integrate_base_link(p,s.u,c.dt,c.offset,0);}
        const double q=c.initial_limit(theta),lo=std::max(-q,-c.half_corridor-bounds.ymin+c.turn_reserve),hi=std::min(q,c.half_corridor-bounds.ymax-c.turn_reserve);
        return std::array<double,2>{hi-lo,std::max({0.,lo+q,q-hi})};
      };
      const int subdivisions=std::max(1,static_cast<int>(std::ceil((right-left)/(.01*pi/180))));
      double l=left;auto vl=evaluate(l);
      for(int j=1;j<=subdivisions;++j){
        const double r=left+(right-left)*j/subdivisions;auto vr=evaluate(r);const double delta=r-l;
        const double width_lower=std::min(vl[0],vr[0])-lipschitz*delta-pad;
        const double gap_upper=std::max(vl[1],vr[1])+(lipschitz+q_lipschitz)*delta/2+pad;
        const double prep_upper=gap_upper/std::sin(l)+c.speed*c.dt;
        const double path_upper=prep_upper+n*c.dt*std::hypot(c.speed,c.offset*c.omega);
        min_width=std::min(min_width,width_lower);max_prep=std::max(max_prep,prep_upper);max_path=std::max(max_path,path_upper);max_body_x=std::max(max_body_x,radius+path_upper);++cells;
        l=r;vl=vr;
      }
    }
    prev_area=area;if(area>=maxangle)break;
  }
  const bool pass=min_width>c.speed*c.dt*std::sin(maxangle) && max_body_x<c.window;
  std::ofstream f(out/"continuous_certificate.json");f<<std::setprecision(17)<<"{\n  \"certified\": "<<(pass?"true":"false")<<",\n  \"angle_domain_deg\": [-40, 40],\n  \"stationary_goal_band_deg\": [-3, 3],\n  \"lipschitz_enclosure_cells\": "<<cells<<",\n  \"max_cell_width_deg\": 0.01,\n  \"roundoff_padding_m\": "<<pad<<",\n  \"safe_turn_interval_width_lower_bound_m\": "<<min_width<<",\n  \"largest_prep_lateral_tick_m\": "<<c.speed*c.dt*std::sin(maxangle)<<",\n  \"prep_path_upper_bound_m\": "<<max_prep<<",\n  \"total_base_path_upper_bound_m\": "<<max_path<<",\n  \"inflated_footprint_abs_x_upper_bound_m\": "<<max_body_x<<",\n  \"allowed_abs_x_m\": "<<c.window<<"\n}\n";
  if(!pass)throw std::runtime_error("Continuous-angle certificate failed");
}
int main(int argc,char**argv){try{
  if(argc<3)throw std::runtime_error("usage: corridor_alignment WORKSPACE OUTPUT_DIR [angle_deg initial_y_m]");
  const auto c=Config::load(argv[1]);const std::filesystem::path out=argv[2];std::filesystem::create_directories(out);
  if(argc==5){const double angle=std::stod(argv[3]),y=std::stod(argv[4]);std::ofstream trace(out/"single_trace.csv");auto r=simulate(angle*pi/180,y,c,&trace);std::cout<<std::setprecision(12)<<"pass="<<r.pass<<" reason="<<r.reason<<" min_clear="<<r.min_clear<<" final_y="<<r.final.y<<" final_yaw="<<r.final.yaw<<"\n";return r.pass?0:2;}
  std::ofstream cases(out/"cases.csv");cases<<std::setprecision(17)<<"suite,angle_deg,lateral_fraction,initial_y_m,pass,reason,baseline_pass,baseline_clearance_m,prep_ticks,duration_s,axle_distance_m,base_distance_m,max_base_displacement_x_m,footprint_xmin_m,footprint_xmax_m,min_safety_clearance_m,final_y_m,final_yaw_deg,turn_start_lower_m,turn_start_upper_m\n";
  long count=0,passed=0,baseline_passed=0,prepped=0;double maxdist=0,maxaxle=0,maxx=0,maxbodyx=0,maxtime=0,maxfinalyaw=0,maxprepdistance=0,minclear=1,mininterval=1;
  auto run=[&](const std::string &suite,double angle,double fraction){
    const double theta=angle*pi/180,y=fraction*c.initial_limit(theta);auto r=simulate(theta,y,c);
    ++count;passed+=r.pass;baseline_passed+=r.baseline_pass;prepped+=r.prep_ticks>0;
    maxdist=std::max(maxdist,r.base_distance);maxaxle=std::max(maxaxle,r.axle_distance);maxx=std::max(maxx,r.max_base_x);maxbodyx=std::max({maxbodyx,-r.bounds.xmin,r.bounds.xmax});maxtime=std::max(maxtime,r.duration);maxfinalyaw=std::max(maxfinalyaw,std::abs(r.final.yaw)*180/pi);minclear=std::min(minclear,r.min_clear);maxprepdistance=std::max(maxprepdistance,r.prep_ticks*c.speed*c.dt);if(std::abs(theta)>c.tolerance+1e-12)mininterval=std::min(mininterval,r.turn_hi-r.turn_lo);
    cases<<suite<<","<<angle<<","<<fraction<<","<<y<<","<<r.pass<<","<<r.reason<<","<<r.baseline_pass<<","<<r.baseline_clear<<","<<r.prep_ticks<<","<<r.duration<<","<<r.axle_distance<<","<<r.base_distance<<","<<r.max_base_x<<","<<r.bounds.xmin<<","<<r.bounds.xmax<<","<<r.min_clear<<","<<r.final.y<<","<<r.final.yaw*180/pi<<","<<r.turn_lo<<","<<r.turn_hi<<"\n";
  };
  for(int i=-160;i<=160;++i)for(int j=-50;j<=50;++j)run("grid",i*.25,j/50.0);
  std::mt19937_64 gen(20261013);std::uniform_real_distribution<double> angles(-40,40),fractions(-1,1);
  for(int i=0;i<10000;++i)run("random",angles(gen),fractions(gen));
  for(double angle:{-40.,-30.,-10.,10.,30.,40.})for(double fraction:{-1.,0.,1.}){
    std::string name="trace_a"+std::to_string(static_cast<int>(angle))+"_f"+std::to_string(static_cast<int>(fraction))+".csv";
    std::ofstream trace(out/name);simulate(angle*pi/180,fraction*c.initial_limit(angle*pi/180),c,&trace);
  }
  std::ofstream summary(out/"summary.json");summary<<std::setprecision(17)<<"{\n  \"cases\": "<<count<<",\n  \"passed\": "<<passed<<",\n  \"failed\": "<<count-passed<<",\n  \"turn_only_passed\": "<<baseline_passed<<",\n  \"pre_adjusted_cases\": "<<prepped<<",\n  \"max_base_path_m\": "<<maxdist<<",\n  \"max_axle_path_m\": "<<maxaxle<<",\n  \"max_base_abs_x_m\": "<<maxx<<",\n  \"max_inflated_footprint_abs_x_m\": "<<maxbodyx<<",\n  \"max_duration_s\": "<<maxtime<<",\n  \"max_final_heading_deg\": "<<maxfinalyaw<<",\n  \"min_clearance_after_margin_m\": "<<minclear<<",\n  \"max_pre_adjust_axle_path_m\": "<<maxprepdistance<<",\n  \"min_turn_start_interval_width_m\": "<<mininterval<<",\n  \"corridor_width_m\": "<<2*c.half_corridor<<",\n  \"footprint_length_m\": "<<c.length<<",\n  \"footprint_width_m\": "<<c.width<<",\n  \"margin_m\": "<<c.margin<<",\n  \"base_offset_x_m\": "<<c.offset<<",\n  \"control_dt_s\": "<<c.dt<<",\n  \"speed_mps\": "<<c.speed<<",\n  \"max_angular_speed_rps\": "<<c.omega<<",\n  \"angular_slew_rps2\": "<<c.accel<<",\n  \"turn_reserve_m\": "<<c.turn_reserve<<",\n  \"random_seed\": 20261013\n}\n";
  certify_continuous_angles(c,out);
  std::cout<<"cases="<<count<<" passed="<<passed<<" failed="<<count-passed<<" turn_only_passed="<<baseline_passed<<" max_base_path_m="<<maxdist<<" max_base_abs_x_m="<<maxx<<" max_footprint_abs_x_m="<<maxbodyx<<" max_duration_s="<<maxtime<<" min_clearance="<<minclear<<"\n";
  return passed==count?0:2;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
