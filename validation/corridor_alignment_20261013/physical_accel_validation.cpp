// Additional actuator sensitivity model: command floor is unchanged, but actual
// v ramps with 0.25 m/s^2 acceleration and 0.80 m/s^2 braking. Actual angular
// velocity ramps symmetrically at 1.80 rad/s^2. This is deliberately stricter than
// the configured loopback plant, whose physical acceleration limits are disabled.
#define main primary_validation_main
#include "corridor_alignment.cpp"
#undef main
#include <map>
struct Plant {Pose2d pose;Twist2d actual{},last_command{};Bounds bounds;double distance=0,time=0;explicit Plant(Pose2d initial):pose(initial){}};
void physical_step(Plant &p,const Twist2d &desired,const Config &c){
  const auto command=output_limit(desired,p.last_command,c);
  if(std::abs(command.linear_x-desired.linear_x)>1e-10 || std::abs(command.angular_z-desired.angular_z)>1e-10)throw std::runtime_error("physical command not publishable");
  constexpr int substeps=20;const double dt=c.dt/substeps;
  for(int k=0;k<substeps;++k){
    const double linear_rate=std::abs(command.linear_x)>std::abs(p.actual.linear_x)+1e-14?c.lin_up:c.lin_down;
    p.actual.linear_x=vehicle_loopback_sim::limit_rate(command.linear_x,p.actual.linear_x,linear_rate,dt);
    p.actual.angular_z=vehicle_loopback_sim::limit_rate(command.angular_z,p.actual.angular_z,c.accel,dt);
    p.bounds.merge(interval_bounds(p.pose,p.actual,c,dt));
    p.pose=vehicle_loopback_sim::integrate_base_link(p.pose,p.actual,dt,c.offset,0);
    p.distance+=std::hypot(p.actual.linear_x,c.offset*p.actual.angular_z)*dt;p.time+=dt;
  }
  p.last_command=command;
}
int main(int argc,char **argv){try{
  if(argc!=3)throw std::runtime_error("usage: physical_accel_validation WORKSPACE OUTPUT_DIR");
  const Config c=Config::load(argv[1]);const std::filesystem::path out=argv[2];
  const int stop_ticks=static_cast<int>(std::ceil(.22/c.dt));
  std::vector<double> distances(201,0);
  for(int n=1;n<=200;++n){Plant p{{0,0,0}};p.bounds=point_bounds(p.pose,c);for(int k=0;k<n;++k)physical_step(p,{c.speed,0},c);for(int k=0;k<stop_ticks;++k)physical_step(p,{0,0},c);distances[n]=p.pose.x;}
  struct TurnInfo {std::vector<Step> pulse;double lo,hi;};std::map<double,TurnInfo> cached;
  std::ofstream cases(out/"physical_accel_cases.csv");cases<<std::setprecision(17)<<"suite,angle_deg,lateral_fraction,pass,min_clearance_after_margin_m,base_path_m,inflated_abs_x_m,final_yaw_deg,duration_s,prep_ticks\n";
  long count=0,passed=0;double maxpath=0,maxx=0,maxtime=0,minclear=1,maxyaw=0;
  auto run=[&](std::string suite,double angle,double fraction){
    const double theta=angle*pi/180,limit=c.initial_limit(theta),y=fraction*limit;
    if(!cached.count(angle)){
      TurnInfo t;t.pulse=turn_template(theta,c);Plant p{{0,0,theta}};p.bounds=point_bounds(p.pose,c);
      for(const auto&s:t.pulse){physical_step(p,s.u,c);}
      for(int k=0;k<stop_ticks;++k){physical_step(p,{0,0},c);}
      t.lo=std::max(-limit,-c.half_corridor-p.bounds.ymin+c.turn_reserve);t.hi=std::min(limit,c.half_corridor-p.bounds.ymax-c.turn_reserve);cached.emplace(angle,std::move(t));
    }
    const auto&t=cached.at(angle);Plant p{{0,y,theta}};p.bounds=point_bounds(p.pose,c);bool pass=true;int ticks=0;
    if(!t.pulse.empty()){
      if(t.lo>t.hi)throw std::runtime_error("physical model has empty safe turn interval");
      const double desired=std::clamp(y,t.lo,t.hi),shift=desired-y;
      if(std::abs(shift)>1e-12){
        const double distance=std::abs(shift/std::sin(theta));const auto iter=std::lower_bound(distances.begin(),distances.end(),distance-1e-12);ticks=iter-distances.begin();
        if(iter==distances.end())throw std::runtime_error("physical prep exceeds distance table");
        const double sign=std::copysign(1,shift/std::sin(theta));
        for(int k=0;k<ticks;++k){physical_step(p,{sign*c.speed,0},c);}
        for(int k=0;k<stop_ticks;++k){physical_step(p,{0,0},c);}
        if(p.pose.y<t.lo-1e-10 || p.pose.y>t.hi+1e-10)pass=false;
      }
      for(const auto&s:t.pulse)physical_step(p,s.u,c);
    }
    for(int k=0;k<stop_ticks;++k)physical_step(p,{0,0},c);
    const double clear=std::min(c.half_corridor+p.bounds.ymin,c.half_corridor-p.bounds.ymax),extent=std::max(-p.bounds.xmin,p.bounds.xmax);
    pass=pass && clear>=-1e-10 && extent<=c.window+1e-10 && std::abs(p.pose.yaw)<=c.tolerance+1e-9 && std::abs(p.actual.linear_x)+std::abs(p.actual.angular_z)<1e-9;
    ++count;passed+=pass;maxpath=std::max(maxpath,p.distance);maxx=std::max(maxx,extent);maxtime=std::max(maxtime,p.time);minclear=std::min(minclear,clear);maxyaw=std::max(maxyaw,std::abs(p.pose.yaw)*180/pi);
    cases<<suite<<","<<angle<<","<<fraction<<","<<pass<<","<<clear<<","<<p.distance<<","<<extent<<","<<p.pose.yaw*180/pi<<","<<p.time<<","<<ticks<<"\n";
  };
  for(int i=-40;i<=40;++i)for(int j=-10;j<=10;++j)run("grid",i,j/10.);
  std::mt19937_64 gen(20261013);std::uniform_real_distribution<double> angles(-40,40),fractions(-1,1);for(int i=0;i<1000;++i)run("random",angles(gen),fractions(gen));
  std::ofstream f(out/"physical_accel_summary.json");f<<std::setprecision(17)<<"{\n  \"cases\": "<<count<<",\n  \"passed\": "<<passed<<",\n  \"failed\": "<<count-passed<<",\n  \"max_base_path_m\": "<<maxpath<<",\n  \"max_inflated_footprint_abs_x_m\": "<<maxx<<",\n  \"max_duration_s\": "<<maxtime<<",\n  \"max_final_heading_deg\": "<<maxyaw<<",\n  \"min_clearance_after_margin_m\": "<<minclear<<",\n  \"actual_linear_acceleration_mps2\": "<<c.lin_up<<",\n  \"actual_linear_deceleration_mps2\": "<<c.lin_down<<",\n  \"actual_angular_acceleration_and_deceleration_rps2\": "<<c.accel<<",\n  \"plant_update_hz\": "<<20/c.dt<<",\n  \"command_update_hz\": "<<1/c.dt<<"\n}\n";
  std::cout<<"physical cases="<<count<<" passed="<<passed<<" failed="<<count-passed<<" max_path="<<maxpath<<" max_body_abs_x="<<maxx<<" min_clear="<<minclear<<"\n";return passed==count?0:2;
}catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}}
