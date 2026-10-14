#include "visual_navigation/pose_recovery.hpp"
#include <algorithm>
#include <cmath>

namespace visual_navigation {
bool PoseRecovery::plan(const Pose &start,
    const mission_control_interfaces::msg::RecoveryCandidate &candidate,
    const std::vector<std::string> &blocked) {
  unsafe_entry_rejected_=false;
  if (plan_direct(start,candidate,blocked)) return true;
  steps_.clear();
  // 固定入口本身放不下车时，前移并不能使这个入口安全。
  if (unsafe_entry_rejected_ || config_.forward_adjustment_m<=0) return false;
  const auto incoming=incoming_override_.empty() ? lane_at_pose(start) : incoming_override_;
  const auto road=std::find_if(lanes_.begin(),lanes_.end(),[&](const Lane &lane) {
    return lane.id==incoming;
  });
  if (road==lanes_.end() ||
      std::find(blocked.begin(),blocked.end(),incoming)!=blocked.end()) return false;
  const auto a=nodes_.at(road->from), b=nodes_.at(road->to);
  double heading=std::atan2(b.y-a.y,b.x-a.x);
  if (std::cos(heading-start.yaw)<0) heading=std::atan2(a.y-b.y,a.x-b.x);
  if (std::abs(std::atan2(std::sin(heading-start.yaw),std::cos(heading-start.yaw)))>.25)
    return false;
  const double c=std::cos(heading),s=std::sin(heading);
  const double along=(start.position.x-a.x)*c+(start.position.y-a.y)*s;
  const Pose target{{a.x+(along+config_.forward_adjustment_m)*c,
                     a.y+(along+config_.forward_adjustment_m)*s},heading};
  Pose adjusted=start;
  // 全程按实时道路反馈前进，动态障碍仍参与车身扫掠及制动检查。
  if (!append_straight(adjusted,target,1,"FORWARD_ADJUSTMENT",incoming)) {
    steps_.clear(); return false;
  }
  const auto adjustment=steps_.back();
  if (!plan_direct(adjusted,candidate,blocked)) { steps_.clear(); return false; }
  Step stop{};
  stop.start=stop.end=stop.target=adjusted;
  stop.kind=Step::Kind::STOP; stop.phase="DIRECTION_STOP";
  stop.duration=.5; stop.lane_id=incoming;
  steps_.insert(steps_.begin(),stop);
  steps_.insert(steps_.begin(),adjustment);
  last_expansions_=static_cast<int>(steps_.size());
  return true;
}
} // namespace visual_navigation
