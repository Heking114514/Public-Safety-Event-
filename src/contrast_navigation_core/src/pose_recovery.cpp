#include "visual_navigation/pose_recovery.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "visual_navigation/rate_limiter.hpp"

namespace visual_navigation {
namespace {
constexpr double pi = 3.14159265358979323846;
double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }
double distance(navigation_safety::Point a, navigation_safety::Point b) {
  return std::hypot(a.x - b.x, a.y - b.y);
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
    return navigation_safety::Point{std::cos(yaw) * (x - ox) + std::sin(yaw) * (y - oy),
      -std::sin(yaw) * (x - ox) + std::cos(yaw) * (y - oy)};
  };
  free_.clear(); static_.clear(); lanes_.clear(); nodes_.clear();
  for (const auto &r : root["arena"]["free_regions"]) {
    auto a = transform(r[0].as<double>(), r[1].as<double>());
    auto b = transform(r[2].as<double>(), r[3].as<double>());
    free_.push_back({std::min(a.x, b.x), std::min(a.y, b.y),
      std::max(a.x, b.x), std::max(a.y, b.y)});
  }
  for (const auto &r : root["arena"]["obstacles"]) {
    navigation_safety::Polygon p;
    for (auto xy : std::array<std::pair<int, int>, 4>{{{0, 1}, {2, 1}, {2, 3}, {0, 3}}})
      p.points.push_back(transform(r[xy.first].as<double>(), r[xy.second].as<double>()));
    static_.push_back(p);
  }
  for (const auto &n : root["nodes"])
    nodes_[n.first.as<std::string>()] = transform(n.second[0].as<double>(), n.second[1].as<double>());
  for (const auto &l : root["lanes"])
    lanes_.push_back({l["id"].as<std::string>(), l["from"].as<std::string>(),
      l["to"].as<std::string>(), l["width_m"].as<double>()});
}
double PoseRecovery::active_seconds(double wall_seconds) {
  if (!paused_) active_time_ += std::max(0.0, wall_seconds - last_wall_);
  last_wall_ = wall_seconds; paused_ = false;
  return active_time_;
}
PoseRecovery::Pose PoseRecovery::integrate(Pose p, double v, double w, double dt) const {
  if (dt <= 0.0) return p;
  const double ox = config_.base_offset_x, oy = config_.base_offset_y;
  p.position.x -= std::cos(p.yaw) * ox - std::sin(p.yaw) * oy;
  p.position.y -= std::sin(p.yaw) * ox + std::cos(p.yaw) * oy;
  if (std::abs(w) < 1e-9) {
    p.position.x += v * dt * std::cos(p.yaw);
    p.position.y += v * dt * std::sin(p.yaw);
  } else {
    p.position.x += v / w * (std::sin(p.yaw + w * dt) - std::sin(p.yaw));
    p.position.y += v / w * (std::cos(p.yaw) - std::cos(p.yaw + w * dt));
  }
  p.yaw = wrap(p.yaw + w * dt);
  p.position.x += std::cos(p.yaw) * ox - std::sin(p.yaw) * oy;
  p.position.y += std::sin(p.yaw) * ox + std::cos(p.yaw) * oy;
  return p;
}
std::vector<PoseRecovery::Pose> PoseRecovery::motion_samples(
    Pose p, double v, double target_w, double initial_w, double duration, double *final_w) const {
  std::vector<Pose> poses{p};
  double w = initial_w;
  for (double elapsed = 0.0; elapsed < duration - 1e-10;) {
    const double dt = std::min(config_.control_dt, duration - elapsed);
    w = LimitRate(target_w, w, config_.angular_acceleration,
      config_.angular_deceleration, dt);
    const double speed = std::abs(v) +
      std::hypot(config_.base_offset_x, config_.base_offset_y) * std::abs(w);
    const int samples = std::max({1, static_cast<int>(std::ceil(speed * dt / 0.003)),
      static_cast<int>(std::ceil(std::abs(w) * dt / 0.035))});
    const Pose begin = p;
    for (int i = 1; i <= samples; ++i)
      poses.push_back(integrate(begin, v, w, dt * i / samples));
    p = poses.back(); elapsed += dt;
  }
  if (final_w) *final_w = w;
  return poses;
}
PoseRecovery::Pose PoseRecovery::reference(const Step &step, double elapsed) const {
  return motion_samples(step.start, step.linear, step.angular, step.initial_angular,
    std::clamp(elapsed, 0.0, step.duration)).back();
}
bool PoseRecovery::safe_motion(Pose p, double v, double w, double duration) const {
  return safe(motion_samples(p, v, w, w, duration));
}
bool PoseRecovery::safe(const std::vector<Pose> &poses) const {
  return navigation_safety::sweep_is_safe(poses, footprint_, free_, static_, dynamic_, 0.003, 0.035);
}
std::vector<mission_control_interfaces::msg::RecoveryCandidate>
PoseRecovery::candidates(const Pose &pose, const navigation_safety::Polygon &obstacle) {
  dynamic_ = {obstacle};
  steps_.clear();
  anchor_.clear();
  blocked_lane_.clear();
  navigation_safety::Point center{};
  for (auto p : obstacle.points) {
    center.x += p.x;
    center.y += p.y;
  }
  if (obstacle.points.empty()) return {};
  center.x /= obstacle.points.size();
  center.y /= obstacle.points.size();

  double best = std::numeric_limits<double>::infinity();
  for (const auto &lane : lanes_) {
    const auto a = nodes_.at(lane.from);
    const auto b = nodes_.at(lane.to);
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length2 = dx * dx + dy * dy;
    if (length2 < 1.0e-9) continue;
    const double projected = std::clamp(
      ((center.x - a.x) * dx + (center.y - a.y) * dy) / length2, 0.0, 1.0);
    const double error = distance(center, {a.x + projected * dx, a.y + projected * dy});
    if (error < best) {
      best = error;
      blocked_lane_ = lane.id;
      anchor_ = (dx * std::cos(pose.yaw) + dy * std::sin(pose.yaw) > 0.0) ?
        lane.from : lane.to;
    }
  }
  if (anchor_.empty() || best > 0.12 ||
      distance(nodes_.at(anchor_), pose.position) > config_.max_radius) return {};

  const auto anchor = nodes_.at(anchor_);
  const auto blocked_it = std::find_if(lanes_.begin(), lanes_.end(),
    [&](const Lane &lane) { return lane.id == blocked_lane_; });
  if (blocked_it == lanes_.end()) return {};
  const auto blocked_other = blocked_it->from == anchor_ ?
    nodes_.at(blocked_it->to) : nodes_.at(blocked_it->from);
  const double incoming_heading = std::atan2(
    blocked_other.y - anchor.y, blocked_other.x - anchor.x);

  std::vector<const Lane *> exits;
  for (const auto &lane : lanes_) {
    if (lane.id == blocked_lane_ || (lane.from != anchor_ && lane.to != anchor_)) continue;
    exits.push_back(&lane);
  }
  if (exits.empty()) return {};
  const std::string junction = exits.size() >= 3 ? "x_junction" : "t_junction";

  std::vector<mission_control_interfaces::msg::RecoveryCandidate> result;
  for (const auto *lane : exits) {
    const auto other = lane->from == anchor_ ? nodes_.at(lane->to) : nodes_.at(lane->from);
    const double heading = std::atan2(other.y - anchor.y, other.x - anchor.x);
    const double delta = wrap(heading - incoming_heading);
    std::string branch = "right";
    if (std::abs(delta) > 2.2) branch = "reverse";
    else if (delta > 0.0) branch = "left";

    mission_control_interfaces::msg::RecoveryCandidate candidate;
    candidate.candidate_id = static_cast<uint32_t>(result.size() + 1);
    candidate.template_id = junction + "_" + branch;
    candidate.resume_node = anchor_;
    candidate.resume_lane = lane->id;
    candidate.resume_heading = heading;
    candidate.borrowed_lane_ids = {lane->id};
    const double entry_distance = std::min(config_.exit_distance,
      0.25 * std::hypot(other.x - anchor.x, other.y - anchor.y));
    candidate.exit_pose.position.x = anchor.x + entry_distance * std::cos(heading);
    candidate.exit_pose.position.y = anchor.y + entry_distance * std::sin(heading);
    candidate.exit_pose.orientation.z = std::sin(heading / 2.0);
    candidate.exit_pose.orientation.w = std::cos(heading / 2.0);
    candidate.motion_cost = distance(pose.position,
      {candidate.exit_pose.position.x, candidate.exit_pose.position.y});
    if (safe({{{candidate.exit_pose.position.x, candidate.exit_pose.position.y}, heading}}))
      result.push_back(candidate);
  }
  return result;
}

bool PoseRecovery::plan(const Pose &start,
    const mission_control_interfaces::msg::RecoveryCandidate &candidate,
    const std::vector<std::string> &blocked) {
  steps_.clear();
  last_expansions_ = 0;
  const auto forbidden = [&](const std::string &lane) {
    return std::find(blocked.begin(), blocked.end(), lane) != blocked.end();
  };
  if (anchor_.empty() || nodes_.find(anchor_) == nodes_.end() ||
      forbidden(candidate.resume_lane) ||
      std::any_of(candidate.borrowed_lane_ids.begin(), candidate.borrowed_lane_ids.end(), forbidden))
    return false;

  dynamic_.resize(std::min<std::size_t>(1, dynamic_.size()));
  for (const auto &lane : lanes_) {
    if (lane.id == blocked_lane_ ||
        std::find(blocked.begin(), blocked.end(), lane.id) == blocked.end()) continue;
    const auto a = nodes_.at(lane.from);
    const auto b = nodes_.at(lane.to);
    const double heading = std::atan2(b.y - a.y, b.x - a.x);
    const double nx = -std::sin(heading) * lane.width / 2.0;
    const double ny = std::cos(heading) * lane.width / 2.0;
    dynamic_.push_back({{{a.x + nx, a.y + ny}, {b.x + nx, b.y + ny},
      {b.x - nx, b.y - ny}, {a.x - nx, a.y - ny}}});
  }

  const auto anchor = nodes_.at(anchor_);
  const Pose goal{{candidate.exit_pose.position.x, candidate.exit_pose.position.y},
                  candidate.resume_heading};
  if (!safe({start}) || !safe({{anchor, start.yaw}}) || !safe({goal})) return false;

  // This is a finite road action rather than an unconstrained local pose search.
  // The live controller tracks every target with odometry and safety sweeps.
  const double retreat_distance = distance(start.position, anchor);
  const double retreat_heading = std::atan2(
    anchor.y - start.position.y, anchor.x - start.position.x) + pi;
  const Pose retreat_target{anchor, wrap(retreat_heading)};
  const double retreat_duration = std::max(0.25, retreat_distance / config_.speed + 0.25);
  steps_.push_back({start, retreat_target, retreat_target, -config_.speed, 0.0,
    retreat_duration, 0.0, "RETREAT_TO_TURN_POINT"});

  const Pose align_target{anchor, candidate.resume_heading};
  const double align_heading_change = std::abs(wrap(candidate.resume_heading - retreat_target.yaw));
  const double align_duration = std::max(2.5, align_heading_change / config_.angular + 1.0);
  steps_.push_back({retreat_target, align_target, align_target, config_.speed, 0.0,
    align_duration, 0.0, "ALIGN_FOR_TURN"});

  const double turn_distance = distance(anchor, goal.position);
  const double turn_duration = std::max(1.5, turn_distance / config_.speed + 0.6);
  steps_.push_back({align_target, goal, goal, config_.speed, 0.0,
    turn_duration, 0.0, "TURN_INTO_EXIT"});
  last_expansions_ = 3;
  return true;
}
} // namespace visual_navigation
