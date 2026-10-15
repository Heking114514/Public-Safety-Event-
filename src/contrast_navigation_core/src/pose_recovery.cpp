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
}
void PoseRecovery::load(const std::string &path, navigation_safety::Footprint footprint,
                        const Config &config) {
  if (!std::isfinite(config.forward_obstacle_clearance_m) ||
      config.forward_obstacle_clearance_m<=0)
    throw std::invalid_argument("forward obstacle clearance must be supplied and positive");
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
  observed_.clear(); dynamic_.clear(); live_obstacles_set_=false;
  incoming_override_.clear();
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
} // namespace visual_navigation
