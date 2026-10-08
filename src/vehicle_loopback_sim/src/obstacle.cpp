#include "vehicle_loopback_sim/obstacle.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace vehicle_loopback_sim
{

StableObstacleGate::StableObstacleGate(double stable_time_s)
{
  if (!std::isfinite(stable_time_s)) {
    throw std::invalid_argument("stable_time_s must be finite");
  }
  stable_time_s_ = std::max(0.0, stable_time_s);
}

void StableObstacleGate::reset()
{
  candidate_index_.reset();
  candidate_since_s_.reset();
}

std::optional<ObstacleDetection> StableObstacleGate::update(
  const std::optional<ObstacleDetection> & detection, double now_s)
{
  if (!detection.has_value()) {
    reset();
    return std::nullopt;
  }
  if (stable_time_s_ <= 0.0) {
    candidate_index_ = detection->obstacle_index;
    candidate_since_s_ = now_s;
    return detection;
  }
  if (candidate_index_ != detection->obstacle_index || !candidate_since_s_.has_value() ||
    now_s < *candidate_since_s_)
  {
    candidate_index_ = detection->obstacle_index;
    candidate_since_s_ = now_s;
    return std::nullopt;
  }
  return now_s - *candidate_since_s_ >= stable_time_s_ ? detection : std::nullopt;
}

SimulatedObstacleDetector::SimulatedObstacleDetector(
  std::vector<Polygon2d> obstacles, ObstacleConfig config)
: obstacles_(std::move(obstacles)), config_(std::move(config))
{
  const double low = std::max(
    config_.min_range_m, config_.trigger_range_m - config_.trigger_jitter_m);
  const double high = std::max(low, config_.trigger_range_m + config_.trigger_jitter_m);
  activation_floor_m_ = std::max(
    0.0, config_.activation_floor_m.has_value() ?
    std::max(config_.min_range_m, *config_.activation_floor_m) : low);
  std::mt19937 generator(
    config_.random_seed < 0 ? std::random_device{}() :
    static_cast<std::mt19937::result_type>(config_.random_seed));
  std::uniform_real_distribution<double> distribution(low, high);
  for (std::size_t index = 0; index < obstacles_.size(); ++index) {
    trigger_ranges_.push_back(distribution(generator));
  }
}

std::optional<ObstacleDetection> SimulatedObstacleDetector::detect(const Pose2d & pose)
{
  std::map<std::size_t, double> ranges;
  for (std::size_t index = 0; index < obstacles_.size(); ++index) {
    const auto distance = corridor_range(pose, obstacles_[index]);
    if (distance.has_value()) {
      ranges[index] = *distance;
    }
  }
  if (active_index_.has_value()) {
    const auto active = ranges.find(*active_index_);
    if (active != ranges.end()) {
      previous_ranges_ = ranges;
      return ObstacleDetection{active->first, active->second, trigger_ranges_[active->first]};
    }
    active_index_.reset();
  }

  std::optional<ObstacleDetection> best;
  for (const auto & [index, distance] : ranges) {
    const auto previous = previous_ranges_.find(index);
    const bool approaching = previous == previous_ranges_.end() ||
      distance <= previous->second + 1.0e-3;
    if (!approaching || distance < activation_floor_m_ || distance > trigger_ranges_[index]) {
      continue;
    }
    if (!best.has_value() || distance < best->range_m) {
      best = ObstacleDetection{index, distance, trigger_ranges_[index]};
    }
  }
  previous_ranges_ = std::move(ranges);
  if (best.has_value()) {
    active_index_ = best->obstacle_index;
  }
  return best;
}

std::optional<double> SimulatedObstacleDetector::corridor_range(
  const Pose2d & pose, const Polygon2d & obstacle) const
{
  if (obstacle.size() < 3U) {
    return std::nullopt;
  }
  std::vector<Point2d> points;
  for (const auto & point : obstacle) {
    points.push_back(sensor_frame(pose, point));
  }
  const double half_width = std::max(0.0, config_.lateral_tolerance_m);
  std::vector<double> candidates;
  for (const auto & [forward, lateral] : points) {
    if (forward >= config_.min_range_m && std::abs(lateral) <= half_width) {
      candidates.push_back(forward);
    }
  }
  const std::vector<double> boundaries{-half_width, 0.0, half_width};
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto & begin = points[index];
    const auto & end = points[(index + 1U) % points.size()];
    for (double boundary : boundaries) {
      const bool crossed =
        (begin.second <= boundary && boundary <= end.second) ||
        (end.second <= boundary && boundary <= begin.second);
      if (!crossed || std::abs(begin.second - end.second) <= 1.0e-12) {
        continue;
      }
      const double ratio = (boundary - begin.second) / (end.second - begin.second);
      const double forward = begin.first + ratio * (end.first - begin.first);
      if (forward >= config_.min_range_m) {
        candidates.push_back(forward);
      }
    }
    const bool crossed_front =
      (begin.first <= config_.min_range_m && config_.min_range_m <= end.first) ||
      (end.first <= config_.min_range_m && config_.min_range_m <= begin.first);
    if (crossed_front && std::abs(begin.first - end.first) > 1.0e-12) {
      const double ratio =
        (config_.min_range_m - begin.first) / (end.first - begin.first);
      const double lateral = begin.second + ratio * (end.second - begin.second);
      if (std::abs(lateral) <= half_width) {
        candidates.push_back(config_.min_range_m);
      }
    }
  }
  return candidates.empty() ? std::nullopt :
    std::optional<double>(*std::min_element(candidates.begin(), candidates.end()));
}

Point2d SimulatedObstacleDetector::sensor_frame(
  const Pose2d & pose, const Point2d & point) const
{
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  const double sensor_x = pose.x + cosine * config_.sensor_x_m - sine * config_.sensor_y_m;
  const double sensor_y = pose.y + sine * config_.sensor_x_m + cosine * config_.sensor_y_m;
  const double dx = point.first - sensor_x;
  const double dy = point.second - sensor_y;
  return {cosine * dx + sine * dy, -sine * dx + cosine * dy};
}

Polygon2d box_obstacle(
  double center_x_m, double center_y_m, double width_m,
  double depth_m, double yaw_rad)
{
  const double half_width = 0.5 * width_m;
  const double half_depth = 0.5 * depth_m;
  const Point2d forward{std::cos(yaw_rad), std::sin(yaw_rad)};
  const Point2d lateral{-std::sin(yaw_rad), std::cos(yaw_rad)};
  Polygon2d result;
  for (const auto & [forward_offset, lateral_offset] :
    {Point2d{-half_depth, -half_width}, Point2d{half_depth, -half_width},
      Point2d{half_depth, half_width}, Point2d{-half_depth, half_width}})
  {
    result.emplace_back(
      center_x_m + forward.first * forward_offset + lateral.first * lateral_offset,
      center_y_m + forward.second * forward_offset + lateral.second * lateral_offset);
  }
  return result;
}

}  // namespace vehicle_loopback_sim
