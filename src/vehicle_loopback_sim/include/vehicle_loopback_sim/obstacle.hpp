#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "vehicle_loopback_sim/motion.hpp"

namespace vehicle_loopback_sim
{

using Point2d = std::pair<double, double>;
using Polygon2d = std::vector<Point2d>;

struct ObstacleConfig
{
  double sensor_x_m{0.0};
  double sensor_y_m{0.0};
  double trigger_range_m{0.25};
  double trigger_jitter_m{0.0};
  double lateral_tolerance_m{0.08};
  double min_range_m{0.03};
  int random_seed{0};
  std::optional<double> activation_floor_m;
};

struct ObstacleDetection
{
  std::size_t obstacle_index{0U};
  double range_m{0.0};
  double trigger_range_m{0.0};
};

class StableObstacleGate
{
public:
  explicit StableObstacleGate(double stable_time_s);
  void reset();
  std::optional<ObstacleDetection> update(
    const std::optional<ObstacleDetection> & detection, double now_s);

private:
  double stable_time_s_{0.0};
  std::optional<std::size_t> candidate_index_;
  std::optional<double> candidate_since_s_;
};

class SimulatedObstacleDetector
{
public:
  SimulatedObstacleDetector(std::vector<Polygon2d> obstacles, ObstacleConfig config);
  std::optional<ObstacleDetection> detect(const Pose2d & pose);

private:
  std::optional<double> corridor_range(
    const Pose2d & pose, const Polygon2d & obstacle) const;
  Point2d sensor_frame(const Pose2d & pose, const Point2d & point) const;

  std::vector<Polygon2d> obstacles_;
  ObstacleConfig config_;
  double activation_floor_m_{0.0};
  std::vector<double> trigger_ranges_;
  std::map<std::size_t, double> previous_ranges_;
  std::optional<std::size_t> active_index_;
};

Polygon2d box_obstacle(
  double center_x_m, double center_y_m, double width_m,
  double depth_m, double yaw_rad = 0.0);

}  // namespace vehicle_loopback_sim
