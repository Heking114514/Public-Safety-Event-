#pragma once

#include <cstddef>

#include "fused_odometry/fusion_logic.hpp"

namespace fused_odometry
{

struct FusionHealthLimits
{
  double max_dead_reckoning_time{2.0};
  double max_dead_reckoning_distance{0.30};
  double max_wheel_only_time{2.0};
  double max_wheel_only_distance{0.30};
};

struct FusionHealthInput
{
  double now_seconds{0.0};
  bool initialized{false};
  bool vision{false};
  bool wheel_measurement_fresh{false};
  bool wheel_available{false};
  bool imu{false};
  bool wheel_yaw_backup{false};
  bool visual_realigned{false};

  double command_velocity{0.0};
  double command_yaw_rate{0.0};
  bool command_fresh{false};
  double wheel_velocity{0.0};
  double visual_velocity{0.0};
  bool visual_motion_valid{false};

  double visual_yaw_rate{0.0};
  bool visual_yaw_valid{false};
  double imu_yaw_rate{0.0};
  double wheel_yaw_rate{0.0};
  bool wheel_yaw_valid{false};
};

struct FusionHealthSnapshot
{
  FusionMode mode{FusionMode::kFault};
  MotionFault motion_fault{MotionFault::kNone};
  bool wheel_healthy{false};
  bool angular_stalled{false};
  double measured_yaw_rate{0.0};
  std::size_t angular_source_count{0};
  double vision_outage_seconds{0.0};
  double dead_reckoning_distance{0.0};
};

// Owns the stateful health decisions which must agree for a single status
// sample. Sensor validation and ROS freshness remain at the node boundary.
class FusionHealthMonitor
{
public:
  FusionHealthMonitor(
    const MotionClassifierConfig & motion_config,
    const AngularStallConfig & angular_stall_config);

  void start_vision_outage(double now_seconds);
  void finish_vision_outage();
  FusionHealthSnapshot evaluate(
    const FusionHealthInput & input, const FusionHealthLimits & limits);

  MotionFault motion_fault() const {return motion_classifier_.fault();}
  bool vision_outage_active() const {return vision_outage_active_;}
  double dead_reckoning_distance() const {return dead_reckoning_distance_;}

private:
  void update_dead_reckoning(
    double now_seconds, double wheel_velocity, bool wheel_healthy);

  MotionClassifier motion_classifier_;
  AngularStallDetector angular_stall_detector_;
  bool vision_outage_active_{false};
  bool wheel_sample_valid_{false};
  double vision_outage_started_at_{0.0};
  double last_wheel_sample_at_{0.0};
  double last_wheel_speed_{0.0};
  double dead_reckoning_distance_{0.0};
};

}  // namespace fused_odometry
