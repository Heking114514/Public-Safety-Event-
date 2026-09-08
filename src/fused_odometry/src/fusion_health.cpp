#include "fused_odometry/fusion_health.hpp"

#include <algorithm>
#include <cmath>

namespace fused_odometry
{

FusionHealthMonitor::FusionHealthMonitor(
  const MotionClassifierConfig & motion_config,
  const AngularStallConfig & angular_stall_config)
: motion_classifier_(motion_config), angular_stall_detector_(angular_stall_config)
{
}

void FusionHealthMonitor::start_vision_outage(double now_seconds)
{
  vision_outage_active_ = true;
  wheel_sample_valid_ = false;
  vision_outage_started_at_ = now_seconds;
  last_wheel_sample_at_ = now_seconds;
  last_wheel_speed_ = 0.0;
  dead_reckoning_distance_ = 0.0;
}

void FusionHealthMonitor::finish_vision_outage()
{
  vision_outage_active_ = false;
  wheel_sample_valid_ = false;
  dead_reckoning_distance_ = 0.0;
}

void FusionHealthMonitor::update_dead_reckoning(
  double now_seconds, double wheel_velocity, bool wheel_healthy)
{
  if (!vision_outage_active_ || !std::isfinite(now_seconds)) {
    return;
  }
  if (!wheel_healthy || !std::isfinite(wheel_velocity)) {
    wheel_sample_valid_ = false;
    last_wheel_sample_at_ = now_seconds;
    return;
  }
  const double speed = std::abs(wheel_velocity);
  const double elapsed = now_seconds - last_wheel_sample_at_;
  if (wheel_sample_valid_ && std::isfinite(elapsed) && elapsed >= 0.0) {
    dead_reckoning_distance_ += 0.5 * (last_wheel_speed_ + speed) * elapsed;
  }
  last_wheel_sample_at_ = now_seconds;
  last_wheel_speed_ = speed;
  wheel_sample_valid_ = true;
}

FusionHealthSnapshot FusionHealthMonitor::evaluate(
  const FusionHealthInput & input, const FusionHealthLimits & limits)
{
  FusionHealthSnapshot result;
  result.motion_fault = motion_classifier_.update(
    input.now_seconds, input.command_velocity, input.wheel_velocity,
    input.visual_velocity, input.command_fresh, input.wheel_measurement_fresh,
    input.visual_motion_valid);
  const bool wheel_fault = result.motion_fault == MotionFault::kSlip ||
    result.motion_fault == MotionFault::kEncoderFailure;
  result.wheel_healthy = input.wheel_available && !wheel_fault;

  if (input.visual_yaw_valid) {
    result.measured_yaw_rate = std::max(
      result.measured_yaw_rate, std::abs(input.visual_yaw_rate));
    ++result.angular_source_count;
  }
  if (input.imu) {
    result.measured_yaw_rate = std::max(
      result.measured_yaw_rate, std::abs(input.imu_yaw_rate));
    ++result.angular_source_count;
  }
  if (result.wheel_healthy && input.wheel_yaw_valid) {
    result.measured_yaw_rate = std::max(
      result.measured_yaw_rate, std::abs(input.wheel_yaw_rate));
    ++result.angular_source_count;
  }
  result.angular_stalled = angular_stall_detector_.update(
    input.now_seconds, input.command_yaw_rate, result.measured_yaw_rate,
    input.command_fresh, result.angular_source_count >= 2);

  update_dead_reckoning(input.now_seconds, input.wheel_velocity, result.wheel_healthy);
  result.vision_outage_seconds = vision_outage_active_ ?
    std::max(0.0, input.now_seconds - vision_outage_started_at_) : 0.0;
  result.dead_reckoning_distance = dead_reckoning_distance_;
  result.mode = select_mode(
      input.initialized, input.vision, result.wheel_healthy, input.imu,
      input.wheel_yaw_backup, input.visual_realigned,
      result.motion_fault == MotionFault::kStalled || result.angular_stalled,
      result.vision_outage_seconds, result.dead_reckoning_distance,
      limits.max_dead_reckoning_time, limits.max_dead_reckoning_distance,
      limits.max_wheel_only_time, limits.max_wheel_only_distance);
  if (!input.initialized && input.initialization_timed_out) {
    // A startup timeout is distinct from a normal no-vision outage. It is the
    // only point at which the pre-initialization wait becomes a latched fault.
    result.mode = FusionMode::kFaultInitTimeout;
  }
  return result;
}

}  // namespace fused_odometry
