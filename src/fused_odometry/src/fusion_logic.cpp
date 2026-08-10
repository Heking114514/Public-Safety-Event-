#include "fused_odometry/fusion_logic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace fused_odometry
{

double wrap_angle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double disagreement_covariance_scale(
  double absolute_residual, double soft_threshold, double residual_cap)
{
  if (!std::isfinite(absolute_residual) || !std::isfinite(soft_threshold) ||
    !std::isfinite(residual_cap) || soft_threshold <= 0.0 || residual_cap <= 0.0)
  {
    return 100.0;
  }
  const double bounded_residual = std::min(std::abs(absolute_residual), residual_cap);
  const double ratio = bounded_residual / soft_threshold;
  return std::clamp(1.0 + ratio * ratio, 1.0, 100.0);
}

bool pose_residual_within(
  const Pose2d & measurement, const Pose2d & reference,
  double max_position_residual, double max_yaw_residual)
{
  if (!std::isfinite(measurement.x) || !std::isfinite(measurement.y) ||
    !std::isfinite(measurement.yaw) || !std::isfinite(reference.x) ||
    !std::isfinite(reference.y) || !std::isfinite(reference.yaw) ||
    !std::isfinite(max_position_residual) || max_position_residual < 0.0 ||
    !std::isfinite(max_yaw_residual) || max_yaw_residual < 0.0)
  {
    return false;
  }
  return std::hypot(measurement.x - reference.x, measurement.y - reference.y) <=
           max_position_residual &&
         std::abs(wrap_angle(measurement.yaw - reference.yaw)) <= max_yaw_residual;
}

void PoseAligner::clear()
{
  offset_ = {};
  initialized_ = false;
}

void PoseAligner::align_to(const Pose2d & raw, const Pose2d & target)
{
  offset_.yaw = wrap_angle(target.yaw - raw.yaw);
  const double cosine = std::cos(offset_.yaw);
  const double sine = std::sin(offset_.yaw);
  offset_.x = target.x - (cosine * raw.x - sine * raw.y);
  offset_.y = target.y - (sine * raw.x + cosine * raw.y);
  initialized_ = true;
}

Pose2d PoseAligner::apply(const Pose2d & raw) const
{
  if (!initialized_) {
    return raw;
  }
  const double cosine = std::cos(offset_.yaw);
  const double sine = std::sin(offset_.yaw);
  return {
    offset_.x + cosine * raw.x - sine * raw.y,
    offset_.y + sine * raw.x + cosine * raw.y,
    wrap_angle(raw.yaw + offset_.yaw)};
}

ResidualGate::ResidualGate(
  double reject_threshold, std::size_t bad_samples, std::size_t recovery_samples)
: reject_threshold_(reject_threshold),
  bad_samples_(std::max<std::size_t>(1, bad_samples)),
  recovery_samples_(std::max<std::size_t>(1, recovery_samples))
{
  if (!std::isfinite(reject_threshold_) || reject_threshold_ <= 0.0) {
    throw std::invalid_argument("reject_threshold must be finite and positive");
  }
}

bool ResidualGate::update(double absolute_residual)
{
  if (!std::isfinite(absolute_residual) || absolute_residual > reject_threshold_) {
    good_count_ = 0;
    if (++bad_count_ >= bad_samples_) {
      rejected_ = true;
    }
  } else {
    bad_count_ = 0;
    if (rejected_) {
      if (++good_count_ >= recovery_samples_) {
        rejected_ = false;
        good_count_ = 0;
      }
    }
  }
  return !rejected_;
}

double ResidualGate::covariance_scale(double absolute_residual, double soft_threshold) const
{
  if (!std::isfinite(absolute_residual) || soft_threshold <= 0.0) {
    return 100.0;
  }
  const double ratio = absolute_residual / soft_threshold;
  return std::clamp(1.0 + ratio * ratio, 1.0, 100.0);
}

RobustWindow::RobustWindow(double duration_seconds)
: duration_seconds_(duration_seconds)
{
  if (!std::isfinite(duration_seconds_) || duration_seconds_ <= 0.0) {
    throw std::invalid_argument("window duration must be finite and positive");
  }
}

void RobustWindow::add(double time_seconds, double sample)
{
  if (!std::isfinite(time_seconds) || !std::isfinite(sample)) {
    return;
  }
  if (!samples_.empty() && time_seconds < samples_.back().first) {
    samples_.clear();
  }
  samples_.emplace_back(time_seconds, sample);
  while (!samples_.empty() && time_seconds - samples_.front().first > duration_seconds_) {
    samples_.pop_front();
  }
}

void RobustWindow::clear()
{
  samples_.clear();
}

bool RobustWindow::ready(std::size_t minimum_samples) const
{
  return samples_.size() >= minimum_samples;
}

double RobustWindow::median() const
{
  if (samples_.empty()) {
    return 0.0;
  }
  std::vector<double> values;
  values.reserve(samples_.size());
  for (const auto & sample : samples_) {
    values.push_back(sample.second);
  }
  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle), values.end());
  double result = values[middle];
  if (values.size() % 2 == 0) {
    const auto lower = std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(middle));
    result = 0.5 * (result + *lower);
  }
  return result;
}

double RobustWindow::mad() const
{
  if (samples_.empty()) {
    return 0.0;
  }
  const double center = median();
  std::vector<double> deviations;
  deviations.reserve(samples_.size());
  for (const auto & sample : samples_) {
    deviations.push_back(std::abs(sample.second - center));
  }
  const std::size_t middle = deviations.size() / 2;
  std::nth_element(
    deviations.begin(), deviations.begin() + static_cast<std::ptrdiff_t>(middle), deviations.end());
  double result = deviations[middle];
  if (deviations.size() % 2 == 0) {
    const auto lower = std::max_element(
      deviations.begin(), deviations.begin() + static_cast<std::ptrdiff_t>(middle));
    result = 0.5 * (result + *lower);
  }
  return result;
}

MotionClassifier::MotionClassifier(const MotionClassifierConfig & config)
: config_(config)
{
  if (config_.command_threshold <= 0.0 || config_.moving_threshold <= 0.0 ||
    config_.stationary_threshold < 0.0 || config_.fault_dwell_seconds <= 0.0 ||
    config_.recovery_dwell_seconds <= 0.0)
  {
    throw std::invalid_argument("invalid motion classifier configuration");
  }
}

MotionFault MotionClassifier::classify(
  double command_velocity, double wheel_velocity, double visual_velocity,
  bool command_valid, bool wheel_valid, bool visual_valid) const
{
  if (!command_valid || !wheel_valid || !visual_valid ||
    std::abs(command_velocity) < config_.command_threshold)
  {
    return MotionFault::kNone;
  }
  const bool wheel_moving = std::abs(wheel_velocity) >= config_.moving_threshold;
  const bool visual_moving = std::abs(visual_velocity) >= config_.moving_threshold;
  const bool wheel_stationary = std::abs(wheel_velocity) <= config_.stationary_threshold;
  const bool visual_stationary = std::abs(visual_velocity) <= config_.stationary_threshold;
  if (wheel_moving && visual_stationary) {
    return MotionFault::kSlip;
  }
  if (wheel_stationary && visual_stationary) {
    return MotionFault::kStalled;
  }
  if (wheel_stationary && visual_moving) {
    return MotionFault::kEncoderFailure;
  }
  return MotionFault::kNone;
}

MotionFault MotionClassifier::update(
  double time_seconds, double command_velocity, double wheel_velocity,
  double visual_velocity, bool command_valid, bool wheel_valid, bool visual_valid)
{
  const MotionFault observed = classify(
    command_velocity, wheel_velocity, visual_velocity, command_valid, wheel_valid, visual_valid);
  if (active_fault_ == MotionFault::kNone) {
    recovery_active_ = false;
    if (observed == MotionFault::kNone) {
      candidate_active_ = false;
      return active_fault_;
    }
    if (!candidate_active_ || candidate_fault_ != observed || time_seconds < candidate_since_) {
      candidate_fault_ = observed;
      candidate_since_ = time_seconds;
      candidate_active_ = true;
    } else if (time_seconds - candidate_since_ >= config_.fault_dwell_seconds) {
      active_fault_ = observed;
      candidate_active_ = false;
    }
    return active_fault_;
  }

  if (observed == active_fault_) {
    recovery_active_ = false;
    candidate_active_ = false;
    return active_fault_;
  }
  if (observed != MotionFault::kNone) {
    recovery_active_ = false;
    if (!candidate_active_ || candidate_fault_ != observed || time_seconds < candidate_since_) {
      candidate_fault_ = observed;
      candidate_since_ = time_seconds;
      candidate_active_ = true;
    } else if (time_seconds - candidate_since_ >= config_.fault_dwell_seconds) {
      active_fault_ = observed;
      candidate_active_ = false;
    }
    return active_fault_;
  }
  candidate_active_ = false;
  if (!recovery_active_ || time_seconds < recovery_since_) {
    recovery_since_ = time_seconds;
    recovery_active_ = true;
  } else if (time_seconds - recovery_since_ >= config_.recovery_dwell_seconds) {
    active_fault_ = MotionFault::kNone;
    recovery_active_ = false;
  }
  return active_fault_;
}

AngularStallDetector::AngularStallDetector(const AngularStallConfig & config)
: config_(config)
{
  if (!std::isfinite(config_.command_threshold) || config_.command_threshold <= 0.0 ||
    !std::isfinite(config_.stationary_threshold) || config_.stationary_threshold < 0.0 ||
    !std::isfinite(config_.fault_dwell_seconds) || config_.fault_dwell_seconds <= 0.0 ||
    !std::isfinite(config_.recovery_dwell_seconds) || config_.recovery_dwell_seconds <= 0.0)
  {
    throw std::invalid_argument("invalid angular stall detector configuration");
  }
}

bool AngularStallDetector::update(
  double time_seconds, double command_yaw_rate, double measured_yaw_rate,
  bool command_valid, bool measurement_valid)
{
  const bool observed = std::isfinite(time_seconds) && command_valid && measurement_valid &&
    std::isfinite(command_yaw_rate) && std::isfinite(measured_yaw_rate) &&
    std::abs(command_yaw_rate) >= config_.command_threshold &&
    std::abs(measured_yaw_rate) <= config_.stationary_threshold;

  if (!stalled_) {
    recovery_active_ = false;
    if (!observed) {
      candidate_active_ = false;
      return false;
    }
    if (!candidate_active_ || time_seconds < candidate_since_) {
      candidate_since_ = time_seconds;
      candidate_active_ = true;
    } else if (time_seconds - candidate_since_ >= config_.fault_dwell_seconds) {
      stalled_ = true;
      candidate_active_ = false;
    }
    return stalled_;
  }

  candidate_active_ = false;
  if (observed) {
    recovery_active_ = false;
    return true;
  }
  if (!recovery_active_ || time_seconds < recovery_since_) {
    recovery_since_ = time_seconds;
    recovery_active_ = true;
  } else if (time_seconds - recovery_since_ >= config_.recovery_dwell_seconds) {
    stalled_ = false;
    recovery_active_ = false;
  }
  return stalled_;
}

const char * motion_fault_name(MotionFault fault)
{
  switch (fault) {
    case MotionFault::kNone: return "NONE";
    case MotionFault::kSlip: return "WHEEL_SLIP";
    case MotionFault::kStalled: return "MECHANICAL_STALL";
    case MotionFault::kEncoderFailure: return "ENCODER_FAILURE";
  }
  return "NONE";
}

FusionMode select_mode(
  bool initialized, bool vision, bool wheel, bool imu, bool wheel_yaw_backup,
  bool visual_realigned, bool stalled, double seconds_without_vision,
  double distance_without_vision, double max_seconds_without_vision,
  double max_distance_without_vision, double max_wheel_only_seconds,
  double max_wheel_only_distance)
{
  if (!initialized) {
    return FusionMode::kFault;
  }
  if (stalled) {
    return FusionMode::kFaultStalled;
  }
  if (vision) {
    if (visual_realigned) {
      return FusionMode::kVisualRealigned;
    }
    if (wheel && imu) {
      return FusionMode::kFull;
    }
    if (!wheel && imu) {
      return FusionMode::kNoWheel;
    }
    if (wheel && !imu) {
      return FusionMode::kNoImu;
    }
    return FusionMode::kVisionOnly;
  }

  // Once IMU yaw-rate is back, wheel + IMU can continue without waiting for
  // visual recovery. The bounded window applies only to pure wheel fallback.
  if (wheel && imu) {
    return FusionMode::kNoVision;
  }
  const bool within_dead_reckoning_limit =
    seconds_without_vision <= max_seconds_without_vision &&
    distance_without_vision <= max_distance_without_vision;
  const bool within_wheel_only_limit =
    seconds_without_vision <= max_wheel_only_seconds &&
    distance_without_vision <= max_wheel_only_distance;
  if (wheel && !imu && wheel_yaw_backup &&
    within_dead_reckoning_limit && within_wheel_only_limit)
  {
    return FusionMode::kWheelOnly;
  }
  return FusionMode::kFault;
}

const char * mode_name(FusionMode mode)
{
  switch (mode) {
    case FusionMode::kFull: return "FULL";
    case FusionMode::kNoVision: return "DEGRADED_NO_VISION";
    case FusionMode::kNoWheel: return "DEGRADED_NO_WHEEL";
    case FusionMode::kNoImu: return "DEGRADED_NO_IMU";
    case FusionMode::kVisionOnly: return "DEGRADED_VISION_ONLY";
    case FusionMode::kWheelOnly: return "DEGRADED_WHEEL_ONLY";
    case FusionMode::kVisualRealigned: return "DEGRADED_VISUAL_REALIGNED";
    case FusionMode::kFaultStalled: return "FAULT_STALLED";
    case FusionMode::kFault: return "FAULT";
  }
  return "FAULT";
}

}  // namespace fused_odometry
