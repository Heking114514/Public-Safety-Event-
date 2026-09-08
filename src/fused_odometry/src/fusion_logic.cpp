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

Pose2d compose_pose(const Pose2d & parent_from_middle, const Pose2d & middle_from_body)
{
  const double cosine = std::cos(parent_from_middle.yaw);
  const double sine = std::sin(parent_from_middle.yaw);
  return {
    parent_from_middle.x + cosine * middle_from_body.x - sine * middle_from_body.y,
    parent_from_middle.y + sine * middle_from_body.x + cosine * middle_from_body.y,
    wrap_angle(parent_from_middle.yaw + middle_from_body.yaw)};
}

Pose2d inverse_pose(const Pose2d & parent_from_body)
{
  const double cosine = std::cos(parent_from_body.yaw);
  const double sine = std::sin(parent_from_body.yaw);
  return {
    -cosine * parent_from_body.x - sine * parent_from_body.y,
    sine * parent_from_body.x - cosine * parent_from_body.y,
    wrap_angle(-parent_from_body.yaw)};
}

Pose2d interpolate_pose(const Pose2d & from, const Pose2d & to, double fraction)
{
  const double bounded = std::clamp(fraction, 0.0, 1.0);
  return {
    from.x + bounded * (to.x - from.x),
    from.y + bounded * (to.y - from.y),
    wrap_angle(from.yaw + bounded * wrap_angle(to.yaw - from.yaw))};
}

bool yaw_rates_excited(double wheel_rate, double imu_rate, double minimum_rate)
{
  return std::isfinite(wheel_rate) && std::isfinite(imu_rate) &&
         std::isfinite(minimum_rate) && minimum_rate > 0.0 &&
         std::abs(wheel_rate) >= minimum_rate && std::abs(imu_rate) >= minimum_rate;
}

bool yaw_rates_consistent(double wheel_rate, double imu_rate, double maximum_residual)
{
  return std::isfinite(wheel_rate) && std::isfinite(imu_rate) &&
         std::isfinite(maximum_residual) && maximum_residual > 0.0 &&
         std::abs(wheel_rate - imu_rate) <= maximum_residual;
}

double wheel_vx_turn_covariance_scale(
  double imu_yaw_rate, double command_yaw_rate,
  bool imu_valid, bool command_valid,
  double downweight_start_rate, double full_downweight_rate,
  double maximum_scale)
{
  if (!std::isfinite(downweight_start_rate) ||
    !std::isfinite(full_downweight_rate) || !std::isfinite(maximum_scale) ||
    downweight_start_rate < 0.0 || full_downweight_rate <= downweight_start_rate ||
    maximum_scale < 1.0)
  {
    return 1.0;
  }

  double turn_rate = 0.0;
  if (imu_valid && std::isfinite(imu_yaw_rate)) {
    turn_rate = std::abs(imu_yaw_rate);
  } else if (command_valid && std::isfinite(command_yaw_rate)) {
    turn_rate = std::abs(command_yaw_rate);
  }
  if (turn_rate <= downweight_start_rate) {
    return 1.0;
  }

  const double fraction = std::clamp(
    (turn_rate - downweight_start_rate) /
    (full_downweight_rate - downweight_start_rate), 0.0, 1.0);
  return 1.0 + fraction * fraction * (maximum_scale - 1.0);
}

bool zero_wheel_vx_during_in_place_turn(
  double command_velocity, double command_yaw_rate, bool command_valid,
  double maximum_linear_speed, double minimum_yaw_rate)
{
  return command_valid && std::isfinite(command_velocity) &&
         std::isfinite(command_yaw_rate) && std::isfinite(maximum_linear_speed) &&
         std::isfinite(minimum_yaw_rate) && maximum_linear_speed >= 0.0 &&
         minimum_yaw_rate >= 0.0 &&
         std::abs(command_velocity) <= maximum_linear_speed &&
         std::abs(command_yaw_rate) >= minimum_yaw_rate;
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

double wheel_visual_rejection_residual(
  double wheel_velocity, double visual_velocity,
  double visual_stationary_threshold, double stationary_wheel_threshold)
{
  if (!std::isfinite(wheel_velocity) || !std::isfinite(visual_velocity) ||
    !std::isfinite(visual_stationary_threshold) ||
    !std::isfinite(stationary_wheel_threshold) || visual_stationary_threshold < 0.0 ||
    stationary_wheel_threshold <= 0.0)
  {
    return std::numeric_limits<double>::infinity();
  }
  const double residual = std::abs(wheel_velocity - visual_velocity);
  if (std::abs(visual_velocity) <= visual_stationary_threshold &&
    std::abs(wheel_velocity) > stationary_wheel_threshold)
  {
    // Promote stationary-vision wheel motion above the regular rejection
    // threshold without changing the residual used for covariance reporting.
    return std::numeric_limits<double>::infinity();
  }
  return residual;
}

bool motion_command_is_stationary(
  double linear_velocity, double angular_velocity, bool command_fresh,
  double maximum_linear_speed, double maximum_angular_speed)
{
  return command_fresh && std::isfinite(linear_velocity) &&
    std::isfinite(angular_velocity) && std::isfinite(maximum_linear_speed) &&
    std::isfinite(maximum_angular_speed) &&
    std::abs(linear_velocity) <= std::max(0.0, maximum_linear_speed) &&
    std::abs(angular_velocity) <= std::max(0.0, maximum_angular_speed);
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

void TranslationAligner::clear()
{
  offset_x_ = 0.0;
  offset_y_ = 0.0;
  initialized_ = false;
}

void TranslationAligner::align_to(const Pose2d & raw, const Pose2d & target)
{
  offset_x_ = target.x - raw.x;
  offset_y_ = target.y - raw.y;
  initialized_ = true;
}

Pose2d TranslationAligner::apply(const Pose2d & raw) const
{
  if (!initialized_) {
    return raw;
  }
  return {raw.x + offset_x_, raw.y + offset_y_, raw.yaw};
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

YawBiasEstimator::YawBiasEstimator(double time_constant_seconds, double maximum_bias)
: time_constant_seconds_(time_constant_seconds), maximum_bias_(maximum_bias)
{
  if (!std::isfinite(time_constant_seconds_) || time_constant_seconds_ <= 0.0 ||
    !std::isfinite(maximum_bias_) || maximum_bias_ < 0.0)
  {
    throw std::invalid_argument("yaw bias parameters must be finite and valid");
  }
}

void YawBiasEstimator::reset()
{
  bias_ = 0.0;
  last_time_seconds_ = 0.0;
  initialized_ = false;
}

double YawBiasEstimator::correct(
  double time_seconds, double measured_rate, double reference_rate,
  bool reference_valid, bool learn)
{
  if (!std::isfinite(measured_rate)) {
    return measured_rate;
  }
  if (!initialized_) {
    initialized_ = std::isfinite(time_seconds);
    last_time_seconds_ = time_seconds;
    return measured_rate - bias_;
  }

  const double dt = time_seconds - last_time_seconds_;
  if (std::isfinite(time_seconds)) {
    last_time_seconds_ = time_seconds;
  }
  if (learn && reference_valid && std::isfinite(reference_rate) &&
    std::isfinite(dt) && dt > 0.0 && dt <= 1.0)
  {
    const double alpha = 1.0 - std::exp(-dt / time_constant_seconds_);
    const double target_bias = measured_rate - reference_rate;
    bias_ = std::clamp(
      bias_ + alpha * (target_bias - bias_), -maximum_bias_, maximum_bias_);
  }
  return measured_rate - bias_;
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
    return FusionMode::kInitializing;
  }
  if (stalled) {
    return FusionMode::kFaultStalled;
  }
  if (vision) {
    if (visual_realigned) {
      return FusionMode::kVisualRealigned;
    }
    // Wheel odometry is supervision-only while vision is healthy. Its
    // availability and motion-fault diagnostics must not change the fusion
    // mode or navigation speed when it does not contribute to the EKF.
    return imu ? FusionMode::kFull : FusionMode::kNoImu;
  }

  const bool within_dead_reckoning_limit =
    seconds_without_vision <= max_seconds_without_vision &&
    distance_without_vision <= max_distance_without_vision;
  const bool within_wheel_only_limit =
    seconds_without_vision <= max_wheel_only_seconds &&
    distance_without_vision <= max_wheel_only_distance;

  // Wheel + IMU is the tunnel fallback, not an unlimited navigation mode.
  // Bound it by both elapsed outage and travelled distance so a permanent
  // camera failure cannot silently run the remainder of a mission.
  if (wheel && imu && within_dead_reckoning_limit) {
    return FusionMode::kNoVision;
  }
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
    case FusionMode::kInitializing: return "WAITING_FOR_INITIALIZATION";
    case FusionMode::kNoVision: return "DEGRADED_NO_VISION";
    case FusionMode::kNoWheel: return "DEGRADED_NO_WHEEL";
    case FusionMode::kNoImu: return "DEGRADED_NO_IMU";
    case FusionMode::kVisionOnly: return "DEGRADED_VISION_ONLY";
    case FusionMode::kWheelOnly: return "DEGRADED_WHEEL_ONLY";
    case FusionMode::kVisualRealigned: return "DEGRADED_VISUAL_REALIGNED";
    case FusionMode::kFaultStalled: return "FAULT_STALLED";
    case FusionMode::kFaultInitTimeout: return "FAULT_INIT_TIMEOUT";
    case FusionMode::kFault: return "FAULT";
  }
  return "FAULT";
}

HealthSeverity health_severity(
  FusionMode mode, bool wheel_healthy, MotionFault motion_fault)
{
  if (mode == FusionMode::kFault || mode == FusionMode::kFaultStalled ||
    mode == FusionMode::kFaultInitTimeout ||
    motion_fault == MotionFault::kStalled)
  {
    return HealthSeverity::kError;
  }
  if (mode != FusionMode::kFull || !wheel_healthy || motion_fault != MotionFault::kNone) {
    return HealthSeverity::kWarning;
  }
  return HealthSeverity::kOk;
}

}  // namespace fused_odometry
