#include "wheel_odometry/odometry_integrator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace wheel_odometry
{
namespace
{
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
constexpr uint64_t kUint32Modulus = uint64_t{1} << 32;
constexpr uint32_t kWrapHighWatermark = 0xf0000000U;
constexpr uint32_t kWrapLowWatermark = 0x0fffffffU;
}  // namespace

OdometryIntegrator::OdometryIntegrator(const IntegratorConfig & config)
: config_(config)
{
  if (!(config_.wheel_radius_m > 0.0) ||
    !(config_.left_encoder_counts_per_revolution > 0.0) ||
    !(config_.right_encoder_counts_per_revolution > 0.0) ||
    !(config_.wheel_track_m > 0.0) || !(config_.left_distance_scale > 0.0) ||
    !(config_.right_distance_scale > 0.0) || !(config_.yaw_slip_scale > 0.0) ||
    !(config_.max_wheel_speed_mps > 0.0) ||
    !(config_.min_dt_s > 0.0) || !(config_.nominal_sample_period_s > 0.0) ||
    config_.nominal_sequence_increment == 0U)
  {
    throw std::invalid_argument("wheel odometry parameters must be finite and positive");
  }

  const double uncertainty_parameters[] = {
    config_.pose_xy_variance, config_.pose_yaw_variance,
    config_.twist_linear_variance, config_.twist_yaw_variance,
    config_.pose_xy_variance_per_meter, config_.pose_xy_variance_per_radian,
    config_.pose_yaw_variance_per_meter, config_.pose_yaw_variance_per_radian,
    config_.pose_xy_variance_per_missing_sample,
    config_.pose_yaw_variance_per_missing_sample,
    config_.pose_xy_variance_per_rebase, config_.pose_yaw_variance_per_rebase,
    config_.twist_linear_variance_per_missing_sample,
    config_.twist_yaw_variance_per_missing_sample,
    config_.twist_linear_variance_per_interval_ratio,
    config_.twist_yaw_variance_per_interval_ratio,
    config_.twist_linear_variance_per_wheel_difference_mps,
    config_.twist_yaw_variance_per_wheel_difference_mps};
  for (const double value : uncertainty_parameters) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument("wheel odometry variances must be finite and non-negative");
    }
  }
  uncertainty_ = {
    config_.pose_xy_variance, config_.pose_yaw_variance,
    config_.twist_linear_variance, config_.twist_yaw_variance};
}

UpdateResult OdometryIntegrator::update(const EncoderSample & sample)
{
  if (!initialized_) {
    set_baseline(sample);
    return {UpdateStatus::kInitialized, false};
  }

  if (sample.sequence == previous_.sequence) {
    ++statistics_.duplicate_samples;
    return {UpdateStatus::kDuplicate, false};
  }
  if (sample.sequence < previous_.sequence &&
    !is_uint32_wrap(previous_.sequence, sample.sequence))
  {
    ++statistics_.rebase_count;
    uncertainty_.pose_xy_variance += config_.pose_xy_variance_per_rebase;
    uncertainty_.pose_yaw_variance += config_.pose_yaw_variance_per_rebase;
    set_baseline(sample);
    return {UpdateStatus::kRebasedSequenceRegression, false};
  }
  if (sample.mcu_time_ms < previous_.mcu_time_ms &&
    !is_uint32_wrap(previous_.mcu_time_ms, sample.mcu_time_ms))
  {
    ++statistics_.rebase_count;
    uncertainty_.pose_xy_variance += config_.pose_xy_variance_per_rebase;
    uncertainty_.pose_yaw_variance += config_.pose_yaw_variance_per_rebase;
    set_baseline(sample);
    return {UpdateStatus::kRebasedTimeRegression, false};
  }

  const uint32_t sequence_delta = sample.sequence - previous_.sequence;
  const uint32_t elapsed_ms = sample.mcu_time_ms - previous_.mcu_time_ms;
  const double dt_s = static_cast<double>(elapsed_ms) * 1.0e-3;
  if (!std::isfinite(dt_s) || dt_s < config_.min_dt_s) {
    ++statistics_.rebase_count;
    uncertainty_.pose_xy_variance += config_.pose_xy_variance_per_rebase;
    uncertainty_.pose_yaw_variance += config_.pose_yaw_variance_per_rebase;
    set_baseline(sample);
    return {UpdateStatus::kRebasedInvalidDt, false, sequence_delta, dt_s};
  }

  const int64_t left_delta_ticks = wrapped_int32_delta(
    previous_.left_total_ticks, sample.left_total_ticks);
  const int64_t right_delta_ticks = wrapped_int32_delta(
    previous_.right_total_ticks, sample.right_total_ticks);
  const double wheel_circumference_m = kTwoPi * config_.wheel_radius_m;
  const double left_distance_m = static_cast<double>(left_delta_ticks) *
    wheel_circumference_m / config_.left_encoder_counts_per_revolution *
    config_.left_distance_scale;
  const double right_distance_m = static_cast<double>(right_delta_ticks) *
    wheel_circumference_m / config_.right_encoder_counts_per_revolution *
    config_.right_distance_scale;

  const double max_distance_m = config_.max_wheel_speed_mps * dt_s;
  if (std::abs(left_distance_m) > max_distance_m ||
    std::abs(right_distance_m) > max_distance_m)
  {
    ++statistics_.rebase_count;
    ++statistics_.tick_jump_count;
    uncertainty_.pose_xy_variance += config_.pose_xy_variance_per_rebase;
    uncertainty_.pose_yaw_variance += config_.pose_yaw_variance_per_rebase;
    set_baseline(sample);
    return {
      UpdateStatus::kRebasedTickJump, false, sequence_delta, dt_s,
      left_delta_ticks, right_delta_ticks};
  }

  const double center_distance_m = 0.5 * (left_distance_m + right_distance_m);
  const double delta_yaw_rad = (right_distance_m - left_distance_m) /
    config_.wheel_track_m * config_.yaw_slip_scale;
  const double midpoint_yaw_rad = state_.yaw_rad + 0.5 * delta_yaw_rad;
  state_.x_m += center_distance_m * std::cos(midpoint_yaw_rad);
  state_.y_m += center_distance_m * std::sin(midpoint_yaw_rad);
  state_.yaw_rad = normalize_angle(state_.yaw_rad + delta_yaw_rad);
  state_.linear_velocity_mps = center_distance_m / dt_s;
  state_.angular_velocity_radps = delta_yaw_rad / dt_s;

  // The MCU updates its encoder sequence faster than it reports cumulative
  // counts, so a healthy serial frame can advance by more than one.
  const uint32_t sequence_increment = config_.nominal_sequence_increment;
  const uint32_t represented_intervals = std::max(
    1U, (sequence_delta + sequence_increment / 2U) / sequence_increment);
  const uint32_t missing_samples = represented_intervals > 1U ? represented_intervals - 1U : 0U;
  const double wheel_travel_m = 0.5 * (std::abs(left_distance_m) + std::abs(right_distance_m));
  const double abs_turn_rad = std::abs(delta_yaw_rad);
  const double wheel_difference_mps = std::abs(right_distance_m - left_distance_m) / dt_s;
  const double expected_dt_s = config_.nominal_sample_period_s *
    static_cast<double>(represented_intervals);
  const double interval_ratio = std::max(0.0, dt_s / expected_dt_s - 1.0);

  uncertainty_.pose_xy_variance +=
    config_.pose_xy_variance_per_meter * wheel_travel_m +
    config_.pose_xy_variance_per_radian * abs_turn_rad +
    config_.pose_xy_variance_per_missing_sample * static_cast<double>(missing_samples);
  uncertainty_.pose_yaw_variance +=
    config_.pose_yaw_variance_per_meter * wheel_travel_m +
    config_.pose_yaw_variance_per_radian * abs_turn_rad +
    config_.pose_yaw_variance_per_missing_sample * static_cast<double>(missing_samples);
  uncertainty_.twist_linear_variance = config_.twist_linear_variance +
    config_.twist_linear_variance_per_missing_sample * static_cast<double>(missing_samples) +
    config_.twist_linear_variance_per_interval_ratio * interval_ratio +
    config_.twist_linear_variance_per_wheel_difference_mps * wheel_difference_mps;
  uncertainty_.twist_yaw_variance = config_.twist_yaw_variance +
    config_.twist_yaw_variance_per_missing_sample * static_cast<double>(missing_samples) +
    config_.twist_yaw_variance_per_interval_ratio * interval_ratio +
    config_.twist_yaw_variance_per_wheel_difference_mps * wheel_difference_mps;

  ++statistics_.integrated_samples;
  if (missing_samples > 0U) {
    ++statistics_.sequence_gap_events;
    statistics_.missing_samples += missing_samples;
  }
  statistics_.accumulated_wheel_travel_m += wheel_travel_m;
  statistics_.accumulated_abs_turn_rad += abs_turn_rad;
  previous_ = sample;

  return {
    UpdateStatus::kIntegrated, true, sequence_delta, dt_s,
    left_delta_ticks, right_delta_ticks};
}

void OdometryIntegrator::reset_pose(double x_m, double y_m, double yaw_rad)
{
  state_ = {};
  state_.x_m = x_m;
  state_.y_m = y_m;
  state_.yaw_rad = normalize_angle(yaw_rad);
  uncertainty_ = {
    config_.pose_xy_variance, config_.pose_yaw_variance,
    config_.twist_linear_variance, config_.twist_yaw_variance};
  statistics_ = {};
  initialized_ = false;
}

const char * OdometryIntegrator::status_string(UpdateStatus status)
{
  switch (status) {
    case UpdateStatus::kInitialized: return "initialized";
    case UpdateStatus::kIntegrated: return "integrated";
    case UpdateStatus::kDuplicate: return "duplicate";
    case UpdateStatus::kRebasedSequenceRegression: return "sequence regression";
    case UpdateStatus::kRebasedTimeRegression: return "MCU time regression";
    case UpdateStatus::kRebasedInvalidDt: return "invalid time delta";
    case UpdateStatus::kRebasedTickJump: return "implausible encoder jump";
  }
  return "unknown";
}

void OdometryIntegrator::set_baseline(const EncoderSample & sample)
{
  previous_ = sample;
  initialized_ = true;
  state_.linear_velocity_mps = 0.0;
  state_.angular_velocity_radps = 0.0;
}

bool OdometryIntegrator::is_uint32_wrap(uint32_t previous, uint32_t current)
{
  return previous >= kWrapHighWatermark && current <= kWrapLowWatermark;
}

int64_t OdometryIntegrator::wrapped_int32_delta(int32_t previous, int32_t current)
{
  int64_t delta = static_cast<int64_t>(current) - static_cast<int64_t>(previous);
  if (delta > std::numeric_limits<int32_t>::max()) {
    delta -= static_cast<int64_t>(kUint32Modulus);
  } else if (delta < std::numeric_limits<int32_t>::min()) {
    delta += static_cast<int64_t>(kUint32Modulus);
  }
  return delta;
}

double OdometryIntegrator::normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace wheel_odometry
