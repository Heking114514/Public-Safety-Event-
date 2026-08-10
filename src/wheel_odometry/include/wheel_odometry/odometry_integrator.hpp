#pragma once

#include <cstdint>
#include <string>

namespace wheel_odometry
{

struct IntegratorConfig
{
  double wheel_radius_m{0.0325};
  double ticks_per_revolution{1925.0};
  double wheel_track_m{0.254};
  double left_distance_scale{1.000};
  double right_distance_scale{1.010};
  double yaw_slip_scale{1.0};
  double max_wheel_speed_mps{2.0};
  double min_dt_s{1.0e-4};
  double nominal_sample_period_s{0.02};
  uint32_t nominal_sequence_increment{1};

  // Variances grow with motion because wheel scale and slip errors accumulate.
  double pose_xy_variance{0.02};
  double pose_yaw_variance{0.05};
  double twist_linear_variance{0.05};
  double twist_yaw_variance{0.10};
  double pose_xy_variance_per_meter{0.02};
  double pose_xy_variance_per_radian{0.005};
  double pose_yaw_variance_per_meter{0.02};
  double pose_yaw_variance_per_radian{0.05};
  double pose_xy_variance_per_missing_sample{0.001};
  double pose_yaw_variance_per_missing_sample{0.002};
  double pose_xy_variance_per_rebase{0.01};
  double pose_yaw_variance_per_rebase{0.02};
  double twist_linear_variance_per_missing_sample{0.01};
  double twist_yaw_variance_per_missing_sample{0.02};
  double twist_linear_variance_per_interval_ratio{0.02};
  double twist_yaw_variance_per_interval_ratio{0.04};
  double twist_linear_variance_per_wheel_difference_mps{0.05};
  double twist_yaw_variance_per_wheel_difference_mps{0.15};
};

struct EncoderSample
{
  uint32_t mcu_time_ms{0};
  uint32_t sequence{0};
  int32_t left_total_ticks{0};
  int32_t right_total_ticks{0};
};

struct OdometryState
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double linear_velocity_mps{0.0};
  double angular_velocity_radps{0.0};
};

struct OdometryUncertainty
{
  double pose_xy_variance{0.0};
  double pose_yaw_variance{0.0};
  double twist_linear_variance{0.0};
  double twist_yaw_variance{0.0};
};

struct IntegratorStatistics
{
  uint64_t integrated_samples{0};
  uint64_t duplicate_samples{0};
  uint64_t sequence_gap_events{0};
  uint64_t missing_samples{0};
  uint64_t rebase_count{0};
  uint64_t tick_jump_count{0};
  double accumulated_wheel_travel_m{0.0};
  double accumulated_abs_turn_rad{0.0};
};

enum class UpdateStatus
{
  kInitialized,
  kIntegrated,
  kDuplicate,
  kRebasedSequenceRegression,
  kRebasedTimeRegression,
  kRebasedInvalidDt,
  kRebasedTickJump,
};

struct UpdateResult
{
  UpdateStatus status{UpdateStatus::kInitialized};
  bool publish{false};
  uint32_t sequence_delta{0};
  double dt_s{0.0};
  int64_t left_delta_ticks{0};
  int64_t right_delta_ticks{0};
};

class OdometryIntegrator
{
public:
  explicit OdometryIntegrator(const IntegratorConfig & config);

  UpdateResult update(const EncoderSample & sample);
  const OdometryState & state() const {return state_;}
  const OdometryUncertainty & uncertainty() const {return uncertainty_;}
  const IntegratorStatistics & statistics() const {return statistics_;}
  void reset_pose(double x_m = 0.0, double y_m = 0.0, double yaw_rad = 0.0);

  static const char * status_string(UpdateStatus status);

private:
  void set_baseline(const EncoderSample & sample);
  static bool is_uint32_wrap(uint32_t previous, uint32_t current);
  static int64_t wrapped_int32_delta(int32_t previous, int32_t current);
  static double normalize_angle(double angle);

  IntegratorConfig config_;
  OdometryState state_;
  OdometryUncertainty uncertainty_;
  IntegratorStatistics statistics_;
  EncoderSample previous_;
  bool initialized_{false};
};

}  // namespace wheel_odometry
