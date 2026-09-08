#pragma once

#include <cstddef>
#include <deque>
#include <string>

namespace fused_odometry
{

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle);

double disagreement_covariance_scale(
  double absolute_residual, double soft_threshold, double residual_cap);

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

Pose2d compose_pose(const Pose2d & parent_from_middle, const Pose2d & middle_from_body);
Pose2d inverse_pose(const Pose2d & parent_from_body);
Pose2d interpolate_pose(const Pose2d & from, const Pose2d & to, double fraction);
bool yaw_rates_excited(double wheel_rate, double imu_rate, double minimum_rate);
bool yaw_rates_consistent(double wheel_rate, double imu_rate, double maximum_residual);

double wheel_vx_turn_covariance_scale(
  double imu_yaw_rate, double command_yaw_rate,
  bool imu_valid, bool command_valid,
  double downweight_start_rate, double full_downweight_rate,
  double maximum_scale);

bool zero_wheel_vx_during_in_place_turn(
  double command_velocity, double command_yaw_rate, bool command_valid,
  double maximum_linear_speed, double minimum_yaw_rate);

bool pose_residual_within(
  const Pose2d & measurement, const Pose2d & reference,
  double max_position_residual, double max_yaw_residual);

double wheel_visual_rejection_residual(
  double wheel_velocity, double visual_velocity,
  double visual_stationary_threshold, double stationary_wheel_threshold);

bool motion_command_is_stationary(
  double linear_velocity, double angular_velocity, bool command_fresh,
  double maximum_linear_speed, double maximum_angular_speed);

class PoseAligner
{
public:
  void clear();
  void align_to(const Pose2d & raw, const Pose2d & target);
  Pose2d apply(const Pose2d & raw) const;
  bool initialized() const {return initialized_;}

private:
  Pose2d offset_{};
  bool initialized_{false};
};

class TranslationAligner
{
public:
  void clear();
  void align_to(const Pose2d & raw, const Pose2d & target);
  Pose2d apply(const Pose2d & raw) const;
  bool initialized() const {return initialized_;}

private:
  double offset_x_{0.0};
  double offset_y_{0.0};
  bool initialized_{false};
};

class ResidualGate
{
public:
  ResidualGate(double reject_threshold, std::size_t bad_samples, std::size_t recovery_samples);
  bool update(double absolute_residual);
  double covariance_scale(double absolute_residual, double soft_threshold) const;
  bool rejected() const {return rejected_;}

private:
  double reject_threshold_;
  std::size_t bad_samples_;
  std::size_t recovery_samples_;
  std::size_t bad_count_{0};
  std::size_t good_count_{0};
  bool rejected_{false};
};

class RobustWindow
{
public:
  explicit RobustWindow(double duration_seconds);
  void add(double time_seconds, double sample);
  void clear();
  std::size_t size() const {return samples_.size();}
  bool ready(std::size_t minimum_samples) const;
  double median() const;
  double mad() const;

private:
  double duration_seconds_;
  std::deque<std::pair<double, double>> samples_;
};

// Estimates only a slowly varying gyro z bias. The reference is normally the
// robust visual yaw rate; callers decide when learning is safe (straight,
// healthy visual tracking). During turns the last bias is held constant.
class YawBiasEstimator
{
public:
  YawBiasEstimator(double time_constant_seconds = 3.0, double maximum_bias = 0.08);

  void reset();
  double correct(
    double time_seconds, double measured_rate, double reference_rate,
    bool reference_valid, bool learn);
  double bias() const {return bias_;}

private:
  double time_constant_seconds_;
  double maximum_bias_;
  double bias_{0.0};
  double last_time_seconds_{0.0};
  bool initialized_{false};
};

enum class MotionFault
{
  kNone,
  kSlip,
  kStalled,
  kEncoderFailure
};

struct MotionClassifierConfig
{
  double command_threshold{0.08};
  double moving_threshold{0.04};
  double stationary_threshold{0.02};
  double fault_dwell_seconds{0.6};
  double recovery_dwell_seconds{1.0};
};

class MotionClassifier
{
public:
  explicit MotionClassifier(const MotionClassifierConfig & config);
  MotionFault update(
    double time_seconds, double command_velocity, double wheel_velocity,
    double visual_velocity, bool command_valid, bool wheel_valid, bool visual_valid);
  MotionFault fault() const {return active_fault_;}

private:
  MotionFault classify(
    double command_velocity, double wheel_velocity, double visual_velocity,
    bool command_valid, bool wheel_valid, bool visual_valid) const;
  MotionClassifierConfig config_;
  MotionFault active_fault_{MotionFault::kNone};
  MotionFault candidate_fault_{MotionFault::kNone};
  double candidate_since_{0.0};
  double recovery_since_{0.0};
  bool candidate_active_{false};
  bool recovery_active_{false};
};

struct AngularStallConfig
{
  double command_threshold{0.30};
  double stationary_threshold{0.10};
  double fault_dwell_seconds{0.8};
  double recovery_dwell_seconds{1.0};
};

class AngularStallDetector
{
public:
  explicit AngularStallDetector(const AngularStallConfig & config);
  bool update(
    double time_seconds, double command_yaw_rate, double measured_yaw_rate,
    bool command_valid, bool measurement_valid);
  bool stalled() const {return stalled_;}

private:
  AngularStallConfig config_;
  double candidate_since_{0.0};
  double recovery_since_{0.0};
  bool candidate_active_{false};
  bool recovery_active_{false};
  bool stalled_{false};
};

const char * motion_fault_name(MotionFault fault);

enum class FusionMode
{
  kFull,
  kNoVision,
  kNoWheel,
  kNoImu,
  kVisionOnly,
  kWheelOnly,
  kVisualRealigned,
  kFaultStalled,
  kFault,
  // ORB has not produced enough coherent samples to establish a visual map.
  // This is a startup wait, not a confirmed runtime fault.
  kInitializing,
  kFaultInitTimeout
};

FusionMode select_mode(
  bool initialized, bool vision, bool wheel, bool imu, bool wheel_yaw_backup,
  bool visual_realigned, bool stalled, double seconds_without_vision,
  double distance_without_vision, double max_seconds_without_vision,
  double max_distance_without_vision, double max_wheel_only_seconds,
  double max_wheel_only_distance);
const char * mode_name(FusionMode mode);

enum class HealthSeverity
{
  kOk,
  kWarning,
  kError
};

HealthSeverity health_severity(
  FusionMode mode, bool wheel_healthy, MotionFault motion_fault);

}  // namespace fused_odometry
