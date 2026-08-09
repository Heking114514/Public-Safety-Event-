#pragma once

#include <cstddef>
#include <deque>
#include <string>

namespace fused_odometry
{

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle);

struct Pose2d
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

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
  kFault
};

FusionMode select_mode(
  bool initialized, bool vision, bool wheel, bool imu, bool wheel_yaw_backup,
  bool visual_realigned, bool stalled, double seconds_without_vision,
  double distance_without_vision, double max_seconds_without_vision,
  double max_distance_without_vision, double max_wheel_only_seconds,
  double max_wheel_only_distance);
const char * mode_name(FusionMode mode);

}  // namespace fused_odometry
