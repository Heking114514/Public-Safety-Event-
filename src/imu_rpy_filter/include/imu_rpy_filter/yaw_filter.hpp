#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace imu_rpy_filter
{

constexpr double kPi = 3.14159265358979323846;

inline double wrap_angle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

class StationaryDetector
{
public:
  StationaryDetector(
    double enter_dwell_time, double exit_dwell_time, double acceleration_enter,
    double acceleration_exit, double gyro_enter, double gyro_exit)
  : enter_dwell_time_(enter_dwell_time),
    exit_dwell_time_(exit_dwell_time),
    acceleration_enter_(acceleration_enter),
    acceleration_exit_(acceleration_exit),
    gyro_enter_(gyro_enter),
    gyro_exit_(gyro_exit)
  {
  }

  bool update(double stamp, double acceleration_error, double gyro_norm)
  {
    const bool quiet =
      acceleration_error <= acceleration_enter_ && gyro_norm <= gyro_enter_;
    const bool moving =
      acceleration_error > acceleration_exit_ || gyro_norm > gyro_exit_;

    if (active_) {
      candidate_since_ = -1.0;
      if (moving) {
        if (exit_candidate_since_ < 0.0) {
          exit_candidate_since_ = stamp;
        } else if (stamp - exit_candidate_since_ >= exit_dwell_time_) {
          active_ = false;
          exit_candidate_since_ = -1.0;
        }
      } else if (exit_candidate_since_ >= 0.0) {
        exit_candidate_since_ = -1.0;
        ++rejected_transient_count_;
      }
      return active_;
    }

    exit_candidate_since_ = -1.0;
    if (quiet) {
      if (candidate_since_ < 0.0) {
        candidate_since_ = stamp;
      } else if (stamp - candidate_since_ >= enter_dwell_time_) {
        active_ = true;
        candidate_since_ = -1.0;
      }
    } else {
      candidate_since_ = -1.0;
    }
    return active_;
  }

  bool active() const {return active_;}
  bool has_candidate() const {return candidate_since_ >= 0.0;}
  bool exit_pending() const {return exit_candidate_since_ >= 0.0;}
  std::uint64_t rejected_transient_count() const {return rejected_transient_count_;}

private:
  double enter_dwell_time_;
  double exit_dwell_time_;
  double acceleration_enter_;
  double acceleration_exit_;
  double gyro_enter_;
  double gyro_exit_;
  double candidate_since_{-1.0};
  double exit_candidate_since_{-1.0};
  bool active_{false};
  std::uint64_t rejected_transient_count_{0};
};

class YawBiasKalman
{
public:
  YawBiasKalman(double gyro_noise, double bias_walk)
  : gyro_noise_(gyro_noise), bias_walk_(bias_walk)
  {
  }

  void predict(double measured_yaw_rate, double dt)
  {
    yaw_ = wrap_angle(yaw_ + (measured_yaw_rate - bias_) * dt);

    const double old_p00 = p00_;
    const double old_p01 = p01_;
    const double old_p10 = p10_;
    const double old_p11 = p11_;
    p00_ = old_p00 - dt * (old_p01 + old_p10) + dt * dt * old_p11 +
      std::pow(gyro_noise_ * dt, 2);
    p01_ = old_p01 - dt * old_p11;
    p10_ = old_p10 - dt * old_p11;
    p11_ = old_p11 + bias_walk_ * bias_walk_ * dt;
  }

  double innovation(double measured_yaw) const
  {
    return wrap_angle(measured_yaw - yaw_);
  }

  void update(double measured_yaw, double measurement_std)
  {
    const double residual = innovation(measured_yaw);
    const double residual_covariance = p00_ + measurement_std * measurement_std;
    const double gain_yaw = p00_ / residual_covariance;
    const double gain_bias = p10_ / residual_covariance;

    yaw_ = wrap_angle(yaw_ + gain_yaw * residual);
    bias_ = std::clamp(bias_ + gain_bias * residual, radians(-5.0), radians(5.0));

    const double old_p00 = p00_;
    const double old_p01 = p01_;
    const double old_p10 = p10_;
    const double old_p11 = p11_;
    p00_ = std::max((1.0 - gain_yaw) * old_p00, 1e-12);
    p01_ = (1.0 - gain_yaw) * old_p01;
    p10_ = old_p10 - gain_bias * old_p00;
    p11_ = std::max(old_p11 - gain_bias * old_p01, 1e-12);
    symmetrize();
  }

  void update_bias(double measured_bias, double measurement_std)
  {
    const double residual = measured_bias - bias_;
    const double residual_covariance = p11_ + measurement_std * measurement_std;
    const double gain_yaw = p01_ / residual_covariance;
    const double gain_bias = p11_ / residual_covariance;

    yaw_ = wrap_angle(yaw_ + gain_yaw * residual);
    bias_ = std::clamp(bias_ + gain_bias * residual, radians(-5.0), radians(5.0));

    const double old_p00 = p00_;
    const double old_p01 = p01_;
    const double old_p10 = p10_;
    const double old_p11 = p11_;
    p00_ = std::max(old_p00 - gain_yaw * old_p10, 1e-12);
    p01_ = old_p01 - gain_yaw * old_p11;
    p10_ = old_p10 - gain_bias * old_p10;
    p11_ = std::max(old_p11 - gain_bias * old_p11, 1e-12);
    symmetrize();
  }

  // A stationary anchor is the filter's own heading at the moment motion
  // stopped, not an independent absolute-yaw measurement. Hold the state
  // without making the covariance artificially smaller.
  void hold_yaw(double yaw)
  {
    yaw_ = wrap_angle(yaw);
  }

  double yaw() const {return yaw_;}
  double bias() const {return bias_;}
  double yaw_variance() const {return p00_;}
  double bias_variance() const {return p11_;}

private:
  static double radians(double degrees)
  {
    return degrees * kPi / 180.0;
  }

  void symmetrize()
  {
    const double off_diagonal = 0.5 * (p01_ + p10_);
    p01_ = off_diagonal;
    p10_ = off_diagonal;
  }

  double yaw_{0.0};
  double bias_{0.0};
  double p00_{std::pow(radians(5.0), 2)};
  double p01_{0.0};
  double p10_{0.0};
  double p11_{std::pow(radians(1.0), 2)};
  double gyro_noise_;
  double bias_walk_;
};

}  // namespace imu_rpy_filter
