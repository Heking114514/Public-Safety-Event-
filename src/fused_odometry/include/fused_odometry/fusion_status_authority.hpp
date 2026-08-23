#ifndef FUSED_ODOMETRY__FUSION_STATUS_AUTHORITY_HPP_
#define FUSED_ODOMETRY__FUSION_STATUS_AUTHORITY_HPP_

#include <chrono>
#include <string>

namespace fused_odometry
{

// Single interpretation of the fusion gate's status for downstream nodes.
// Nodes may keep their own sensor-pair timing, but they must not independently
// decide whether visual correction is globally allowed.
class FusionStatusAuthority
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void update(const std::string & status, TimePoint received_at = Clock::now())
  {
    status_ = status;
    received_at_ = received_at;
    received_ = true;
  }

  bool received() const {return received_;}

  bool fresh(TimePoint now, double timeout_seconds) const
  {
    return received_ && std::chrono::duration<double>(now - received_at_).count() >= 0.0 &&
      std::chrono::duration<double>(now - received_at_).count() <= timeout_seconds;
  }

  bool visual_correction_allowed(TimePoint now, double timeout_seconds) const
  {
    if (!fresh(now, timeout_seconds))
      return false;
    return status_ == "FULL" || status_ == "DEGRADED_NO_IMU" ||
      status_ == "DEGRADED_NO_WHEEL" || status_ == "DEGRADED_VISION_ONLY" ||
      status_ == "DEGRADED_VISUAL_REALIGNED";
  }

  const std::string & status() const {return status_;}

private:
  bool received_{false};
  std::string status_;
  TimePoint received_at_{};
};

}  // namespace fused_odometry

#endif  // FUSED_ODOMETRY__FUSION_STATUS_AUTHORITY_HPP_
