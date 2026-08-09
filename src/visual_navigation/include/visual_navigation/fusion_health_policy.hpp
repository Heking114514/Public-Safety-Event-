#ifndef VISUAL_NAVIGATION__FUSION_HEALTH_POLICY_HPP_
#define VISUAL_NAVIGATION__FUSION_HEALTH_POLICY_HPP_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

namespace visual_navigation
{

struct FusionHealthDecision
{
  bool allowed{false};
  bool fault{false};
  double speed_scale{0.0};
};

struct FusionSpeedScales
{
  double fallback{0.5};
  double no_vision{0.5};
  double no_imu{0.65};
  double no_wheel{0.6};
  double vision_only{0.4};
  double wheel_only{0.25};
  double visual_realigned{0.5};
};

inline bool LocalizationCanStart(
  bool odometry_valid, const FusionHealthDecision &health, bool tracking_valid)
{
  return odometry_valid && health.allowed && !health.fault && tracking_valid;
}

inline bool ShouldLatchFusedLocalizationLoss(
  bool localization_was_valid, bool odometry_valid,
  const FusionHealthDecision &health)
{
  return health.fault ||
    (localization_was_valid && (!odometry_valid || !health.allowed));
}

class FusionHealthPolicy
{
public:
  FusionHealthPolicy() = default;

  FusionHealthPolicy(std::vector<std::string> allowed_states, double degraded_speed_scale)
  : allowed_states_(std::move(allowed_states)),
    speed_scales_({degraded_speed_scale, degraded_speed_scale, degraded_speed_scale,
      degraded_speed_scale, degraded_speed_scale, degraded_speed_scale,
      degraded_speed_scale})
  {
    ClampScales();
  }

  FusionHealthPolicy(
    std::vector<std::string> allowed_states, FusionSpeedScales speed_scales)
  : allowed_states_(std::move(allowed_states)), speed_scales_(speed_scales)
  {
    ClampScales();
  }

  FusionHealthDecision Evaluate(const std::string &state) const
  {
    FusionHealthDecision decision;
    decision.fault = state == "FAULT" || state.compare(0, 6, "FAULT_") == 0;
    if (decision.fault)
      return decision;

    decision.allowed = std::find(
      allowed_states_.begin(), allowed_states_.end(), state) != allowed_states_.end();
    if (!decision.allowed)
      return decision;

    decision.speed_scale = SpeedScale(state);
    return decision;
  }

private:
  static double ClampScale(double value)
  {
    return std::max(0.0, std::min(1.0, value));
  }

  void ClampScales()
  {
    speed_scales_.fallback = ClampScale(speed_scales_.fallback);
    speed_scales_.no_vision = ClampScale(speed_scales_.no_vision);
    speed_scales_.no_imu = ClampScale(speed_scales_.no_imu);
    speed_scales_.no_wheel = ClampScale(speed_scales_.no_wheel);
    speed_scales_.vision_only = ClampScale(speed_scales_.vision_only);
    speed_scales_.wheel_only = ClampScale(speed_scales_.wheel_only);
    speed_scales_.visual_realigned = ClampScale(speed_scales_.visual_realigned);
  }

  double SpeedScale(const std::string &state) const
  {
    if (state == "FULL")
      return 1.0;
    if (state == "DEGRADED_NO_VISION")
      return speed_scales_.no_vision;
    if (state == "DEGRADED_NO_IMU")
      return speed_scales_.no_imu;
    if (state == "DEGRADED_NO_WHEEL")
      return speed_scales_.no_wheel;
    if (state == "DEGRADED_VISION_ONLY")
      return speed_scales_.vision_only;
    if (state == "DEGRADED_WHEEL_ONLY")
      return speed_scales_.wheel_only;
    if (state == "DEGRADED_VISUAL_REALIGNED")
      return speed_scales_.visual_realigned;
    return state.compare(0, 9, "DEGRADED_") == 0 ? speed_scales_.fallback : 1.0;
  }

  std::vector<std::string> allowed_states_;
  FusionSpeedScales speed_scales_;
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__FUSION_HEALTH_POLICY_HPP_
