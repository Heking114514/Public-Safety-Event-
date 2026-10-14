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

  explicit FusionHealthPolicy(std::vector<std::string> allowed_states)
  : allowed_states_(std::move(allowed_states))
  {
  }

  FusionHealthDecision Evaluate(const std::string &state) const
  {
    FusionHealthDecision decision;
    decision.fault = state == "FAULT" || state.compare(0, 6, "FAULT_") == 0;
    if (decision.fault)
      return decision;

    decision.allowed = std::find(
      allowed_states_.begin(), allowed_states_.end(), state) != allowed_states_.end();
    return decision;
  }

private:
  std::vector<std::string> allowed_states_;
};

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__FUSION_HEALTH_POLICY_HPP_
