#ifndef VISUAL_NAVIGATION__RATE_LIMITER_HPP_
#define VISUAL_NAVIGATION__RATE_LIMITER_HPP_

#include <algorithm>
#include <cmath>

namespace visual_navigation
{

inline double LimitRate(
  double target, double current, double rising_rate, double falling_rate, double dt)
{
  if (!std::isfinite(target) || !std::isfinite(current) || !std::isfinite(dt) || dt <= 0.0)
    return current;
  const double increase = std::max(0.0, rising_rate) * dt;
  const double decrease = std::max(0.0, falling_rate) * dt;
  return std::max(current - decrease, std::min(current + increase, target));
}

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__RATE_LIMITER_HPP_
