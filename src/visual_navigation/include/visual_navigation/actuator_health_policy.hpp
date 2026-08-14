#ifndef VISUAL_NAVIGATION__ACTUATOR_HEALTH_POLICY_HPP_
#define VISUAL_NAVIGATION__ACTUATOR_HEALTH_POLICY_HPP_

namespace visual_navigation
{

inline bool ActuatorHealthIsValid(
  bool required, bool has_status, bool status_is_fresh, bool connected)
{
  return !required || (has_status && status_is_fresh && connected);
}

inline bool ShouldHoldForActuatorRecovery(bool navigation_active, bool actuator_health_valid)
{
  return navigation_active && !actuator_health_valid;
}

}  // namespace visual_navigation

#endif  // VISUAL_NAVIGATION__ACTUATOR_HEALTH_POLICY_HPP_
