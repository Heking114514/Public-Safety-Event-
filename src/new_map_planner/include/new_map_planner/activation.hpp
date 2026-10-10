#ifndef NEW_MAP_PLANNER__ACTIVATION_HPP_
#define NEW_MAP_PLANNER__ACTIVATION_HPP_

namespace new_map_planner {

// Recovery plans have their own execution contract and must not use the
// ordinary forward navigation path publisher.
constexpr bool ShouldPublishNavigationPath(bool requested_activate,
                                            bool recovery_only) noexcept {
  return requested_activate && !recovery_only;
}

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__ACTIVATION_HPP_
