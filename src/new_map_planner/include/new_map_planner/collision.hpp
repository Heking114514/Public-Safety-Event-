#ifndef NEW_MAP_PLANNER__COLLISION_HPP_
#define NEW_MAP_PLANNER__COLLISION_HPP_

#include "new_map_planner/types.hpp"
#include "navigation_safety/checker.hpp"

#include <vector>

namespace new_map_planner {

class CollisionChecker {
public:
  explicit CollisionChecker(const PlannerConfig &config);

  bool pose_is_free(const Pose &pose,
                    const std::vector<Polygon> &dynamic_obstacles) const;
  bool segment_is_free(const Point &begin, const Point &end,
                       const std::vector<Polygon> &dynamic_obstacles) const;
  bool segment_is_free(const Point &begin, const Point &end, double heading,
                       const std::vector<Polygon> &dynamic_obstacles) const;
  bool sweep_is_free(const std::vector<Pose> &poses,
                     const std::vector<Polygon> &dynamic_obstacles) const;
  std::vector<int8_t> occupancy_data() const;

private:
  const PlannerConfig &config_;
  navigation_safety::Footprint footprint_;
  std::vector<navigation_safety::Rectangle> free_regions_;
  std::vector<navigation_safety::Polygon> static_obstacles_;
};

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__COLLISION_HPP_
