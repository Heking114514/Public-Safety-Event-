#ifndef NEW_MAP_PLANNER__TRANSFORM_HPP_
#define NEW_MAP_PLANNER__TRANSFORM_HPP_

#include "new_map_planner/types.hpp"

namespace new_map_planner {

Point MapToArena(const Point &point, const MapTransform &transform);
Point ArenaToMap(const Point &point, const MapTransform &transform);
Pose MapToArena(const Pose &pose, const MapTransform &transform);
Pose ArenaToMap(const Pose &pose, const MapTransform &transform);
Polygon MapToArena(const Polygon &polygon, const MapTransform &transform);

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__TRANSFORM_HPP_
