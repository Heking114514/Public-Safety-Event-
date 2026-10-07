#ifndef NEW_MAP_PLANNER__CONFIG_HPP_
#define NEW_MAP_PLANNER__CONFIG_HPP_

#include "new_map_planner/types.hpp"

#include <string>

namespace new_map_planner {

PlannerConfig LoadConfig(const std::string &map_path,
                         const std::string &vehicle_path);
void ValidateConfig(const PlannerConfig &config);

} // namespace new_map_planner

#endif // NEW_MAP_PLANNER__CONFIG_HPP_
