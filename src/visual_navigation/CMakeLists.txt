cmake_minimum_required(VERSION 3.10)
project(visual_navigation)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mission_control_interfaces REQUIRED)

add_library(corner_shunt_core src/corner_plan.cpp)
target_include_directories(corner_shunt_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# waypoint_navigator.cpp composes focused navigation members from src/;
# the ROS-facing node name and interfaces are unchanged.
add_executable(waypoint_navigator
  src/waypoint_navigator.cpp
  src/node_init.cpp
  src/route.cpp
  src/runtime.cpp
  src/obstacle_recovery.cpp
  src/corner_shunt.cpp
  src/control.cpp
  src/maneuvers.cpp
  src/path_following.cpp
  src/progress.cpp
  src/navigation.cpp
  src/output.cpp)
target_include_directories(waypoint_navigator PRIVATE include)
target_link_libraries(waypoint_navigator corner_shunt_core)
ament_target_dependencies(
  waypoint_navigator
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  mission_control_interfaces
)

# Frozen pre-shunt controller imported from the 71ddfde workspace snapshot.
# It is a separate executable so A/B simulation exercises the original code
# without compile-time flags or shared controller branches.
add_executable(waypoint_navigator_legacy
  legacy/src/waypoint_navigator.cpp
  legacy/src/node_init.cpp
  legacy/src/route.cpp
  legacy/src/runtime.cpp
  legacy/src/obstacle_recovery.cpp
  legacy/src/control.cpp
  legacy/src/maneuvers.cpp
  legacy/src/path_following.cpp
  legacy/src/progress.cpp
  legacy/src/navigation.cpp
  legacy/src/output.cpp)
target_include_directories(waypoint_navigator_legacy PRIVATE legacy/include)
ament_target_dependencies(
  waypoint_navigator_legacy
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  mission_control_interfaces
)

install(TARGETS corner_shunt_core
  EXPORT corner_shunt_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS waypoint_navigator waypoint_navigator_legacy
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/route_editor.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch routes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY include/ DESTINATION include)

install(FILES README.md NAVIGATION_CODE_GUIDE.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(_warn_deprecated_was_defined FALSE)
  if(DEFINED CMAKE_WARN_DEPRECATED)
    set(_warn_deprecated_was_defined TRUE)
    set(_warn_deprecated "${CMAKE_WARN_DEPRECATED}")
  endif()
  # Humble's vendored gtest still declares CMake 3.8. Suppress only that
  # external deprecation while the first test target initializes gtest.
  set(CMAKE_WARN_DEPRECATED OFF CACHE INTERNAL
    "Suppress external gtest_vendor deprecation during initialization" FORCE)
  ament_add_gtest(fusion_health_policy_test test/fusion_health_policy_test.cpp)
  if(_warn_deprecated_was_defined)
    set(CMAKE_WARN_DEPRECATED "${_warn_deprecated}" CACHE INTERNAL
      "Restored CMake deprecation warning setting" FORCE)
  else()
    unset(CMAKE_WARN_DEPRECATED CACHE)
    unset(CMAKE_WARN_DEPRECATED)
  endif()
  target_include_directories(fusion_health_policy_test PRIVATE include)
  ament_add_gtest(actuator_health_policy_test test/actuator_health_policy_test.cpp)
  target_include_directories(actuator_health_policy_test PRIVATE include)
  ament_add_gtest(rate_limiter_test test/rate_limiter_test.cpp)
  target_include_directories(rate_limiter_test PRIVATE include)
  ament_add_gtest(path_control_test test/path_control_test.cpp)
  target_include_directories(path_control_test PRIVATE include)
  ament_add_gtest(corner_shunt_test test/corner_shunt_test.cpp)
  target_include_directories(corner_shunt_test PRIVATE include)
  target_link_libraries(corner_shunt_test corner_shunt_core)
  ament_add_gtest(path_progress_supervisor_test test/path_progress_supervisor_test.cpp)
  target_include_directories(path_progress_supervisor_test PRIVATE include)
  ament_add_gtest(turn_progress_supervisor_test test/turn_progress_supervisor_test.cpp)
  target_include_directories(turn_progress_supervisor_test PRIVATE include)
  ament_add_gtest(navigation_supervisor_test test/navigation_supervisor_test.cpp)
  target_include_directories(navigation_supervisor_test PRIVATE include)
  ament_add_gtest(route_manager_test test/route_manager_test.cpp)
  target_include_directories(route_manager_test PRIVATE include)
  ament_target_dependencies(route_manager_test nav_msgs geometry_msgs)
  ament_add_gtest(navigation_input_cache_test test/navigation_input_cache_test.cpp)
  target_include_directories(navigation_input_cache_test PRIVATE include)
  ament_target_dependencies(navigation_input_cache_test rclcpp)
  ament_add_gtest(odometry_input_validation_test test/odometry_input_validation_test.cpp)
  target_include_directories(odometry_input_validation_test PRIVATE include)
  ament_target_dependencies(odometry_input_validation_test nav_msgs)
  ament_add_gtest(path_tracking_controller_test test/path_tracking_controller_test.cpp)
  target_include_directories(path_tracking_controller_test PRIVATE include)
  ament_add_gtest(waypoint_brake_controller_test test/waypoint_brake_controller_test.cpp)
  target_include_directories(waypoint_brake_controller_test PRIVATE include)
  ament_add_gtest(turn_settle_controller_test test/turn_settle_controller_test.cpp)
  target_include_directories(turn_settle_controller_test PRIVATE include)
  ament_add_gtest(control_state_test test/control_state_test.cpp)
  target_include_directories(control_state_test PRIVATE include)
  ament_add_gtest(obstacle_recovery_test test/obstacle_recovery_test.cpp)
  target_include_directories(obstacle_recovery_test PRIVATE include)
  add_launch_test(test/waypoint_navigator_launch_test.py TIMEOUT 30)
endif()

ament_export_include_directories(include)
ament_export_targets(corner_shunt_coreTargets HAS_LIBRARY_TARGET)

ament_package()
