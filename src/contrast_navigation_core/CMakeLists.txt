cmake_minimum_required(VERSION 3.10)
project(contrast_navigation_core)

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
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(navigation_safety REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mission_control_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(contrast_waypoint_navigator_core STATIC
  src/node_init.cpp
  src/route.cpp
  src/launch_departure.cpp
  src/runtime.cpp
  src/obstacle_recovery.cpp
  src/pose_recovery.cpp
  src/pose_recovery_control.cpp
  src/corner_shunt.cpp
  src/control.cpp
  src/maneuvers.cpp
  src/path_following.cpp
  src/progress.cpp
  src/navigation.cpp
  src/output.cpp)

set_target_properties(contrast_waypoint_navigator_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

target_include_directories(contrast_waypoint_navigator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  contrast_waypoint_navigator_core
  builtin_interfaces
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  mission_control_interfaces
  navigation_safety)

target_link_libraries(contrast_waypoint_navigator_core yaml-cpp)

install(TARGETS contrast_waypoint_navigator_core
  EXPORT contrast_waypoint_navigator_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/
  DESTINATION include)

install(FILES README.md CHANGELOG.md DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(contrast_waypoint_navigator_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  yaml_cpp_vendor
  builtin_interfaces
  geometry_msgs
  mission_control_interfaces
  navigation_safety
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs)

ament_package()
