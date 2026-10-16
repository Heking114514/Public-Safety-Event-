cmake_minimum_required(VERSION 3.10)
project(final_navigation)

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
find_package(contrast_navigation_core REQUIRED)

add_executable(final_navigation
  src/main.cpp
  src/turn_maneuver.cpp)
target_link_libraries(final_navigation
  contrast_navigation_core::contrast_waypoint_navigator_core)
ament_target_dependencies(
  final_navigation
  builtin_interfaces
  rclcpp
  nav_msgs
  navigation_safety
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  mission_control_interfaces)

install(TARGETS final_navigation
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md CHANGELOG.md AB_REPORT.md FULL_MISSION_REPORT.md
  OBSTACLE_VALIDATION_REPORT.md POSE_RECOVERY_VALIDATION_REPORT.md
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/route_editor.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)
  ament_add_gtest(turn_maneuver_test
    test/turn_maneuver_test.cpp
    src/turn_maneuver.cpp)
  target_link_libraries(turn_maneuver_test
    contrast_navigation_core::contrast_waypoint_navigator_core
    yaml-cpp)
  ament_target_dependencies(turn_maneuver_test
    ament_index_cpp navigation_safety)
endif()

ament_package()
