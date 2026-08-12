cmake_minimum_required(VERSION 3.8)
project(visual_navigation)

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

add_executable(waypoint_navigator src/waypoint_navigator.cpp)
target_include_directories(waypoint_navigator PRIVATE include)
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

install(TARGETS waypoint_navigator
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../route_editor.py
  scripts/arena_route_planner.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch routes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(FILES README.md NAVIGATION_CODE_GUIDE.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(fusion_health_policy_test test/fusion_health_policy_test.cpp)
  target_include_directories(fusion_health_policy_test PRIVATE include)
  ament_add_gtest(actuator_health_policy_test test/actuator_health_policy_test.cpp)
  target_include_directories(actuator_health_policy_test PRIVATE include)
  ament_add_gtest(rate_limiter_test test/rate_limiter_test.cpp)
  target_include_directories(rate_limiter_test PRIVATE include)
  ament_add_gtest(path_control_test test/path_control_test.cpp)
  target_include_directories(path_control_test PRIVATE include)
  ament_add_pytest_test(arena_route_planner_test test/test_arena_route_planner.py)
endif()

ament_package()
