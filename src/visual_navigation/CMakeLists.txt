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

install(PROGRAMS ${CMAKE_CURRENT_SOURCE_DIR}/../../route_editor.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch routes
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

install(FILES README.md NAVIGATION_CODE_GUIDE.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(fusion_health_policy_test test/fusion_health_policy_test.cpp)
  target_include_directories(fusion_health_policy_test PRIVATE include)
  ament_add_gtest(actuator_health_policy_test test/actuator_health_policy_test.cpp)
  target_include_directories(actuator_health_policy_test PRIVATE include)
  ament_add_gtest(rate_limiter_test test/rate_limiter_test.cpp)
  target_include_directories(rate_limiter_test PRIVATE include)
  ament_add_gtest(path_control_test test/path_control_test.cpp)
  target_include_directories(path_control_test PRIVATE include)
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
  ament_add_gtest(path_tracking_controller_test test/path_tracking_controller_test.cpp)
  target_include_directories(path_tracking_controller_test PRIVATE include)
  ament_add_gtest(waypoint_brake_controller_test test/waypoint_brake_controller_test.cpp)
  target_include_directories(waypoint_brake_controller_test PRIVATE include)
  ament_add_gtest(turn_settle_controller_test test/turn_settle_controller_test.cpp)
  target_include_directories(turn_settle_controller_test PRIVATE include)
  add_launch_test(test/waypoint_navigator_launch_test.py TIMEOUT 30)
endif()

ament_package()
