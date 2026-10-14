cmake_minimum_required(VERSION 3.10)
project(contrast_controller)

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

add_executable(contrast_controller
  src/main.cpp
  src/turn_maneuver.cpp)
target_link_libraries(contrast_controller
  contrast_navigation_core::contrast_waypoint_navigator_core)
ament_target_dependencies(
  contrast_controller
  builtin_interfaces
  rclcpp
  nav_msgs
  navigation_safety
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  mission_control_interfaces)

install(TARGETS contrast_controller
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md CHANGELOG.md AB_REPORT.md OBSTACLE_VALIDATION_REPORT.md
  POSE_RECOVERY_VALIDATION_REPORT.md
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/comparison_mission.py scripts/run_pose_recovery_scenarios.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(vehicle_loopback_sim REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)
  find_package(yaml-cpp REQUIRED)
  ament_add_gtest(turn_maneuver_test
    test/turn_maneuver_test.cpp
    src/turn_maneuver.cpp)
  target_link_libraries(turn_maneuver_test
    contrast_navigation_core::contrast_waypoint_navigator_core
    vehicle_loopback_sim::loopback_sim_core
    yaml-cpp)
  ament_target_dependencies(turn_maneuver_test
    ament_index_cpp navigation_safety)
  ament_add_pytest_test(contrast_launch_contract_test
    test/test_launch_contract.py)
  ament_add_pytest_test(contrast_checkpoint_test
    test/test_checkpoint.py)
endif()

ament_package()
