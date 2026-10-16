cmake_minimum_required(VERSION 3.10)
project(vehicle_loopback_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mission_control_interfaces REQUIRED)
find_package(navigation_safety REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(loopback_sim_core
  src/motion.cpp
  src/obstacle.cpp)
target_include_directories(loopback_sim_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(loopback_sim_core geometry_msgs nav_msgs)

add_executable(vehicle_loopback_sim_node
  src/main.cpp
  src/runtime.cpp
  src/sim_obstacle.cpp
  src/simulator.cpp)
target_include_directories(vehicle_loopback_sim_node PRIVATE include)
target_link_libraries(vehicle_loopback_sim_node loopback_sim_core yaml-cpp)
ament_target_dependencies(vehicle_loopback_sim_node
  ament_index_cpp
  geometry_msgs
  mission_control_interfaces
  navigation_safety
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros)

add_executable(simulation_clock_node src/simulation_clock.cpp)
ament_target_dependencies(simulation_clock_node rclcpp rosgraph_msgs std_msgs)

ament_python_install_package(vehicle_loopback_sim)

install(TARGETS loopback_sim_core vehicle_loopback_sim_node simulation_clock_node
  EXPORT loopback_sim_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY vehicle_loopback_sim/web/ DESTINATION share/${PROJECT_NAME}/web)
install(FILES README.md CHANGELOG.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/loopback_mission_node
  scripts/final_navigation_mission_node
  scripts/handoff_loopback.py
  scripts/recovery_evidence.py
  scripts/road_blockage_check.py
  scripts/run_pose_recovery_scenarios.py
  scripts/vehicle_loopback_gui_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(loopback_sim_core_test test/core_test.cpp)
  target_link_libraries(loopback_sim_core_test loopback_sim_core)
  ament_add_pytest_test(loopback_sim_python_test test
    ENV
      "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}"
      "PYTHONDONTWRITEBYTECODE=1")
endif()

ament_export_include_directories(include)
ament_export_targets(loopback_sim_coreTargets HAS_LIBRARY_TARGET)
ament_package()
