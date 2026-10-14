cmake_minimum_required(VERSION 3.10)
project(navigation_mission_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mission_control_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(new_map_planner REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(mission_manager_core
  src/events.cpp
  src/mission.cpp
  src/obstacles.cpp)
target_include_directories(mission_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mission_manager_core yaml-cpp)

add_executable(mission_manager_node
  src/environment.cpp
  src/main.cpp
  src/node.cpp
  src/planning.cpp
  src/pose_recovery.cpp
  src/recovery.cpp
  src/rfid.cpp
  src/services.cpp)
target_include_directories(mission_manager_node PRIVATE include)
target_link_libraries(mission_manager_node mission_manager_core yaml-cpp)
ament_target_dependencies(mission_manager_node
  ament_index_cpp
  geometry_msgs
  mission_control_interfaces
  nav_msgs
  new_map_planner
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs)

add_executable(reporter_stub_node src/reporter_stub.cpp)
ament_target_dependencies(reporter_stub_node
  mission_control_interfaces
  rclcpp
  rclcpp_action)

install(TARGETS mission_manager_core mission_manager_node reporter_stub_node
  EXPORT mission_manager_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md CHANGELOG.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_targets(mission_manager_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(yaml_cpp_vendor)
ament_package()
